// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"

#include "Core/RopeWrapTarget.h"
#include "Components/PrimitiveComponent.h"
#include "DynamicRopeLog.h"
#include "Engine/World.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RopeComponentInternal.h"
#include "RopeMathHelpers.h"
#include "Subsystem/RopeSimSubsystem.h"

using RopeComponentPrivate::LogWrappingFailureState;
using RopeComponentPrivate::ReleaseCooldownSeconds;

#pragma region File_Local_Helpers

namespace
{
	constexpr float KinematicBridgeStretchWarningRatio = 1.25f;

	const FRopeSurfaceAnchor* FindSurfaceAnchorByNode(
		const TArray<FRopeSurfaceAnchor>& Anchors, int32 NodeIndex)
	{
		return Anchors.FindByPredicate(
			[NodeIndex](const FRopeSurfaceAnchor& Anchor)
			{
				return Anchor.NodeIndex == NodeIndex;
			});
	}

	bool ResolveAnchorCenterlineWorld(const FRopeSurfaceAnchor& Anchor, FVector& OutWorld)
	{
		const USceneComponent* Mesh = Anchor.Mesh.Get();
		if (!Mesh)
		{
			return false;
		}

		const FTransform BindingWorld = ResolveBindingWorld(Mesh, Anchor.Bone);
		const FVector SurfaceWorld =
			BindingWorld.TransformPosition(Anchor.LocalSurfacePosition);
		const FVector NormalWorld =
			BindingWorld.TransformVectorNoScale(Anchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		OutWorld = SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;
		return true;
	}

	bool ResolveAnchorCenterlineWorld(
		const FRopeSurfaceAnchor& Anchor,
		const USceneComponent* FallbackMesh,
		FName FallbackBone,
		const USceneComponent*& OutMesh,
		FName& OutBone,
		FVector& OutWorld)
	{
		OutMesh = Anchor.Mesh.IsValid() ? Anchor.Mesh.Get() : FallbackMesh;
		if (!OutMesh)
		{
			return false;
		}

		OutBone = Anchor.Bone.IsNone() ? FallbackBone : Anchor.Bone;
		const FTransform BindingWorld = ResolveBindingWorld(OutMesh, OutBone);
		const FVector SurfaceWorld =
			BindingWorld.TransformPosition(Anchor.LocalSurfacePosition);
		const FVector NormalWorld =
			BindingWorld.TransformVectorNoScale(Anchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		OutWorld = SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;
		return true;
	}

}

#pragma endregion File_Local_Helpers_And_Debug

#pragma region Wrapping_Public_API

bool URopeComponent::BuildWielderMovementConstraint(
	FRopeWielderMovementConstraint& OutConstraint,
	float PendingReelDeltaTime) const
{
	OutConstraint = FRopeWielderMovementConstraint();
	if (!HoldConfig.bEnforceWielderLengthConstraint ||
		(Phase != ERopePhase::Wrapping && Phase != ERopePhase::Wrapped) ||
		Sim.SegmentLength <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const FRopeSurfaceAnchor* FirstAnchor = nullptr;
	const USceneComponent* StateMesh = nullptr;
	FName StateBone = NAME_None;

	auto ConsiderAnchor = [this, &FirstAnchor](const FRopeSurfaceAnchor& Anchor)
	{
		if (Anchor.NodeIndex < 0 || !Sim.Positions.IsValidIndex(Anchor.NodeIndex))
		{
			return;
		}
		if (!FirstAnchor || Anchor.NodeIndex < FirstAnchor->NodeIndex)
		{
			FirstAnchor = &Anchor;
		}
	};

	if (Phase == ERopePhase::Wrapping)
	{
		StateMesh = WrappingPhase.State.Mesh.Get();
		StateBone = WrappingPhase.State.BoneName;
		for (const FRopeSurfaceAnchor& Anchor : WrappingPhase.State.Anchors)
		{
			ConsiderAnchor(Anchor);
		}
		for (const FRopeSurfaceAnchor& Anchor : WrappingPhase.State.SecondarySeedAnchors)
		{
			ConsiderAnchor(Anchor);
		}
		ConsiderAnchor(WrappingPhase.State.LatchAnchor);
	}
	else
	{
		StateMesh = WrapController.State.Mesh.Get();
		StateBone = WrapController.State.BoneName;
		for (const FRopeSurfaceAnchor& Anchor : WrapController.State.Anchors)
		{
			ConsiderAnchor(Anchor);
		}
	}

	const USceneComponent* TargetComponent = nullptr;
	FName TargetBone = NAME_None;
	FVector AnchorWorld = FVector::ZeroVector;
	int32 AnchorNode = INDEX_NONE;

	if (FirstAnchor)
	{
		if (!ResolveAnchorCenterlineWorld(
			*FirstAnchor, StateMesh, StateBone, TargetComponent, TargetBone, AnchorWorld))
		{
			return false;
		}
		AnchorNode = FirstAnchor->NodeIndex;
	}
	else if (Phase == ERopePhase::Wrapped)
	{
		// Legacy seeds can contain only Latched. Keep their binding semantics until the legacy
		// path is removed, but still choose the minimum hand-side node deterministically.
		const FRopeLatchNode* FirstLatch = nullptr;
		for (const FRopeLatchNode& Latch : WrapController.State.Latched)
		{
			if (Latch.NodeIndex >= 0 && Sim.Positions.IsValidIndex(Latch.NodeIndex) &&
				(!FirstLatch || Latch.NodeIndex < FirstLatch->NodeIndex))
			{
				FirstLatch = &Latch;
			}
		}
		if (!FirstLatch || !StateMesh)
		{
			return false;
		}

		TargetComponent = StateMesh;
		TargetBone = FirstLatch->Bone.IsNone() ? StateBone : FirstLatch->Bone;
		AnchorWorld =
			ResolveBindingWorld(TargetComponent, TargetBone).TransformPosition(FirstLatch->BoneLocalPos);
		AnchorNode = FirstLatch->NodeIndex;
	}
	else
	{
		return false;
	}

	// A rope wrapped onto its own owner has no external relative endpoint. Moving the owner moves
	// both points together, so a world-space leash would falsely lock the actor in place.
	if (!TargetComponent || (GetOwner() && TargetComponent->GetOwner() == GetOwner()))
	{
		return false;
	}

	OutConstraint.PivotWorld = AnchorWorld;
	// Velocity must belong to the actual anchor point, not just the component origin. Physics
	// bodies provide the exact linear + angular point velocity. For a kinematic/animated binding,
	// target tick prerequisites make AnchorWorld current; finite-difference it against the prior
	// PostPhysics sample instead of reusing the one-frame-old smoothed velocity.
	OutConstraint.PivotVelocity = TargetComponent->GetComponentVelocity();
	if (UPrimitiveComponent* Primitive =
		const_cast<UPrimitiveComponent*>(Cast<UPrimitiveComponent>(TargetComponent));
		Primitive && Primitive->IsSimulatingPhysics(TargetBone))
	{
		OutConstraint.PivotVelocity =
			Primitive->GetPhysicsLinearVelocityAtPoint(AnchorWorld, TargetBone);
	}
	else if (Phase == ERopePhase::Wrapped &&
		LengthConstraintState.bPrevGeometryValid &&
		LengthConstraintState.PrevAnchorNode == AnchorNode)
	{
		const float VelocityDt =
			PendingReelDeltaTime > KINDA_SMALL_NUMBER
				? PendingReelDeltaTime
				: (GetWorld()
					? GetWorld()->GetDeltaSeconds()
					: 0.0f);
		if (VelocityDt > KINDA_SMALL_NUMBER)
		{
			OutConstraint.PivotVelocity =
				(AnchorWorld -
					LengthConstraintState.PrevAnchorWorldPoint) /
				VelocityDt;
		}
	}
	OutConstraint.AnchorWorld = AnchorWorld;
	float ConstraintSegmentLength = Sim.SegmentLength;
	// RopeSimSubsystem applies reel in PostPhysics, while the Wielder's hard boundary runs
	// in PrePhysics. Predict exactly that pending material-length change here so a continuous
	// reel-in cannot leave one frame of impossible fixed-end stretch behind.
	if (Phase == ERopePhase::Wrapped &&
		PendingReelDeltaTime > KINDA_SMALL_NUMBER &&
		!FMath::IsNearlyZero(ReelRate) &&
		Sim.Num() >= 2)
	{
		const float MaxLen = FMath::Max(RopeLength, MinRopeLength);
		const float PredictedLength = FMath::Clamp(
			Sim.RopeLength - ReelRate * PendingReelDeltaTime,
			FMath::Min(MinRopeLength, MaxLen),
			MaxLen);
		ConstraintSegmentLength =
			PredictedLength / static_cast<float>(Sim.Num() - 1);
	}
	OutConstraint.MaxDistance =
		static_cast<float>(AnchorNode) * ConstraintSegmentLength;
	OutConstraint.AnchorNode = AnchorNode;
	OutConstraint.TargetComponent = TargetComponent;
	OutConstraint.TargetBone = TargetBone;
	return OutConstraint.IsValid();
}

bool URopeComponent::ConstrainWielderLocation(
	FVector DesiredPinWorld,
	FVector& OutConstrainedPinWorld,
	FVector& OutBoundaryNormal,
	bool& bOutWasConstrained) const
{
	OutConstrainedPinWorld = DesiredPinWorld;
	OutBoundaryNormal = FVector::ZeroVector;
	bOutWasConstrained = false;

	FRopeWielderMovementConstraint Constraint;
	if (!BuildWielderMovementConstraint(Constraint))
	{
		return false;
	}
	if (HoldConfig.TetherCompliance > KINDA_SMALL_NUMBER)
	{
		// An elastic cable intentionally permits boundary extension. The same compliance
		// drives the analytic/Chaos backend, so this convenience API must not reintroduce
		// a separate unconditional hard projection.
		return true;
	}

	const RopeMovementConstraint::FProjectionResult Projection =
		RopeMovementConstraint::ProjectPoint(
			DesiredPinWorld, Constraint.PivotWorld, Constraint.MaxDistance);
	OutConstrainedPinWorld = Projection.Position;
	OutBoundaryNormal = Projection.OutwardNormal;
	bOutWasConstrained = Projection.bConstrained;
	return true;
}

void URopeComponent::FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog)
{
	// The shared ending for every release trigger, whether manual, a cut, tension, distance or a lost
	// target: change phase, return the nodes, discard the transient state, start the cooldown and fire
	// the events. Anything that differs by reason has already been settled by the caller.
	// The wrapped mesh is captured before Release and Reset clear the wrap state, because the central
	// release signal carries it so target reaction components can decide whether it was their mesh that
	// let go; the Blueprint release delegate does not include a mesh.
	const USceneComponent* WrappedMesh = WrapController.State.Mesh.Get();
	// ReleaseWrapAs is entered not only from Wrapped but also from Contacting, Wrapping and GuidedThrow,
	// all of which are before the commit, and before the commit the central signal must not fire; see the
	// DispatchReleased comment, where this rope's abort would wrongly recover a target another rope had
	// wrapped and knocked down. It is captured before WrapController.Release clears the state.
	const bool bWasWrapped = WrapController.IsActive();
	SetPhase(ERopePhase::Releasing, *ReasonLog);
	WrapController.Release(Reason);
	ReleaseKinematicVirtualBridgesToSolver();
	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;
	DispatchReleased(WrappedMesh, Bone, Reason, bWasWrapped);
	// The tip attachment is not destroyed here: its lifetime spans BeginPlay to EndPlay in every mode.
	// Destroying it on release would leave no tip in Free, which makes bSyncTipMeshOnFree meaningless.
}

void URopeComponent::FinishPreCommitReleaseToFlight(FName Bone, const TCHAR* PhaseLog)
{
	// The shared ending for leaving before a wrap is established, that is between capture and wrapping.
	// It reports after cleaning up, meaning after the transition to Flight and discarding the transient
	// state. Reporting before the cleanup would let a handler's call to ReleaseWrap() pass a gate that
	// still reads as Contacting, producing a double release, and the Releasing phase the handler settled
	// would immediately be overwritten by this SetPhase(Flight). AbortGuidedThrow follows the same
	// contract.
	// The bone is captured by value as an argument and stays valid after ResetTransientPhaseState clears
	// the tracker. This is before the commit, so only the per-instance delegate fires and the central
	// OnAnyRopeReleased does not, which prevents wrongly recovering a target another rope has wrapped.
	SetPhase(ERopePhase::Flight, PhaseLog);
	ReleaseKinematicVirtualBridgesToSolver();
	ResetTransientPhaseState();
	DispatchReleased(nullptr, Bone, ERopeReleaseReason::Broken, /*bWasWrapped*/ false);
}

void URopeComponent::DispatchReleased(const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason, bool bWasWrapped)
{
	// If a wrapped notification is still in progress, meaning a handler called ReleaseWrap from inside
	// it, the notification is deferred: firing now would let subscribers receive the release before the
	// wrap; see the comment on FDeferredReleaseNotice.
	// The state is already cleaned up, so only the notification waits, and the queue is drained the
	// moment the wrapped notification finishes.
	if (WrappedDispatchDepth > 0)
	{
		FDeferredReleaseNotice& Notice = DeferredReleaseNotices.AddDefaulted_GetRef();
		Notice.WrappedMesh = const_cast<USceneComponent*>(WrappedMesh);
		Notice.Bone = Bone;
		Notice.Reason = Reason;
		Notice.bWasWrapped = bWasWrapped;
		return;
	}

	// Per-instance: the end of an engagement that began with a capture or a wrap is always reported, so
	// the events stay paired.
	NotifyReleased(Bone, Reason);
	OnRopeReleased.Broadcast(Bone, Reason);
	// The central signal is the counterpart of OnAnyRopeWrapped and only fires when a wrap was actually
	// established. Firing it on an abort before the commit would let this rope's abort wrongly recover
	// the same target another rope had wrapped and knocked down.
	if (bWasWrapped)
	{
		if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
		{
			// The rope itself is carried too: when several ropes have wrapped one target, a subscriber
			// cannot tell from the mesh alone which of its engagements ended. It pairs with
			// FRopeWrappedEventInfo::Rope.
			SimSubsystem->OnAnyRopeReleased.Broadcast(this, WrappedMesh, Bone, Reason);
		}
	}
}

void URopeComponent::ReleaseWrap()
{
	ReleaseWrapAs(ERopeReleaseReason::Manual);
}

void URopeComponent::CutRope()
{
	ReleaseWrapAs(ERopeReleaseReason::Cut);
}

void URopeComponent::ReleaseWrapAs(ERopeReleaseReason Reason)
{
	if (Phase != ERopePhase::Wrapped && Phase != ERopePhase::Contacting && Phase != ERopePhase::Wrapping &&
		Phase != ERopePhase::GuidedThrow)
		return;

	FName Bone = NAME_None;

	if (Phase == ERopePhase::Wrapped)
	{
		Bone = WrapController.State.BoneName;
	}
	else if (Phase == ERopePhase::Wrapping)
	{
		Bone = WrappingPhase.State.BoneName;
	}
	else if (Phase == ERopePhase::GuidedThrow)
	{
		Bone = GuidedThrowState.Prepared.Bone;
	}
	else
	{
		Bone = ContactTracker.CandidateBone;
	}

	FinishWrapRelease(Bone, Reason, FString::Printf(TEXT("%s, bone=%s"),
		Reason == ERopeReleaseReason::Cut ? TEXT("cut") : TEXT("manual"), *Bone.ToString()));
}

#pragma endregion Wrapping_Public_API

#pragma region Contacting

// ===== Contacting ===========================================================

void URopeComponent::UpdateContacting(float DeltaTime)
{
	// Put the GPU Flight capture's pending step and the CPU simulation state onto one timeline first. On
	// a frame where that fails, neither the dwell, the dismissal nor the seed is updated from a stale
	// centreline. In particular TryBuildResidentStep clears bGpuSteppedThisFrame during Contacting, so the
	// persistent flag raised at the moment of capture is what has to be used.
	bool bSyncedGpuHandoff = false;
	if (bPendingGpuCaptureHandoff)
	{
		URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld());
		if (!SimSubsystem || !SimSubsystem->SyncGpuPositionsForHandoff(*this))
		{
			return;
		}
		bPendingGpuCaptureHandoff = false;
		bSyncedGpuHandoff = true;
	}
	// If the swept actual contact on the capture frame alone already satisfied the decision dwell, that
	// crossing is a valid contact. Forcing a current-only re-detection after synchronizing would lose it
	// on the GPU alone whenever a collision-free guided step passed completely through a thin limb, so
	// the stored surface seed is used to reach exactly the same decision once, matching the CPU's
	// immediate wrap.
	if (bSyncedGpuHandoff && ShouldStartWrapping())
	{
		// On the deferred scene distance field path the CPU mirror may have been stale when
		// BuildContactingState ran. The candidate is restored from the surface seed's node and point, its
		// travel speed and plane are refreshed against the post-step simulation state, and then committed.
		TArray<FRopeContactCandidate> SyncedCaptureCandidates;
		SyncedCaptureCandidates.Reserve(PendingWrapSeed.Anchors.Num());
		for (const FRopeSurfaceAnchor& Anchor : PendingWrapSeed.Anchors)
		{
			const USceneComponent* Mesh = Anchor.Mesh.Get();
			if (!Mesh || !Sim.Positions.IsValidIndex(Anchor.NodeIndex))
			{
				continue;
			}
			const FTransform BindingWorld = ResolveBindingWorld(Mesh, Anchor.Bone);
			FRopeContactCandidate& Candidate = SyncedCaptureCandidates.AddDefaulted_GetRef();
			Candidate.bValid = true;
			Candidate.NodeIndex = Anchor.NodeIndex;
			Candidate.Mesh = Mesh;
			Candidate.Bone = Anchor.Bone;
			Candidate.Source = ERopeContactCandidateSource::Actual;
			Candidate.SourceMask = static_cast<uint8>(ERopeContactCandidateSource::Actual);
			Candidate.WorldPoint = BindingWorld.TransformPosition(Anchor.LocalSurfacePosition);
			Candidate.Normal = BindingWorld.TransformVectorNoScale(Anchor.LocalNormal)
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		}
		if (SyncedCaptureCandidates.Num() > 0)
		{
			CaptureTravelFrame = FRopeCaptureTravelFrame::Compute(
				Sim, SyncedCaptureCandidates, DeltaTime);
		}
		StartWrappingFromContacting();
		return;
	}

	// Total time spent here, used by the stall safety net below; the wrap decision itself uses the
	// tracker's dwell.
	ContactingElapsed += DeltaTime;

	// Real contacts are re-collected every frame. Running a timer from a single snapshot taken at the
	// moment of capture meant, first, that dismissal effectively never fired because the tracker was
	// never updated, and second, that on a moving target such as a ragdoll or a dragon the divergence
	// between the seed and the real geometry accumulated for the whole decision time. Contacting has no
	// solve, so the nodes are stationary, and replaying the old previous-to-current path would revive a
	// contact the rope had already left on every frame.
	// The general actual re-collection is combined with the assisted exact-primary current-centreline
	// supplement. The latter does not replay the temporal or predictive path of the last Flight frame, so
	// dwell does not accumulate for something that has not touched yet.
	const FRopeFlightContactDetector::FParams DetectParams = MakeFlightDetectParams(DeltaTime);
	TArray<FRopeContactCandidate>& Candidates = ContactCandidateScratch;
	Candidates.Reset();
	FRopeFlightContactDetector::AddCurrentCenterlineContactCandidates(
		Sim, SimFrame.FrameColliders, DetectParams, Candidates);
	AddSynchronousAssistedAimContactCandidates(DeltaTime, DetectParams, Candidates);
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, Candidates);
	// The CanWrapTarget gate, through the helper shared with Flight candidate production.
	RemoveNonWrappableCandidates(Candidates);

	// Update the tracker: the dwell accumulates while the bone is the same, resets when the dominant bone
	// changes, which guards against false positives on a transition frame by genuinely applying the
	// dwell-restart contract after capture as well, and drains when contact breaks until the tracker
	// empties and falls through to the dismissal below. Brief flicker is tolerated for as long as the
	// dwell accumulated so far.
	const bool bRequireAimPrimary = ResolveMode == ERopeWrapResolveMode::AssistedJudged
		&& AimTargeting.IsLockActive(Phase);
	ContactTracker.Update(Candidates, DeltaTime,
		bRequireAimPrimary ? AimTargeting.GetLockedTargetMesh() : nullptr,
		bRequireAimPrimary ? AimTargeting.GetLockedTargetBone() : NAME_None,
		bRequireAimPrimary);

	if (ShouldDismissContacting())
	{
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Contacting dismissed before wrap: reason=ContactTrackerEmpty candidates=%d trackerBone=%s trackerNodes=%d dwell=%.3fs elapsed=%.3fs colliders=%d"),
			*GetName(), Candidates.Num(), *ContactTracker.CandidateBone.ToString(),
			ContactTracker.CandidateNodes.Num(), ContactTracker.DwellTime,
			ContactingElapsed, SimFrame.FrameColliders.Num());
		// Left before the wrap was established: report per-instance after cleaning up, through
		// FinishPreCommitReleaseToFlight, which owns the re-entrancy contract.
		FinishPreCommitReleaseToFlight(ContactTracker.CandidateBone, TEXT("contact lost before wrapping"));
		return;
	}

#if !UE_BUILD_SHIPPING
	extern TAutoConsoleVariable<int32> CVarRopeFlightDebug;
	if (CVarRopeFlightDebug.GetValueOnGameThread() > 0)
	{
		UE_LOG(LogDynamicRope, Log,
			TEXT("[%s] CONTACTDBG dt=%.1fms cand=%d tracker=%s nodes=%d dwell=%.3f elapsed=%.3f colliders=%d"),
			*GetName(), DeltaTime * 1000.0f, Candidates.Num(),
			*ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num(),
			ContactTracker.DwellTime, ContactingElapsed, SimFrame.FrameColliders.Num());
	}
#endif

	// Refresh the seed so path generation starts from the newest contact geometry on the frame Wrapping
	// begins.
	if (Candidates.Num() > 0 && !ContactTracker.CandidateBone.IsNone())
	{
		PendingWrapSeed = BuildWrapSeedFromContactingState(Candidates);
		if (bSyncedGpuHandoff)
		{
			// The snapshot built during Flight's Finalize may be based on the asynchronous GPU mirror. The
			// travel frame is rebuilt from the same post-step pose and candidates only on the first
			// Contacting frame where synchronization succeeded.
			CaptureTravelFrame = FRopeCaptureTravelFrame::Compute(Sim, Candidates, DeltaTime);
		}
	}

	if (ShouldStartWrapping())
	{
		StartWrappingFromContacting();
		return;
	}

	// The stall safety net now covers only malformed/incomplete seeds: real contact commits immediately,
	// so no user-configured dwell time participates in this timeout.
	constexpr float StallTimeout = 1.0f;
	if (ContactingElapsed >= StallTimeout)
	{
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Contacting stalled before wrap: candidates=%d trackerBone=%s trackerNodes=%d targets=%d dwell=%.3fs elapsed=%.3fs timeout=%.3fs"),
			*GetName(), Candidates.Num(), *ContactTracker.CandidateBone.ToString(),
			ContactTracker.CandidateNodes.Num(), ContactTracker.Targets.Num(),
			ContactTracker.DwellTime, ContactingElapsed, StallTimeout);
		// Left before the wrap was established: report per-instance after cleaning up, through
		// FinishPreCommitReleaseToFlight.
		FinishPreCommitReleaseToFlight(ContactTracker.CandidateBone,
			*FString::Printf(TEXT("contacting stalled %.2fs with an incomplete seed"), ContactingElapsed));
	}
}

bool URopeComponent::ShouldDismissContacting() const
{
	return ContactTracker.CandidateBone.IsNone() || ContactTracker.CandidateNodes.Num() == 0;
}

bool URopeComponent::ShouldStartWrapping() const
{
	// URopeComponent::EvaluateFlightCapture already requires a real swept/current contact. Once that
	// produces a valid seed there is no additional time gate: delaying in a solver-free Contacting phase
	// can only lose the surface on the following frame.
	return PendingWrapSeed.Latched.Num() > 0
		&& !PendingWrapSeed.BoneName.IsNone();
}

FRopeWrapState URopeComponent::BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const
{
	FRopeWrapState Seed;
	Seed.BoneName = ContactTracker.CandidateBone;
	Seed.Mesh = ContactTracker.CandidateMesh;
	const int32 NodeIndex = RopeMath::HeadValidNodeIndex(ContactTracker.CandidateNodes, Sim.Positions);
	if (NodeIndex == INDEX_NONE)
	{
		return Seed;
	}

	// The dominant seed occupies slot 0 of the seed's latches and anchors; StartWrappingFromContacting
	// consumes slot 0 as the starting point of the path build by contract. If assembling the anchor
	// fails, the latch is still kept.
	{
		FRopeLatchNode Latch;
		FRopeSurfaceAnchor Anchor;
		const USceneComponent* ResolvedMesh = nullptr;
		const bool bAnchorBuilt = BuildSeedLatchForTarget(Candidates,
			ContactTracker.CandidateBone, ContactTracker.CandidateMesh, NodeIndex,
			/*RopeDistance*/ 0.0f, Latch, Anchor, ResolvedMesh);
		Seed.Latched.Add(Latch);
		if (!Seed.Mesh.IsValid() && ResolvedMesh)
		{
			Seed.Mesh = ResolvedMesh;
		}
		if (bAnchorBuilt)
		{
			Seed.Anchors.Add(Anchor);
		}
	}

	// Secondary seeds, used when MaxWrapSeeds is above 1: a target further along the tail than the
	// dominant one that has accumulated dwell on a different (mesh, bone) is adopted as an extra seed,
	// such as the opposite leg when catching both. Targets on the head side are not accepted, because
	// Wrapping's path and mask own only the stretch after the latch, so there is no way to pin a node on
	// the head side. A secondary seed is valid only once its anchor exists, since it is held against its
	// bone with no path and therefore needs a surface frame. Without a dominant anchor no secondary is
	// accepted either: Anchors[0] is contractually the dominant slot, the starting point of the path in
	// StartWrappingFromContacting, and must not be polluted by a secondary anchor.
	if (WrapConfig.MaxWrapSeeds > 1 && Seed.Anchors.Num() > 0)
	{
		// The minimum separation between nodes, in segments. It guarantees the stretch the dominant helix
		// needs and stops neighbouring nodes splitting between different seeds.
		constexpr int32 MinSeedNodeSeparation = 2;

		TArray<const FRopeTrackedContactTarget*> Sorted;
		for (const FRopeTrackedContactTarget& Target : ContactTracker.Targets)
		{
			const bool bDominant = Target.Bone == ContactTracker.CandidateBone &&
				Target.Mesh == ContactTracker.CandidateMesh;
			if (!bDominant && Target.Nodes.Num() > 0)
			{
				Sorted.Add(&Target);
			}
		}
		// Nearest the dominant, that is on the head side, first, which gives a stable adoption order
		// between frames.
		Sorted.Sort([this](const FRopeTrackedContactTarget& A, const FRopeTrackedContactTarget& B)
		{
			return RopeMath::HeadValidNodeIndex(A.Nodes, Sim.Positions)
				< RopeMath::HeadValidNodeIndex(B.Nodes, Sim.Positions);
		});

		for (const FRopeTrackedContactTarget* Target : Sorted)
		{
			if (Seed.Latched.Num() >= WrapConfig.MaxWrapSeeds)
			{
				break;
			}

			const int32 SecondaryNode = RopeMath::HeadValidNodeIndex(Target->Nodes, Sim.Positions);
			if (SecondaryNode == INDEX_NONE)
			{
				continue;
			}

			bool bTooClose = false;
			for (const FRopeLatchNode& Existing : Seed.Latched)
			{
				if (SecondaryNode < Existing.NodeIndex + MinSeedNodeSeparation)
				{
					bTooClose = true;
					break;
				}
			}
			if (bTooClose)
			{
				continue;
			}

			FRopeLatchNode Latch;
			FRopeSurfaceAnchor Anchor;
			const USceneComponent* ResolvedMesh = nullptr;
			if (BuildSeedLatchForTarget(Candidates, Target->Bone, Target->Mesh, SecondaryNode,
				static_cast<float>(SecondaryNode - NodeIndex) * Sim.SegmentLength, Latch, Anchor, ResolvedMesh))
			{
				Seed.Latched.Add(Latch);
				Seed.Anchors.Add(Anchor);
			}
		}
	}

	return Seed;
}

bool URopeComponent::BuildSeedLatchForTarget(const TArray<FRopeContactCandidate>& Candidates,
	FName Bone, const USceneComponent* TrackedMesh, int32 NodeIndex, float RopeDistance,
	FRopeLatchNode& OutLatch, FRopeSurfaceAnchor& OutAnchor, const USceneComponent*& OutMesh) const
{
	OutLatch.NodeIndex = NodeIndex;
	OutLatch.Bone = Bone;
	OutMesh = TrackedMesh;

	const FRopeContactCandidate* LatchCandidate = nullptr;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid ||
			Candidate.NodeIndex != NodeIndex ||
			Candidate.Bone != Bone)
		{
			continue;
		}

		// Guards against misattribution on a frame where two actors sharing a bone name are both touched,
		// for the same reason the tracker keys on the pair of mesh and bone. The candidate's mesh is taken
		// as-is only when the tracker has none, which preserves the previous fallback.
		if (TrackedMesh && Candidate.Mesh && Candidate.Mesh != TrackedMesh)
		{
			continue;
		}

		if (!LatchCandidate || Candidate.Penetration > LatchCandidate->Penetration)
		{
			LatchCandidate = &Candidate;
		}
	}

	if (!OutMesh && LatchCandidate)
	{
		OutMesh = LatchCandidate->Mesh;
	}

	if (!LatchCandidate || !OutMesh)
	{
		return false;
	}

	const FVector NormalWorld = LatchCandidate->Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector TangentWorld = FRopeFlightContactDetector::ExpectedWrapTangent(Sim, *LatchCandidate, GetForwardVector());
	if (Sim.Positions.IsValidIndex(NodeIndex + 1))
	{
		TangentWorld = Sim.Positions[NodeIndex + 1] - Sim.Positions[NodeIndex];
	}
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(NormalWorld));

	const FTransform BoneXform = ResolveBindingWorld(OutMesh, Bone);

	OutAnchor.NodeIndex = NodeIndex;
	OutAnchor.Bone = Bone;
	OutAnchor.Mesh = OutMesh;
	OutAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(LatchCandidate->WorldPoint);
	OutAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	OutAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	OutAnchor.StartWorldPosition = Sim.Positions[NodeIndex];
	OutAnchor.SurfaceOffset = FMath::Max(0.0f, Radius);
	OutAnchor.RopeDistance = RopeDistance;
	return true;
}

#pragma endregion Contacting

#pragma region Wrapping

// ===== Wrapping =============================================================

void URopeComponent::StartWrappingFromContacting()
{
	// PendingWrapSeed is converted into wrapping phase state rather than being handed straight to
	// BeginWrap.
	// A new attempt never inherits the incremental scan state of the previous composite virtual run.
	ResetKinematicVirtualBridges();
	WrappingPhase.State.Reset();

	// The mesh to wrap is established by the contact, propagated from FRopeContact::SourceMesh into the
	// seed. Finding it empty here means the seed is malformed: substituting the owner's mesh would attach
	// to the wrong bone in a cross-actor wrap, so it returns rather than falling back.
	const USceneComponent* Mesh = PendingWrapSeed.Mesh.Get();
	if (!Mesh || PendingWrapSeed.BoneName.IsNone() || PendingWrapSeed.Latched.Num() == 0)
	{
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] WRAP FAILURE: site=StartWrapping reason=InvalidWrappingSeed mesh=%s bone=%s latched=%d anchors=%d trackerBone=%s trackerNodes=%d dwell=%.3fs contacting=%.3fs simNodes=%d"),
			*GetName(), Mesh ? *Mesh->GetName() : TEXT("None"), *PendingWrapSeed.BoneName.ToString(),
			PendingWrapSeed.Latched.Num(), PendingWrapSeed.Anchors.Num(),
			*ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num(),
			ContactTracker.DwellTime, ContactingElapsed, Sim.Num());
		// Left before the wrap was established: report per-instance after cleaning up, through
		// FinishPreCommitReleaseToFlight.
		FinishPreCommitReleaseToFlight(ContactTracker.CandidateBone, TEXT("invalid wrapping seed"));
		return;
	}

	// Always take exactly the first latch node as the reference.
	const FRopeLatchNode& Latch = PendingWrapSeed.Latched[0];
	FRopeSurfaceAnchor LatchAnchor;

	// The normal path: BuildWrapSeedFromContactingState() has already built a surface anchor from a real
	// contact candidate, so it is used as-is.
	if (PendingWrapSeed.Anchors.Num() > 0)
	{
		LatchAnchor = PendingWrapSeed.Anchors[0];
		LatchAnchor.Mesh = Mesh;
	}
	// Emergency only: the fallback below is a stopgap anchor path used when there is no accurate SDF
	// surface anchor from a contact candidate.
	// It substitutes the current rope particle position and a placeholder normal and tangent as the
	// starting point, so the wrap quality and direction may wobble.
	// The normal path is for PendingWrapSeed.Anchors[0] to hold a real contact surface point, normal and
	// tangent.
	// Since Contacting began reassembling the seed from the newest candidates every frame, Anchors[0] is
	// always filled while any candidate exists, so this path is believed to be effectively unreachable;
	// the warning below is there to observe whether it is ever reached in practice before removing it.
	else if (Sim.Positions.IsValidIndex(Latch.NodeIndex))
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] StartWrapping fell back to the synthetic latch anchor (no contact-based anchor in seed, bone=%s) — wrap quality may wobble. Thought unreachable; report if seen."),
			*GetName(), *Latch.Bone.ToString());

		const FVector NormalWorld = FVector::UpVector;
		FVector TangentWorld = FVector::ForwardVector;

		if (Sim.Positions.IsValidIndex(Latch.NodeIndex + 1))
		{
			// The tangent uses the direction to the next rope node where possible. It captures which way
			// the rope extends towards the tail, which matters later when the composite analytic helix or
			// the sequential surface vector field decides the winding direction.
			TangentWorld = (Sim.Positions[Latch.NodeIndex + 1] - Sim.Positions[Latch.NodeIndex])
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		}

		const FTransform BoneXform = ResolveBindingWorld(Mesh, Latch.Bone);
		LatchAnchor.NodeIndex = Latch.NodeIndex;
		LatchAnchor.Bone = Latch.Bone;
		LatchAnchor.Mesh = Mesh;
		// Store the current latch node position as though it were a bone-local surface position, and use
		// an up vector as a placeholder normal rather than a real SDF normal.
		LatchAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Sim.Positions[Latch.NodeIndex]);
		LatchAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		LatchAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		LatchAnchor.StartWorldPosition = Sim.Positions[Latch.NodeIndex];
		LatchAnchor.SurfaceOffset = FMath::Max(0.0f, Radius);
		LatchAnchor.RopeDistance = 0.0f;
	}

	// Seed multiplexing: Anchors[0] is the dominant seed, which is the path's starting point, and the
	// rest are secondary seed anchors. They have to be loaded into the state before Begin so the path
	// build clamps NumTailNodes to end before the first secondary node; see the field's comment.
	// Only nodes further along the tail than the dominant latch are valid. Seed assembly guarantees that,
	// but it is filtered once more here in case the seed came from an older frame.
	for (int32 AnchorIndex = 1; AnchorIndex < PendingWrapSeed.Anchors.Num(); ++AnchorIndex)
	{
		const FRopeSurfaceAnchor& Secondary = PendingWrapSeed.Anchors[AnchorIndex];
		if (Secondary.NodeIndex > LatchAnchor.NodeIndex && Sim.Positions.IsValidIndex(Secondary.NodeIndex))
		{
			WrappingPhase.State.SecondarySeedAnchors.Add(Secondary);
		}
	}

	if (!WrappingPhase.Begin(LatchAnchor,
		FMath::Max(0.01f, WrapConfig.WrappingMotionDuration), Sim, MakeWrappingContext()))
	{
		LogWrappingFailureState(GetName(), TEXT("StartWrapping.Begin"), WrappingPhase.State, Sim);
		// Left before the wrap was established: report per-instance after cleaning up, through
		// FinishPreCommitReleaseToFlight.
		FinishPreCommitReleaseToFlight(ContactTracker.CandidateBone, TEXT("no valid wrapping anchors"));
		return;
	}

	SetPhase(ERopePhase::Wrapping, *FString::Printf(TEXT("bone=%s, %d anchor(s), %d secondary seed(s)"),
		*WrappingPhase.State.BoneName.ToString(), WrappingPhase.State.Anchors.Num(),
		WrappingPhase.State.SecondarySeedAnchors.Num()));
}

void URopeComponent::UpdateWrapping(float DeltaTime)
{
	WrappingPhase.State.Elapsed += DeltaTime;

	if (!WrappingPhase.IsStillValid())
	{
		if (WrappingPhase.State.PathBuildFailureReason.IsEmpty())
		{
			WrappingPhase.State.PathBuildFailureReason = TEXT("InvalidWrappingState");
		}
		LogWrappingFailureState(GetName(), TEXT("UpdateWrapping.IsStillValid"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, TEXT("invalid wrapping state"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	const FRopeWrappingPhase::FContext WrappingCtx = MakeWrappingContext();
	WrappingPhase.AdvancePathBuild(Sim, WrappingCtx);

	// When the composite analytic helix falls back to a single bone after a terminal failure, the virtual
	// nodes already pinned are returned to the solver immediately. The new single-bone path's front and
	// mass overrides are reapplied on the same frame below.
	if (!WrappingPhase.State.bPathUsesPoseSpaceIsland &&
		(KinematicVirtualBridges.Num() > 0 || KinematicVirtualBridgeRunCursor > 0))
	{
		ReleaseKinematicVirtualBridgesToSolver();
	}

	// The last safety net, for the case where initializing the fallback itself failed. Projection failures
	// arising later during the single-bone run are left to the existing partial-path quality test.
	if (WrappingPhase.State.bPathBuildFailed &&
		WrappingPhase.State.PathBuildFailureReason == TEXT("SingleBoneFallbackInitializationFailure"))
	{
		LogWrappingFailureState(GetName(), TEXT("UpdateWrapping.CompositeFallbackExhausted"),
			WrappingPhase.State, Sim);
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Wrap cancelled: algorithm=CompositeAnalyticHelix->SingleBone reason=FallbackInitializationFailed "
				"path=%d/%d anchors=%d"),
			*GetName(), WrappingPhase.State.Path.Num(), WrappingPhase.State.NumTailNodes,
			WrappingPhase.State.Anchors.Num());
		SetPhase(ERopePhase::Releasing, TEXT("Composite analytic helix and fallback initialization both failed"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// A safeguard: when surface path generation fails part-way through, and the angle wrapped up to that
	// point is below the threshold, the rope releases instead of latching onto something it barely
	// touched. The measure is the wrapped angle in degrees, FailedWrapMinAngleDeg, where 0 disables the
	// guard, rather than a number of turns: turns demand a rope of 2*pi*r and become physically
	// unreachable on a large target such as a dragon's torso. See the config comment for detail.
	float FailedWrapAngleDeg = 0.0f;
	if (WrappingPhase.ShouldAbortFailedShortWrap(Sim, WrappingCtx, WrapConfig.FailedWrapMinAngleDeg, FailedWrapAngleDeg))
	{
		LogWrappingFailureState(GetName(), TEXT("UpdateWrapping.FailedShortWrap"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("wrap path failed early, angle=%.0fdeg < %.0fdeg"),
			FailedWrapAngleDeg, WrapConfig.FailedWrapMinAngleDeg));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	WrappingPhase.ApplyWrappingMotionOverrides(Sim, DeltaTime, WrappingCtx, SimFrame.OverrideFrame);

	WrappingPhase.ApplyWrappingKinematicMask(Sim, SimFrame.OverrideFrame);

	if (WrappingPhase.State.bPathUsesPoseSpaceIsland)
	{
		// It runs after ApplyWrappingMotionOverrides has pinned the real surface node on the right and
		// ApplyWrappingKinematicMask has returned the virtual nodes to their default dynamic state, so this
		// final override tightens only the stretch the front has closed.
		UpdateWrappingKinematicVirtualBridges(
			WrappingPhase.State.VirtualBridgeRuns,
			WrappingPhase.State.Anchors,
			WrappingPhase.State.FrontDistance);
		HoldKinematicVirtualBridges();
	}

	WrappingPhase.UpdateAnchorSpanStability(DeltaTime);

	if (WrappingPhase.IsReadyToCommit(Sim, WrapConfig))
	{
		CommitWrapping();
		return;
	}
}

FRopeWrappingPhase::FContext URopeComponent::MakeWrappingContext() const
{
	// A fallback for CaptureTravelPlane alone: on a throw with no whip guide plane, such as a direct
	// Blueprint call, the travel plane normal derived from the capture-time snapshot, that is the velocity
	// crossed with the direction the rope lies in, is supplied instead. Nothing is injected under the
	// default, BoneCenteredGuidePlane, which uses only the normal obtained from the whip guide.
	bool bGuidePlane = bHasFlightGuidePlaneNormal;
	FVector GuidePlane = FlightGuidePlaneNormal;
	if (!bGuidePlane &&
		WrapConfig.WrappingAxisSource == ERopeWrappingAxisSource::CaptureTravelPlane &&
		CaptureTravelFrame.bValid && CaptureTravelFrame.bHasPlaneNormal)
	{
		bGuidePlane = true;
		GuidePlane = CaptureTravelFrame.PlaneNormal;
	}

	// Apply the wrap target gate, CanWrapTarget, to the wrapping path as well, which prevents the
	// inconsistency of the path build alone picking up and anchoring to a target that aiming, the preview
	// and the judgement have all already filtered out. With the default gate, which permits everything,
	// the result is identical to the frame colliders.
	RopeWrapTargets::FilterWrappableColliders(SimFrame.FrameColliders,
		[this](const USceneComponent* Mesh, FName Bone) { return CanWrapTarget(Mesh, Bone); },
		WrappableColliders);

	FRopeWrappingPhase::FContext Ctx{
		WrapConfig,
		WrappableColliders,
		Radius,
		GetName(),
		false,
		bGuidePlane,
		GuidePlane,
		CaptureTravelFrame.bValid ? &CaptureTravelFrame : nullptr
	};
	// Resolving the automatic value is the component boundary's responsibility. Unlike the preview path,
	// which overwrites a copy inside its input struct, the config here is passed by reference, so the
	// resolved value is carried in a separate field. Without injecting it, a rope with an automatic
	// contact query radius would fall through to a query radius of zero on the wrapping path alone.
	Ctx.ResolvedContactRadius = GetEffectiveContactQueryRadius();
	Ctx.ResolveMode = ResolveMode;
	return Ctx;
}

void URopeComponent::CommitWrapping()
{
	const USceneComponent* Mesh = WrappingPhase.State.Mesh.Get();

	// Release immediately if the wrapping information is not usable.
	if (!Mesh || WrappingPhase.State.BoneName.IsNone() || WrappingPhase.State.Anchors.Num() == 0)
	{
		WrappingPhase.State.PathBuildFailureReason = TEXT("CommitStateInvalid");
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.StateValidation"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, TEXT("commit failed"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// The wrapped angle at commit time, in degrees, shared by the commit quality gate below and the
	// transition log. It is the same measure the early abort on failure uses, so the angle in the log can
	// be compared and tuned against directly. Where it cannot be computed, as with a degenerate axis, it
	// is reported as -1.
	float CommitAngleDeg = -1.0f;
	WrappingPhase.ComputeBuiltPathWrapAngle(Sim, MakeWrappingContext(), CommitAngleDeg);

	// The commit quality gate, opt-in through CommitMinWrapAngleDeg where 0 keeps the previous behaviour:
	// even when the path completed normally or arrived through the settle timeout, a poor wrap whose angle
	// is below the floor is not confirmed as Wrapped.
	if (WrapConfig.CommitMinWrapAngleDeg > 0.0f && CommitAngleDeg >= 0.0f
		&& CommitAngleDeg < WrapConfig.CommitMinWrapAngleDeg)
	{
		WrappingPhase.State.PathBuildFailureReason = FString::Printf(
			TEXT("CommitAngleBelowThreshold(%.1f<%.1f)"), CommitAngleDeg, WrapConfig.CommitMinWrapAngleDeg);
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.AngleQualityGate"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("commit quality below threshold, angle=%.0fdeg < %.0fdeg"),
			CommitAngleDeg, WrapConfig.CommitMinWrapAngleDeg));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// The shape-based binding gate, opt-in through CommitMinWrapCoverageDeg where 0 keeps the previous
	// behaviour: when the angular coverage about the axis, a geometric measure that oscillation does not
	// inflate as described on FRopeWrapConfig, is below the floor, the wrap has failed to enclose the
	// target and is not committed. Where it cannot be computed, from too few path points or a degenerate
	// axis and reported as -1, the gate is skipped rather than penalizing a rope for being hard to
	// measure. It is always included in the transition log as an observation for tuning.
	float CommitCoverageDeg = -1.0f;
	if (!WrappingPhase.ComputeWrapEnclosureCoverage(CommitCoverageDeg))
	{
		CommitCoverageDeg = -1.0f;
	}
	if (WrapConfig.CommitMinWrapCoverageDeg > 0.0f && CommitCoverageDeg >= 0.0f
		&& CommitCoverageDeg < WrapConfig.CommitMinWrapCoverageDeg)
	{
		WrappingPhase.State.PathBuildFailureReason = FString::Printf(
			TEXT("CommitCoverageBelowThreshold(%.1f<%.1f)"), CommitCoverageDeg,
			WrapConfig.CommitMinWrapCoverageDeg);
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.CoverageQualityGate"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("commit enclosure below threshold, coverage=%.0fdeg < %.0fdeg"),
			CommitCoverageDeg, WrapConfig.CommitMinWrapCoverageDeg));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	if (WrappingPhase.State.bPathUsesPoseSpaceIsland)
	{
		int32 BridgePointCount = 0;
		int32 VirtualPointCount = 0;
		int32 SurfaceSwitchCount = 0;
		TArray<FName, TInlineAllocator<16>> VisitedBones;
		FName PreviousSurfaceBone = NAME_None;
		for (int32 PathIndex = 0; PathIndex < WrappingPhase.State.Path.Num(); ++PathIndex)
		{
			const FRopeWrapPathPoint& Point = WrappingPhase.State.Path[PathIndex];
			BridgePointCount += Point.bBridge ? 1 : 0;
			VirtualPointCount += Point.bVirtual ? 1 : 0;
			if (!Point.bBridge && !Point.bVirtual && !Point.Bone.IsNone())
			{
				VisitedBones.AddUnique(Point.Bone);
				if (!PreviousSurfaceBone.IsNone() && Point.Bone != PreviousSurfaceBone)
				{
					++SurfaceSwitchCount;
				}
				PreviousSurfaceBone = Point.Bone;
			}
		}

		FString VisitedBoneList;
		for (const FName Bone : VisitedBones)
		{
			if (!VisitedBoneList.IsEmpty())
			{
				VisitedBoneList += TEXT(",");
			}
			VisitedBoneList += Bone.ToString();
		}
		const float BuiltDistance = WrappingPhase.State.Path.Num() > 0
			? WrappingPhase.State.Path.Last().DistanceFromLatch
			: 0.0f;
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Composite wrap path summary: points=%d surfacePoints=%d bridgePoints=%d virtualPoints=%d "
				"switches=%d visitedBones=%d builtDistance=%.2fcm angle=%.0fdeg coverage=%.0fdeg "
				"projectionMisses=%d bones=[%s]"),
			*GetName(), WrappingPhase.State.Path.Num(),
			WrappingPhase.State.Path.Num() - BridgePointCount - VirtualPointCount,
			BridgePointCount, VirtualPointCount,
			SurfaceSwitchCount, VisitedBones.Num(), BuiltDistance,
			CommitAngleDeg, CommitCoverageDeg,
			WrappingPhase.State.PathCompositeProjectionFailureCount, *VisitedBoneList);
	}

	const FRopeWrapState Seed = WrappingPhase.BuildCommitSeed(Sim);
	if (Seed.Anchors.Num() == 0)
	{
		WrappingPhase.State.PathBuildFailureReason = TEXT("CommitSeedHasNoValidAnchors");
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.BuildCommitSeed"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, TEXT("no valid latches"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// The bridges built during Wrapping are not discarded and Path is not scanned again. The anchors of
	// the final commit seed are reinterpreted as authoritative, which refreshes only the bindings of the
	// same bridges and activates them all.
	if (!FinalizeKinematicVirtualBridges(WrappingPhase.State.VirtualBridgeRuns, Seed.Anchors))
	{
		WrappingPhase.State.PathBuildFailureReason = TEXT("KinematicVirtualBridgeFinalizeFailed");
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.FinalizeVirtualBridges"),
			WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, TEXT("virtual bridge finalization failed"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// The mesh to wrap propagates through Seed.Mesh, which came from the contact and covers the
	// cross-actor case.
	WrapController.BeginWrap(Sim, Seed, SimFrame.OverrideFrame);
	HoldKinematicVirtualBridges();
	ApplyWrappedMassMask(/*bResetDynamicNodeVelocity*/ true);
	// Stops a non-anchor node released on this frame carrying its remaining strain into the next wrapped
	// tick. The next Prepare resets it to false and returns to the user's configured maximum stretch for
	// the settled wrapped state.
	SimFrame.bForceNonStretchThisFrame = true;

	SetPhase(ERopePhase::Wrapped, *FString::Printf(TEXT("bone=%s, %d latched node(s), angle=%.0fdeg, coverage=%.0fdeg"),
		*Seed.BoneName.ToString(), Seed.Latched.Num(), CommitAngleDeg, CommitCoverageDeg));
	// Wrapping PrePhysics may already have created the Chaos tether. A successful commit keeps its
	// identity/warm-start; abort/release paths still use the default teardown.
	ResetTransientPhaseState(/*bPreservePhysicalTether*/ true);
	const FRopeWrappedEventInfo WrappedInfo = MakeWrappedEventInfo(Seed, CommitAngleDeg, CommitCoverageDeg);
	DispatchWrapped(WrappedInfo);
}

void URopeComponent::DispatchWrapped(const FRopeWrappedEventInfo& Info)
{
	// The single broadcast point for an established wrap: the native subclass hook, then the per-instance
	// Blueprint delegate, then the world's central signal, which lets a target reaction component respond
	// by subscribing without knowing which rope wrapped it. It is shared by both routes to an established
	// wrap, the GuaranteedWrap preview and the judged one.
	// Release notifications are queued for the duration, so even if a handler immediately calls
	// ReleaseWrap, subscribers always observe the order wrapped then released; see the comment on
	// FDeferredReleaseNotice. No code in the notification can throw, but pairing the depth increment with
	// its decrement is this function's single responsibility, so it always decrements at the end.
	++WrappedDispatchDepth;
	NotifyWrapped(Info);
	OnRopeWrapped.Broadcast(Info);
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->OnAnyRopeWrapped.Broadcast(Info);
	}
	--WrappedDispatchDepth;

	FlushDeferredReleaseNotices();
}

void URopeComponent::FlushDeferredReleaseNotices()
{
	// While a nested wrapped notification is still outstanding, only the outermost one drains, which is
	// the single point that guarantees the ordering.
	if (WrappedDispatchDepth > 0 || DeferredReleaseNotices.Num() == 0)
	{
		return;
	}

	// A handler may call release again while draining, so the queue is emptied first and a copy is walked.
	TArray<FDeferredReleaseNotice> Pending = MoveTemp(DeferredReleaseNotices);
	DeferredReleaseNotices.Reset();
	for (const FDeferredReleaseNotice& Notice : Pending)
	{
		DispatchReleased(Notice.WrappedMesh.Get(), Notice.Bone, Notice.Reason, Notice.bWasWrapped);
	}
}

FRopeWrappedEventInfo URopeComponent::MakeWrappedEventInfo(const FRopeWrapState& Seed,
	float AngleDeg, float CoverageDeg) const
{
	FRopeWrappedEventInfo Info;
	Info.Bone = Seed.BoneName;
	// The rope that established it, which is the engagement set key for subscribers of the central signal.
	// The release signal carries the same pointer.
	Info.Rope = const_cast<URopeComponent*>(this);
	// The event payload is read-only by intent, so the target mesh's constness is cast away purely to
	// expose it to Blueprint; it is not a licence to modify it.
	Info.Mesh = const_cast<USceneComponent*>(Seed.Mesh.Get());
	Info.ResolveMode = ResolveMode;
	Info.AngleDeg = AngleDeg;
	Info.CoverageDeg = CoverageDeg;
	Info.AnchorCount = Seed.Anchors.Num();

	// Every bone the anchors span, with the representative bone first and duplicates removed, which is the
	// full picture when a wrap establishes across several bones such as both legs.
	if (!Seed.BoneName.IsNone())
	{
		Info.Bones.Add(Seed.BoneName);
	}
	for (const FRopeSurfaceAnchor& Anchor : Seed.Anchors)
	{
		if (!Anchor.Bone.IsNone())
		{
			Info.Bones.AddUnique(Anchor.Bone);
		}
	}
	return Info;
}

void URopeComponent::AbortWrapping(ERopeReleaseReason Reason)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] AbortWrapping reason=%d"),
		*GetName(), static_cast<int32>(Reason));

	// Pairing with Captured: Wrapping is only ever entered from Contacting, which fires Captured, so this
	// abort always pairs with a preceding one. It is before the commit, so only the per-instance delegate
	// fires; the central signal is for committed wraps alone. The bone name is captured by value and
	// reported only after the cleanup is complete, following the same contract as
	// FinishPreCommitReleaseToFlight: reporting first would run a handler's ReleaseWrap or re-throw on top
	// of wrapping state that is still alive, and the cleanup below would then overwrite the result.
	// Every caller enters having already called SetPhase(Releasing).
	const FName AbortedBone = WrappingPhase.State.BoneName;

	WrappingPhase.ReleaseAnchoredNodesToSolver(Sim, SimFrame.OverrideFrame);
	ReleaseKinematicVirtualBridgesToSolver();

	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;

	DispatchReleased(nullptr, AbortedBone, Reason, /*bWasWrapped*/ false);
}

void URopeComponent::UpdateWrappingKinematicVirtualBridges(
	const TArray<FRopeVirtualBridgeRun>& Runs,
	const TArray<FRopeSurfaceAnchor>& Anchors, float FrontDistance)
{
	// The wrapping phase discovers each virtual run in Path exactly once. The component connects the
	// anchors on both sides of a run it has not consumed yet to build the runtime bridge once, and retries
	// from the same run next frame when an anchor is late.
	KinematicVirtualBridgeRunCursor = FMath::Clamp(
		KinematicVirtualBridgeRunCursor, 0, Runs.Num());
	while (KinematicVirtualBridgeRunCursor < Runs.Num())
	{
		const FRopeVirtualBridgeRun& Run = Runs[KinematicVirtualBridgeRunCursor];
		const FRopeSurfaceAnchor* LeftAnchor = FindSurfaceAnchorByNode(Anchors, Run.LeftNodeIndex);
		const FRopeSurfaceAnchor* RightAnchor = FindSurfaceAnchorByNode(Anchors, Run.RightNodeIndex);
		if (!LeftAnchor || !RightAnchor)
		{
			// Where the order of path points and anchor additions diverged, the current run is left
			// unconsumed and retried next frame.
			break;
		}

		const USceneComponent* LeftMesh = LeftAnchor->Mesh.Get();
		const USceneComponent* RightMesh = RightAnchor->Mesh.Get();
		if (LeftMesh && LeftMesh == RightMesh)
		{
			FKinematicVirtualBridge& Bridge = KinematicVirtualBridges.AddDefaulted_GetRef();
			Bridge.NodeIndices = Run.VirtualNodeIndices;
			Bridge.LeftAnchor = *LeftAnchor;
			Bridge.RightAnchor = *RightAnchor;
			Bridge.RestSpanLength = static_cast<float>(Run.RightNodeIndex - Run.LeftNodeIndex) * Sim.SegmentLength;
			Bridge.ActivationFrontDistance = RightAnchor->RopeDistance;
			Bridge.bActive = false;

			UE_LOG(LogRopeWrap, Verbose,
				TEXT("[%s] Wrapping virtual bridge registered: leftNode=%d rightNode=%d "
					"virtualNodes=%d activationDistance=%.2fcm"),
				*GetName(), Run.LeftNodeIndex, Run.RightNodeIndex, Bridge.NodeIndices.Num(),
				Bridge.ActivationFrontDistance);
		}
		++KinematicVirtualBridgeRunCursor;
	}

	// The front reaching the distance of the right-hand anchor means ApplyWrappingMotionOverrides has
	// pinned both boundaries to real surface positions. From that frame the interior virtual nodes are
	// tied into a straight line, which removes the sway before Wrapped.
	const float FrontTolerance = FMath::Max(0.01f, Sim.SegmentLength * 0.001f);
	for (FKinematicVirtualBridge& Bridge : KinematicVirtualBridges)
	{
		if (!Bridge.bActive && FrontDistance + FrontTolerance >= Bridge.ActivationFrontDistance)
		{
			Bridge.bActive = true;
			UE_LOG(LogRopeWrap, Log,
				TEXT("[%s] Wrapping virtual bridge activated: leftNode=%d rightNode=%d "
					"virtualNodes=%d front=%.2fcm activation=%.2fcm"),
				*GetName(), Bridge.LeftAnchor.NodeIndex, Bridge.RightAnchor.NodeIndex,
				Bridge.NodeIndices.Num(), FrontDistance, Bridge.ActivationFrontDistance);
		}
	}
}

bool URopeComponent::FinalizeKinematicVirtualBridges(
	const TArray<FRopeVirtualBridgeRun>& Runs,
	const TArray<FRopeSurfaceAnchor>& CommitAnchors)
{
	// The commit neither searches for the same run again nor recreates the bridge. It first confirms that
	// the list produced and registered during Wrapping is complete, then replaces only the bindings with
	// copies of the final seed anchors.
	if (KinematicVirtualBridgeRunCursor != Runs.Num() || KinematicVirtualBridges.Num() != Runs.Num())
	{
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Virtual bridge finalization rejected: discoveredRuns=%d consumedRuns=%d bridges=%d"),
			*GetName(), Runs.Num(), KinematicVirtualBridgeRunCursor, KinematicVirtualBridges.Num());
		return false;
	}

	for (int32 RunIndex = 0; RunIndex < Runs.Num(); ++RunIndex)
	{
		const FRopeVirtualBridgeRun& Run = Runs[RunIndex];
		FKinematicVirtualBridge& Bridge = KinematicVirtualBridges[RunIndex];
		if (Bridge.LeftAnchor.NodeIndex != Run.LeftNodeIndex ||
			Bridge.RightAnchor.NodeIndex != Run.RightNodeIndex ||
			Bridge.NodeIndices != Run.VirtualNodeIndices)
		{
			UE_LOG(LogRopeWrap, Error,
				TEXT("[%s] Virtual bridge finalization rejected: run %d no longer matches registered bridge"),
				*GetName(), RunIndex);
			return false;
		}

		const FRopeSurfaceAnchor* LeftAnchor = FindSurfaceAnchorByNode(CommitAnchors, Run.LeftNodeIndex);
		const FRopeSurfaceAnchor* RightAnchor = FindSurfaceAnchorByNode(CommitAnchors, Run.RightNodeIndex);
		if (!LeftAnchor || !RightAnchor)
		{
			UE_LOG(LogRopeWrap, Error,
				TEXT("[%s] Virtual bridge finalization rejected: run=%d missing anchor left=%d right=%d"),
				*GetName(), RunIndex, Run.LeftNodeIndex, Run.RightNodeIndex);
			return false;
		}

		const USceneComponent* LeftMesh = LeftAnchor->Mesh.Get();
		const USceneComponent* RightMesh = RightAnchor->Mesh.Get();
		if (!LeftMesh || LeftMesh != RightMesh)
		{
			UE_LOG(LogRopeWrap, Error,
				TEXT("[%s] Virtual bridge finalization rejected: run=%d crosses components left=%s right=%s"),
				*GetName(), RunIndex, *GetNameSafe(LeftMesh), *GetNameSafe(RightMesh));
			return false;
		}

		Bridge.LeftAnchor = *LeftAnchor;
		Bridge.RightAnchor = *RightAnchor;
		Bridge.ActivationFrontDistance = RightAnchor->RopeDistance;
		Bridge.bActive = true;
	}

	if (Runs.Num() > 0)
	{
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Kinematic virtual bridges finalized in place: runs=%d"),
			*GetName(), KinematicVirtualBridges.Num());
	}
	return true;
}

void URopeComponent::HoldKinematicVirtualBridges()
{
	if (KinematicVirtualBridges.Num() == 0)
	{
		return;
	}

	SimFrame.OverrideFrame.EnsureSize(Sim.Num());
	for (FKinematicVirtualBridge& Bridge : KinematicVirtualBridges)
	{
		if (!Bridge.bActive)
		{
			// The stretch where path generation is complete but the front has not yet reached the
			// right-hand anchor is left to the solver.
			continue;
		}

		FVector LeftWorld;
		FVector RightWorld;
		if (!ResolveAnchorCenterlineWorld(Bridge.LeftAnchor, LeftWorld) ||
			!ResolveAnchorCenterlineWorld(Bridge.RightAnchor, RightWorld))
		{
			continue;
		}

		const int32 NumBridgeSegments = Bridge.NodeIndices.Num() + 1;
		if (NumBridgeSegments <= 1)
		{
			continue;
		}

		const float ChordLength = static_cast<float>(FVector::Dist(LeftWorld, RightWorld));
		const float ChordSegmentLength = ChordLength / static_cast<float>(NumBridgeSegments);
		const float StretchRatio = ChordLength /
			FMath::Max(Bridge.RestSpanLength, KINDA_SMALL_NUMBER);
		if (StretchRatio > KinematicBridgeStretchWarningRatio && !Bridge.bLoggedStretchWarning)
		{
			Bridge.bLoggedStretchWarning = true;
			UE_LOG(LogRopeWrap, Warning,
				TEXT("[%s] Kinematic virtual bridge stretched: nodes=%d chord=%.2fcm "
					"restSpan=%.2fcm segment=%.2fcm ratio=%.2f"),
				*GetName(), Bridge.NodeIndices.Num(), ChordLength,
				Bridge.RestSpanLength, ChordSegmentLength, StretchRatio);
		}

		for (int32 BridgeNodeIndex = 0; BridgeNodeIndex < Bridge.NodeIndices.Num(); ++BridgeNodeIndex)
		{
			const int32 NodeIndex = Bridge.NodeIndices[BridgeNodeIndex];
			if (!Sim.Positions.IsValidIndex(NodeIndex) || !Sim.InvMass.IsValidIndex(NodeIndex))
			{
				continue;
			}

			const float Alpha = static_cast<float>(BridgeNodeIndex + 1) /
				static_cast<float>(NumBridgeSegments);
			const FVector TargetWorld = FMath::Lerp(LeftWorld, RightWorld, Alpha);
			// Overwrite both the current and previous positions so the correction injects no velocity, and
			// hard pin it so the solver cannot push it again.
			SimFrame.OverrideFrame.SetPosition(NodeIndex, TargetWorld, /*bZeroVelocity*/ true);
			SimFrame.OverrideFrame.SetInvMass(NodeIndex, 0.0f);
		}
	}
}

void URopeComponent::ResetKinematicVirtualBridges()
{
	// The bridge bindings and the cursor consuming the wrapping phase's output have to be cleared together,
	// or the next wrap would carry on reading the previous run. This function does not restore mass, so
	// use the release helpers to deactivate a live bridge.
	KinematicVirtualBridges.Reset();
	KinematicVirtualBridgeRunCursor = 0;
	bWrappedMassMaskDirty = true;
}

void URopeComponent::ReleaseKinematicVirtualBridgesToSolver()
{
	// The path shared by the composite-to-single fallback, aborts and a normal release. Every registered
	// virtual node is returned to dynamic mass regardless of whether it is active, so no stale hard pin
	// survives into the next phase.
	if (KinematicVirtualBridges.Num() > 0)
	{
		SimFrame.OverrideFrame.EnsureSize(Sim.Num());
		for (const FKinematicVirtualBridge& Bridge : KinematicVirtualBridges)
		{
			for (const int32 NodeIndex : Bridge.NodeIndices)
			{
				if (!Sim.InvMass.IsValidIndex(NodeIndex))
				{
					continue;
				}

				// Stop it at its current position so the difference between the current and previous
				// positions does not become a velocity spike on the frame the bridge is released.
				const bool bStartPin = NodeIndex == 0 && Sim.bStartPinned;
				SimFrame.OverrideFrame.SetInvMass(NodeIndex, bStartPin ? 0.0f : 1.0f);
				SimFrame.OverrideFrame.SetPrevFromPosition(NodeIndex);
			}
		}
	}

	ResetKinematicVirtualBridges();
}

#pragma endregion Wrapping

#pragma region Wrapped_Hold_And_Pull_Sampling

// ===== Wrapped ==============================================================
// The Wrapped case of PrepareSimFrame runs the four helpers below in a fixed order:
// HoldWrappedNodesToBone → UpdateWrappedPullSample → ApplyWrappedTraction → CheckWrappedAutoRelease

bool URopeComponent::HoldWrappedNodesToBone(float DeltaTime)
{
	// Step one: latched nodes follow their skinned bones on the game thread. A latched node has an inverse
	// mass of 0, so the solve covers only the free stretches.
	// Hold returning false means the wrapped mesh is gone, as when a cross-actor target actor is
	// destroyed, so the nodes are returned to the solver and the rope releases safely; Hold itself is what
	// avoids dereferencing the dangling pointer.
	if (!WrapController.Hold(Sim, DeltaTime, SimFrame.OverrideFrame))
	{
		const FName Bone = WrapController.State.BoneName;
		FinishWrapRelease(Bone, ERopeReleaseReason::Broken,
			FString::Printf(TEXT("wrap target mesh lost, bone=%s"), *Bone.ToString()));
		return false;
	}
	HoldKinematicVirtualBridges();
	if (bWrappedMassMaskDirty)
	{
		ApplyWrappedMassMask();
	}
	return true;
}

bool URopeComponent::CheckWrappedAutoRelease(float DeltaTime)
{
	// Step three, for GuaranteedWrap: the automatic tension and distance releases do not apply, because
	// the guarantee of establishing without fail is symmetric and only an explicit release, whether
	// ReleaseWrap, CutRope or a game event, is valid. The other modes behave as before.
	// A release caused by losing the target mesh is a safety contract rather than an automatic release, so
	// it applies in every mode; see HoldWrappedNodesToBone.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		return false;
	}

	// Step four, part one, the tension release: the rope lets go once the maximum tension has stayed above
	// TensionReleaseForce for TensionReleaseTime, which ignores momentary spikes. 0 disables it. The flow
	// matches the lost-mesh release and only the reason differs.
	if (HoldConfig.TensionReleaseForce > 0.0f)
	{
		TensionOverTime = (WrapController.State.Tension > HoldConfig.TensionReleaseForce)
			? TensionOverTime + DeltaTime : 0.0f;
		if (TensionOverTime >= HoldConfig.TensionReleaseTime)
		{
			const FName Bone = WrapController.State.BoneName;
			FinishWrapRelease(Bone, ERopeReleaseReason::Tension,
				FString::Printf(TEXT("tension release %.0f > %.0f, bone=%s"),
					WrapController.State.Tension, HoldConfig.TensionReleaseForce, *Bone.ToString()));
			return true;
		}
	}

	// Step four, part two, the distance release: the rope lets go once the straight-line distance from the
	// hand to the anchor exceeds the available rope length by more than the limit. It uses the same source
	// as the tether overshoot, namely the violation UpdateConstraintTether refreshed this frame.
	// Being geometric it is decided immediately with no duration, since unlike tension it carries no noise.
	if (HoldConfig.DistanceReleaseSlack > 0.0f &&
		LengthConstraintState.LastViolation > HoldConfig.DistanceReleaseSlack)
	{
		const FName Bone = WrapController.State.BoneName;
		FinishWrapRelease(Bone, ERopeReleaseReason::Distance,
			FString::Printf(TEXT("distance release overshoot %.0f > %.0f, bone=%s"),
				LengthConstraintState.LastViolation, HoldConfig.DistanceReleaseSlack, *Bone.ToString()));
		return true;
	}
	return false;
}

void URopeComponent::ApplyWrappedMassMask(bool bResetDynamicNodeVelocity)
{
	TSet<int32> AnchorNodes;

	for (const FRopeSurfaceAnchor& Anchor : WrapController.State.Anchors)
	{
		if (Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
		{
			AnchorNodes.Add(Anchor.NodeIndex);
		}
	}

	for (const FRopeLatchNode& Latch : WrapController.State.Latched)
	{
		if (Sim.InvMass.IsValidIndex(Latch.NodeIndex))
		{
			AnchorNodes.Add(Latch.NodeIndex);
		}
	}

	for (const FKinematicVirtualBridge& Bridge : KinematicVirtualBridges)
	{
		if (!Bridge.bActive)
		{
			continue;
		}

		for (const int32 NodeIndex : Bridge.NodeIndices)
		{
			if (Sim.InvMass.IsValidIndex(NodeIndex))
			{
				AnchorNodes.Add(NodeIndex);
			}
		}
	}

	SimFrame.OverrideFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		const bool bAnchor = AnchorNodes.Contains(i);
		const bool bFixed = bStartPin || bAnchor;
		SimFrame.OverrideFrame.SetInvMass(i, bFixed ? 0.0f : 1.0f);
		if (bResetDynamicNodeVelocity && !bFixed)
		{
			SimFrame.OverrideFrame.SetPrevFromPosition(i);
		}
	}
	bWrappedMassMaskDirty = false;
}

#pragma endregion Wrapped_Hold_And_Pull_Sampling

