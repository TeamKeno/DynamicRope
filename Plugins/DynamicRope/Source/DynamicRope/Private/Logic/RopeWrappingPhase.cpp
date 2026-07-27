// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrappingPhase.h"
#include "Components/SceneComponent.h"
// ResolveBindingWorld is the single point at which a wrap binding, meaning a bone, socket or component, is resolved to a transform.
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"
// RopeMath::AnyTangentFromNormal, shared to avoid a duplicate definition in a unity build.
#include "RopeMathHelpers.h"

#pragma region Wrapping Lifecycle, Commit, and Preview

bool FRopeWrappingPhase::Begin(const FRopeSurfaceAnchor& LatchAnchor, float Duration,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	// A new throw starts from the composite analytic helix when the conditions are met. This flag is set only when
	// that path ended in a terminal failure within the same throw, which prevents a second composite attempt and an
	// endless fallback.
	State.bPathUsesSingleBoneFallback = false;
	State.PathBuildFailureReason.Reset();
	State.PathCompositeProjectionFailureCount = 0;
	State.BoneName = LatchAnchor.Bone;
	State.Mesh = LatchAnchor.Mesh;
	State.Elapsed = 0.0f;
	State.Duration = Duration;
	State.FirstNode = TNumericLimits<int32>::Max();
	State.LastNode = INDEX_NONE;

	if (!BeginProgressiveWrapPathBuild(LatchAnchor, Sim, Ctx))
	{
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Wrap begin failed: reason=%s bone=%s node=%d mesh=%s simNodes=%d segment=%.2fcm colliders=%d"),
			*Ctx.OwnerName,
			State.PathBuildFailureReason.IsEmpty() ? TEXT("UnknownInitializationFailure") : *State.PathBuildFailureReason,
			*LatchAnchor.Bone.ToString(), LatchAnchor.NodeIndex,
			*GetNameSafe(LatchAnchor.Mesh.Get()),
			Sim.Num(), Sim.SegmentLength,
			Ctx.Colliders.Num());
		return false;
	}

	State.StableTime = 0.0f;
	State.LastStableFirstNode = State.FirstNode;
	State.LastStableLastNode = State.LastNode;
	State.LastStableAnchorCount = State.Anchors.Num();
	return true;
}

void FRopeWrappingPhase::UpdateAnchorSpanStability(float DeltaTime)
{
	const bool bSameStableSpan =
		State.FirstNode == State.LastStableFirstNode &&
		State.LastNode == State.LastStableLastNode &&
		State.Anchors.Num() == State.LastStableAnchorCount;

	if (bSameStableSpan)
	{
		State.StableTime += DeltaTime;
	}
	else
	{
		State.StableTime = 0.0f;
		State.LastStableFirstNode = State.FirstNode;
		State.LastStableLastNode = State.LastNode;
		State.LastStableAnchorCount = State.Anchors.Num();
	}
}

bool FRopeWrappingPhase::IsReadyToCommit(const FRopeSimState& Sim, const FRopeWrapConfig& Config) const
{
	const bool bHasEnoughAnchors = State.Anchors.Num() > 0;
	float MaxTailDelay = 0.0f;
	float BuiltPathMaxDistance = 0.0f;
	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		BuiltPathMaxDistance = FMath::Max(BuiltPathMaxDistance, Anchor.RopeDistance);
		MaxTailDelay = FMath::Max(MaxTailDelay,
			(Anchor.RopeDistance / SegmentLength) * Config.WrappingTailDelayPerSegment);
	}
	const float RequestedFrontDistance = State.NumTailNodes > 0
		? static_cast<float>(State.NumTailNodes - 1) * Sim.SegmentLength
		: 0.0f;
	const float CommitFrontDistance = State.bPathBuildFailed
		? BuiltPathMaxDistance
		: RequestedFrontDistance;

	// A normal angle-mapped path commits only once both the animation phase and the physical placement have finished.
	// A failed path build or a degenerate fallback may have no angle target, so those keep the previous distance policy.
	const bool bUseAngularCompletion = State.bFrontUsesAngleMapping &&
		State.bPathBuildComplete && !State.bPathBuildFailed &&
		State.FrontTargetWrapAngleRad > KINDA_SMALL_NUMBER;
	const float EffectiveCommitDistance = bUseAngularCompletion
		? State.FrontTargetDistance
		: CommitFrontDistance;
	const float DistanceCompletionTolerance = FMath::Max(0.01f, SegmentLength * 0.001f);
	const float AngleCompletionToleranceRad = FMath::DegreesToRadians(0.1f);
	const bool bDistanceDone = State.FrontDistance + DistanceCompletionTolerance >=
		EffectiveCommitDistance;
	const bool bAngleDone = !bUseAngularCompletion ||
		State.FrontWrapAngleRad + AngleCompletionToleranceRad >=
			State.FrontTargetWrapAngleRad;
	const bool bFrontDone = bAngleDone && bDistanceDone;

	// The previous per-segment deadline applies to the distance fallback alone. An angle-mapped path waits only for a
	// short settle after the front reaches its target, plus the minimum phase duration, which removes the duplicated tail delay.
	const bool bLegacyMotionDone = State.Elapsed >= State.Duration + MaxTailDelay;
	const bool bMinimumPhaseDurationDone = State.Elapsed >= State.Duration;
	const bool bPostFrontSettleDone = State.FrontReachedTargetElapsed >= 0.0f &&
		State.Elapsed + KINDA_SMALL_NUMBER >=
			State.FrontReachedTargetElapsed + FMath::Max(0.0f, Config.WrappingPostFrontSettleTime);
	const bool bCompletionTimingDone = bUseAngularCompletion
		? (bMinimumPhaseDurationDone && bPostFrontSettleDone)
		: bLegacyMotionDone;
	const bool bStable = !bUseAngularCompletion ||
		State.StableTime + KINDA_SMALL_NUMBER >= FMath::Max(0.0f, Config.WrappingStableTime);
	const bool bTimedOutWithAnchors =
		Config.WrappingMaxSettleTime > 0.0f &&
		State.Elapsed >= Config.WrappingMaxSettleTime;
	const bool bPathReadyToCommit =
		!State.bPathBuildActive ||
		State.bPathBuildComplete ||
		State.bPathBuildFailed;

	// A timeout does not skip the front's target itself, but as before it bypasses the timing gate as a last resort
	// for the case where stabilization or settling abnormally never finishes.
	const bool bNormalCompletion = bFrontDone && bStable && bCompletionTimingDone;
	const bool bTimeoutCompletion = bFrontDone && bTimedOutWithAnchors;
	return bHasEnoughAnchors && bPathReadyToCommit &&
		(bNormalCompletion || bTimeoutCompletion);
}

bool FRopeWrappingPhase::ShouldAbortFailedShortWrap(const FRopeSimState& Sim, const FContext& Ctx,
	float MinRequiredAngleDeg, float& OutAngleDeg) const
{
	OutAngleDeg = 0.0f;
	if (MinRequiredAngleDeg <= 0.0f || !State.bPathBuildFailed)
	{
		return false;
	}

	if (!ComputeBuiltPathWrapAngle(Sim, Ctx, OutAngleDeg))
	{
		return false;
	}

	return OutAngleDeg < MinRequiredAngleDeg;
}

FRopeWrapState FRopeWrappingPhase::BuildCommitSeed(const FRopeSimState& Sim) const
{
	FRopeWrapState Seed;
	Seed.BoneName = State.LatchAnchor.Bone;
	Seed.Mesh = ResolveWrappingMesh(State, State.LatchAnchor);

	// Fills in the information for each latched node.
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (!Sim.Positions.IsValidIndex(Anchor.NodeIndex))
		{
			continue;
		}

		FRopeLatchNode Latch;
		Latch.NodeIndex = Anchor.NodeIndex;
		Latch.Bone = Anchor.Bone;
		Seed.Latched.Add(Latch);
		Seed.Anchors.Add(Anchor);
	}

	// Seed multiplexing: the secondary seed anchors join the commit too. BeginWrap and Hold already support a bone and
	// mesh per anchor, so from here on they are treated exactly like path anchors. Duplicate nodes are prevented by
	// the path length clamp, guaranteed at build time, but are filtered once more in case a seed has drifted out of step.
	for (const FRopeSurfaceAnchor& Secondary : State.SecondarySeedAnchors)
	{
		if (!Sim.Positions.IsValidIndex(Secondary.NodeIndex))
		{
			continue;
		}

		const bool bNodeTaken = Seed.Anchors.ContainsByPredicate(
			[&Secondary](const FRopeSurfaceAnchor& Existing)
			{
				return Existing.NodeIndex == Secondary.NodeIndex;
			});
		if (bNodeTaken)
		{
			continue;
		}

		FRopeLatchNode Latch;
		Latch.NodeIndex = Secondary.NodeIndex;
		Latch.Bone = Secondary.Bone;
		Seed.Latched.Add(Latch);
		Seed.Anchors.Add(Secondary);
	}

	return Seed;
}

void FRopeWrappingPhase::ReleaseAnchoredNodesToSolver(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const
{
	OutFrame.EnsureSize(Sim.Num());
	auto ReturnNode = [&Sim, &OutFrame](int32 NodeIndex)
	{
		if (!Sim.InvMass.IsValidIndex(NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(NodeIndex) ||
			!Sim.Positions.IsValidIndex(NodeIndex))
		{
			return;
		}

		OutFrame.SetInvMass(NodeIndex, 1.0f);
		OutFrame.SetPrevFromPosition(NodeIndex);
	};

	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		ReturnNode(Anchor.NodeIndex);
	}
	// The secondary seed nodes are returned along with the rest on an abort, since their positions were being overwritten by the front hold and the same previous-equals-current jump has to be prevented.
	for (const FRopeSurfaceAnchor& Secondary : State.SecondarySeedAnchors)
	{
		ReturnNode(Secondary.NodeIndex);
	}
}

bool FRopeWrappingPhase::BuildPreviewCenterline(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx, TArray<FVector>& OutCenterline) const
{
	OutCenterline.Reset();
	if (!LatchAnchor.Mesh.IsValid() || LatchAnchor.Bone.IsNone() ||
		!Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) || Sim.Num() < 2)
	{
		return false;
	}

	FRopeWrapConfig PreviewConfig = Ctx.Config;
	PreviewConfig.WrappingPathBuildStepsPerFrame = 4096;
	FContext PreviewCtx{ PreviewConfig, Ctx.Colliders, Ctx.SurfaceOffset, Ctx.OwnerName, true };

	// The preview inherits the same axis, contact and resolve context as the runtime.
	PreviewCtx.bHasGuidePlaneNormal = Ctx.bHasGuidePlaneNormal;
	PreviewCtx.GuidePlaneNormal = Ctx.GuidePlaneNormal;
	PreviewCtx.TravelFrame = Ctx.TravelFrame;
	PreviewCtx.ResolvedContactRadius = Ctx.ResolvedContactRadius;
	PreviewCtx.ResolveMode = Ctx.ResolveMode;

	FRopeWrappingPhase PreviewPhase;
	if (!PreviewPhase.Begin(LatchAnchor,
		FMath::Max(0.01f, PreviewConfig.WrappingMotionDuration), Sim, PreviewCtx))
	{
		return false;
	}

	const int32 MaxIterations = FMath::Max(1, Sim.Num() * 4);
	for (int32 Iteration = 0;
		Iteration < MaxIterations &&
		PreviewPhase.State.bPathBuildActive &&
		!PreviewPhase.State.bPathBuildComplete &&
		!PreviewPhase.State.bPathBuildFailed;
		++Iteration)
	{
		PreviewPhase.AdvancePathBuild(Sim, PreviewCtx);
	}

	if (PreviewPhase.State.Path.Num() == 0)
	{
		return false;
	}

	OutCenterline = Sim.Positions;
	const float SurfaceOffset = FMath::Max(0.0f, PreviewCtx.SurfaceOffset);
	int32 LastDrivenNode = INDEX_NONE;
	for (int32 PathIndex = 0; PathIndex < PreviewPhase.State.Path.Num(); ++PathIndex)
	{
		const int32 NodeIndex = LatchAnchor.NodeIndex + PathIndex;
		if (!OutCenterline.IsValidIndex(NodeIndex))
		{
			break;
		}

		const FRopeWrapPathPoint& Point = PreviewPhase.State.Path[PathIndex];
		OutCenterline[NodeIndex] = GetPathPointCenterlineWorld(Point, SurfaceOffset);
		LastDrivenNode = NodeIndex;
	}

	if (OutCenterline.IsValidIndex(LastDrivenNode))
	{
		const float SegmentLength = FMath::Max(Sim.SegmentLength, 1.0f);
		for (int32 NodeIndex = LastDrivenNode + 1; NodeIndex < OutCenterline.Num(); ++NodeIndex)
		{
			OutCenterline[NodeIndex] = OutCenterline[NodeIndex - 1] - FVector::UpVector * SegmentLength;
		}
	}

	return OutCenterline.Num() >= 2;
}

#pragma endregion
