// Copyright 2026 TeamKeno. All Rights Reserved.

#include "RopeComponent.h"

#include "Collision/RopeCollider.h"
#include "CollisionQueryParams.h"
#include "Core/RopeWrapTarget.h"
#include "Debug/RopeDebugSnapshot.h"
#include "DynamicRopeLog.h"
// The aim blocking trace: the one engine world query the throw path makes.
#include "Engine/World.h"
#include "Logic/RopeThrowPreviewBuilder.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RopeComponentInternal.h"
#include "RopeMathHelpers.h"
#include "Subsystem/RopeSimSubsystem.h"

using RopeComponentPrivate::PhaseName;
using RopeComponentPrivate::ReleaseCooldownSeconds;
using RopeComponentPrivate::ResolveAimGuideHitWorld;

namespace
{
	void ClearPreparedGuideFrameLocal(FRopePreparedThrowPreview& Prepared)
	{
		Prepared.bUseGuideFrameLocal = false;
		Prepared.GuideFrameComponent = nullptr;
		Prepared.GuideFrameLocalPoints.Reset();
		Prepared.GuideFrameLocalOrigin = FVector::ZeroVector;
	}

	void InjectPinnedFrameVelocityForFreeReturn(FRopeSimState& Sim)
	{
		if (!Sim.bStartPinned || Sim.Num() < 2)
		{
			return;
		}

		const FVector PinDelta = Sim.StartPinTarget - Sim.StartPinPrev;
		if (PinDelta.IsNearlyZero())
		{
			return;
		}

		for (int32 i = 1; i < Sim.Num(); ++i)
		{
			if (!Sim.PrevPositions.IsValidIndex(i) || !Sim.InvMass.IsValidIndex(i) || Sim.InvMass[i] <= 0.0f)
			{
				continue;
			}

			Sim.PrevPositions[i] -= PinDelta;
		}
	}
}

#pragma region Throw_Public_API

void URopeComponent::Throw()
{
	// The frame basis convention is described on FRopeThrowContext::MakeDefault in RopeTypes.cpp. There
	// is exactly one customization point, ResolveThrowContext, and ThrowWithContext routes through it.
	ThrowWithContext(FRopeThrowContext::MakeDefault(*this, ThrowParams));
}

void URopeComponent::ThrowWithContext(const FRopeThrowContext& ThrowContext)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] Throw requested (phase=%s, mode=%d, forward=%s)"),
		*GetName(), PhaseName(Phase), static_cast<int32>(ResolveMode),
		*ThrowContext.FrameForward.GetSafeNormal().ToCompactString());

	EnsureRopeInitialized();

	// The direct Blueprint and AI path for GuaranteedWrap: Throw can be called without the wielder's
	// aiming flow and the guarantee still holds, because the component builds the prepared preview
	// itself and throws along the committed path. A successful build means the aimed target is wrapped
	// without fail; a failed one, whether there is no target or it is out of range, falls back to an
	// arcing throw towards the end of the ray rather than refusing.
	// The build parameters have a single source on the rope, so they always agree with the wielder path.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		// GuaranteedWrap is only throwable from the Loaded phase. After embedding and releasing back to
		// Free, EnterLoaded() must be called before throwing again.
		if (!CanThrowNow())
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Guaranteed throw rejected: not in Loaded (phase=%s). Call EnterLoaded() first."),
				*GetName(), PhaseName(Phase));
			return;
		}

		FRopePreparedThrowPreview Prepared;
		FString FailureReason;
		const FRopeThrowContext ResolvedThrow = ResolveThrowContext(ThrowContext);
		if (BuildPreparedWrappingPreviewFromResolvedContext(ResolvedThrow, Prepared, &FailureReason))
		{
			// The target was acquired, so it embeds without fail through GuidedThrow, which calls
			// OnDeployFromLoaded internally.
			if (!ThrowWithPreparedPreview(Prepared))
			{
				UE_LOG(LogDynamicRope, Log, TEXT("[%s] Guaranteed prepared throw failed after build: %s"),
					*GetName(), FailureReason.IsEmpty() ? TEXT("prepared throw failed") : *FailureReason);
			}
			return;
		}

		// The preview failed, whether there is no target or it is out of range, so rather than refusing
		// this falls back to an arcing throw towards the end of the ray, which embeds in nothing and
		// falls to Free. Aimed and arcing throws share one path, since the guarantee applies to the
		// target that was aimed at.
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Guaranteed throw: no aim target (%s) — arc toss toward ray-end."),
			*GetName(), FailureReason.IsEmpty() ? TEXT("no preview") : *FailureReason);
		// The throw strength contract is checked before any state changes in OnDeployFromLoaded.
		float FreeThrowSpeed = 0.0f;
		if (!TryResolveValidThrowSpeed(ResolvedThrow, FreeThrowSpeed))
		{
			return;
		}
		const FVector FreeEndpoint = ResolveFreeThrowEndpoint(ResolvedThrow);
		OnDeployFromLoaded();
		StartFreeGuidedThrow(ResolvedThrow, FreeEndpoint);
		return;
	}

	StartFreshThrow(ThrowContext);
}

bool URopeComponent::ThrowWithPreparedPreview(const FRopePreparedThrowPreview& Prepared)
{
	// The core entry point for GuaranteedWrap. Unlike StartFreshThrow it does not send the rope to
	// Flight: the path, contact and anchor the preview build chose have to be authoritative, or the
	// real outcome would diverge from the preview.
	EnsureRopeInitialized();
	if (!Prepared.IsValid() || Prepared.RenderPreview.Points.Num() < 2 || Sim.Num() < 2)
	{
		return false;
	}

	// GuaranteedWrap is only throwable from the Loaded phase; this is the same gate as ThrowWithContext,
	// guarding against the wielder calling in directly.
	if (!CanThrowNow())
	{
		return false;
	}

	// The subclass wrap target gate. The preview build is a static builder and knows nothing about it,
	// so it is applied here at the entry point.
	if (!CanWrapTarget(Prepared.Mesh.Get(), Prepared.Bone))
	{
		return false;
	}

	// The throw strength contract, checked before ResetStateForNewThrow changes any state, so an invalid
	// speed is refused without deploying or reinitializing anything.
	float PreparedThrowSpeed = 0.0f;
	if (!TryResolveValidThrowSpeed(Prepared.ThrowContext, PreparedThrowSpeed))
	{
		return false;
	}

	ResetStateForNewThrow();
	// Insurance that the tip attachment exists. The normal path already secured it during BeginPlay;
	// this covers toggling bUseTipMesh at runtime and is a no-op when one is already present.
	EnsureTipMesh();

	FRopePreparedThrowPreview ResolvedPrepared = Prepared;
	// Resolve the owner-local path stored at the moment of input against the owner transform as it is
	// when the throw executes.
	ResolvedPrepared.RenderPreview = Prepared.ResolveRenderPreviewWorld();
	ResolvedPrepared.ThrowContext.Origin = Prepared.ResolveGuideOriginWorld();
	ClearPreparedGuideFrameLocal(ResolvedPrepared);
	ApplyPierceSocketTargetsToPrepared(ResolvedPrepared);
	AimTargeting.SetWrapTargetLock(ResolvedPrepared.ThrowContext);

	const FString PhaseReason = FString::Printf(TEXT("prepared points=%d, bone=%s"),
		ResolvedPrepared.RenderPreview.Points.Num(), *ResolvedPrepared.Bone.ToString());
	if (!BeginGuidedThrowState(MoveTemp(ResolvedPrepared), /*bFreeThrow*/ false))
	{
		return false;
	}

	// Deploy on leaving Loaded: restore the rope's visibility and its full length. This is the default
	// implementation and can be overridden.
	OnDeployFromLoaded();

	SetPhase(ERopePhase::GuidedThrow, *PhaseReason);
	return true;
}

const TArray<IRopeCollider*>& URopeComponent::GetAimQueryColliders() const
{
	return SimFrame.AimRayColliderQueryBounds.IsValid ? SimFrame.AimFrameColliders : SimFrame.FrameColliders;
}

FRopeAimTargeting::FQueryContext URopeComponent::MakeAimQueryContext() const
{
	// The fallback dimensions: the ray length is the larger of the current and initial rope lengths, and
	// the query radius is the larger of the tube and contact radii.
	FRopeAimTargeting::FQueryContext Ctx;
	Ctx.Colliders = &GetAimQueryColliders();
	Ctx.FallbackRayLength = FMath::Max(Sim.RopeLength, RopeLength);
	Ctx.FallbackQueryRadius = FMath::Max(Radius, GetEffectiveContactQueryRadius());
	// The world blocking probe, injected because FRopeAimTargeting has no world of its own.
	Ctx.TraceWorldBlocker = [this](const FVector& Start, const FVector& End,
		FVector& OutBlockPoint, float& OutDistance)
	{
		return TraceWorldAimBlocker(Start, End, OutBlockPoint, OutDistance);
	};
	return Ctx;
}

bool URopeComponent::TraceWorldAimBlocker(const FVector& Start, const FVector& End,
	FVector& OutBlockPoint, float& OutDistance) const
{
	OutBlockPoint = End;
	OutDistance = 0.0f;
	// No world means the component default object or a unit test, where nothing can block.
	const UWorld* World = GetWorld();
	if (!World || (End - Start).IsNearlyZero())
	{
		return false;
	}

	// The thrower is excluded for the same reason the collider gather excludes its own owner: the rope's
	// own body, tip mesh and tether proxy sit right on the ray origin and would block every throw.
	FCollisionQueryParams Params(SCENE_QUERY_STAT(RopeAimBlocker), /*bInTraceComplex*/ false);
	Params.AddIgnoredActor(GetOwner());

	// Visibility, with no per-rope channel setting: it is already the channel that answers "does this
	// geometry block sight", and that answer belongs to the geometry. Level geometry the rope should be
	// able to aim through says so through its own Visibility response, which keeps one answer in one place
	// instead of splitting it between the level and every rope.
	FHitResult Hit;
	if (!World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params))
	{
		return false;
	}

	OutBlockPoint = Hit.ImpactPoint;
	OutDistance = static_cast<float>(FVector::DotProduct(Hit.ImpactPoint - Start, (End - Start).GetSafeNormal()));
	return true;
}

float URopeComponent::GetAimRayEffectiveQueryRadius(float RequestedRadius) const
{
	return FRopeAimTargeting::ResolveEffectiveQueryRadius(MakeAimQueryContext(), RequestedRadius);
}

void URopeComponent::SetAimRayColliderQueryBounds(const FVector& Origin, const FVector& AimDir,
	float RayLength, float QueryRadius)
{
	// Invalid input returns FBox(ForceInit), which is equivalent to clearing it and extends the gather
	// by nothing.
	SimFrame.AimRayColliderQueryBounds =
		FRopeAimTargeting::MakeAimRayQueryBounds(MakeAimQueryContext(), Origin, AimDir, RayLength, QueryRadius);
}

void URopeComponent::ClearAimRayColliderQueryBounds()
{
	SimFrame.AimRayColliderQueryBounds = FBox(ForceInit);
	// The aim list is filled only from these bounds, so it is cleared on the same frame the bounds
	// disappear. That keeps a provider pointer from the previous frame from surviving until the next
	// time aiming happens; their lifetime is that frame alone.
	SimFrame.AimFrameColliders.Reset();
	AimTargeting.ResetQuery();
}

void URopeComponent::QueueAimRayQuery(const FRopeAimRayThrowRequest& Request)
{
	if (!Request.IsValid())
	{
		ClearAimRayColliderQueryBounds();
		return;
	}

	AimTargeting.QueuePendingQuery(Request);
	// If a real throw was queued earlier in this same PrePhysics window, the HUD request must not
	// overwrite its input bounds. The real request has to be covered exactly by this frame's gather,
	// and the HUD result would not be consumed after the phase transition anyway.
	if (!AimTargeting.HasPendingThrow() &&
		(!PendingGuaranteedAimThrow.bQueued || PendingGuaranteedAimThrow.bResolved))
	{
		SetAimRayColliderQueryBounds(
			Request.RayOrigin, Request.RayDirection, Request.RayLength, Request.QueryRadius);
	}
}

bool URopeComponent::GetLatestAimRayQueryResult(FRopeAimRayQueryResult& OutResult) const
{
	return AimTargeting.GetLatestQueryResult(OutResult);
}

bool URopeComponent::RefreshAimRayQueryColliders(const FRopeAimRayThrowRequest& Request)
{
	// Kept for source compatibility only. Despite the name it does not re-run the providers
	// immediately; consume the result through GetLatestAimRayQueryResult after the normal gather.
	QueueAimRayQuery(Request);
	return false;
}

bool URopeComponent::ResolveAimRayThrowContext(const FRopeAimRayThrowRequest& Request,
	FRopeThrowContext& OutContext, FRopeAimRayHitResult* OutHit, FRopeAimRayHitResult* OutBlockedHit) const
{
	return FRopeAimTargeting::ResolveAimRayThrowContext(MakeAimQueryContext(), Request,
		[this](const USceneComponent* Mesh, FName Bone) { return CanWrapTarget(Mesh, Bone); },
		OutContext, OutHit, OutBlockedHit);
}

void URopeComponent::QueueAimRayThrow(const FRopeAimRayThrowRequest& Request)
{
	// An invalid request falls back to an immediate throw. This is before StartFreshThrow, so the
	// component handles it directly rather than the logic classes.
	if (!Request.IsValid())
	{
		StartFreshThrow(Request.BaseContext);
		Request.OnResolved.ExecuteIfBound();
		return;
	}

	AimTargeting.QueuePendingThrow(Request);
	SetAimRayColliderQueryBounds(
		Request.RayOrigin, Request.RayDirection, Request.RayLength, Request.QueryRadius);
}

bool URopeComponent::QueueGuaranteedAimThrow(const FRopeAimRayThrowRequest& Request, bool bExecuteWhenReady)
{
	if (ResolveMode != ERopeWrapResolveMode::GuaranteedWrap || !CanThrowNow() || PendingGuaranteedAimThrow.bQueued)
	{
		return false;
	}

	PendingGuaranteedAimThrow.Reset();
	PendingGuaranteedAimThrow.Request = Request;
	PendingGuaranteedAimThrow.bQueued = true;
	PendingGuaranteedAimThrow.bExecuteWhenReady = bExecuteWhenReady;
	SetAimRayColliderQueryBounds(
		Request.RayOrigin, Request.RayDirection, Request.RayLength, Request.QueryRadius);
	return true;
}

bool URopeComponent::RequestExecuteQueuedGuaranteedAimThrow()
{
	if (!PendingGuaranteedAimThrow.bQueued)
	{
		return false;
	}

	PendingGuaranteedAimThrow.bExecuteWhenReady = true;
	if (PendingGuaranteedAimThrow.bResolved)
	{
		ExecutePendingGuaranteedAimThrow();
	}
	return true;
}

void URopeComponent::CancelQueuedGuaranteedAimThrow()
{
	PendingGuaranteedAimThrow.Reset();
}

bool URopeComponent::BuildPreparedWrappingPreview(const FRopeThrowContext& ThrowContext,
	FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason) const
{
	// A prepared preview only means anything before the throw, that is in Free, Releasing and Loaded,
	// where Loaded is the ready-to-throw state of GuaranteedWrap and both the aiming preview and the
	// throw itself use this build. Phases from Flight onwards are already in real contact or wrapped and
	// have no preview: FullSimulation and AssistedJudged have none at all, and for GuaranteedWrap this
	// prepared path is the only preview there is.
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Releasing && Phase != ERopePhase::Loaded)
	{
		OutPrepared.Reset();
		RopeMath::SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("prepared preview rejected: phase=%s"), PhaseName(Phase)));
		return false;
	}
	return BuildPreparedWrappingPreviewFromResolvedContext(
		ResolveThrowContext(ThrowContext), OutPrepared, OutFailureReason);
}

bool URopeComponent::BuildPreparedWrappingPreviewFromResolvedContext(
	const FRopeThrowContext& ResolvedThrowContext,
	FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason) const
{
	OutPrepared.Reset();

	FRopeThrowPreviewBuilder::FInput Input;
	Input.Sim = &Sim;
	// The preview path build has to see the same list as the aim ray hit test: the aiming snapshot while
	// aiming, which includes distant targets, and the physics snapshot otherwise, as on the direct
	// Blueprint throw path. See the comment on GetAimQueryColliders.
	Input.Colliders = &GetAimQueryColliders();
	// Inject the wrap target gate, the same pattern ResolveAimRayThrowContext uses on the aim path, so
	// the preview filters targets on the same basis as aiming. Without it a forbidden target would
	// appear in the preview only to be refused at the throw entry point.
	Input.CanWrapTarget = [this](const USceneComponent* Mesh, FName Bone) { return CanWrapTarget(Mesh, Bone); };
	Input.ThrowContext = ResolvedThrowContext;
	Input.WrapConfig = WrapConfig;
	Input.WrapConfig.ContactQueryRadius = GetEffectiveContactQueryRadius(); // Carry the resolved automatic value.
	// Propagate the resolve mode to the preview builder: GuaranteedWrap takes the single-anchor embed
	// path instead of building a wrapping helix.
	Input.ResolveMode = ResolveMode;
	Input.RopeRadius = Radius;
	Input.RopeNumSides = NumSides;
	Input.OwnerName = GetName();
	const bool bBuilt = FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, OutPrepared, OutFailureReason);
	if (bBuilt)
	{
		ApplyPierceSocketTargetsToPrepared(OutPrepared);
	}
	return bBuilt;
}

void URopeComponent::DispatchCaptured(FName Bone)
{
	// The notification order follows the engine's Notify convention: the native hook first, then the
	// Blueprint delegate.
	NotifyCaptured(Bone);
	OnRopeCaptured.Broadcast(Bone);
}

#pragma endregion Throw_Public_API

#pragma region Throw_Phase_State

void URopeComponent::ResolvePendingAimQuery()
{
	FRopeAimRayThrowRequest Request;
	if (!AimTargeting.TakePendingQuery(Request))
	{
		return;
	}

	FRopeAimRayQueryResult Result;
	Result.Request = Request;
	Result.ResolvedContext = Request.BaseContext;
	Result.bHitTarget = ResolveAimRayThrowContext(
		Request, Result.ResolvedContext, &Result.Hit, &Result.BlockedHit);
	AimTargeting.StoreLatestQueryResult(Result);
}

void URopeComponent::ResolvePendingGuaranteedAimThrow()
{
	if (!PendingGuaranteedAimThrow.bQueued || PendingGuaranteedAimThrow.bResolved)
	{
		return;
	}

	FRopeThrowContext& ResolvedContext = PendingGuaranteedAimThrow.ResolvedContext;
	ResolvedContext = PendingGuaranteedAimThrow.Request.BaseContext;
	ResolveAimRayThrowContext(PendingGuaranteedAimThrow.Request, ResolvedContext);

	FString FailureReason;
	BuildPreparedWrappingPreviewFromResolvedContext(
		ResolvedContext, PendingGuaranteedAimThrow.Prepared, &FailureReason);
	PendingGuaranteedAimThrow.bResolved = true;
	PendingGuaranteedAimThrow.Request.OnPrepared.ExecuteIfBound(PendingGuaranteedAimThrow.Prepared);

	if (PendingGuaranteedAimThrow.bExecuteWhenReady)
	{
		ExecutePendingGuaranteedAimThrow();
	}
}

bool URopeComponent::ExecutePendingGuaranteedAimThrow()
{
	if (!PendingGuaranteedAimThrow.bQueued || !PendingGuaranteedAimThrow.bResolved)
	{
		return false;
	}

	FPendingGuaranteedAimThrow Pending = MoveTemp(PendingGuaranteedAimThrow);
	PendingGuaranteedAimThrow.Reset();

	bool bExecuted = false;
	if (CanThrowNow())
	{
		if (Pending.Prepared.IsValid())
		{
			bExecuted = ThrowWithPreparedPreview(Pending.Prepared);
		}
		else
		{
			// A miss on the input frame is not re-queried at notify time. The throw uses the direction
			// established then, as a free arc.
			EnsureRopeInitialized();
			const FRopeThrowContext ResolvedThrow = ResolveThrowContext(Pending.ResolvedContext);
			// The throw strength contract is checked before any state changes in OnDeployFromLoaded.
			float FreeThrowSpeed = 0.0f;
			if (TryResolveValidThrowSpeed(ResolvedThrow, FreeThrowSpeed))
			{
				const FVector FreeEndpoint = ResolveFreeThrowEndpoint(ResolvedThrow);
				OnDeployFromLoaded();
				bExecuted = StartFreeGuidedThrow(ResolvedThrow, FreeEndpoint);
			}
		}
	}

	if (bExecuted)
	{
		Pending.Request.OnResolved.ExecuteIfBound();
	}
	else
	{
		Pending.Request.OnRejected.ExecuteIfBound();
	}
	return bExecuted;
}

void URopeComponent::ResolvePendingAimThrow()
{
	// StartFreshThrow resets the transient state, so take the request by value first and clear the
	// pending slot before calling it.
	FRopeAimRayThrowRequest Request;
	if (!AimTargeting.TakePendingThrow(Request))
	{
		return;
	}

	FRopeThrowContext ResolvedContext;
	ResolveAimRayThrowContext(Request, ResolvedContext);
	StartFreshThrow(ResolvedContext);
	Request.OnResolved.ExecuteIfBound();
}

void URopeComponent::FilterFrameCollidersForAimWrapTarget()
{
	const bool bPromoteLockedTarget = AimTargeting.IsLockActive(Phase);
	if (bPromoteLockedTarget)
	{
		// The collider gather runs before the pending aim throw is resolved. A distant target exists only
		// in AimFrameColliders on this first Flight frame, so promote just the skeletal colliders that
		// match the lock policy into the physics and detection lists, without letting unrelated or
		// world-static colliders near the aim ray leak through.
		FBox RefreshedTargetBounds(ForceInit);
		for (IRopeCollider* Collider : SimFrame.AimFrameColliders)
		{
			if (!Collider || Collider->IsWorldStatic())
			{
				continue;
			}

			FName ColliderBone = NAME_None;
			const USceneComponent* ColliderMesh = nullptr;
			Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
			// IsWrapTarget in FullSimulation deliberately permits everything. Promotion must not use that
			// broad predicate and instead applies the actual locked mesh identity, which keeps unrelated
			// colliders near the aim ray out of the physics list.
			if (ColliderMesh && ColliderMesh == AimTargeting.GetLockedTargetMesh())
			{
				SimFrame.FrameColliders.AddUnique(Collider);
				RefreshedTargetBounds += Collider->GetWorldBounds();
			}
		}
		// A transient provider budget or mapping miss keeps the previous bounds, which leaves a chance to
		// gather again next frame. A successful refresh replaces them with the latest collider union so a
		// moving target is followed.
		if (RefreshedTargetBounds.IsValid)
		{
			SimFrame.LockedTargetColliderQueryBounds = RefreshedTargetBounds;
		}
	}
	else
	{
		SimFrame.LockedTargetColliderQueryBounds = FBox(ForceInit);
	}
	AimTargeting.FilterCollidersToTarget(Phase, ResolveMode, SimFrame.FrameColliders);
}

#pragma endregion Throw_Phase_State

#pragma region Throw

// ===== Throw ================================================================

FRopeThrowContext URopeComponent::ResolveThrowContext(const FRopeThrowContext& ThrowContext) const
{
	FRopeThrowContext Resolved = ThrowContext;

	// The final frame is guaranteed orthonormal and right-handed. Whatever a producer supplied, whether
	// MakeDefault, the wielder or a direct Blueprint call, everything downstream, meaning the whip
	// guide's swing basis and the preview builder, trusts only this result. Normalizing the three axes
	// independently would let a non-orthogonal custom basis, or one whose axes came from different
	// fallbacks, pass through inconsistently.
	// The convention: Forward is the reference axis and is only normalized, preserving its direction. Up
	// is orthogonalized against Forward by Gram-Schmidt; if the supplied Up is parallel to Forward it
	// falls back to world up and then to the component up, and to an arbitrary perpendicular if all are
	// parallel.
	// Right is always re-derived as Up cross Forward and the supplied Right is ignored: swinging to the
	// other side is expressed through the SwingPlane options, not by flipping Right.
	Resolved.FrameForward = FRopeWhipGuide::SafeNormalOr(Resolved.FrameForward, GetForwardVector());
	const FVector Forward = Resolved.FrameForward;

	FVector Up = FVector::ZeroVector;
	const FVector UpCandidates[] = { ThrowContext.FrameUp, FVector::UpVector, GetUpVector() };
	for (const FVector& Candidate : UpCandidates)
	{
		FVector Projected = Candidate - FVector::DotProduct(Candidate, Forward) * Forward;
		if (Projected.Normalize(KINDA_SMALL_NUMBER))
		{
			Up = Projected;
			break;
		}
	}
	if (Up.IsNearlyZero())
	{
		// A vertical throw with a vertical component axis, which falls back to an arbitrary perpendicular.
		Up = RopeMath::AnyTangentFromNormal(Forward);
	}
	Resolved.FrameUp = Up;
	Resolved.FrameRight = FVector::CrossProduct(Up, Forward);

	if (Resolved.ThrowSpeed <= 0.0f)
	{
		Resolved.ThrowSpeed = ThrowParams.ThrowSpeed;
	}
	if (Resolved.Origin.IsNearlyZero())
	{
		Resolved.Origin = GetComponentLocation();
	}

	return Resolved;
}

bool URopeComponent::TryResolveValidThrowSpeed(const FRopeThrowContext& Context, float& OutThrowSpeed) const
{
	// ThrowSpeed is contractually positive, in cm/s. Zero or negative would send the whip and guided
	// durations back through their defensive fallbacks and produce the contradiction of a slow throw
	// that is suddenly fast. A context value of 0 resolves to the component default from
	// ThrowParams.ThrowSpeed.
	OutThrowSpeed = Context.ThrowSpeed > 0.0f ? Context.ThrowSpeed : ThrowParams.ThrowSpeed;
	constexpr float MinValidThrowSpeed = 1.0f;
	if (OutThrowSpeed < MinValidThrowSpeed)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] Throw rejected: ThrowSpeed must be positive (got %.2f cm/s)."),
			*GetName(), OutThrowSpeed);
		return false;
	}
	return true;
}

FVector URopeComponent::ComputeThrowInheritedVelocity(const FRopeThrowContext& ThrowContext) const
{
	// Character movement is carried at the MotionInheritance multiplier, while the hand socket's
	// animation swing, meaning the hand velocity relative to the character, is carried separately at
	// one. HandAnimationVelocity is already relative to the owner, having been measured by the wielder
	// from a component-local position delta, so movement is not counted twice and no reversed velocity
	// appears even at a MotionInheritance of 0. It also does not depend on whether a physics body
	// exists, unlike reading a physics linear velocity.
	return ThrowContext.OwnerVelocity * ThrowParams.MotionInheritance +
		ThrowContext.HandAnimationVelocity;
}

void URopeComponent::StartFreshThrow(const FRopeThrowContext& ThrowContext)
{
	const FRopeThrowContext ResolvedThrow = ResolveThrowContext(ThrowContext);

	// The throw strength gate, applied before any state changes: an invalid throw speed is refused
	// without deploying or reinitializing, which backs up the editor's clamp.
	float ResolvedThrowSpeed = 0.0f;
	if (!TryResolveValidThrowSpeed(ResolvedThrow, ResolvedThrowSpeed))
	{
		return;
	}

	// Starting a throw is four steps in a fixed order: clear the previous state, reset the chain and
	// reseed the GPU buffers, start the whip swing, and inject the Verlet velocity. The last step uses
	// the aim direction the third established, through WhipGuide.GetAimDir, so the order is a contract.
	ResetStateForNewThrow();
	// Store the mesh and bone the ray established as the primary. AssistedJudged also permits other
	// bones on the same mesh as candidates and for the path, while only GuaranteedWrap restricts
	// Flight, Contacting and Wrapping to the exact bone.
	AimTargeting.SetWrapTargetLock(ResolvedThrow);
	ResetChainForThrow(ResolvedThrow.Origin);
	BeginWhipSwingFromThrow(ResolvedThrow);
	InjectThrowVelocityIntoVerlet(ResolvedThrow);
	// Insurance that the tip attachment exists; the normal path already secured it during BeginPlay, and
	// this is a no-op when one is present.
	EnsureTipMesh();

	SetPhase(ERopePhase::Flight, *FString::Printf(TEXT("fresh throw impulse, aim=%s, speed=%.1f"),
		*WhipGuide.GetAimDir().ToCompactString(), ResolvedThrow.ThrowSpeed));
}

void URopeComponent::ResetStateForNewThrow()
{
	// A new throw releases any wrap being held, as a manual release, discards the transient state of the
	// phase in progress, and throws immediately with no cooldown.
	// A committed wrap has to report its release, matching FinishWrapRelease. Otherwise OnAnyRopeReleased
	// never fires and a cross-actor target, such as a ragdoll, stays stuck permanently even though the
	// rope has let go.
	// The mesh and bone are captured before Release clears the state, and the notification is sent after
	// the state has been cleaned up, following the DispatchReleased re-entrancy contract.
	const USceneComponent* WrappedMesh = WrapController.State.Mesh.Get();
	const FName WrappedBone = WrapController.State.BoneName;
	const bool bWasWrapped = WrapController.IsActive();
	if (bWasWrapped)
	{
		WrapController.Release(ERopeReleaseReason::Manual);
	}
	ResetKinematicVirtualBridges();
	ResetTransientPhaseState();
	// The distant bounds of the previous throw must be cut before the new throw's target identity is
	// established. A Contacting to Flight return within the same throw also uses
	// ResetTransientPhaseState, so clearing the cache belongs on this throw boundary alone.
	SimFrame.LockedTargetColliderQueryBounds = FBox(ForceInit);
	ReleaseCooldown = 0.0f;
	if (bWasWrapped)
	{
		DispatchReleased(WrappedMesh, WrappedBone, ERopeReleaseReason::Manual, /*bWasWrapped*/ true);
	}
}

bool URopeComponent::BeginGuidedThrowState(FRopePreparedThrowPreview&& Prepared, bool bFreeThrow)
{
	if (Sim.Num() < 2)
	{
		return false;
	}

	// The final defence for the throw strength contract. The normal path already validated it before
	// changing state, through TryResolveValidThrowSpeed; this guards direct callers such as tests and
	// future paths, and reaching here is still before any state change.
	float GuidedThrowSpeed = 0.0f;
	if (!TryResolveValidThrowSpeed(Prepared.ThrowContext, GuidedThrowSpeed))
	{
		return false;
	}

	const FVector Origin = Prepared.ThrowContext.Origin;
	GuidedThrowState.bActive = true;
	GuidedThrowState.bFreeThrow = bFreeThrow;
	GuidedThrowState.Prepared = MoveTemp(Prepared);
	GuidedThrowState.StartPositions = Sim.Positions;
	GuidedThrowState.Elapsed = 0.0f;
	// The GuidedThrow duration is also derived from the throw speed: the hand-to-target distance divided
	// by ThrowSpeed, clamped to a stable range of 0.08 to 2.0 seconds. Further targets take longer and
	// faster throws arrive sooner. The coordinates use the same preview world resolution as
	// UpdateGuidedThrow.
	{
		const FRopePreparedThrowPreview& Prep = GuidedThrowState.Prepared;
		const float GuideThrowSpeed = Prep.ThrowContext.ThrowSpeed > KINDA_SMALL_NUMBER
			? Prep.ThrowContext.ThrowSpeed : ThrowParams.ThrowSpeed;
		const FVector HandWorld = Prep.ResolveGuideOriginWorld();
		const FVector TargetWorld = Prep.ResolveGuidePointWorld(Sim.Num() - 1);
		const float TravelDist = static_cast<float>((TargetWorld - HandWorld).Size());
		GuidedThrowState.Duration = FMath::Clamp(
			TravelDist / FMath::Max(GuideThrowSpeed, KINDA_SMALL_NUMBER), 0.08f, 2.0f);
	}

	Sim.bStartPinned = true;
	Sim.StartPinPrev = Origin;
	Sim.StartPinTarget = Origin;
	if (Sim.Positions.IsValidIndex(0))
	{
		Sim.Positions[0] = Origin;
		Sim.PrevPositions[0] = Origin;
	}
	return true;
}

void URopeComponent::ResetChainForThrow(const FVector& HandOrigin)
{
	// This is where the whole chain is repositioned, so the GPU resident buffer's reseed generation is
	// raised here too: the reset and the reseed are one action, and separating them invites doing only
	// half of it.
	++SimFrame.SimGeneration;
	bWrappedMassMaskDirty = true;

	if (Sim.Num() < 2)
	{
		return;
	}

	// Pin the hand, node 0, at the throw origin. Every node has its previous position set equal to its
	// current one, giving zero velocity; the throw velocity is injected separately in the last step.
	Sim.bStartPinned = true;
	Sim.StartPinPrev = HandOrigin;
	Sim.StartPinTarget = HandOrigin;
	Sim.Positions[0] = HandOrigin;
	Sim.PrevPositions[0] = HandOrigin;
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		Sim.InvMass[i] = (i == 0) ? 0.0f : 1.0f;
		Sim.SetStill(i);
	}
}

void URopeComponent::BeginWhipSwingFromThrow(const FRopeThrowContext& ResolvedThrow)
{
	// Everything WhipGuide.Begin needs, meaning the aim and guide axes and the inherited velocity, is
	// derived from the resolved throw. Confining that assembly here leaves the caller, StartFreshThrow,
	// reading as a list of step names.
	const FRopeWhipGuide::FSwingBasis SwingBasis = FRopeWhipGuide::ResolveSwingBasis(
		ResolvedThrow, ResolvedThrow.SwingPlane, ResolvedThrow.CustomSwingPlaneNormal);
	const FVector InheritedVelocity = ComputeThrowInheritedVelocity(ResolvedThrow);
	FlightGuidePlaneNormal = SwingBasis.GuideRight.GetSafeNormal();
	bHasFlightGuidePlaneNormal = !FlightGuidePlaneNormal.IsNearlyZero();

	// Build the whip swing guide frame and activate it; degenerate cases fall back to the component axes.
	WhipGuide.Begin(SwingBasis.AimDir, ResolvedThrow.Origin,
		ResolvedThrow.FrameForward, SwingBasis.GuideUp, SwingBasis.GuideRight,
		ResolvedThrow.ThrowSpeed, InheritedVelocity,
		ResolvedThrow.bHasAimGuideHit, ResolvedThrow.AimGuideHitWorldPos,
		ResolvedThrow.AimGuideSteerStartAlpha, ResolvedThrow.AimGuideLockAlpha);

	if (Sim.Num() >= 2)
	{
		// Snap the guided nodes onto the guide curve at time zero, with zero velocity.
		WhipGuide.SnapToInitialPose(Sim, MakeWhipGuideConfig());
	}
}

void URopeComponent::InjectThrowVelocityIntoVerlet(const FRopeThrowContext& ResolvedThrow)
{
	const int32 LastNode = Sim.Num() - 1;
	if (LastNode < 1)
	{
		return;
	}

	// Verlet integration expresses velocity implicitly as the current position minus the previous one,
	// divided by dt. Pushing the previous position backwards along the desired velocity by v * dt leaves
	// the position untouched while carrying that velocity from the next step onwards, which is a pure
	// velocity injection.
	// Distribution: the weight rises from the hand towards the tip, through a smoothstep plus a tail
	// weighting, and TipVelocityBoost drives the tip harder so it runs ahead like a whip. The inherited
	// velocity, from the owner and the socket, is uniform across every node. ReferenceDt is a fixed
	// conversion basis, independent of the first step's actual dt, so throw strength does not vary with
	// the framerate.
	const FVector ThrowDir = WhipGuide.GetAimDir();
	const float ReferenceDt = 1.0f / 60.0f;
	const float BaseImpulse = ResolvedThrow.ThrowSpeed * ReferenceDt;
	const float TipBoost = FMath::Clamp(ThrowParams.TipVelocityBoost, 0.25f, 3.0f);
	const FVector InheritedVelocityImpulse = ComputeThrowInheritedVelocity(ResolvedThrow) * ReferenceDt;
	const int32 FirstTailNode = FMath::Clamp(FMath::FloorToInt(static_cast<float>(LastNode) * WhipConfig.GuidedLength), 1, LastNode);
	for (int32 i = 1; i <= LastNode; ++i)
	{
		const float AlongRope = static_cast<float>(i) / static_cast<float>(LastNode);
		const float TailWeight = TailWeightByIndex(i, FirstTailNode, LastNode);
		const float Weight = FMath::Lerp(RopeMath::SmoothStep(AlongRope), 1.0f, TailWeight * 0.5f);
		const float Impulse = BaseImpulse * Weight * FMath::Lerp(1.0f, TipBoost, TailWeight);
		Sim.PrevPositions[i] -= ThrowDir * Impulse + InheritedVelocityImpulse;
	}
}

void URopeComponent::AbortGuidedThrow(ERopeReleaseReason Reason, const TCHAR* ReasonLog)
{
	// A throw into open space never had a target and therefore never opened an engagement, so firing a
	// release would be an unpaired phantom signal; see the DispatchReleased contract. Only an aimed
	// throw reports.
	const bool bNotify = !GuidedThrowState.bFreeThrow;
	// ResetTransientPhaseState clears GuidedThrowState, so copy it first.
	const FName Bone = GuidedThrowState.Prepared.Bone;

	SetPhase(ERopePhase::Releasing, ReasonLog);
	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;

	// Report only after the state is fully cleaned up, because a handler may call back into ReleaseWrap
	// and the like, following the OnRopeReleased contract.
	// This is before the commit, so no central signal is sent.
	if (bNotify)
	{
		DispatchReleased(nullptr, Bone, Reason, /*bWasWrapped*/ false);
	}
}

void URopeComponent::UpdateGuidedThrow(float DeltaTime)
{
	// A throw into open space follows the straight render preview with no target mesh or bone, so the
	// path alone is checked rather than IsValid, which requires a target.
	const bool bFree = GuidedThrowState.bFreeThrow;
	const bool bValidPath = bFree ? GuidedThrowState.Prepared.RenderPreview.IsValid()
		: GuidedThrowState.Prepared.IsValid();
	if (!GuidedThrowState.bActive || !bValidPath || Sim.Num() < 2)
	{
	// The target mesh was lost or the path became invalid, which is an involuntary failure and reported
	// as Broken.
		AbortGuidedThrow(ERopeReleaseReason::Broken, TEXT("guided throw invalid"));
		return;
	}

	// The interrupt hook during a guaranteed throw, defaulting to false so the guarantee holds. A
	// subclass returns true only when a game rule has to break it, such as the target dying or
	// teleporting away.
	// A throw into open space is not polled: there is no guarantee to break, and its prepared data is a
	// stub with a null mesh, so the hook cannot judge the cases it is documented for. Use ReleaseWrap()
	// when such a throw needs cancelling.
	if (!bFree && ShouldAbortGuaranteedThrow(GuidedThrowState.Prepared))
	{
		// A game rule broke it deliberately, which is distinguished from an internal failure so consumers
		// can react differently.
		AbortGuidedThrow(ERopeReleaseReason::ThrowAborted, TEXT("guided throw aborted by game rule"));
		return;
	}

	GuidedThrowState.Elapsed += DeltaTime;
	const float Alpha = FMath::Clamp(GuidedThrowState.Elapsed / FMath::Max(GuidedThrowState.Duration, 0.01f), 0.0f, 1.0f);
	const float EasedAlpha = Alpha * Alpha * (3.0f - 2.0f * Alpha);
	const FRopePreparedThrowPreview& Prepared = GuidedThrowState.Prepared;

	// An upward parabolic arc, shared by aimed and open-space throws: the tip bulges above the straight
	// line from the hand to the target and comes back down to land. The apex is at an alpha of 0.5 and
	// the offset is zero at 0 and 1.
	// The interpolation along the rope is linear, so at every instant the rope itself is straight and
	// only the tip traces a parabola. The offset being zero at an alpha of 1 preserves the landing point
	// exactly.
	const FVector ArcOriginW = Prepared.ResolveGuideOriginWorld();
	const FVector ArcTipW = Prepared.ResolveGuidePointWorld(Sim.Num() - 1);
	// Blueprint writes at runtime bypass the clamp in the property metadata, so fold it to the same range
	// where it is consumed.
	const float ArcHeightRatio = FMath::Clamp(ThrowParams.GuidedThrowArcHeightRatio, 0.0f, 0.5f);
	const float ArcHeight = ArcHeightRatio * static_cast<float>((ArcTipW - ArcOriginW).Size());
	const float ArcT = 4.0f * Alpha * (1.0f - Alpha);
	const int32 LastNode = Sim.Num() - 1;

	// On an aimed, that is non-open-space, throw the tip target is re-aimed at the target bone's current
	// position every frame, so the tip converges on the body point that was aimed at even while the
	// target moves, with no pop on landing. The preview guide points are thrower-local and cannot follow
	// the target's movement, so only the endpoint is replaced with the live target.
	const bool bTrackAimTarget = !bFree && Prepared.ThrowContext.bHasAimGuideLocalHit;
	const FVector AimTargetWorld = bTrackAimTarget
		? ResolveAimGuideHitWorld(Prepared.ThrowContext)
		: FVector::ZeroVector;

	// A throw into open space re-draws its guide line every frame from the live hand toward the
	// endpoint fixed at the throw. The preview's stored line is anchored at the throw-time origin, and
	// nodes interpolating toward it while the wielder moves head for a spot the hand has already
	// passed — the rope reads as being yanked backwards. The endpoint itself never moves: only the
	// hand end of the line follows.
	const FVector FreeHandLive = (bFree && Sim.bStartPinned)
		? Sim.StartPinTarget
		: Prepared.ThrowContext.Origin;

	// The open-space endpoint is not a fixed point but the rope pulled taut along the aim ray: the
	// furthest point on the input-frame ray still within rope reach of the live hand, re-resolved
	// every frame and clamped by the world blocker. A stationary wielder keeps the throw-time
	// endpoint exactly; running forward extends the flight along the ray, so the thrown rope keeps
	// its momentum instead of dying at the aimed spot and dropping; and running away shortens it —
	// the rope simply cannot reach farther than its length from the hand. The line therefore lands
	// taut, and slack only remains when the ray is blocked, which is exactly when the droop below
	// takes over.
	FVector FreeEndpointLive = ArcTipW;
	if (bFree)
	{
		const FVector RayOrigin = Prepared.ThrowContext.Origin;
		const FVector RayDir = RopeMath::SafeNormalOr(ArcTipW - RayOrigin, Prepared.ThrowContext.FrameForward);
		float TautDistance = FRopeAimTargeting::ResolveRayLengthForReach(
			RayOrigin, RayDir, FreeHandLive, Sim.RopeLength);
		if (TautDistance <= KINDA_SMALL_NUMBER)
		{
			// The hand strayed farther from the ray than the rope is long (or the reach degenerated):
			// no ray point is reachable, so keep the throw-time endpoint and let the landing solve
			// resolve the overstretch.
			TautDistance = static_cast<float>((ArcTipW - RayOrigin).Size());
		}
		FreeEndpointLive = FRopeAimTargeting::ResolveOpenSpaceThrowEndpoint(MakeAimQueryContext(),
			RayOrigin, RayDir, TautDistance, FMath::Max(Radius, 1.0f));
	}

	// Whatever line length the taut endpoint could not restore — it is shorter than the rope only
	// when the ray was world-blocked — is laid in as a parabolic droop, deepest mid-line and zero at
	// the hand and the endpoint. A compressed straight line cannot hold its slack: the equality
	// distance constraints fold the free end back toward the pinned hand as it falls, which reads as
	// the rope being dragged to the wielder instead of dropping at the guide spot. With the droop the
	// landed shape is already length-feasible and the tip simply falls from the endpoint.
	float FreeSagDepth = 0.0f;
	if (bFree)
	{
		FreeSagDepth = RopeMath::ComputeFreeGuideSagDepth(
			static_cast<float>((FreeEndpointLive - FreeHandLive).Size()), Sim.RopeLength);
		// Ramped by the eased time once more (the node interpolation applies it again below): the
		// flight shows only a modest slack and the full drape lays down as the rope arrives, at about
		// free-fall speed. Without this a running wielder's shrinking line drops a deep droop in
		// mid-flight, and the floor clamp then holds the rope's belly against the ground for the rest
		// of the flight — reading as the rope suddenly sticking flat to the floor. At landing the
		// eased time is one, so the length-feasible landing shape is unchanged.
		FreeSagDepth *= EasedAlpha;
		if (FreeSagDepth > KINDA_SMALL_NUMBER)
		{
			// The guided flight solves no collisions, so the droop is clamped against the world: one
			// downward probe from the line midpoint — the deepest point of the parabola — keeps the
			// mid-rope from dipping through the floor, with the rope's radius of clearance.
			const FVector LineMid = (FreeHandLive + FreeEndpointLive) * 0.5f;
			const float SagClearance = FMath::Max(Radius, 1.0f);
			FVector SagBlockPoint = FVector::ZeroVector;
			float SagBlockDistance = 0.0f;
			if (TraceWorldAimBlocker(LineMid,
				LineMid - FVector::UpVector * (FreeSagDepth + SagClearance),
				SagBlockPoint, SagBlockDistance))
			{
				FreeSagDepth = FMath::Max(SagBlockDistance - SagClearance, 0.0f);
			}
		}
	}

	SimFrame.OverrideFrame.EnsureSize(Sim.Num());
	for (int32 NodeIndex = 0; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		// The hand anchor, node 0, is never interpolated and must stay attached to the hand's current
		// position. The guide origin is the location as it was at the moment of the throw, and a throw
		// into open space has no owner-local guide frame and is pinned entirely to the throw context
		// origin, so aiming at it would detach node 0 from the hand whenever the character moves during
		// the throw. GuidedThrow disables the solve, so the solver's hand pin does not apply either;
		// it is pinned to the current hand directly here.
		// PrepareSimFrame already refreshed StartPinTarget from this frame's component location.
		if (NodeIndex == 0 && Sim.bStartPinned)
		{
			SimFrame.OverrideFrame.SetPosition(0, Sim.StartPinTarget, /*bZeroVelocity*/ true);
			SimFrame.OverrideFrame.SetInvMass(0, 0.0f);
			continue;
		}

		const float NodeFrac = (LastNode > 0) ? static_cast<float>(NodeIndex) / static_cast<float>(LastNode) : 0.0f;

		// Every other node interpolates from its starting position towards the guide and has the
		// time-based upward arc offset added.
		// On an aimed throw only the tip node is re-aimed at the live target position; the rest keep
		// their preview guide points. A throw into open space takes its point on the live-hand line
		// instead (see FreeHandLive above).
		const FVector Target = bFree
			? FMath::Lerp(FreeHandLive, FreeEndpointLive, NodeFrac)
				- FVector::UpVector * (FreeSagDepth * 4.0f * NodeFrac * (1.0f - NodeFrac))
			: ((bTrackAimTarget && NodeIndex == LastNode)
				? AimTargetWorld
				: Prepared.ResolveGuidePointWorld(NodeIndex));
		const FVector Start = GuidedThrowState.StartPositions.IsValidIndex(NodeIndex)
			? GuidedThrowState.StartPositions[NodeIndex]
			: Sim.Positions[NodeIndex];
		FVector Position = FMath::Lerp(Start, Target, EasedAlpha);
		Position += FVector::UpVector * (ArcHeight * ArcT * NodeFrac);

		SimFrame.OverrideFrame.SetPosition(NodeIndex, Position, /*bZeroVelocity*/ true);
		SimFrame.OverrideFrame.SetInvMass(NodeIndex, 0.0f);
	}

	if (Alpha >= 1.0f)
	{
		if (bFree)
		{
			// A throw into open space completes without embedding: the unpinned nodes are returned to
			// physics and it falls to Free.
			for (int32 NodeIndex = 0; NodeIndex < Sim.Num(); ++NodeIndex)
			{
				SimFrame.OverrideFrame.SetInvMass(NodeIndex, (NodeIndex == 0 && Sim.bStartPinned) ? 0.0f : 1.0f);
				SimFrame.OverrideFrame.SetPrevFromPosition(NodeIndex);
			}
			SetPhase(ERopePhase::Free, TEXT("free guided throw landed"));
			ResetTransientPhaseState();
		}
		else
		{
			FinishGuidedThrow();
		}
	}
}

void URopeComponent::FinishGuidedThrow()
{
	// This is for aimed throws only. A throw into open space lands in Free through UpdateGuidedThrow and
	// never reaches here.
	const FRopePreparedThrowPreview Prepared = GuidedThrowState.Prepared;
	if (!Prepared.IsValid() || Prepared.Anchors.Num() == 0 || !Prepared.Mesh.IsValid())
	{
		AbortGuidedThrow(ERopeReleaseReason::Broken, TEXT("guided throw commit failed"));
		return;
	}

	// The anchors the preview builder produced are promoted directly into the wrapped seed, so no contact
	// is searched for again on completion and the rope is pinned to exactly the bones and local anchors
	// the preview used.
	FRopeWrapState Seed;
	Seed.BoneName = Prepared.Bone;
	Seed.Mesh = Prepared.Mesh;
	Seed.Anchors = Prepared.Anchors;
	for (const FRopeSurfaceAnchor& Anchor : Seed.Anchors)
	{
		FRopeLatchNode Latch;
		Latch.NodeIndex = Anchor.NodeIndex;
		Latch.Bone = Anchor.Bone;
		Seed.Latched.Add(Latch);
	}

	// Pierce: freeze the mesh pose so the tip socket sits embedded at the aim hit point, and carry it on
	// the anchor, which freezes the rotation while keeping the tail connected.
	// LocalMeshTransform is the tip's render pose in bone-local space and LocalSurfacePosition is where
	// the rope connects, at the tail, in the same space.
	// With no sockets configured the anchor is left as it is, which falls back to the current behaviour
	// of the origin at the hit point and the tip following the segment.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap && Seed.Anchors.Num() > 0)
	{
		FRopeSurfaceAnchor& Anchor = Seed.Anchors[0];
		const USceneComponent* Mesh = Seed.Mesh.Get();
		FVector HitPoint = Anchor.StartWorldPosition; // Set to the embed point by the builder.
		ResolvePreparedPierceHitPoint(Prepared, HitPoint);
		FVector PierceDir = (HitPoint - Sim.Positions[0]).GetSafeNormal();
		if (PierceDir.IsNearlyZero())
		{
			PierceDir = Prepared.ThrowContext.FrameForward.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		}

		FTransform ComponentWorld;
		FVector TailWorld;
		if (Mesh && !Anchor.Bone.IsNone() && ComputePierceEmbed(HitPoint, PierceDir, ComponentWorld, TailWorld))
		{
			const FTransform BoneXform = ResolveBindingWorld(Mesh, Anchor.Bone);
			Anchor.LocalMeshTransform = ComponentWorld.GetRelativeTransform(BoneXform);
			Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(TailWorld);
		}
	}

	ResetKinematicVirtualBridges();
	WrapController.BeginWrap(Sim, Seed, SimFrame.OverrideFrame);
	if (!WrapController.State.IsWrapped())
	{
		AbortGuidedThrow(ERopeReleaseReason::Broken, TEXT("guided throw begin wrap failed"));
		return;
	}

	ApplyWrappedMassMask(/*bResetDynamicNodeVelocity*/ true);

	// GuaranteedWrap passes through neither Flight nor Contacting, so Captured never fired even once,
	// yet the release does fire, which left consumers with the unpaired pair of a release with no
	// capture. Arriving is capturing here, so it is fired to produce the same Captured then Wrapped
	// order as the other modes. It is placed after BeginWrap succeeds so there is no window in which
	// Captured fires and Wrapped never follows.
	DispatchCaptured(Seed.BoneName);

	SetPhase(ERopePhase::Wrapped, *FString::Printf(TEXT("guided throw bone=%s, %d anchor(s)"),
		*Seed.BoneName.ToString(), Seed.Anchors.Num()));
	ResetTransientPhaseState();
	// A preview-based commit passes through no judgement, so the judgement values are contractually -1,
	// meaning not measured.
	const FRopeWrappedEventInfo WrappedInfo = MakeWrappedEventInfo(Seed, /*AngleDeg*/ -1.0f, /*CoverageDeg*/ -1.0f);
	DispatchWrapped(WrappedInfo);
}

FVector URopeComponent::ResolveFreeThrowEndpoint(const FRopeThrowContext& ResolvedThrow) const
{
	// One rope radius of clearance is enough to keep the tube off the surface and needs no knob of its own.
	return FRopeAimTargeting::ResolveOpenSpaceThrowEndpoint(MakeAimQueryContext(),
		ResolvedThrow.Origin, ResolvedThrow.FrameForward,
		FMath::Max(Sim.RopeLength, RopeLength), FMath::Max(Radius, 1.0f));
}

bool URopeComponent::StartFreeGuidedThrow(const FRopeThrowContext& ThrowContext, const FVector& EndpointWorld)
{
	// A throw into open space, with no target: it plays the straight line from the hand origin to the end
	// of the ray as an arc, and on completion falls to Free without embedding.
	// The preview holds only that straight world line; ResolveGuidePointWorld falls back to the world
	// points when there is no owner-local frame.
	EnsureRopeInitialized();
	if (Sim.Num() < 2)
	{
		return false;
	}

	const FVector Origin = ThrowContext.Origin;

	FRopePreparedThrowPreview Free;
	Free.bValid = false; // No target: the free path proceeds through GuidedThrowState.bFreeThrow and needs no IsValid.
	Free.ThrowContext = ThrowContext;
	Free.RenderPreview.Points.SetNum(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(Sim.Num() - 1);
		Free.RenderPreview.Points[i] = FMath::Lerp(Origin, EndpointWorld, Alpha);
	}
	Free.RenderPreview.Radius = FMath::Max(0.1f, Radius * 1.05f);
	Free.RenderPreview.NumSides = FMath::Clamp(NumSides, 3, 32);

	ResetStateForNewThrow();
	const FString PhaseReason = FString::Printf(TEXT("free throw to ray-end, len=%.0f"),
		static_cast<float>((EndpointWorld - Origin).Size()));
	if (!BeginGuidedThrowState(MoveTemp(Free), /*bFreeThrow*/ true))
	{
		return false;
	}

	SetPhase(ERopePhase::GuidedThrow, *PhaseReason);
	return true;
}

FRopeWhipGuide::FConfig URopeComponent::MakeWhipGuideConfig() const
{
	// The swing duration is controlled by throw speed alone: passing the internal reference values is
	// enough, and ResolveGuideDuration scales the effective duration as the reference duration multiplied
	// by the reference speed and divided by the actual throw speed, so faster throws swing for less time.
	static constexpr float ReferenceWhipDuration = 0.35f;
	static constexpr float ReferenceWhipThrowSpeed = 1500.0f;

	FRopeWhipGuide::FConfig Config;
	Config.Duration = ReferenceWhipDuration;
	Config.GuidedLength = WhipConfig.GuidedLength;
	Config.SweepAngleDegrees = WhipConfig.SweepAngleDegrees;
	Config.ReferenceThrowSpeed = ReferenceWhipThrowSpeed;
	Config.ComponentRopeLength = RopeLength;
	Config.FullSimInitialCurveFraction = WhipConfig.FullSimInitialCurveFraction;
	Config.FullSimStraightenTimeFraction = WhipConfig.FullSimStraightenTimeFraction;
	// Pass the component settings through so the CPU, the GPU and the preview all use the same aim-hit
	// endpoint envelope and direction bias.
	Config.AimHitRootSolverFraction = WhipConfig.AimHitRootSolverFraction;
	Config.AimHitTipSolverFraction = WhipConfig.AimHitTipSolverFraction;
	Config.AimHitDirectionBias = WhipConfig.AimHitDirectionBias;
	return Config;
}

float URopeComponent::TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const
{
	if (LastNode <= FirstTailNode)
	{
		return NodeIndex >= LastNode ? 1.0f : 0.0f;
	}

	const float T = static_cast<float>(NodeIndex - FirstTailNode) / static_cast<float>(LastNode - FirstTailNode);
	return RopeMath::SmoothStep(T);
}

#pragma endregion Throw

#pragma region Flight

// ===== Flight ===============================================================

FRopeFlightContactDetector::FParams URopeComponent::MakeFlightDetectParams(float DeltaTime) const
{
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = GetEffectiveContactQueryRadius();
	Params.RopeRadius = Radius;
	Params.PredictiveContactFrames = WrapConfig.PredictiveContactFrames;
	Params.MinLatchNodes = WrapConfig.MinLatchNodes;
	Params.FallbackForward = GetForwardVector();
	// The detection sweep resolution, which is what prevents tunnelling. The same value is carried on the
	// GPU step through RequestContactDetection.
	Params.ContactSweepStep = WrapConfig.ContactSweepStep;
	Params.ContactMaxSweepSamples = WrapConfig.ContactMaxSweepSamples;
	// The substep delta, (1/60) divided by the substep count, is the bridge that puts the rope's Verlet
	// displacement, which is the last substep's delta, into the same units as the surface velocity in
	// cm/s. It is the same expression the solver uses. It is not the frame delta; see the comment on
	// FParams for why.
	Params.SubstepDeltaTime = (1.0f / 60.0f) / static_cast<float>(FMath::Clamp(SolverConfig.Substeps, 1, 16));
	// The frame delta, used by the predictive contact extrapolation to convert a substep displacement
	// into a frame displacement; see the comment on FParams::FrameDeltaTime.
	Params.FrameDeltaTime = DeltaTime;
	return Params;
}

void URopeComponent::RemoveNonWrappableCandidates(TArray<FRopeContactCandidate>& Candidates) const
{
	// The subclass wrap target gate, CanWrapTarget. Filtered targets are removed entirely so they are
	// invisible to the tracker and the capture decision, which stops the tracker latching onto a
	// forbidden target and stalling the phase. The default implementation permits everything, so the
	// filter is a no-op, and with only a few dozen candidates per frame at most the cost is negligible.
	// Flight, which produces the candidates, and Contacting, which re-collects them, share it: if the
	// condition changed on only one side, the two phases would judge from different candidate sets,
	// which is a subtle bug. Both therefore go through this helper.
	Candidates.RemoveAll([this](const FRopeContactCandidate& Candidate)
	{
		// Candidates from the GPU delay or injected externally are put through the mode policy too.
		// AssistedJudged keeps other bones on the same mesh as material for secondary and multi-bone
		// wrapping, while only GuaranteedWrap restricts it to the exact mesh and bone.
		return !AimTargeting.IsWrapTarget(Phase, ResolveMode, Candidate.Mesh, Candidate.Bone) ||
			!CanWrapTarget(Candidate.Mesh, Candidate.Bone);
	});
}

TArray<FRopeContactCandidate>& URopeComponent::GetOrBuildFlightContactCandidates(float DeltaTime,
	const FRopeFlightContactDetector::FParams& DetectParams)
{
	TArray<FRopeContactCandidate>& Candidates = SimFrame.bGpuContactsThisFrame
		? SimFrame.GpuFlightCandidates
		: ContactCandidateScratch;

	if (SimFrame.bGpuContactsThisFrame)
	{
		// On the GPU detection path both actual and predictive candidates come from the GPU kernel, with
		// the subsystem restoring their attribution and deduplication. The frame arrays are post-processed
		// in place rather than copied or allocated per frame.
		// Only the relative motion evaluation runs on the game thread, because ExpectedWrapTangent needs
		// the hand position at node 0.
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightGpuContacts);
		FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, Candidates);
	}
	else
	{
		Candidates.Reset();
		NextGuideTargetScratch.Reset();
		BuildCpuFlightContactCandidates(DeltaTime, DetectParams, Candidates);
	}

	// AssistedJudged can optionally disable the solver push-out, which makes a contact last a single
	// frame as a pulse. Leaving that pulse to the asynchronous GPU readback alone risks losing it
	// permanently between in-flight copies, so the guide targets the CPU already holds are tested
	// synchronously against the exact aim primary collider, which gives the same result on the CPU
	// fallback.
	AddSynchronousAssistedAimContactCandidates(DeltaTime, DetectParams, Candidates);
	// The full-simulation counterpart: a whip-guided crossing of a thin isolated target can complete inside
	// one frame at a low frame rate, and only this synchronous guide-path sweep preserves it as a same-frame
	// Actual contact (see the declaration).
	AddSynchronousWhipGuidedContactCandidates(DetectParams, Candidates);

	// The CanWrapTarget gate, through the helper shared with the Contacting re-collection.
	RemoveNonWrappableCandidates(Candidates);
	return Candidates;
}

void URopeComponent::BuildCpuFlightContactCandidates(float DeltaTime,
	const FRopeFlightContactDetector::FParams& DetectParams, TArray<FRopeContactCandidate>& OutCandidates)
{
	// Only CPU predictive contact needs a view of the whip data. On the GPU path the subsystem already
	// computed the same next-frame targets before dispatch and carried them on the GPU step, so they are
	// not built again during Finalize.
	// With prediction disabled the detector early-outs anyway, so no preview is built.
	// NextGuideTargetScratch is the member buffer the view points at; it is reset on its next CPU use
	// after detection finishes.
	FRopeFlightContactDetector::FWhipGuideView WhipView;
	if (WrapConfig.PredictiveContactFrames > KINDA_SMALL_NUMBER && WhipGuide.GetGuidedNodeMask().Num() > 0)
	{
		WhipGuide.PreviewNextTargets(DeltaTime, Sim, MakeWhipGuideConfig(), NextGuideTargetScratch);
		WhipView.GuidedNodeMask = &WhipGuide.GetGuidedNodeMask();
		WhipView.CurrentTargets = &WhipGuide.GetCurrentTargets();
		WhipView.PrevTargets = &WhipGuide.GetPrevTargets();
		WhipView.NextTargets = &NextGuideTargetScratch;
	}

	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightActualContacts);
		FRopeFlightContactDetector::DetectContactCandidates(Sim, SimFrame.FrameColliders, DetectParams, OutCandidates);
	}
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightPredictiveContacts);
		FRopeFlightContactDetector::AddPredictedContactCandidates(Sim, SimFrame.FrameColliders, DetectParams, WhipView, OutCandidates);
	}
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightEvaluateCandidates);
		FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, OutCandidates);
	}
}

void URopeComponent::AddSynchronousAssistedAimContactCandidates(float DeltaTime,
	const FRopeFlightContactDetector::FParams& DetectParams,
	TArray<FRopeContactCandidate>& InOutCandidates)
{
	const bool bFlight = Phase == ERopePhase::Flight;
	const bool bContacting = Phase == ERopePhase::Contacting;
	if (ResolveMode != ERopeWrapResolveMode::AssistedJudged || !AimTargeting.IsLockActive(Phase) ||
		(!bFlight && !bContacting) || (bFlight && WhipGuide.GetGuidedNodeMask().Num() == 0))
	{
		return;
	}

	// AssistedJudged keeps neighbouring bones on the same mesh as wrapping material, but the capture
	// primary is the exact aimed bone. Testing only the deepest single contact across the whole list
	// would let the torso or a neighbouring bone hide the aimed one, so this probe alone is narrowed to
	// the exact target.
	TArray<IRopeCollider*> PrimaryColliders;
	for (IRopeCollider* Collider : SimFrame.FrameColliders)
	{
		if (!Collider || Collider->IsWorldStatic())
		{
			continue;
		}
		FName ColliderBone = NAME_None;
		const USceneComponent* ColliderMesh = nullptr;
		Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
		if (AimTargeting.IsPrimaryTarget(ColliderMesh, ColliderBone))
		{
			PrimaryColliders.Add(Collider);
		}
	}
	if (PrimaryColliders.Num() == 0)
	{
		return;
	}

	FRopeFlightContactDetector::FParams ReliableParams = DetectParams;
	ReliableParams.ContactMaxSweepSamples = FMath::Max(
		ReliableParams.ContactMaxSweepSamples,
		FRopeFlightContactDetector::ReliableGuidedSweepMaxSamples);

	if (bFlight)
	{
		FRopeFlightContactDetector::FWhipGuideView WhipView;
		WhipView.GuidedNodeMask = &WhipGuide.GetGuidedNodeMask();
		WhipView.CurrentTargets = &WhipGuide.GetCurrentTargets();
		WhipView.PrevTargets = &WhipGuide.GetPrevTargets();
		const bool bIncludePrediction = ReliableParams.PredictiveContactFrames > KINDA_SMALL_NUMBER;
		if (bIncludePrediction)
		{
			NextGuideTargetScratch.Reset();
			WhipGuide.PreviewNextTargets(DeltaTime, Sim, MakeWhipGuideConfig(), NextGuideTargetScratch);
			WhipView.NextTargets = &NextGuideTargetScratch;
		}

		FRopeFlightContactDetector::AddGuidedContactCandidates(
			Sim, PrimaryColliders, ReliableParams, WhipView, InOutCandidates);
		if (bIncludePrediction)
		{
			FRopeFlightContactDetector::AddPredictedContactCandidates(
				Sim, PrimaryColliders, ReliableParams, WhipView, InOutCandidates);
		}
	}
	else
	{
		// The movement pulse from the last Flight frame's previous to current targets is not reused. Only
		// whether the current rope centreline, synchronized by the GPU handoff, genuinely touches the
		// exact primary is considered, which prevents dwell accumulating falsely.
		FRopeFlightContactDetector::AddCurrentCenterlineContactCandidates(
			Sim, PrimaryColliders, ReliableParams, InOutCandidates);
	}
	// Newly added game-thread candidates are given the same relative motion fields as the GPU and CPU
	// ones. Capture does not currently gate on that score, but the tracker's dominant scoring and the
	// debug observations have to keep the same contract.
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, ReliableParams, InOutCandidates);
}

void URopeComponent::AddSynchronousWhipGuidedContactCandidates(
	const FRopeFlightContactDetector::FParams& DetectParams,
	TArray<FRopeContactCandidate>& InOutCandidates)
{
	// Contract and rationale on the declaration. AssistedJudged with an active lock keeps its narrowed
	// exact-primary probe, and GuaranteedWrap never captures from Flight, so both stand down here.
	if (Phase != ERopePhase::Flight || WhipGuide.GetGuidedNodeMask().Num() == 0 ||
		ResolveMode == ERopeWrapResolveMode::GuaranteedWrap ||
		(ResolveMode == ERopeWrapResolveMode::AssistedJudged && AimTargeting.IsLockActive(Phase)))
	{
		return;
	}

	// Every wrappable collider takes part: world-static push-out shapes carry no bone and cannot seed a
	// wrap, so they are skipped before the sweep rather than per contact.
	TArray<IRopeCollider*> GuidedSweepColliders;
	for (IRopeCollider* Collider : SimFrame.FrameColliders)
	{
		if (Collider && !Collider->IsWorldStatic())
		{
			GuidedSweepColliders.Add(Collider);
		}
	}
	if (GuidedSweepColliders.Num() == 0)
	{
		return;
	}

	FRopeFlightContactDetector::FWhipGuideView WhipView;
	WhipView.GuidedNodeMask = &WhipGuide.GetGuidedNodeMask();
	WhipView.CurrentTargets = &WhipGuide.GetCurrentTargets();
	WhipView.PrevTargets = &WhipGuide.GetPrevTargets();
	FRopeFlightContactDetector::AddGuidedContactCandidates(
		Sim, GuidedSweepColliders, DetectParams, WhipView, InOutCandidates);
	// The same relative-motion contract as every other candidate source, for the tracker's dominant
	// scoring and the debug observations.
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, InOutCandidates);
}

FRopeFlightCaptureEvaluation URopeComponent::EvaluateFlightCapture(
	const TArray<FRopeContactCandidate>& Candidates,
	const FRopeFlightContactDetector::FParams& DetectParams) const
{
	// An assisted aim lock pins only the dominant target to the aimed bone. Tracker.Update aggregates
	// every candidate, so other bones on the same mesh remain in Targets as material for secondary dwell
	// and the multi-bone path.
	const bool bRequireAimPrimary = ResolveMode == ERopeWrapResolveMode::AssistedJudged
		&& AimTargeting.IsLockActive(Phase);
	FRopeFlightCapturePolicy Policy;
	if (bRequireAimPrimary)
	{
		Policy.PreferredMesh = AimTargeting.GetLockedTargetMesh();
		Policy.PreferredBone = AimTargeting.GetLockedTargetBone();
		Policy.bRequirePreferred = true;
	}

	FRopeFlightCaptureEvaluation Evaluation;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightEvaluateCapture);
		Evaluation = FRopeFlightContactDetector::EvaluateCapture(Candidates, DetectParams, Policy);
	}

	// Prediction exists to make the next real sweep reliable, not to stop the rope before it reaches the
	// surface. Contacting has no solve, so entering it from predictive-only candidates freezes the rope
	// short of the body and the current-centreline recheck immediately dismisses the contact. Require the
	// configured latch-node count to have an Actual source on the selected target. A merged candidate
	// keeps the predictive metadata but carries Actual in SourceMask, so real swept contacts still capture
	// on the same frame.
	if (Evaluation.bShouldCapture)
	{
		const uint8 ActualMask = static_cast<uint8>(ERopeContactCandidateSource::Actual);
		TSet<int32> ActualNodes;
		for (const FRopeContactCandidate& Candidate : Candidates)
		{
			if (Candidate.bValid
				&& Candidate.Mesh == Evaluation.Tracker.CandidateMesh
				&& Candidate.Bone == Evaluation.Tracker.CandidateBone
				&& (Candidate.SourceMask & ActualMask) != 0)
			{
				ActualNodes.Add(Candidate.NodeIndex);
			}
		}
		Evaluation.bShouldCapture =
			ActualNodes.Num() >= FMath::Max(1, DetectParams.MinLatchNodes);
	}

	// GuaranteedWrap does not reach Flight on the normal path: ThrowWithContext sends both the aimed
	// throw, through the prepared preview, and the open-space throw, as an arc to the end of the ray, to
	// GuidedThrow. Should it reach Flight by any route, capture is forbidden anyway, because a
	// GuaranteedWrap is only ever established from the anchors GuidedThrow committed to. This is a
	// defensive backstop.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		Evaluation.bShouldCapture = false;
	}
	return Evaluation;
}

bool URopeComponent::ApplyFlightCaptureEvaluation(float DeltaTime,
	const TArray<FRopeContactCandidate>& Candidates, FRopeFlightCaptureEvaluation& Evaluation)
{
	if (Evaluation.bShouldCapture)
	{
		FlightNoContactElapsed = 0.0f;
		// The GPU path only queued a step on the render thread's pending queue before this Finalize, so
		// the CPU simulation state may not be at that post-step pose yet. A step that needs no global
		// distance field, meaning an aim collision-free one, is consumed and read back atomically now, so
		// a Flight swept hit is established on the same capture frame as it would be on the CPU. Only
		// steps that need the scene's distance field are handed to Contacting with a persistent marker
		// and retried after a valid view dispatch.
		bPendingGpuCaptureHandoff = SimFrame.bGpuSteppedThisFrame;
		if (bPendingGpuCaptureHandoff)
		{
			if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
			{
				bPendingGpuCaptureHandoff = !SimSubsystem->SyncGpuPositionsForHandoff(*this);
			}
		}
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightBuildContactingState);
		BuildContactingState(MoveTemp(Evaluation.Tracker), Candidates, DeltaTime);
		SetPhase(ERopePhase::Contacting, *FString::Printf(TEXT("bone=%s, %d node(s)"),
			*ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num()));
		DispatchCaptured(ContactTracker.CandidateBone);
		// The capture frame is itself one frame of real contact. If the default decision time, which is
		// about one frame, is already satisfied, the rope moves to Wrapping immediately rather than
		// waiting to detect again next frame, which reduces bouncing off a moving target.
		// On the CPU the current simulation state is already authoritative, so the immediate transition
		// stands. On the GPU it transitions after re-confirming contact against the current centreline on
		// the Contacting frame that follows the persistent handoff.
		if (ShouldStartWrapping() && !bPendingGpuCaptureHandoff)
		{
			StartWrappingFromContacting();
		}
		return true;
	}

	// Failing to capture before the whip ends is treated as a failed throw and returns the rope to Free.
	if (!WhipGuide.IsActive())
	{
		// PREDICTIVE-GRACE BEGIN -- remove only this block to restore the strict no-contact timeout.
		// Prediction still cannot capture: it only keeps Flight alive while the selected target remains
		// immediately ahead, giving the real sweep a chance to confirm contact on a following frame.
		// Actual-only, invalid, and non-selected candidates do not extend the timeout.
		const uint8 PredictiveMask =
			static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree) |
			static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided);
		bool bHasSelectedPredictiveCandidate = false;
		for (const FRopeContactCandidate& Candidate : Candidates)
		{
			if (Candidate.bValid
				&& !Evaluation.Tracker.CandidateBone.IsNone()
				&& Candidate.Mesh == Evaluation.Tracker.CandidateMesh
				&& Candidate.Bone == Evaluation.Tracker.CandidateBone
				&& (Candidate.SourceMask & PredictiveMask) != 0)
			{
				bHasSelectedPredictiveCandidate = true;
				break;
			}
		}
		if (bHasSelectedPredictiveCandidate)
		{
			FlightNoContactElapsed = 0.0f;
			return false;
		}
		// PREDICTIVE-GRACE END

		const float FlightReturnTime = ThrowParams.FlightNoContactReturnTime > 0.0f
			? ThrowParams.FlightNoContactReturnTime
			: ReleaseCooldownSeconds;
		FlightNoContactElapsed += DeltaTime;
		if (FlightNoContactElapsed >= FlightReturnTime)
		{
			InjectPinnedFrameVelocityForFreeReturn(Sim);
			SetPhase(ERopePhase::Free, *FString::Printf(TEXT("flight failed %.3fs"), FlightNoContactElapsed));
			ResetTransientPhaseState();
		}
	}
	else
	{
		FlightNoContactElapsed = 0.0f;
	}
	return false;
}

void URopeComponent::BuildContactingState(FRopeContactTracker&& EvaluatedTracker,
	const TArray<FRopeContactCandidate>& Candidates, float DeltaTime)
{
	ContactTracker = MoveTemp(EvaluatedTracker);
	// The zero-second aggregation during evaluation only selects a target, so the capture frame's real
	// contact time is applied here. When an actual swept hit is confirmed on an assisted lock, it counts
	// as at least one nominal frame of dwell at 60 Hz.
	// Above 120 Hz, adding the raw delta alone and deferring to the next current-only frame would let a
	// collision-free guide pass clear through a thin limb, which both the GPU and the CPU would miss. A
	// longer user-configured decision time still demands further contact, and predictive-only candidates
	// are not boosted, so something that has not touched yet is never promoted straight to a wrap.
	const uint8 ActualMask = static_cast<uint8>(ERopeContactCandidateSource::Actual);
	const bool bDominantHasActual = Candidates.ContainsByPredicate(
		[this, ActualMask](const FRopeContactCandidate& Candidate)
		{
			return Candidate.bValid
				&& Candidate.Mesh == ContactTracker.CandidateMesh
				&& Candidate.Bone == ContactTracker.CandidateBone
				&& (Candidate.SourceMask & ActualMask) != 0;
		});
	const bool bConfirmedAssistedActual = ResolveMode == ERopeWrapResolveMode::AssistedJudged
		&& AimTargeting.IsLockActive(Phase)
		&& ContactTracker.CandidateMesh == AimTargeting.GetLockedTargetMesh()
		&& ContactTracker.CandidateBone == AimTargeting.GetLockedTargetBone()
		&& bDominantHasActual;
	const float BaseCapturedFrameDwell = FMath::Max(0.0f, DeltaTime);
	constexpr float NominalContactFrameSeconds = 1.0f / 60.0f;
	const float DominantCapturedFrameDwell = bConfirmedAssistedActual
		? FMath::Max(BaseCapturedFrameDwell, NominalContactFrameSeconds)
		: BaseCapturedFrameDwell;
	ContactTracker.DwellTime = FMath::Max(ContactTracker.DwellTime, DominantCapturedFrameDwell);
	for (FRopeTrackedContactTarget& Target : ContactTracker.Targets)
	{
		if (Target.Nodes.Num() > 0)
		{
			const bool bDominantTarget = Target.Mesh == ContactTracker.CandidateMesh
				&& Target.Bone == ContactTracker.CandidateBone;
			Target.DwellTime = FMath::Max(Target.DwellTime,
				bDominantTarget ? DominantCapturedFrameDwell : BaseCapturedFrameDwell);
		}
	}
	ContactingElapsed = 0.0f;
	PendingWrapSeed = BuildWrapSeedFromContactingState(Candidates);

	// This is the last chance to snapshot the travel frame: from Contacting onwards there is no solve, so
	// the nodes come to rest, their current and previous positions converge and the velocity information
	// dies. The CaptureTravelPlane axis in Wrapping consumes it.
	CaptureTravelFrame = FRopeCaptureTravelFrame::Compute(Sim, Candidates, DeltaTime);
}

#pragma endregion Flight

