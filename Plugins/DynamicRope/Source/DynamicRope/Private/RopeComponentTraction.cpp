// Copyright 2026 TeamKeno. All Rights Reserved.

#include "RopeComponent.h"

#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "DynamicRopeLog.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Core/RopeMovementConstraint.h"
#include "Logic/RopeLengthConstraintSolver.h"
#include "Logic/RopeTractionSolver.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RopeMathHelpers.h"
#include "Templates/Function.h"

#if !UE_BUILD_SHIPPING
// Wrapped lift diagnostics: every N frames per rope, one line with the material length, the leg limit, the
// measured hand-to-anchor span, the violation, the backend and the tether state — the numbers needed to see
// why a reel-in is (or is not) hoisting its target. Off by default; enable with "dr.Rope.LiftDebug 15".
static TAutoConsoleVariable<int32> CVarRopeLiftDebug(
	TEXT("dr.Rope.LiftDebug"), 0,
	TEXT("Log Wrapped tether/reel lift state every N frames per rope (0 = off)."));
#endif

namespace
{
	// The taut-check hysteresis and the release grace period are internal constants, settled by measurement
	// and folded into the single TautSensitivity knob rather than exposed to designers. Adjust them here.
	constexpr float TautSlackReleaseScaleConst = 2.0f;       // slack/sag gate maintenance multiplier (≥1, entry/release threshold separation)
	constexpr float TautReleaseGraceTimeConst = 0.1f;        // bChainTaut release grace period (seconds)
	constexpr float ActivePullTautReleaseRatioConst = 0.5f;  // Active Pull load threshold hysteresis [0..1]
	constexpr float TautMinTensionReleaseRatioConst = 0.5f;  // Minimum transfer tension hysteresis [0..1]
	constexpr float TetherLiftLaunchSpeedConst = 100.0f;     // Ground character Walking → Falling transition upward threshold (cm/s)
}

#pragma region Tension_Query

float URopeComponent::GetSegmentTension(int32 SegmentIndex) const
{
	return Sim.SegmentTension.IsValidIndex(SegmentIndex) ? Sim.SegmentTension[SegmentIndex] : 0.0f;
}

float URopeComponent::GetMaxTension() const
{
	float MaxTension = 0.0f;
	for (const float T : Sim.SegmentTension)
	{
		MaxTension = FMath::Max(MaxTension, T);
	}
	return MaxTension;
}
#pragma endregion Tension_Query

#pragma region Wrapped_Pull_Sampling_And_Application

void URopeComponent::UpdateWrappedPullSample(float DeltaTime, const FRopeSimState& ObservationSim)
{
	// Gameplay tension is the material-length reaction, not delayed XPBD segment strain.
	// ApplyWrappedTraction refreshes this again after the current frame's backend has solved.
	WrapController.State.Tension = GetConstraintTension();

	// Always produce the pull sample: the debugger and Blueprint observe it, and traction and release take it as shared input. The direction follows the first straight leg, so it goes around corners.
	PullDrive.LastPullSample = FRopePullSample();
	WrapController.ComputePull(ObservationSim, HoldConfig.PullBendThresholdDeg, PullDrive.LastPullSample);

	// Taut is geometry, not a prerequisite XPBD load. Prefer the same live hand/anchor
	// material boundary used by movement authority. This removes the circular dependency
	// "segment must stretch -> tension appears -> tether may enforce no stretch".
	FRopeWielderMovementConstraint LiveConstraint;
	const bool bHasLiveConstraint = BuildWielderMovementConstraint(LiveConstraint);
	// The slack ratio and the sag cap both resolve from the single TautSensitivity knob (GetEffectiveTaut*),
	// and the hysteresis is an internal constant. The live material-length path and the legacy geometry path
	// share these values so they react in the **same direction** — otherwise the slider would do nothing with
	// bEnforceWielderLengthConstraint on, which is the default.
	// Entry and hold use different thresholds: once taut, the tolerance widens so the decision cannot chatter
	// at the boundary (the grace latch below covers the rest).
	const float EffectiveTautMaxSag = GetEffectiveTautMaxSag();
	const float EffectiveTautSlackRatio = GetEffectiveTautSlackRatio();
	const float TautHysteresis = PullDrive.bChainTaut ? TautSlackReleaseScaleConst : 1.0f;
	const float SagLimit = EffectiveTautMaxSag * TautHysteresis;
	const bool bSagTaut =
		EffectiveTautMaxSag <= 0.0f || PullDrive.LastPullSample.MaxLegSag <= SagLimit;

	// Live boundary: the traction start distance comes from TautSensitivity, floored at
	// LengthConstraintActivationSlop for numerical stability — whichever is larger wins. The sag gate, which
	// is what "only once it visibly straightens" means, is shared here too.
	const float LiveSlackAllowance = FMath::Max(
		FMath::Max(HoldConfig.LengthConstraintActivationSlop, 0.0f),
		LiveConstraint.MaxDistance * EffectiveTautSlackRatio * TautHysteresis);
	const float LiveDistance =
		static_cast<float>(FVector::Distance(GetComponentLocation(), LiveConstraint.PivotWorld));
	const bool bLiveBoundaryTaut = bHasLiveConstraint
		&& LiveDistance >= FMath::Max(0.0f, LiveConstraint.MaxDistance - LiveSlackAllowance)
		&& bSagTaut;

	// Legacy/self-wrap fallback still uses sag + chord geometry. SegmentTension is deliberately
	// excluded from gameplay taut; it remains only as a legacy analytic-path contamination guard.
	const bool bLegacyGeometryTaut =
		bSagTaut
		&& (EffectiveTautSlackRatio <= 0.0f || RopeTraction::EvaluateChainTautGate(
			PullDrive.LastPullSample.TautChordLen,
			PullDrive.LastPullSample.FreeRestLen,
			EffectiveTautSlackRatio,
			TautSlackReleaseScaleConst,
			PullDrive.bChainTaut));
	const bool bRawChainTaut = PullDrive.LastPullSample.bValid
		&& (bHasLiveConstraint ? bLiveBoundaryTaut : bLegacyGeometryTaut);
	// Release grace period, a time latch. With the minimum transmitted tension threshold at its default of 0,
	// entry and hold use the same threshold and the hysteresis disappears; on a GPU rope, which mirrors one to
	// two frames late, the decision then flickers frame by frame right at the boundary and the rope alternates
	// between cutting all velocity and letting it go — which the wielder feels as juddering.
	// So entry stays immediate and only the release waits out TautReleaseGraceTime. An invalid sample drops to
	// false immediately with no grace, which keeps the contract above honest: the grace period must not stretch
	// an observation gap into a claim that the rope is taut.
	if (bRawChainTaut)
	{
		PullDrive.bChainTaut = true;
		PullDrive.TautGraceRemaining = TautReleaseGraceTimeConst;
	}
	else if (!PullDrive.LastPullSample.bValid)
	{
		PullDrive.bChainTaut = false;
		PullDrive.TautGraceRemaining = 0.0f;
	}
	else if (PullDrive.bChainTaut && PullDrive.TautGraceRemaining > 0.0f)
	{
		PullDrive.TautGraceRemaining -= DeltaTime;
		PullDrive.bChainTaut = PullDrive.TautGraceRemaining > 0.0f;
	}
	else
	{
		PullDrive.bChainTaut = false;
	}

	// Pull smoothing, stage two. (1) Fractional aim-node smoothing: the integer AimNode hops discretely
	// between frames — the whole direction jumps and the tether overshoot goes discontinuous — so the index is
	// smoothed as a float and interpolated between nodes. (2) Direction EMA: trims what noise is left on top
	// of the node position, such as the GPU mirror's lag. The first valid frame after a wrap begins is seeded
	// from the measurement, so there is no start-up lag.
	if (!PullDrive.LastPullSample.bValid)
	{
		return;
	}

	// Raw look-ahead before smoothing, with the integer aim node — the debugger compares raw against smoothed.
	PullDrive.LastPullDirRaw = PullDrive.LastPullSample.Direction;

	// (1) Smooth the aim index over time, then interpolate the fractional aim position.
	const float RawAimF = static_cast<float>(PullDrive.LastPullSample.AimNode);
	PullDrive.SmoothedAimNodeF = (PullDrive.SmoothedAimNodeF < 0.0f)
		? RawAimF // The first valid frame is seeded with the measurement value (no lag).
		: FMath::Lerp(PullDrive.SmoothedAimNodeF, RawAimF, RopeTraction::ExpSmoothAlpha(HoldConfig.PullAimSmoothTime, DeltaTime));
	const float AimF = FMath::Clamp(PullDrive.SmoothedAimNodeF, 0.0f, static_cast<float>(PullDrive.LastPullSample.AnchorNode));
	const FVector AimPos = RopeTraction::SampleFractionalAim(
		ObservationSim.Positions, AimF, PullDrive.LastPullSample.AnchorNode);
	PullDrive.LastPullSample.AimNodeF = AimF;
	PullDrive.LastPullSample.AimPos = AimPos;

	// (2) Recompute the direction from the continuous aim point, then apply the direction EMA. A degenerate case (aim == anchor) keeps the raw direction.
	const FVector DirF =
		(AimPos - ObservationSim.Positions[PullDrive.LastPullSample.AnchorNode]).GetSafeNormal();
	const FVector DirIn = DirF.IsNearlyZero() ? PullDrive.LastPullSample.Direction : DirF;
	PullDrive.SmoothedPullDir = RopeTraction::SmoothDirection(
		PullDrive.SmoothedPullDir, DirIn, RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	PullDrive.LastPullSample.Direction = PullDrive.SmoothedPullDir;
}

void URopeComponent::ApplyWrappedTraction(float DeltaTime)
{
	// The endpoint cache lives only for the synchronous span of this function, so a virtual ApplyPullForce
	// that calls Super reuses the same target resolution, while an ApplyPullForce called from outside cannot
	// leak a stale cache.
	WrappedEndpointCache.Reset();
	// Pullability check, shared by the tether's distribution observation (LastTargetShare) and the active pull's climb-in direction.
	if (PullDrive.LastPullSample.bValid)
	{
		UpdateTargetPullable();
	}

	// Automatic traction, part one — the tether: a single λ impulse constraint, plus the physics constraint on
	// a ragdoll. There is no slack ledger to break, because λ's position recovery term is bounded by
	// MaxBiasSpeed and so leaves no excess injection behind — that ledger was a relic of the per-end servo era.
	UpdateConstraintTether(DeltaTime);
	WrapController.State.Tension = GetConstraintTension();

	// Geometry starts a pull; load is an optional gameplay threshold, never a prerequisite
	// for the passive length solve. Threshold zero explicitly means geometry-only so a
	// stationary taut cable can begin an active pull and create its own reaction.
	if (HoldConfig.ActivePullTautTension <= 0.0f)
	{
		PullDrive.bPullTaut = PullDrive.bChainTaut;
	}
	else
	{
		PullDrive.bPullTaut = PullDrive.bChainTaut && RopeTraction::EvaluateTautGate(
			GetConstraintTension(),
			HoldConfig.ActivePullTautTension,
			ActivePullTautReleaseRatioConst,
			PullDrive.bPullTaut);
	}

	// Active pull, part two — a constant force. The force the user set (SetActivePull, or the Wielder) applies
	// only while the rope is under tension. The gate is the bPullTaut latch that step 2 updated: the previous
	// frame's chain geometry (bChainTaut) and the tension threshold together, with the threshold and its
	// hysteresis in HoldConfig, where a default threshold of 0 means simply tension > ~0.
	// Two layers can bypass the taut requirement: the config (bActivePullRequiresTaut = false, a rope-wide
	// policy) and the per-call flag (bActivePullIgnoresTaut, the SetActivePull argument, which is how an
	// animation pull window marks its span). Either way, a Wrapped rope with a valid sample is authorized.
	// The force is constant and unrelated to tension, so there is no feedback runaway.
	const bool bActivePullPassesGate = PullDrive.ActivePullForce > 0.0f && PullDrive.LastPullSample.bValid
		&& (!HoldConfig.bActivePullRequiresTaut || PullDrive.bActivePullIgnoresTaut || PullDrive.bPullTaut);
#if WITH_GAMEPLAY_DEBUGGER
	// The request value (ActivePullForce) alone cannot tell the debugger whether the force was actually
	// applied, because the taut gate and the two bypass layers are decided here. So whether the gate passed is
	// recorded as-is — separately from force being discarded later at the receiver.
	DebugActivePullPassedGate = bActivePullPassesGate;
#endif
	if (bActivePullPassesGate)
	{
		// Too heavy to pull (not pullable): put the force on the wielder instead and draw them toward the
		// anchor — climb-in. LastPullSample.Direction points anchor → hand, so negating it gives hand →
		// anchor: "if one of us has to move, it is me." That is what pulls the wielder to a wall, a heavy
		// ragdoll or a dragon, and is the basis of three-dimensional manoeuvring. Otherwise the force goes to
		// the target and draws it toward the wielder.
		if (!PullDrive.bTargetPullable)
		{
			ApplyPullForceToWielder(-PullDrive.LastPullSample.Direction * PullDrive.ActivePullForce, DeltaTime);
		}
		else
		{
			ApplyPullForce(PullDrive.LastPullSample.Direction * PullDrive.ActivePullForce, PullDrive.LastPullSample, DeltaTime);
		}
	}

#if !UE_BUILD_SHIPPING
	const int32 LiftDebugPeriod = CVarRopeLiftDebug.GetValueOnGameThread();
	if (LiftDebugPeriod > 0 && (GFrameCounter % static_cast<uint64>(LiftDebugPeriod)) == 0)
	{
		FRopeWielderMovementConstraint DbgConstraint;
		const bool bDbgLive = BuildWielderMovementConstraint(DbgConstraint);
		const FRopeResolvedWrappedEndpoints* DbgEndpoints = GetOrResolveWrappedEndpoints();
		UE_LOG(LogDynamicRope, Log,
			TEXT("[%s.%s] LIFTDBG len=%.1f leg=%.1f req=%.1f C=%.2f backend=%d tension=%.0f chaos=%d limit=%.1f reel=%.1f pull=%.0f tgtKind=%d anchorNode=%d anchorZ=%.0f"),
			*GetNameSafe(GetOwner()), *GetName(), Sim.RopeLength,
			bDbgLive ? DbgConstraint.MaxDistance : -1.0f,
			bDbgLive
				? static_cast<float>(FVector::Distance(GetComponentLocation(), DbgConstraint.PivotWorld))
				: -1.0f,
			LengthConstraintState.LastViolation,
			static_cast<int32>(LengthConstraintState.Backend),
			GetConstraintTension(),
			PhysicalTetherConstraint ? 1 : 0,
			PhysicalTetherLimit,
			ReelRate,
			PullDrive.ActivePullForce,
			DbgEndpoints ? static_cast<int32>(DbgEndpoints->Target.Kind) : -1,
			bDbgLive ? DbgConstraint.AnchorNode : -1,
			bDbgLive ? static_cast<float>(DbgConstraint.PivotWorld.Z) : 0.0f);
	}
#endif
	WrappedEndpointCache.Reset();
}

#pragma endregion Wrapped_Pull_Sampling_And_Application

#pragma region Active_Pull_API

void URopeComponent::SetActivePull(float Force, bool bIgnoreTautGate)
{
	PullDrive.ActivePullForce = FMath::Max(0.0f, Force);
	PullDrive.bActivePullIgnoresTaut = bIgnoreTautGate;
}
#pragma endregion Active_Pull_API

#pragma region Traction_Endpoint_And_Tether_Policies

namespace
{
	// Walk up the parent chain from the bone to the nearest bone with a *simulating* physics body, or None.
	// The wrapped bone can be one without a body in the physics asset — a twist bone, say — and taking the
	// bone name at face value would find it non-simulating and fall through to the character movement branch,
	// where a ragdoll setup has movement disabled (MOVE_None) and AddForce is silently dropped. Promoting the
	// bone here is what avoids that.
	FName FindNearestSimulatingBone(const USkeletalMeshComponent* Mesh, FName Bone)
	{
		while (!Bone.IsNone())
		{
			if (Mesh->IsSimulatingPhysics(Bone))
			{
				return Bone;
			}
			Bone = Mesh->GetParentBone(Bone);
		}
		return NAME_None;
	}

	// Is there a kinematic body anywhere up the parent chain from the simulating bone? That is a partial
	// ragdoll, and it means the kinematic constraint absorbs the traction completely — an infinite-mass wall —
	// so no servo or force on the bone reaches the actor. In that case the receiver must resolve past the bone
	// and down to the moving object, the character. Without one (everything simulating, root body included) it
	// is a free ragdoll and the whole body is pulled through the joints.
	// Bones with no body at all (twist, IK) constrain nothing and are skipped.
	bool IsSimBoneBoundToKinematic(const USkeletalMeshComponent* Mesh, FName SimBone)
	{
		for (FName Bone = Mesh->GetParentBone(SimBone); !Bone.IsNone(); Bone = Mesh->GetParentBone(Bone))
		{
			// A body that exists but does not simulate is a kinematic constraint. Simulation state is queried
			// through the component API, because FBodyInstance::IsInstanceSimulatingPhysics is a non-exported
			// inline and cannot be linked against.
			if (Mesh->GetBodyInstance(Bone) != nullptr && !Mesh->IsSimulatingPhysics(Bone))
			{
				return true;
			}
		}
		return false;
	}

	// Can the character movement consume force right now? Under MOVE_None (DisableMovement, the ragdoll setup
	// convention) AddForce only accumulates and is never consumed, so the force is lost to a fake success —
	// it has to go to another receiver instead.
	// The wrap target's actor is taken directly, whichever it is: skeletal, static or a physics prop. The
	// check is on the owning actor.
	UCharacterMovementComponent* GetForceConsumingMovement(const AActor* Owner)
	{
		const ACharacter* Character = Cast<ACharacter>(Owner);
		UCharacterMovementComponent* Movement = Character ? Character->GetCharacterMovement() : nullptr;
		return (Movement && Movement->MovementMode != MOVE_None) ? Movement : nullptr;
	}

	// Mass (kg) of the body force is applied to: for a skeletal bone, that body's mass, otherwise — or when
	// there is no body, or its mass is 0 — the component's mass. A physics body's mass is maintained by the
	// engine as collision volume × density, so nothing extra needs setting.
	// (It depends on a UObject, which is why RopeTraction's pure maths does not include it: the mass is read
	// here and passed in.)
	float ResolveBodyMass(const UPrimitiveComponent* Prim, FName BoneName)
	{
		float Mass = 0.0f;
		if (!BoneName.IsNone())
		{
			if (const USkeletalMeshComponent* Skel = Cast<const USkeletalMeshComponent>(Prim))
			{
				if (const FBodyInstance* Body = Skel->GetBodyInstance(BoneName))
				{
					Mass = static_cast<float>(Body->GetBodyMass());
				}
			}
		}
		return (Mass > KINDA_SMALL_NUMBER) ? Mass : static_cast<float>(Prim->GetMass());
	}

	// ===== Tether endpoint resolution — climb the ladder exactly once =====
	// "What receives the force" is decided once, here, and the kind, the application point and the effective
	// mass all come out together. There used to be two ladders — one for the distribution mass, one for the
	// actual application point — and on top of them the application lambda re-cast the receiver to work out
	// backwards which rung it had landed on. Get that order wrong and you get a bug where the mass is read as
	// an anchor while the force lands somewhere else, which is exactly what the partial-ragdoll root gate did.
	// One shared resolution makes that divergence structurally impossible.
	// ERopeEndpointKind and FRopeTetherEndpoint are the shared types in Core/RopeTractionTypes.h, and the
	// component caches the target and wielder results for the Wrapped frame so pullability, tether and the
	// basic pull all read the same ones.

	// Receiver resolution, shared by target and wielder. The ladder runs: skeletal free-ragdoll bone →
	// simulating primitive → simulating root → character → anchor.
	// (A partial ragdoll — a simulating bone held by a kinematic body above it — is not taken by rung 1 and
	// falls through to character or anchor.)
	//  - MeshComp is State.Mesh for the target and nullptr for the wielder, which skips the skeletal and
	//    primitive rungs and starts at the root.
	//  - A physics body's mass is maintained by the engine as collision volume × density; nothing to set.
	//  - A grounded character braces with a finite mass (Mass × GroundBraceFactor, the resistance of digging
	//    in), an airborne one is just Mass, and MOVE_None is an anchor.
	FRopeTetherEndpoint ResolveTetherEndpoint(USceneComponent* MeshComp, AActor* Owner, FName WrappedBone, float GroundBraceFactor)
	{
		FRopeTetherEndpoint Out;
		Out.Actor = Owner;

		// (1) Skeletal **free ragdoll** — a simulating bone articulated all the way to the root body. The
		// wrapped bone is promoted up the parent chain to the nearest *simulating* bone, which is what covers
		// a twist bone with no body of its own. The mass stored here is the **whole body's total** (GetMass),
		// because the coarse wielder/target split of the hard Chaos reaction and the pullability check would
		// otherwise read a bone body — a 3 kg forearm — as a light object. A compliant analytic solve refines
		// this default into the selected bone's translation-plus-rotation point Jacobian once the real
		// attachment point is known.
		//
		// A **partial ragdoll**, with a kinematic body in the parent chain above the simulating bone, is not
		// taken here and falls through. No amount of servoing the bone helps: the kinematic constraint absorbs
		// it and only the rope stretches. The only thing that can actually be dragged is the moving object —
		// the character rung, with CMC active — and failing that, nothing, because an anchor of infinite mass
		// is the physical truth. The wrapped bone's visual response, the arm coming along with the pull, still
		// follows.
		if (USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(MeshComp))
		{
			const FName SimBone = FindNearestSimulatingBone(Skel, WrappedBone);
			if (!SimBone.IsNone() && !IsSimBoneBoundToKinematic(Skel, SimBone))
			{
				Out.Kind = ERopeEndpointKind::SimBody;
				Out.Prim = Skel;
				Out.Bone = SimBone;
				// Whole-body mass. If that degenerates to 0 — no bodies created, say — fall back to the previous bone's body, then the component mass.
				const float WholeMass = static_cast<float>(Skel->GetMass());
				Out.Mass = (WholeMass > KINDA_SMALL_NUMBER) ? WholeMass : ResolveBodyMass(Skel, SimBone);
				return Out;
			}
		}
		// (2) The target component itself is a simulating primitive, such as a light physics prop.
		if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(MeshComp))
		{
			if (Prim->IsSimulatingPhysics())
			{
				Out.Kind = ERopeEndpointKind::SimBody;
				Out.Prim = Prim;
				Out.Mass = ResolveBodyMass(Prim, NAME_None);
				return Out;
			}
		}
		if (!Owner)
		{
			return Out; // None.
		}
		// (3) The owning actor's root primitive is simulating, as a physics actor. The wielder starts here, since its MeshComp is nullptr.
		if (UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(Owner->GetRootComponent()))
		{
			if (Root->IsSimulatingPhysics())
			{
				Out.Kind = ERopeEndpointKind::SimBody;
				Out.Prim = Root;
				Out.Mass = ResolveBodyMass(Root, NAME_None);
				return Out;
			}
		}
		// (4) A character. Under MOVE_None — the ragdoll setup convention — the movement cannot consume force, so it is treated as an anchor.
		if (UCharacterMovementComponent* Movement = GetForceConsumingMovement(Owner))
		{
			Out.Kind = ERopeEndpointKind::Character;
			Out.Movement = Movement;
			const float BraceScale = Movement->IsMovingOnGround() ? FMath::Max(GroundBraceFactor, 1.0f) : 1.0f;
			Out.Mass = Movement->Mass * BraceScale;
			return Out;
		}
		// (5) Static, kinematic, MOVE_None, or a non-simulating non-character → anchor, mass 0. Only a positional fallback can move it.
		Out.Kind = ERopeEndpointKind::Anchor;
		return Out;
	}

	// Turn the resolved receiver into the request the public extension hook (ApplyTractionToReceiver) reads.
	// Source, Direction and Amount are filled by each application path — the units differ per path, so see the request type's comment.
	FRopeTractionRequest MakeTractionRequest(const FRopeTetherEndpoint& Endpoint, ERopeTractionSource Source,
		const FVector& Dir, float Amount, float DeltaTime, bool bWielderSide)
	{
		FRopeTractionRequest Req;
		Req.Source = Source;
		Req.ReceiverKind = Endpoint.Kind;
		Req.Prim = Endpoint.Prim;
		Req.Bone = Endpoint.Bone;
		Req.Movement = Endpoint.Movement;
		Req.Actor = Endpoint.Actor;
		Req.Direction = Dir;
		Req.Amount = Amount;
		Req.DeltaTime = DeltaTime;
		Req.bWielderSide = bWielderSide;
		return Req;
	}

	// Effective inverse mass (w = 1 / effective mass) for the tether's automatic distribution. 0 is an anchor, meaning infinite mass.
	float EndpointInvMass(const FRopeTetherEndpoint& Endpoint)
	{
		return RopeTraction::InvMassFromMass(Endpoint.Mass);
	}

	bool BuildPointMassProperties(
		const FRopeTetherEndpoint& Endpoint,
		RopeTraction::FRopePointMassProperties& Out)
	{
		if (Endpoint.Kind != ERopeEndpointKind::SimBody ||
			!Endpoint.Prim)
		{
			return false;
		}
		FBodyInstance* Body =
			Endpoint.Prim->GetBodyInstance(Endpoint.Bone);
		if (!Body)
		{
			return false;
		}
		Out.Mass = static_cast<float>(Body->GetBodyMass());
		Out.InertiaTensor = Body->GetBodyInertiaTensor();
		Out.MassSpaceToWorld = Body->GetMassSpaceToWorldSpace();
		return Out.Mass > KINDA_SMALL_NUMBER;
	}

	float EndpointPointInvMass(
		const FRopeTetherEndpoint& Endpoint,
		const FVector& PointWorld,
		const FVector& DirectionWorld)
	{
		RopeTraction::FRopePointMassProperties Body;
		if (BuildPointMassProperties(Endpoint, Body))
		{
			const float PointInvMass =
				RopeTraction::ComputePointInverseMass(
					Body, PointWorld, DirectionWorld);
			if (PointInvMass > KINDA_SMALL_NUMBER)
			{
				return PointInvMass;
			}
		}
		return EndpointInvMass(Endpoint);
	}

	FVector EndpointVelocityAtPoint(
		const FRopeTetherEndpoint& Endpoint,
		const FVector& PointWorld)
	{
		switch (Endpoint.Kind)
		{
		case ERopeEndpointKind::SimBody:
			return Endpoint.Prim
				? Endpoint.Prim->GetPhysicsLinearVelocityAtPoint(
					PointWorld, Endpoint.Bone)
				: FVector::ZeroVector;
		case ERopeEndpointKind::Character:
			return Endpoint.Movement
				? Endpoint.Movement->Velocity
				: FVector::ZeroVector;
		default:
			return FVector::ZeroVector;
		}
	}


	// Shared context for applying the tether: both ends resolved, plus the extension gateway. Observation, the λ solve and storing the state stay with the component.
	struct FRopeTetherContext
	{
		const FRopeTetherEndpoint& Target;
		const FRopeTetherEndpoint& Wielder;
		float DeltaTime;
		// Extension gateway for applying to a receiver, delegating to the component's virtual. Returning true skips the built-in application.
		TFunctionRef<bool(const FRopeTractionRequest&)> TractionGate;
	};

	// Dispatch the application to the resolved receiver. Target and wielder share this one skeleton, which the
	// tether uses at both of its application points, and the callback already knows what it received, so
	// nothing has to be re-cast. Both the step and its return are this frame's axial ΔV in cm/s.
	//  - SimApply(Prim, Bone, Dir, DeltaV): a simulating physics body.
	//  - CharacterApply(Movement, Dir, DeltaV): a CMC-driven character.
	//  - AnchorApply(Actor, Dir, DeltaV): an anchor (static, MOVE_None) — does nothing, since w = 0 makes ΔV 0.
	float ApplyToTetherEndpoint(
		const FRopeTetherContext& Ctx, bool bWielderSide, const FVector& Dir, float Step,
		TFunctionRef<float(UPrimitiveComponent*, FName, const FVector&, float)> SimApply,
		TFunctionRef<void(UCharacterMovementComponent*, const FVector&, float)> CharacterApply,
		TFunctionRef<void(AActor*, const FVector&, float)> AnchorApply)
	{
		const FRopeTetherEndpoint& Endpoint = bWielderSide ? Ctx.Wielder : Ctx.Target;

		// Extension gateway (URopeComponent::ApplyTractionToReceiver): skips the built-in application when a
		// subclass handled it. All four of the tether's application points come through this skeleton, so this
		// one line covers the whole tether path.
		// The return follows Step's contract — a shortfall of 0 — matching the sim-body not-applied branch.
		if (Ctx.TractionGate(MakeTractionRequest(Endpoint, ERopeTractionSource::Tether, Dir, Step,
			Ctx.DeltaTime, bWielderSide)))
		{
			return Step;
		}

		switch (Endpoint.Kind)
		{
		case ERopeEndpointKind::SimBody:
			return SimApply(Endpoint.Prim, Endpoint.Bone, Dir, Step);
		case ERopeEndpointKind::Character:
			CharacterApply(Endpoint.Movement, Dir, Step);
			break;
		case ERopeEndpointKind::Anchor:
			AnchorApply(Endpoint.Actor, Dir, Step);
			break;
		default:
			break;
		}
		return Step; // sim-body not applied → shortfall 0.
	}

}

const FRopeResolvedWrappedEndpoints* URopeComponent::GetOrResolveWrappedEndpoints()
{
	if (WrappedEndpointCache.bValid)
	{
		return &WrappedEndpointCache;
	}

	USceneComponent* MeshComp = const_cast<USceneComponent*>(WrapController.State.Mesh.Get());
	if (!MeshComp || !PullDrive.LastPullSample.bValid)
	{
		return nullptr;
	}

	WrappedEndpointCache.Target = ResolveTetherEndpoint(
		MeshComp, MeshComp->GetOwner(), PullDrive.LastPullSample.Bone, HoldConfig.GroundBraceFactor);
	WrappedEndpointCache.Wielder = ResolveTetherEndpoint(
		nullptr, GetOwner(), NAME_None, HoldConfig.GroundBraceFactor);
	WrappedEndpointCache.TargetMesh = MeshComp;
	WrappedEndpointCache.TargetBone = PullDrive.LastPullSample.Bone;
	WrappedEndpointCache.bValid = true;
	return &WrappedEndpointCache;
}

float URopeComponent::ComputeWielderLengthReactionShare(
	const FRopeWielderMovementConstraint& Constraint) const
{
	if (!Constraint.IsValid())
	{
		return 1.0f;
	}

	USceneComponent* TargetComponent =
		const_cast<USceneComponent*>(Constraint.TargetComponent.Get());
	const FRopeTetherEndpoint Target = ResolveTetherEndpoint(
		TargetComponent,
		TargetComponent ? TargetComponent->GetOwner() : nullptr,
		Constraint.TargetBone,
		HoldConfig.GroundBraceFactor);
	const FRopeTetherEndpoint Wielder = ResolveTetherEndpoint(
		nullptr, GetOwner(), NAME_None, HoldConfig.GroundBraceFactor);
	if (Phase == ERopePhase::Wrapping &&
		Target.Kind != ERopeEndpointKind::SimBody)
	{
		// Wrapped PostPhysics applies the complementary analytic target share. Wrapping
		// deliberately does not run that traction stage yet, so a nonphysical target has
		// no same-frame receiver for its share; close relative velocity on the Wielder
		// instead until commit. SimBody targets still receive their share through Chaos.
		return 1.0f;
	}
	const float WTarget = EndpointInvMass(Target);
	const float WWielder = EndpointInvMass(Wielder);
	const float WSum = WTarget + WWielder;
	return WSum > KINDA_SMALL_NUMBER
		? FMath::Clamp(WWielder / WSum, 0.0f, 1.0f)
		: 1.0f;
}

float URopeComponent::ComputeWielderLengthPositionCorrectionShare(
	const FRopeWielderMovementConstraint& Constraint) const
{
	if (!Constraint.IsValid() ||
		HoldConfig.TetherCompliance > KINDA_SMALL_NUMBER)
	{
		return 1.0f;
	}

	USceneComponent* TargetComponent =
		const_cast<USceneComponent*>(Constraint.TargetComponent.Get());
	const FRopeTetherEndpoint Target = ResolveTetherEndpoint(
		TargetComponent,
		TargetComponent ? TargetComponent->GetOwner() : nullptr,
		Constraint.TargetBone,
		HoldConfig.GroundBraceFactor);
	return Target.Kind == ERopeEndpointKind::SimBody
		? ComputeWielderLengthReactionShare(Constraint)
		: 1.0f;
}

bool URopeComponent::IsWielderPhysicallySimulated() const
{
	// Rung 3 of the receiver ladder, reached because the wielder passes no MeshComp: the owner's root
	// primitive is simulating. A character whose bones ragdoll is deliberately not included — its root is
	// still the kinematic capsule, so the movement adapter remains the authority on that end.
	const FRopeTetherEndpoint Wielder = ResolveTetherEndpoint(
		nullptr, GetOwner(), NAME_None, HoldConfig.GroundBraceFactor);
	return Wielder.Kind == ERopeEndpointKind::SimBody && Wielder.Prim != nullptr;
}

void URopeComponent::PrepareWielderLengthConstraint(
	const FRopeWielderMovementConstraint& Constraint,
	const FVector& OutwardNormal,
	float RejectedSeparatingSpeed,
	float PositionViolation,
	bool bAtLimit,
	bool bHardProjectionApplied,
	float DeltaTime)
{
	if (!Constraint.IsValid() || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	if (bHardProjectionApplied &&
		bAtLimit &&
		(RejectedSeparatingSpeed > KINDA_SMALL_NUMBER ||
			PositionViolation > KINDA_SMALL_NUMBER))
	{
		LengthConstraintState.RecordWielderAttempt(
			GFrameCounter,
			Constraint.AnchorNode,
			PositionViolation,
			RejectedSeparatingSpeed,
			OutwardNormal,
			bAtLimit);
	}

	USceneComponent* TargetComponent =
		const_cast<USceneComponent*>(Constraint.TargetComponent.Get());
	AActor* TargetOwner = TargetComponent ? TargetComponent->GetOwner() : nullptr;
	const FRopeTetherEndpoint Target = ResolveTetherEndpoint(
		TargetComponent, TargetOwner, Constraint.TargetBone, HoldConfig.GroundBraceFactor);

	if (Target.Kind != ERopeEndpointKind::SimBody ||
		!Target.Prim ||
		HoldConfig.TetherCompliance > KINDA_SMALL_NUMBER)
	{
		// Runtime SimulatePhysics/compliance transitions are immediate. A compliant cable
		// uses the common analytic lambda solve (including its force cap) for every endpoint
		// type; Chaos is reserved for the uncapped, truly rigid substep constraint.
		TeardownPhysicalTether();
		return;
	}

	// The Wielder already kept only its generalized position-correction share. The
	// complementary target share must therefore be driven from that exact resulting hand
	// point. Reconstructing another offset from the original violation double-corrects the
	// position (full hand projection + target motion), creating artificial slack/tension
	// pulses and separating a zero-length hand/target pair.
	const FVector ChaosProxyWorld = GetComponentLocation();
	UpdatePhysicalTether(
		Target.Prim,
		Target.Bone,
		Constraint.AnchorWorld,
		ChaosProxyWorld,
		Constraint.MaxDistance,
		DeltaTime,
		// The hand point itself, so a simulating owner can carry the constraint directly.
		/*bCornerIsOwnerAttachPoint*/ true);
	if (PhysicalTetherConstraint)
	{
		PhysicalTetherPrePhysicsFrame = GFrameCounter;
	}
}

#pragma endregion Traction_Endpoint_And_Tether_Policies

#pragma region Tether_And_Pull_Application

FVector URopeComponent::ComputeSmoothedWielderDir(const FVector& Aim, const FVector& DirToAim, float DeltaTime, bool bInstantaneous)
{
	// Direction: along the first straight leg of the rope from the hand (node 0). The aim point (AimPos) sits
	// at the wall's corner, so the rope stays straight up to it. If the aim collapses onto the hand (chord
	// ≈ 0), it falls back to the reverse of anchor → aim, which is hand → anchor.
	const FVector HandPos = GetComponentLocation();
	FVector WielderDirRaw = Aim - HandPos;
	if (!WielderDirRaw.Normalize(KINDA_SMALL_NUMBER))
	{
		WielderDirRaw = -DirToAim;
	}
	// Airborne swing (bInstantaneous): the EMA is skipped and the instantaneous geometry is used. While the
	// orbit turns quickly the axis lags by ω·τ, on the order of ten degrees, and the tangential component of
	// that error brakes and accelerates the swing on every frame it fires, fighting AirControl.
	// What the EMA was for — damping the random walk of the servo top-up — leaves only ground corner noise in
	// a system built on a single λ impulse and a speed clamp. The state keeps being seeded raw meanwhile, so
	// re-entering the EMA on landing is continuous.
	if (bInstantaneous)
	{
		PullDrive.SmoothedWielderPullDir = WielderDirRaw;
		return WielderDirRaw;
	}
	// Direction EMA, using the same constant and the same function as the target side's SmoothedPullDir. Raw,
	// the direction bounces every frame on AimPos node noise, edge transitions and near-degenerate cases, and
	// the clamp and top-up then act on a different axis each time, growing the vector in a random walk.
	PullDrive.SmoothedWielderPullDir = RopeTraction::SmoothDirection(
		PullDrive.SmoothedWielderPullDir, WielderDirRaw, RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	return PullDrive.SmoothedWielderPullDir;
}

void URopeComponent::UpdateConstraintTether(float DeltaTime)
{
	// One logical C <= 0 material constraint with mutually exclusive application backends:
	//   1) simulated target -> Chaos constraint,
	//   2) hard-projected Wielder -> rejected-motion reaction,
	//   3) no movement adapter -> legacy analytic endpoint impulse.
	// Position authority and tension therefore observe the same live material boundary.
	LengthConstraintState.BeginFrame(DeltaTime);
	if (DeltaTime <= 1e-4f)
	{
		LengthConstraintState.bPrevGeometryValid = false;
		return;
	}

	FRopeWielderMovementConstraint LiveConstraint;
	const bool bHasLiveConstraint = BuildWielderMovementConstraint(LiveConstraint);
	const bool bHasPullSample = PullDrive.LastPullSample.bValid;
	if (!bHasLiveConstraint && !bHasPullSample)
	{
		LengthConstraintState.bPrevGeometryValid = false;
		return;
	}
	const int32 ConstraintAnchorNode = bHasLiveConstraint
		? LiveConstraint.AnchorNode
		: PullDrive.LastPullSample.AnchorNode;
	const FVector Anchor = bHasLiveConstraint
		? LiveConstraint.PivotWorld
		: PullDrive.LastPullSample.WorldPoint;
	const float MaterialLength = bHasLiveConstraint
		? LiveConstraint.MaxDistance
		: PullDrive.LastPullSample.FreeRestLen;
	const float RequiredLength = bHasLiveConstraint
		? static_cast<float>(FVector::Distance(GetComponentLocation(), Anchor))
		: PullDrive.LastPullSample.PathChordLen;
	float C = RequiredLength - MaterialLength;
	LengthConstraintState.LastViolation = FMath::Max(C, 0.0f);
	const bool bHardWielderAttempt =
		LengthConstraintState.HasWielderAttempt(GFrameCounter, ConstraintAnchorNode);

	// The reel, the material length and a kinematic anchor's rate are all sampled from the same live geometry.
	// The activation tolerance must never add physical cable length.
	float RestRate = 0.0f;
	if (LengthConstraintState.bPrevGeometryValid &&
		LengthConstraintState.PrevAnchorNode == ConstraintAnchorNode)
	{
		RestRate =
			(MaterialLength - LengthConstraintState.PrevMaterialLength) / DeltaTime;
		const FVector RawAnchorVel =
			(Anchor - LengthConstraintState.PrevAnchorWorldPoint) / DeltaTime;
		LengthConstraintState.SmoothedAnchorPointVelocity = FMath::Lerp(
			LengthConstraintState.SmoothedAnchorPointVelocity,
			RawAnchorVel,
			RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
		// Ground-truth velocity of the hand point (the wielder's tip). An anchor-kind wielder — a kinematic
		// carrier such as a helicopter or a moving platform — reports no velocity, so it is filled in by finite
		// difference. The wielder's mirror of the target side's SmoothedAnchorPointVelocity.
		const FVector RawWielderVel =
			(GetComponentLocation() - LengthConstraintState.PrevWielderWorldPoint) / DeltaTime;
		LengthConstraintState.SmoothedWielderPointVelocity = FMath::Lerp(
			LengthConstraintState.SmoothedWielderPointVelocity,
			RawWielderVel,
			RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	}
	LengthConstraintState.PrevMaterialLength = MaterialLength;
	LengthConstraintState.PrevAnchorWorldPoint = Anchor;
	LengthConstraintState.PrevWielderWorldPoint = GetComponentLocation();
	LengthConstraintState.PrevAnchorNode = ConstraintAnchorNode;
	LengthConstraintState.bPrevGeometryValid = true;

	FRopeResolvedWrappedEndpoints LiveEndpoints;
	const FRopeResolvedWrappedEndpoints* Endpoints = GetOrResolveWrappedEndpoints();
	if (!Endpoints && bHasLiveConstraint)
	{
		// Pull sampling needs at least one hand-side segment and is therefore invalid for a
		// legitimate node-0 constraint (and during short observation gaps). Resolve the same
		// live binding directly so hard projection/Chaos can still publish reaction tension.
		USceneComponent* TargetComponent =
			const_cast<USceneComponent*>(LiveConstraint.TargetComponent.Get());
		LiveEndpoints.Target = ResolveTetherEndpoint(
			TargetComponent,
			TargetComponent ? TargetComponent->GetOwner() : nullptr,
			LiveConstraint.TargetBone,
			HoldConfig.GroundBraceFactor);
		LiveEndpoints.Wielder = ResolveTetherEndpoint(
			nullptr, GetOwner(), NAME_None, HoldConfig.GroundBraceFactor);
		LiveEndpoints.TargetMesh = TargetComponent;
		LiveEndpoints.TargetBone = LiveConstraint.TargetBone;
		Endpoints = &LiveEndpoints;
	}
	if (!Endpoints)
	{
		return;
	}
	USceneComponent* MeshComp = Endpoints->TargetMesh.Get();
	if (!MeshComp)
	{
		return;
	}
	const bool bSelfWrap =
		(GetOwner() != nullptr && MeshComp->GetOwner() == GetOwner());
	const float ResolvedInvMassTarget =
		EndpointInvMass(Endpoints->Target);
	const float ResolvedInvMassWielder =
		bSelfWrap ? 0.0f : EndpointInvMass(Endpoints->Wielder);
	const float ResolvedWSum =
		ResolvedInvMassTarget + ResolvedInvMassWielder;
	// Chaos needs a share before the rope directions/point Jacobians below are built.
	// Analytic paths replace this with their exact point-mass split before solving.
	PullDrive.LastTargetShare =
		(ResolvedWSum > KINDA_SMALL_NUMBER)
			? (ResolvedInvMassTarget / ResolvedWSum)
			: 0.0f;

	const bool bPhysicalTarget = (Endpoints->Target.Kind == ERopeEndpointKind::SimBody
		&& Endpoints->Target.Prim != nullptr);
	const bool bUseChaosBackend =
		bPhysicalTarget &&
		HoldConfig.TetherCompliance <= KINDA_SMALL_NUMBER;

	const FVector Aim = bHasPullSample
		? PullDrive.LastPullSample.AimPos
		: GetComponentLocation();
	if (bUseChaosBackend)
	{
		// Simulated targets are exclusively owned by Chaos. Never add the analytic
		// rejected-motion impulse on top of this constraint.
		LengthConstraintState.Backend = ERopeLengthConstraintBackend::Chaos;
		const float LegRest = bHasLiveConstraint
			? MaterialLength
			: FMath::Max(0.0f,
				(static_cast<float>(PullDrive.LastPullSample.AnchorNode) -
					PullDrive.LastPullSample.AimNodeF) * Sim.SegmentLength);
		const bool bDrivenByWielderThisFrame =
			PhysicalTetherPrePhysicsFrame == GFrameCounter &&
			PhysicalTetherTarget.Get() == Endpoints->Target.Prim &&
			PhysicalTetherBone == Endpoints->Target.Bone;
		if (bDrivenByWielderThisFrame)
		{
			// PrePhysics already wrote the attempted, unclamped hand point and material-length
			// limit. Do not replace it with the look-ahead Aim contract after Chaos.
			SamplePhysicalTetherForce(DeltaTime);
		}
		else
		{
			// Legacy/custom-mover fallback when no Wielder drove the authoritative path.
			// A live material constraint always uses the actual hand point. Pull Aim is a
			// look-ahead node and can sit deep inside a curved/sagging leg; pairing it with
			// the full hand-to-anchor rest length silently creates unrelated slack.
			const FVector ProxyWorld = bHasLiveConstraint
				? GetComponentLocation()
				: Aim;
			UpdatePhysicalTether(
				Endpoints->Target.Prim,
				Endpoints->Target.Bone,
				Anchor,
				ProxyWorld,
				LegRest,
				DeltaTime,
				// Only the live path's corner is the hand point. Aim is a look-ahead node standing in for an
				// external pulley, so it must stay on the kinematic proxy even for a simulating owner.
				/*bCornerIsOwnerAttachPoint*/ bHasLiveConstraint);
		}
		return;
	}
	TeardownPhysicalTether();

	// Only the fallback that still derives C from delayed/deformable particle chords needs
	// the historical partial-stretch contamination guard. Live material geometry and hard
	// movement attempts never depend on XPBD SegmentTension.
	const bool bLegacyPathLoaded = bHasPullSample && RopeTraction::EvaluateTautGate(
		PullDrive.LastPullSample.MinFreeTension,
		HoldConfig.TautMinTension,
		TautMinTensionReleaseRatioConst,
		PullDrive.bChainTaut);
	if (!bHasLiveConstraint && !bHardWielderAttempt &&
		(!PullDrive.bChainTaut || !bLegacyPathLoaded))
	{
		return;
	}

	// The live constraint uses the exact hand/anchor normal. Legacy paths retain their
	// smoothed look-ahead direction for corner noise.
	const FVector Span = (Aim - Anchor).GetSafeNormal();
	const FVector CurrentLiveOutward =
		(GetComponentLocation() - Anchor).GetSafeNormal();
	const FVector LiveOutward =
		!CurrentLiveOutward.IsNearlyZero()
			? CurrentLiveOutward
			: (bHardWielderAttempt
				? LengthConstraintState.WielderAttemptOutwardNormal
				: FVector::ZeroVector);
	const FVector LegacyTargetDir =
		PullDrive.SmoothedPullDir.IsNearlyZero() ? Span : PullDrive.SmoothedPullDir;
	const FVector DirTarget =
		(bHasLiveConstraint && !LiveOutward.IsNearlyZero()) ? LiveOutward : LegacyTargetDir;
	if (DirTarget.IsNearlyZero())
	{
		return; // degenerate(aiming=anchor) — direction cannot be defined.
	}
	// Skip the wielder's direction EMA during an airborne swing and use the instantaneous geometry. While the
	// orbit turns quickly the axis lags by a fraction of a degree to a few degrees, and the tangential
	// component of that error brakes and accelerates the swing on every frame it fires — the moment where an
	// AirControl boost stops doing anything. On the ground the EMA stays, because corner and node noise are large.
	const bool bWielderAirborne = (Endpoints->Wielder.Kind == ERopeEndpointKind::Character
		&& Endpoints->Wielder.Movement && Endpoints->Wielder.Movement->IsFalling());
	const FVector DirWielder =
		(bHasLiveConstraint && !LiveOutward.IsNearlyZero())
			? -LiveOutward
			: ComputeSmoothedWielderDir(Aim, DirTarget, DeltaTime, bWielderAirborne);
	if (DirWielder.IsNearlyZero())
	{
		return;
	}

	// Self-wrap (owner == target): both ends are the same body, so the impulse pair would cancel itself. The
	// wielder end is treated as an anchor (w = 0) and only the target end moves, matching the legacy special case.
	// Opening speed s = −(vT·dT + vW·dW) − dRest/dt. Each end's velocity is measured at the same rung the
	// receiver resolution chose, so a moving anchor's cruise — a dragon's, say — rides on vT and is followed
	// without a separate feed-forward. That replaces the legacy anchor velocity EMA.
	const FVector TargetPoint = Anchor;
	const FVector WielderPoint = GetComponentLocation();
	const float InvMassTarget = EndpointPointInvMass(
		Endpoints->Target, TargetPoint, DirTarget);
	const float InvMassWielder = bSelfWrap
		? 0.0f
		: EndpointPointInvMass(
			Endpoints->Wielder, WielderPoint, DirWielder);
	const float WSum = InvMassTarget + InvMassWielder;
	// Analytic material response and actual point impulse use the same Jacobian. In
	// particular an off-COM wrap includes rotational inverse mass instead of pretending
	// the selected bone's COM translation is the rope attachment response.
	PullDrive.LastTargetShare =
		(WSum > KINDA_SMALL_NUMBER)
			? (InvMassTarget / WSum)
			: 0.0f;

	// ---- Target hard projection (kinematic character carry) ----
	// When the wielder's end has infinite mass — an anchor, such as a helicopter carrying the rope — and the
	// target is a CMC character, applying λ as a velocity is not enough: the positional error is capped by
	// TetherMaxBiasSpeed, so a carrier faster than that cap stretches the rope without limit.
	// This is the target-side mirror of the wielder's hard projection (ConstrainWielderLocation): move the
	// capsule along the current radial direction by the shortfall, removing the positional error within the
	// same frame. If a wall blocks it, whatever is left stays as C and is picked up by λ and the observation.
	// With finite mass at both ends the λ pair already owns the distribution, so this does not fire and cannot
	// double-correct. Elastic mode (TetherCompliance > 0) is excluded, since the stretch there is deliberate.
	if (HoldConfig.bEnforceTargetLengthConstraint
		&& HoldConfig.TetherCompliance <= KINDA_SMALL_NUMBER
		&& bHasLiveConstraint && !bSelfWrap
		&& Endpoints->Target.Kind == ERopeEndpointKind::Character
		&& Endpoints->Wielder.Kind == ERopeEndpointKind::Anchor
		&& Endpoints->Target.Actor && Endpoints->Target.Movement
		&& !LiveOutward.IsNearlyZero()
		&& C > FMath::Max(HoldConfig.LengthConstraintActivationSlop, 0.0f))
	{
		AActor* TargetActor = Endpoints->Target.Actor;
		const FVector OldLoc = TargetActor->GetActorLocation();
		TargetActor->SetActorLocation(OldLoc + LiveOutward * C, /*bSweep*/ true);
		const FVector Applied = TargetActor->GetActorLocation() - OldLoc;
		C = FMath::Max(
			C - static_cast<float>(FVector::DotProduct(Applied, LiveOutward)), 0.0f);
		LengthConstraintState.LastViolation = C;
		// A grounded character lifted with real speed has to reach Falling before Walking's floor snap and Z
		// removal cancel it out — the same convention Launch uses. A horizontal pull (Applied.Z ≈ 0) stays
		// under the threshold, so dragging along the ground behaves as before.
		if (Endpoints->Target.Movement->IsMovingOnGround()
			&& Applied.Z > TetherLiftLaunchSpeedConst * DeltaTime)
		{
			Endpoints->Target.Movement->SetMovementMode(MOVE_Falling);
		}
	}

	// An anchor-kind object — static, kinematic or animated — reports no physical velocity, so its anchor
	// point is filled from a ground-truth EMA. That way a moving object being towed is followed through the
	// separation offset regardless of the bias cap, while a still anchor contributes ≈ 0 and changes nothing.
	// The second safety net is ClampInjectedVelocity(TetherMaxSpeed) on the application side.
	// A SimBody instead reads vCOM + ω×r at the rope's attachment point, and a character reads its CMC velocity.
	const FVector VelTarget = (Endpoints->Target.Kind == ERopeEndpointKind::Anchor)
			? (bHasLiveConstraint
				? LiveConstraint.PivotVelocity
				: LengthConstraintState.SmoothedAnchorPointVelocity)
			: EndpointVelocityAtPoint(
				Endpoints->Target, TargetPoint);
	const float SepTarget = -static_cast<float>(FVector::DotProduct(VelTarget, DirTarget));
	// An anchor-kind wielder — a kinematic carrier — uses the same ground-truth EMA symmetrically: the
	// carrier's departure speed enters λ through the separation offset and is followed regardless of
	// TetherMaxBiasSpeed. A still owner contributes ≈ 0 and changes nothing.
	const FVector VelWielder = (Endpoints->Wielder.Kind == ERopeEndpointKind::Anchor)
		? LengthConstraintState.SmoothedWielderPointVelocity
		: EndpointVelocityAtPoint(
			Endpoints->Wielder, WielderPoint);
	const float SepWielder = bSelfWrap ? 0.0f
		: -static_cast<float>(FVector::DotProduct(VelWielder, DirWielder));
	const float SepSpeed =
		RopeMovementConstraint::ComputeConstraintSeparatingSpeed(
			bHardWielderAttempt,
			LengthConstraintState.WielderAttemptSeparatingSpeed,
			SepTarget + SepWielder,
			RestRate);

	RopeLengthConstraint::FInput In;
	LengthConstraintState.Backend = bHardWielderAttempt
		? ERopeLengthConstraintBackend::HardReaction
		: ERopeLengthConstraintBackend::Analytic;
	// C is sampled after the hard projection, so its rejected position is already gone and
	// normal hard frames see C=0. Any positive value left here is new live error (for example
	// an external target that moved later in PrePhysics) and must not be hidden by the attempt
	// stamp. The original rejected distance still enters only through its recorded speed.
	In.Violation = C;
	In.SeparatingSpeed = SepSpeed;
	In.EffectiveInverseMass = WSum;
	In.ActivationSlop = FMath::Max(HoldConfig.LengthConstraintActivationSlop, 0.0f);
	// The projection's original error is absent from live C. A later-moving external
	// target can create a new positive residual in the same frame, and that residual needs
	// normal position bias even though a hard attempt stamp also exists.
	In.SettleAlpha =
		(bHardWielderAttempt && C <= KINDA_SMALL_NUMBER)
			? 0.0f
			: RopeTraction::ExpSmoothAlpha(
				HoldConfig.TetherSettleTime, DeltaTime);
	In.MaxBiasSpeed = FMath::Max(HoldConfig.TetherMaxBiasSpeed, 0.0f);
	In.Compliance = HoldConfig.TetherCompliance;
	// A strictly inextensible constraint cannot also cap its reaction under arbitrary
	// kinematic input: exceeding a cap must either stretch, slip, or break. Rigid mode
	// preserves length and reports the full reaction (MaxTetherTension is then an overload
	// threshold); compliant mode may yield and therefore uses the configured force cap.
	In.MaxTension = HoldConfig.TetherCompliance > KINDA_SMALL_NUMBER
		? HoldConfig.MaxTetherTension
		: 0.0f;
	const RopeLengthConstraint::FResult SolveResult =
		RopeLengthConstraint::Solve(In, DeltaTime);
	const float Lambda = SolveResult.Lambda;
	LengthConstraintState.LastLambda = Lambda;

	// Effective distribution share, which the wielder gate and the debugger both read, is the inverse-mass ratio. Set to this frame's value whether or not λ fired.
	if (Lambda <= 0.0f)
	{
		return; // Already approaching sufficiently or both ends are anchored.
	}

	// ---- Applied: ΔV = λ × w (cm/s) at each end, along that end's leg direction ----
	// Every path touches only the rope-axis component, plus the orthogonal damping, so swing and gravity
	// survive, and the resulting speed is clamped a second time at TetherMaxSpeed.
	// A rigid simulating target belongs exclusively to Chaos; a compliant simulating target and a simulating
	// wielder take a real physics impulse of ΔV × effective mass. It goes through the existing dispatch
	// skeleton (ApplyToTetherEndpoint), so the extension gateway sees it identically
	// (ApplyTractionToReceiver, with Amount as ΔV in cm/s).
	const float SpeedCap = FMath::Max(HoldConfig.TetherMaxSpeed, 0.0f);
	// Orthogonal damping needs a dt correction: the setting is a per-frame rate at 60fps, so the effective
	// rate is 1 − (1 − d)^(dt·60). Using the per-frame ratio directly made the damping stronger the higher the
	// frame rate — frame-rate-dependent physics.
	const float PerpDampCfg = FMath::Clamp(HoldConfig.TetherPerpDamping, 0.0f, 1.0f);
	const float PerpDamp = (PerpDampCfg > 0.0f && PerpDampCfg < 1.0f)
		? (1.0f - FMath::Pow(1.0f - PerpDampCfg, DeltaTime * 60.0f))
		: PerpDampCfg;
	auto ApplySimBody = [&](
		UPrimitiveComponent* Prim,
		FName BoneName,
		const FVector& Dir,
		const FVector& PointWorld,
		float DeltaV,
		float PointInvMass) -> float
	{
		// A compliant target, or a physics wielder. ΔV = λ·w was computed with the same attachment-point
		// Jacobian the solver used, so the real rope impulse is exactly J = λ·d = (ΔV/w)·d.
		// AddImpulseAtLocation produces the ω×r and the torque along with it, and the observation reads the
		// velocity at that same point.
		// (The orthogonal damping covers every axis. Excluding the vertical component is a possible future
		// option: gravity's fall settles into equilibrium with the damping, and the weightless look of hanging
		// at a low terminal speed of about g·dt/rate is countered by tuning this magnitude instead.)
		if (PointInvMass <= KINDA_SMALL_NUMBER)
		{
			return DeltaV;
		}
		const FVector CurVel =
			Prim->GetPhysicsLinearVelocityAtPoint(
				PointWorld, BoneName);
		FVector PointImpulse =
			Dir * (DeltaV / PointInvMass);

		FRopeTetherEndpoint SimEndpoint;
		SimEndpoint.Kind = ERopeEndpointKind::SimBody;
		SimEndpoint.Prim = Prim;
		SimEndpoint.Bone = BoneName;
		RopeTraction::FRopePointMassProperties Body;
		if (BuildPointMassProperties(SimEndpoint, Body))
		{
			if (PerpDamp > 0.0f)
			{
				const FVector PerpVel =
					CurVel -
					Dir * static_cast<float>(
						FVector::DotProduct(CurVel, Dir));
				const float PerpSpeed =
					static_cast<float>(PerpVel.Size());
				if (PerpSpeed > KINDA_SMALL_NUMBER)
				{
					const FVector PerpDir =
						-PerpVel / PerpSpeed;
					const float PerpInvMass =
						RopeTraction::ComputePointInverseMass(
							Body, PointWorld, PerpDir);
					if (PerpInvMass > KINDA_SMALL_NUMBER)
					{
						PointImpulse +=
							PerpDir *
							(PerpSpeed * PerpDamp /
								PerpInvMass);
					}
				}
			}

			// Predict the actual point response (including angular motion), then scale the
			// whole injected impulse if the gameplay safety cap would be exceeded.
			const FVector PredictedDelta =
				RopeTraction::ComputePointVelocityDelta(
					Body, PointWorld, PointImpulse);
			const FVector SafeVelocity =
				RopeTraction::ClampInjectedVelocity(
					CurVel + PredictedDelta,
					CurVel,
					SpeedCap);
			const FVector SafeDelta = SafeVelocity - CurVel;
			const float PredictedSizeSq =
				static_cast<float>(PredictedDelta.SizeSquared());
			if (PredictedSizeSq > SMALL_NUMBER)
			{
				const float ImpulseScale = FMath::Clamp(
					static_cast<float>(
						FVector::DotProduct(
							SafeDelta, PredictedDelta)) /
						PredictedSizeSq,
					0.0f,
					1.0f);
				PointImpulse *= ImpulseScale;
			}
			Prim->AddImpulseAtLocation(
				PointImpulse, PointWorld, BoneName);
		}
		else
		{
			// Defensive fallback for an endpoint whose body vanished after resolution.
			const FVector NewVel =
				RopeTraction::ClampInjectedVelocity(
					CurVel + Dir * DeltaV,
					CurVel,
					SpeedCap);
			Prim->AddImpulse(
				(NewVel - CurVel) / PointInvMass,
				BoneName,
				/*bVelChange*/ false);
		}
		return DeltaV;
	};
	auto ApplyCharacter = [&](UCharacterMovementComponent* Movement, const FVector& Dir, float DeltaV)
	{
		// CMC: add velocity directly, which is the contract for taking effect this frame. λ's position
		// recovery term is bounded by MaxBiasSpeed, so there is no excess injection and no separate ledger or
		// slack break is needed. (Projecting horizontally while grounded is a possible future option; for now
		// the whole axis is injected.)
		const FVector OldVel = Movement->Velocity;
		Movement->Velocity = RopeTraction::ClampInjectedVelocity(OldVel + Dir * DeltaV, OldVel, SpeedCap);
		// A grounded character whose upward injection passes the threshold becomes Falling, because Walking
		// would convert the Z velocity into a floor constraint on the next tick and discard it, disabling the
		// lift. Switching mode with the same convention Launch uses is what lets the injection survive. A
		// horizontal pull passes through untouched, since its Z injection is ≈ 0.
		if (static_cast<float>(Movement->Velocity.Z - OldVel.Z) > TetherLiftLaunchSpeedConst
			&& Movement->IsMovingOnGround())
		{
			Movement->SetMovementMode(MOVE_Falling);
		}
	};

	auto TractionGate = [this](const FRopeTractionRequest& Req) { return ApplyTractionToReceiver(Req); };
	const FRopeTetherContext Ctx{ Endpoints->Target, Endpoints->Wielder, DeltaTime, TractionGate };

	const float DvTarget = Lambda * InvMassTarget;
	if (DvTarget > KINDA_SMALL_NUMBER)
	{
		ApplyToTetherEndpoint(Ctx, /*bWielderSide*/ false, DirTarget, DvTarget,
			[&](UPrimitiveComponent* P, FName B, const FVector& D, float S)
			{
				return ApplySimBody(
					P, B, D, TargetPoint, S, InvMassTarget);
			},
			[&](UCharacterMovementComponent* M, const FVector& D, float S) { ApplyCharacter(M, D, S); },
			[&](AActor*, const FVector&, float) { /* Anchor = w 0, so ΔV is also 0 — unreachable*/ });
	}
	// The hard movement adapter already removed this exact outward velocity from the
	// Wielder. Applying its analytic share again would create an inward rebound.
	const float DvWielder =
		(bSelfWrap || bHardWielderAttempt) ? 0.0f : Lambda * InvMassWielder;
	if (DvWielder > KINDA_SMALL_NUMBER)
	{
		ApplyToTetherEndpoint(Ctx, /*bWielderSide*/ true, DirWielder, DvWielder,
			[&](UPrimitiveComponent* P, FName B, const FVector& D, float S)
			{
				return ApplySimBody(
					P, B, D, WielderPoint, S, InvMassWielder);
			},
			[&](UCharacterMovementComponent* M, const FVector& D, float S) { ApplyCharacter(M, D, S); },
			[&](AActor*, const FVector&, float) { /* Anchor no operation*/ });
	}
}

void URopeComponent::UpdatePhysicalTether(UPrimitiveComponent* TargetPrim, FName Bone,
	const FVector& AnchorWorld, const FVector& CornerWorld, float LegRestLen, float DeltaTime,
	bool bCornerIsOwnerAttachPoint)
{
	// The ragdoll half of the constraint tether, as an engine physics constraint: the hand side at the corner
	// tied to an anchor point on the target's body, with a spherical limit at the leg's rest length.
	// A per-frame game-thread velocity impulse cannot do this job. On a jointed body it is trapped between a
	// whole-body-sized kick, which runs away, and a bone-sized λ, which collapses the traction force; and
	// under an airborne load such as a suspended prop it cannot stop the floating and the pendulum pumping
	// either. A Chaos constraint is released **together with** gravity, the joints and ground contact inside
	// the substep, which gives whole-body traction without runaway and genuine pendulum behaviour — the
	// standard way to hang a load from a hand.
	// The constraint frame is held in the anchor's body-local space (a bone's transform for a skeletal target,
	// following the same tip-lever convention), which is how the engine resolves the alignment torque.
	AActor* Owner = GetOwner();
	if (!Owner || !TargetPrim)
	{
		return;
	}

	// Which body carries the hand side (Frame1). The kinematic proxy below is an infinite mass: the target is
	// drawn in and the hand side feels nothing. That is right for a character, whose movement adapter applies
	// its own reaction, and right for a corner that stands in for an external pulley — but wrong when the
	// rope's owner is itself simulating, because then nothing absorbs the reaction and the rope can never drag
	// the owner. A simulating owner is therefore bound directly, and Chaos splits the reaction across the two
	// bodies by inverse mass inside the same substep.
	// It needs the corner to be a point genuinely fixed in that body, which is the rope's own attachment point.
	// The pull-sample fallback places the corner at a look-ahead node on the rope instead, so it stays on the
	// proxy. Nor may the owner's body *be* the target: a self-wrap would constrain one body to itself.
	UPrimitiveComponent* WielderPrim = nullptr;
	FName WielderBone = NAME_None;
	if (bCornerIsOwnerAttachPoint)
	{
		const FRopeTetherEndpoint Wielder = ResolveTetherEndpoint(
			nullptr, Owner, NAME_None, HoldConfig.GroundBraceFactor);
		if (Wielder.Kind == ERopeEndpointKind::SimBody && Wielder.Prim && Wielder.Prim != TargetPrim)
		{
			WielderPrim = Wielder.Prim;
			WielderBone = Wielder.Bone;
		}
	}

	// Rebuild it when the target or bone changed — a promotion, or a re-wrap of the anchor — or when the hand
	// side swapped carriers, which is what a runtime SimulatePhysics toggle on the owner looks like from here.
	if (PhysicalTetherConstraint
		&& (PhysicalTetherTarget.Get() != TargetPrim || PhysicalTetherBone != Bone
			|| PhysicalTetherWielder.Get() != WielderPrim || PhysicalTetherWielderBone != WielderBone))
	{
		TeardownPhysicalTether();
	}

	// The target body's current local anchor (constraint Frame2): a bone transform for a skeletal target,
	// following the tip-lever convention, or the component transform for a component body. It is pinned at
	// creation, so if the wrap anchor is relocated within the same (target, bone) — a promotion, or a seed
	// joining — the rope's anchor and the constraint's anchor diverge and leave a permanent violation, which
	// reads as vibration. Past a drift threshold the constraint is torn down and rebuilt immediately by the
	// creation block below. It does not fire under normal conditions.
	const USkeletalMeshComponent* SkelBody = Cast<USkeletalMeshComponent>(TargetPrim);
	FTransform BodyTM = TargetPrim->GetComponentTransform();
	if (SkelBody && !Bone.IsNone())
	{
		const int32 BoneIndex = SkelBody->GetBoneIndex(Bone);
		if (BoneIndex != INDEX_NONE)
		{
			BodyTM = SkelBody->GetBoneTransform(BoneIndex);
		}
	}
	const FVector AnchorLocal = BodyTM.InverseTransformPosition(AnchorWorld);
	// The instantaneous local anchor shakes hard every frame on a fast ragdoll, because the rope's sim is a
	// lagging GPU mirror while the bone transform is current. A 5 cm guard on that instantaneous value
	// rebuilds every frame even with no real relocation — thrashing, so the constraint never gets a warm start
	// and the shaking becomes real (measured at 70% rebuild rate and 0 strength 89% of the time).
	// So the local anchor is smoothed with an EMA and the guard reads that instead: the lag noise averages out
	// and the smoothed value stays near the pinned anchor, while a sustained relocation — a seed joining, a
	// promotion — moves the average past the threshold. The threshold is 10 cm to leave headroom.
	if (PhysicalTetherConstraint)
	{
		const float SmoothAlpha = RopeTraction::ExpSmoothAlpha(0.12f, DeltaTime); // ≈0.12s time constant.
		PhysicalTetherSmoothedAnchorLocal = FMath::Lerp(PhysicalTetherSmoothedAnchorLocal, AnchorLocal, SmoothAlpha);
		if (FVector::DistSquared(PhysicalTetherSmoothedAnchorLocal, PhysicalTetherAnchorLocal) > FMath::Square(10.0f))
		{
			TeardownPhysicalTether();
		}
	}

	// The hand attachment in the owner actor's space, for the directly bound case. This reads the component
	// transform rather than the lagging Sim mirror, and the rope is rigidly attached to its owner, so it is
	// constant frame to frame and needs no smoothing — but re-attaching the rope to a different socket during
	// a wrap does relocate it for real, and Frame1 was pinned at creation, so it is guarded all the same.
	const FVector WielderActorLocal = WielderPrim
		? Owner->GetActorTransform().InverseTransformPosition(CornerWorld)
		: FVector::ZeroVector;
	if (PhysicalTetherConstraint && WielderPrim
		&& FVector::DistSquared(WielderActorLocal, PhysicalTetherWielderActorLocal) > FMath::Square(10.0f))
	{
		TeardownPhysicalTether();
	}

	if (WielderPrim)
	{
		// Directly bound: there is no corner to carry, so any proxy left over from a previous binding goes.
		// The carrier change above already tore the constraint down, so this never orphans a live one.
		if (PhysicalTetherProxy)
		{
			PhysicalTetherProxy->DestroyComponent();
			PhysicalTetherProxy = nullptr;
		}
	}
	else
	{
		if (!PhysicalTetherProxy)
		{
			PhysicalTetherProxy = NewObject<USphereComponent>(Owner,
				MakeUniqueObjectName(Owner, USphereComponent::StaticClass(), TEXT("RopeTetherProxy")));
			PhysicalTetherProxy->SetupAttachment(this);
			PhysicalTetherProxy->SetAbsolute(true, true, true); // World layout (regardless of rope component transform).
			PhysicalTetherProxy->InitSphereRadius(4.0f);
			// It needs a body, as one side of the constraint, but no collision and no queries — PhysicsOnly with
			// every channel ignored.
			// Why queries must stay off: USphereComponent's default object type is WorldDynamic, and a channel
			// response of Ignore only affects **channel** queries, since an object-type query returns any shape
			// whose object type matches. With queries on, URopeStaticBodyProvider's OverlapMultiByObjectType scan
			// would pick this proxy up, turn it into a push-out collider that follows the corner every frame, and
			// shove this rope's own wrap nodes around.
			PhysicalTetherProxy->SetCollisionEnabled(ECollisionEnabled::PhysicsOnly);
			PhysicalTetherProxy->SetCollisionResponseToAllChannels(ECR_Ignore);
			PhysicalTetherProxy->SetSimulatePhysics(false); // Kinematic — Move to corner every frame.
			PhysicalTetherProxy->SetHiddenInGame(true);
			PhysicalTetherProxy->RegisterComponent();
		}
		// Kinematic movement — Chaos reads the movement velocity and pulls the constraint with it, so the corner and the hand are tracked.
		PhysicalTetherProxy->SetWorldLocation(CornerWorld);
	}

	if (!PhysicalTetherConstraint)
	{
		PhysicalTetherConstraint = NewObject<UPhysicsConstraintComponent>(Owner,
			MakeUniqueObjectName(Owner, UPhysicsConstraintComponent::StaticClass(), TEXT("RopeTetherConstraint")));
		// The constraint component rides its Frame1 carrier: the proxy, or this rope component, whose own
		// location *is* the attachment point in the directly bound case. Scale is forced absolute because
		// UpdateConstraintFrames divides the derived frame origin by the component's scale, and a designer
		// scaling the rope must not move the constraint's anchor with it.
		PhysicalTetherConstraint->SetupAttachment(
			WielderPrim ? static_cast<USceneComponent*>(this) : PhysicalTetherProxy);
		PhysicalTetherConstraint->SetAbsolute(false, false, true);
		PhysicalTetherConstraint->RegisterComponent();
		PhysicalTetherConstraint->SetWorldLocation(CornerWorld);
		PhysicalTetherConstraint->SetDisableCollision(false);
		// A single body plus positional projection can snap/rebound at the moving limit, so
		// component bodies keep projection disabled. A directly bound owner keeps it disabled too:
		// projection closes the error by teleporting a body, which on something as heavy as a
		// vehicle is a visible jump rather than a correction. This backend is exclusively the
		// inextensible path and must always be a hard Chaos limit. Positive compliance is
		// owned by the common analytic material solver and never reaches this function.
		// The profile must be configured before SetConstrainedComponents initializes Chaos.
		FConstraintProfileProperties& Profile =
			PhysicalTetherConstraint->ConstraintInstance.ProfileInstance;
		if (!SkelBody || WielderPrim)
		{
			Profile.bEnableProjection = false;
		}
		Profile.LinearLimit.bSoftConstraint = false;
		Profile.LinearLimit.Stiffness = 0.0f;
		Profile.LinearLimit.Damping = 0.0f;
		Profile.LinearLimit.Restitution = 0.0f;
		PhysicalTetherConstraint->SetConstrainedComponents(
			WielderPrim ? WielderPrim : static_cast<UPrimitiveComponent*>(PhysicalTetherProxy),
			WielderPrim ? WielderBone : NAME_None,
			TargetPrim, Bone);
		// Constraint frame origins: the hand side is the corner, and the target side is the anchor in
		// body-local space (AnchorLocal above — the wrap freezes bone-local anchors under the same convention,
		// tip lever included). The distance limit spans those two points.
		// SetConstrainedComponents has just derived both frames from this component's world position, which is
		// the corner. Frame1 is therefore already the corner expressed in its carrier's body space, correct for
		// the proxy (its own origin) and for the owner's body alike, and — unlike SetRefPosition — the engine's
		// own derivation is the one that accounts for reference scale. Only Frame2 has to be moved off the
		// corner and onto the anchor. The proxy case still overwrites Frame1 with an exact zero, since its
		// body origin is the corner by construction and the derived value is that zero up to float error.
		if (!WielderPrim)
		{
			PhysicalTetherConstraint->ConstraintInstance.SetRefPosition(EConstraintFrame::Frame1, FVector::ZeroVector);
		}
		PhysicalTetherConstraint->ConstraintInstance.SetRefPosition(EConstraintFrame::Frame2, AnchorLocal);
		// The rope constrains no rotation — every angle is free.
		PhysicalTetherConstraint->SetAngularSwing1Limit(ACM_Free, 0.0f);
		PhysicalTetherConstraint->SetAngularSwing2Limit(ACM_Free, 0.0f);
		PhysicalTetherConstraint->SetAngularTwistLimit(ACM_Free, 0.0f);
		PhysicalTetherTarget = TargetPrim;
		PhysicalTetherBone = Bone;
		PhysicalTetherWielder = WielderPrim;
		PhysicalTetherWielderBone = WielderBone;
		PhysicalTetherWielderActorLocal = WielderActorLocal;
		PhysicalTetherAnchorLocal = AnchorLocal;
		PhysicalTetherSmoothedAnchorLocal = AnchorLocal; // Seed EMA as a generating anchor (to prevent first frame spurious drift).
		PhysicalTetherLimit = -1.0f; // Forced update below.
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] physical tether created: %s/%s, hand side %s"),
			*GetName(), *GetNameSafe(TargetPrim), *Bone.ToString(),
			WielderPrim ? *GetNameSafe(WielderPrim) : TEXT("kinematic proxy"));
	}

	// Spherical distance limit = material leg length. Node zero is an exact zero-radius
	// contract, not a 1 cm convenience slack: represent it with XYZ Locked because Chaos
	// Limited(0) is not a portable zero-distance limit across solver paths.
	const float Limit = FMath::Max(LegRestLen, 0.0f);
	const bool bWasZeroLimit =
		PhysicalTetherLimit >= 0.0f &&
		PhysicalTetherLimit <= KINDA_SMALL_NUMBER;
	const bool bIsZeroLimit =
		Limit <= KINDA_SMALL_NUMBER;
	if (bWasZeroLimit != bIsZeroLimit ||
		!FMath::IsNearlyEqual(PhysicalTetherLimit, Limit, 0.5f))
	{
		const ELinearConstraintMotion Motion =
			bIsZeroLimit
				? LCM_Locked
				: LCM_Limited;
		PhysicalTetherConstraint->SetLinearXLimit(Motion, Limit);
		PhysicalTetherConstraint->SetLinearYLimit(Motion, Limit);
		PhysicalTetherConstraint->SetLinearZLimit(Motion, Limit);
		PhysicalTetherLimit = Limit;
	}

	SamplePhysicalTetherForce(DeltaTime);
}

void URopeComponent::SamplePhysicalTetherForce(float DeltaTime)
{
	if (!PhysicalTetherConstraint || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// Physical backend is authoritative for this frame. Store its measured force in the
	// common lambda channel; do not add/max an analytic hard-reaction estimate on top.
	LengthConstraintState.Backend = ERopeLengthConstraintBackend::Chaos;
	FVector LinearForce = FVector::ZeroVector;
	FVector AngularForce = FVector::ZeroVector;
	PhysicalTetherConstraint->GetConstraintForce(LinearForce, AngularForce);
	LengthConstraintState.LastLambda =
		static_cast<float>(LinearForce.Size()) * DeltaTime;
	LengthConstraintState.LastLambdaDt = DeltaTime;
}

void URopeComponent::TeardownPhysicalTether()
{
#if !UE_BUILD_SHIPPING
	// Rebuild-thrash visibility: a teardown per diagnostic line means the constraint never keeps a warm start.
	if (PhysicalTetherConstraint && CVarRopeLiftDebug.GetValueOnGameThread() > 0)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] LIFTDBG tether teardown (target=%s bone=%s limit=%.1f)"),
			*GetName(), *GetNameSafe(PhysicalTetherTarget.Get()), *PhysicalTetherBone.ToString(),
			PhysicalTetherLimit);
	}
#endif
	if (PhysicalTetherConstraint)
	{
		PhysicalTetherConstraint->BreakConstraint();
		PhysicalTetherConstraint->DestroyComponent();
		PhysicalTetherConstraint = nullptr;
	}
	if (PhysicalTetherProxy)
	{
		PhysicalTetherProxy->DestroyComponent();
		PhysicalTetherProxy = nullptr;
	}
	PhysicalTetherTarget = nullptr;
	PhysicalTetherBone = NAME_None;
	PhysicalTetherWielder = nullptr;
	PhysicalTetherWielderBone = NAME_None;
	PhysicalTetherWielderActorLocal = FVector::ZeroVector;
	PhysicalTetherLimit = -1.0f;
	PhysicalTetherPrePhysicsFrame = MAX_uint64;
	PhysicalTetherAnchorLocal = FVector::ZeroVector;
}

USkeletalMeshComponent* URopeComponent::GetWrappedMesh() const
{
	// State.Mesh is a USceneComponent, generalized to allow static wraps, while this function still promises a
	// skeletal mesh: for a static target the cast fails and it returns null. The caller reaches the target
	// actor through GetOwner instead.
	return const_cast<USkeletalMeshComponent*>(Cast<USkeletalMeshComponent>(WrapController.State.Mesh.Get()));
}

void URopeComponent::ApplyPullForce(const FVector& Force, const FRopePullSample& Pull, float DeltaTime)
{
	// The wrap target component, which may belong to another actor. The rope's contract is that it only reads
	// its target, so the weak pointer is const; a pull is a deliberate gameplay intervention — applying force —
	// so this is the one place it is pulled back to non-const.
	// State.Mesh is a USceneComponent now that static wraps are supported, and the receiver chain is agnostic
	// to the target's type: force is applied to a light physics prop or a static object exactly as to a
	// skeletal one. Null only happens when the target is destroyed.
	USceneComponent* MeshComp = const_cast<USceneComponent*>(WrapController.State.Mesh.Get());
	if (!MeshComp)
	{
		return;
	}
	AActor* Owner = MeshComp->GetOwner();

	// Force = the pull direction × the tension cap. A physics body takes it through the tension-capped velocity drive (ApplyPullVelocityDrive).
	const float MaxTension = static_cast<float>(Force.Size());
	const FVector Dir = (MaxTension > KINDA_SMALL_NUMBER) ? (Force / MaxTension) : FVector::ZeroVector;

	// The receiver is resolved with the same ladder the tether uses (ResolveTetherEndpoint); the active pull
	// no longer walks a ladder of its own.
	// It used to, in a different order — the pull looked at character movement before simulating primitives —
	// so wrapping a simulating primitive owned by an active CMC character had the pull driving the movement
	// while the tether drove the primitive. The movement branch was only ever meant as a *fallback*, "this is
	// an animated bone and cannot be pushed, so push the mover instead", but sitting ahead of the simulation
	// test it intercepted targets that could be pushed. Unified to specific (physics body) before general (mover).
	// (The pull does not use the effective mass the resolution carries — the tension-capped drive reads the body mass directly.)
	const bool bCanReuseWrappedEndpoint = WrappedEndpointCache.bValid &&
		WrappedEndpointCache.TargetMesh.Get() == MeshComp && WrappedEndpointCache.TargetBone == Pull.Bone;
	const FRopeTetherEndpoint Endpoint = bCanReuseWrappedEndpoint
		? WrappedEndpointCache.Target
		: ResolveTetherEndpoint(MeshComp, Owner, Pull.Bone, HoldConfig.GroundBraceFactor);

	// Extension gateway: a subclass — custom movement, a vehicle — that intercepted this receiver and handled it skips the built-in application.
	if (ApplyTractionToReceiver(MakeTractionRequest(Endpoint, ERopeTractionSource::ActivePull, Dir, MaxTension,
		DeltaTime, /*bWielderSide*/ false)))
	{
		return;
	}

	switch (Endpoint.Kind)
	{
	case ERopeEndpointKind::SimBody:
		// A simulating body (a promoted skeletal bone, a simulating primitive, or a simulating root) takes the
		// tension-capped velocity drive directly: an impulse at the centre of mass, so no torque and no spin,
		// no overshoot and no juddering, and a heavy target simply lags. The angular velocity clamp mops up any
		// residual spin.
		ApplyPullVelocityDrive(Endpoint.Prim, Endpoint.Bone, Dir, MaxTension, DeltaTime);
		ClampPulledBodyVelocity(Endpoint.Prim, Endpoint.Bone);
		// (The old "apply to bone and mover both" path for partial ragdolls is gone: the receiver resolution no
		//  longer classifies a partial ragdoll as a bone — that is rung 1 of ResolveTetherEndpoint — so a bone
		//  endpoint arriving here is always a free ragdoll, where force travels through the joints to the whole
		//  body. That restores the symmetry of the tether and the pull seeing the same receiver.)
		return;

	case ERopeEndpointKind::Character:
		// Force cannot be applied to an animation-driven bone, so the whole mover is pulled instead, and the
		// limb IK or ragdoll response follows from there. Under MOVE_None it never reaches this branch — the
		// resolution classifies it as an anchor, and the warning below fires.
		Endpoint.Movement->AddForce(Force);
		return;

	default:
		break;
	}

	// No receiver at all (anchor or None: a bone chain with no simulating body, a non-simulating component with
	// movement disabled, or a non-character with a non-simulating root). Warn once per wrap that the force is
	// quietly going nowhere.
	if (!PullDrive.bLoggedPullNoReceiver)
	{
		PullDrive.bLoggedPullNoReceiver = true;
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] Pull has no force receiver: target=%s bone=%s has no simulating body up its parent chain, owner=%s has no force-consuming CharacterMovement (not a Character, or movement disabled) and its root is not simulating — pull force is dropped."),
			*GetName(), *MeshComp->GetName(), *Pull.Bone.ToString(), *GetNameSafe(Owner));
	}
}

void URopeComponent::ApplyPullVelocityDrive(UPrimitiveComponent* Prim, FName BoneName, const FVector& Dir, float MaxTension, float DeltaTime) const
{
	// Tension-capped velocity drive, replacing the old constant force: drive the target along the pull
	// direction toward VTarget, clamping this frame's impulse to J = min(mass × ΔV, MaxTension × dt).
	//  - Light target: J = mass × ΔV, within the tension budget, so it reaches the target speed *exactly*
	//    with no overshoot. That removes the juddering a constant force (a = F/m) produced by blowing past the
	//    target within one frame.
	//  - Heavy target: J = MaxTension × dt, at the tension limit, so it accelerates by ΔV = J/mass per frame
	//    and lags behind, which is the realistic result.
	// The impulse is at the centre of mass (AddImpulse with no position), so there is no torque and no spin.
	// bVelChange = false means a real impulse, divided by mass.
	const float VTarget = FMath::Max(0.0f, HoldConfig.ActivePullMaxLinearSpeed);
	if (!Prim || VTarget <= 0.0f || MaxTension <= KINDA_SMALL_NUMBER || Dir.IsNearlyZero() || DeltaTime <= 0.0f)
	{
		return;
	}
	// Acceleration only, with no reverse thrust, and exact arrival (Alpha = 1). The cap is on the impulse
	// (tension × dt), the same framework the tether's reel uses. The one difference is direction: the tether
	// also brakes to settle on its boundary, while the active pull only accelerates, because the user lets go.
	const RopeTraction::FRopeAxisServo Servo{ VTarget, /*Alpha*/ 1.0f, /*bBidirectional*/ false, /*bCancelOutward*/ false };
	const float VAlong = static_cast<float>(FVector::DotProduct(Prim->GetPhysicsLinearVelocity(BoneName), Dir));
	const float J = RopeTraction::ClampAxisImpulse(RopeTraction::ComputeAxisDeltaV(VAlong, Servo), ResolveBodyMass(Prim, BoneName), MaxTension * DeltaTime);
	if (!FMath::IsNearlyZero(J))
	{
		Prim->AddImpulse(Dir * J, BoneName, /*bVelChange*/ false);
	}
}

void URopeComponent::ClampPulledBodyVelocity(UPrimitiveComponent* Prim, FName BoneName) const
{
	if (!Prim)
	{
		return;
	}
	// Angular velocity cap, a safety net for residual spin. Applying force at the centre of mass removes the
	// source of pull torque, but ragdoll joint dynamics still produce some. (Linear traction is
	// ApplyPullVelocityDrive's tension-capped impulse; this handles angular velocity only.)
	const float MaxAngDeg = FMath::Max(0.0f, HoldConfig.ActivePullMaxAngularSpeed);
	if (MaxAngDeg > 0.0f)
	{
		const float MaxAngRad = FMath::DegreesToRadians(MaxAngDeg);
		const FVector AngVel = Prim->GetPhysicsAngularVelocityInRadians(BoneName);
		if (AngVel.SizeSquared() > MaxAngRad * MaxAngRad)
		{
			Prim->SetPhysicsAngularVelocityInRadians(AngVel.GetClampedToMaxSize(MaxAngRad), /*bAddToCurrent*/ false, BoneName);
		}
	}
}

void URopeComponent::UpdateTargetPullable()
{
	// This Wrapped frame's pullability decision, shared by the climb-in direction and the distribution
	// observation. Computing it every frame regardless of overshoot is what makes the tether's recovery
	// (UpdateTether) and the active pull's direction (ApplyWrappedTraction) read the same answer.
	const FRopeResolvedWrappedEndpoints* Endpoints = GetOrResolveWrappedEndpoints();
	if (!Endpoints)
	{
		return; // Target disappears (destroyed) — Hold will release soon. Maintain previous check.
	}
	USceneComponent* MeshComp = Endpoints->TargetMesh.Get();
	if (!MeshComp)
	{
		return;
	}

	// A rope wrapped around its own owner has no meaningful distribution, so the target is always pullable.
	const bool bSelfWrap = (GetOwner() != nullptr && MeshComp->GetOwner() == GetOwner());
	bool bPullable;
	if (bSelfWrap)
	{
		bPullable = true;
	}
	else
	{
		// Effective mass at both ends: a grounded character folds in ground friction through
		// GroundBraceFactor, and MOVE_None or static is an anchor of infinite mass. It uses the same
		// resolution the tether application does (ResolveTetherEndpoint), so the pullability decision and the
		// real application point cannot disagree.
		const float WT = EndpointInvMass(Endpoints->Target);
		const float WW = EndpointInvMass(Endpoints->Wielder);
		const float InfMass = TNumericLimits<float>::Max();
		const float EffMassTarget = (WT > KINDA_SMALL_NUMBER) ? (1.0f / WT) : InfMass; // invMass 0 = anchor (infinite).
		const float EffMassWielder = (WW > KINDA_SMALL_NUMBER) ? (1.0f / WW) : InfMass;
		if (!PullDrive.bTargetPullableInit)
		{
			bPullable = (EffMassTarget <= EffMassWielder); // First valid frame: seeded with pure comparison without hysteresis.
		}
		else
		{
			// The hysteresis is an unexposed internal constant, a stabilizer that keeps a decision right at the
			// boundary from flipping every frame. The one exposed knob that sets *where* the crossover lies is
			// GroundBraceFactor; this is just the deadband around that line.
			constexpr float PullMassHysteresis = 1.1f;
			bPullable = DecideTargetPullable(EffMassTarget, EffMassWielder, PullDrive.bTargetPullable, PullMassHysteresis);
		}
	}
	PullDrive.bTargetPullable = bPullable;
	PullDrive.bTargetPullableInit = true;
	PullDrive.LastTargetShare = bPullable ? 1.0f : 0.0f; // Effective quotient (binary) read by wielder gate/debugger.
}

bool URopeComponent::DecideTargetPullable(float EffMassTarget, float EffMassWielder, bool bPrev, float MarginRatio)
{
	const float Margin = FMath::Max(MarginRatio, 1.0f);
	if (bPrev)
	{
		// Currently pullable: it only flips to not-pullable once the target is more than Margin times heavier than the wielder.
		return !(EffMassTarget > EffMassWielder * Margin);
	}
	// Currently not pullable: it only flips back once the target is lighter than the wielder × (1/Margin).
	return (EffMassTarget * Margin <= EffMassWielder);
}

void URopeComponent::ApplyPullForceToWielder(const FVector& Force, float DeltaTime)
{
	// Climb-in (target not pullable): apply the active pull force to the wielder, the rope's owner, so they
	// are drawn toward the anchor instead. Mirrors the owner side of ApplyPullForce: CharacterMovement, then simulating root.
	AActor* RopeOwner = GetOwner();
	if (!RopeOwner)
	{
		return;
	}

	// The receiver is **resolved** first, so what nearly received the force can be handed to the extension
	// gateway as-is. The order is the same as before (CharacterMovement, then simulating root) and comes from
	// here down the ResolveTetherEndpoint ladder. Reordering it would put the sim route ahead of the mover and
	// change how climb-in behaves.
	FRopeTetherEndpoint Receiver;
	Receiver.Actor = RopeOwner;
	if (UCharacterMovementComponent* Movement = GetForceConsumingMovement(RopeOwner))
	{
		Receiver.Kind = ERopeEndpointKind::Character;
		Receiver.Movement = Movement;
	}
	else if (UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(RopeOwner->GetRootComponent()))
	{
		if (Root->IsSimulatingPhysics())
		{
			Receiver.Kind = ERopeEndpointKind::SimBody;
			Receiver.Prim = Root;
		}
	}

	const float Magnitude = static_cast<float>(Force.Size());
	const FVector Dir = (Magnitude > KINDA_SMALL_NUMBER) ? (Force / Magnitude) : FVector::ZeroVector;
	if (ApplyTractionToReceiver(MakeTractionRequest(Receiver, ERopeTractionSource::ActivePull, Dir, Magnitude,
		DeltaTime, /*bWielderSide*/ true)))
	{
		return;
	}

	switch (Receiver.Kind)
	{
	case ERopeEndpointKind::Character:
		Receiver.Movement->AddForce(Force);
		return;
	case ERopeEndpointKind::SimBody:
		Receiver.Prim->AddForce(Force);
		return;
	default:
		// No receiver (not a character and no simulating root): drop it quietly — this setup has no climb-in.
		return;
	}
}

#pragma endregion Tether_And_Pull_Application

