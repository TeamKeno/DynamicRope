// Copyright Epic Games, Inc. All Rights Reserved.

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

namespace
{
	// C1: taut check hysteresis·release grace period is an internal constant that has been ground truth tuned (integrated into a single TautSensitivity,
	// removed from designer exposure). To adjust the values ​​here.
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

	// Pull sample output (always — debugger/BP observation + shared input from traction/release). direction follows the first straight leg (space).
	PullDrive.LastPullSample = FRopePullSample();
	WrapController.ComputePull(ObservationSim, HoldConfig.PullBendThresholdDeg, PullDrive.LastPullSample);

	// Taut is geometry, not a prerequisite XPBD load. Prefer the same live hand/anchor
	// material boundary used by movement authority. This removes the circular dependency
	// "segment must stretch -> tension appears -> tether may enforce no stretch".
	FRopeWielderMovementConstraint LiveConstraint;
	const bool bHasLiveConstraint = BuildWielderMovementConstraint(LiveConstraint);
	// slack ratio·sag cap is interpreted in a single TautSensitivity (GetEffectiveTaut*), hysteresis is an internal constant.
	// Because the live material-length path and legacy geometry path must react in the **same direction** (default
	// The two paths share these values (so that the slider is felt even when bEnforceWielderLengthConstraint=true).
	// Entry/maintenance hysteresis: Once checked as tight, widens the tolerance to prevent chattering of the threshold boundary (+ grace latch below).
	const float EffectiveTautMaxSag = GetEffectiveTautMaxSag();
	const float EffectiveTautSlackRatio = GetEffectiveTautSlackRatio();
	const float TautHysteresis = PullDrive.bChainTaut ? TautSlackReleaseScaleConst : 1.0f;
	const float SagLimit = EffectiveTautMaxSag * TautHysteresis;
	const bool bSagTaut =
		EffectiveTautMaxSag <= 0.0f || PullDrive.LastPullSample.MaxLegSag <= SagLimit;

	// live boundary: Derive the traction start distance slack from TautSensitivity, but LengthConstraintActivationSlop
	// Maintain at the minimum allowable value for numerical stability (whichever is greater). Also shared is a sag gate for “only when visually unfolded”.
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
	// release grace period (time latch): Minimum transfer tension observation is at threshold 0 (default), the entry/maintenance threshold is the same, so hysteresis is
	// disappears, and in the GPU rope, it is a delayed mirror of 1~2 frames, and in the alert state, the check flutters in units of frames, and "the entire velocity
	// Cut ↔ Freedom" alternates (wielder excitement). Entry is immediate, only release occurs with a grace period during TautReleaseGraceTime to prevent chattering.
	// Hang up. sample invalid is immediately false without a grace period (maintaining the above firm contract — it does not extend the observation gap tautologically).
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

	// Pull smoothing (stage 2): (1) aiming node fractional smoothing — Discrete hops between frames of integer AimNode (direction jump overall)
	// + tether excess discontinuity) is eliminated by interpolation between float EMA + node. (2) direction EMA — node location remaining above it
	// Trim noise (GPU mirror delay, etc.). The first valid frame after the start of wrap is seeded with the measurement value (no lag).
	if (!PullDrive.LastPullSample.bValid)
	{
		return;
	}

	// raw look-ahead before smoothing (integer aiming) — Debugger raw vs smoothed comparison.
	PullDrive.LastPullDirRaw = PullDrive.LastPullSample.Direction;

	// (1) Aiming index time smoothing → fractional aiming position interpolation.
	const float RawAimF = static_cast<float>(PullDrive.LastPullSample.AimNode);
	PullDrive.SmoothedAimNodeF = (PullDrive.SmoothedAimNodeF < 0.0f)
		? RawAimF // The first valid frame is seeded with the measurement value (no lag).
		: FMath::Lerp(PullDrive.SmoothedAimNodeF, RawAimF, RopeTraction::ExpSmoothAlpha(HoldConfig.PullAimSmoothTime, DeltaTime));
	const float AimF = FMath::Clamp(PullDrive.SmoothedAimNodeF, 0.0f, static_cast<float>(PullDrive.LastPullSample.AnchorNode));
	const FVector AimPos = RopeTraction::SampleFractionalAim(
		ObservationSim.Positions, AimF, PullDrive.LastPullSample.AnchorNode);
	PullDrive.LastPullSample.AimNodeF = AimF;
	PullDrive.LastPullSample.AimPos = AimPos;

	// (2) After recalculating the direction with continuous aiming, direction EMA. If degenerate(aiming=anchor), the raw direction is maintained.
	const FVector DirF =
		(AimPos - ObservationSim.Positions[PullDrive.LastPullSample.AnchorNode]).GetSafeNormal();
	const FVector DirIn = DirF.IsNearlyZero() ? PullDrive.LastPullSample.Direction : DirF;
	PullDrive.SmoothedPullDir = RopeTraction::SmoothDirection(
		PullDrive.SmoothedPullDir, DirIn, RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	PullDrive.LastPullSample.Direction = PullDrive.SmoothedPullDir;
}

void URopeComponent::ApplyWrappedTraction(float DeltaTime)
{
	// The endpoint cache is maintained only during the synchronous call section of this function. Even if virtual ApplyPullForce calls Super
	// The same target analysis is reused, and ApplyPullForce called separately from the outside does not leak cache.
	WrappedEndpointCache.Reset();
	// Climbable check: The tether distribution observation (LastTargetShare) and the active Pull's climb-in direction are shared.
	if (PullDrive.LastPullSample.bValid)
	{
		UpdateTargetPullable();
	}

	// ③-1 Automatic traction (tether): single λ impulse constraint + ragdoll physics constraint (Docs/PoC/05). No slack breaks —
	// The position recovery term of λ is bounded by MaxBiasSpeed, so there is no excess injection left in the ledger (a relic from the legacy servo era).
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

	// ③-2 Active Pull (constant force): The force given by the user input (SetActivePull/Wielder) is applied only when tension is present.
	// Tension check is bPullTaut latch updated by ② = previous chain geometry (bChainTaut) ∧ tension threshold
	// (threshold/hysteresis is HoldConfig — default threshold 0 = tension > ~0).
	// Ignore tautology 2nd layer: config(bActivePullRequiresTaut=false, rope full policy) / per-call(bActivePullIgnoresTaut,
	// SetActivePull argument — for animation pull window section). In either case, if it is Wrapped + valid sample, it is approved.
	// It is a constant unrelated to tension, so there is no feedback runaway.
	const bool bActivePullPassesGate = PullDrive.ActivePullForce > 0.0f && PullDrive.LastPullSample.bValid
		&& (!HoldConfig.bActivePullRequiresTaut || PullDrive.bActivePullIgnoresTaut || PullDrive.bPullTaut);
#if WITH_GAMEPLAY_DEBUGGER
	// The debugger cannot know the actual application from the request value (ActivePullForce) alone — the pull gate and bypass layer 2 are here.
	// Because it is divided. Whether the gate passes or not is left as is (this is separate from the case where power is discarded at the receiver stage).
	DebugActivePullPassedGate = bActivePullPassesGate;
#endif
	if (bActivePullPassesGate)
	{
		// If the object is heavy and cannot be pulled (not pullable), put the force on the wielder and pull it towards the anchor (climb-in):
		// LastPullSample.Direction is the anchor → hand direction, so the sign is reversed = hand → anchor — "If I have to be dragged, I will go."
		// (pull to wall/heavy ragdoll/dragon = 3D maneuver). If possible, apply it to the target and pull it toward the wielder.
		if (!PullDrive.bTargetPullable)
		{
			ApplyPullForceToWielder(-PullDrive.LastPullSample.Direction * PullDrive.ActivePullForce, DeltaTime);
		}
		else
		{
			ApplyPullForce(PullDrive.LastPullSample.Direction * PullDrive.ActivePullForce, PullDrive.LastPullSample, DeltaTime);
		}
	}
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
	// From bone, go up the parent chain to find the bone of the nearest "physics body being simulated" (or None).
	// The Wrapped bone may be a bodyless bone in the physics asset, such as a twisted bone — in that case, just look at the bone name.
	// If you check the non-simulation and fall into the character movement branch, the ragdoll setup has the movement turned off (MOVE_None).
	// AddForce is silently abandoned. Force/velocity bone is promoted to this function to find it.
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

	// Check if there is a kinematic body on the simulation bone (parent chain) = partial ragdoll. If there is, the restraint will completely cause bone traction.
	// Absorbed (infinite mass wall) The servo/force applied to the bone is not transmitted to the actor — in this case, the receiver interpretation is not to the bone, but to the bone.
	// You must go down to the moving object (character). If not (simulate everything, including the root body), it is a Free ragdoll in which the entire body is pulled by the joints.
	// Bones without bodies (twist/IK) are not constrained, so they are skipped.
	bool IsSimBoneBoundToKinematic(const USkeletalMeshComponent* Mesh, FName SimBone)
	{
		for (FName Bone = Mesh->GetParentBone(SimBone); !Bone.IsNone(); Bone = Mesh->GetParentBone(Bone))
		{
			// Body Existence + Non-Simulation = Kinematic Constraints. (Simulation status is asked through component API —
			// FBodyInstance::IsInstanceSimulatingPhysics cannot be linked because it is non-export inline.)
			if (Mesh->GetBodyInstance(Bone) != nullptr && !Mesh->IsSimulatingPhysics(Bone))
			{
				return true;
			}
		}
		return false;
	}

	// Can the character movement consume power now? If MOVE_None (DisableMovement — ragdoll setup convention)
	// AddForce is only accumulated and not consumed, so the "fake success" power is lost — in that case it is passed on to another recipient.
	// Directly receives the target actor for wrap (regardless of whether the target is a skeletal/static/physical prop — check based on the owning actor).
	UCharacterMovementComponent* GetForceConsumingMovement(const AActor* Owner)
	{
		const ACharacter* Character = Cast<ACharacter>(Owner);
		UCharacterMovementComponent* Movement = Character ? Character->GetCharacterMovement() : nullptr;
		return (Movement && Movement->MovementMode != MOVE_None) ? Movement : nullptr;
	}

	// Mass (kg) of the body subject to application: If it is a skeletal bone, then the mass of the body, otherwise (or if there is no body or the mass is 0)
	// component mass. The physical body mass is a value that the UE automatically maintains as collision volume × density, so no separate setting is required.
	// (Because it depends on UObject, RopeTraction is not included in pure math — mass is read from here and passed there.)
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

	// ===== Tether endpoint (receiver) interpretation — cross the ladder only once =====
	// “What is received” is checked only once here and the type, application point, and effective mass are confirmed together. In the past, distribution mass and actual
	// Each ladder with the same application point was duplicated, and on it, the application lambda recast to determine which rung it was.
	// I inferred backwards — if the order is wrong, it becomes a bug where “mass is seen as an anchor, but force is applied somewhere else” (partial ragdoll in CL 392)
	// the root gate actually did). If one interpretation is shared, deviation is structurally impossible.
	// ERopeEndpointKind/FRopeTetherEndpoint is a shared check type in Core/RopeTractionTypes.h. The component is
	// Target/wielder results are cached within the same Wrapped frame and shared by pullable/tether/basic pull.

	// Recipient interpretation (target/wielder shared). Sequence: Skeletal Free ragdoll bone → Simulation primitive → Simulation root → Character → Anchor.
	// (Partial ragdoll — sim bone tied to upper kinematic body — rung 1 does not pick up and falls through to character/anchor.)
	//  - MeshComp: State.Mesh if target, nullptr if wielder (automatic skip of skeletal/primitive rung → from root).
	//  - Physical body mass is a value that the UE automatically maintains as collision volume × density, so no separate setting is required.
	//  - Character ground is a finite brace (Mass×GroundBraceFactor — stepping resistance), air is Mass, and MOVE_None is an anchor.
	FRopeTetherEndpoint ResolveTetherEndpoint(USceneComponent* MeshComp, AActor* Owner, FName WrappedBone, float GroundBraceFactor)
	{
		FRopeTetherEndpoint Out;
		Out.Actor = Owner;

		// (1) Skeletal **Free Ragdoll** (simulated bone only articulated to the root body): Promoted from a Wrapped bone to the parent chain.
		// Applies to the nearest *simulation bone* (corresponding to a twist bone without a body). The default mass saved here is **full body
		// mass is the sum**(GetMass) — the coarse wielder/target share of hard Chaos reaction force and pullability are bone bodies
		// (3kg forearm) as a “light object”. Compliant analytic solve is the real world
		// After the attachment is determined, this default value is refined to the translation + rotation point Jacobian of the selected bone.
		//
		// **Partial ragdoll** (kinematic body in the parent chain above the simulated bone) is not received here and falls through downwards: bone
		// No matter how much you servo, the kinematic restraint absorbs it and only the rope stretches (rung 1 limit, which was put on hold on 2026-07-15 — this fallthrough
		// That is resolved). The only thing that can actually be dragged is a moving object (character rung — CMC active), or else there is nothing.
		// (anchor = infinite mass is the physical truth). The visual response of the Wrapped bone (presentation with accompanying arms) follows (Docs/PoC/05 §7).
		if (USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(MeshComp))
		{
			const FName SimBone = FindNearestSimulatingBone(Skel, WrappedBone);
			if (!SimBone.IsNone() && !IsSimBoneBoundToKinematic(Skel, SimBone))
			{
				Out.Kind = ERopeEndpointKind::SimBody;
				Out.Prim = Skel;
				Out.Bone = SimBone;
				// Whole body mass (total body sum). If degenerate (0 due to non-generation of body, etc.), previous bone body → component mass fallback.
				const float WholeMass = static_cast<float>(Skel->GetMass());
				Out.Mass = (WholeMass > KINDA_SMALL_NUMBER) ? WholeMass : ResolveBodyMass(Skel, SimBone);
				return Out;
			}
		}
		// (2) The target component itself is a primitive being simulated (light physics prop, etc.).
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
		// (3) Owning actor root primitive is simulating (constructing physics actor). The wielder starts here because MeshComp=nullptr.
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
		// (4) Character: If MOVE_None (ragdoll setup convention), the movement does not consume force, so it is used as an anchor.
		if (UCharacterMovementComponent* Movement = GetForceConsumingMovement(Owner))
		{
			Out.Kind = ERopeEndpointKind::Character;
			Out.Movement = Movement;
			const float BraceScale = Movement->IsMovingOnGround() ? FMath::Max(GroundBraceFactor, 1.0f) : 1.0f;
			Out.Mass = Movement->Mass * BraceScale;
			return Out;
		}
		// (5) static/Kinematic/MOVE_None/Non-simulated Non-character → Anchor (mass 0). Only location fallback is possible.
		Out.Kind = ERopeEndpointKind::Anchor;
		return Out;
	}

	// Moves the interpreted receiver to the request read by the public extension hook (ApplyTractionToReceiver).
	// Source/Direction/Amount is filled by each application path (units are different for each path — refer to the request type annotation).
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

	// Effective inverse mass (w = 1/effective mass) for tether automatic distribution. 0 = anchor (infinite mass).
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


	// Tether application shared context — both ends receiver interpretation + extension gateway. Observation/λ calculation and state storage are performed by the component.
	struct FRopeTetherContext
	{
		const FRopeTetherEndpoint& Target;
		const FRopeTetherEndpoint& Wielder;
		float DeltaTime;
		// Receiver application extension gateway (delegating to the component's virtual). If true, built-in application is skipped.
		TFunctionRef<bool(const FRopeTractionRequest&)> TractionGate;
	};

	// Dispatches application to the interpreted recipient (destination/wielder shared skeleton — Constraint used by two application points on the tether).
	// The callback already knows what it received (no recasting required). Step/return is this frame axis ΔV (cm/s).
	//  - SimApply(Prim, Bone, Dir, DeltaV): Physics simulation body.
	//  - CharacterApply(Movement, Dir, DeltaV): CMC driven character.
	//  - AnchorApply(Actor, Dir, DeltaV): Anchor(static/MOVE_None) — No action (w=0, so ΔV is 0).
	float ApplyToTetherEndpoint(
		const FRopeTetherContext& Ctx, bool bWielderSide, const FVector& Dir, float Step,
		TFunctionRef<float(UPrimitiveComponent*, FName, const FVector&, float)> SimApply,
		TFunctionRef<void(UCharacterMovementComponent*, const FVector&, float)> CharacterApply,
		TFunctionRef<void(AActor*, const FVector&, float)> AnchorApply)
	{
		const FRopeTetherEndpoint& Endpoint = bWielderSide ? Ctx.Wielder : Ctx.Target;

		// Extension gateway (URopeComponent::ApplyTractionToReceiver): Skips built-in application if handled by subclass.
		// Since all four application points of the tether pass through this skeleton, one line here covers the entire tether path.
		// return is the same contract as Step(= shortfall 0) — sim-body unapplied branch.
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
		DeltaTime);
	if (PhysicalTetherConstraint)
	{
		PhysicalTetherPrePhysicsFrame = GFrameCounter;
	}
}

#pragma endregion Traction_Endpoint_And_Tether_Policies

#pragma region Tether_And_Pull_Application

FVector URopeComponent::ComputeSmoothedWielderDir(const FVector& Aim, const FVector& DirToAim, float DeltaTime, bool bInstantaneous)
{
	// direction = along the first straight leg of the rope from the hand (node ​​0). Aiming (AimPos) is toward the corner of the wall, and the rope is straight.
	// If aiming=hand (chord ~0), it falls back to the reverse direction of anchor→aiming (=hand→anchor).
	const FVector HandPos = GetComponentLocation();
	FVector WielderDirRaw = Aim - HandPos;
	if (!WielderDirRaw.Normalize(KINDA_SMALL_NUMBER))
	{
		WielderDirRaw = -DirToAim;
	}
	// Aerial Swing (bInstantaneous): EMA omitted, instantaneous geometry remains — axis lagged while orbiting rapidly (ω·τ ≈
	// The tangential error component of ten degrees brakes/accelerates the swing of each utterance frame and interferes with AirControl operation. circle of ema
	// The purpose (to defend the random walk of the servo top-up) is that in the λ single impulse + speed clamp system, only the ground corner noise remains.
	// state continues to be seeded raw, ensuring continuous EMA re-entry upon landing.
	if (bInstantaneous)
	{
		PullDrive.SmoothedWielderPullDir = WielderDirRaw;
		return WielderDirRaw;
	}
	// direction EMA (same constant/same function as SmoothedPullDir on target side): Raw with AimPos node noise/edge transition/proximity degenerate
	// If the direction bounces every frame, the clamp/top-up moves to a different axis each time and the vector increases in a random walk (runaway).
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

	// Reel/material-length and kinematic anchor rates are sampled from the same live
	// geometry. Activation tolerance must not add physical cable length.
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
		// hand point (wielder tip) ground truth velocity — Anchor-kind wielder (kinematic carrier: helicopter/mobile platform)
		// There is no velocity, so it is filled with finite differences. The wielder mirror of SmoothedAnchorPointVelocity on the target side.
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
				DeltaTime);
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
	// Omit wielder direction EMA during mid-air swing (instantaneous geometry) — while orbit is spinning fast (ω·τ ≈ decimal degrees lag)
	// The tangential error component of the lag axis brakes/accelerates the swing of each utterance frame and interferes with operation (2026-07-22 PIE,
	// "A moment of force" where AirControl boost becomes ineffective). Ground maintains EMA due to large corner/node noise.
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

	// Self wrap(owner == target): Since both ends are the same body, the pairing is self-cancelling — the wielder end is treated as an anchor (w=0)
	// Only the end of the target moves (same as legacy special case).
	// Opening velocity s = −(vT·dT + vW·dW) − dRest/dt. The terminal velocity is measured at the same rung as the receiver interpretation —
	// The cruise of a moving anchor (dragon) is carried on vT and is followed without separate feedforward (replaces the legacy anchor velocity EMA).
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

	// ---- Target Hard Projection (Kinematic Character Carry) ----
	// If the wielder's end is infinite mass (Anchor — kinematic carrier such as a helicopter) and the target is a CMC character, applying λ velocity is not enough.
	// The number of position errors is capped by the bias cap (TetherMaxBiasSpeed), so when the carrier is faster than that, the rope stretches infinitely.
	// Target mirror for hard projection (ConstrainWielderLocation) on the wielder side: Move the capsule to the current radius direction by the amount of the shortfall.
	// to eliminate the position error in the same frame (if blocked by a wall, the remainder remains as C and is received by λ/observation).
	// If both ends are finite masses, the λ pair permission owns the distribution and therefore does not fire (avoiding double compensation). elastic mode
	// (TetherCompliance>0) is excluded because it is an intentional elongation.
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
		// If the grounded character has been lifted with a significant velocity, it will fall into Falling before the floor snap/Z deletion of Walking is reversed.
		// (Launch convention). Horizontal towing (Applied.Z ≈ 0) passes below threshold — ground drag behavior is maintained.
		if (Endpoints->Target.Movement->IsMovingOnGround()
			&& Applied.Z > TetherLiftLaunchSpeedConst * DeltaTime)
		{
			Endpoints->Target.Movement->SetMovementMode(MOVE_Falling);
		}
	}

	// Anchor-kind objects (static/kinematic/animated) have no physical velocity, so anchor points are filled with ground truth EMA —
	// Moving object towing is followed by spread offset regardless of the bias cap (stationary anchors have ≈0 = no effect;
	// The secondary safety net is ClampInjectedVelocity(TetherMaxSpeed) on the application side). SimBody must be a rope
	// Reads vCOM+ω×r of the attachment point, and reads the CMC velocity of the character.
	const FVector VelTarget = (Endpoints->Target.Kind == ERopeEndpointKind::Anchor)
			? (bHasLiveConstraint
				? LiveConstraint.PivotVelocity
				: LengthConstraintState.SmoothedAnchorPointVelocity)
			: EndpointVelocityAtPoint(
				Endpoints->Target, TargetPoint);
	const float SepTarget = -static_cast<float>(FVector::DotProduct(VelTarget, DirTarget));
	// Anchor-kind wielders (kinematic carriers) also use the ground truth EMA symmetrically — the carrier's departure velocity is λ.
	// It is subject to gap offset and is followed regardless of the bias cap (TetherMaxBiasSpeed) (≈0 = no effect for stationary owners).
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

	// Effective distribution share (wielder gate/debugger compatible) = inverse mass ratio — λ Set to this frame value regardless of utterance.
	if (Lambda <= 0.0f)
	{
		return; // Already approaching sufficiently or both ends are anchored.
	}

	// ---- Approved: ΔV = λ × w(cm/s) for each end, each leg direction ----
	// All paths only touch the rope axis (+ orthogonal damping) component, so swing/gravity is preserved, and the resulting speed is TetherMaxSpeed.
	// Secondary clamping is performed. The rigid body simulation target is exclusive to Chaos, and the compliant simulation target and simulation wielder are real.
	// Receives physical impulse (ΔV × effective mass). Because it passes through the existing dispatch skeleton (ApplyToTetherEndpoint),
	// The extension gateway (ApplyTractionToReceiver, Amount = ΔV cm/s) is also the same.
	const float SpeedCap = FMath::Max(HoldConfig.TetherMaxSpeed, 0.0f);
	// Orthogonal damping dt correction: Setting value is per-frame rate based on 60fps → effective rate = 1−(1−d)^(dt·60). Previously, the ratio
	// Per-frame was used as is, and the higher the frame rate, the stronger the damping was. It was a frame rate dependent physics.
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
		// Compliant target or physical wielder: with same attachment-point Jacobian as solver
		// Since ΔV=λ*w was calculated, the actual rope impulse is exactly J=λ*d=(ΔV/w)*d.
		// AddImpulseAtLocation creates ω×r and torque together, and observation also reads the same point velocity.
		// (Orthogonal damping is for all axes — the option to exclude the vertical component will be reviewed and returned. Gravity drop is in equilibrium with damping.
		// The "weightless" look of being trapped at low velocities (≈g·dt/rate) is countered by tuning the magnitude of this value.)
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
		// CMC: Direct addition of velocity (contract to reflect this frame). The position recovery term of λ is bounded by MaxBiasSpeed.
		// There is no excess injection, and there is no need for a separate ledger/slack break. (The horizontal projection plan during grounding will be reviewed and returned —
		// Maintain full axis injection.)
		const FVector OldVel = Movement->Velocity;
		Movement->Velocity = RopeTraction::ClampInjectedVelocity(OldVel + Dir * DeltaV, OldVel, SpeedCap);
		// A grounded character whose upward injection exceeds the threshold is Falling — Walking will have Z velocity converted to floor restraint on the next tick.
		// Because it is discarded (disabling lifting), you must pass the mode using the same convention as Launch for the injection to survive. horizontal towing
		// Passes because Z injection ≈ 0.
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
	const FVector& AnchorWorld, const FVector& CornerWorld, float LegRestLen, float DeltaTime)
{
	// (Constraint tether — target half of sim body) Engine physics constraint: [kinematic proxy for corner ↔ anchor for target body
	// point] to the spherical limit of the leg rest length. GT per-frame velocity impulse is the "whole body size kick" of the joint body →
	// In addition to the dilemma of “runaway” vs. “bone size λ → traction force collapse” (2026-07-20 Pierce 7th iteration), the air load (suspended prop)
	// Even floating/pendulum pumping (2026-07-22 PIE) cannot be solved, but Chaos constraint is **together** with gravity, joints, and ground contact in the substep.
	// is released, resulting in full-body traction without runaway and true pendulum behavior (the standard pattern of "hanging the load in the hands").
	// The engine solves the alignment torque by holding the constraint frame as the body-local of the anchor (skeletal = bone TM, spear tip lever convention).
	AActor* Owner = GetOwner();
	if (!Owner || !TargetPrim)
	{
		return;
	}

	// Regenerate (promote/rewrap anchor) if target/bone has changed.
	if (PhysicalTetherConstraint
		&& (PhysicalTetherTarget.Get() != TargetPrim || PhysicalTetherBone != Bone))
	{
		TeardownPhysicalTether();
	}

	// Target body - current value of local anchor (constraint Frame2) — skeletal = bone TM (spear tip lever convention), component body =
	// component TM. It is a value that is pinned upon creation, so when the wrap anchor is relocated within the same (target, bone) (promotion/seed joining)
	// The rope anchor and constraint anchor are misaligned, resulting in constant violation = vibration — If the drift exceeds the threshold, dismantle and lower
	// Immediately regenerates from the creation block (a guard that does not fire in normal conditions).
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
	// The instantaneous anchorLocal is the delay difference between the rope Sim (GPU delayed mirror) and the current bone TM, which increases significantly every frame in fast ragdoll.
	// Shakes — If you place a 5cm guard at the instantaneous value, it regenerates (thrashes) every frame even without real relocation, creating a constraint.
	// I can't get a warm start and I actually feel trembled (measurement: regeneration 70%, strength 0 89%). So we smooth anchorLocal with EMA
	// Check with those values: delay noise is averaged out (smoothing value stays near the pinned anchor), and constant
	// Only relocation (seed joining/promotion) moves the average to exceed the threshold. Raise the threshold to 10cm to leave room.
	if (PhysicalTetherConstraint)
	{
		const float SmoothAlpha = RopeTraction::ExpSmoothAlpha(0.12f, DeltaTime); // ≈0.12s time constant.
		PhysicalTetherSmoothedAnchorLocal = FMath::Lerp(PhysicalTetherSmoothedAnchorLocal, AnchorLocal, SmoothAlpha);
		if (FVector::DistSquared(PhysicalTetherSmoothedAnchorLocal, PhysicalTetherAnchorLocal) > FMath::Square(10.0f))
		{
			TeardownPhysicalTether();
		}
	}

	if (!PhysicalTetherProxy)
	{
		PhysicalTetherProxy = NewObject<USphereComponent>(Owner,
			MakeUniqueObjectName(Owner, USphereComponent::StaticClass(), TEXT("RopeTetherProxy")));
		PhysicalTetherProxy->SetupAttachment(this);
		PhysicalTetherProxy->SetAbsolute(true, true, true); // World layout (regardless of rope component transform).
		PhysicalTetherProxy->InitSphereRadius(4.0f);
		// Requires a body (one side of the constraint) and no collisions or queries — PhysicsOnly + ignore all channels.
		// Why you shouldn't turn on queries: The default object type for USphereComponent is WorldDynamic;
		// Channel response Ignore only listens to **channel queries** (object type queries only return the shape's object type)
		// bone). So, if query is on, URopeStaticBodyProvider's OverlapMultiByObjectType scan
		// is captured, becomes a push-out collider that follows the corner every frame, and pushes the wrap node of my rope.
		PhysicalTetherProxy->SetCollisionEnabled(ECollisionEnabled::PhysicsOnly);
		PhysicalTetherProxy->SetCollisionResponseToAllChannels(ECR_Ignore);
		PhysicalTetherProxy->SetSimulatePhysics(false); // Kinematic — Move to corner every frame.
		PhysicalTetherProxy->SetHiddenInGame(true);
		PhysicalTetherProxy->RegisterComponent();
	}
	// Kinematic movement — Chaos looks at movement velocity and pulls constraints (moving corner/hand tracking).
	PhysicalTetherProxy->SetWorldLocation(CornerWorld);

	if (!PhysicalTetherConstraint)
	{
		PhysicalTetherConstraint = NewObject<UPhysicsConstraintComponent>(Owner,
			MakeUniqueObjectName(Owner, UPhysicsConstraintComponent::StaticClass(), TEXT("RopeTetherConstraint")));
		PhysicalTetherConstraint->SetupAttachment(PhysicalTetherProxy);
		PhysicalTetherConstraint->RegisterComponent();
		PhysicalTetherConstraint->SetWorldLocation(CornerWorld);
		PhysicalTetherConstraint->SetDisableCollision(false);
		// A single body plus positional projection can snap/rebound at the moving limit, so
		// component bodies keep projection disabled. This backend is exclusively the
		// inextensible path and must always be a hard Chaos limit. Positive compliance is
		// owned by the common analytic material solver and never reaches this function.
		// The profile must be configured before SetConstrainedComponents initializes Chaos.
		FConstraintProfileProperties& Profile =
			PhysicalTetherConstraint->ConstraintInstance.ProfileInstance;
		if (!SkelBody)
		{
			Profile.bEnableProjection = false;
		}
		Profile.LinearLimit.bSoftConstraint = false;
		Profile.LinearLimit.Stiffness = 0.0f;
		Profile.LinearLimit.Damping = 0.0f;
		Profile.LinearLimit.Restitution = 0.0f;
		PhysicalTetherConstraint->SetConstrainedComponents(PhysicalTetherProxy, NAME_None, TargetPrim, Bone);
		// constraint frame Origin: proxy side = proxy origin (corner), target side = anchor's body-local (above AnchorLocal —
		// wrap freezes bone-local anchors (same conventions, including spear tip lever). The distance limit is between these two points.
		PhysicalTetherConstraint->ConstraintInstance.SetRefPosition(EConstraintFrame::Frame1, FVector::ZeroVector);
		PhysicalTetherConstraint->ConstraintInstance.SetRefPosition(EConstraintFrame::Frame2, AnchorLocal);
		// rope has no rotation constraints — all angles are Free.
		PhysicalTetherConstraint->SetAngularSwing1Limit(ACM_Free, 0.0f);
		PhysicalTetherConstraint->SetAngularSwing2Limit(ACM_Free, 0.0f);
		PhysicalTetherConstraint->SetAngularTwistLimit(ACM_Free, 0.0f);
		PhysicalTetherTarget = TargetPrim;
		PhysicalTetherBone = Bone;
		PhysicalTetherAnchorLocal = AnchorLocal;
		PhysicalTetherSmoothedAnchorLocal = AnchorLocal; // Seed EMA as a generating anchor (to prevent first frame spurious drift).
		PhysicalTetherLimit = -1.0f; // Forced update below.
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] physical tether created: %s/%s"),
			*GetName(), *GetNameSafe(TargetPrim), *Bone.ToString());
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
	PhysicalTetherLimit = -1.0f;
	PhysicalTetherPrePhysicsFrame = MAX_uint64;
	PhysicalTetherAnchorLocal = FVector::ZeroVector;
}

USkeletalMeshComponent* URopeComponent::GetWrappedMesh() const
{
	// State.Mesh is USceneComponent (generalized compared to static wrap). “Skeletal mesh” return agreement maintained —
	// If the target is static, the cast fails and is null (the target actor response is from caller to GetOwner).
	return const_cast<USkeletalMeshComponent*>(Cast<USkeletalMeshComponent>(WrapController.State.Mesh.Get()));
}

void URopeComponent::ApplyPullForce(const FVector& Force, const FRopePullSample& Pull, float DeltaTime)
{
	// Wrap target component (cross-actor possible). Contractually, rope only reads the target, so weak is const,
	// Pull is an intended gameplay intervention (applying force), so it is explicitly pulled as non-const only here.
	// State.Mesh now supports USceneComponent (generalized over static wrap) — agnostic of the target type as a recipient chain.
	// Apply force (same logic as skeletal for light physics prop/static objects). Null only occurs when the target disappears (destroys).
	USceneComponent* MeshComp = const_cast<USceneComponent*>(WrapController.State.Mesh.Get());
	if (!MeshComp)
	{
		return;
	}
	AActor* Owner = MeshComp->GetOwner();

	// Force = pulling direction × maximum tension. The physical body is applied with a tension cap velocity drive (ApplyPullVelocityDrive).
	const float MaxTension = static_cast<float>(Force.Size());
	const FVector Dir = (MaxTension > KINDA_SMALL_NUMBER) ? (Force / MaxTension) : FVector::ZeroVector;

	// Recipient interpretation uses the same ladder as Tether (ResolveTetherEndpoint) — Active Pull does not walk its own ladder separately.
	// Removed. In the past, the order was different (Pull looked at character movements before sim primitives) and “active CMC characters were
	// When Wrapping a "owned sim primitive", Pull pulls the movement and Tether pulls the primitive. The movement branch is
	// Originally, it was a *fallback* that said "It is an animation bone, so it cannot be pushed, so push the moving object", but it is ahead of the simulation test, so it can be pushed.
	// Interception of the target — unified in the order of specific (physical body) → general (moving body).
	// (Pull does not use the effective mass that comes with the analysis — the tension cap drive reads the body mass directly.)
	const bool bCanReuseWrappedEndpoint = WrappedEndpointCache.bValid &&
		WrappedEndpointCache.TargetMesh.Get() == MeshComp && WrappedEndpointCache.TargetBone == Pull.Bone;
	const FRopeTetherEndpoint Endpoint = bCanReuseWrappedEndpoint
		? WrappedEndpointCache.Target
		: ResolveTetherEndpoint(MeshComp, Owner, Pull.Bone, HoldConfig.GroundBraceFactor);

	// Extension gateway: If the subclass (custom movement/vehicle) intercepted on a per-receiver basis has processed it, the built-in application is omitted.
	if (ApplyTractionToReceiver(MakeTractionRequest(Endpoint, ERopeTractionSource::ActivePull, Dir, MaxTension,
		DeltaTime, /*bWielderSide*/ false)))
	{
		return;
	}

	switch (Endpoint.Kind)
	{
	case ERopeEndpointKind::SimBody:
		// Simulation body (skeletal promotion bone / simulation primitive / simulation root): tension cap velocity is applied directly to the drive.
		// (center of gravity impulse, no torque/spin, no overshoot, no dust/slip, if it is heavy, there is a back sag). Suppresses residual spin with angle velocity clamps.
		ApplyPullVelocityDrive(Endpoint.Prim, Endpoint.Bone, Dir, MaxTension, DeltaTime);
		ClampPulledBodyVelocity(Endpoint.Prim, Endpoint.Bone);
		// (“bone + moving object dual application” for partial ragdolls has been removed: receiver interpretation no longer rates partial ragdolls as bones
		// — ResolveTetherEndpoint rung 1 — the bone endpoint that comes here is always
		// It is a Free ragdoll, and the force is transmitted throughout the body through the joints. Restoring symmetry where the tether and pull see the same recipient.)
		return;

	case ERopeEndpointKind::Character:
		// Since force cannot be applied to the animation driving bone, the entire moving object is tractioned (PoC 4.2: Even simple force transfer to the bone/root.
		// limb IK/ragdoll responses follow). If MOVE_None, it does not come here (interpretation is classified as anchor → warning below).
		Endpoint.Movement->AddForce(Force);
		return;

	default:
		break;
	}

	// No recipient (anchor/None = bone chain without sim body, non-simulated component + movement disabled, non-character + non-simulated root):
	// Notifies once per wrap that the power is quietly disappearing.
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
	// tension cap velocity drive (replaces constant force): Drives the target in the pulling direction target velocity (VTarget), but applies this frame.
	// Clamp impulse to J = min(mass×ΔV, MaxTension×dt).
	//  - Light target: J = mass×ΔV (tension margin) → *exactly* reaches target velocity (no overshoot). In one frame the constant force is a=F/m
	//    The problem of bouncing past the target (dust/jaw-jak) disappears.
	//  - Heavy object: J = MaxTension×dt (tension limit) → accelerates slowly to per-frame ΔV=J/mass → lags behind (realistic).
	// impulse is the center of gravity (AddImpulse without position), so there is no torque/spin. bVelChange=false = actual impulse (divided by mass).
	const float VTarget = FMath::Max(0.0f, HoldConfig.ActivePullMaxLinearSpeed);
	if (!Prim || VTarget <= 0.0f || MaxTension <= KINDA_SMALL_NUMBER || Dir.IsNearlyZero() || DeltaTime <= 0.0f)
	{
		return;
	}
	// Acceleration only (no reverse thrust) + accurate arrival (Alpha=1). The cap is impulse (tension × dt) — it is the same framework as the tether reel.
	// The only difference is whether there is a positive direction (the tether even brakes to settle the boundary, but the active pull only accelerates because the user releases it).
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
	// Angular velocity cap (residual spin safety net): The cause of pull torque is removed by applying force to the center of gravity, but the remaining ragdoll joint dynamics
	// Contains the spin. (Linear traction is handled by the tension cap impulse of ApplyPullVelocityDrive — here, only angular velocity.)
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
	// Check(climb-in direction/distribution observation shared) of this Wrapped frame. Calculate each frame regardless of overshoot
	// Causes tether retrieval (UpdateTether) and active Pull direction (ApplyWrappedTraction) to read the same check.
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

	// A rope (owner==object) Wrapped around itself is meaningless in distribution → the target is always pullable.
	const bool bSelfWrap = (GetOwner() != nullptr && MeshComp->GetOwner() == GetOwner());
	bool bPullable;
	if (bSelfWrap)
	{
		bPullable = true;
	}
	else
	{
		// Effective mass at both ends (ground character reflects ground friction with GroundBraceFactor, MOVE_None/static is anchor = infinite).
		// Uses the same interpretation (ResolveTetherEndpoint) as for tether application — there is no discrepancy between the attraction check and the actual application point.
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
			// hysteresis is a non-exposed internal constant (stabilizer) — it prevents checks on boundaries from flipping every frame.
			// There is only one exposure knob that determines the "intersection location", GroundBraceFactor, and this value is just a deadband around that line.
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
		// Currently "can be dragged": only flips to non-sticky when the target becomes more than Margin times heavier than the wielder.
		return !(EffMassTarget > EffMassWielder * Margin);
	}
	// Currently "Unable to be dragged": Flipped to possible only when the target becomes lighter than the wielder × (1/Margin).
	return (EffMassTarget * Margin <= EffMassWielder);
}

void URopeComponent::ApplyPullForceToWielder(const FVector& Force, float DeltaTime)
{
	// (not pullable) Apply an active Pull force to the wielder (rope owner) — the target is heavy instead.
	// climb-in, where the wielder is pulled towards the anchor. Mirror the owner side of ApplyPullForce: CharacterMovement → Simulate Root.
	AActor* RopeOwner = GetOwner();
	if (!RopeOwner)
	{
		return;
	}

	// The receiver is **interpreted** first — this is to pass “what almost got stuck” to the expansion gateway as is.
	// The analysis order is the same as before (CharacterMovement → Simulation Root): from here to ResolveTetherEndpoint ladder.
	// If you change, the sim route will be ahead of the movement, changing the climb-in action.
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
		// No recipients (non-character + non-sim route): Quiet drop — no climb-in configuration.
		return;
	}
}

#pragma endregion Tether_And_Pull_Application

