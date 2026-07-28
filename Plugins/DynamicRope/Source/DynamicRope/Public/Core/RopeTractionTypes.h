// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"

class USceneComponent;
class UPrimitiveComponent;
class UCharacterMovementComponent;
class AActor;

/**
 * Type of traction recipient — single check result of "what receives the rope force" (RopeComponent.cpp ResolveTetherEndpoint).
 * This is the result of combining the tether and active pull's separate ladder steps into one analysis, so the mass distribution and actual application point are
 * It is structurally impossible to misalign. The extension hook (ApplyTractionToReceiver) is also used to describe the receiver.
 */
enum class ERopeEndpointKind : uint8
{
	None,      // No recipient (no owner).
	SimBody,   // Physics Simulation Body: Skeletal promotion bone / target primitive / owning root.
	Character, // The character on which the CMC is actually running (except MOVE_None).
	Anchor,    // static/Kinematic/MOVE_None/Non-simulation Non-character — infinite mass (only positional fallback for movement).
};

/**
 * Receiver interpretation results shared by Tether/Active Pull. A UObject pointer is only consumed for one GT frame and is not owned.
 * Confirm the type, actual application body, and basic effective mass at once so that check and application do not see different endpoints.
 * SimBody's analytic material solver uses this mass after the actual world attachment point and direction are determined.
 * Refine to point Jacobian (translation + rotation).
 */
struct FRopeTetherEndpoint
{
	ERopeEndpointKind Kind = ERopeEndpointKind::None;
	UPrimitiveComponent* Prim = nullptr;
	FName Bone = NAME_None;
	UCharacterMovementComponent* Movement = nullptr;
	AActor* Actor = nullptr;
	float Mass = 0.0f;
};

/** Wrapped A non-owning cache that shares target/wielder endpoint resolution for one frame.*/
struct FRopeResolvedWrappedEndpoints
{
	FRopeTetherEndpoint Target;
	FRopeTetherEndpoint Wielder;
	TWeakObjectPtr<USceneComponent> TargetMesh;
	FName TargetBone = NAME_None;
	bool bValid = false;

	void Reset() { *this = FRopeResolvedWrappedEndpoints(); }
};

/** Which traction path does this permission come from — allows subclasses to react differently depending on the path.*/
enum class ERopeTractionSource : uint8
{
	/** Automatic traction: Number of times the λ impulse constraint is applied to both ends (UpdateConstraintTether).*/
	Tether,
	/** User input Active Pull (constant force). Includes both target accreditation and climb-in (wielder accreditation).*/
	ActivePull,
};

/**
 * Request description (POD, non-owning pointer — valid only during the call) **just before** rope grants traction to the receiver.
 * URopeComponent::ApplyTractionToReceiver is the only type that receives this, and any force/velocity interventions the rope makes are
 * is described as one type (path is classified by Source).
 */
struct FRopeTractionRequest
{
	ERopeTractionSource Source = ERopeTractionSource::Tether;
	ERopeEndpointKind ReceiverKind = ERopeEndpointKind::None;

	/** SimBody: Primitive and bone to be approved (if bone is not present, component unit).*/
	UPrimitiveComponent* Prim = nullptr;
	FName Bone = NAME_None;

	/** Character: Movement to be approved.*/
	UCharacterMovementComponent* Movement = nullptr;

	/** Recipient owns actor (Kind irrelevant, filled in if present). Custom movements usually find their components here.*/
	AActor* Actor = nullptr;

	/** Is direction (unit vector).*/
	FVector Direction = FVector::ZeroVector;

	/**
	 * Size by source — The units are different, so be sure to read it together with the source.
	 *   Tether = Change in axis velocity this frame ΔV = λ×effective inverse mass (cm/s)
	 *   ActivePull = magnitude of force = tension cap(N)
	 */
	float Amount = 0.0f;

	float DeltaTime = 0.0f;

	/** Is it from the wielder (rope owner) side? If false, the wound target side.*/
	bool bWielderSide = false;
};

/**
 * Pull sample: Describes the pull that the wrap anchor receives from the rope as data (Docs/PoC/01_PostWrapModel.md 4.2).
 * FRopeWrapController::ComputePull fills (UObject-Free), and the component converts to force application (character/physics bone).
 */
struct FRopePullSample
{
	bool    bValid = false;

	/** First anchor node on the hand side (node ​​at the point of force application).*/
	int32   AnchorNode = INDEX_NONE;

	/** End of first straight leg (integer node where walk stops) — raw aiming in direction (computePull output; debug/diagnostics).*/
	int32   AimNode = INDEX_NONE;

	/** Anchored bone (physical bone force application target).*/
	FName   Bone = NAME_None;

	/** Anchor node world location (force application point).*/
	FVector WorldPoint = FVector::ZeroVector;

	/** Pull unit direction (aiming side from anchor = following rope path; component when consumed is fractional+EMA smoothing).*/
	FVector Direction = FVector::ZeroVector;

	/** Anchor-hand side adjacent segment tension (units of FRopeSimState::SegmentTension).*/
	float   Tension = 0.0f;

	/**
	 * smoothed fractional aiming index([0, AnchorNode); <0 = not set). With AimPos, consumers (components)
	 * Fill the AimNode with float time smoothing (ComputePull only yields integer AimNode) — tether/direction is this continuous
	 * to eliminate discrete hops (direction jump + traction interruption) between frames of the integer aiming node.
	 */
	float   AimNodeF = -1.0f;

	/** Aiming world position interpolated between nodes (AimNodeF position).*/
	FVector AimPos = FVector::ZeroVector;

	/**
	 * Anchor→Hand corner-leg chord sum (cm). Instead of stopping at the first corner, continue walking to the hand (node 0) and walk in a straight line on each leg.
	 * Accumulated distance — Comparison with FreeRestLen is the observed "total chain tension" check (RopeTraction::
	 * EvaluateChainTautGate). Sag makes the chord shorter than the rest, and the taut rope hanging at the corner makes the chord for each leg
	 * It is considered taut because it is close to rest. **Chords for each leg are clamped to the rest length of that leg** — moving
	 * When the anchor stretches the leg on the anchor side (segment > rest), the chord exceeds the rest, causing slack in the remaining rope.
	 * Prevents offset and concealment (PIE actual measurement 620/600cm case). However, if the slack is crumpled in a zigzag pattern, the legs are split into small pieces.
	 * There is still a blind spot attached to rest — that is covered by the MinFreeTension gate.
	 *
	 * ⚠ The chord defect is only reduced by the **square** of the sag (chord 590 on a 600cm rope = ~45cm visible sag) —
	 * The checker for “visually stretched” is MaxLegSag (cm, linear), not this ratio. This value corresponds to a moderate large sag and
	 * remains as a rough backstop for compression (node agglomeration).
	 */
	float   TautChordLen = 0.0f;

	/**
	 * Anchor→Hand Corner-Leg Chord Sum **Non-Clamp** Value (cm) — Unlike TautChordLen, rest clamp is not performed for each leg.
	 * No, stretched legs increase the sum. legacy/custom-mover without live movement binding
	 * remains only as a diagnostic observation for fallback. Authoritative C is live hand↔first-anchor distance − material
	 * length.
	 *
	 * ⚠ In legacy fallback, C > 0 is not enough: ragdoll bone fluctuation
	 * If only the leg adjacent to the anchor is stretched to the strain limit, the sum will exceed the rest even if the rest is stretched, causing the slack rope to
	 * C > 0 (partial stretch contamination). When λ ignites in that false C, a positive return runaway of traction → oscillation → stretch occurs.
	 * , so only the fallback maintains the geometry/legacy tension contamination guard.
	 */
	float   PathChordLen = 0.0f;

	/** Free span (hand~anchor) rest length(cm) = AnchorNode × SegmentLength (automatically reflects rewrapping reduction).*/
	float   FreeRestLen = 0.0f;

	/**
	 * **Minimum value** of Free span (hand~anchor) segment tension (unit of FRopeSimState::SegmentTension). The tight rope is
	 * Since tension is transmitted to the entire section from the anchor to the hand, the minimum value is a positive number, and if there is slack in even one section,
	 * (Compression/Creasing — XPBD tension only counts tension) is 0 — Zigzag slack/partial stretch that the chord sum geometry cannot see.
	 * This is what determines it. If it is not yet solved (array is empty), it is 0 (GPU rope is a 1~2 frame delayed mirror).
	 */
	float   MinFreeTension = 0.0f;

	/**
	 * Maximum sag (cm) for each leg = Maximum vertical distance that the internal node of each corner-leg deviates from the chord straight line of that leg.
	 * Direct observation of "visually straightened" — **linear** to sag cm, unlike chord rate (which only responds to the square of sag)
	 * responds (PIE actual measurements: actual sag of rope ~45cm with chord 590/600 (98.3%)). Since it is a leg unit, there is no tension hanging at the corner.
	 * The rope (straightness of each leg) has a small value, and the gentle catenary sag is revealed as it is in cm.
	 */
	float   MaxLegSag = 0.0f;
};
