// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/WeakObjectPtrTemplates.h"

class USceneComponent;
class UPrimitiveComponent;
class UCharacterMovementComponent;
class AActor;

/**
 * What kind of thing receives the rope's traction — the single resolution of "who takes the force"
 * (ResolveTetherEndpoint). Folding the tether's and the active pull's separate ladders into one resolution
 * is what makes it structurally impossible for the mass distribution and the actual application point to
 * disagree. The extension hook (ApplyTractionToReceiver) uses it to describe the receiver too.
 */
enum class ERopeEndpointKind : uint8
{
	None,      // No receiver — there is no owner.
	SimBody,   // A simulating physics body: a promoted skeletal bone, the target primitive, or the owning root.
	Character, // A character whose CharacterMovement is actually running (anything but MOVE_None).
	Anchor,    // Static, kinematic, MOVE_None, or a non-simulating non-character — infinite mass, so only a positional fallback can move it.
};

/**
 * The endpoint resolution the tether and the active pull share. The UObject pointers are borrowed for one
 * game-thread frame and never owned.
 * Resolving the kind, the body force is actually applied to, and the base effective mass all at once is what
 * keeps the gate and the application from looking at different endpoints.
 * For a SimBody, the analytic material solver refines this mass into a point Jacobian — translation plus
 * rotation — once the real world attachment point and direction are known.
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

/** Non-owning cache that shares one frame's target and wielder endpoint resolution across a Wrapped frame. */
struct FRopeResolvedWrappedEndpoints
{
	FRopeTetherEndpoint Target;
	FRopeTetherEndpoint Wielder;
	TWeakObjectPtr<USceneComponent> TargetMesh;
	FName TargetBone = NAME_None;
	bool bValid = false;

	void Reset() { *this = FRopeResolvedWrappedEndpoints(); }
};

/** Which traction path a request came from, so a subclass can react differently per path. */
enum class ERopeTractionSource : uint8
{
	/** Automatic traction: the λ impulse constraint applied as a pair at both ends (UpdateConstraintTether). */
	Tether,
	/** Active pull from user input, a constant force. Covers both the target and climb-in on the wielder. */
	ActivePull,
};

/**
 * What the rope is about to apply to a receiver (POD; the pointers are borrowed and valid only for the call).
 * URopeComponent::ApplyTractionToReceiver is its only consumer, and every force or velocity the rope injects
 * is described by this one type, with Source telling the paths apart.
 */
struct FRopeTractionRequest
{
	ERopeTractionSource Source = ERopeTractionSource::Tether;
	ERopeEndpointKind ReceiverKind = ERopeEndpointKind::None;

	/** SimBody: the primitive and bone to apply to. With no bone, the whole component. */
	UPrimitiveComponent* Prim = nullptr;
	FName Bone = NAME_None;

	/** Character: the movement component to apply to. */
	UCharacterMovementComponent* Movement = nullptr;

	/** The receiver's owning actor, filled in whenever there is one regardless of kind. Custom movement usually finds its components from here. */
	AActor* Actor = nullptr;

	/** Pull direction (unit vector). */
	FVector Direction = FVector::ZeroVector;

	/**
	 * Magnitude, whose units depend on Source — always read the two together.
	 *   Tether     = this frame's change in axial velocity, ΔV = λ × effective inverse mass (cm/s)
	 *   ActivePull = force magnitude, the tension cap (N)
	 */
	float Amount = 0.0f;

	float DeltaTime = 0.0f;

	/** Is this the wielder (rope owner) side? False means the wrapped target side. */
	bool bWielderSide = false;
};

/**
 * The pull a wrap anchor takes from the rope, expressed as data. FRopeWrapController::ComputePull fills it
 * without touching a UObject, and the component turns it into force on a character or a physics bone.
 */
struct FRopePullSample
{
	bool    bValid = false;

	/** First anchor node on the hand side — where the force is applied. */
	int32   AnchorNode = INDEX_NONE;

	/** End of the first straight leg, where the walk stopped. The raw integer aim direction from ComputePull; diagnostic. */
	int32   AimNode = INDEX_NONE;

	/** The anchored bone, which is the physics body force is applied to. */
	FName   Bone = NAME_None;

	/** World position of the anchor node — the point force is applied at. */
	FVector WorldPoint = FVector::ZeroVector;

	/** Unit pull direction, from the anchor toward the aim node along the rope path. The component smooths it (fractional index, then EMA) before use. */
	FVector Direction = FVector::ZeroVector;

	/** Tension in the segment next to the anchor on the hand side, in FRopeSimState::SegmentTension units. */
	float   Tension = 0.0f;

	/**
	 * Smoothed fractional aim index, in [0, AnchorNode); below 0 means unset. The component fills this and
	 * AimPos by smoothing AimNode over time as a float, because ComputePull only yields an integer AimNode.
	 * This continuous index is what removes the discrete hop between frames — a jumping direction and an
	 * interrupted traction — that an integer aim node produces.
	 */
	float   AimNodeF = -1.0f;

	/** Aim position interpolated between nodes, at AimNodeF. */
	FVector AimPos = FVector::ZeroVector;

	/**
	 * Sum of the straight chords of each corner leg from the anchor to the hand (cm). Rather than stopping at
	 * the first corner, the walk continues to the hand (node 0) and accumulates a straight distance per leg.
	 * Comparing it against FreeRestLen is the whole-chain taut gate (RopeTraction::EvaluateChainTautGate): sag
	 * makes a chord shorter than its rest length, while a taut rope hanging over a corner keeps every leg's
	 * chord close to rest and so reads as taut.
	 * **Each leg's chord is clamped to that leg's rest length.** Without the clamp, a moving anchor stretching
	 * the leg beside it (segment beyond rest) pushes that chord past rest, and the excess offsets and hides
	 * slack elsewhere in the rope — measured at 620 against 600 cm in play.
	 * A blind spot remains: slack crumpled into a zigzag splits into short legs that each sit near their rest
	 * length. The MinFreeTension gate is what covers that.
	 *
	 * ⚠ The chord deficit only grows with the **square** of the sag — a 590 cm chord on a 600 cm rope is
	 * roughly 45 cm of visible sag. What actually measures "looks straight" is MaxLegSag, in linear cm, not
	 * this ratio. This value covers a moderately large sag and stands as a rough backstop against compression,
	 * where nodes bunch up.
	 */
	float   TautChordLen = 0.0f;

	/**
	 * The same anchor-to-hand corner-leg chord sum **without the per-leg clamp** (cm), so a stretched leg
	 * inflates it. It survives only as a diagnostic for the legacy and custom-mover fallback, which has no
	 * live movement binding. The authoritative violation C is the live hand-to-first-anchor distance minus the
	 * material length.
	 *
	 * ⚠ On the legacy fallback, C > 0 alone is not enough. A ragdoll bone jittering can stretch the leg beside
	 * the anchor to the strain limit on its own, which pushes the sum past rest and reports C > 0 for a rope
	 * that is actually slack — a partial stretch contaminating the reading. λ igniting on that false C is a
	 * positive feedback runaway: traction, oscillation, more stretch. That is why the fallback alone keeps the
	 * geometry and legacy tension contamination guards.
	 */
	float   PathChordLen = 0.0f;

	/** Rest length of the free span from hand to anchor (cm) = AnchorNode × SegmentLength, so reeling shortens it automatically. */
	float   FreeRestLen = 0.0f;

	/**
	 * **Lowest** segment tension across the free span from hand to anchor, in FRopeSimState::SegmentTension
	 * units. A taut rope carries tension along the whole span, so the minimum is positive; a single slack
	 * stretch — compressed or crumpled, since XPBD counts tension only — drives it to 0. That is what catches
	 * the zigzag slack and partial stretch the chord-sum geometry cannot see. 0 as well when the rope has
	 * never solved and the array is empty (a GPU rope mirrors one to two frames late).
	 */
	float   MinFreeTension = 0.0f;

	/**
	 * Largest sag across the legs (cm): the furthest an interior node of any corner leg strays from that leg's
	 * chord. This is the direct measure of "looks straight", and unlike the chord ratio — which only responds
	 * to the square of the sag — it responds **linearly** in cm. Measured in play: a chord ratio of 98.3%
	 * (590 of 600) alongside about 45 cm of real sag. Being per leg, a taut rope hanging over a corner scores
	 * low, while a gentle catenary shows up at its true depth in cm.
	 */
	float   MaxLegSag = 0.0f;
};
