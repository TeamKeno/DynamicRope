// Copyright Epic Games, Inc. All Rights Reserved.
//
// The shared, UObject-free maths of the traction axis drive, covering both the tether and active
// pull. Every case, that is both tether modes at both ends plus active pull, has the same skeleton:
// servo the velocity along the rope axis towards a target. Only the target, the approach and the
// limits differ, so those differences are described by FRopeAxisServo and the arithmetic is written
// once, here. It can be unit tested without a world; see Tests/RopeTractionSolverTests.cpp.
//
// What is deliberately absent is application, that is which UObject API the result is fed into. The
// meaning differs per receiver and cannot be unified: on a character movement component in
// particular, setting Movement->Velocity directly takes effect this frame while
// Movement->AddImpulse takes effect on the next tick, through ApplyAccumulatedForces, so the two are
// not interchangeable. The caller, URopeComponent, takes only the delta velocity or impulse from
// here and applies it the way its own receiver requires.

#pragma once

#include "CoreMinimal.h"

namespace RopeTraction
{
	/**
	 * The specification of an axis velocity servo. It deals only with the component along the rope
	 * axis, which is the direction of application; the orthogonal components, such as gravity and
	 * swing, are preserved by the caller.
	 */
	struct FRopeAxisServo
	{
		/** Target axis speed (cm/s), positive along the direction of application. */
		float TargetSpeed = 0.0f;
		/** How much of the remaining gap to close this frame, from 0 to 1. 1 reaches the target exactly
		 *  with no damping; smaller values approach it smoothly over several frames. */
		float Alpha = 1.0f;
		/** False accelerates only, applying force when short of the target and never slowing down. True
		 *  also brakes, removing momentum beyond the target, which prevents coasting and overshoot. */
		bool bBidirectional = false;
		/** True cancels any outward, that is negative, axis velocity to zero first and then approaches
		 *  the target. The cancellation is immediate and complete, and ignores Alpha; it exists for
		 *  cancelling a character movement component's walking velocity. */
		bool bCancelOutward = false;
	};

	/**
	 * The axis delta velocity to apply this frame (cm/s). 0 means do nothing, which is how the caller
	 * decides to skip.
	 * A one-directional servo, with bBidirectional false, returns 0 once at or beyond the target so it
	 * never thrusts backwards or brakes.
	 */
	DYNAMICROPE_API float ComputeAxisDeltaV(float CurAlong, const FRopeAxisServo& Servo);

	/**
	 * Turns a delta velocity into an actual impulse: J = mass * deltaV, clamped symmetrically to plus
	 * or minus MaxImpulse when a limit is set, so acceleration and braking are treated alike.
	 * MaxImpulse is the maximum tension multiplied by dt. 0 means unlimited, which gives an exact servo
	 * that produces the requested delta velocity regardless of mass.
	 * The threshold tension at which a mass M reaches a speed V within one frame is approximately
	 * M * V * fps; below that the receiver lags behind, which reads as weight.
	 */
	DYNAMICROPE_API float ClampAxisImpulse(float DeltaV, float Mass, float MaxImpulse);

	/**
	 * Caps the absolute speed after a velocity injection at the larger of SpeedCap and the existing
	 * speed. If the direction wobbles, injections land on a different axis each frame and the vector
	 * can keep growing while falling, where nothing damps it; this is the second line of defence
	 * against that runaway, the first being the direction EMA.
	 * External motion that is already faster, such as free fall, is preserved. A SpeedCap of 0 disables
	 * the clamp and passes the value through.
	 */
	DYNAMICROPE_API FVector ClampInjectedVelocity(const FVector& NewVel, const FVector& OldVel, float SpeedCap);

	/** Effective inverse mass, w = 1 / effective mass. A mass of 0, or near it, is an anchor of
	 *  infinite mass and gives 0. */
	DYNAMICROPE_API float InvMassFromMass(float Mass);

	/**
	 * The instantaneous mass properties of a rigid body receiving a rope impulse at a world point.
	 * InertiaTensor holds the diagonal components in the MassSpaceToWorld rotation frame.
	 */
	struct FRopePointMassProperties
	{
		float Mass = 0.0f;
		FVector InertiaTensor = FVector::ZeroVector;
		FTransform MassSpaceToWorld = FTransform::Identity;
	};

	/** The instantaneous velocity change K*J at a world point when ImpulseWorld is applied at that same
	 *  point. */
	DYNAMICROPE_API FVector ComputePointVelocityDelta(
		const FRopePointMassProperties& Body,
		const FVector& PointWorld,
		const FVector& ImpulseWorld);

	/** The scalar rope Jacobian J M^-1 J^T for a world point and direction. */
	DYNAMICROPE_API float ComputePointInverseMass(
		const FRopePointMassProperties& Body,
		const FVector& PointWorld,
		const FVector& DirectionWorld);

	/**
	 * The framerate-independent exponential smoothing coefficient alpha = 1 - exp(-dt / Tau). However
	 * large dt grows, alpha stays at or below 1 so it never overshoots, and it converges on the same
	 * time constant of Tau seconds at any framerate; a constant alpha would make the response depend on
	 * the framerate instead.
	 * A Tau at or below 0 disables smoothing, giving alpha = 1 and reaching the target in one frame.
	 */
	DYNAMICROPE_API float ExpSmoothAlpha(float Tau, float DeltaTime);

	/**
	 * An exponential moving average over a direction, for unit vectors only. If the traction direction
	 * jumps around between frames, the clamps and top-ups land on a different axis each time and the
	 * vector grows by random walk, which is the runaway this guards against as the first line of
	 * defence; the second is the speed cap in ClampInjectedVelocity.
	 *  - When Current is near zero, meaning unseeded, it is seeded from Target so the first valid frame
	 *    has no lag.
	 *  - Otherwise it interpolates and renormalizes. If a 180 degree reversal makes the interpolation
	 *    cancel exactly to zero, it reseeds from Target: without that, the direction would stay at zero
	 *    and the axis would vanish, leaving the caller relying on its fallback.
	 * Target is assumed to be a unit vector, normalized by the caller.
	 */
	DYNAMICROPE_API FVector SmoothDirection(const FVector& Current, const FVector& Target, float Alpha);

	/**
	 * A fractional aim position, interpolating linearly between aim nodes. Using the integer aim node
	 * directly makes the direction jump wholesale as it hops discretely between frames, and the
	 * overshoot step in node-sized jumps, which reads as traction stuttering; this makes it continuous.
	 * AimF is clamped by the caller to the range from 0 to AnchorNode. An index outside the range
	 * returns a zero vector.
	 */
	DYNAMICROPE_API FVector SampleFractionalAim(const TArray<FVector>& Positions, float AimF, int32 AnchorNode);

	/**
	 * The taut gate for active pull, as a hysteresis latch. Tautness is defined by tension, derived
	 * from the XPBD lambda; the tether overshoot is geometric and is not used here.
	 * A threshold at or below 0, the default, treats any tension at all as taut. Above 0, engaging
	 * requires exceeding the threshold while staying engaged, with bWasTaut set, only requires
	 * exceeding the threshold multiplied by ReleaseRatio, which stops tension jitter at the boundary
	 * from flapping the gate on and off. ReleaseRatio is clamped to the range 0 to 1.
	 */
	DYNAMICROPE_API bool EvaluateTautGate(float Tension, float Threshold, float ReleaseRatio, bool bWasTaut);

	/**
	 * The whole-chain geometric taut gate, as a hysteresis latch. The rope counts as taut along its
	 * whole length once the sum of the corner-to-corner leg chords from the anchor to the hand
	 * (ChordLen) reaches the free-span rest length (RestLen) multiplied by (1 - SlackRatio). Sag makes a
	 * leg's chord shorter than its rest length, while a taut rope running over corners has each leg's
	 * chord close to its rest length and is correctly recognized as taut, so corners are not penalized.
	 * Local observations near the anchor, such as segment tension or a sub-leg overshoot, cannot answer
	 * whether the whole rope is straight, because a moving target produces them even on a slack rope
	 * when a pinned node momentarily stretches its neighbours; this gate complements them.
	 * Staying taut, with bWasTaut set, relaxes the requirement by SlackRatio multiplied by ReleaseScale,
	 * which is at least 1, so chord jitter at the boundary does not flap the gate. A RestLen at or below
	 * 0 returns false, meaning no verdict is possible.
	 */
	DYNAMICROPE_API bool EvaluateChainTautGate(float ChordLen, float RestLen, float SlackRatio, float ReleaseScale, bool bWasTaut);

	/**
	 * The inputs to the tether lambda constraint, in centimetres, kilograms and seconds; see
	 * SolveTetherLambda.
	 */
	struct FRopeTetherConstraint
	{
		/**
		 * The violation C, that is the required path length minus the material length (cm). Below 0
		 * means slack.
		 * C == 0 is the boundary, so an inextensible reaction lambda still arises there when SepSpeed is
		 * positive.
		 */
		float C = 0.0f;

		/**
		 * The separation speed (cm/s, positive while separating), equal to -(vT.dT + vW.dW) - dRest/dt.
		 * vT and vW are the velocities of the two ends and dT and dW are the inward unit directions at
		 * each end, meaning towards the other along the rope. Reeling in shrinks the rest length, making
		 * dRest/dt negative, which enters as a positive contribution so lambda pulls by that much: the
		 * reel is expressed purely as a change in rest length with no separate traction path.
		 */
		float SepSpeed = 0.0f;

		/** The effective inverse mass at each end (1/kg). 0 is an anchor of infinite mass, and that end
		 *  receives no delta velocity. */
		float InvMassTarget = 0.0f;
		float InvMassWielder = 0.0f;

		/**
		 * The positional recovery gain beta, from 0 to 1, which commands an approach speed closing this
		 * fraction of C during this frame. Compute it with ExpSmoothAlpha(TetherSettleTime, dt) so it is
		 * framerate independent. 0 leaves a velocity-only constraint, cancelling separation without
		 * closing any C already accumulated, which permits drift.
		 */
		float SettleAlpha = 1.0f;

		/**
		 * The cap on the commanded positional recovery speed (cm/s, 0 for unlimited). beta * C / dt
		 * spikes on frames where C is large, such as immediately after a commit that is already
		 * violated. This is the one term that leaves a bias in the momentum, unlike cancelling
		 * separation, so the cap is also the ceiling on coasting after the rope goes slack, which makes
		 * it the maximum approach speed the tether can ever produce.
		 */
		float MaxBiasSpeed = 0.0f;

		/**
		 * Material compliance alpha (s^2/kg, the inverse of stiffness). 0 is inextensible, the default.
		 * Above 0 it uses an implicit Kelvin-Voigt tension with k = 1/alpha and generalized critical
		 * damping.
		 */
		float Compliance = 0.0f;

		/** The tension limit (kg*cm/s^2, 0 for unlimited): lambda is capped at this value multiplied by
		 *  dt. This is the physical knob behind a heavy target lagging behind and behind limited-tension
		 *  behaviour generally. */
		float MaxTension = 0.0f;
	};

	/**
	 * Solves the tether lambda constraint, once per frame and analytically with no iteration. It
	 * returns the tension impulse lambda, which is at or above 0 (kg*cm/s).
	 * Applying it is the caller's job: each end receives dv = d * (lambda * w). Because it is a pair of
	 * equal and opposite impulses, the distribution follows automatically, with the heavier end moving
	 * less and an anchor not at all, and because it produces only relative approach it injects no
	 * energy, unlike independent per-end servos; the bias term is the sole exception, described on
	 * MaxBiasSpeed. Its properties are:
	 *  - One-directional: it returns 0 when slack, with C below 0, or when already approaching faster
	 *    than the target, so the rope neither pushes nor brakes an approach. Exactly at the boundary,
	 *    with C == 0 and still separating, an inextensible reaction arises.
	 *  - It returns 0 when both ends are anchors, that is the sum of the inverse masses is near zero,
	 *    because nothing can move; exceeding the limit is handled by the distance release instead.
	 *  - When the MaxTension limit binds, the remaining C carries over to the next frame, which is what
	 *    makes a heavy target lag behind.
	 * Unit tests: the TetherLambda family in Tests/RopeTractionSolverTests.cpp.
	 */
	DYNAMICROPE_API float SolveTetherLambda(const FRopeTetherConstraint& In, float DeltaTime);
}
