// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The traction and smoothing state used while wrapped. It gathers into one type the state that
// persists between frames and is read and updated by two of the four stages of a wrapped tick:
// producing the observations (UpdateWrappedPullSample) and applying traction, that is the tether and
// active pull (ApplyWrappedTraction). The logic stays on URopeComponent, which needs UObject access
// and calls the extension hooks; this holds only state and the reset contract.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTractionTypes.h"

struct FRopePullDriveState
{
	/**
	 * This frame's pull output, produced every frame while wrapped. It is the source for Blueprint
	 * queries and the debugger's arrow.
	 * Direction is overwritten each frame with SmoothedPullDir below, so consumers, meaning the tether
	 * and active pull, use the time-smoothed value.
	 */
	FRopePullSample LastPullSample;

	/**
	 * The time-smoothed pull direction, as an exponential moving average. It smooths the look-ahead
	 * direction from ComputePull, which is already a spatial average, across frames to absorb residual
	 * jitter and the noise of the delayed GPU mirror. A zero vector means uninitialized and is seeded
	 * from the measurement on the first valid frame after a wrap starts. Reset in ResetTransient. The
	 * tether and active pull share this direction.
	 */
	FVector SmoothedPullDir = FVector::ZeroVector;

	/**
	 * The time-smoothed wielder traction direction, from the hand at node 0 towards the rope's first
	 * leg, as an exponential moving average using the same time constant as SmoothedPullDir. A zero
	 * vector means uninitialized and is seeded on the first valid frame; reset in ResetTransient.
	 * If the direction jumps around between frames, the velocity top-up lands on a different axis each
	 * time and the vector grows by random walk, so stabilizing the direction is the first line of
	 * defence against that runaway; the speed cap in ClampInjectedVelocity is the second.
	 */
	FVector SmoothedWielderPullDir = FVector::ZeroVector;

	/** The look-ahead direction before smoothing, that is the raw input to the moving average. The
	 *  debugger draws raw and smoothed side by side to diagnose jitter. */
	FVector LastPullDirRaw = FVector::ZeroVector;

	/**
	 * The time-smoothed pull aim node, kept fractional. The integer aim node chosen by ComputePull is
	 * averaged as a float so it interpolates between nodes, which makes the direction and the tether
	 * continuous and removes the discrete hops. Below 0 means uninitialized and is seeded on the first
	 * valid frame after a wrap starts; reset to -1 in ResetTransient. The time constant is
	 * PullAimSmoothTime.
	 */
	float SmoothedAimNodeF = -1.0f;

	/** The current active pull force, set by SetActivePull, where 0 is off. It is only applied while
	 *  wrapped and taut, subject to the bPullTaut gate below. */
	float ActivePullForce = 0.0f;

	/**
	 * Whether this active pull ignores the taut gate, taken from the per-call argument to
	 * SetActivePull. When true it is applied on a valid sample alone, regardless of the
	 * bActivePullRequiresTaut setting, which exists for the scripted section of an animation pull
	 * window.
	 * It is input state like ActivePullForce, so ResetTransient leaves it alone; clearing it is
	 * SetActivePull's job.
	 */
	bool bActivePullIgnoresTaut = false;

	/**
	 * This frame's taut gate state, as a hysteresis latch from RopeTraction::EvaluateTautGate.
	 * UpdateWrappedPullSample refreshes it every wrapped frame, and both the application of active pull
	 * in ApplyWrappedTraction and URopeComponent::IsPullTaut() read it. It is false whenever there is
	 * no valid pull sample, which includes not being wrapped. Reset in ResetTransient.
	 */
	bool bPullTaut = false;

	/**
	 * This frame's whole-chain geometric taut gate state, as a hysteresis latch from
	 * RopeTraction::EvaluateChainTautGate. It compares the sum of the corner-to-corner leg chords from
	 * the anchor to the hand against the free-span rest length to decide whether the whole rope is
	 * straight.
	 * UpdateWrappedPullSample refreshes it every wrapped frame, and applying traction, both the tether
	 * and active pull, reads it as a precondition: local observations such as tension near the anchor
	 * or a sub-leg overshoot occur even on a slack rope, so nothing is pulled while this gate is
	 * closed. bPullTaut is this value combined with the tension threshold. Reset in ResetTransient.
	 */
	bool bChainTaut = false;

	/**
	 * The time remaining on the grace period before bChainTaut releases (s); the constant is
	 * HoldConfig.TautReleaseGraceTime. It is refilled on every frame the taut condition holds, and once
	 * the condition breaks the latch is kept until it drains. That stops per-frame chatter in the
	 * minimum transmitted tension observation, caused by a threshold of 0, meaning no hysteresis,
	 * combined with the delayed GPU mirror, from leaking out as an alternation between fully damped and
	 * free that shows up as the wielder juddering.
	 * Reset in ResetTransient.
	 */
	float TautGraceRemaining = 0.0f;

	// Passive material-length reaction state (violation/lambda/attempted movement/rest rate)
	// lives in FRopeLengthConstraintState. PullDrive owns only pull command, direction and
	// receiver-policy state.

	/**
	 * This frame's effective target share, from 0 to 1, read by the wielder gate
	 * (URopeWielderComponent::IsWielderTetherActive) and by the debugger. A value of 1 leaves the
	 * wielder no share at all. UpdateTargetPullable fills it in with the binary result of the drag
	 * test, 1 for pullable and 0 for not, and on frames where lambda actually fired,
	 * UpdateConstraintTether overwrites it with the inverse mass ratio w_t divided by the sum of the
	 * inverse masses.
	 */
	float LastTargetShare = 1.0f;

	/**
	 * The sticky verdict on whether the target can be dragged, which is the authority on the direction
	 * of an active pull climb-in: true when the target's effective mass is at or below the wielder's.
	 * While bTargetPullableInit is false it is unseeded and will be seeded on the next valid frame by a
	 * plain comparison with no hysteresis; once seeded it only flips through its internal hysteresis.
	 * ResetTransient returns it to the unseeded state.
	 */
	bool bTargetPullable = true;
	bool bTargetPullableInit = false;

	/** A latch that limits the "no pull force receiver" warning to once per wrap. Reset in
	 *  ResetTransient. */
	bool bLoggedPullNoReceiver = false;

	/**
	 * Resets only the transient "wrap in progress" state, which is discarded on a phase transition.
	 * Called by URopeComponent::ResetTransientPhaseState.
	 * Deliberately preserved: ActivePullForce and bActivePullIgnoresTaut, which are held input state
	 * cleared by SetActivePull(0); and LastPullDirRaw, which is a leftover for the debugger's display
	 * and is overwritten by the next wrapped frame.
	 */
	void ResetTransient()
	{
		LastPullSample = FRopePullSample();
		// Return the smoothing state to its uninitialized value so the next wrap reseeds it from a
		// fresh measurement.
		SmoothedPullDir = FVector::ZeroVector;
		SmoothedWielderPullDir = FVector::ZeroVector;
		SmoothedAimNodeF = -1.0f;
		bTargetPullableInit = false; // Reseed from a plain comparison when the next wrap starts.
		bPullTaut = false;
		bChainTaut = false;
		TautGraceRemaining = 0.0f;
		bLoggedPullNoReceiver = false;
	}
};
