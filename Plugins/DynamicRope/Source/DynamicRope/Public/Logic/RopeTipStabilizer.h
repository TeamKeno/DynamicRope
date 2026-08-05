// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Presentation-side stabilizer for the tip attachment while it follows the rope's free end.
// XPBD contact resolution leaves millimetre-scale residual motion on a rope at rest — the hand pin
// keeps moving with the wielder's idle animation, so whole-rope sleep cannot be relied on — and the
// socket-follow placement multiplies angular noise by the tail-to-head lever arm, so the raw
// per-frame follow visibly shivers. This class owns the deadband latch and the smoothed sample; it
// references no UObject or world state, so the hold, converge and pass-through contracts are unit
// testable. Reading the sim and pushing the transform stay on URopeComponent.

#pragma once

#include "CoreMinimal.h"

class DYNAMICROPE_API FRopeTipStabilizer
{
public:
	/** Tunables, rebuilt per call from the component's members; the class stores no config. The
	 *  defaults mirror the component's tuning. */
	struct FParams
	{
		/** Position changes below this hold the latched target (cm). Fades out with speed. */
		float PositionDeadband = 5.0f;

		/** Direction changes below this hold the latched target (degrees). Fades out with speed. */
		float AngleDeadbandDeg = 3.0f;

		/** Half-life of the converge toward a moved target (s). Zero or less snaps. */
		float SmoothingHalfLife = 0.03f;

		/** Raw tip speed (cm/s) at which stabilization has fully faded to raw pass-through. */
		float FadeOutSpeed = 120.0f;

		/** A raw jump beyond this reseeds onto the input instead of converging (teleport guard, cm). */
		float TeleportDistance = 100.0f;
	};

	/** One follow sample, raw or stabilized: the rope-attach point and the follow direction (unit). */
	struct FSample
	{
		FVector Position = FVector::ZeroVector;
		FVector Direction = FVector::ForwardVector;
	};

	/** The tail direction from the last SampleCount segments, averaged with weights rising toward
	 *  the tip, which attacks the per-segment noise before the deadband ever sees it. SampleCount 1
	 *  reproduces the raw last segment. Falls back to the raw last segment when the average
	 *  degenerates (a folded rope cancels itself out), and to Fallback below 2 nodes. */
	static FVector ComputeTailDirection(TConstArrayView<FVector> Positions, int32 SampleCount,
		const FVector& Fallback);

	/** Discards all state; the next Stabilize returns its input exactly and reseeds. Called on phase
	 *  transitions and sim reseeds by the component, so no stale latch survives a placement-contract
	 *  branch (Loaded, the wrapped embed) back into the follow fallback. */
	void Reset();

	/** Filters one follow sample. The position and direction deadbands latch independently — a
	 *  drifting position must not unlatch a steady direction, or the angle band would be dead
	 *  whenever the rope creeps. Within its band each latch holds still; the comparison is against
	 *  the latch, not the last output, which is what stops a value hovering around the threshold
	 *  from re-shivering. Beyond it the latch moves to the input and the output converges by
	 *  exponential smoothing (position) and normal slerp (direction), never a snap, so a slow reel
	 *  or drag ramps instead of staircasing. Both the deadbands and the smoothing fade to raw as
	 *  the tip's *sustained* speed approaches FadeOutSpeed — sustained meaning net travel from a
	 *  trailing average, not the per-frame delta, so in-place jitter (which spikes the per-frame
	 *  delta hardest exactly when filtering matters most) cannot fade the filter out; only actual
	 *  travel (Flight, a dragged rope) passes through without lag. */
	FSample Stabilize(const FSample& Raw, float DeltaTime, const FParams& Params);

	bool HasState() const { return bHasState; }

private:
	/** Whether Held/Smoothed/LastRawPosition/SlowRawPosition have been seeded from an input yet. */
	bool bHasState = false;

	/** The deadband-latched target the output converges toward. */
	FSample Held;

	/** The last output, which the next converge continues from. */
	FSample Smoothed;

	/** The last raw position, for the teleport guard alone. */
	FVector LastRawPosition = FVector::ZeroVector;

	/** A trailing average of the raw position; the distance from it is the sustained-speed source. */
	FVector SlowRawPosition = FVector::ZeroVector;
};
