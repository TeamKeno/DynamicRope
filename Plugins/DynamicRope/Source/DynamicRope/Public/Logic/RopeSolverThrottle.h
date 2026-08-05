// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The solve throttle, a class with no UObject dependency. It provides sleep for stationary ropes in
// the Free and Wrapped phases, which skips the solve and its dispatch, and a distance LOD that
// reduces the iteration count. It owns the state, that is the sleep timer, the measurement caches and
// the LOD scale, along with the decisions to enter and leave sleep. UObject context is injected per
// call: reading the camera to compute the distance, and logging sleep transitions, stay on
// URopeComponent, where UpdateSleepState returning true on the frame the rope falls asleep tells it
// when to log. It can be unit tested without a world.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeConfigTypes.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeSimTypes.h"

class IRopeCollider;

class DYNAMICROPE_API FRopeSolverThrottle
{
public:
	//~ Sleep, in the Free and Wrapped phases, which skips the solve once the rope is judged
	//~ stationary.
	bool IsAsleep() const { return bAsleep; }

	/** Wakes immediately, resetting the timer and the drift cache. Called on entering a phase that
	 *  cannot sleep and when a wake condition passes. The measurement cache,
	 *  SleepPrevFramePositions, is left alone because UpdateSleepState discards it itself in
	 *  non-sleeping phases. */
	void Wake() { bAsleep = false; SleepTimer = 0.0f; SleepNodePositions.Reset(); }

	/** The sleep measurement, run every frame during Finalize. In the Free and Wrapped phases, with
	 *  sleep permitted, the rope falls asleep once the maximum per-frame node speed has stayed below
	 *  the threshold for SleepDelay. In other phases the accumulator and caches are discarded so the
	 *  state cannot be polluted.
	 *  A frame with bHoldAwake set only blocks entry, resetting the timer while keeping the measurement
	 *  cache. That is the gate that keeps a wrapped rope awake while active pull is armed even though
	 *  it is stationary, because traction impulses assume integration is running.
	 *  Returns true on the call where the rope has just fallen asleep, so the caller, the component,
	 *  can log the transition. */
	bool UpdateSleepState(ERopePhase Phase, const FRopeSimState& Sim, const FRopeSolverConfig& Config,
		float DeltaTime, bool bHoldAwake = false);

	/** The wake decision, made during Prepare. It wakes on movement of the pin at the hand, on node
	 *  drift caused by logic writes while asleep, of which the bone following of a wrapped hold is the
	 *  main case so a moving wrap target or elevator wakes here, while reeling, and on a moving
	 *  collider nearby. Colliders is expected to be the frame snapshot already culled to the rope's
	 *  bounds, that is SimFrame.FrameColliders. */
	bool ShouldWakeFromSleep(const FRopeSimState& Sim, const FRopeSolverConfig& Config, float ReelRate,
		const TArray<IRopeCollider*>& Colliders) const;

	//~ Distance LOD, which reduces iterations at a distance. Only iterations are reduced, because
	//~ stability is governed by the substep count.
	/** Refreshes the LOD scale, during Prepare and every frame. An unset camera distance, as on a
	 *  server or with no camera, gives full quality, that is 1. */
	void ComputeSolverLOD(const FRopeSolverConfig& Config, const TOptional<float>& CameraDistance);

	float GetSolverLODScale() const { return SolverLODScale; }

	/** The effective iteration count with the LOD applied, shared by the CPU solve and the GPU step. */
	int32 LODScaledIterations(int32 ConfigIterations) const
	{
		return FMath::Max(1, FMath::RoundToInt(static_cast<float>(ConfigIterations) * SolverLODScale));
	}

private:
	/** Whether the solve is being skipped because the rope was judged stationary in Free or Wrapped. */
	bool  bAsleep = false;

	/** How long the rope has stayed below the speed threshold (s). */
	float SleepTimer = 0.0f;

	/** The pin position when sleep was entered, which wakes the rope if it moves. */
	FVector SleepPinPos = FVector::ZeroVector;

	/** The node positions when sleep was entered, which is the reference for the drift wake and
	 *  detects the bone movement a wrapped hold writes into the mirror. Cleared by Wake. */
	TArray<FVector> SleepNodePositions;

	/** The cache used to measure per-frame displacement, refreshed during Finalize. */
	TArray<FVector> SleepPrevFramePositions;

	/** The distance LOD iteration scale, computed during Prepare, where 1 is full quality. */
	float SolverLODScale = 1.0f;
};
