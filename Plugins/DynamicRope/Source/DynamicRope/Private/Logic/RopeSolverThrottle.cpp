// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeSolverThrottle.h"
#include "Collision/RopeCollider.h"

bool FRopeSolverThrottle::UpdateSleepState(ERopePhase Phase, const FRopeSimState& Sim,
	const FRopeSolverConfig& Config, float DeltaTime, bool bHoldAwake)
{
	// Measured in Free and Wrapped alone, and only where sleeping is permitted. Elsewhere the accumulation is
	// discarded to prevent the state being contaminated, and the cache is rebuilt on entering the next sleepable
	// phase. Wrapped is a phase where the logic, meaning Hold, traction and the automatic release, keeps running while
	// the solve alone rests, so the same settling measurement as Free applies: when the bone moves, the node
	// displacement Hold wrote is picked up by the measurement directly.
	const bool bSleepPhase = (Phase == ERopePhase::Free || Phase == ERopePhase::Wrapped);
	if (!bSleepPhase || !Config.bAllowSleep || bAsleep || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		SleepTimer = 0.0f;
		SleepPrevFramePositions.Reset();
		return false;
	}

	// The maximum node displacement between frames, giving a speed. It compares against a per-frame cache rather than
	// the Verlet substep displacement, so it works regardless of the substep count or the GPU mirror's latency.
	bool bJustSlept = false;
	if (SleepPrevFramePositions.Num() == Sim.Num())
	{
		float MaxDistSq = 0.0f;
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			MaxDistSq = FMath::Max(MaxDistSq, static_cast<float>(FVector::DistSquared(Sim.Positions[i], SleepPrevFramePositions[i])));
		}
		const float MaxSpeed = FMath::Sqrt(MaxDistSq) / DeltaTime;
		// Holding it awake prevents entering sleep alone, by resetting the timer; the measurement cache keeps updating so that the delay restarts cleanly when the gate is released.
		SleepTimer = (!bHoldAwake && MaxSpeed < Config.SleepVelocityThreshold) ? SleepTimer + DeltaTime : 0.0f;
		if (SleepTimer >= Config.SleepDelay)
		{
			bAsleep = true;
			SleepPinPos = Sim.StartPinTarget;
			// The baseline for waking on drift, which detects a logic write while asleep, in ShouldWakeFromSleep.
			SleepNodePositions = Sim.Positions;
			// Logging the transition is the caller's, meaning the component's, responsibility.
			bJustSlept = true;
		}
	}
	SleepPrevFramePositions = Sim.Positions;
	return bJustSlept;
}

bool FRopeSolverThrottle::ShouldWakeFromSleep(const FRopeSimState& Sim, const FRopeSolverConfig& Config,
	float ReelRate, const TArray<IRopeCollider*>& Colliders) const
{
	if (!Config.bAllowSleep)
	{
		return true;
	}
	// The pin, meaning the hand, has moved since it went to sleep, so the character moved.
	if (FVector::DistSquared(Sim.StartPinTarget, SleepPinPos) > FMath::Square(1.0f))
	{
		return true;
	}
	// A logic write while asleep moved the nodes since it went to sleep. The typical case is Wrapped's Hold following
	// its bone, meaning the wrap target or the elevator is moving. In Free nothing writes the nodes while asleep, so
	// this is naturally a no-op. The threshold is 0.5 cm, the same as the collider rest test.
	if (SleepNodePositions.Num() == Sim.Num())
	{
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			if (FVector::DistSquared(Sim.Positions[i], SleepNodePositions[i]) > 0.25)
			{
				return true;
			}
		}
	}
	// It is reeling in or paying out.
	if (!FMath::IsNearlyZero(ReelRate))
	{
		return true;
	}
	// A moving collider nearby: the list supplied has already been culled against the rope's bounds by the subsystem,
	// so only nearby ones remain. A stationary bone, whose previous and current transforms match, is ignored, and the
	// small wobble of an animation idle is filtered out by the 0.5 cm threshold.
	for (const IRopeCollider* Collider : Colliders)
	{
		if (!Collider)
		{
			continue;
		}
		FTransform PrevX, CurrX;
		if (Collider->GetFrameMotion(PrevX, CurrX) && !PrevX.Equals(CurrX, 0.5f))
		{
			return true;
		}
		FVector PrevA, PrevB;
		float InvDt = 0.0f;
		if (Collider->GetGPUCapsuleMotion(PrevA, PrevB, InvDt))
		{
			FVector A, B;
			float R = 0.0f;
			if (Collider->GetGPUCapsule(A, B, R)
				&& (FVector::DistSquared(A, PrevA) > 0.25 || FVector::DistSquared(B, PrevB) > 0.25))
			{
				return true;
			}
		}
	}
	return false;
}

void FRopeSolverThrottle::ComputeSolverLOD(const FRopeSolverConfig& Config, const TOptional<float>& CameraDistance)
{
	SolverLODScale = 1.0f;
	if (!Config.bEnableDistanceLOD || Config.LODStartDistance <= 0.0f || !CameraDistance.IsSet())
	{
		// Inactive, or with no camera as on a server, means full quality.
		return;
	}
	const float Range = FMath::Max(Config.LODEndDistance - Config.LODStartDistance, 1.0f);
	const float Alpha = FMath::Clamp((CameraDistance.GetValue() - Config.LODStartDistance) / Range, 0.0f, 1.0f);
	SolverLODScale = FMath::Lerp(1.0f, FMath::Clamp(Config.LODMinIterationScale, 0.05f, 1.0f), Alpha);
}
