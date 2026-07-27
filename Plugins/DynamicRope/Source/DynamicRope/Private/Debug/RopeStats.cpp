// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/RopeStats.h"
#include "RopeComponent.h"
#include "Core/RopeLifecycleTypes.h"

DEFINE_STAT(STAT_RopeSim_Tick);
DEFINE_STAT(STAT_RopeSim_Gather);
DEFINE_STAT(STAT_RopeSim_Prepare);
DEFINE_STAT(STAT_RopeSim_Solve);
DEFINE_STAT(STAT_RopeSim_Finalize);

DEFINE_STAT(STAT_Rope_Active);
DEFINE_STAT(STAT_Rope_TotalParticles);
DEFINE_STAT(STAT_Rope_FrameColliders);
DEFINE_STAT(STAT_Rope_GpuStepped);
DEFINE_STAT(STAT_Rope_CpuSolved);
DEFINE_STAT(STAT_Rope_Sleeping);
DEFINE_STAT(STAT_Rope_GdfDispatched);

DEFINE_STAT(STAT_Rope_PhasePhysics);
DEFINE_STAT(STAT_Rope_PhaseLogic);

void RopeStats::RecordFrameStats(TConstArrayView<TObjectPtr<URopeComponent>> Ropes, const FRopeFrameCounters& Frame)
{
#if STATS
	// If the group is not collecting, the walk itself is skipped, so the hot path costs nothing when stats are off.
	if (!FThreadStats::IsCollectingData(GET_STATID(STAT_Rope_Active)))
	{
		return;
	}

	int64 TotalParticles = 0;
	int32 NumPhysicsPhase = 0;
	int32 NumGpuStepped = 0;
	int32 NumCpuSolved = 0;
	int32 NumSleeping = 0;
	for (const TObjectPtr<URopeComponent>& RopePtr : Ropes)
	{
		const URopeComponent* Rope = RopePtr.Get();
		if (!Rope)
		{
			continue;
		}
		TotalParticles += Rope->GetNodeCount();

	// The boundary between physics and logic is whether the phase is one the solver drives, being Free or Flight. Everything else, from Contacting through Loaded, is logic.
		const ERopePhase Phase = Rope->GetPhase();
		if (Phase == ERopePhase::Free || Phase == ERopePhase::Flight)
		{
			++NumPhysicsPhase;
		}

	// The solve path breakdown: a GPU step first, then solved but not on the GPU, meaning the CPU fallback, and the remainder did not solve at all, which has no counter and is subtracted from the active count.
		if (Rope->IsGpuSteppedThisFrame())
		{
			++NumGpuStepped;
		}
		else if (Rope->WasSolvedThisFrame())
		{
			++NumCpuSolved;
		}

		if (Rope->IsSleeping())
		{
			++NumSleeping;
		}
	}

	const int32 NumRopes = Ropes.Num();

	SET_DWORD_STAT(STAT_Rope_Active, NumRopes);
	SET_DWORD_STAT(STAT_Rope_TotalParticles, static_cast<int32>(TotalParticles));
	SET_DWORD_STAT(STAT_Rope_FrameColliders, Frame.FrameColliders);
	SET_DWORD_STAT(STAT_Rope_GpuStepped, NumGpuStepped);
	SET_DWORD_STAT(STAT_Rope_CpuSolved, NumCpuSolved);
	SET_DWORD_STAT(STAT_Rope_Sleeping, NumSleeping);
	SET_DWORD_STAT(STAT_Rope_GdfDispatched, Frame.NumGdfDispatched);

	SET_DWORD_STAT(STAT_Rope_PhasePhysics, NumPhysicsPhase);
	SET_DWORD_STAT(STAT_Rope_PhaseLogic, FMath::Max(0, NumRopes - NumPhysicsPhase));
#endif
}
