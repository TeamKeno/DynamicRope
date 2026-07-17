// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/RopeStats.h"
#include "RopeComponent.h"
#include "Core/RopeTypes.h"

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
DEFINE_STAT(STAT_Rope_Idle);
DEFINE_STAT(STAT_Rope_Sleeping);
DEFINE_STAT(STAT_Rope_GdfEnabled);
DEFINE_STAT(STAT_Rope_GdfDispatched);

DEFINE_STAT(STAT_Rope_PhaseFree);
DEFINE_STAT(STAT_Rope_PhaseFlight);
DEFINE_STAT(STAT_Rope_PhaseContacting);
DEFINE_STAT(STAT_Rope_PhaseWrapping);
DEFINE_STAT(STAT_Rope_PhaseWrapped);
DEFINE_STAT(STAT_Rope_PhaseGuidedThrow);
DEFINE_STAT(STAT_Rope_PhaseReleasing);
DEFINE_STAT(STAT_Rope_PhaseReel);

void RopeStats::RecordFrameStats(TConstArrayView<TObjectPtr<URopeComponent>> Ropes, const FRopeFrameCounters& Frame)
{
#if STATS
	// 그룹 미수집이면 순회 자체를 건너뛴다(수집 안 할 때 핫 패스 오버헤드 0).
	if (!FThreadStats::IsCollectingData(GET_STATID(STAT_Rope_Active)))
	{
		return;
	}

	// ERopePhase 값 개수만큼(Free..Reel). 인덱스 = static_cast<int32>(Phase).
	int32 PhaseCounts[static_cast<int32>(ERopePhase::Reel) + 1] = {};
	int64 TotalParticles = 0;
	int32 NumGpuStepped = 0;
	int32 NumCpuSolved = 0;
	int32 NumSleeping = 0;
	int32 NumGdfEnabled = 0;
	for (const TObjectPtr<URopeComponent>& RopePtr : Ropes)
	{
		const URopeComponent* Rope = RopePtr.Get();
		if (!Rope)
		{
			continue;
		}
		TotalParticles += Rope->GetNodeCount();

		const int32 PhaseIdx = static_cast<int32>(Rope->GetPhase());
		if (PhaseIdx >= 0 && PhaseIdx < UE_ARRAY_COUNT(PhaseCounts))
		{
			++PhaseCounts[PhaseIdx];
		}

		// 솔브 경로 분할: GPU step > (솔브했지만 GPU 아님 = CPU 폴백) > 나머지는 Idle.
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
		if (Rope->bUseWorldGDF)
		{
			++NumGdfEnabled;
		}
	}

	const int32 NumRopes = Ropes.Num();
	const int32 NumIdle = FMath::Max(0, NumRopes - NumGpuStepped - NumCpuSolved);

	SET_DWORD_STAT(STAT_Rope_Active, NumRopes);
	SET_DWORD_STAT(STAT_Rope_TotalParticles, static_cast<int32>(TotalParticles));
	SET_DWORD_STAT(STAT_Rope_FrameColliders, Frame.FrameColliders);
	SET_DWORD_STAT(STAT_Rope_GpuStepped, NumGpuStepped);
	SET_DWORD_STAT(STAT_Rope_CpuSolved, NumCpuSolved);
	SET_DWORD_STAT(STAT_Rope_Idle, NumIdle);
	SET_DWORD_STAT(STAT_Rope_Sleeping, NumSleeping);
	SET_DWORD_STAT(STAT_Rope_GdfEnabled, NumGdfEnabled);
	SET_DWORD_STAT(STAT_Rope_GdfDispatched, Frame.NumGdfDispatched);

	SET_DWORD_STAT(STAT_Rope_PhaseFree, PhaseCounts[static_cast<int32>(ERopePhase::Free)]);
	SET_DWORD_STAT(STAT_Rope_PhaseFlight, PhaseCounts[static_cast<int32>(ERopePhase::Flight)]);
	SET_DWORD_STAT(STAT_Rope_PhaseContacting, PhaseCounts[static_cast<int32>(ERopePhase::Contacting)]);
	SET_DWORD_STAT(STAT_Rope_PhaseWrapping, PhaseCounts[static_cast<int32>(ERopePhase::Wrapping)]);
	SET_DWORD_STAT(STAT_Rope_PhaseWrapped, PhaseCounts[static_cast<int32>(ERopePhase::Wrapped)]);
	SET_DWORD_STAT(STAT_Rope_PhaseGuidedThrow, PhaseCounts[static_cast<int32>(ERopePhase::GuidedThrow)]);
	SET_DWORD_STAT(STAT_Rope_PhaseReleasing, PhaseCounts[static_cast<int32>(ERopePhase::Releasing)]);
	SET_DWORD_STAT(STAT_Rope_PhaseReel, PhaseCounts[static_cast<int32>(ERopePhase::Reel)]);
#endif
}
