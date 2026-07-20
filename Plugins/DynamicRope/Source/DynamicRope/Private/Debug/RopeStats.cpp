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
DEFINE_STAT(STAT_Rope_Sleeping);
DEFINE_STAT(STAT_Rope_GdfDispatched);

DEFINE_STAT(STAT_Rope_PhasePhysics);
DEFINE_STAT(STAT_Rope_PhaseLogic);

void RopeStats::RecordFrameStats(TConstArrayView<TObjectPtr<URopeComponent>> Ropes, const FRopeFrameCounters& Frame)
{
#if STATS
	// 그룹 미수집이면 순회 자체를 건너뛴다(수집 안 할 때 핫 패스 오버헤드 0).
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

		// physics/logic 경계 = 솔버가 굴리는 페이즈(Free/Flight)인가 아닌가. 나머지(Contacting..Reel)는 전부 로직.
		const ERopePhase Phase = Rope->GetPhase();
		if (Phase == ERopePhase::Free || Phase == ERopePhase::Flight)
		{
			++NumPhysicsPhase;
		}

		// 솔브 경로 분할: GPU step > (솔브했지만 GPU 아님 = CPU 폴백) > 나머지는 솔브 없음(카운터 없음, Active에서 차감).
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
