// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeSimSubsystem.h"
#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Solver/RopeXPBDSolver.h" // RopeSolverSubsteps
#include "RopeGPUSolver.h"          // FRopeGPUSolver / FRopeGPUJob (DynamicRopeShaders 모듈)
#include "Engine/World.h"
#include "Async/ParallelFor.h"
#include "HAL/IConsoleManager.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

// 0=CPU(ParallelFor) 솔버, 1=GPU compute 솔버(M1: 물리 전용 — integrate/distance/bending; 충돌은 여전히 CPU).
// 런타임 토글. CPU 경로는 ground-truth로 유지된다.
static TAutoConsoleVariable<int32> CVarRopeGPUSolver(
	TEXT("r.DynamicRope.GPUSolver"),
	0,
	TEXT("DynamicRope: 0=CPU ParallelFor 솔버(기본), 1=GPU compute 솔버(물리 전용, 충돌 CPU 유지)."),
	ECVF_Default);

void URopeSimSubsystem::RegisterRope(URopeComponent* Rope)
{
	if (Rope)
	{
		Ropes.AddUnique(Rope);
		UE_LOG(LogDynamicRope, Verbose, TEXT("RegisterRope: %s (%d total)"), *Rope->GetName(), Ropes.Num());
	}
}

void URopeSimSubsystem::UnregisterRope(URopeComponent* Rope)
{
	Ropes.RemoveSingleSwap(Rope);
	UE_LOG(LogDynamicRope, Verbose, TEXT("UnregisterRope: %s (%d remaining)"),
		Rope ? *Rope->GetName() : TEXT("null"), Ropes.Num());
}

URopeSimSubsystem* URopeSimSubsystem::Get(const UWorld* World)
{
	return World ? World->GetSubsystem<URopeSimSubsystem>() : nullptr;
}

void URopeSimSubsystem::Tick(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SubsystemTick);

	// 무효 항목 정리.
	for (int32 i = Ropes.Num() - 1; i >= 0; --i)
	{
		if (!IsValid(Ropes[i]))
		{
			Ropes.RemoveAtSwap(i);
		}
	}
	if (Ropes.Num() == 0)
	{
		return;
	}

	// Phase 1 (GT): 준비 — init/pin/provider gather + collider 스냅샷 + 로직 phase 처리.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Prepare);
		for (URopeComponent* Rope : Ropes)
		{
			Rope->PrepareSimFrame(DeltaTime);
		}
	}

	// Phase 2: Free/Flight의 solver step. 두 경로 모두 POD(Sim) + const collider 스냅샷만 만진다.
	if (CVarRopeGPUSolver.GetValueOnGameThread() != 0)
	{
		// GPU 경로(M1: 물리 전용 — integrate/distance/bending). 단일 디스패치로 전 로프 배치.
		// ENQUEUE_RENDER_COMMAND는 워커스레드 불가 → ParallelFor 대신 GT에서 잡 수집 후 1회 호출.
		// 충돌/접촉은 GPU가 건드리지 않으므로 Finalize의 CPU 경로가 그대로 처리한다.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveGPU);
		TArray<FRopeGPUJob> Jobs;
		Jobs.Reserve(Ropes.Num());
		for (URopeComponent* Rope : Ropes)
		{
			if (!Rope->bSolveThisFrame || Rope->Sim.Num() < 2)
			{
				continue;
			}
			// CPU와 동일한 고정-timestep 부킹(accumulator 소비). NumSub<=0이면 이번 frame 솔브 없음.
			const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(Rope->Sim, Rope->SolverConfig, DeltaTime);
			if (Schedule.NumSub <= 0)
			{
				continue;
			}

			// POD 잡 구성(DynamicRopeShaders는 런타임 타입에 의존하지 않으므로 raw 포인터+스칼라로 전달).
			// SolveBatch는 동기라 GetData() 포인터가 호출 동안 유효하다.
			FRopeSimState& S = Rope->Sim;
			const FRopeSolverConfig& Cfg = Rope->SolverConfig;
			FRopeGPUJob Job;
			Job.Positions         = S.Positions.GetData();
			Job.PrevPositions     = S.PrevPositions.GetData();
			Job.InvMass           = S.InvMass.GetData();
			Job.NumNodes          = S.Num();
			Job.SegmentLength     = S.SegmentLength;
			Job.bStartPinned      = S.bStartPinned;
			Job.StartPinPrev      = S.StartPinPrev;
			Job.StartPinTarget    = S.StartPinTarget;
			Job.StretchCompliance = Cfg.StretchCompliance;
			Job.BendCompliance    = Cfg.BendCompliance;
			Job.Damping           = Cfg.Damping;
			Job.Iterations        = Cfg.Iterations;
			Job.Gravity           = Cfg.Gravity;
			Job.NumSub            = Schedule.NumSub;
			Job.FixedDt           = Schedule.FixedDt;
			Jobs.Add(Job);
		}
		if (Jobs.Num() > 0)
		{
			FRopeGPUSolver::SolveBatch(Jobs);
		}
	}
	else
	{
		// CPU 경로(기본): 로프는 서로 독립 + collider 스냅샷 read-only → 스레드 안전.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveParallel);
		ParallelFor(Ropes.Num(), [this, DeltaTime](int32 Index)
		{
			Ropes[Index]->SolveSimFrame(DeltaTime);
		});
	}

	// Phase 3 (GT): 마무리 — Flight 접촉 감지/캡처(UObject·이벤트) + 렌더 dirty.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Finalize);
		for (URopeComponent* Rope : Ropes)
		{
			Rope->FinalizeSimFrame(DeltaTime);
		}
	}

	// TODO: LOD/sleep(멀거나 안정된 로프 스킵), 프레임당 총 솔브 비용 상한.
	// TODO: tick 순서 — 충돌은 애니메이션(본 트랜스폼) 이후가 필요. 정밀 정렬은 TG_PostPhysics tick function.
}

TStatId URopeSimSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URopeSimSubsystem, STATGROUP_Tickables);
}

bool URopeSimSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	// 게임/PIE에서만 시뮬레이션(에디터 프리뷰/인스펙터 월드 제외 → 컴포넌트도 그때만 BeginPlay 등록).
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}
