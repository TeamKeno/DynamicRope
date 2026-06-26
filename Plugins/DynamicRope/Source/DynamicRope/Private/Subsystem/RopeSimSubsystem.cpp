// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeSimSubsystem.h"
#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Solver/RopeXPBDSolver.h" // RopeSolverSubsteps
#include "Collision/RopeCollider.h" // IRopeCollider::GetGPUCapsule
#include "RopeGPUSolver.h"          // FRopeGPUSolver / FRopeGPUJob / FRopeGPUCapsule (DynamicRopeShaders 모듈)
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
		TArray<FRopeGPUJob>        Jobs;
		TArray<FRopeGPUCapsule>    AllCaps;   // 전 로프 capsule 평탄 배열(완성 후 Job.Capsules 포인터 fixup).
		TArray<FRopeGPUSDFCollider> AllSDF;   // 전 로프 SDF collider 평탄 배열(완성 후 Job.SDFColliders fixup).
		TArray<int32>              JobCapOffset;
		TArray<int32>              JobSDFOffset;
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

			// 충돌: 이 로프의 collider를 capsule(M2)/SDF(M3)로 분류해 추출. 둘 다 아니면 GPU 충돌에서 제외.
			// FrameColliders는 Prepare에서 GT로 gather된 per-rope 스냅샷(GPU 토글과 무관하게 채워짐).
			const int32 CapOffset = AllCaps.Num();
			const int32 SDFOffset = AllSDF.Num();
			for (IRopeCollider* Collider : Rope->FrameColliders)
			{
				if (!Collider)
				{
					continue;
				}
				FRopeGPUCapsule Cap;
				if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
				{
					AllCaps.Add(Cap);
					continue;
				}
				FRopeSDFColliderView View;
				if (Collider->GetGPUSDF(View))
				{
					FRopeGPUSDFCollider S;
					S.Distances   = View.Distances;
					S.ResX        = View.ResX;
					S.ResY        = View.ResY;
					S.ResZ        = View.ResZ;
					S.LocalMin    = View.LocalMin;
					S.LocalSize   = View.LocalSize;
					S.BoneToWorld = View.BoneToWorld;
					S.VolumeKey   = View.VolumeKey;
					AllSDF.Add(S);
				}
			}
			const int32 CapCount = AllCaps.Num() - CapOffset;
			const int32 SDFCount = AllSDF.Num() - SDFOffset;

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
			Job.NumCapsules       = CapCount;
			Job.CollisionRadius   = Cfg.CollisionRadius;
			Job.Friction          = Cfg.Friction;
			Job.SweepStep         = Cfg.SweepStep;
			Job.MaxSweepSamples   = Cfg.MaxSweepSamples;
			Job.NumSDFColliders   = SDFCount;
			Job.NumSub            = Schedule.NumSub;
			Job.FixedDt           = Schedule.FixedDt;
			Jobs.Add(Job);
			JobCapOffset.Add(CapOffset);
			JobSDFOffset.Add(SDFOffset);
		}
		if (Jobs.Num() > 0)
		{
			// AllCaps/AllSDF 완성 후 포인터 fixup(앞서 Add 중 재할당으로 무효화될 수 있어 여기서 한다).
			for (int32 j = 0; j < Jobs.Num(); ++j)
			{
				Jobs[j].Capsules     = (Jobs[j].NumCapsules > 0)     ? (AllCaps.GetData() + JobCapOffset[j]) : nullptr;
				Jobs[j].SDFColliders = (Jobs[j].NumSDFColliders > 0) ? (AllSDF.GetData() + JobSDFOffset[j]) : nullptr;
			}
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
