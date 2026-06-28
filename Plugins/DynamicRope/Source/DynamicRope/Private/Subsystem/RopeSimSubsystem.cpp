// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeSimSubsystem.h"
#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Solver/RopeXPBDSolver.h" // RopeSolverSubsteps
#include "Collision/RopeCollider.h" // IRopeCollider::GetGPUCapsule
#include "Collision/RopeColliderProvider.h" // IRopeColliderProvider (중앙 collider gather)
#include "RopeGPUSolver.h"          // FRopeGPUSolver / FRopeGPUResidentStep / FRopeGPUCapsule (DynamicRopeShaders 모듈)
#include "Engine/World.h"
#include "GameFramework/Actor.h"    // AActor::GetOwner (provider 소스 필터링)
#include "Components/ActorComponent.h"
#include "Async/ParallelFor.h"
#include "HAL/IConsoleManager.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

// 0=CPU(ParallelFor) 솔버, 1=GPU compute 솔버(M5: 센터라인 GPU 상주 — 매 프레임 in-place 전진, 결과는 약간 지연된
// 미러로 회수). whip 프레임은 CPU 폴백. 런타임 토글. CPU 경로는 ground-truth로 유지된다.
static TAutoConsoleVariable<int32> CVarRopeGPUSolver(
	TEXT("r.DynamicRope.GPUSolver"),
	0,
	TEXT("DynamicRope: 0=CPU ParallelFor 솔버(기본), 1=GPU 상주 compute 솔버(M5, 충돌 포함; whip은 CPU 폴백)."),
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
	if (Rope)
	{
		// GPU 상주 버퍼/리드백 해제(렌더 스레드에서). 캐시에서도 제거.
		const uint32 RopeId = Rope->GetUniqueID();
		GpuSolver.ReleaseRope(RopeId);
		GpuLatest.Remove(RopeId);
	}
	UE_LOG(LogDynamicRope, Verbose, TEXT("UnregisterRope: %s (%d remaining)"),
		Rope ? *Rope->GetName() : TEXT("null"), Ropes.Num());
}

URopeSimSubsystem* URopeSimSubsystem::Get(const UWorld* World)
{
	return World ? World->GetSubsystem<URopeSimSubsystem>() : nullptr;
}

void URopeSimSubsystem::RegisterColliderProvider(UActorComponent* Provider)
{
	if (Provider)
	{
		ColliderProviders.AddUnique(Provider);
		UE_LOG(LogRopeCollision, Verbose, TEXT("RegisterColliderProvider: %s (%d total)"),
			*Provider->GetName(), ColliderProviders.Num());
	}
}

void URopeSimSubsystem::UnregisterColliderProvider(UActorComponent* Provider)
{
	ColliderProviders.RemoveSingleSwap(Provider);
}

void URopeSimSubsystem::BuildFrameColliders()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_BuildColliders);
	FrameProviders.Reset();

	// 전 로프 bounds 합집합(provider broad-phase용). 현 provider들은 무시하지만 인터페이스 계약 유지 — 향후
	// bounds-aware provider는 활성 영역으로 컬할 수 있다. 솔버가 다시 per-rope로 좁힌다.
	FBox AllBounds(ForceInit);
	for (URopeComponent* Rope : Ropes)
	{
		if (!IsValid(Rope))
		{
			continue;
		}
		for (const FVector& P : Rope->Sim.Positions)
		{
			AllBounds += P;
		}
	}
	if (AllBounds.IsValid)
	{
		AllBounds = AllBounds.ExpandBy(50.0f); // contact reach 여유.
	}

	// 등록된 provider마다 1회 gather(프레임당 1회 — 로프 수와 무관). 죽은 provider는 정리.
	for (int32 i = ColliderProviders.Num() - 1; i >= 0; --i)
	{
		UActorComponent* Comp = ColliderProviders[i];
		if (!IsValid(Comp))
		{
			ColliderProviders.RemoveAtSwap(i);
			continue;
		}
		IRopeColliderProvider* Provider = Cast<IRopeColliderProvider>(Comp);
		if (!Provider)
		{
			continue;
		}
		FFrameProviderColliders FP;
		FP.Owner = Comp->GetOwner();
		Provider->GatherColliders(AllBounds, FP.Colliders);
		if (FP.Colliders.Num() > 0)
		{
			FrameProviders.Add(MoveTemp(FP));
		}
	}
}

void URopeSimSubsystem::GatherCollidersForRope(const URopeComponent& Rope, TArray<IRopeCollider*>& OutColliders) const
{
	OutColliders.Reset();

	// 기본: 월드의 모든 provider와 충돌하되 자기 owner(던진 본인) provider는 제외(throw 시 self-tangle 방지).
	// 다른 액터 body 잡기(cross-actor)는 그 액터가 "전체"에 포함되므로 자동. owner 충돌이 필요하면 옵트인.
	const AActor* OwnerToExclude = Rope.bIncludeOwnerColliders ? nullptr : Rope.GetOwner();

	for (const FFrameProviderColliders& FP : FrameProviders)
	{
		if (FP.Owner == OwnerToExclude && OwnerToExclude != nullptr)
		{
			continue; // 자기 owner provider 제외.
		}
		OutColliders.Append(FP.Colliders);
	}
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

	const bool bUseGPU = CVarRopeGPUSolver.GetValueOnGameThread() != 0;

	// GPU 상주(M5): RT 리드백이 채운 RopeId별 최신(약 1~2프레임 지연) 위치를 회수해 캐시. 아래 Phase 2에서
	// Free/Flight 로프의 Sim(렌더/충돌 미러)에 반영한다. 순차 의존성은 GPU 영속 버퍼 안에서 충족된다.
	if (bUseGPU)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GPUGetLatest);
		GpuSolver.GetLatest(GpuLatest);
	}

	// Phase 1a (GT): collider 중앙 수집 — 등록된 provider에서 프레임당 1회 빌드 후 로프별 필터로 FrameColliders 채움.
	// (로프마다 월드를 스캔하던 것을 대체. collider 포인터는 provider 소유라 이번 프레임 solve/finalize 동안 유효.)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GatherColliders);
		BuildFrameColliders();
		for (URopeComponent* Rope : Ropes)
		{
			GatherCollidersForRope(*Rope, Rope->FrameColliders);
		}
	}

	// Phase 1b (GT): 준비 — init/pin + 로직 phase 처리(collider는 위에서 이미 채워짐).
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Prepare);
		for (URopeComponent* Rope : Ropes)
		{
			Rope->PrepareSimFrame(DeltaTime);
		}
	}

	// Phase 2: Free/Flight의 solver step.
	if (bUseGPU)
	{
		// GPU 상주 경로(M5a). 로프별 영속 버퍼를 매 프레임 in-place로 전진(라운드트립 스톨/슬로모 없음).
		// whip 프레임은 CPU가 위치를 가이드하므로 그 로프만 CPU 솔브로 폴백(가이드 위치 보존). 충돌/접촉은
		// Finalize의 CPU 경로가 (약간 지연된) 미러로 처리한다.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveGPU);

		TArray<FRopeGPUResidentStep> Steps;
		Steps.Reserve(Ropes.Num());
		for (URopeComponent* Rope : Ropes)
		{
			FRopeSimState& S = Rope->Sim;
			// GPU 상주 대상: Free/Flight(bSolveThisFrame)이고 whip이 아니며 노드수가 한도 내일 때.
			const bool bGpuPhase = Rope->Phase == ERopePhase::Free || Rope->Phase == ERopePhase::Flight;
			const bool bGpuRope = bGpuPhase && Rope->bSolveThisFrame && !Rope->bWhipSwingActive
				&& S.Num() >= 2 && S.Num() <= FRopeGPUSolver::MaxNodes;
			// M5b: 이 프레임에 GPU step되는 로프만 렌더가 resident PosBuf를 직접 읽는다(아니면 stale → CPU 미러).
			Rope->bGpuSteppedThisFrame = bGpuRope;
			if (!bGpuRope)
			{
				// whip/폴백: CPU 솔브(Free/Flight일 때만). logic phase는 bSolveThisFrame=false라 자동 스킵.
				if (Rope->bSolveThisFrame)
				{
					Rope->SolveSimFrame(DeltaTime);
				}
				continue;
			}

			const uint32 RopeId = Rope->GetUniqueID();

			// 직전 회수분을 Sim(미러)에 반영. generation이 현재와 일치할 때만(= GPU가 현재 시드를 따라잡음);
			// 재시드 직후 catch-up 중이면 CPU Sim을 그대로 둬 시드 소스를 보존한다.
			if (const FRopeResidentLatest* L = GpuLatest.Find(RopeId))
			{
				if (L->Generation == Rope->SimGeneration && L->NumNodes == S.Num()
					&& L->Positions.Num() == S.Num() && L->PrevPositions.Num() == S.Num())
				{
					S.Positions     = L->Positions;
					S.PrevPositions = L->PrevPositions;
				}
			}

			// 잡은 끝(node 0)을 현재 핀 위치로 정확히 맞춘다 — GPU 미러는 ~1~2프레임 지연이라 손과 어긋난다.
			// 렌더/접촉용 보정(GPU 솔브 자체는 PinTarget으로 매 스텝 핀을 처리하므로 시뮬레이션엔 영향 없음).
			if (S.bStartPinned && S.Num() > 0)
			{
				S.Positions[0]     = S.StartPinTarget;
				S.PrevPositions[0] = S.StartPinPrev;
			}

			// 고정-timestep 스케줄(CPU accumulator) — 매 프레임 계산이라 시간손실 없음.
			const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(S, Rope->SolverConfig, DeltaTime);

			// 상주 step 구성(self-contained). 시드 데이터는 매 프레임 제공(RT는 재시드 시에만 GPU 업로드).
			const FRopeSolverConfig& Cfg = Rope->SolverConfig;
			FRopeGPUResidentStep Step;
			Step.RopeId            = RopeId;
			Step.Generation        = Rope->SimGeneration;
			Step.NumNodes          = S.Num();
			Step.SeedPositions     = S.Positions;
			Step.SeedPrevPositions = S.PrevPositions;
			Step.InvMass           = S.InvMass;
			Step.SegmentLength     = S.SegmentLength;
			Step.bStartPinned      = S.bStartPinned;
			Step.StartPinPrev      = S.StartPinPrev;
			Step.StartPinTarget    = S.StartPinTarget;
			Step.StretchCompliance = Cfg.StretchCompliance;
			Step.BendCompliance    = Cfg.BendCompliance;
			Step.Damping           = Cfg.Damping;
			Step.Iterations        = Cfg.Iterations;
			Step.Gravity           = Cfg.Gravity;
			Step.CollisionRadius   = Cfg.CollisionRadius;
			Step.Friction          = Cfg.Friction;
			Step.TipFrictionScale  = Cfg.TipFrictionScale;
			Step.SweepStep         = Cfg.SweepStep;
			Step.MaxSweepSamples   = Cfg.MaxSweepSamples;
			Step.NumSub            = Schedule.NumSub;
			Step.FixedDt           = Schedule.FixedDt;

			// 충돌: 이 로프의 collider를 capsule(M2)/SDF(M3)로 분류. FrameColliders는 Prepare에서 GT gather된 스냅샷.
			for (IRopeCollider* Collider : Rope->FrameColliders)
			{
				if (!Collider)
				{
					continue;
				}
				FRopeGPUCapsule Cap;
				if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
				{
					Step.Capsules.Add(Cap);
					continue;
				}
				FRopeSDFColliderView View;
				if (Collider->GetGPUSDF(View))
				{
					FRopeGPUSDFCollider Sdf;
					Sdf.Distances   = View.Distances;
					Sdf.ResX        = View.ResX;
					Sdf.ResY        = View.ResY;
					Sdf.ResZ        = View.ResZ;
					Sdf.LocalMin    = View.LocalMin;
					Sdf.LocalSize   = View.LocalSize;
					Sdf.BoneToWorld = View.BoneToWorld;
					Sdf.PrevBoneToWorld = View.PrevBoneToWorld; // GPU CCD/표면속도 드래그.
					Sdf.InvDeltaTime    = View.InvDeltaTime;
					Sdf.VolumeKey   = View.VolumeKey;
					Step.SDFColliders.Add(Sdf);
				}
			}

			Steps.Add(MoveTemp(Step));
		}
		if (Steps.Num() > 0)
		{
			GpuSolver.Step(MoveTemp(Steps));
		}
	}
	else
	{
		// CPU 경로(기본): 로프는 서로 독립 + collider 스냅샷 read-only → 스레드 안전.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveParallel);
		ParallelFor(Ropes.Num(), [this, DeltaTime](int32 Index)
		{
			Ropes[Index]->bGpuSteppedThisFrame = false; // CPU 경로 → resident 렌더 안 함(M5b).
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
