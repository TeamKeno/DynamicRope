// Copyright Epic Games, Inc. All Rights Reserved.
//
// 'stat DynamicRope' 대시보드 그룹 — 프레임 단위 시뮬 코스트/부하를 한 화면에 모은다(용도 C: 성능 추적 +
// 런타임 상태 확인 겸용). per-component 'stat RopeFlight'/'stat RopeWrapped'(RopeDebugDraw)는 그대로 두고,
// 이 그룹은 URopeSimSubsystem::Tick 프레임 전체를 조망한다.
//   - CYCLE stat(GT 타이밍): Tick 단계(Gather/Prepare/Solve/Finalize)를 ms로.
//   - DWORD 카운터(부하): 활성 로프 수·페이즈 분포·파티클 총량·솔브 경로 분기·collider 수.
//
// 솔브 경로 3분할(매 프레임 Active = GpuStepped + CpuSolved + Idle):
//   - GPU Stepped : 이 프레임 GPU 상주 스텝(솔브 또는 로직 override)로 dispatch된 로프.
//   - CPU Solved  : 솔브는 했으나 GPU가 아닌 로프 = 진짜 CPU 폴백(노드 > MaxNodes(256), 또는 렌더 가능 RHI
//                   없음 -nullrhi/서버). TryBuildResidentStep의 bGpuRope=false && bSolveThisFrame 분기.
//   - Idle        : 이 프레임 솔브 자체가 없는 로프 — Contacting(동결)·슬립(Free 정지)·로직 override-only 없음 등.
// Sleeping은 위 분할과 직교하는 상태 카운터(Free 정지로 잠든 로프; 대개 Idle에 포함된다).
// GDF는 Enabled(설정 bUseWorldGDF)와 Dispatched(이 프레임 실제 GDF와 함께 dispatch)를 분리 — 후자는 phase/
// 슬립/충돌 게이트에 따라 프레임마다 흔들리는 게 정상이다(엔진 온디맨드 GDF 빌드 신호와 동일한 수).
//
// stat 매크로는 STATS 비활성 빌드(shipping 등)에서 자동 no-op이 되므로 별도 #if 가드가 필요없다. EXTERN 선언은
// 여기, DEFINE_STAT 실체는 RopeStats.cpp에 둔다 — 그래서 SCOPE_CYCLE_COUNTER/SET_DWORD_STAT를 서브시스템 등
// 다른 번역 단위에서 바로 쓸 수 있다.
//
// [후속 seam] GPU RT 내부 타이밍(RopeRT_PackSDF 등 — SDF 재업로드 병목, memory: sdf-global-volume-cache)과 GPU
// 버퍼 메모리는 DynamicRopeShaders 모듈 소유라 이 런타임 헤더에서 정의할 수 없다. 지금은 Unreal Insights의
// TRACE_CPUPROFILER 경로에 남아 있고, stat HUD로 끌어오려면 shaders 모듈이 같은 "DynamicRope" 그룹명으로 자체
// stat을 선언·기록해야 한다(아래 [GPU] TODO).

#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"
#include "Stats/Stats.h"
#include "UObject/ObjectPtr.h"

class URopeComponent;

// 'stat DynamicRope' — 런타임 시뮬 프레임 코스트/부하 대시보드.
DECLARE_STATS_GROUP(TEXT("DynamicRope"), STATGROUP_DynamicRope, STATCAT_Advanced);

// ── [GT 타이밍] URopeSimSubsystem::Tick 단계별 소요(GT 벽시계). GPU 경로의 Solve는 dispatch enqueue 비용만
//    잡힌다(실제 GPU 솔브/감지는 RT — Insights 참조). CPU 폴백 경로의 Solve는 ParallelFor 실 솔브 비용. ──────
DECLARE_CYCLE_STAT_EXTERN(TEXT("Tick (total)"), STAT_RopeSim_Tick, STATGROUP_DynamicRope, );
DECLARE_CYCLE_STAT_EXTERN(TEXT("Gather Colliders"), STAT_RopeSim_Gather, STATGROUP_DynamicRope, );
DECLARE_CYCLE_STAT_EXTERN(TEXT("Prepare"), STAT_RopeSim_Prepare, STATGROUP_DynamicRope, );
DECLARE_CYCLE_STAT_EXTERN(TEXT("Solve (enqueue/parallel)"), STAT_RopeSim_Solve, STATGROUP_DynamicRope, );
DECLARE_CYCLE_STAT_EXTERN(TEXT("Finalize"), STAT_RopeSim_Finalize, STATGROUP_DynamicRope, );

// ── [부하] 이번 프레임 로프 규모/솔브 경로 분기. Active = GpuStepped + CpuSolved + Idle. ────────────────
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Active Ropes"), STAT_Rope_Active, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Total Particles"), STAT_Rope_TotalParticles, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Frame Colliders"), STAT_Rope_FrameColliders, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("GPU Stepped Ropes"), STAT_Rope_GpuStepped, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("CPU Solved Ropes (fallback)"), STAT_Rope_CpuSolved, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Idle Ropes (no solve)"), STAT_Rope_Idle, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Sleeping Ropes"), STAT_Rope_Sleeping, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("GDF-Enabled Ropes"), STAT_Rope_GdfEnabled, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("World GDF Dispatches"), STAT_Rope_GdfDispatched, STATGROUP_DynamicRope, );

// ── [페이즈 분포] ERopePhase별 활성 로프 수(합 = Active Ropes). 부하가 물리(Free/Flight)인지 로직
//    (Wrapping/Wrapped 등)인지 한눈에 — 설계의 physics/logic 분리와 대응. ────────────────────────────
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Free"), STAT_Rope_PhaseFree, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Flight"), STAT_Rope_PhaseFlight, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Contacting"), STAT_Rope_PhaseContacting, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Wrapping"), STAT_Rope_PhaseWrapping, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Wrapped"), STAT_Rope_PhaseWrapped, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: GuidedThrow"), STAT_Rope_PhaseGuidedThrow, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Releasing"), STAT_Rope_PhaseReleasing, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Reel"), STAT_Rope_PhaseReel, STATGROUP_DynamicRope, );

// [GPU — TODO/후속] DynamicRopeShaders 모듈에서 같은 "DynamicRope" 그룹명으로 선언·기록할 항목:
//   RopeRT_PackSDF / RopeRT_AddSolvePass / RopeRT_GraphExecute (CYCLE, RT 타이밍) + GPU 영속 버퍼 메모리(MEMORY).
//   현재는 Unreal Insights의 TRACE_CPUPROFILER_EVENT_SCOPE 경로로만 계측된다.

namespace RopeStats
{
	// 프레임 스칼라 — 서브시스템만 아는 값(SimFrame이 private, GPU 경로 분기)을 helper로 넘긴다. 나머지 로프별
	// 카운터(페이즈/GPU스텝/CPU솔브/슬립/GDF설정/파티클)는 RecordFrameStats가 public 게터로 직접 집계한다.
	struct FRopeFrameCounters
	{
		int32 NumGdfDispatched = 0;  // 이번 프레임 GDF와 함께 dispatch된 GPU 스텝 수(엔진 온디맨드 빌드 신호와 동일)
		int32 FrameColliders = 0;    // 전 로프 FrameColliders 합(SimFrame private → 서브시스템이 집계)
	};

	/** 'stat DynamicRope' 프레임 부하/페이즈 카운터 갱신(그룹 수집 중일 때만; 아니면 순회 스킵). 단계 타이밍은
	 *  SCOPE_CYCLE_COUNTER로 각 단계 스코프에서 직접 기록한다. */
	void RecordFrameStats(TConstArrayView<TObjectPtr<URopeComponent>> Ropes, const FRopeFrameCounters& Frame);
}
