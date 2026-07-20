// Copyright Epic Games, Inc. All Rights Reserved.
//
// 'stat DynamicRope' 대시보드 그룹 — 프레임 단위 시뮬 코스트/부하를 한 화면에 모은다(용도 C: 성능 추적 +
// 런타임 상태 확인 겸용). per-component 'stat RopeFlight'/'stat RopeWrapped'(RopeDebugDraw)는 그대로 두고,
// 이 그룹은 URopeSimSubsystem::Tick 프레임 전체를 조망한다.
//   - CYCLE stat(GT 타이밍): Tick 단계(Gather/Prepare/Solve/Finalize)를 ms로.
//   - DWORD 카운터(부하): 활성 로프 수·솔브 경로 분기·파티클 총량·collider 수·물리/로직 분포.
//
// [범위 계약] 이 그룹은 **GT 프레임 대시보드 전용**이다. GPU RT 타이밍/VRAM/대역폭은 별도 'stat DynamicRopeGPU'
// 그룹(DynamicRopeShaders 모듈, RopeGPUStatGroup.h)이 소유한다 — 한 그룹에 다 넣었더니 53행이 되어 stat HUD
// 화면을 넘겼다(HUD는 cycle→memory→counter 순으로 그리고 스크롤이 없어서, 정작 보고 싶은 counter 섹션이 화면
// 아래로 잘려나갔다). 새 stat을 추가할 때 이 분리를 지켜라: GT 프레임 비용은 여기, GPU/RT는 저기.
//
// 솔브 경로 분할(매 프레임 Active = GpuStepped + CpuSolved + 나머지(솔브 없음)):
//   - GPU Stepped : 이 프레임 GPU 상주 스텝(솔브 또는 로직 override)로 dispatch된 로프.
//   - CPU Solved  : 솔브는 했으나 GPU가 아닌 로프 = 진짜 CPU 폴백(노드 > MaxNodes(256), 또는 렌더 가능 RHI
//                   없음 -nullrhi/서버). TryBuildResidentStep의 bGpuRope=false && bSolveThisFrame 분기.
//   - (암묵) idle : 둘 다 아닌 로프 — Contacting(동결)·슬립(Free 정지)·로직 override-only 없음 등. Active에서
//                   두 값을 빼면 나오므로 별도 행을 두지 않는다(per-rope 내역은 RopePerf 디버거).
// Sleeping은 위 분할과 직교하는 상태 카운터(Free 정지로 잠든 로프; 대개 idle에 포함된다).
// World GDF Dispatches는 이 프레임 실제 GDF와 함께 dispatch된 수 — phase/슬립/충돌 게이트에 따라 프레임마다
// 흔들리는 게 정상이다(엔진 온디맨드 GDF 빌드 신호와 동일한 수). 설정(bUseWorldGDF) 자체는 기본값이 true라
// 세면 Active와 같아지므로 stat으로 두지 않는다 — per-rope 설정은 RopePerf 디버거가 gdf/gdf-off로 보여준다.
//
// 페이즈는 설계의 physics/logic 경계(Free·Flight = 솔버, 나머지 = 로직)만 2행으로 요약한다. ERopePhase 8종
// 내역은 'RopePerf' 게임플레이 디버거 카테고리가 per-rope로 이미 보여주므로 HUD에서 8행을 쓰지 않는다.
//
// stat 매크로는 STATS 비활성 빌드(shipping 등)에서 자동 no-op이 되므로 별도 #if 가드가 필요없다. EXTERN 선언은
// 여기, DEFINE_STAT 실체는 RopeStats.cpp에 둔다 — 그래서 SCOPE_CYCLE_COUNTER/SET_DWORD_STAT를 서브시스템 등
// 다른 번역 단위에서 바로 쓸 수 있다.

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

// ── [부하] 이번 프레임 로프 규모/솔브 경로 분기. Active - GpuStepped - CpuSolved = 솔브 없는 로프. ──────
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Active Ropes"), STAT_Rope_Active, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Total Particles"), STAT_Rope_TotalParticles, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Frame Colliders"), STAT_Rope_FrameColliders, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("GPU Stepped Ropes"), STAT_Rope_GpuStepped, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("CPU Solved Ropes (fallback)"), STAT_Rope_CpuSolved, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Sleeping Ropes"), STAT_Rope_Sleeping, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("World GDF Dispatches"), STAT_Rope_GdfDispatched, STATGROUP_DynamicRope, );

// ── [페이즈 분포] 설계의 physics/logic 경계만 요약(합 = Active Ropes). 부하가 솔버에서 나오는지 로직에서
//    나오는지 한눈에 — 페이즈 8종 내역은 'RopePerf' 게임플레이 디버거가 per-rope로 보여준다. ────────────
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Physics (Free/Flight)"), STAT_Rope_PhasePhysics, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Logic (Contacting..Reel)"), STAT_Rope_PhaseLogic, STATGROUP_DynamicRope, );

namespace RopeStats
{
	// 프레임 스칼라 — 서브시스템만 아는 값(SimFrame이 private, GPU 경로 분기)을 helper로 넘긴다. 나머지 로프별
	// 카운터(페이즈/GPU스텝/CPU솔브/슬립/파티클)는 RecordFrameStats가 public 게터로 직접 집계한다.
	struct FRopeFrameCounters
	{
		int32 NumGdfDispatched = 0;  // 이번 프레임 GDF와 함께 dispatch된 GPU 스텝 수(엔진 온디맨드 빌드 신호와 동일)
		int32 FrameColliders = 0;    // 전 로프 FrameColliders 합(SimFrame private → 서브시스템이 집계)
	};

	/** 'stat DynamicRope' 프레임 부하/페이즈 카운터 갱신(그룹 수집 중일 때만; 아니면 순회 스킵). 단계 타이밍은
	 *  SCOPE_CYCLE_COUNTER로 각 단계 스코프에서 직접 기록한다. */
	void RecordFrameStats(TConstArrayView<TObjectPtr<URopeComponent>> Ropes, const FRopeFrameCounters& Frame);
}
