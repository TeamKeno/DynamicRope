// Copyright Epic Games, Inc. All Rights Reserved.
//
// binding-semantics 레이어: wrap 이 결정된 *이후*의 모든 것. 이것은 물리가 아니라 LOGIC 이다 —
// 접촉 노드를 bone-local 로 latch 하고, skinning 으로 Hold 하고, pull 하고, release 한다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeSimTypes.h"
#include "Core/RopeTractionTypes.h"
#include "Core/RopeWrappingTypes.h"
// 랩 대상 추상화: FRopeBindingFrame/ResolveBindingWorld(Hold 바인딩, seam A) + FRopeWrapTargetKey(seam B)
#include "Core/RopeWrapTarget.h"

class USkeletalMeshComponent;
class IRopeCollider;

class DYNAMICROPE_API FRopeWrapController
{
public:
	FRopeWrapState State;

	/**
	 * 시드된 접촉 노드들을 bone-local 공간으로 동결(freeze)한다(physics → logic handoff).
	 * 감길 mesh 는 Seed.Mesh 로 확정되어 있어야 한다(접촉에서 전파 — cross-actor 포함).
	 * 없으면 아무것도 latch 하지 않고 상태를 리셋한다.
	 * 위치·질량 쓰기는 Sim 직접이 아니라 OutFrame(노드별 override 산출물)에 담는다(G2) —
	 * 호출자가 CPU Sim 적용과 GPU override 패킹에 같은 프레임을 쓴다.
	 */
	void BeginWrap(const FRopeSimState& Sim, const FRopeWrapState& Seed, FRopeNodeOverrideFrame& OutFrame);

	/**
	 * wrap 이 애니메이션을 따라가도록 매 프레임 latched 노드들의 (skinning 된) bone 위 타깃을
	 * OutFrame에 담는다(Pos=Prev=본 위 앵커, InvMass=0 — 적용은 호출자 통로).
	 * @return wrap 을 계속 유지할 수 있으면 true. 묶였던 mesh 가 사라졌으면(예: cross-actor 대상
	 *         액터 파괴) false — 호출자는 노드를 솔버에 돌려주고 release 해야 한다.
	 */
	bool Hold(const FRopeSimState& Sim, float Dt, FRopeNodeOverrideFrame& OutFrame);

	/**
	 * Pull(당김) 산출: 손 쪽 첫 앵커가 로프로부터 받는 당김(방향 + 장력)을 데이터로 채운다.
	 * 방향 = 앵커에서 손 쪽으로 로프를 따라 걸으며 찾은 "첫 직선 다리"의 끝 노드를 향하는 단위벡터.
	 * 걷는 중 다음 세그먼트가 지금까지의 누적 다리 방향에서 BendThresholdDeg 이상 꺾이면 멈춘다(코너). 곧은
	 * 로프는 손(노드 0)까지 걸어가 정확히 chord(앵커→손 직선)가 되고, 벽/모서리에 걸리면 그 직전에서 멈춰
	 * 로프의 실제 경로(첫 다리)를 따라 당긴다(직선 chord는 장애물을 관통). 누적 다리 방향 기준이라 한 노드의
	 * 처짐/지터로 조기 종료되지 않는다(공간 평균; 프레임 간 잔여 지터는 호출자의 EMA가 시간 평균). 장력 =
	 * 손 쪽 인접 세그먼트의 SegmentTension(슬랙이면 0이라 힘도 0). 힘 인가(캐릭터/물리 본)와 시간 스무딩은
	 * UObject/상태 작업이라 호출자(컴포넌트) 몫 — 여기는 순수 데이터(unit-test 가능).
	 * @return 유효한 앵커/세그먼트가 있어 Out이 채워졌으면 true(장력 0이어도 true).
	 */
	bool ComputePull(const FRopeSimState& Sim, float BendThresholdDeg, FRopePullSample& Out) const;

	/** unlatch 하고 제어권을 솔버에게 돌려준다. */
	void Release(ERopeReleaseReason Reason);

	bool IsActive() const { return State.IsWrapped(); }
};
