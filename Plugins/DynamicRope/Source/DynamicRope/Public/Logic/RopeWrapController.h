// Copyright Epic Games, Inc. All Rights Reserved.
//
// binding-semantics 레이어: wrap 이 결정된 *이후*의 모든 것. 이것은 물리가 아니라 LOGIC 이다 —
// 접촉 노드를 bone-local 로 latch 하고, skinning 으로 Hold 하고, pull 하고, release 한다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class USkeletalMeshComponent;
class IRopeCollider;

class DYNAMICROPE_API FRopeWrapController
{
public:
	FRopeWrapState State;

	/**
	 * 접촉 결정(physics → logic 게이트). 각 노드를 콜라이더들에 대해 질의하여 dominant 하게 접촉된 bone 을
	 * 찾고, 커밋하기 전에 MinLatchNodes 개의 노드가 WrapDecisionTime 동안 지속적으로 접촉할 것을
	 * 요구한다. 한 번 true 를 반환하며 BeginWrap 을 위해 OutSeed(노드 인덱스 + bone)를 채운다.
	 * 후보를 프레임에 걸쳐 내부적으로 추적한다. Contacting 틱마다 호출한다.
	 */
	bool DecideWrap(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
		const FRopeWrapConfig& Config, float Dt, FRopeWrapState& OutSeed);

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

	/** 붙잡힌 limb 를 절차적으로 타깃 쪽으로 끌어당긴다(IK/tension). */
	void Pull(FRopeSimState& Sim, const FVector& PullTarget);

	/** unlatch 하고 제어권을 솔버에게 돌려준다. */
	void Release(ERopeReleaseReason Reason);

	bool IsActive() const { return State.IsWrapped(); }

private:
	// 결정을 위한 지속 접촉 누적값(일시적이며 wrap 상태의 일부가 아니다).
	FName         CandidateBone = NAME_None;
	float         CandidateTime = 0.0f;
	TArray<int32> CandidateNodes;
};
