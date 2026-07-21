// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class USceneComponent;

/**
 * narrow-phase 컨택트: rope 노드 하나 vs collider 하나, IRopeCollider::Query가 반환한다.
 *
 * CONTRACT — FROZEN 2026-06-24 (2026-06-27 SurfaceVelocity 추가: 기본 0인 가산 필드라 하위호환).
 * 모든 IRopeCollider(capsule, bone-SDF, world-GDF)는 이를 반드시 준수해야 한다.
 * 단일 (node, collider) 쌍을 기술한다. 집계는 호출자의 몫이다(solver는 push-out을 합산하고,
 * DecideWrap은 노드별로 penetration이 가장 깊은 bone을 선택한다).
 *
 *   bHit         노드 구체(center = query WorldPos, radius = query Radius)가 collider와 겹친다.
 *                false => 나머지 필드는 모두 정의되지 않음. 호출자는 이를 무시해야 한다.
 *   Normal       UNIT, collider에서 노드를 향해 바깥쪽을 가리킨다(push-out 방향).
 *                불변식: NodePos += Normal*Penetration 은 노드를 표면 위에 올려놓는다.
 *                *** 부호가 load-bearing이다: 안쪽을 향하는 normal은 rope를 몸체 안으로 빨아들인다. ***
 *                축퇴(노드가 medial axis 위에 있음) => 임의의 안정적인 단위 벡터(capsule: +Z).
 *   Penetration  Normal을 따른 overlap 깊이, bHit일 때 > 0. QUERY 반지름 기준으로 측정된다:
 *                (ColliderRadius + QueryRadius) - Distance. 호출자는 solver push-out에는 QueryRadius 0을,
 *                wrap-decision skin에는 WrapConfig.ContactQueryRadius를 전달한다.
 *   SurfacePoint 노드에서 가장 가까운 collider 표면 위의 점(보조/디버그). solver에는 필수가 아니며,
 *                저렴하게 구할 수 있을 때 채운다.
 *   Bone         skeletal collider에서는 반드시 non-None — bone 귀속(attribution)으로 DecideWrap이
 *                wrap을 건다. 멀티-bone SDF는 가장 가까운 표면을 소유한 bone을 반드시 보고해야 한다. world => None.
 *   SourceMesh   Bone을 소유한 skeletal mesh. 액터 간 follow를 전달한다(-> FRopeWrapState::Mesh).
 *                비-skeletal collider에서는 null.
 *   SurfaceVelocity 접촉점에서 collider 표면의 월드 속도(cm/s). solver가 상대 접선 속도 마찰로
 *                로프를 끌고 가는 데 쓴다(움직이는 몸이 정지한 로프를 좌우로 쓸어내게 함).
 *                정적/미지원 collider는 0(= 정적 표면)으로 둔다 — 기존 동작과 동일.
 */
struct FRopeContact
{
	bool    bHit = false;
	FVector Normal = FVector::UpVector;
	float   Penetration = 0.0f;
	FVector SurfacePoint = FVector::ZeroVector;
	FName   Bone = NAME_None;
	const USceneComponent* SourceMesh = nullptr;
	FVector SurfaceVelocity = FVector::ZeroVector;
};

/** rope 중심선: 파티클의 체인. solver / 로직 / 렌더의 단일 진실 공급원(single source of truth). */
struct FRopeSimState
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;
	float           SegmentLength = 0.0f;
	float           RopeLength = 0.0f;

	/**
	 * 고정된 시작점(hand/socket). solver는 substep에 걸쳐 Prev->Target으로 쓸어 이동시키므로
	 * 빠른 앵커 점프가 에너지를 주입하는(체인을 폭발시킬) 대신 흡수된다.
	 */
	bool            bStartPinned = false;
	FVector         StartPinPrev = FVector::ZeroVector;
	FVector         StartPinTarget = FVector::ZeroVector;

	/** Fixed timestep accumulator. The solver consumes real frame time in fixed-size substeps. */
	float           TimeAccumulator = 0.0f;

	/**
	 * 세그먼트별 장력(힘, 스트레치=양수만). XPBD distance 제약의 수렴 λ에서 유도: F = max(0, -λ)/h².
	 * 단위는 질량 1 노드 기준 mass·cm/s²(상대값) — 임계치는 실측으로 튜닝한다. CPU 솔버가 Step 끝에
	 * 채우고, GPU 상주 로프는 λ 리드백(1~2프레임 지연)이 채운다. 솔브 없는 프레임은 직전 값 유지.
	 * 크기 = Num()-1(비어 있을 수 있음 — 아직 한 번도 솔브 안 됨).
	 */
	TArray<float>   SegmentTension;

	int32 Num() const { return Positions.Num(); }
	void  Reset() { Positions.Reset(); PrevPositions.Reset(); InvMass.Reset(); SegmentTension.Reset(); TimeAccumulator = 0.0f; }

	//~ Verlet 어휘(순수 인라인 — 컨텍스트/정책 없음). 반복 관용구에 이름을 붙여 부호·차원 실수를 막는다.
	//  솔버 적분 루프(RopeXPBDSolver)와 던지기 속도 주입 루프는 의도적으로 raw 표현을 유지한다 —
	//  전자는 .usf 커널과의 1:1 파리티 대조가 우선, 후자는 누적형(변위 단위 임펄스)이라 형태가 다르다.

	/** 노드 i의 한 프레임 변위(Pos - Prev). Verlet에서 속도 ∝ 변위(dt 나누기 전). */
	FVector Displacement(int32 i) const { return Positions[i] - PrevPositions[i]; }

	/** 노드 i의 한 프레임 이동 거리(cm/프레임). "빠른 노드" 등 임계 판정은 소비자의 정책이다. */
	float NodeSpeed(int32 i) const { return Displacement(i).Size(); }

	/** 노드 i의 속도 0(Prev = Pos). 시드/리시드 경로 전용 — 로직 페이즈의 위치·속도 쓰기는
	 *  FRopeNodeOverrideFrame 단일 통로를 탄다(G2, GPU 상주 동기화). */
	void SetStill(int32 i) { PrevPositions[i] = Positions[i]; }
};

/**
 * FRopeNodeOverrideFrame::Flags의 노드별 비트. ERopeGPUOverride(RopeGPUSolver.h)와 수치 1:1이어야
 * 한다(서브시스템이 검증) — Core는 Shaders 모듈에 의존하지 않으므로 상수를 미러로 둔다.
 */
namespace RopeNodeOverride
{
	/** Pos[i] = Positions[i] */
	constexpr uint8 Position         = 1 << 0;

	/** Prev[i] = PrevPositions[i] (Verlet 속도 주입) */
	constexpr uint8 Prev             = 1 << 1;

	/** Prev[i] = Pos[i] (속도 0; Position 적용 *후* 값) */
	constexpr uint8 PrevFromPosition = 1 << 2;

	/** InvMass[i] = InvMass[i] */
	constexpr uint8 InvMass          = 1 << 3;
}

/**
 * 로직 페이즈의 한 프레임 산출물(G2): "타깃 계산은 GT, 적용은 통로 하나로".
 * Wrapping/Wrapped/Releasing 등 로직이 Sim에 쓰고 싶은 위치·속도·질량을 여기에 scatter하면,
 * PrepareSimFrame 끝에서 CPU Sim에 1회 적용되고(ApplyToSim — 기존 직접 쓰기와 동일한 결과),
 * GPU 상주 로프에는 같은 데이터가 override 패스(FRopeGPUResidentStep)로 실려 재시드 없이
 * 커널에서 적용된다. 같은 노드를 여러 번 채우면 나중 것이 이긴다(순차 Sim 쓰기와 동일).
 * 주의: Prev(명시)와 PrevFromPosition을 한 프레임에 섞어 채우지 말 것 — 커널 적용 순서상
 * PrevFromPosition이 항상 이겨 채운 순서와 무관해진다(로직 페이즈는 PrevFromPosition만 쓴다).
 */
struct FRopeNodeOverrideFrame
{
	/** 노드별 RopeNodeOverride 비트 OR(비어 있으면 이번 프레임 산출물 없음). */
	TArray<uint8>   Flags;
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;

	bool HasAny() const { return Flags.Num() > 0; }

	void Reset()
	{
		Flags.Reset();
		Positions.Reset();
		PrevPositions.Reset();
		InvMass.Reset();
	}

	/** 첫 scatter 시 노드 수만큼 0으로 확보(프레임 내 재호출은 no-op). */
	void EnsureSize(int32 NumNodes)
	{
		if (Flags.Num() != NumNodes)
		{
			Flags.SetNumZeroed(NumNodes);
			Positions.SetNumZeroed(NumNodes);
			PrevPositions.SetNumZeroed(NumNodes);
			InvMass.SetNumZeroed(NumNodes);
		}
	}

	/** 위치 고정: Pos=World, bZeroVelocity면 Prev=Pos(속도 0 — wrapping/hold의 표준 쓰기). */
	void SetPosition(int32 NodeIndex, const FVector& World, bool bZeroVelocity)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::Position | (bZeroVelocity ? RopeNodeOverride::PrevFromPosition : 0);
			Positions[NodeIndex] = World;
		}
	}

	/** 질량 덮어쓰기(마스크/복원). */
	void SetInvMass(int32 NodeIndex, float Value)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::InvMass;
			InvMass[NodeIndex] = Value;
		}
	}

	/** 속도 제거만(Prev=현재 Pos — 위치는 그대로). release 계열의 튐 방지. */
	void SetPrevFromPosition(int32 NodeIndex)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::PrevFromPosition;
		}
	}

	/** CPU 적용 — GPU 커널의 override 스테이지와 같은 순서(Pos → Prev → Prev=Pos → InvMass). */
	void ApplyToSim(FRopeSimState& Sim) const
	{
		const int32 N = FMath::Min(Flags.Num(), Sim.Num());
		for (int32 i = 0; i < N; ++i)
		{
			const uint8 F = Flags[i];
			if (F == 0)
			{
				continue;
			}
			if (F & RopeNodeOverride::Position)         { Sim.Positions[i] = Positions[i]; }
			if (F & RopeNodeOverride::Prev)             { Sim.PrevPositions[i] = PrevPositions[i]; }
			if (F & RopeNodeOverride::PrevFromPosition) { Sim.PrevPositions[i] = Sim.Positions[i]; }
			if (F & RopeNodeOverride::InvMass)          { Sim.InvMass[i] = InvMass[i]; }
		}
	}
};
