// Copyright Epic Games, Inc. All Rights Reserved.
//
// Position-based(XPBD) 로프 solver. UObject 의존성 없이 오직 FRopeSimState 위에서만 동작하므로
// 유닛 테스트가 가능하다. 런타임 정규 경로는 GPU 이식본(FRopeGPUSolver, RopeXPBD.usf)이고, 이 CPU
// 구현은 폴백(cook/-nullrhi/서버/노드 수 초과) + 파리티/유닛 테스트 기준점이다. Free/Flight에서 전체를,
// Wrapping/Wrapped에서는 질량 마스크되지 않은 자유 구간만 돈다(Contacting/Releasing은 솔브 없음).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class IRopeCollider;

/** 한 프레임의 고정 timestep substep 스케줄. CPU 솔버와 GPU 솔버가 공유한다. */
struct FRopeSubstepSchedule
{
	/** 이번 프레임에 돌릴 substep 수(0이면 이번 프레임 솔브 스킵). */
	int32 NumSub = 0;

	/** substep당 고정 dt(초). */
	float FixedDt = 0.0f;
};

/**
 * State.TimeAccumulator에 DeltaSeconds를 누적하고, 고정 크기 substep 단위로 소비하여 이번 프레임의
 * 스케줄을 반환한다(spiral-of-death 상한 포함). accumulator를 갱신(차감)하므로 State는 비-const.
 * CPU(FRopeXPBDSolver::Step)와 GPU(FRopeGPUSolver) 양쪽이 동일 스케줄을 쓰도록 한 곳으로 추출한 것.
 */
DYNAMICROPE_API FRopeSubstepSchedule RopeSolverSubsteps(FRopeSimState& State, const FRopeSolverConfig& Config, float DeltaSeconds);

/**
 * 노드 1개의 접촉 제약 상태. DetectContacts(substep당 1회 swept, CCD)가 어느 collider에 닿았는지
 * (ColliderIndex)와 활성 여부를 정한다. SolveContacts는 매 iteration 그 collider를 *fresh로 재질의*해
 * 현재 위치의 실제 표면 거리/법선을 얻어 재투영한다 → 곡면/오목 크리스에서도 캐시 평면 staleness 없이
 * collision이 distance/bending과 동등하게 경쟁한다. Lambda는 누적 법선 임펄스(= 접촉 법선력)로 마찰
 * Coulomb 한계 μ·Lambda·w에 쓰인다(XPBD: λ가 곧 제약력). Normal/SurfaceVel은 매 재질의마다 갱신.
 */
struct FRopeContactState
{
	bool    bActive = false;

	/** SolveContacts가 매 iteration 재질의할 collider. */
	int32   ColliderIndex = INDEX_NONE;

	FVector Normal = FVector::ZeroVector;
	FVector SurfaceVel = FVector::ZeroVector;
	float   Lambda = 0.0f;
};

class DYNAMICROPE_API FRopeXPBDSolver
{
public:
	/** 한 프레임 진행: substep 단위 integrate + distance/bending/collision 제약. */
	void Step(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, float DeltaSeconds) const;

private:
	void Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const;

	/**
	 * XPBD distance: StretchCompliance로 segment 길이를 강제한다. Lambda는 substep의 iteration 전반에 걸쳐
	 * 누적되며(segment 제약마다 한 항목), 이로써 강성이 step/iter 수에 독립적이게 된다.
	 */
	void SolveDistance(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	/** XPBD bending: BendCompliance를 적용한 i<->i+2 "support stick"(rest = 2*SegmentLength). */
	void SolveBending(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	/**
	 * 접촉 검출(substep당 1회 또는 CollisionPasses회): swept query(CCD)로 각 노드의 첫 접촉을 찾아 표면 밖으로
	 * 즉시 push-out하고, 접촉면을 평면(RestPoint/Normal)으로 캐시한다(Out Contacts). ColliderBounds는 broad-phase
	 * AABB(+Radius), SubAlpha0/1은 움직이는 collider의 substep sub-포즈 구간. 이후 SolveContacts가 매 iteration
	 * 이 캐시 평면을 싸게 강제하고, ApplyContactFriction이 substep 끝에 Coulomb 마찰을 적용한다.
	 */
	void DetectContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		float SubAlpha0, float SubAlpha1, TArray<FRopeContactState>& Contacts) const;

	/**
	 * 활성 노드에 *근접한 모든* collider를 매 iteration fresh로 재질의(point query)해 각각 표면 밖으로 재투영하고
	 * 법선 임펄스 Lambda(>=0, 한쪽 접촉)를 누적한다. rigid(compliance 0). 캐시 하나가 아니라 겹치는 뼈들을 모두
	 * 방어하므로(단일-캐시 관통 버그 수정 — GPU .usf의 노드당 전 collider 루프와 일치), distance/bending과 같은
	 * Gauss-Seidel sweep에서 경쟁 → 장력에 안 밀린다. ColliderBounds는 노드-점 broad-phase 컬(먼 collider 스킵).
	 */
	void SolveContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		TArray<FRopeContactState>& Contacts) const;

	/**
	 * 세그먼트(에지) 충돌: 노드 점 충돌은 두 노드 사이 직선이 얇은 표면(팔·다리 등)을 가로지르는 chording을
	 * 못 막는다(양 끝 노드는 표면 밖, 사이 직선만 관통). 각 세그먼트를 내부 샘플점(길이/SweepStep 기반)으로
	 * 보고, 침투한 샘플을 표면 밖으로 밀며 보정을 barycentric((1-t):t)으로 양 끝 노드에 분배한다. 양 끝이 모두
	 * pin(invMass 0)인 wrap 구간은 못 움직이므로 스킵. distance/bending과 같은 sweep에서 경쟁하도록 매 iteration 호출.
	 */
	void SolveSegmentContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds, bool bReverse) const;

	/**
	 * substep 끝에 Coulomb 마찰 1회 적용: 접선 보정량을 μ(테이퍼)·Lambda·w로 상한(Lambda=누적 법선력). 작은
	 * 상대 운동은 전량 제거(정지마찰), 그립 초과분은 슬립. SubDt로 표면 속도(cm/s)를 substep 변위로 환산.
	 */
	void ApplyContactFriction(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<FRopeContactState>& Contacts, float SubDt) const;
};
