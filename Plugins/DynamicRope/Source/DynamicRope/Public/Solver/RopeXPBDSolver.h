// Copyright Epic Games, Inc. All Rights Reserved.
//
// Position-based(XPBD) 로프 solver. UObject 의존성 없이 오직 FRopeSimState 위에서만 동작하므로
// 유닛 테스트가 가능하다. 런타임 정규 경로는 GPU 이식본(FRopeGPUSolver, RopeXPBD.usf)이고, 이 CPU
// 구현은 폴백(cook/-nullrhi/서버/노드 수 초과) + 파리티/유닛 테스트 기준점이다. Free/Flight에서 전체를,
// Wrapping/Wrapped에서는 질량 마스크되지 않은 자유 구간만 돈다(Contacting/Releasing은 솔브 없음).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeConfigTypes.h"
#include "Core/RopeSimTypes.h"

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
 * 노드 1개의 접촉 제약 상태. DetectContacts(substep당 1회 swept, CCD)가 활성 여부와 접촉 법선/표면
 * 속도를 정하고, SolveContacts는 매 iteration 후보 collider들을 *fresh로 재질의*해 현재 위치의 실제
 * 표면 거리/법선을 얻어 재투영한다 → 곡면/오목 크리스에서도 캐시 평면 staleness 없이 collision이
 * distance/bending과 동등하게 경쟁한다. Lambda는 누적 법선 임펄스(= 접촉 법선력)로 마찰
 * Coulomb 한계 μ·Lambda·w에 쓰인다(XPBD: λ가 곧 제약력). Normal/SurfaceVel은 매 재질의마다 갱신.
 */
struct FRopeContactState
{
	bool    bActive = false;

	FVector Normal = FVector::ZeroVector;
	FVector SurfaceVel = FVector::ZeroVector;
	float   Lambda = 0.0f;
};

/**
 * 한 detect 패스에서 추려둔 노드/세그먼트별 collider 후보 목록.
 *
 * DetectContacts는 어차피 (노드 × collider) broad-phase 박스 판정을 전부 도는데 그 결과를 버리는 바람에,
 * SolveContacts/SolveSegmentContacts가 매 iteration 전 collider를 다시 훑고 있었다(CPU 폴백 프레임 비용의
 * 지배항 — 특히 활성 게이트가 아예 없던 세그먼트 충돌). 그 판정을 detect 시점에 한 번만 하고 그 패스의
 * iteration들이 재사용한다: iteration당 O(N·C)가 O(N·후보수)로 줄고, 아무 collider와도 안 겹치는
 * 세그먼트는 내부 샘플 루프 진입 자체를 건너뛴다.
 *
 * 후보는 노드 sweep AABB / 세그먼트 구간 AABB를 Margin(세그먼트 rest 길이)만큼 넓혀 뽑는다 — iteration
 * 도중 distance/bending/세그먼트 보정이 노드를 끌어당겨도 그 여유 안에서는 collider를 놓치지 않는다.
 * 한 항목에 겹치는 collider가 MaxPerItem을 넘으면 그 항목만 전량 루프로 폴백하므로, 어떤 경우에도
 * 검출 결과가 줄지 않는다(순수한 비용 절감이지 동작 변경이 아니다).
 */
struct DYNAMICROPE_API FRopeColliderCandidates
{
	/** 항목(노드/세그먼트) 1개당 후보 상한. 초과하면 그 항목만 전량 루프 폴백. */
	static constexpr int32 MaxPerItem = 12;

	/** false면 후보를 못 만든 상태(broad-phase bounds 부재) → 호출부는 전량 루프. */
	bool bValid = false;

	/** 노드별 sweep AABB(Prev→Pos). collider마다 다시 만들지 않도록 detect 앞머리에서 1회 채운다. */
	TArray<FBox>  NodeBounds;

	/** 노드별 후보 collider 인덱스. stride = MaxPerItem. */
	TArray<int32> NodeIndices;
	TArray<int32> NodeNum;
	TArray<bool>  bNodeOverflow;

	/** 세그먼트 k(= 노드 k~k+1) 별 후보. stride = MaxPerItem. */
	TArray<int32> SegIndices;
	TArray<int32> SegNum;
	TArray<bool>  bSegOverflow;

	/** 이번 detect 패스분을 비운다. 버퍼 할당은 유지되므로 substep마다 재할당하지 않는다. */
	void Reset(int32 NumNodes);

	void AddNode(int32 NodeIndex, int32 ColliderIndex);
	void AddSegment(int32 SegIndex, int32 ColliderIndex);

	/**
	 * 노드/세그먼트 하나를 풀 때 순회할 개수. bOutAll이 true면 후보를 못 쓰는 경우라 반환값은 전체
	 * collider 수이고 슬롯 번호가 곧 collider 인덱스다(폴백). false면 NodeAt/SegAt으로 풀어야 한다.
	 */
	int32 NodeCount(int32 NodeIndex, int32 NumColliders, bool& bOutAll) const;
	int32 SegCount(int32 SegIndex, int32 NumColliders, bool& bOutAll) const;

	FORCEINLINE int32 NodeAt(int32 NodeIndex, int32 Slot) const { return NodeIndices[NodeIndex * MaxPerItem + Slot]; }
	FORCEINLINE int32 SegAt(int32 SegIndex, int32 Slot) const { return SegIndices[SegIndex * MaxPerItem + Slot]; }
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
	 * 여기서 어차피 도는 broad-phase 판정을 Out Candidates(노드/세그먼트별 collider 후보)로 남겨,
	 * 뒤따르는 iteration들이 같은 판정을 되풀이하지 않게 한다.
	 */
	void DetectContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		float SubAlpha0, float SubAlpha1, TArray<FRopeContactState>& Contacts,
		FRopeColliderCandidates& Candidates) const;

	/**
	 * 활성 노드에 *근접한 모든* collider를 매 iteration fresh로 재질의(point query)해 각각 표면 밖으로 재투영하고
	 * 법선 임펄스 Lambda(>=0, 한쪽 접촉)를 누적한다. rigid(compliance 0). 캐시 하나가 아니라 겹치는 뼈들을 모두
	 * 방어하므로(단일-캐시 관통 버그 수정 — GPU .usf의 노드당 전 collider 루프와 일치), distance/bending과 같은
	 * Gauss-Seidel sweep에서 경쟁 → 장력에 안 밀린다. 순회 대상은 Candidates가 detect 때 추려둔 그 노드의
	 * 후보뿐이고(상한 초과 시에만 전량 폴백), ColliderBounds는 그 안에서 노드-점 broad-phase 컬로 남는다.
	 */
	void SolveContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		const FRopeColliderCandidates& Candidates, TArray<FRopeContactState>& Contacts) const;

	/**
	 * 세그먼트(에지) 충돌: 노드 점 충돌은 두 노드 사이 직선이 얇은 표면(팔·다리 등)을 가로지르는 chording을
	 * 못 막는다(양 끝 노드는 표면 밖, 사이 직선만 관통). 각 세그먼트를 내부 샘플점(길이/SweepStep 기반)으로
	 * 보고, 침투한 샘플을 표면 밖으로 밀며 보정을 barycentric((1-t):t)으로 양 끝 노드에 분배한다. 양 끝이 모두
	 * pin(invMass 0)인 wrap 구간은 못 움직이므로 스킵. distance/bending과 같은 sweep에서 경쟁하도록 매 iteration 호출.
	 * Candidates에 후보가 하나도 없는 세그먼트는 내부 샘플 루프 진입 전에 통째로 스킵한다(대부분의 세그먼트가
	 * 여기 해당 — 이 게이트가 없어서 세그먼트 충돌이 CPU 폴백 최대 비용원이었다).
	 */
	void SolveSegmentContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		const FRopeColliderCandidates& Candidates, bool bReverse) const;

	/**
	 * substep 끝에 Coulomb 마찰 1회 적용: 접선 보정량을 μ(테이퍼)·Lambda·w로 상한(Lambda=누적 법선력). 작은
	 * 상대 운동은 전량 제거(정지마찰), 그립 초과분은 슬립. SubDt로 표면 속도(cm/s)를 substep 변위로 환산.
	 */
	void ApplyContactFriction(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<FRopeContactState>& Contacts, float SubDt) const;

	/**
	 * Strain limiting(최대 신장 클램프): XPBD iteration이 적으면 긴 체인이 고정 노드(핀/앵커, InvMass 0)에
	 * 매달릴 때 Gauss-Seidel 보정이 끝까지 전파되지 못해 고정 노드 인접 세그먼트에 신장이 폭주한다. 순차
	 * sweep(전방+후방)으로 각 세그먼트를 ≤ MaxStretchRatio × SegmentLength로 하드 투영해 한 번에 체인 전체로
	 * 보정을 전파한다. 위치 이동은 속도 중립(prev도 함께 이동) — 클램프가 Verlet 속도를 주입/제거하지 않는다.
	 * MaxStretchRatio < 1이면 no-op. GPU RopeXPBD.usf의 strain-limit 스테이지와 미러.
	 */
	void SolveStrainLimit(FRopeSimState& State, const FRopeSolverConfig& Config) const;
};
