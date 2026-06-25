// Copyright Epic Games, Inc. All Rights Reserved.
//
// Position-based(XPBD) 로프 solver. UObject 의존성 없이 오직 FRopeSimState 위에서만 동작하므로,
// 유닛 테스트가 가능하고 추후 compute shader로 이식할 수 있다. Flight/Contacting 상태에서만 실행된다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class IRopeCollider;

class DYNAMICROPE_API FRopeXPBDSolver
{
public:
	/** 한 프레임 진행: substep 단위 integrate + distance/bending/collision 제약. */
	void Step(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, float DeltaSeconds) const;

private:
	void Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const;

	// XPBD distance: StretchCompliance로 segment 길이를 강제한다. Lambda는 substep의 iteration 전반에 걸쳐
	// 누적되며(segment 제약마다 한 항목), 이로써 강성이 step/iter 수에 독립적이게 된다.
	void SolveDistance(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	// XPBD bending: BendCompliance를 적용한 i<->i+2 "support stick"(rest = 2*SegmentLength).
	void SolveBending(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	// Config.CollisionRadius로 query하여(로프 두께) 노드를 표면 밖으로 push-out하고, Config.Friction으로
	// 접선 속도를 감쇠한다. ColliderBounds는 collider별 월드 AABB(+Radius)로, broad-phase에서 먼 collider의
	// 비싼 Query(역변환+SDF 샘플)를 건너뛰는 데 쓴다(Step에서 1회 계산해 전달).
	void SolveCollisions(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds) const;
};
