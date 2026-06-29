// Copyright Epic Games, Inc. All Rights Reserved.
//
// 닫힌(또는 거의 닫힌) 삼각형 메시 전체에 대한 generalized winding number로 점의 안/밖을 판정한다.
// UE GeometryCore의 fast winding number(BVH + 다극 근사, query당 O(log T))를 사용하며, 베이커가
// 본별 거리에 부호를 매길 때 쓴다. 부호 단위 테스트도 동일 경로를 검증하도록 이 분류기를 공유한다.
//
// 왜 본별이 아니라 전체 메시인가: "안/밖"은 몸 전체(닫힌 표면)에 대한 전역 속성이다. 본 하나의 삼각형
// 패치는 열려 있어, 짧고 넓은 본 토막은 내부에서도 winding이 0.5를 못 넘어 바깥으로 오판된다.

#pragma once

#include "CoreMinimal.h"

/** 메시 전체 fast winding number 기반 안/밖 분류기. 빌드 후 동시(read-only) 질의 안전. */
class FRopeSDFWindingClassifier
{
public:
	// 삼각형 소프로 분류기를 만든다(내부적으로 정점을 삼각형별로 복제 → 시임/비매니폴드에도 강건).
	// 삼각형 t의 정점 = Positions[Indices[3t+0..2]]. Indices.Num()은 3의 배수, 값은 Positions 범위 내여야
	// 한다(범위 밖 삼각형은 건너뜀). 모두 같은 좌표 공간(예: 컴포넌트 공간)이어야 하며 질의 점도 동일 공간.
	FRopeSDFWindingClassifier(TConstArrayView<FVector3f> Positions, TConstArrayView<uint32> Indices);
	~FRopeSDFWindingClassifier();

	// |generalized winding number(P)| > 0.5 이면 안쪽. 닫힌 메시에서 내부 |w|≈1, 바깥 ≈0의 중점이며,
	// abs라 메시 삼각형 방향(CW/CCW)과 무관하게 동작한다. P는 생성 시 정점과 같은 공간.
	bool IsInside(const FVector& P) const;

	// 유효한 트리가 만들어졌는가(삼각형 0개면 false → IsInside는 항상 false).
	bool IsValid() const { return bValid; }

private:
	// GeometryCore 타입을 헤더에 노출하지 않도록 Pimpl(이 헤더를 포함하는 쪽은 GeometryCore 불필요).
	struct FImpl;
	TUniquePtr<FImpl> Impl;
	bool bValid = false;
};
