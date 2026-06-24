// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — 실험용이며 출시 대상 아님. PoC/ 아래의 모든 것은 폐기 가능한 코드다.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "RopeCapsuleProvider.generated.h"

/** 월드 공간의 swept-sphere(capsule): 선분 [A, B]를 Radius만큼 부풀린 것. */
struct FRopeCapsule
{
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;
};

UINTERFACE(MinimalAPI)
class URopeCapsuleProvider : public UInterface
{
	GENERATED_BODY()
};

/**
 * rope solver에 capsule을 공급할 수 있는 모든 것 — 테스트용 capsule 액터,
 * 캐릭터의 skeletal limb 등. rope는 레벨 안의 provider를 자동으로 탐색한다.
 */
class IRopeCapsuleProvider
{
	GENERATED_BODY()

public:
	/** 이 provider의 현재 월드 공간 capsule들을 OutCapsules에 추가한다. */
	virtual void GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const = 0;

	/**
	 * S4 two-way coupling: rope가 이 provider의 capsule에 가하는 반작용을 보고한다 —
	 * 월드 공간 접촉점에서의 월드 공간 impulse. 뉴턴 제3법칙:
	 * rope가 바디 밖으로 밀어낸 만큼, 바디는 반대 방향으로 밀리는 힘을 느낀다.
	 * 기본 구현은 no-op이라, 움직이지 않는 provider(예: 애니메이션되는 skeletal limb)는 그냥 무시한다.
	 */
	virtual void ApplyRopeReaction(const FVector& WorldImpulse, const FVector& WorldLocation) {}
};
