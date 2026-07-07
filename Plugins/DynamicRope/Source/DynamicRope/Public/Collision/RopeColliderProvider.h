// Copyright Epic Games, Inc. All Rights Reserved.
//
// 매 프레임 rope solver에 collider를 공급하는 컴포넌트/오브젝트. skeletal provider는
// 후보 bone들에 대해 per-bone collider(지금은 capsule, 추후 SDF)를 빌드한다.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "RopeColliderProvider.generated.h"

class IRopeCollider;

UINTERFACE(MinimalAPI)
class URopeColliderProvider : public UInterface
{
	GENERATED_BODY()
};

class IRopeColliderProvider
{
	GENERATED_BODY()

public:
	/**
	 * RopeBounds와 겹치는 collider를 추가한다(broad phase는 여기서 수행). 가리키는 collider들은
	 * 해당 프레임 solve가 끝날 때까지 유효 상태를 유지해야 한다.
	 */
	virtual void GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders) = 0;

	/**
	 * 이 provider가 정적 월드 지오메트리 collider를 공급하는지(예: URopeStaticBodyProvider). true면
	 * 서브시스템의 로프별 "자기 owner provider 제외"에서 면제된다 — 정적 월드는 "던진 본인의 몸"이
	 * 될 수 없는데, 로프 소유 액터에 붙였다는 이유만으로 월드 충돌이 조용히 사라지는 것을 막는다.
	 */
	virtual bool ProvidesWorldStaticColliders() const { return false; }
};
