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
};
