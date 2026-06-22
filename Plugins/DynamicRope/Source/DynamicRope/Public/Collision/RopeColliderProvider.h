// Copyright Epic Games, Inc. All Rights Reserved.
//
// A component/object that supplies colliders to the rope solver each frame. The skeletal
// provider builds per-bone colliders (capsule now, SDF later) for the candidate bones.

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
	 * Append colliders overlapping RopeBounds (broad phase done here). The pointed-to colliders
	 * must stay valid for the remainder of the frame's solve.
	 */
	virtual void GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders) = 0;
};
