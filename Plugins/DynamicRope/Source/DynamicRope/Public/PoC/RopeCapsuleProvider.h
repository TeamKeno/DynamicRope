// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. Everything under PoC/ is disposable.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "RopeCapsuleProvider.generated.h"

/** A world-space swept-sphere (capsule): the segment [A, B] inflated by Radius. */
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
 * Anything that can feed capsules to the rope solver — a test capsule actor,
 * a character's skeletal limbs, etc. The rope auto-discovers providers in the level.
 */
class IRopeCapsuleProvider
{
	GENERATED_BODY()

public:
	/** Append this provider's current world-space capsules to OutCapsules. */
	virtual void GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const = 0;
};
