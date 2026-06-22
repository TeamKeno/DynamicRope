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

	/**
	 * S4 two-way coupling: the rope reports the reaction it exerts on this provider's
	 * capsule(s) — a world-space impulse at a world-space contact point. Newton's 3rd law:
	 * what the rope pushed out of the body, the body feels pushed in the opposite sense.
	 * Default no-op so providers that don't move (e.g. animated skeletal limbs) just ignore it.
	 */
	virtual void ApplyRopeReaction(const FVector& WorldImpulse, const FVector& WorldLocation) {}
};
