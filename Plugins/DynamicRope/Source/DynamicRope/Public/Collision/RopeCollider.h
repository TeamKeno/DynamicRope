// Copyright Epic Games, Inc. All Rights Reserved.
//
// Collider abstraction. The solver queries IRopeCollider and never knows whether it is a
// capsule, a per-bone SDF, or the world distance field.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class USkeletalMeshComponent;

/** Abstract collider the rope solver queries. */
class DYNAMICROPE_API IRopeCollider
{
public:
	virtual ~IRopeCollider() = default;

	/** Closest-surface query for a rope node of the given radius. Returns penetration + normal. */
	virtual FRopeContact Query(const FVector& WorldPos, float Radius) const = 0;

	/** World-space bounds for broad-phase culling. */
	virtual FBox GetWorldBounds() const = 0;
};

/** Analytic capsule (swept-sphere segment). v1 / fallback; later replaced by per-bone SDF. */
class DYNAMICROPE_API FCapsuleCollider : public IRopeCollider
{
public:
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;
	FName   Bone = NAME_None;

	// Skeletal mesh this capsule's bone belongs to. Carried into the contact so the wrap can
	// follow the *correct* mesh (the one that owns the caught bone), even across actors.
	const USkeletalMeshComponent* SourceMesh = nullptr;

	FCapsuleCollider() = default;
	FCapsuleCollider(const FVector& InA, const FVector& InB, float InRadius, FName InBone = NAME_None,
		const USkeletalMeshComponent* InSourceMesh = nullptr)
		: A(InA), B(InB), Radius(InRadius), Bone(InBone), SourceMesh(InSourceMesh) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FBox GetWorldBounds() const override;
};
