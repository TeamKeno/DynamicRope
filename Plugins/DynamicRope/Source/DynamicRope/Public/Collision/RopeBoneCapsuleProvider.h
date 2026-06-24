// Copyright Epic Games, Inc. All Rights Reserved.
//
// Minimal IRopeColliderProvider: builds a capsule per listed bone (bone -> parent segment) from a
// skeletal mesh each frame. This is the v1 collider source for contact/wrap testing; the per-bone
// SDF provider replaces it later (M2-SDF) behind the same interface.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
#include "Collision/RopeCollider.h"
#include "RopeBoneCapsuleProvider.generated.h"

class USkeletalMeshComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeBoneCapsuleProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeBoneCapsuleProvider();

	/** Mesh whose bones become colliders. Auto-resolved from the owner if left null. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<USkeletalMeshComponent> SkeletalMesh = nullptr;

	/** Bones to expose as capsules. Each capsule spans the bone to its parent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TArray<FName> Bones;

	/** Capsule radius around each bone segment (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision", meta = (ClampMin = "0.0", Units = "cm"))
	float CapsuleRadius = 8.0f;

	/** Draw the generated bone capsules each frame (green = overlaps the rope bounds, grey = culled). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	bool bDrawDebug = false;

	//~ IRopeColliderProvider
	virtual void GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders) override;

private:
	// Backing storage rebuilt each GatherColliders; pointers handed out stay valid for the frame.
	TArray<FCapsuleCollider> Capsules;

	USkeletalMeshComponent* ResolveMesh();
};
