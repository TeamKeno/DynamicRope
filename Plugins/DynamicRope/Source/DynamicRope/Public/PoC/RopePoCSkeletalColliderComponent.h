// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. Everything under PoC/ is disposable.
// S3 (weeks 3-4): source rope-collision capsules from a character's animated limbs.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PoC/RopeCapsuleProvider.h"
#include "RopePoCSkeletalColliderComponent.generated.h"

class USkeletalMeshComponent;

/**
 * Feeds the rope solver with capsules taken from a skeletal mesh's bones.
 * Reads the mesh's Physics Asset bodies (capsules/spheres) and transforms them into
 * world space every frame, so they animate for free. Drop this on a character actor
 * and the rope (bAutoFindColliders) discovers it automatically.
 */
UCLASS(ClassGroup = (DynamicRopePoC), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopePoCSkeletalColliderComponent : public UActorComponent, public IRopeCapsuleProvider
{
	GENERATED_BODY()

public:
	URopePoCSkeletalColliderComponent();

	/** Skeletal mesh whose bones produce capsules. If null, the owner's first SkeletalMeshComponent is used. */
	UPROPERTY(EditAnywhere, Category = "Rope Collider")
	TObjectPtr<USkeletalMeshComponent> TargetMesh = nullptr;

	/** Only these bones produce capsules. Empty = every body in the physics asset. */
	UPROPERTY(EditAnywhere, Category = "Rope Collider")
	TArray<FName> Bones;

	/** Read capsules from the mesh's Physics Asset (recommended). Falls back to manual bone-to-child capsules if none. */
	UPROPERTY(EditAnywhere, Category = "Rope Collider")
	bool bUsePhysicsAsset = true;

	/** Radius used for the manual fallback when no physics asset is available (cm). */
	UPROPERTY(EditAnywhere, Category = "Rope Collider", meta = (ClampMin = "0.1", UIMin = "0.1", Units = "cm"))
	float ManualRadius = 8.0f;

	/** Draw the generated capsules each frame. */
	UPROPERTY(EditAnywhere, Category = "Rope Collider")
	bool bDrawDebug = false;

	//~ IRopeCapsuleProvider
	virtual void GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const override;

private:
	USkeletalMeshComponent* ResolveMesh() const;
	void GatherFromPhysicsAsset(USkeletalMeshComponent* Mesh, TArray<FRopeCapsule>& OutCapsules) const;
	void GatherManual(USkeletalMeshComponent* Mesh, TArray<FRopeCapsule>& OutCapsules) const;
	void DrawCapsules(const TArray<FRopeCapsule>& Capsules, int32 FirstNew) const;
};
