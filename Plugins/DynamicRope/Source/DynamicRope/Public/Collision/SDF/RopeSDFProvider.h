// Copyright Epic Games, Inc. All Rights Reserved.
//
// The skeletal collider provider that supplies URopeSDFData as IRopeColliders. It builds an
// FRopeSDFCollider each frame by transforming the per-bone volumes by their current bone-to-world
// transforms. It shares its base, URopeSkeletalColliderProvider, with URopeBoneCapsuleProvider, so
// the two can coexist or replace one another without blocking. Unbaked volumes are skipped, which
// makes it a safe no-op with no data. Registration, mesh resolution, per-frame deduplication and the
// gather pipeline all belong to the base.

#pragma once

#include "CoreMinimal.h"
#include "Collision/RopeSkeletalColliderProvider.h"
#include "Collision/SDF/RopeSDFCollider.h"
#include "RopeSDFProvider.generated.h"

class URopeSDFData;
class USkeletalMeshComponent;

/** The axis the SDF slice heatmap runs along; the plane it draws is parallel to the other two axes. */
UENUM()
enum class ERopeSDFSliceAxis : uint8
{
	X,
	Y,
	Z
};

/** Chooses which of the baked bones are exposed as colliders. It is a runtime filter and leaves the
 *  bake untouched. */
UENUM()
enum class ERopeSDFBoneFilterMode : uint8
{
	/** Use every baked bone (default - no filtering). */
	All,
	/** Collide only with the bones listed in Bone Filter. */
	Include,
	/** Collide with every baked bone except those listed in Bone Filter. */
	Exclude
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeSDFProvider : public URopeSkeletalColliderProvider
{
	GENERATED_BODY()

public:
	/** The per-bone SDF volume asset. No colliders are supplied while it is empty. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<URopeSDFData> SDFData = nullptr;

	/**
	 * Runtime filter selecting which baked bones are exposed as colliders (for debugging / isolation).
	 * All = use every baked bone (existing behaviour); Include/Exclude apply the Bone Filter list below.
	 * Baking is left untouched - toggles live in the details panel with no re-bake.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	ERopeSDFBoneFilterMode BoneFilterMode = ERopeSDFBoneFilterMode::All;

	/**
	 * Bones targeted in Include/Exclude mode (ignored when mode is All).
	 * The dropdown lists only bones actually baked into the SDFData asset, not the whole skeleton.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision",
		meta = (EditCondition = "BoneFilterMode != ERopeSDFBoneFilterMode::All", GetOptions = "GetBakedBoneNames"))
	TArray<FName> BoneFilter;

protected:
	//~ URopeSkeletalColliderProvider
	virtual void RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt) override;
	virtual void AppendColliderPointers(FRopeColliderGatherContext& Gather) override;
	virtual bool HasColliderData() const override;

private:
	/** The candidates offered in the Bone Filter dropdown: the bone names actually baked into
	 *  SDFData. */
	UFUNCTION()
	TArray<FName> GetBakedBoneNames() const;

	// The backing storage, rebuilt once per frame. The pointers handed out are valid for that frame.
	TArray<FRopeSDFCollider> Colliders;

	// The previous frame's bone-to-world transform per bone, used to derive surface velocity, which
	// produces drag; building a collider pairs the current transform with the previous one.
	TMap<FName, FTransform> PrevBoneToWorld;
};
