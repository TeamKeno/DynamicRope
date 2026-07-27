// Copyright Epic Games, Inc. All Rights Reserved.
//
// The minimal skeletal collider provider and the first collider source: it builds a capsule per bone
// from the skeletal mesh each frame, either along the bone-to-parent segment or from the physics
// asset's shapes. It sits behind the same base as the per-bone SDF provider
// (URopeSDFProvider), so either can be chosen per target; being analytic, capsules need no bake and
// are cheap. Registration, mesh resolution, per-frame deduplication and the gather pipeline all
// belong to the base, leaving only the capsule build (RebuildColliders) and the pointer append
// (AppendColliderPointers) here.

#pragma once

#include "CoreMinimal.h"
#include "Collision/RopeSkeletalColliderProvider.h"
#include "Collision/RopeCollider.h"
#include "RopeBoneCapsuleProvider.generated.h"

class USkeletalMeshComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeBoneCapsuleProvider : public URopeSkeletalColliderProvider
{
	GENERATED_BODY()

public:
	/**
	 * The bones to expose as capsules. Each capsule spans from that bone to its parent, with a radius
	 * of CapsuleRadius.
	 * Leave it empty for automatic mode, which builds capsules from the mesh's physics asset bodies,
	 * using the real per-bone dimensions of their capsule, sphere and box shapes, where a box is
	 * approximated by a capsule along its longest axis. With no physics asset, or none of its shapes
	 * usable, as when it holds only convexes, it falls back to every bone-to-parent segment in the
	 * reference skeleton, excluding those shorter than AutoMinBoneLength. That fallback can include
	 * spurious segments from IK and twist bones, so a physics asset is recommended for characters.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TArray<FName> Bones;

	/** The radius of the capsule around each bone segment (cm). Used only on the explicit Bones path
	 *  and the skeleton fallback; a physics asset supplies its shapes' own radii. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision", meta = (ClampMin = "0.0", Units = "cm"))
	float CapsuleRadius = 8.0f;

	/** On the skeleton fallback of automatic mode, bone segments shorter than this (cm) are excluded,
	 *  which cuts out finger and twist bone noise. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision|Tuning", meta = (ClampMin = "0.0", Units = "cm"))
	float AutoMinBoneLength = 5.0f;

protected:
	//~ URopeSkeletalColliderProvider
	virtual void RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt) override;
	virtual void AppendColliderPointers(FRopeColliderGatherContext& Gather) override;

private:
	/** The backing storage, rebuilt once per frame. The pointers handed out are valid for that frame. */
	TArray<FCapsuleCollider> Capsules;

	/**
	 * The previous frame's endpoints, A and B, per capsule, aligned by build order index. They are the
	 * previous-state source for surface velocity, which produces drag, and for relative continuous
	 * collision, and correspond to PrevBoneToWorld on the SDF provider. The build order is stable
	 * between frames because its source, whether the Bones list, the physics asset or the skeleton, is
	 * the same each time, so indices line up. If the count changes, meaning the configuration changed,
	 * this is reset and that frame is treated as static with zero velocity.
	 */
	TArray<TPair<FVector, FVector>> PrevEndpoints;

	/**
	 * Builds this frame's capsule list, that is the endpoints, radii and bones, into Capsules. It tries
	 * the explicit Bones list first, then the physics asset, then the skeleton fallback. The previous
	 * endpoints and the reciprocal delta time are joined on by RebuildColliders using the aligned
	 * indices.
	 */
	void BuildCapsules(USkeletalMeshComponent* Mesh);
};
