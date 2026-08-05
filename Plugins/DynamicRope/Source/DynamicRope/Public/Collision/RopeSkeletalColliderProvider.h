// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The abstract base for providers that supply per-bone colliders from a skeletal mesh each frame. It
// gathers the wiring shared by the capsule provider (URopeBoneCapsuleProvider) and the SDF provider
// (URopeSDFProvider): registering with and unregistering from the subsystem, resolving the mesh,
// deduplicating the build to once per frame, and the gather pipeline that appends colliders and maps
// them to regions. A subclass fills in only its own storage, whether an array of capsule or SDF
// colliders, and its build source, through the two hooks RebuildColliders and AppendColliderPointers.
//
// Scope: skeletal targets only. World and static providers, meaning the global distance field and
// static body providers, share none of the mesh resolution, bone or previous-transform machinery and
// have a different ProvidesWorldStaticColliders contract, so they do not derive from this base and
// implement IRopeColliderProvider directly.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
#include "RopeSkeletalColliderProvider.generated.h"

class USkeletalMeshComponent;

UCLASS(Abstract)
class DYNAMICROPE_API URopeSkeletalColliderProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeSkeletalColliderProvider();

	//~ UActorComponent. Registers with and unregisters from the RopeSimSubsystem's central registry,
	//~ which gathers once per frame.
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** The mesh whose bones become colliders. Leave it null to resolve it from the owner
	 *  automatically. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<USkeletalMeshComponent> SkeletalMesh = nullptr;

	//~ IRopeColliderProvider. The shared gather pipeline: per-frame deduplication, the subclass build,
	//~ and the region mapping.
	virtual void GatherColliders(FRopeColliderGatherContext& Gather) override;

protected:
	/** Resolves and caches SkeletalMesh, falling back to the owner's first USkeletalMeshComponent when
	 *  it is unset. */
	USkeletalMeshComponent* ResolveMesh();

	/**
	 * Rebuilds this frame's colliders into the subclass's storage. Called once per frame, deduplicated
	 * through BuiltFrame. The subclass owns resetting its storage and refreshing the previous state
	 * used for surface velocity.
	 * @param Mesh   The resolved skeletal mesh, guaranteed non-null.
	 * @param InvDt  The reciprocal of the frame delta, used to derive surface velocity. It is 0 on the
	 *               first frame and on paused frames, which gives zero velocity.
	 *
	 * A UObject cannot have a C++ pure virtual, since an abstract class cannot be instantiated and the
	 * class default object must be, so this uses the engine's PURE_VIRTUAL idiom: a body is provided
	 * that is fatal if it is ever actually called. Real instances are always subclasses, so it never
	 * fires.
	 */
	virtual void RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt)
		PURE_VIRTUAL(URopeSkeletalColliderProvider::RebuildColliders, );

	/** Appends the collider pointers from the subclass's storage to Gather.Colliders, including the
	 *  reserve. Called on every central gather pass. */
	virtual void AppendColliderPointers(FRopeColliderGatherContext& Gather)
		PURE_VIRTUAL(URopeSkeletalColliderProvider::AppendColliderPointers, );

	/**
	 * Whether there is anything to build from besides the mesh. It defaults to true, since a capsule
	 * provider needs only the mesh. The SDF provider gates on whether it has SDF data, so that without
	 * data it never enters the frame build and simply supplies no colliders.
	 */
	virtual bool HasColliderData() const { return true; }

private:
	/** The frame counter value at which colliders were last built, which deduplicates the rebuild when
	 *  several ropes call in during the same frame. */
	uint64 BuiltFrame = static_cast<uint64>(-1);

	/**
	 * Whether a build actually happened this frame. When no rope region is anywhere near the mesh this
	 * stays false and supplying colliders is skipped entirely, which prevents appending stale ones. The
	 * first call of the frame decides it and later calls in the same frame reuse the answer.
	 */
	bool bBuiltThisFrame = false;
};
