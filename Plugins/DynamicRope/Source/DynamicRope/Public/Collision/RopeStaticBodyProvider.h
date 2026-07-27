// Copyright Epic Games, Inc. All Rights Reserved.
//
// The IRopeColliderProvider for static world geometry. Each frame it scans the rope-active area,
// which is the union AABB of every rope, with an ECC_WorldStatic overlap and extracts the simple
// collision of the nearby static bodies, meaning their spheres, capsules and boxes, as analytic
// colliders. The point is to handle box corners exactly through analytic queries, where the global
// distance field rounds them off; that field remains the far-field fallback for landscapes and huge
// meshes with no simple collision.
//
// Placement: one per world is enough, on any persistent actor, whether the game mode, a level actor
// or a rope's own actor. Because ProvidesWorldStaticColliders() is true, attaching it to a rope's
// actor does not trip the owner exclusion.
// Convex simple collision is extracted as a plane-set collider, FRopeConvexCollider, with anything
// over the plane limit falling back to an OBB, and instanced static meshes are supported per region
// overlap.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
#include "Collision/RopeStaticCollider.h"
#include "RopeStaticBodyProvider.generated.h"

class UBodySetup;
class UPrimitiveComponent;
class UInstancedStaticMeshComponent;

// Not exposed in the editor, so it appears in neither the Add Component list nor Blueprint
// subclassing. This provider is a controller-owned subobject that ARopeController creates through
// CreateDefaultSubobject, and the subsystem spawns exactly one per world automatically, so it is
// never placed by hand. Tune it through IgnoredComponents on the controller's details panel, or that
// of a subclass. NotBlueprintable forbids Blueprint subclasses; extend by subclassing ARopeController
// instead.
UCLASS(NotBlueprintable)
class DYNAMICROPE_API URopeStaticBodyProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeStaticBodyProvider();

	//~ UActorComponent. Registers with and unregisters from the RopeSimSubsystem's central registry,
	//~ which gathers once per frame.
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// The collider budget and the convex plane limit are managed in one place, in the project settings
	// on UDynamicRopeSettings, rather than on the component. The duplicate guard already enforces one
	// provider per world, so per-component numeric budgets add nothing over the global settings. Only
	// values that are genuinely per provider, such as IgnoredComponents below, stay on the component.
	// The provider reads the settings directly in BuildColliders.

	/** Components excluded from gathering, such as geometry the rope is meant to pass through. It is a
	 *  per-provider value, so it lives on the component. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TArray<TObjectPtr<UPrimitiveComponent>> IgnoredComponents;

	//~ IRopeColliderProvider
	virtual void GatherColliders(FRopeColliderGatherContext& Gather) override;
	virtual bool ProvidesWorldStaticColliders() const override { return true; }

private:
	/** The backing storage, rebuilt once per frame. The pointers handed out are valid for that frame. */
	TArray<FRopeBoxCollider> Boxes;
	TArray<FRopeStaticCapsuleCollider> Capsules;

	/** Convex simple collision, and the six-plane routing for shear boxes. */
	TArray<FRopeConvexCollider> Convexes;

	/**
	 * A per-frame cache of each collider's world AABB, parallel to Boxes, Capsules and Convexes.
	 * RecordExtractedGroup computes it once during extraction, so the region mapping in GatherColliders
	 * uses the cache instead of recomputing GetWorldBounds for every collider and region pair.
	 */
	TArray<FBox> BoxWorldBounds;
	TArray<FBox> CapWorldBounds;
	TArray<FBox> CvxWorldBounds;

	/**
	 * A per-frame cache of each collider's source actor, parallel to Boxes, Capsules and Convexes.
	 * RecordExtractedGroup fills it in during extraction and GatherColliders passes it through to
	 * Gather.ColliderSourceActors in the same order as the pool. The subsystem uses it to decide, per
	 * body, whether a collider belongs to the rope's own actor: this provider sweeps the world and
	 * therefore also picks up shapes attached to that actor, such as a tether proxy, and exempting it at
	 * the provider level alone would not stop those from pushing their own rope.
	 */
	TArray<const AActor*> BoxSourceActors;
	TArray<const AActor*> CapSourceActors;
	TArray<const AActor*> CvxSourceActors;

	/**
	 * An extraction group: the per-type index ranges added by one component, or one instanced mesh
	 * call, together with their union bounds. It preserves, per group, what the gather's region overlap
	 * already knows about which ropes a body is near, which replaces the subsystem's per-rope re-cull
	 * over the whole pool with a group union pre-rejection followed by per-collider assignment for the
	 * groups that hit; that is the mapping stage at the end of GatherColliders. A group overlapping
	 * several regions is assigned to all of them.
	 */
	struct FExtractedGroup
	{
		int32 BoxStart = 0, BoxCount = 0;
		int32 CapStart = 0, CapCount = 0;
		int32 CvxStart = 0, CvxCount = 0;
		FBox Bounds = FBox(ForceInit);
	};
	TArray<FExtractedGroup> Groups;

	/**
	 * Records everything added to Boxes, Capsules and Convexes since the given snapshot indices as a
	 * group, together with its union bounds. It does nothing when nothing was added. SourceActor is the
	 * owner of the component holding those shapes, used for the per-body owner exclusion, or nullptr
	 * when unknown.
	 */
	void RecordExtractedGroup(int32 BoxStart, int32 CapStart, int32 CvxStart, const AActor* SourceActor);

	/**
	 * The frame counter value at which colliders were last built, which deduplicates the rebuild when
	 * several ropes call in during the same frame. Unlike the skeletal providers, this one genuinely
	 * uses the rope region list, which the subsystem passes identically for every call in a frame.
	 */
	uint64 BuiltFrame = static_cast<uint64>(-1);

	/**
	 * For the surface velocity of dynamic bodies: the previous frame's world transform per component,
	 * refreshed every frame, from which the current minus previous difference produces the surface
	 * velocity and the substep continuous collision. The keys are weak, so entries for destroyed
	 * components disappear naturally on the next refresh.
	 */
	TMap<TWeakObjectPtr<UPrimitiveComponent>, FTransform> PrevCompXforms;

	/**
	 * Overlaps each physics and aim region and extracts the aggregate geometry of the nearby static
	 * bodies into Boxes, Capsules and Convexes. Overlap between regions is deduplicated per component
	 * or instance, so each is extracted once per frame, which avoids both the waste of a union AABB
	 * spanning empty space and contention over the budget.
	 * Regions are processed in Gather.RegionGatherOrder, active ropes first, so that on a frame where
	 * the global limit binds, whatever misses the scan is an idle or sleeping rope.
	 */
	void BuildColliders(const FRopeColliderGatherContext& Gather);

	/**
	 * Adds one component's UBodySetup simple collision as world-space colliders. Returns false once the
	 * budget is exhausted.
	 * The budget and the convex plane limit are read from the project settings by the caller,
	 * BuildColliders, and passed in, which keeps a single source.
	 * PrevCompTM and InvDeltaTime supply the surface velocity of a dynamic body; for a static one, pass
	 * PrevCompTM equal to CompTM and an InvDeltaTime of 0, giving zero surface velocity. The collider
	 * shapes are also built at the previous transform to populate the previous state.
	 */
	bool AppendBodyColliders(const UBodySetup& Setup, const FTransform& CompTM, const FTransform& PrevCompTM,
		float InvDeltaTime, int32 MaxColliders, int32 MaxConvexPlanes);

	/**
	 * For instanced static meshes: enumerates only the instances overlapping the region and extracts
	 * the shared body setup at each instance's world transform, since every instance shares the same
	 * mesh collision. SeenIndices deduplicates per instance index for an instanced mesh spanning several
	 * regions, skipping indices already extracted; deduplicating per component would be wrong here,
	 * because it would miss different instances in different regions.
	 * Returns false once the budget is exhausted. This is what supports foliage and modular assets.
	 */
	bool AppendInstancedBodyColliders(UInstancedStaticMeshComponent& ISM, const FBox& Region,
		TSet<int32>& SeenIndices, int32 MaxColliders, int32 MaxConvexPlanes);
};
