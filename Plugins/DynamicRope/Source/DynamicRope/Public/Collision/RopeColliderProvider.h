// Copyright Epic Games, Inc. All Rights Reserved.
//
// Components and objects that supply colliders to the rope solver each frame. A skeletal provider
// builds a per-bone collider, capsule or SDF, for each candidate bone.
//
// A provider already knows which region, that is which rope, each collider belongs to at the moment
// it gathers, so it returns that mapping alongside a flat deduplicated pool. Returning the pool
// alone would throw the mapping away and force the subsystem to rebuild it by re-testing the whole
// pool against every rope's bounds, which costs O(ropes x colliders) every frame.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
// RopeColliderGather::MapCollidersToRegionsByBounds uses GetWorldBounds.
#include "Collision/RopeCollider.h"
#include "RopeColliderProvider.generated.h"

class AActor;

/**
 * The input and output bundle for one frame's collider gather. The subsystem, in
 * BuildFrameColliders, creates one per provider and passes it in; the provider fills in the pool
 * (Colliders) and, where it can, the per-region index mapping.
 */
struct FRopeColliderGatherContext
{
	/**
	 * Input: the active physics and aim regions. With N ropes, the layout is [0, N) for the physics
	 * regions and [N, 2N) for the aim regions of the same ropes in the same order. The aim region of a
	 * rope that is not aiming, and any slot with no region at all, holds its place as an invalid box,
	 * which providers skip.
	 * A bounds-aware provider uses this list to exclude the empty space between widely separated ropes
	 * from its scan.
	 */
	TArrayView<const FBox> RopeRegions;

	/** The number of physics regions, N, at the front of RopeRegions. The N entries after them are the
	 *  aim regions in the same order. */
	int32 NumPhysicsRegions = 0;

	/**
	 * Optional input: the order to process regions in, scanning the listed region indices first. It
	 * exists for providers with a global extraction limit, such as the static body provider: consuming
	 * that limit first come, first served would let clutter around an idle rope early in the list eat
	 * the budget and starve an active rope of collisions. The subsystem therefore puts active physics
	 * regions, meaning ropes in a phase that is in use and not asleep, first, and places every aim
	 * region after every physics region.
	 * It only changes the order; the region indices themselves are unchanged, so RegionColliderIndices
	 * is unaffected. When empty, regions are processed in index order from 0 to N-1. Providers with no
	 * limit may ignore it.
	 */
	TArrayView<const int32> RegionGatherOrder;

	/**
	 * Output: the flat deduplicated pool. Colliders and regions are many-to-many, since regions
	 * overlap, so the provider deduplicates per component or bone and adds each collider once, while
	 * membership in several regions is expressed through the mapping below. The colliders pointed to
	 * belong to the provider's own storage and must stay valid until this frame's solve finishes.
	 */
	TArray<IRopeCollider*> Colliders;

	/**
	 * Recommended output: for each region r, the indices into Colliders of the colliders it contains.
	 * Consumed only while bHasRegionMapping is set, in which case its length must equal
	 * RopeRegions.Num(); the subsystem downgrades to the fallback on a mismatch.
	 */
	TArray<TArray<int32>> RegionColliderIndices;

	/**
	 * When true the subsystem completes the per-rope assignment straight from RegionColliderIndices,
	 * with no bounds re-testing. When false, the default, the subsystem re-culls per rope using
	 * collider bounds, which is the legitimate fallback for a provider that cannot produce a mapping.
	 */
	bool bHasRegionMapping = false;

	/**
	 * Optional output: the source actor of each collider, parallel to Colliders, meaning the owner of
	 * the component that holds that shape. Filling it in makes the subsystem's owner exclusion a
	 * per-collider, that is per-body, decision instead of a per-provider one.
	 * It is needed by any provider that serves world geometry while also sweeping up shapes attached to
	 * the rope's own actor, such as the static body provider: exempting it per provider would turn
	 * those shapes into push-out colliders against their own rope. See the comment on
	 * ProvidesWorldStaticColliders.
	 * A length that does not match Colliders is not trusted and falls back to the per-provider
	 * decision. Leave entries with an unknown source as nullptr. Its lifetime matches the collider
	 * pointers, that is this frame.
	 */
	TArray<const AActor*> ColliderSourceActors;
};

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
	 * Gathers this frame's colliders, performing the broad phase here. The subsystem calls it once per
	 * frame with the physics and aim region lists in Gather.RopeRegions; the published layout contract
	 * is described on Gather.NumPhysicsRegions.
	 *  - A provider that genuinely uses regions, such as the static body provider, deduplicates its
	 *    per-region overlap results into the pool and returns which region caught which collider in
	 *    RegionColliderIndices.
	 *  - A provider that ignores regions, such as a skeleton provider, can build every collider into
	 *    the pool and then produce the mapping with
	 *    RopeColliderGather::MapCollidersToRegionsByBounds, whose union pre-rejection drops a distant
	 *    rope entirely in constant time.
	 *  - A provider that cannot produce a mapping simply leaves bHasRegionMapping false and the
	 *    subsystem falls back to re-culling by bounds.
	 */
	virtual void GatherColliders(FRopeColliderGatherContext& Gather) = 0;

	/**
	 * Whether this provider supplies static world geometry colliders, as URopeStaticBodyProvider does.
	 * When true it is exempt from the subsystem's per-rope exclusion of the rope's own owner provider,
	 * because static world geometry can never be "the thrower's own body" and world collision must not
	 * disappear silently merely because the provider happens to be attached to the rope's actor.
	 *
	 * Note that the exemption is per provider and is therefore too broad on its own. Such a provider
	 * sweeps the world and can pick up shapes attached to the rope's owner, such as a tether proxy, a
	 * tip mesh or a held weapon, and those become push-out colliders against their own rope. It must
	 * therefore also return per-collider sources in Gather.ColliderSourceActors so the subsystem can
	 * filter out the owner per body, leaving world geometry, which has a different source actor,
	 * untouched.
	 */
	virtual bool ProvidesWorldStaticColliders() const { return false; }
};

namespace RopeColliderGather
{
	/**
	 * Whether one collider belongs to the owner being excluded. On paths exempt from the per-provider
	 * owner exclusion, such as a static world provider, this is what keeps world geometry while
	 * dropping the shapes belonging to the rope's own actor.
	 *
	 * Fallback contract, where both cases return false and therefore exclude nothing:
	 *  - OwnerToExclude is nullptr, meaning the rope opted into its owner's colliders through
	 *    bIncludeOwnerColliders.
	 *  - The index is absent from SourceActors, meaning the provider supplied no sources or the length
	 *    did not match, which leaves the decision to the per-provider rule.
	 * Incomplete source information therefore never silently over-excludes: it does not fail in the
	 * direction of collisions disappearing.
	 */
	inline bool IsExcludedOwnerBody(TConstArrayView<const AActor*> SourceActors, int32 Index,
		const AActor* OwnerToExclude)
	{
		return OwnerToExclude != nullptr
			&& SourceActors.IsValidIndex(Index)
			&& SourceActors[Index] == OwnerToExclude;
	}

	/**
	 * Shared mapping helper for skeleton-style providers, which build every collider and then assign
	 * them. It assigns the pool range [StartIndex, Colliders.Num()), that is the colliders this
	 * provider added during this call, to the regions.
	 * A group union bounds test rejects first, so a distant region costs one comparison per region,
	 * which is linear in regions per mesh, and only the nearby ropes that pass the union are assigned
	 * precisely by per-collider bounds, using the same test the subsystem's re-cull would.
	 */
	inline void MapCollidersToRegionsByBounds(FRopeColliderGatherContext& Gather, int32 StartIndex)
	{
		Gather.bHasRegionMapping = true;
		Gather.RegionColliderIndices.SetNum(Gather.RopeRegions.Num());
		const int32 EndIndex = Gather.Colliders.Num();
		if (StartIndex >= EndIndex)
		{
			return;
		}

		// Cache the collider bounds once, and build the group union used for pre-rejection.
		TArray<FBox, TInlineAllocator<64>> Bounds;
		Bounds.Reserve(EndIndex - StartIndex);
		FBox GroupBounds(ForceInit);
		for (int32 i = StartIndex; i < EndIndex; ++i)
		{
			const FBox Box = Gather.Colliders[i] ? Gather.Colliders[i]->GetWorldBounds() : FBox(ForceInit);
			Bounds.Add(Box);
			if (Box.IsValid)
			{
				GroupBounds += Box;
			}
		}
		if (!GroupBounds.IsValid)
		{
			return;
		}

		for (int32 r = 0; r < Gather.RopeRegions.Num(); ++r)
		{
			const FBox& Region = Gather.RopeRegions[r];
			if (!Region.IsValid || !GroupBounds.Intersect(Region))
			{
				continue;
			}
			TArray<int32>& Out = Gather.RegionColliderIndices[r];
			for (int32 i = StartIndex; i < EndIndex; ++i)
			{
				const FBox& Box = Bounds[i - StartIndex];
				if (Box.IsValid && Box.Intersect(Region))
				{
					Out.Add(i);
				}
			}
		}
	}
}
