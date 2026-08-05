// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Collision/RopeStaticBodyProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
// The collider budget and the convex plane limit, which have a single source.
#include "Settings/DynamicRopeSettings.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
// The skeletal exclusion in BuildColliders: skinned components are the bone providers' domain.
#include "Components/SkinnedMeshComponent.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodySetup.h"
// FKConvexElem::GetPlanes, for extracting world planes.
#include "PhysicsEngine/ConvexElem.h"
// Extracting simple collision into push-out colliders, through the helper shared with the wrap target
// provider. The free geometry maths functions moved there as well.
#include "Collision/RopeBodyColliderExtraction.h"

URopeStaticBodyProvider::URopeStaticBodyProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeStaticBodyProvider::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->RegisterColliderProvider(this);
	}
}

void URopeStaticBodyProvider::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
}

void URopeStaticBodyProvider::GatherColliders(FRopeColliderGatherContext& Gather)
{
	// Built once per frame, deduplicated. The physics and aim regions arrive together, and the overlap
	// results, together with the extraction groups, produced on the first central gather pass are shared by
	// every region that frame. A later pass in the same frame rebuilds only the mapping for its own region
	// and reuses the backing pool.
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		Boxes.Reset();
		Capsules.Reset();
		Convexes.Reset();
		Groups.Reset();
		BoxWorldBounds.Reset();
		CapWorldBounds.Reset();
		CvxWorldBounds.Reset();
		BoxSourceActors.Reset();
		CapSourceActors.Reset();
		CvxSourceActors.Reset();
		BuildColliders(Gather);
	}

	const int32 PoolBase = Gather.Colliders.Num();
	Gather.Colliders.Reserve(PoolBase + Boxes.Num() + Capsules.Num() + Convexes.Num());
	for (FRopeBoxCollider& Box : Boxes)
	{
		Gather.Colliders.Add(&Box);
	}
	for (FRopeStaticCapsuleCollider& Cap : Capsules)
	{
		Gather.Colliders.Add(&Cap);
	}
	for (FRopeConvexCollider& Convex : Convexes)
	{
		Gather.Colliders.Add(&Convex);
	}

	// The per-collider source actors are emitted in the same order as the pool, boxes then capsules then
	// convexes, which is what lets the subsystem decide the owner exclusion per body rather than per
	// provider. Anything appended earlier is not this provider's concern, so those slots are filled with
	// null to keep the array parallel with the pool.
	Gather.ColliderSourceActors.Reset();
	Gather.ColliderSourceActors.SetNumZeroed(PoolBase);
	Gather.ColliderSourceActors.Reserve(Gather.Colliders.Num());
	Gather.ColliderSourceActors.Append(BoxSourceActors);
	Gather.ColliderSourceActors.Append(CapSourceActors);
	Gather.ColliderSourceActors.Append(CvxSourceActors);

	// The region mapping: the extraction groups, which are the component-level results the overlap already
	// decided, are pre-rejected by their union bounds, and only the groups that hit are assigned precisely
	// by per-collider bounds. The pool order matches the appends above, boxes then capsules then convexes,
	// so a flat index is built from the per-type local index plus its offset.
	Gather.bHasRegionMapping = true;
	Gather.RegionColliderIndices.SetNum(Gather.RopeRegions.Num());
	const int32 CapFlatBase = PoolBase + Boxes.Num();
	const int32 CvxFlatBase = CapFlatBase + Capsules.Num();
	for (int32 r = 0; r < Gather.RopeRegions.Num(); ++r)
	{
		const FBox& Region = Gather.RopeRegions[r];
		if (!Region.IsValid)
		{
			continue;
		}
		TArray<int32>& Out = Gather.RegionColliderIndices[r];
		for (const FExtractedGroup& Group : Groups)
		{
			if (!Group.Bounds.IsValid || !Group.Bounds.Intersect(Region))
			{
				continue;
			}
			for (int32 i = Group.BoxStart; i < Group.BoxStart + Group.BoxCount; ++i)
			{
				if (BoxWorldBounds[i].Intersect(Region))
				{
					Out.Add(PoolBase + i);
				}
			}
			for (int32 i = Group.CapStart; i < Group.CapStart + Group.CapCount; ++i)
			{
				if (CapWorldBounds[i].Intersect(Region))
				{
					Out.Add(CapFlatBase + i);
				}
			}
			for (int32 i = Group.CvxStart; i < Group.CvxStart + Group.CvxCount; ++i)
			{
				if (CvxWorldBounds[i].Intersect(Region))
				{
					Out.Add(CvxFlatBase + i);
				}
			}
		}
	}
}

void URopeStaticBodyProvider::RecordExtractedGroup(int32 BoxStart, int32 CapStart, int32 CvxStart,
	const AActor* SourceActor)
{
	FExtractedGroup Group;
	Group.BoxStart = BoxStart;
	Group.BoxCount = Boxes.Num() - BoxStart;
	Group.CapStart = CapStart;
	Group.CapCount = Capsules.Num() - CapStart;
	Group.CvxStart = CvxStart;
	Group.CvxCount = Convexes.Num() - CvxStart;
	if (Group.BoxCount + Group.CapCount + Group.CvxCount <= 0)
	{
		return;
	}
	// The world AABB is computed once per collider and stored in both the group bounds and the cache, so
	// the region mapping in GatherColliders does not recompute it for every collider and region pair. The
	// cache is parallel to the collider arrays.
	// The source actors are filled in per collider in the same loop, in arrays parallel to the collider
	// arrays.
	BoxWorldBounds.SetNum(Boxes.Num());
	BoxSourceActors.SetNumZeroed(Boxes.Num());
	for (int32 i = Group.BoxStart; i < Group.BoxStart + Group.BoxCount; ++i)
	{
		const FBox WB = Boxes[i].GetWorldBounds();
		BoxWorldBounds[i] = WB;
		BoxSourceActors[i] = SourceActor;
		Group.Bounds += WB;
	}
	CapWorldBounds.SetNum(Capsules.Num());
	CapSourceActors.SetNumZeroed(Capsules.Num());
	for (int32 i = Group.CapStart; i < Group.CapStart + Group.CapCount; ++i)
	{
		const FBox WB = Capsules[i].GetWorldBounds();
		CapWorldBounds[i] = WB;
		CapSourceActors[i] = SourceActor;
		Group.Bounds += WB;
	}
	CvxWorldBounds.SetNum(Convexes.Num());
	CvxSourceActors.SetNumZeroed(Convexes.Num());
	for (int32 i = Group.CvxStart; i < Group.CvxStart + Group.CvxCount; ++i)
	{
		const FBox WB = Convexes[i].GetWorldBounds();
		CvxWorldBounds[i] = WB;
		CvxSourceActors[i] = SourceActor;
		Group.Bounds += WB;
	}
	Groups.Add(Group);
}

void URopeStaticBodyProvider::BuildColliders(const FRopeColliderGatherContext& Gather)
{
	TArrayView<const FBox> RopeRegions = Gather.RopeRegions;
	UWorld* World = GetWorld();
	if (!World || RopeRegions.Num() == 0)
	{
		// With no active ropes, meaning an empty region list, there is nothing to scan for.
		return;
	}

	// The collider budget, the convex plane limit and whether dynamic objects are included are all managed
	// in one place, in the project settings.
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	const int32 MaxColliders = Settings ? FMath::Max(1, Settings->StaticBodyMaxColliders) : 128;
	const int32 MaxConvexPlanes = Settings ? FMath::Max(4, Settings->StaticBodyMaxConvexPlanes) : 32;
	const bool  bIncludeDynamic = Settings ? Settings->bIncludeWorldDynamic : true;
	const bool  bIncludePhysics = Settings ? Settings->bIncludePhysicsBodies : false;

	FCollisionObjectQueryParams ObjParams(ECC_WorldStatic);
	if (bIncludeDynamic)
	{
		ObjParams.AddObjectTypesToQuery(ECC_WorldDynamic);
	}
	if (bIncludePhysics)
	{
		ObjParams.AddObjectTypesToQuery(ECC_PhysicsBody);
	}
	const FCollisionQueryParams QueryParams(FName(TEXT("RopeStaticBodyGather")), /*bInTraceComplex*/ false);

	// The frame delta used for surface velocity, which drives drag and continuous collision. It is derived
	// from the change in transform of the components processed this frame.
	const float FrameDt = World->GetDeltaSeconds();
	const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;
	// This frame's component transforms, which become the previous state next frame. Only the components
	// processed are stored, so destroyed or departed entries expire naturally.
	// It is frame-global: it accumulates across every region and is swapped exactly once after the loop,
	// which preserves the surface velocity continuity invariant.
	TMap<TWeakObjectPtr<UPrimitiveComponent>, FTransform> CurrCompXforms;

	// Frame-global deduplication state, outside the region loop. An ordinary component is extracted once,
	// per component.
	TSet<const UPrimitiveComponent*> Seen;
	// An instanced mesh can have different instances caught by different regions, so it is deduplicated per
	// instance index rather than per component.
	TMap<UInstancedStaticMeshComponent*, TSet<int32>> SeenInstances;

	// The broad phase overlaps each rope's active region, which excludes the empty space between widely
	// separated ropes from the scan and removes both the waste and the budget contention of one union AABB
	// over every rope. Duplicate results between regions are filtered by the deduplication state above, so
	// each is extracted once per frame.
	// Regions are processed in the order the subsystem sorted them, active ropes first, so that on a frame
	// where the global limit binds, whatever is pushed back and misses the scan is an idle or sleeping
	// rope. It only affects order; region indices are unchanged, so the extraction groups and the mapping
	// are unaffected. An empty or mismatched order list falls back to index order.
	const bool bUseGatherOrder = Gather.RegionGatherOrder.Num() == RopeRegions.Num();
	TArray<FOverlapResult> Overlaps;
	bool bBudgetClipped = false;
	for (int32 OrderSlot = 0; OrderSlot < RopeRegions.Num(); ++OrderSlot)
	{
		const int32 RegionIndex = bUseGatherOrder ? Gather.RegionGatherOrder[OrderSlot] : OrderSlot;
		if (!RopeRegions.IsValidIndex(RegionIndex))
		{
			continue;
		}
		const FBox& Region = RopeRegions[RegionIndex];
		if (bBudgetClipped)
		{
			break;
		}
		if (!Region.IsValid)
		{
			continue;
		}
		Overlaps.Reset();
		World->OverlapMultiByObjectType(Overlaps, Region.GetCenter(), FQuat::Identity, ObjParams,
			FCollisionShape::MakeBox(Region.GetExtent()), QueryParams);

		for (const FOverlapResult& Overlap : Overlaps)
		{
			UPrimitiveComponent* Prim = Overlap.Component.Get();
			if (!Prim)
			{
				continue;
			}
			// Skeletal meshes are never extracted here, whatever their object type — a ragdoll is
			// PhysicsBody, but a mesh set to WorldDynamic would slip in too. Their collision belongs to
			// the bone capsule and SDF providers, which place it per bone; this path would call
			// GetBodySetup, which on a skeletal component returns only the first bone's setup, and plant
			// those shapes at the component transform — a phantom collider at the feet, fighting the bone
			// providers around the very bodies ropes wrap and tether.
			if (Prim->IsA<USkinnedMeshComponent>())
			{
				continue;
			}
			// Excluding trigger and overlap volumes: an object-type overlap does not consider the other
			// side's channel response and catches query-only bodies, so a detection-only volume, such as the
			// invisible query-only box of a pressure plate, would become a solid collider and push the rope.
			// They are filtered out by the contract that a rope behaves like a physical object: only a shape
			// with physics collision enabled that blocks the physics body channel is entitled to push it.
			// Something invisible that still blocks physically, such as a blocking volume, passes, which is
			// consistent since it stops ragdolls and props and should stop a rope too; deliberate exceptions
			// are what IgnoredComponents is for.
			if (!Prim->IsPhysicsCollisionEnabled()
				|| Prim->GetCollisionResponseToChannel(ECC_PhysicsBody) != ECR_Block)
			{
				continue;
			}
			// Instanced static meshes are handled per instance, because one component places the shared mesh
			// collision at many instance transforms.
			// GetBodySetup returns the original local shapes, which know nothing of the instance transforms,
			// so each nearby instance has to be extracted at its own world transform; hierarchical instanced
			// meshes are caught through the same base class. They are not added to the per-component set and
			// are deduplicated per instance index instead, so instances caught by a different region are not
			// missed.
			// Per-instance previous transforms are not tracked, so instances are treated as a static
			// snapshot, with the previous transform equal to the current and no surface velocity.
			if (UInstancedStaticMeshComponent* ISM = Cast<UInstancedStaticMeshComponent>(Prim))
			{
				if (IgnoredComponents.Contains(Prim))
				{
					continue;
				}
				// Record the group of instance colliders this call added. Even a partial extraction cut short
				// by the budget records what it did add, so nothing is left out of the mapping.
				const int32 BoxStart = Boxes.Num(), CapStart = Capsules.Num(), CvxStart = Convexes.Num();
				const bool bWithinBudget = AppendInstancedBodyColliders(*ISM, Region, SeenInstances.FindOrAdd(ISM), MaxColliders, MaxConvexPlanes);
				RecordExtractedGroup(BoxStart, CapStart, CvxStart, ISM->GetOwner());
				if (!bWithinBudget)
				{
					bBudgetClipped = true;
					break;
				}
				continue;
			}
			// An ordinary component: the frame-global set extracts it once even across several regions, which
			// also deduplicates an overlap reported separately per body.
			if (Seen.Contains(Prim))
			{
				continue;
			}
			Seen.Add(Prim);
			if (IgnoredComponents.Contains(Prim))
			{
				continue;
			}
			const UBodySetup* Setup = Prim->GetBodySetup();
			if (!Setup)
			{
				continue;
			}
			// Look up this component's previous transform; without one it is treated as static this frame,
			// with no surface velocity.
			const FTransform CompTM = Prim->GetComponentTransform();
			const FTransform* PrevPtr = PrevCompXforms.Find(Prim);
			const FTransform PrevTM = PrevPtr ? *PrevPtr : CompTM;
			const float CompInvDt = PrevPtr ? InvDt : 0.0f;
			CurrCompXforms.Add(Prim, CompTM);

			{
				// Record the group of colliders this component added. Preserving the proximity information the
				// region overlap already produced is what allows the per-rope assignment with no re-cull in
				// the subsystem; a group overlapping several regions is assigned to each during mapping.
				const int32 BoxStart = Boxes.Num(), CapStart = Capsules.Num(), CvxStart = Convexes.Num();
				const bool bWithinBudget = AppendBodyColliders(*Setup, CompTM, PrevTM, CompInvDt, MaxColliders, MaxConvexPlanes);
				RecordExtractedGroup(BoxStart, CapStart, CvxStart, Prim->GetOwner());
				if (!bWithinBudget)
				{
					bBudgetClipped = true;
					break;
				}
			}
		}
	}

	// Swap in this frame's transforms as next frame's previous state, exactly once per frame.
	PrevCompXforms = MoveTemp(CurrCompXforms);

	if (bBudgetClipped)
	{
		UE_LOG(LogRopeCollision, Verbose,
			TEXT("StaticBodyProvider on %s: collider budget (%d) exceeded — remaining static bodies dropped this frame."),
			*GetNameSafe(GetOwner()), MaxColliders);
	}
	UE_LOG(LogRopeCollision, VeryVerbose, TEXT("StaticBodyProvider on %s: built %d box(es) + %d capsule(s) + %d convex from %d overlapped component(s)."),
		*GetNameSafe(GetOwner()), Boxes.Num(), Capsules.Num(), Convexes.Num(), Seen.Num());
}

bool URopeStaticBodyProvider::AppendBodyColliders(const UBodySetup& Setup, const FTransform& CompTM,
	const FTransform& PrevCompTM, float InvDeltaTime, int32 MaxColliders, int32 MaxConvexPlanes)
{
	// The extraction itself is delegated to the shared helper, which the wrap target provider also uses.
	// This provider's member arrays are passed as the outputs, and only the convex fallback log is written
	// in this provider's context, with its owner's name. The budget and previous-transform contracts are
	// preserved by the helper.
	return RopeBodyColliderExtraction::AppendBodyColliders(
		Setup, CompTM, PrevCompTM, InvDeltaTime, MaxColliders, MaxConvexPlanes,
		Boxes, Capsules, Convexes,
		[this](int32 NumPlanes)
		{
			UE_LOG(LogRopeCollision, Verbose,
				TEXT("StaticBodyProvider on %s: convex elem unusable (%d planes) — falling back to ElemBox OBB."),
				*GetNameSafe(GetOwner()), NumPlanes);
		});
}

bool URopeStaticBodyProvider::AppendInstancedBodyColliders(UInstancedStaticMeshComponent& ISM,
	const FBox& Region, TSet<int32>& SeenIndices, int32 MaxColliders, int32 MaxConvexPlanes)
{
	// The mesh collision shared by every instance, in local space. An instanced mesh component does not
	// override GetBodySetup and inherits the static mesh component's, which is the mesh's body setup.
	const UBodySetup* Setup = ISM.GetBodySetup();
	if (!Setup)
	{
		// No collision, so it is skipped without consuming any budget.
		return true;
	}

	// Enumerate only the instances overlapping the region, as a world-space box, which narrows dense
	// foliage down to what is nearby.
	// For an instanced mesh spanning several regions, indices already extracted are skipped, which prevents
	// duplicate colliders.
	const TArray<int32> Indices = ISM.GetInstancesOverlappingBox(Region, /*bBoxInWorldSpace=*/true);
	for (int32 Index : Indices)
	{
		bool bAlreadySeen = false;
		SeenIndices.Add(Index, &bAlreadySeen);
		if (bAlreadySeen)
		{
			continue;
		}
		FTransform InstanceTM;
		if (!ISM.GetInstanceTransform(Index, InstanceTM, /*bWorldSpace=*/true))
		{
			continue;
		}
		// The shared collision is placed at the instance's world transform, which is its local transform
		// composed with the component-to-world transform. That is exactly how an ordinary static mesh is
		// placed by its component transform, so AppendBodyColliders is reused unchanged.
		// Per-instance previous transforms are not tracked, so instances are treated as static, with the
		// previous transform equal to the current and no surface velocity.
		if (!AppendBodyColliders(*Setup, InstanceTM, InstanceTM, 0.0f, MaxColliders, MaxConvexPlanes))
		{
			// The budget is exhausted; instances share the same collider budget as every other body.
			return false;
		}
	}
	return true;
}
