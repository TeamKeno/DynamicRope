// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeSkeletalColliderProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

namespace
{
	// Whether any of the rope regions, meaning the broad-phase query bounds, overlaps the mesh's current world bounds.
	// If no rope is nearby, the rebuild of every bone collider is skipped. It is conservative: unknown bounds are
	// treated as true and the build proceeds.
	// The region already includes the contact and prediction margins and the mesh bounds already include the skinning
	// range, so an overlap means contact is possible and the build is required.
	bool AnyRopeRegionNearMesh(const USkeletalMeshComponent& Mesh, TArrayView<const FBox> Regions)
	{
		const FBox MeshBox = Mesh.Bounds.GetBox();
		if (!MeshBox.IsValid)
		{
			return true;
		}
		for (const FBox& Region : Regions)
		{
			if (Region.IsValid && Region.Intersect(MeshBox))
			{
				return true;
			}
		}
		return false;
	}
}

URopeSkeletalColliderProvider::URopeSkeletalColliderProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeSkeletalColliderProvider::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->RegisterColliderProvider(this);
	}
}

void URopeSkeletalColliderProvider::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
}

USkeletalMeshComponent* URopeSkeletalColliderProvider::ResolveMesh()
{
	if (!SkeletalMesh)
	{
		if (AActor* Owner = GetOwner())
		{
			SkeletalMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}
	}
	return SkeletalMesh;
}

void URopeSkeletalColliderProvider::GatherColliders(FRopeColliderGatherContext& Gather)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh)
	{
		UE_LOG(LogRopeCollision, Verbose, TEXT("%s on %s: no skeletal mesh resolved — no colliders."),
			*GetClass()->GetName(), *GetNameSafe(GetOwner()));
		return;
	}
	if (!HasColliderData())
	{
		// There is no data, as when no SDF data is assigned, so the subclass logs the reason and this is a no-op.
		return;
	}

	// Built once per frame, deduplicated, so that several ropes catching the same mesh do not rebuild the colliders.
	// The per-region assignment is produced by MapCollidersToRegionsByBounds below; the build itself is
	// region-independent and covers every bone.
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
	// The proximity gate: if no rope region is near this mesh, the rebuild of every bone collider is skipped. Bones
	// move with the animation every frame, which makes a cache pointless, so with no rope nearby they are not built at
	// all and neither the build nor the append happens.
		bBuiltThisFrame = AnyRopeRegionNearMesh(*Mesh, Gather.RopeRegions);
		if (bBuiltThisFrame)
		{
			// The frame delta used to derive the surface velocity, meaning the drag. The subclass produces each collider's surface velocity as the current minus the previous transform over the delta.
			const float FrameDt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
			const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;
			RebuildColliders(Mesh, InvDt);
		}
	}
	if (!bBuiltThisFrame)
	{
		// No rope was nearby this frame, so no colliders are supplied, which prevents appending stale ones.
		return;
	}

	// Passes the cached collider pointers, which stay valid for the frame. The region mapping rejects early on the
	// mesh, being the union of its colliders, and assigns per-collider bounds for the ropes that pass. A distant rope
	// costs one comparison per mesh, which is what replaces the subsystem re-culling the whole pool per rope.
	const int32 StartIndex = Gather.Colliders.Num();
	AppendColliderPointers(Gather);
	RopeColliderGather::MapCollidersToRegionsByBounds(Gather, StartIndex);
}
