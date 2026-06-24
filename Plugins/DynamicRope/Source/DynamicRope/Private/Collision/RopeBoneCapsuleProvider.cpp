// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeBoneCapsuleProvider.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "DrawDebugHelpers.h"

URopeBoneCapsuleProvider::URopeBoneCapsuleProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

USkeletalMeshComponent* URopeBoneCapsuleProvider::ResolveMesh()
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

void URopeBoneCapsuleProvider::GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh)
	{
		return;
	}

	// Include every listed bone; the precise per-node capsule narrow-phase decides actual contact.
	// (No AABB broad-phase cull here: with a handful of bones it would only risk false negatives.)
	Capsules.Reset();
	for (const FName& Bone : Bones)
	{
		if (Bone.IsNone())
		{
			continue;
		}
		const FName    Parent = Mesh->GetParentBone(Bone);
		const FVector  P0 = Mesh->GetSocketTransform(Bone).GetLocation();
		const FVector  P1 = Parent.IsNone() ? P0 : Mesh->GetSocketTransform(Parent).GetLocation();
		Capsules.Add(FCapsuleCollider(P0, P1, CapsuleRadius, Bone, Mesh));

#if ENABLE_DRAW_DEBUG
		if (bDrawDebug)
		{
			if (UWorld* World = GetWorld())
			{
				const FVector Center = (P0 + P1) * 0.5f;
				const float   HalfHeight = static_cast<float>((P1 - P0).Size()) * 0.5f + CapsuleRadius;
				const FQuat   Rot = FRotationMatrix::MakeFromZ(P1 - P0).ToQuat();
				DrawDebugCapsule(World, Center, HalfHeight, CapsuleRadius, Rot, FColor::Green, false, -1.0f, 0, 0.5f);
			}
		}
#endif
	}

	// Hand out pointers only after Capsules is fully built (no reallocation past this point).
	OutColliders.Reserve(OutColliders.Num() + Capsules.Num());
	for (FCapsuleCollider& Cap : Capsules)
	{
		OutColliders.Add(&Cap);
	}
}
