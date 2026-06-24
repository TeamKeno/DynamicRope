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

	// 나열된 모든 본을 포함한다. 실제 contact 여부는 정밀한 per-node capsule narrow-phase가 결정한다.
	// (여기서는 AABB broad-phase 컬링을 하지 않는다. 본이 몇 개뿐이라 false negative 위험만 키울 뿐이다.)
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

	// Capsules가 완전히 구성된 뒤에만 포인터를 넘긴다(이 지점 이후로는 재할당 없음).
	OutColliders.Reserve(OutColliders.Num() + Capsules.Num());
	for (FCapsuleCollider& Cap : Capsules)
	{
		OutColliders.Add(&Cap);
	}
}
