// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFProvider.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "DrawDebugHelpers.h"

URopeSDFProvider::URopeSDFProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

USkeletalMeshComponent* URopeSDFProvider::ResolveMesh()
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

void URopeSDFProvider::GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || !SDFData)
	{
		return;
	}

	Colliders.Reset();
	for (const FRopeBoneSDFVolume& Volume : SDFData->BoneVolumes)
	{
		if (Volume.Bone.IsNone() || !Volume.IsBaked())
		{
			continue; // 미베이크/무효 볼륨은 건너뛴다.
		}

		const FTransform BoneToWorld = Mesh->GetSocketTransform(Volume.Bone);

		// 브로드페이즈(B2): 본 로컬 bounds를 월드로 변환해 rope bounds와 겹칠 때만 narrow-phase 대상.
		const FBox WorldBounds = Volume.LocalBounds.TransformBy(BoneToWorld);
		if (!WorldBounds.Intersect(RopeBounds))
		{
			continue;
		}

		Colliders.Add(FRopeSDFCollider(&Volume, BoneToWorld, Volume.Bone, Mesh));

#if ENABLE_DRAW_DEBUG
		if (bDrawDebug)
		{
			if (UWorld* World = GetWorld())
			{
				DrawDebugBox(World, WorldBounds.GetCenter(), WorldBounds.GetExtent(), FColor::Green, false, -1.0f, 0, 0.5f);
			}
		}
#endif
	}

	// Colliders가 완전히 구성된 뒤에만 포인터를 넘긴다(이 지점 이후로는 재할당 없음).
	OutColliders.Reserve(OutColliders.Num() + Colliders.Num());
	for (FRopeSDFCollider& Collider : Colliders)
	{
		OutColliders.Add(&Collider);
	}
}
