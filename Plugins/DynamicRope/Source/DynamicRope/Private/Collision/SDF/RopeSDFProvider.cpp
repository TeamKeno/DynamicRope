// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFProvider.h"
#include "Collision/SDF/RopeSDFData.h"
#include "DynamicRopeLog.h"
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

void URopeSDFProvider::GatherColliders(const FBox& /*RopeBounds*/, TArray<IRopeCollider*>& OutColliders)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || !SDFData)
	{
		UE_LOG(LogRopeCollision, Verbose, TEXT("SDFProvider on %s: %s missing — no colliders."),
			*GetNameSafe(GetOwner()), !Mesh ? TEXT("skeletal mesh") : TEXT("SDFData asset"));
		return;
	}

	// 프레임당 1회만 빌드(디둡). per-rope 컬링은 solver의 collider AABB broad-phase가 담당하므로,
	// 여기서는 RopeBounds 컬 없이 베이크된 모든 볼륨을 빌드한다(collider 구성은 저렴 — 비싼 Query를 solver가 컬).
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		Colliders.Reset();
		for (const FRopeBoneSDFVolume& Volume : SDFData->BoneVolumes)
		{
			if (Volume.Bone.IsNone() || !Volume.IsBaked())
			{
				continue; // 미베이크/무효 볼륨은 건너뛴다.
			}

			const FTransform BoneToWorld = Mesh->GetSocketTransform(Volume.Bone);
			Colliders.Add(FRopeSDFCollider(&Volume, BoneToWorld, Volume.Bone, Mesh));

#if ENABLE_DRAW_DEBUG
			if (bDrawDebug)
			{
				if (UWorld* World = GetWorld())
				{
					const FBox WorldBounds = Volume.LocalBounds.TransformBy(BoneToWorld);
					DrawDebugBox(World, WorldBounds.GetCenter(), WorldBounds.GetExtent(), FColor::Green, false, -1.0f, 0, 0.5f);
				}
			}
#endif
		}

		UE_LOG(LogRopeCollision, VeryVerbose, TEXT("SDFProvider on %s: built %d collider(s) from %d baked volume(s)."),
			*GetNameSafe(GetOwner()), Colliders.Num(), SDFData->BoneVolumes.Num());
	}

	// 캐시된 collider 포인터를 넘긴다(해당 프레임 동안 유효).
	OutColliders.Reserve(OutColliders.Num() + Colliders.Num());
	for (FRopeSDFCollider& Collider : Colliders)
	{
		OutColliders.Add(&Collider);
	}
}
