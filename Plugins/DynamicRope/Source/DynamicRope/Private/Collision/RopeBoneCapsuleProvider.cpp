// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeBoneCapsuleProvider.h"
#include "DynamicRopeLog.h"
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

void URopeBoneCapsuleProvider::GatherColliders(const FBox& /*RopeBounds*/, TArray<IRopeCollider*>& OutColliders)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh)
	{
		UE_LOG(LogRopeCollision, Verbose, TEXT("CapsuleProvider on %s: no skeletal mesh resolved — no colliders."),
			*GetNameSafe(GetOwner()));
		return;
	}

	// 프레임당 1회만 빌드(디둡): 같은 메시를 잡는 여러 로프가 호출해도 capsule을 재구성하지 않는다.
	// per-rope 컬링은 solver의 collider AABB broad-phase가 담당하므로 RopeBounds는 여기서 쓰지 않는다.
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
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

		UE_LOG(LogRopeCollision, VeryVerbose, TEXT("CapsuleProvider on %s: built %d capsule(s) from %d bone(s)."),
			*GetNameSafe(GetOwner()), Capsules.Num(), Bones.Num());
	}

	// 캐시된 capsule 포인터를 넘긴다(해당 프레임 동안 유효).
	OutColliders.Reserve(OutColliders.Num() + Capsules.Num());
	for (FCapsuleCollider& Cap : Capsules)
	{
		OutColliders.Add(&Cap);
	}
}
