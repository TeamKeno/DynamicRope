// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeBoneCapsuleProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Debug/RopeDebugDraw.h" // 디버그 드로우 중앙화(RopeDebug::DrawCapsule)
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"

URopeBoneCapsuleProvider::URopeBoneCapsuleProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeBoneCapsuleProvider::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->RegisterColliderProvider(this);
	}
}

void URopeBoneCapsuleProvider::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
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

			// 디버그 드로우 중앙화: bDrawDebug(per-instance) 또는 r.DynamicRope.Debug(.Colliders)로 게이트.
			RopeDebug::DrawCapsule(GetWorld(), P0, P1, CapsuleRadius, bDrawDebug);
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
