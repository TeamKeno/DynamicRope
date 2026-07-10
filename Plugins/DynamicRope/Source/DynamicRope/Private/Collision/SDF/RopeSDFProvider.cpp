// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFProvider.h"
#include "Collision/SDF/RopeSDFData.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"

URopeSDFProvider::URopeSDFProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeSDFProvider::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->RegisterColliderProvider(this);
	}
}

void URopeSDFProvider::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
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

TArray<FName> URopeSDFProvider::GetBakedBoneNames() const
{
	// BoneFilter 드롭다운 후보: SDFData에 실제 베이크된 본 이름만(스켈레톤 전체가 아님).
	TArray<FName> Names;
	if (SDFData)
	{
		for (const FRopeBoneSDFVolume& Volume : SDFData->BoneVolumes)
		{
			if (!Volume.Bone.IsNone() && Volume.IsBaked())
			{
				Names.AddUnique(Volume.Bone);
			}
		}
	}
	return Names;
}

void URopeSDFProvider::GatherColliders(FRopeColliderGatherContext& Gather)
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

		// 표면 속도(드래그) 산출용 프레임 dt. 본별 (현재-이전)/dt 로 콜라이더가 표면 속도를 만든다.
		const float FrameDt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
		const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;

		for (const FRopeBoneSDFVolume& Volume : SDFData->BoneVolumes)
		{
			if (Volume.Bone.IsNone() || !Volume.IsBaked())
			{
				continue; // 미베이크/무효 볼륨은 건너뛴다.
			}

			// 런타임 본 필터(베이크는 그대로, collider 노출만 가린다 — 디버깅 격리용).
			if (BoneFilterMode != ERopeSDFBoneFilterMode::All)
			{
				const bool bListed = BoneFilter.Contains(Volume.Bone);
				const bool bKeep = (BoneFilterMode == ERopeSDFBoneFilterMode::Include) ? bListed : !bListed;
				if (!bKeep)
				{
					continue;
				}
			}

			const FTransform BoneToWorld = Mesh->GetSocketTransform(Volume.Bone);
			// 이전 프레임 트랜스폼(없으면 현재 = 첫 프레임 속도 0). lookup 후 다음 프레임용으로 갱신.
			const FTransform* PrevPtr = PrevBoneToWorld.Find(Volume.Bone);
			const FTransform PrevXform = PrevPtr ? *PrevPtr : BoneToWorld;
			PrevBoneToWorld.Add(Volume.Bone, BoneToWorld);
			Colliders.Add(FRopeSDFCollider(&Volume, BoneToWorld, PrevXform, InvDt, Volume.Bone, Mesh));
		}

		UE_LOG(LogRopeCollision, VeryVerbose, TEXT("SDFProvider on %s: built %d collider(s) from %d baked volume(s)."),
			*GetNameSafe(GetOwner()), Colliders.Num(), SDFData->BoneVolumes.Num());
	}

	// 캐시된 collider 포인터를 넘긴다(해당 프레임 동안 유효).
	const int32 StartIndex = Gather.Colliders.Num();
	Gather.Colliders.Reserve(StartIndex + Colliders.Num());
	for (FRopeSDFCollider& Collider : Colliders)
	{
		Gather.Colliders.Add(&Collider);
	}

	// region 매핑: 메시(볼륨 유니언) 선-거절 → 걸린 로프만 볼륨별 bounds 배정(캡슐 provider와 동일 원리).
	RopeColliderGather::MapCollidersToRegionsByBounds(Gather, StartIndex);
}
