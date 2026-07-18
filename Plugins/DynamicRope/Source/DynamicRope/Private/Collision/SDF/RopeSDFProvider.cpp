// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFProvider.h"
#include "Collision/SDF/RopeSDFData.h"
#include "DynamicRopeLog.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"

bool URopeSDFProvider::HasColliderData() const
{
	if (!SDFData)
	{
		UE_LOG(LogRopeCollision, Verbose, TEXT("SDFProvider on %s: SDFData asset missing — no colliders."),
			*GetNameSafe(GetOwner()));
		return false;
	}
	return true;
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

void URopeSDFProvider::RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt)
{
	// per-rope 컬링은 solver의 collider AABB broad-phase가 담당하므로, 여기서는 RopeBounds 컬 없이
	// 베이크된 모든 볼륨을 빌드한다(collider 구성은 저렴 — 비싼 Query를 solver가 컬). HasColliderData가
	// 이미 SDFData 유효를 보장한다.
	Colliders.Reset();

	for (const FRopeBoneSDFVolume& Volume : SDFData->BoneVolumes)
	{
		if (Volume.Bone.IsNone() || !Volume.IsBaked())
		{
			// 미베이크/무효 볼륨은 건너뛴다.
			continue;
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
		// 볼륨 안정 키 = 에셋 런타임 ID(로드마다 부여) << 16 | 본 인덱스. raw 포인터 대신 써서 언로드 오샘플 방지.
		const int32 BoneIdx = static_cast<int32>(&Volume - SDFData->BoneVolumes.GetData());
		const uint64 VolKey = (SDFData->GetRuntimeVolumeId() << 16) | static_cast<uint64>(BoneIdx & 0xFFFF);
		Colliders.Add(FRopeSDFCollider(&Volume, BoneToWorld, PrevXform, InvDt, Volume.Bone, Mesh, VolKey));
	}

	UE_LOG(LogRopeCollision, VeryVerbose, TEXT("SDFProvider on %s: built %d collider(s) from %d baked volume(s)."),
		*GetNameSafe(GetOwner()), Colliders.Num(), SDFData->BoneVolumes.Num());
}

void URopeSDFProvider::AppendColliderPointers(FRopeColliderGatherContext& Gather)
{
	Gather.Colliders.Reserve(Gather.Colliders.Num() + Colliders.Num());
	for (FRopeSDFCollider& Collider : Colliders)
	{
		Gather.Colliders.Add(&Collider);
	}
}
