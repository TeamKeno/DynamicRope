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

namespace
{
	/**
	 * 비균등 스케일 경고(본당 1회). SDF 질의는 로컬↔월드 거리 환산을 **단일 스칼라**(스케일 최대 성분)로
	 * 근사하므로 — CPU FRopeSDFCollider::Query / GPU RopeQuerySDFWorld 양쪽 동일 — 축마다 스케일이 다르면
	 * 접촉 밴드와 침투 깊이가 축별로 어긋난다. 근사 자체는 의도된 계약이지만 지금까지 아무 신호가 없어
	 * "왜 이 메시만 로프가 파고드나"를 추적할 단서가 없었다. 음수 스케일(미러링)은 이제 정상 지원한다 —
	 * 여기서 보는 것은 성분 간 *비율*뿐이라 -1 균등 미러는 경고하지 않는다.
	 */
	void WarnOnNonUniformScaleOnce(const FTransform& BoneToWorld, FName Bone)
	{
#if !UE_BUILD_SHIPPING
		const FVector Abs = BoneToWorld.GetScale3D().GetAbs();
		const double MaxC = Abs.GetMax();
		const double MinC = Abs.GetMin();
		if (MaxC <= KINDA_SMALL_NUMBER || MaxC - MinC <= 0.01 * MaxC)
		{
			return;
		}
		static TSet<FName> WarnedBones;
		if (WarnedBones.Contains(Bone))
		{
			return;
		}
		WarnedBones.Add(Bone);
		UE_LOG(LogRopeCollision, Warning,
			TEXT("SDF 콜라이더 본 '%s'의 스케일이 비균등하다(%s) — SDF 거리 환산은 최대 성분 스칼라 근사라 ")
			TEXT("축별로 접촉 밴드/침투가 어긋난다. 균등 스케일을 권장한다."),
			*Bone.ToString(), *BoneToWorld.GetScale3D().ToCompactString());
#endif
	}
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
		WarnOnNonUniformScaleOnce(BoneToWorld, Volume.Bone);
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
