// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeSDFData를 IRopeCollider로 공급하는 skeletal collider provider. 본별 볼륨을 현재 본 월드
// 트랜스폼으로 변환해 매 프레임 FRopeSDFCollider를 빌드한다. URopeBoneCapsuleProvider와 같은
// 베이스(URopeSkeletalColliderProvider)라 캡슐과 공존/대체 가능(비블로킹). 미베이크 볼륨은 건너뛰므로
// 데이터가 비어도 안전한 no-op. 등록/메시 해석/프레임 디둡/gather 파이프라인은 베이스가 소유한다.

#pragma once

#include "CoreMinimal.h"
#include "Collision/RopeSkeletalColliderProvider.h"
#include "Collision/SDF/RopeSDFCollider.h"
#include "RopeSDFProvider.generated.h"

class URopeSDFData;
class USkeletalMeshComponent;

/** SDF slice heatmap이 통과하는 축(평면은 나머지 두 축에 평행). */
UENUM()
enum class ERopeSDFSliceAxis : uint8
{
	X,
	Y,
	Z
};

/** 베이크된 본 중 어떤 본을 실제 collider로 노출할지 고르는 모드(베이크는 그대로, 런타임 필터). */
UENUM()
enum class ERopeSDFBoneFilterMode : uint8
{
	/** Use every baked bone (default - no filtering). */
	All,
	/** Collide only with the bones listed in Bone Filter. */
	Include,
	/** Collide with every baked bone except those listed in Bone Filter. */
	Exclude
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeSDFProvider : public URopeSkeletalColliderProvider
{
	GENERATED_BODY()

public:
	/** 본별 SDF 볼륨 에셋. 비어 있으면 collider를 공급하지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<URopeSDFData> SDFData = nullptr;

	/**
	 * Runtime filter selecting which baked bones are exposed as colliders (for debugging / isolation).
	 * All = use every baked bone (existing behaviour); Include/Exclude apply the Bone Filter list below.
	 * Baking is left untouched - toggles live in the details panel with no re-bake.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	ERopeSDFBoneFilterMode BoneFilterMode = ERopeSDFBoneFilterMode::All;

	/**
	 * Bones targeted in Include/Exclude mode (ignored when mode is All).
	 * The dropdown lists only bones actually baked into the SDFData asset, not the whole skeleton.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision",
		meta = (EditCondition = "BoneFilterMode != ERopeSDFBoneFilterMode::All", GetOptions = "GetBakedBoneNames"))
	TArray<FName> BoneFilter;

protected:
	//~ URopeSkeletalColliderProvider
	virtual void RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt) override;
	virtual void AppendColliderPointers(FRopeColliderGatherContext& Gather) override;
	virtual bool HasColliderData() const override;

private:
	/** BoneFilter 드롭다운(GetOptions)에 노출할 후보: SDFData에 베이크된 본 이름들. */
	UFUNCTION()
	TArray<FName> GetBakedBoneNames() const;

	// 프레임당 1회 재구성되는 백킹 스토리지. 넘겨준 포인터는 해당 프레임 동안 유효하다.
	TArray<FRopeSDFCollider> Colliders;

	// 본별 이전 프레임 BoneToWorld. 표면 속도(드래그) 산출용 — collider 빌드 시 (현재, 이전)으로 속도를 만든다.
	TMap<FName, FTransform> PrevBoneToWorld;
};
