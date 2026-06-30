// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeSDFData를 IRopeCollider로 공급하는 provider. 본별 볼륨을 현재 본 월드 트랜스폼으로 변환해
// 매 프레임 FRopeSDFCollider를 빌드한다. URopeBoneCapsuleProvider와 동일 인터페이스라 캡슐과
// 공존/대체 가능(비블로킹). 미베이크 볼륨은 건너뛰므로 데이터가 비어도 안전한 no-op.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
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
class DYNAMICROPE_API URopeSDFProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeSDFProvider();

	//~ UActorComponent — RopeSimSubsystem 중앙 레지스트리에 등록/해제(프레임당 1회 중앙 gather).
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** 본별 SDF 볼륨 에셋. 비어 있으면 collider를 공급하지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<URopeSDFData> SDFData = nullptr;

	/** 본 트랜스폼을 제공하는 메시. null로 두면 owner에서 자동 해석된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<USkeletalMeshComponent> SkeletalMesh = nullptr;

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

	//~ IRopeColliderProvider
	virtual void GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders) override;

private:
	/** BoneFilter 드롭다운(GetOptions)에 노출할 후보: SDFData에 베이크된 본 이름들. */
	UFUNCTION()
	TArray<FName> GetBakedBoneNames() const;

	// 프레임당 1회 재구성되는 백킹 스토리지. 넘겨준 포인터는 해당 프레임 동안 유효하다.
	TArray<FRopeSDFCollider> Colliders;

	// 본별 이전 프레임 BoneToWorld. 표면 속도(드래그) 산출용 — collider 빌드 시 (현재, 이전)으로 속도를 만든다.
	TMap<FName, FTransform> PrevBoneToWorld;

	// 마지막으로 collider를 빌드한 GFrameCounter. 같은 프레임에 여러 로프가 호출해도 재빌드 안 함(디둡).
	uint64 BuiltFrame = static_cast<uint64>(-1);

	USkeletalMeshComponent* ResolveMesh();
};
