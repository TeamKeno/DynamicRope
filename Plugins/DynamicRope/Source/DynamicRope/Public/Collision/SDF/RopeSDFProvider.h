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

	/** 빌드된 볼륨의 월드 bounds를 매 프레임 그린다(녹색 = rope bounds와 겹침). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	bool bDrawDebug = false;

#if WITH_EDITORONLY_DATA
	//~ SDF 시각화(에디터 전용 비주얼라이저 FRopeSDFVisualizer가 읽는 토글). 런타임 충돌과 무관 — 쿠킹 빌드에서
	//~ 제외되도록 WITH_EDITORONLY_DATA로 감싼다(BP 런타임 접근 불가라 BlueprintReadWrite도 뺀다). ----------------
	/** 본별 SDF 볼륨의 bounds 박스를 그린다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug")
	bool bDrawSDFBounds = false;

	/** 좁은밴드 voxel을 부호별 색 점으로 그린다(안=빨강, 밖=파랑, ≈0=흰색). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug")
	bool bDrawSDFVoxels = false;

	/** voxel 표시 밴드 두께(cm). |distance| <= 이 값인 voxel만 그린다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug", meta = (ClampMin = "0.0", Units = "cm"))
	float SDFBandThreshold = 3.0f;

	/** 베이크 전 미리보기: 각 볼륨 본에 해석적 구 SDF를 합성해 그린다(실제 데이터 대신). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug")
	bool bSDFSyntheticPreview = false;

	/** 합성 미리보기 구의 반지름(cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug", meta = (ClampMin = "1.0", Units = "cm"))
	float SDFSyntheticRadius = 10.0f;

	//~ Slice plane heatmap -------------------------------------------------
	/** 볼륨을 가로지르는 평면 위 distance를 발산형 색(음=파랑, 0=흰, 양=빨강)으로 표시한다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug")
	bool bDrawSDFSlice = false;

	/** slice 평면이 통과하는 축. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug", meta = (EditCondition = "bDrawSDFSlice"))
	ERopeSDFSliceAxis SDFSliceAxis = ERopeSDFSliceAxis::Z;

	/** 축을 따른 slice 위치(0~1, 정규화). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bDrawSDFSlice"))
	float SDFSlicePosition = 0.5f;

	/** slice 샘플 격자 한 변의 개수. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug", meta = (ClampMin = "2", EditCondition = "bDrawSDFSlice"))
	int32 SDFSliceResolution = 24;

	/** 색 매핑 스케일(cm): |distance| = 이 값에서 완전 포화. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug", meta = (ClampMin = "0.1", Units = "cm", EditCondition = "bDrawSDFSlice"))
	float SDFSliceColorScale = 10.0f;

	//~ Gradient arrows (= Query 법선) --------------------------------------
	/** 좁은밴드 샘플에서 gradient(바깥쪽 = Query가 반환할 법선) 방향을 화살표로 그린다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug")
	bool bDrawSDFGradient = false;

	/** gradient 화살표 길이(cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision|SDF Debug", meta = (ClampMin = "0.5", Units = "cm", EditCondition = "bDrawSDFGradient"))
	float SDFGradientLength = 4.0f;
#endif // WITH_EDITORONLY_DATA

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
