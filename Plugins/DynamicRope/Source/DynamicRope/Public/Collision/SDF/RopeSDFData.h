// Copyright Epic Games, Inc. All Rights Reserved.
//
// 본별(per-bone) signed distance field를 담는 에셋. provider가 런타임에 본 트랜스폼으로 월드 변환하여
// FRopeSDFCollider로 노출한다. 캡슐 provider와 동일한 IRopeCollider 계약 뒤에 위치한다.
// 베이크/오써링은 DynamicRopeEditor의 SDF 도크탭이 담당한다(이 단계에서는 그릇만 정의).

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "RopeSDFData.generated.h"

class USkeletalMesh;

/**
 * 베이크 1회에 대한 디자이너용 설정 값. 베이크 입력이자, 에셋에 함께 저장되어 재오써링 시
 * "이 에셋이 어떤 설정으로 구워졌는가"를 알려주는 비교 기준이 된다(URopeSDFData::LastBakeSettings).
 * 기본값은 신규 베이크의 출발점이다. 에디터 베이커(FRopeSDFBaker)가 이 타입을 그대로 입력으로 받는다.
 */
USTRUCT(BlueprintType)
struct FRopeSDFBakeSettings
{
	GENERATED_BODY()

	/** 샘플 간격(cm, 큐브 voxel). 작을수록 표면이 선명해지고 메모리/시간이 늘어난다. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF")
	float VoxelSize = 1.5f;

	/** 축당 샘플 상한. 본 grid가 이를 넘으면 VoxelSize를 키워 맞춘다. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF")
	int32 MaxResolution = 48;

	/** |거리|를 이 밴드(cm)로 clamp. 밴드 밖 값은 충돌과 무관하다. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF")
	float NarrowBand = 6.0f;

	/** 삼각형을 본에 배정하기 위한 최소 평균 스킨 가중치 [0..1]. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF")
	float WeightThreshold = 0.2f;

	/** voxel화 전 본 삼각형 AABB를 확장(cm) — 스킨 바깥에도 밴드 여유를 둔다. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF")
	float BoundsPadding = 3.0f;
};

/**
 * 단일 본에 귀속된 좁은밴드 signed distance grid. 본 로컬 공간에 구워지며, 샘플은 행 우선
 * (idx = x + y*Res.X + z*Res.X*Res.Y)으로 저장된다. distance 단위는 cm, 바깥쪽이 양수.
 * Distances가 비어 있으면(미베이크) provider는 이 본의 collider를 만들지 않는다.
 *
 * NOTE: 이 단계에서는 float 평면 배열로 둔다. FFloat16/uint16 좁은밴드 압축은 베이크 본작업(B3)에서.
 */
USTRUCT()
struct FRopeBoneSDFVolume
{
	GENERATED_BODY()

	/** 이 볼륨이 귀속되는 본. FRopeContact.Bone으로 전파되어 DecideWrap이 wrap을 attribute한다. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF")
	FName Bone = NAME_None;

	/** distance grid가 덮는 본 로컬 공간 AABB. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF")
	FBox LocalBounds = FBox(ForceInit);

	/** grid 해상도(voxel 개수, 축별). */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF")
	FIntVector Resolution = FIntVector::ZeroValue;

	/** voxel 한 변 길이(cm). LocalBounds/Resolution에서 유도되는 캐시 값. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF")
	float VoxelSize = 0.0f;

	/** signed distance 샘플(cm, 바깥쪽 +). 길이 = Resolution.X*Y*Z. 비어 있으면 미베이크. */
	UPROPERTY()
	TArray<float> Distances;

	/** 베이크가 끝나 샘플 수가 해상도와 일치하는가. */
	bool IsBaked() const
	{
		const int64 Expected = static_cast<int64>(Resolution.X) * Resolution.Y * Resolution.Z;
		return Expected > 0 && Distances.Num() == Expected;
	}
};

/**
 * 한 스켈레탈 메시에 대한 본별 SDF 묶음. URopeSDFProvider가 참조하여 매 프레임 collider를 공급한다.
 * Content Browser에서 생성(팩토리)하고 SDF 도크탭에서 메시를 지정해 베이크한다.
 */
// 에디터 UI(에셋 피커 등) tooltip은 영어로 노출한다 — 한국어 주석 대신 명시적 ToolTip 메타를 사용.
UCLASS(BlueprintType, meta = (ToolTip = "Per-bone signed distance field set baked from one skeletal mesh. URopeSDFProvider references it to supply a collider per bone each frame. Create it in the Content Browser, then assign a Source Mesh and bake it from the Rope SDF Authoring tab."))
class DYNAMICROPE_API URopeSDFData : public UDataAsset
{
	GENERATED_BODY()

public:
	/** SDF가 구워진 원본 메시(soft — 런타임 강제 로드 안 함, 오써링/검증 참조용). */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF")
	TSoftObjectPtr<USkeletalMesh> SourceMesh;

	/** 본별 distance 볼륨. provider가 본 트랜스폼으로 월드 변환해 노출한다. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF")
	TArray<FRopeBoneSDFVolume> BoneVolumes;

	/**
	 * 마지막 베이크에 사용된 설정. 오써링 패널이 타깃 로드 시 읽어와 현재 결과와 비교/조정의 기준으로
	 * 삼는다. 이 필드가 추가되기 전 구워진 에셋은 기본값이 직렬화되어 있어, 한 번 다시 베이크해야
	 * 실제 값이 기록된다.
	 */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF")
	FRopeSDFBakeSettings LastBakeSettings;

	/** 본 이름으로 볼륨을 찾는다. 없으면 nullptr. */
	const FRopeBoneSDFVolume* FindVolume(FName Bone) const;

	/** 하나라도 베이크된 볼륨이 있는가. */
	bool HasAnyBakedVolume() const;
};
