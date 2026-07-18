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

/** SDF 거리 양자화 비트수. 코드 범위와 복셀당 바이트 수(1 or 2)를 결정한다. */
UENUM()
enum class ERopeSDFQuantBits : uint8
{
	/** 복셀당 1바이트(0..255). 최소 용량. 스텝 = 밴드범위/255. */
	UInt8  UMETA(DisplayName = "8-bit (smallest)"),
	/** 복셀당 2바이트(0..65535, 리틀엔디안). 스텝 = 밴드범위/65535 (256× 미세). 용량 2배. */
	UInt16 UMETA(DisplayName = "16-bit (finer)"),
};

/**
 * 베이크 1회에 대한 디자이너용 설정 값. 베이크 입력이자, 에셋에 함께 저장되어 재오써링 시
 * "이 에셋이 어떤 설정으로 구워졌는가"를 알려주는 비교 기준이 된다(URopeSDFData::LastBakeSettings).
 * 기본값은 신규 베이크의 출발점이다. 에디터 베이커(FRopeSDFBaker)가 이 타입을 그대로 입력으로 받는다.
 * 에디터 UI(에셋 에디터 details 등) tooltip은 영어로 노출한다 — 한국어 주석 대신 명시적 ToolTip 메타를 사용.
 */
USTRUCT(BlueprintType, meta = (ToolTip = "Designer-facing settings for a single bake. Used as bake input and stored on the asset (URopeSDFData.LastBakeSettings) as the comparison baseline when re-authoring."))
struct FRopeSDFBakeSettings
{
	GENERATED_BODY()

	/** 샘플 간격(cm, 큐브 voxel). 작을수록 표면이 선명해지고 메모리/시간이 늘어난다. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Sample spacing in cm (cube voxel). Smaller sharpens the surface but increases memory and bake time."))
	float VoxelSize = 2.0f;

	/** 축당 샘플 상한. 본 grid가 이를 넘으면 VoxelSize를 키워 맞춘다. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Maximum samples per axis. If a bone's grid would exceed this, VoxelSize is increased to fit."))
	int32 MaxResolution = 48;

	/** 거리 양자화 비트수. 16-bit는 8-bit보다 256배 미세하지만 에셋/RAM 용량 2배. 8-bit가 최소. 기본 16-bit. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "SDF distance quantization bit depth. 16-bit is 256x finer than 8-bit but doubles asset/RAM size; 8-bit is the smallest. Default 16-bit."))
	ERopeSDFQuantBits Quantization = ERopeSDFQuantBits::UInt16;

	/** 바깥(자유공간) 방향 감지 밴드(cm). 표면 밖으로 이 거리까지 유효한 거리/법선을 저장한다 → 로프가
	    이만큼 떨어진 지점부터 몸을 감지·반응한다. 접촉은 CollisionRadius에서 일어나므로 그 2~3배가 안정.
	    안쪽(몸 속) 밴드는 베이크 시 본별 내부 최대 깊이로 자동 산출되어(설정 불필요) 내부 전체를 덮는다. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ClampMin = "0.1", Units = "cm", ToolTip = "Outward (free-space) detection band in cm. Stores valid distance/normal up to this far outside the surface, so the rope starts reacting to the body from this distance. Contact happens at CollisionRadius, so ~2-3x that is stable. The inward (inside-body) band is auto-sized per bone at bake time to the deepest interior distance (no setting needed), so the whole interior is covered."))
	float NarrowBand = 3.0f;

	/** 삼각형을 본에 배정하기 위한 최소 평균 스킨 가중치 [0..1]. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Minimum average skin weight [0..1] for a triangle to be assigned to a bone."))
	float WeightThreshold = 0.2f;

	/** voxel화 전 본 삼각형 AABB를 확장(cm) — 스킨 바깥에도 밴드 여유를 둔다. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Expand each bone's triangle AABB by this much (cm) before voxelizing, leaving band margin beyond the skin."))
	float BoundsPadding = 0.0f;

	/** 단면 girth(가는 쪽 두께)가 이 값(cm) 미만인 본은 baking에서 제외(drop)한다 — 부모로 합치지 않고
	    그냥 굽지 않는다. 로프는 자기 굵기보다 가는 특징엔 못 걸리므로, 이 SDF를 쓸 가장 가는 로프의
	    CollisionRadius 정도(또는 그 이상)로 둔다. 0이면 drop 비활성(전체 본 baking). */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ClampMin = "0.0", Units = "cm", ToolTip = "Bones whose cross-section girth is thinner than this (cm) are dropped from baking (not merged into the parent - just not baked). A rope cannot catch features finer than its own radius, so set this near (or above) the CollisionRadius of the thinnest rope that will use this SDF. 0 disables dropping (bake every bone)."))
	float MinBoneGirth = 8.0f;
};

/**
 * 단일 본에 귀속된 좁은밴드 signed distance grid. 본 로컬 공간에 구워지며, 샘플은 행 우선
 * (idx = x + y*Res.X + z*Res.X*Res.Y)으로 저장된다. distance 단위는 cm, 바깥쪽이 양수.
 * Distances가 비어 있으면(미베이크) provider는 이 본의 collider를 만들지 않는다.
 *
 * 에디터 UI(에셋 에디터 details 등) tooltip은 영어로 노출한다 — 한국어 주석 대신 명시적 ToolTip 메타를 사용.
 */
USTRUCT(meta = (ToolTip = "Narrow-band signed distance grid attributed to a single bone, baked in bone-local space. Distance is in cm, positive outside."))
struct FRopeBoneSDFVolume
{
	GENERATED_BODY()

	/** 이 볼륨이 귀속되는 본. FRopeContact.Bone으로 전파되어 접촉 집계가 wrap을 attribute한다. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Bone this volume is attributed to. Propagates to FRopeContact.Bone so DecideWrap can attribute the wrap."))
	FName Bone = NAME_None;

	/** distance grid가 덮는 본 로컬 공간 AABB. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Bone-local AABB covered by the distance grid."))
	FBox LocalBounds = FBox(ForceInit);

	/** grid 해상도(voxel 개수, 축별). */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Grid resolution (voxel count per axis)."))
	FIntVector Resolution = FIntVector::ZeroValue;

	/** voxel 한 변 길이(cm). LocalBounds/Resolution에서 유도되는 캐시 값. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Voxel edge length (cm). Cached value derived from LocalBounds/Resolution."))
	float VoxelSize = 0.0f;

	/** signed distance 양자화 코드 바이트 블롭. 복셀당 BytesPerCode(1=uint8, 2=uint16 리틀엔디안) 바이트,
	    행 우선. 비대칭 밴드 [-NarrowBandInner,+NarrowBandOuter]를 [0, MaxCode]로 선형 매핑(바깥 +).
	    길이 = Resolution.X*Y*Z * BytesPerCode. 비어 있으면 미베이크. DecodeDistance로 cm 거리 복원. */
	UPROPERTY()
	TArray<uint8> Distances;

	/** 이 볼륨이 구워진 양자화 비트수 → Distances 바이트 레이아웃(1 or 2바이트/복셀)을 결정. 기본 UInt8:
	    이 필드가 없던 구 에셋(uint8 1바이트/복셀)이 재베이크 없이 그대로 디코드되도록 한다. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Quantization bit depth this volume was baked with. Determines the Distances byte layout (1 or 2 bytes per voxel)."))
	ERopeSDFQuantBits QuantBits = ERopeSDFQuantBits::UInt8;

	/** 복셀당 바이트 수(1=uint8, 2=uint16). */
	FORCEINLINE int32 BytesPerCode() const { return QuantBits == ERopeSDFQuantBits::UInt16 ? 2 : 1; }

	/** 안쪽(몸 속) dequant 밴드(cm). 코드 0이 -NarrowBandInner에 대응. 베이크 시 본별 내부 최대 깊이로
	    자동 산출 → 몸통 내부 전체가 밴드 안(깊이 박힌 노드도 최근접 표면 방향으로 회복). 내부가 없으면 0. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Inward (inside-body) dequant band in cm. Code 0 maps to -NarrowBandInner. Auto-sized per bone at bake to the deepest interior distance, so the whole interior is covered (a deeply-penetrating node still recovers toward the nearest surface). 0 if the bone has no interior."))
	float NarrowBandInner = 0.0f;

	/** 바깥(자유공간) dequant 밴드(cm). 코드 255가 +NarrowBandOuter에 대응. 베이크 설정의 감지 밴드값.
	    0이면 미베이크/무효(또는 구 포맷 로드 실패 → 재베이크 필요). */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Outward (free-space) dequant band in cm. Code 255 maps to +NarrowBandOuter. Equals the bake detection band. 0 means unbaked/invalid (or a load from the old format failed - rebake needed)."))
	float NarrowBandOuter = 0.0f;

	/** 양자화 전체 범위(cm) = 안쪽 + 바깥쪽. 코드 0..255가 -Inner..+Outer에 대응. 0이면 무효. */
	FORCEINLINE float QuantRange() const { return NarrowBandInner + NarrowBandOuter; }

	/** 베이크가 끝나 바이트 수가 (해상도 × BytesPerCode)와 일치하고 dequant 범위가 유효한가. */
	bool IsBaked() const
	{
		const int64 Expected = static_cast<int64>(Resolution.X) * Resolution.Y * Resolution.Z;
		return Expected > 0 && QuantRange() > 0.0f
			&& static_cast<int64>(Distances.Num()) == Expected * BytesPerCode();
	}

	/** 코드 → signed distance(cm, 바깥 +). 범위/인덱스가 무효면 0. BytesPerCode에 따라 1 or 2바이트 읽음. */
	FORCEINLINE float DecodeDistance(int32 Index) const
	{
		const float Range = QuantRange();
		const int32 Bpc = BytesPerCode();
		const int32 Base = Index * Bpc;
		if (Range <= 0.0f || Index < 0 || !Distances.IsValidIndex(Base + Bpc - 1))
		{
			return 0.0f;
		}
		uint32 Code = Distances[Base];
		if (Bpc >= 2)
		{
			// 리틀엔디안.
			Code |= static_cast<uint32>(Distances[Base + 1]) << 8;
		}
		const float MaxCodeF = (Bpc >= 2) ? 65535.0f : 255.0f;
		return static_cast<float>(Code) * (Range / MaxCodeF) - NarrowBandInner;
	}

	/** signed distance(cm) → 정수 코드 [0, MaxCode]. [-NBInner,+NBOuter]로 clamp. */
	static FORCEINLINE uint32 EncodeCode(float Distance, float NBInnerCm, float NBOuterCm, uint32 MaxCode)
	{
		const float Range = NBInnerCm + NBOuterCm;
		if (Range <= 0.0f)
		{
			// 무효 범위 폴백(중앙 ≈ 0).
			return MaxCode / 2;
		}
		// [-NBInner,+NBOuter] -> [0,1].
		const float T = (Distance + NBInnerCm) / Range;
		return static_cast<uint32>(FMath::Clamp(FMath::RoundToInt(T * static_cast<float>(MaxCode)), 0, static_cast<int32>(MaxCode)));
	}

	/** D를 인코딩해 blob의 Index 위치(복셀 단위)에 BytesPerCode 바이트로 리틀엔디안 저장.
	    Out은 Count*BytesPerCode 크기여야 한다. 인덱스별 바이트 범위가 겹치지 않아 병렬 안전. */
	static FORCEINLINE void EncodeInto(TArray<uint8>& Out, int32 Index, float Distance, float NBInnerCm, float NBOuterCm, int32 BytesPerCode)
	{
		const uint32 MaxCode = (BytesPerCode >= 2) ? 65535u : 255u;
		const uint32 Code = EncodeCode(Distance, NBInnerCm, NBOuterCm, MaxCode);
		const int32 Base = Index * BytesPerCode;
		Out[Base] = static_cast<uint8>(Code & 0xFF);
		if (BytesPerCode >= 2)
		{
			Out[Base + 1] = static_cast<uint8>((Code >> 8) & 0xFF);
		}
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
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Skeletal mesh the SDF was baked from (soft reference — not force-loaded at runtime; used for authoring/validation)."))
	TSoftObjectPtr<USkeletalMesh> SourceMesh;

	/** 본별 distance 볼륨. provider가 본 트랜스폼으로 월드 변환해 노출한다. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Per-bone distance volumes. The provider transforms each into world space by its bone transform."))
	TArray<FRopeBoneSDFVolume> BoneVolumes;

	/**
	 * 마지막 베이크에 사용된 설정. 오써링 패널이 타깃 로드 시 읽어와 현재 결과와 비교/조정의 기준으로
	 * 삼는다. 이 필드가 추가되기 전 구워진 에셋은 기본값이 직렬화되어 있어, 한 번 다시 베이크해야
	 * 실제 값이 기록된다.
	 */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Settings used for the last bake. The authoring panel restores these when the asset is loaded so you can compare and adjust. Assets baked before this field existed serialize defaults until re-baked."))
	FRopeSDFBakeSettings LastBakeSettings;

	/** 본 이름으로 볼륨을 찾는다. 없으면 nullptr. */
	const FRopeBoneSDFVolume* FindVolume(FName Bone) const;

	/** 하나라도 베이크된 볼륨이 있는가. */
	bool HasAnyBakedVolume() const;

	/** 이 에셋 인스턴스의 런타임 전용 고유 ID(로드마다 부여, 재사용 없음, 직렬화 안 함). GPU SDF 캐시가 raw
	 *  포인터 대신 이 ID + 본 인덱스로 볼륨을 키잉해, 에셋 언로드→메모리 재사용 시 옛 복셀 오샘플을 막는다(lazy). */
	uint64 GetRuntimeVolumeId() const;

private:
	/** GetRuntimeVolumeId lazy 캐시(0=미할당). const 게터가 최초 호출 시 채운다(mutable). */
	mutable uint64 RuntimeVolumeId = 0;
};
