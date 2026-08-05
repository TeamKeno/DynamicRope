// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The asset holding a per-bone signed distance field. At runtime the provider transforms each volume
// by its bone transform and exposes it as an FRopeSDFCollider, behind the same IRopeCollider
// contract as the capsule provider. Baking and authoring are handled by the SDF dock tab in the
// DynamicRopeEditor module.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "RopeSDFData.generated.h"

class USkeletalMesh;

/** The bit depth SDF distances are quantized to, which fixes both the code range and the bytes per
 *  voxel, either 1 or 2. */
UENUM()
enum class ERopeSDFQuantBits : uint8
{
	/** One byte per voxel, 0 to 255, the smallest option. The step is the band range divided by 255. */
	UInt8  UMETA(DisplayName = "8-bit (smallest)"),
	/** Two bytes per voxel, 0 to 65535, little-endian. The step is the band range divided by 65535,
	 *  which is 256 times finer, and the size doubles. */
	UInt16 UMETA(DisplayName = "16-bit (finer)"),
};

/**
 * The designer-facing settings for a single bake. They are both the bake input and, once stored on
 * the asset as URopeSDFData::LastBakeSettings, the baseline that says which settings the asset was
 * baked with when it is authored again. The defaults are the starting point for a new bake, and the
 * editor baker, FRopeSDFBaker, takes this type directly as its input.
 */
USTRUCT(BlueprintType, meta = (ToolTip = "Designer-facing settings for a single bake. Used as bake input and stored on the asset (URopeSDFData.LastBakeSettings) as the comparison baseline when re-authoring."))
struct FRopeSDFBakeSettings
{
	GENERATED_BODY()

	/** Sample spacing in cm, as a cube voxel. Smaller sharpens the surface at the cost of memory and
	 *  bake time. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Sample spacing in cm (cube voxel). Smaller sharpens the surface but increases memory and bake time."))
	float VoxelSize = 2.0f;

	/** The maximum number of samples per axis. A bone whose grid would exceed it has its voxel size
	 *  increased to fit. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Maximum samples per axis. If a bone's grid would exceed this, VoxelSize is increased to fit."))
	int32 MaxResolution = 48;

	/** The distance quantization bit depth. 16-bit is 256 times finer than 8-bit but doubles the asset
	 *  and memory size; 8-bit is the smallest. The default is 16-bit. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "SDF distance quantization bit depth. 16-bit is 256x finer than 8-bit but doubles asset/RAM size; 8-bit is the smallest. Default 16-bit."))
	ERopeSDFQuantBits Quantization = ERopeSDFQuantBits::UInt16;

	/** The outward detection band into free space (cm). Valid distances and normals are stored up to
	    this far outside the surface, so the rope starts reacting to the body from that distance.
	    Contact happens at the collision radius, so roughly two to three times that is stable.
	    The inward band, inside the body, is sized automatically per bone at bake time from the deepest
	    interior distance and needs no setting, so the whole interior is covered. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ClampMin = "0.1", Units = "cm", ToolTip = "Outward (free-space) detection band in cm. Stores valid distance/normal up to this far outside the surface, so the rope starts reacting to the body from this distance. Contact happens at CollisionRadius, so ~2-3x that is stable. The inward (inside-body) band is auto-sized per bone at bake time to the deepest interior distance (no setting needed), so the whole interior is covered."))
	float NarrowBand = 3.0f;

	/** The minimum average skin weight, from 0 to 1, for a triangle to be assigned to a bone. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Minimum average skin weight [0..1] for a triangle to be assigned to a bone."))
	float WeightThreshold = 0.2f;

	/** Expands each bone's triangle AABB by this much (cm) before voxelizing, which leaves band margin
	 *  beyond the skin. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Expand each bone's triangle AABB by this much (cm) before voxelizing, leaving band margin beyond the skin."))
	float BoundsPadding = 0.0f;

	/** Bones whose cross-sectional girth is thinner than this (cm) are dropped from the bake entirely,
	    rather than merged into their parent. A rope cannot catch on features finer than its own
	    radius, so set this near or above the collision radius of the thinnest rope that will use this
	    SDF. 0 disables dropping and bakes every bone. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ClampMin = "0.0", Units = "cm", ToolTip = "Bones whose cross-section girth is thinner than this (cm) are dropped from baking (not merged into the parent - just not baked). A rope cannot catch features finer than its own radius, so set this near (or above) the CollisionRadius of the thinnest rope that will use this SDF. 0 disables dropping (bake every bone)."))
	float MinBoneGirth = 8.0f;
};

/**
 * A narrow-band signed distance grid attributed to a single bone, baked in bone-local space. Samples
 * are stored in row-major order, so the index is x + y * Res.X + z * Res.X * Res.Y. Distances are in
 * cm and positive outside.
 * When Distances is empty, meaning the volume is not baked, the provider builds no collider for that
 * bone.
 */
USTRUCT(meta = (ToolTip = "Narrow-band signed distance grid attributed to a single bone, baked in bone-local space. Distance is in cm, positive outside."))
struct FRopeBoneSDFVolume
{
	GENERATED_BODY()

	/** The bone this volume is attributed to. It propagates to FRopeContact::Bone so contact
	 *  aggregation can attribute the wrap. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Bone this volume is attributed to. Propagates to FRopeContact.Bone so DecideWrap can attribute the wrap."))
	FName Bone = NAME_None;

	/** The bone-local AABB the distance grid covers. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Bone-local AABB covered by the distance grid."))
	FBox LocalBounds = FBox(ForceInit);

	/** The grid resolution, as a voxel count per axis. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Grid resolution (voxel count per axis)."))
	FIntVector Resolution = FIntVector::ZeroValue;

	/** The voxel edge length (cm), cached from LocalBounds and Resolution. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Voxel edge length (cm). Cached value derived from LocalBounds/Resolution."))
	float VoxelSize = 0.0f;

	/** The blob of quantized signed distance codes: BytesPerCode bytes per voxel, where 1 is uint8 and
	    2 is little-endian uint16, in row-major order. The asymmetric band from -NarrowBandInner to
	    +NarrowBandOuter is mapped linearly onto 0 to MaxCode, positive outside.
	    Its length is Resolution.X * Y * Z * BytesPerCode, and it is empty when unbaked. Use
	    DecodeDistance to recover a distance in cm. */
	UPROPERTY()
	TArray<uint8> Distances;

	/** The quantization bit depth this volume was baked with, which determines the byte layout of
	    Distances, either 1 or 2 bytes per voxel. It defaults to UInt8 so that an older asset baked
	    before this field existed, which used one byte per voxel, still decodes without a rebake. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Quantization bit depth this volume was baked with. Determines the Distances byte layout (1 or 2 bytes per voxel)."))
	ERopeSDFQuantBits QuantBits = ERopeSDFQuantBits::UInt8;

	/** The number of bytes per voxel: 1 for uint8 and 2 for uint16. */
	FORCEINLINE int32 BytesPerCode() const { return QuantBits == ERopeSDFQuantBits::UInt16 ? 2 : 1; }

	/** The inward dequantization band, inside the body (cm), where code 0 maps to -NarrowBandInner. It
	    is sized automatically per bone at bake time from the deepest interior distance, so the whole
	    interior falls within the band and even a deeply penetrating node still recovers towards the
	    nearest surface. It is 0 when the bone has no interior. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Inward (inside-body) dequant band in cm. Code 0 maps to -NarrowBandInner. Auto-sized per bone at bake to the deepest interior distance, so the whole interior is covered (a deeply-penetrating node still recovers toward the nearest surface). 0 if the bone has no interior."))
	float NarrowBandInner = 0.0f;

	/** The outward dequantization band, in free space (cm), where the maximum code maps to
	    +NarrowBandOuter. It equals the detection band from the bake settings. 0 means unbaked or
	    invalid, which also covers a failed load from the older format and requires a rebake. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Outward (free-space) dequant band in cm. Code 255 maps to +NarrowBandOuter. Equals the bake detection band. 0 means unbaked/invalid (or a load from the old format failed - rebake needed)."))
	float NarrowBandOuter = 0.0f;

	/** The full quantization range (cm), that is the inward plus the outward band, over which the code
	 *  range maps from -Inner to +Outer. 0 means invalid. */
	FORCEINLINE float QuantRange() const { return NarrowBandInner + NarrowBandOuter; }

	/** Whether the bake completed: the byte count matches the resolution multiplied by BytesPerCode and
	 *  the dequantization range is valid. */
	bool IsBaked() const
	{
		const int64 Expected = static_cast<int64>(Resolution.X) * Resolution.Y * Resolution.Z;
		return Expected > 0 && QuantRange() > 0.0f
			&& static_cast<int64>(Distances.Num()) == Expected * BytesPerCode();
	}

	/** Decodes a code into a signed distance (cm, positive outside). Returns 0 when the range or index
	 *  is invalid. It reads 1 or 2 bytes depending on BytesPerCode. */
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
			// Little-endian.
			Code |= static_cast<uint32>(Distances[Base + 1]) << 8;
		}
		const float MaxCodeF = (Bpc >= 2) ? 65535.0f : 255.0f;
		return static_cast<float>(Code) * (Range / MaxCodeF) - NarrowBandInner;
	}

	/** Encodes a signed distance (cm) into an integer code from 0 to MaxCode, clamping it to the band
	 *  from -NBInnerCm to +NBOuterCm. */
	static FORCEINLINE uint32 EncodeCode(float Distance, float NBInnerCm, float NBOuterCm, uint32 MaxCode)
	{
		const float Range = NBInnerCm + NBOuterCm;
		if (Range <= 0.0f)
		{
			// Fallback for an invalid range, which puts it at the midpoint, roughly 0.
			return MaxCode / 2;
		}
		// Map the band from -NBInner..+NBOuter onto 0..1.
		const float T = (Distance + NBInnerCm) / Range;
		return static_cast<uint32>(FMath::Clamp(FMath::RoundToInt(T * static_cast<float>(MaxCode)), 0, static_cast<int32>(MaxCode)));
	}

	/** Encodes a distance and writes it into the blob at the given voxel index, as BytesPerCode
	    little-endian bytes. Out must already be Count * BytesPerCode in size. Byte ranges never overlap
	    between indices, so this is safe to call in parallel. */
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
 * The set of per-bone SDFs for one skeletal mesh. URopeSDFProvider references it to supply colliders
 * each frame. Create it from the Content Browser through its factory, then assign a mesh and bake it
 * from the SDF dock tab.
 */
UCLASS(BlueprintType, meta = (ToolTip = "Per-bone signed distance field set baked from one skeletal mesh. URopeSDFProvider references it to supply a collider per bone each frame. Create it in the Content Browser, then assign a Source Mesh and bake it from the Rope SDF Authoring tab."))
class DYNAMICROPE_API URopeSDFData : public UDataAsset
{
	GENERATED_BODY()

public:
	/** The mesh the SDF was baked from, held as a soft reference so it is not force-loaded at runtime.
	 *  It is used for authoring and validation. */
	UPROPERTY(EditAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Skeletal mesh the SDF was baked from (soft reference — not force-loaded at runtime; used for authoring/validation)."))
	TSoftObjectPtr<USkeletalMesh> SourceMesh;

	/** The per-bone distance volumes, which the provider transforms into world space by their bone
	 *  transforms. */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Per-bone distance volumes. The provider transforms each into world space by its bone transform."))
	TArray<FRopeBoneSDFVolume> BoneVolumes;

	/**
	 * The settings used for the last bake. The authoring panel restores them when a target is loaded,
	 * as the baseline for comparing and adjusting the current result. An asset baked before this field
	 * existed serializes the defaults, so it must be baked once more before the real values are
	 * recorded.
	 */
	UPROPERTY(VisibleAnywhere, Category = "Rope|SDF", meta = (ToolTip = "Settings used for the last bake. The authoring panel restores these when the asset is loaded so you can compare and adjust. Assets baked before this field existed serialize defaults until re-baked."))
	FRopeSDFBakeSettings LastBakeSettings;

	/** Finds a volume by bone name, or nullptr. */
	const FRopeBoneSDFVolume* FindVolume(FName Bone) const;

	/** Whether any volume has been baked. */
	bool HasAnyBakedVolume() const;

	/** A runtime-only unique ID for this asset instance, assigned per load, never reused, and not
	 *  serialized. The GPU SDF cache keys volumes by this ID plus a bone index rather than by raw
	 *  pointer, which prevents sampling stale voxels after an asset is unloaded and its memory reused.
	 *  It is assigned lazily. */
	uint64 GetRuntimeVolumeId() const;

private:
	/** The lazy cache behind GetRuntimeVolumeId, where 0 means unassigned. The const getter fills it in
	 *  on the first call, hence mutable. */
	mutable uint64 RuntimeVolumeId = 0;
};
