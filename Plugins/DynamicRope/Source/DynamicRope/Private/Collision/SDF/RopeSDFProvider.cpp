// Copyright 2026 TeamKeno. All Rights Reserved.

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
	// The candidates for the bone filter dropdown: the bone names actually baked into the SDF data, rather than the whole skeleton.
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
	 * A non-uniform scale warning, issued once per bone. An SDF query approximates the conversion between local and
	 * world distances with a single scalar, the largest component of the scale, on both the CPU in
	 * FRopeSDFCollider::Query and the GPU in RopeQuerySDFWorld, so a scale that differs per axis leaves the contact
	 * band and the penetration depth inconsistent across axes. The approximation itself is the intended contract, but
	 * with no signal at all there was nothing to go on when tracking down why the rope sinks into one mesh in
	 * particular. Negative scales, meaning mirroring, are now properly supported: what is examined here is the ratio
	 * between the components alone, so a uniform mirror of minus one does not warn.
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
			TEXT("The scale of SDF collider bone '%s' is non-uniform (%s). SDF distance conversion approximates with ")
			TEXT("the largest scale component, so the contact band and the penetration differ across axes. A uniform scale is recommended."),
			*Bone.ToString(), *BoneToWorld.GetScale3D().ToCompactString());
#endif
	}
}

void URopeSDFProvider::RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt)
{
	// Per-rope culling is the responsibility of the solver's collider AABB broad phase, so every baked volume is built
	// here with no culling against the rope bounds: building a collider is cheap and the solver culls the expensive
	// queries. HasColliderData already guarantees the SDF data is valid.
	Colliders.Reset();

	for (const FRopeBoneSDFVolume& Volume : SDFData->BoneVolumes)
	{
		if (Volume.Bone.IsNone() || !Volume.IsBaked())
		{
			// Unbaked or invalid volumes are skipped.
			continue;
		}

		// The runtime bone filter, which leaves the bake as it is and hides the collider from exposure alone, for isolating a problem while debugging.
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
		// The previous frame's transform, defaulting to the current one, which gives zero velocity on the first frame. It is updated for the next frame after the lookup.
		const FTransform* PrevPtr = PrevBoneToWorld.Find(Volume.Bone);
		const FTransform PrevXform = PrevPtr ? *PrevPtr : BoneToWorld;
		PrevBoneToWorld.Add(Volume.Bone, BoneToWorld);
		// The stable volume key is the asset's runtime identifier, assigned per load, shifted left by sixteen and combined with the bone index. Using it rather than a raw pointer prevents mis-sampling after an unload.
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
