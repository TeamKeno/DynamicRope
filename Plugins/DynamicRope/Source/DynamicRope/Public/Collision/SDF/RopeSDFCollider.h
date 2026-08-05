// Copyright 2026 TeamKeno. All Rights Reserved.
//
// A collider that wraps a single per-bone SDF volume as an IRopeCollider. It transforms the
// bone-local grid by the bone's current world transform to answer the solver's node queries. It is
// the SDF counterpart of FCapsuleCollider and obeys the same frozen FRopeContact contract: the normal
// is an outward unit vector, the penetration is measured against the query radius, and the bone is
// never None.

#pragma once

#include "CoreMinimal.h"
#include "Collision/RopeCollider.h"

struct FRopeBoneSDFVolume;
// Wrap target abstraction: the attributed mesh is generalized to USceneComponent, although the SDF
// path only ever passes skeletal ones.
class USceneComponent;

/** An analytic collider over one bone-local SDF volume, presenting the same interface as the capsule
 *  collider. */
class DYNAMICROPE_API FRopeSDFCollider : public IRopeCollider
{
public:
	// The bone-local distance grid. The pointer is owned by the provider and is valid for the duration
	// of that frame's solve.
	const FRopeBoneSDFVolume* Volume = nullptr;

	// The bone-to-world transform that places the grid in the world, refreshed from the mesh each
	// frame.
	FTransform BoneToWorld = FTransform::Identity;

	// The previous frame's bone-to-world transform, used to derive surface velocity, which produces
	// drag. On the first frame it equals BoneToWorld, giving zero velocity.
	FTransform PrevBoneToWorld = FTransform::Identity;

	// The reciprocal of the frame delta, which converts surface displacement into a velocity in cm/s.
	// 0 gives zero surface velocity, treating the collider as static.
	float InvDeltaTime = 0.0f;

	// The bone this volume belongs to, propagated to FRopeContact::Bone.
	FName Bone = NAME_None;

	// The mesh owning the bone, passed through the contact so cross-actor follow works. Typed as
	// USceneComponent for generality.
	const USceneComponent* SourceMesh = nullptr;

	// A stable identifier for the volume, computed by the provider from the URopeSDFData runtime ID
	// and the bone index. GetGPUSDF passes it through on the view, where it is the GPU SDF cache key.
	// Using it instead of a raw pointer prevents mis-sampling when an asset is unloaded and its address
	// is reused.
	uint64 VolumeKey = 0;

	FRopeSDFCollider() = default;
	FRopeSDFCollider(const FRopeBoneSDFVolume* InVolume, const FTransform& InBoneToWorld,
		const FTransform& InPrevBoneToWorld, float InInvDeltaTime,
		FName InBone, const USceneComponent* InSourceMesh, uint64 InVolumeKey = 0)
		: Volume(InVolume), BoneToWorld(InBoneToWorld), PrevBoneToWorld(InPrevBoneToWorld)
		, InvDeltaTime(InInvDeltaTime), Bone(InBone), SourceMesh(InSourceMesh), VolumeKey(InVolumeKey) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FRopeSurfaceProjection ProjectToSurface(const FVector& WorldPos, float MaxDistance) const override;
	// The relative-motion swept query: it interpolates the previous and current bone transforms by the
	// substep alpha and samples the node's path in the collider's local relative frame, so a moving bone
	// catches the node on its leading face. It also fills in the surface velocity.
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const override;
	virtual FBox GetWorldBounds() const override;
	virtual bool GetGPUSDF(FRopeSDFColliderView& OutView) const override;
	// This frame's bone motion, from the previous transform to the current one. The solver uses it to
	// hoist the substep sub-poses.
	virtual bool GetFrameMotion(FTransform& OutPrev, FTransform& OutCurr) const override
	{
		OutPrev = PrevBoneToWorld; OutCurr = BoneToWorld; return true;
	}
	virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const override
	{
		OutBone = Bone;
		OutMesh = SourceMesh;
	}
};
