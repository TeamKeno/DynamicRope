// Copyright Epic Games, Inc. All Rights Reserved.
//
// Analytic colliders for static world geometry, that is the simple collision of static bodies. The
// global distance field is a voxel clipmap, so its corners are eroded by roughly the voxel size and a
// rope slides over a box corner and passes through it. The box collider here is an analytic clamp
// query, which gives an exact diagonal normal at corners and edges.
// URopeStaticBodyProvider extracts these from the UBodySetup of nearby static bodies and serves them,
// which demotes the global distance field to a far-field fallback for landscapes and huge meshes
// with no simple collision.

#pragma once

#include "CoreMinimal.h"
#include "Collision/RopeCollider.h"

/**
 * An analytic oriented box collider. By default it is static world geometry: it has no frame motion,
 * so its surface velocity is zero, and with no bone and no source mesh IsWorldStatic() is true, which
 * excludes it from detection and leaves it push-out only.
 * Filling in a virtual bone and a source mesh makes IsWorldStatic() false, so it takes part in
 * detection and carries that attribution on the contact, which wraps it through the existing
 * contact-to-wrap path while obeying the frozen contract.
 */
class DYNAMICROPE_API FRopeBoxCollider : public IRopeCollider
{
public:
	/** The box centre and rotation in world space, and its local half extents with scale already
	 *  applied. */
	FVector Center = FVector::ZeroVector;
	FQuat   Rot = FQuat::Identity;
	FVector HalfExtents = FVector::ZeroVector;

	/**
	 * For the surface velocity of a dynamic body: the previous frame's centre and rotation, plus the
	 * reciprocal frame delta. Providers fill these in for moving bodies.
	 * An InvDeltaTime of 0, the default, means static, in which case the previous values are ignored.
	 * The scale is assumed not to change between frames.
	 */
	FVector PrevCenter = FVector::ZeroVector;
	FQuat   PrevRot = FQuat::Identity;
	float   InvDeltaTime = 0.0f;

	/**
	 * A wrappable box has a non-None bone, which is a virtual one, plus a source mesh naming the target
	 * component, and takes part in detection. The defaults leave it static world geometry, push-out
	 * only and excluded from detection.
	 */
	FName Bone = NAME_None;
	const USceneComponent* SourceMesh = nullptr;

	FRopeBoxCollider() = default;
	FRopeBoxCollider(const FVector& InCenter, const FQuat& InRot, const FVector& InHalfExtents)
		: Center(InCenter), Rot(InRot), HalfExtents(InHalfExtents), PrevCenter(InCenter), PrevRot(InRot) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const override;
	virtual FBox GetWorldBounds() const override;

	/** A virtual bone makes this a wrap target and therefore non-static, so it is detected. Without one
	 *  it is static world geometry, push-out only and excluded from detection. */
	virtual bool IsWorldStatic() const override { return Bone.IsNone(); }
	virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const override
	{
		OutBone = Bone;
		OutMesh = SourceMesh;
	}
	virtual bool GetGPUBox(FVector& OutCenter, FQuat& OutRot, FVector& OutHalfExtents) const override
	{
		OutCenter = Center;
		OutRot = Rot;
		OutHalfExtents = HalfExtents;
		return true;
	}
	virtual bool GetGPUBoxMotion(FVector& OutPrevCenter, FQuat& OutPrevRot, float& OutInvDeltaTime) const override
	{
		if (InvDeltaTime <= 0.0f)
		{
			// Static, so the caller falls back to a previous transform equal to the current one and a
			// reciprocal delta of 0.
			return false;
		}
		OutPrevCenter = PrevCenter;
		OutPrevRot = PrevRot;
		OutInvDeltaTime = InvDeltaTime;
		return true;
	}
	// ProjectToSurface keeps the default implementation.
};

/**
 * A static capsule or sphere, where a sphere is a degenerate capsule with coincident endpoints. It
 * reuses FCapsuleCollider's queries, but by default, with no bone, IsWorldStatic() is true so it is
 * excluded from detection and is push-out only.
 * Filling in a virtual bone and a source mesh makes it wrappable and detected, on the same convention
 * as FRopeBoxCollider; that is how the full-set mode of URopeWrapTargetComponent serves sphyl and
 * sphere elements, with the GPU attribution and surface velocity handled by the parent machinery
 * unchanged. For a dynamic body, fill in the reciprocal delta and the previous endpoints as well.
 */
class DYNAMICROPE_API FRopeStaticCapsuleCollider : public FCapsuleCollider
{
public:
	using FCapsuleCollider::FCapsuleCollider;

	/** The same convention as the box: a virtual bone makes it a wrap target and therefore detected,
	 *  and without one it is static world geometry, push-out only. */
	virtual bool IsWorldStatic() const override { return Bone.IsNone(); }
};

/**
 * An analytic convex collider, expressed as a set of planes, for static world geometry only, that is
 * the convex simple collision of static bodies.
 * The query takes the maximum plane: the signed distance to the most violated plane is used as the
 * penetration response. That is exact inside the shape, and outside it underestimates the distance
 * near edges and vertices, because an infinite plane is nearer than the finite edge is, so contact
 * engages slightly early. That is conservative and free of tunnelling, which is enough for a penalty
 * response. Computing the exact closest point on an edge or vertex, as the box does, requires
 * adjacency information and is expensive, so this approximation is the standard choice for static
 * world collision.
 * Under the frozen FRopeContact contract it is non-skeletal, so its bone is None, its source mesh is
 * null and its surface velocity is zero.
 */
class DYNAMICROPE_API FRopeConvexCollider : public IRopeCollider
{
public:
	/**
	 * The body-local planes, with outward unit normals and scale applied but no rigid transform. With
	 * PlaneDot(p) = dot(N, p) - W, a point inside is below zero on every plane.
	 * A world plane is the local plane composed with the rigid transform of Rot and Trans. Only that
	 * rigid transform changes between frames; the local planes are invariant, assuming the scale does
	 * not change.
	 */
	TArray<FPlane> LocalPlanes;

	/** The body-local AABB, used to cull queries. */
	FBox LocalBounds = FBox(ForceInit);

	/** The body's rigid transform, that is the component's rotation and translation, both current and
	 *  from the previous frame for dynamic surface velocity and continuous collision. An InvDeltaTime
	 *  of 0 means static. */
	FQuat   Rot = FQuat::Identity;
	FVector Trans = FVector::ZeroVector;
	FQuat   PrevRot = FQuat::Identity;
	FVector PrevTrans = FVector::ZeroVector;
	float   InvDeltaTime = 0.0f;

	FRopeConvexCollider() = default;

	/** A convenience constructor for the static case: local planes, local bounds and a rigid transform
	 *  that defaults to the identity, making world space equal local space. For tests and static
	 *  paths. */
	FRopeConvexCollider(TArray<FPlane>&& InLocalPlanes, const FBox& InLocalBounds,
		const FQuat& InRot = FQuat::Identity, const FVector& InTrans = FVector::ZeroVector)
		: LocalPlanes(MoveTemp(InLocalPlanes)), LocalBounds(InLocalBounds)
		, Rot(InRot), Trans(InTrans), PrevRot(InRot), PrevTrans(InTrans) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const override;
	virtual FBox GetWorldBounds() const override;
	virtual bool IsWorldStatic() const override { return true; }
	virtual bool GetGPUConvex(TConstArrayView<FPlane>& OutLocalPlanes, FBox& OutLocalBounds,
		FQuat& OutRot, FVector& OutTrans, FQuat& OutPrevRot, FVector& OutPrevTrans, float& OutInvDeltaTime) const override
	{
		OutLocalPlanes = LocalPlanes;
		OutLocalBounds = LocalBounds;
		OutRot = Rot; OutTrans = Trans;
		OutPrevRot = PrevRot; OutPrevTrans = PrevTrans;
		OutInvDeltaTime = InvDeltaTime;
		return true;
	}
	// ProjectToSurface keeps the default implementation.
};
