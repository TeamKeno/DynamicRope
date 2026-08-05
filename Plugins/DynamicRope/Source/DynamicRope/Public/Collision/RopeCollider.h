// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Collider abstraction. The solver only ever calls IRopeCollider::Query — it never knows whether the
// collider behind it is a capsule, a per-bone SDF, or a world distance field.
//
// [GPU contract — read this before writing a custom collider] The runtime solve is GPU-only. A custom
// collider that implements just the CPU contract (Query / QuerySwept) works in unit tests and in the CPU
// fallback (cook, -nullrhi, or a rope over the node cap), but the GPU step needs one of GetGPUCapsule,
// GetGPUSDF, GetGPUBox or GetGPUConvex. With all four returning false the collider drops out of the GPU
// solve, and the subsystem (PackStepColliders) warns once per session.
// Packing rules: a non-static (skeletal) collider may be a capsule or an SDF; a static one (IsWorldStatic)
// may be a capsule, a box or a convex.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeSimTypes.h"

// Wrap targets are generalized to USceneComponent, so SourceMesh and bone attribution can name a static component too.
class USceneComponent;

/**
 * SDF collider view for the GPU solver: a bone-local distance grid plus the bone-to-world transform, exposed
 * without a runtime type.
 * Distances points into storage the collider or asset owns and is valid for that frame only. VolumeKey is a
 * stable volume identifier, assigned per load and never reused.
 * Distances holds uint8 quantization codes; the consumer dequantizes as d = code × (range/255) - NBInner,
 * where range = NBInner + NBOuter and outer is positive. The inner and outer bands differ, which is why the
 * offset is -NBInner.
 */
struct FRopeSDFColliderView
{
	/** Blob of code bytes, BytesPerCode per voxel, row-major, little-endian. */
	const uint8* Distances = nullptr;

	/** Bytes per voxel (1 = uint8, max 255; 2 = uint16, max 65535). */
	int32        BytesPerCode = 1;

	/** Inner dequantization band (cm). Code 0 maps to -NarrowBandInner. */
	float        NarrowBandInner = 0.0f;

	/** Outer dequantization band (cm). The maximum code maps to +NarrowBandOuter. */
	float        NarrowBandOuter = 0.0f;

	int32        ResX = 0;
	int32        ResY = 0;
	int32        ResZ = 0;

	/** Bone-local grid origin (LocalBounds.Min) and size. */
	FVector      LocalMin = FVector::ZeroVector;
	FVector      LocalSize = FVector::ZeroVector;

	FTransform   BoneToWorld = FTransform::Identity;

	/** Previous frame's bone transform, for GPU CCD and for surface-velocity drag. */
	FTransform   PrevBoneToWorld = FTransform::Identity;

	/** 1 / frame dt, so surface velocity is (curr - prev) × InvDeltaTime. 0 means static. */
	float        InvDeltaTime = 0.0f;

	/** Stable volume identifier (URopeSDFData::GetRuntimeVolumeId() << 16 | bone index). Assigned per load and
	 *  never reused, so the GPU SDF cache cannot sample the wrong volume after an unload and reload. 0 = unset. */
	uint64       VolumeKey = 0;
};

/**
 * Parameters for a swept (continuous) collision query: find the first contact while the node travels from
 * WorldStart to WorldEnd within this substep. A moving collider spreads its frame motion (prev → curr) across
 * the substeps through SubAlpha0/1, so what is measured is the node's motion *relative* to the bone; a static
 * collider ignores the alphas. The point is that a fast bone overtaking a resting rope catches it on the
 * approaching side instead of shoving it through from behind.
 */
struct FRopeSweptQuery
{
	/** Node position at the start and end of the substep (PrevPos → Pos). */
	FVector WorldStart = FVector::ZeroVector;
	FVector WorldEnd   = FVector::ZeroVector;

	/** Rope thickness — the query radius. */
	float   NodeRadius = 0.0f;

	/** Sample spacing (cm). */
	float   SweepStep  = 2.0f;

	/** Cap on samples for this segment. */
	int32   MaxSamples = 16;

	/**
	 * Sub-pose of a moving collider for this substep. The solver blends the prev and curr transforms it got
	 * from GetFrameMotion by alpha, precomputed once per collider outside the node loop so the blend is not
	 * redone per node. With bUseSubPose false the collider has a single, current pose — a still bone, or one
	 * that is not an SDF. Nothing here mutates the shared collider, so a parallel solve stays safe.
	 */
	bool       bUseSubPose  = false;
	FTransform SubPoseStart = FTransform::Identity;
	FTransform SubPoseEnd   = FTransform::Identity;

	/**
	 * Where this substep falls within the frame's motion (0 = previous frame, 1 = current). The solver always
	 * fills these. A collider with no rigid transform — a capsule, whose two joints move independently — uses
	 * them to interpolate its own prev state directly, as the transform-free counterpart to SubPoseStart/End.
	 */
	float SubAlpha0 = 0.0f;
	float SubAlpha1 = 1.0f;
};

/** Result of projecting onto a collider surface. Unlike a contact query, it describes the surface point nearest a given point. */
struct FRopeSurfaceProjection
{
	bool bHit = false;
	FVector SurfacePoint = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	float Distance = 0.0f;
	FName Bone = NAME_None;
	const USceneComponent* SourceMesh = nullptr;
};

/**
 * Shared separation guard for QuerySwept — a world-space generalization of what the SDF QuerySwept does in
 * local frames. When a node starts the substep inside the contact skin and ends *outside* the surface, it must
 * not be re-pinned; otherwise the contact node is dragged back to where the substep started and never falls,
 * so it rides along on a moving bone, or a low-tension end node sticks to it.
 * It returns true only when the node starts in contact, ends outside the skin, and its displacement relative
 * to the contact material point is positive along the outward normal at the start.
 * Penetration — moving inward and coming out the far side of the skin — is deliberately not filtered here,
 * because the sweep catches that at the first contact instead.
 */
namespace RopeCollision
{
	/**
	 * Number of point samples needed to keep the uncapped spacing at or below SweepStep while preserving
	 * MaxSamples as the existing point-query budget. Using floor here makes a path just shorter than
	 * 2*SweepStep use only its two endpoints, leaving almost twice the configured gap between them.
	 */
	inline int32 SweptSampleCount(double Travel, float SweepStep, int32 MaxSamples)
	{
		const double Step = FMath::Max(static_cast<double>(SweepStep), 0.1);
		return FMath::Clamp(1 + FMath::CeilToInt(Travel / Step), 1, FMath::Max(1, MaxSamples));
	}

	inline bool IsSweptSeparating(const FVector& NodeStart, const FVector& NodeEnd,
		const FVector& ContactPointStart, const FVector& ContactPointEnd,
		const FVector& StartOutwardNormal, bool bStartInContact, bool bEndInContact)
	{
		if (!bStartInContact || bEndInContact)
		{
			return false;
		}
		const FVector RelativeDisplacement = (NodeEnd - NodeStart) - (ContactPointEnd - ContactPointStart);
		return FVector::DotProduct(RelativeDisplacement, StartOutwardNormal) > 0.0f;
	}
}

/** The abstract collider the rope solver queries. */
class DYNAMICROPE_API IRopeCollider
{
public:
	virtual ~IRopeCollider() = default;

	/**
	 * Nearest-surface query for the rope node's sphere (centre WorldPos, radius Radius).
	 * Fill FRopeContact exactly as that struct's frozen contract requires. It must be const and free of side
	 * effects, since it runs per node × substep × iteration. Radius 0 is valid — that is the solver push-out path.
	 */
	virtual FRopeContact Query(const FVector& WorldPos, float Radius) const = 0;

	/**
	 * Surface projection query. Unlike the collision query it looks for the surface point *nearest* WorldPos
	 * rather than one overlapping it, and may return false when nothing is within MaxDistance. Used to keep a
	 * path stuck to a surface, as the wrapping path build does.
	 */
	virtual FRopeSurfaceProjection ProjectToSurface(const FVector& WorldPos, float MaxDistance) const
	{
		FRopeSurfaceProjection Projection;
		const FRopeContact Contact = Query(WorldPos, MaxDistance);
		if (!Contact.bHit)
		{
			return Projection;
		}

		Projection.bHit = true;
		Projection.SurfacePoint = Contact.SurfacePoint;
		Projection.Normal = Contact.Normal;
		Projection.Distance = FVector::Dist(WorldPos, Contact.SurfacePoint);
		Projection.Bone = Contact.Bone;
		Projection.SourceMesh = Contact.SourceMesh;
		return Projection;
	}

	/**
	 * Swept query: find the first contact along the node's substep path, accounting for a moving collider's
	 * relative motion.
	 * The default implementation is a static fallback — it ignores collider motion and samples the straight
	 * WorldStart → WorldEnd line against the current pose. A moving SDF collider overrides it to account for
	 * the relative motion.
	 * On a first contact it returns bHit = true with OutHitWorldPos as the node's world position there, and the
	 * solver places the node at OutHitWorldPos + Normal × Penetration.
	 */
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
	{
		// Separation guard (RopeCollision::IsSweptSeparating): do not re-pin a contact that starts inside the
		// skin and is leaving the surface. This is the static fallback, so collider motion is zero and the
		// contact material point does not move, making the relative displacement just the node's.
		// The end-point query only runs once a start contact exists (short-circuit).
		const FRopeContact Start = Query(Q.WorldStart, Q.NodeRadius);
		if (Start.bHit && RopeCollision::IsSweptSeparating(Q.WorldStart, Q.WorldEnd,
			Start.SurfacePoint, Start.SurfacePoint, Start.Normal,
			/*bStartInContact*/ true, /*bEndInContact*/ Query(Q.WorldEnd, Q.NodeRadius).bHit))
		{
			OutHitWorldPos = Q.WorldEnd;
			return FRopeContact();
		}
		const double L = FVector::Dist(Q.WorldStart, Q.WorldEnd);
		const int32 NumSamples = RopeCollision::SweptSampleCount(L, Q.SweepStep, Q.MaxSamples);
		for (int32 k = 0; k < NumSamples; ++k)
		{
			const double T = (NumSamples <= 1) ? 1.0 : static_cast<double>(k) / static_cast<double>(NumSamples - 1);
			const FVector P = FMath::Lerp(Q.WorldStart, Q.WorldEnd, T);
			const FRopeContact Contact = Query(P, Q.NodeRadius);
			if (Contact.bHit)
			{
				OutHitWorldPos = P;
				return Contact;
			}
		}
		return FRopeContact();
	}

	/** World-space bounds for broad-phase culling. */
	virtual FBox GetWorldBounds() const = 0;

	/**
	 * Is this collider static world geometry? Default false, meaning skeletal or otherwise dynamic.
	 * A static collider cannot be wrapped and must be kept out of the contact detection pipeline: detection
	 * keeps only the single deepest contact per node, so a wall contact masking a bone contact would silently
	 * cost that node its wrap capture. The filter runs in PackStepColliders' two-pass packing on the GPU, and
	 * in the Flight detector's input on the CPU. Static colliders still take part in the solve push-out normally.
	 */
	virtual bool IsWorldStatic() const { return false; }

	/**
	 * For the GPU solver: if this collider is an analytic capsule, fill the world-space segment (A-B) and
	 * radius and return true. Default false. RTTI is off, so this virtual accessor is how a capsule is
	 * identified instead of a dynamic_cast. SDF and other colliders stay off the GPU capsule path and are
	 * handled separately as a Texture3D SDF.
	 */
	virtual bool GetGPUCapsule(FVector& OutA, FVector& OutB, float& OutRadius) const { return false; }

	/**
	 * A GPU capsule's frame motion: previous endpoints plus 1 / frame dt. Meaningful only where GetGPUCapsule
	 * returns true. Default false, meaning static, and the caller falls back to prev = current with InvDt = 0.
	 * A capsule has no rigid transform — its two joints move independently — so it passes the endpoint pair
	 * directly rather than going through GetFrameMotion. The GPU uses it for surface-velocity drag and for
	 * substep relative-motion CCD, mirroring the SDF's PrevBoneToWorld and InvDeltaTime.
	 */
	virtual bool GetGPUCapsuleMotion(FVector& OutPrevA, FVector& OutPrevB, float& OutInvDeltaTime) const { return false; }

	/**
	 * For the GPU solver: if this collider is a per-bone SDF, fill the grid and transform view and return true.
	 * Default false. Like the capsule accessor, this identifies the collider without RTTI, and it is mutually
	 * exclusive with GetGPUCapsule.
	 */
	virtual bool GetGPUSDF(FRopeSDFColliderView& OutView) const { return false; }

	/**
	 * For the GPU solver: if this collider is an analytic box (OBB), fill the world-space centre, rotation and
	 * half-extents and return true. Default false, and mutually exclusive with GetGPUCapsule and GetGPUSDF —
	 * the same RTTI-free identification pattern.
	 */
	virtual bool GetGPUBox(FVector& OutCenter, FQuat& OutRot, FVector& OutHalfExtents) const { return false; }

	/**
	 * A GPU box's frame motion: previous centre and rotation plus 1 / frame dt. Meaningful only where GetGPUBox
	 * returns true. Default false, meaning static, and the caller falls back to prev = current with InvDt = 0.
	 * It is what lets a moving static body — a platform, a door — drag the rope through surface velocity, and
	 * what stops it tunnelling under substep CCD. Mirrors the capsule's GetGPUCapsuleMotion.
	 */
	virtual bool GetGPUBoxMotion(FVector& OutPrevCenter, FQuat& OutPrevRot, float& OutInvDeltaTime) const { return false; }

	/**
	 * For the GPU solver: if this collider is an analytic convex, fill the body-local plane set (unit outward
	 * normals, with PlaneDot(p) = dot(N, p) - W, before any scale or rigid transform), the local AABB, and the
	 * body's rigid transform — current rotation and translation, the previous pair, and 1 / frame dt — then
	 * return true. Default false.
	 * A world plane is the local plane composed with that rigid transform; a moving body's prev transform gives
	 * surface velocity and substep CCD, and a static one passes prev = curr with InvDt = 0. OutLocalPlanes is a
	 * view into storage the collider owns, valid for that frame.
	 */
	virtual bool GetGPUConvex(TConstArrayView<FPlane>& OutLocalPlanes, FBox& OutLocalBounds,
		FQuat& OutRot, FVector& OutTrans, FQuat& OutPrevRot, FVector& OutPrevTrans, float& OutInvDeltaTime) const
	{
		return false;
	}

	/**
	 * Fill this frame's motion for a moving collider — the prev → curr world transform — and return true.
	 * Default false, meaning static or motionless. The solver uses it to compute each substep's sub-pose for a
	 * swept collision once, outside the node loop, hoisting the relative-motion CCD setup.
	 * It must be const and read-only: colliders are shared between ropes and solved in parallel.
	 */
	virtual bool GetFrameMotion(FTransform& OutPrev, FTransform& OutCurr) const { return false; }

	/**
	 * Attribution for GPU contact detection: which bone and mesh this collider belongs to. The GPU emits only
	 * an index, and the caller turns that index back into a (bone, mesh) pair with this. It must report the
	 * same values as FRopeContact's Bone and SourceMesh, since both feed the same decision pipeline.
	 * Defaults to None and null.
	 */
	virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const
	{
		OutBone = NAME_None;
		OutMesh = nullptr;
	}
};

/**
 * Analytic capsule (a swept-sphere segment). The v1 bone collider: it and the per-bone SDF (FRopeSDFCollider)
 * are chosen per provider behind the IRopeCollider interface, and the solver cannot tell which it has.
 */
class DYNAMICROPE_API FCapsuleCollider : public IRopeCollider
{
public:
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;
	FName   Bone = NAME_None;

	/**
	 * Previous endpoints plus 1 / frame dt. The provider caches and fills the previous endpoints per bone, as
	 * the counterpart to the SDF provider's PrevBoneToWorld. InvDeltaTime 0, the default, means a static
	 * capsule: prev is ignored.
	 * The contact material point is identified by the segment parameter t, so its previous position is
	 * Lerp(PrevA, PrevB, t). Spin about the capsule's own axis cannot be represented this way, but on a bone
	 * capsule that component is negligible.
	 */
	FVector PrevA = FVector::ZeroVector;
	FVector PrevB = FVector::ZeroVector;
	float   InvDeltaTime = 0.0f;

	/**
	 * The mesh owning this capsule's bone. Carrying it on the contact is what lets a wrap cross actors and
	 * follow the *right* mesh — the one that owns the captured bone. The type is USceneComponent so static
	 * targets fit the same field; a capsule only ever passes a skeletal one.
	 */
	const USceneComponent* SourceMesh = nullptr;

	FCapsuleCollider() = default;
	FCapsuleCollider(const FVector& InA, const FVector& InB, float InRadius, FName InBone = NAME_None,
		const USceneComponent* InSourceMesh = nullptr)
		: A(InA), B(InB), Radius(InRadius), Bone(InBone), PrevA(InA), PrevB(InB), SourceMesh(InSourceMesh) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FRopeSurfaceProjection ProjectToSurface(const FVector& WorldPos, float MaxDistance) const override;
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const override;
	virtual FBox GetWorldBounds() const override;
	virtual bool GetGPUCapsule(FVector& OutA, FVector& OutB, float& OutRadius) const override;
	virtual bool GetGPUCapsuleMotion(FVector& OutPrevA, FVector& OutPrevB, float& OutInvDeltaTime) const override;
	virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const override
	{
		OutBone = Bone;
		OutMesh = SourceMesh;
	}
};
