// Copyright Epic Games, Inc. All Rights Reserved.
//
// Collider abstraction. The solver just queries the IRopeCollider, whether it is a capsule or a per-bone SDF.
// Or I have no idea whether it is a world distance field.
//
// [GPU contract — custom collider caution] Runtime solve is GPU single path. CPU contract (Query/QuerySwept) only
// The custom collider implemented works in unit tests/CPU fallback (cook·-nullrhi·node number exceeded), but GPU
// step requires one of GetGPUCapsule / GetGPUSDF / GetGPUBox / GetGPUConvex to be implemented — all four.
// If false, it is excluded from GPU solve and the subsystem (PackStepColliders) leaves a warning once per session.
// Packing rules: non-static (skeletal) only capsule/SDF, static (IsWorldStatic) only capsule/box/convex.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeSimTypes.h"

// wrap target abstraction (Decision 0): Generalize SourceMesh/attribution mesh to USceneComponent (versus static opt-in).
class USceneComponent;

/**
 * SDF collider view for GPU solver (M3). Bone local distance grid + Bone → world transform is exposed without runtime type.
 * Distances is a collider/asset owning pointer (valid for that frame). VolumeKey is a volume stable identifier (given per load, never reused).
 * Distances are uint8 quantization code — consumer dequant(d = code*(range/255) - NBInner,
 * range = NBInner+NBOuter, outer +). The inner/outer bands are different, so the offset is -NBInner.
 */
struct FRopeSDFColliderView
{
	/** Blob of code bytes (BytesPerCode per voxel, row first). Little endian.*/
	const uint8* Distances = nullptr;

	/** Bytes per voxel (1=uint8 max255, 2=uint16 max65535).*/
	int32        BytesPerCode = 1;

	/** Inner dequant band (cm). Code 0 → -NarrowBandInner.*/
	float        NarrowBandInner = 0.0f;

	/** Outer dequant band (cm). Code max → +NarrowBandOuter.*/
	float        NarrowBandOuter = 0.0f;

	int32        ResX = 0;
	int32        ResY = 0;
	int32        ResZ = 0;

	/** bone local grid origin (LocalBounds.Min) and size.*/
	FVector      LocalMin = FVector::ZeroVector;
	FVector      LocalSize = FVector::ZeroVector;

	FTransform   BoneToWorld = FTransform::Identity;

	/** previous frame bone transform (for GPU CCD/surfacevelocity dragging).*/
	FTransform   PrevBoneToWorld = FTransform::Identity;

	/** 1/framedt(surface velocity = (curr-prev)*InvDeltaTime). If 0, static.*/
	float        InvDeltaTime = 0.0f;

	/** Volume stable identifier (URopeSDFData::GetRuntimeVolumeId()<<16 | bone index). No grant/reuse for each load →
	 *  The GPU SDF cache does not sample incorrectly even when unloaded and then reused. 0=Not set.*/
	uint64       VolumeKey = 0;
};

/**
 * Swept (continuous) collision query parameters. While the node moves from WorldStart->WorldEnd in this substep
 * Find the first contact. The moving collider distributes the frame motion (prev->curr) to the substep with SubAlpha0/1.
 * node-collider relative motion is bone (stationary collider ignores alpha). When a fast bone overtakes a stationary rope
 * The purpose is to grab it from the approach (front) side instead of pushing it through the back.
 */
struct FRopeSweptQuery
{
	/** Substep start/end node location (PrevPos → Pos).*/
	FVector WorldStart = FVector::ZeroVector;
	FVector WorldEnd   = FVector::ZeroVector;

	/** rope thickness (query radius).*/
	float   NodeRadius = 0.0f;

	/** Sample interval (cm).*/
	float   SweepStep  = 2.0f;

	/** sample cap per section.*/
	int32   MaxSamples = 16;

	/**
	 * Sub-pose for this substep of the moving collider. The solver blends the prev/curr received through GetFrameMotion into alpha.
	 * Precalculate once per collider (hoisting outside the node loop → prevent blend recalculation for each node). If bUseSubPose=false
	 * The collider is a bone (stationary bone/non-SDF) with a single (current) pose. Safe for parallel solving as it does not mutate shared colliders.
	 */
	bool       bUseSubPose  = false;
	FTransform SubPoseStart = FTransform::Identity;
	FTransform SubPoseEnd   = FTransform::Identity;

	/**
	 * Frame motion section ratio of this substep (0=previous frame, 1=current frame). The solver always fills in.
	 * A collider (capsule — two joints move separately) without rigid transform uses its own prev state instead of sub-pose.
	 * Used for direct interpolation (transform-Free counterpart of SubPoseStart/End).
	 */
	float SubAlpha0 = 0.0f;
	float SubAlpha1 = 1.0f;
};

/** collider surface projection result. Unlike contact check, it describes the surface point closest to a given point.*/
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
 * QuerySwept separation guard shared check (a world-space generalization of what SDF QuerySwept does with local frames). node contacts
 * If you are starting inside the skin and exiting *outside* the surface, do not re-pin — otherwise the contact node will
 * It returns to the starting point of the substep and never falls (it is dragged along the moving bone or the end node of low tension is attached).
 * Only when start contact + end point skin outside + relative displacement (node movement - contact material point movement) is positive in the start outside normal direction
 * true. Penetration (inward movement → outside the skin on the other side) is not filtered out here because the sweep catches it at the first contact.
 */
namespace RopeCollision
{
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

/** The abstract collider that the rope solver queries.*/
class DYNAMICROPE_API IRopeCollider
{
public:
	virtual ~IRopeCollider() = default;

	/**
	 * Nearest surface query for the rope node sphere (center WorldPos, radius Radius).
	 * Fill the FRopeContact according to the FROZEN contract of the corresponding struct. Must be const / no side effects
	 * (called every node x substep x iteration). Radius == 0 is also valid (solver push-out path).
	 */
	virtual FRopeContact Query(const FVector& WorldPos, float Radius) const = 0;

	/**
	 * Query dedicated to surface projection. Unlike collision Query, it searches for surface points that are close to WorldPos rather than “overlapping.”
	 * If it is farther than MaxDistance, false can be returned. It is used to continuously attach to the surface, such as creating a Wrapping path.
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
	 * Swept query: Finds the first contact along the node's substep path (+ relative motion of the moving collider).
	 * The default implementation is a static fallback — ignores collider motion (alpha) and takes the WorldStart->WorldEnd straight line to the current pose.
	 * sample (same as existing solver operation). A moving SDF collider overrides this to reflect relative motion.
	 * At first contact, bHit=true, OutHitWorldPos = node world location of the contact point. solver
	 * Place the node with OutHitWorldPos + Normal*Penetration.
	 */
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
	{
		// Separation guard (RopeCollision::IsSweptSeparating): Re-pin if the contact starts inside the skin and is separating outside the surface.
		// Do not. Because it is a static fallback, collider motion is 0 → contact material point movement is 0 (same point transfer, relative displacement=nodedisplacement).
		// Endpoint query starts only when contact occurs (shortened evaluation).
		const FRopeContact Start = Query(Q.WorldStart, Q.NodeRadius);
		if (Start.bHit && RopeCollision::IsSweptSeparating(Q.WorldStart, Q.WorldEnd,
			Start.SurfacePoint, Start.SurfacePoint, Start.Normal,
			/*bStartInContact*/ true, /*bEndInContact*/ Query(Q.WorldEnd, Q.NodeRadius).bHit))
		{
			OutHitWorldPos = Q.WorldEnd;
			return FRopeContact();
		}
		const double L = FVector::Dist(Q.WorldStart, Q.WorldEnd);
		const int32 NumSamples = FMath::Clamp(1 + FMath::FloorToInt(L / FMath::Max(Q.SweepStep, 0.1f)), 1, FMath::Max(1, Q.MaxSamples));
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

	/** world space bounds for broad-phase culling.*/
	virtual FBox GetWorldBounds() const = 0;

	/**
	 * Whether this collider is static world geometry (static body). Default false (skeletal/dynamic).
	 * static collider is not eligible for Wrapping and should be excluded from the contact detection (detect) pipeline —
	 * detect leaves only the one deepest contact per node, so if the wall contact obscures the bone contact, the node's
	 * wrap capture fails quietly (2-pass packing of PackStepColliders on GPU, Flight detector on CPU)
	 * input filter bones this flag). Participates normally in solve (push-out).
	 */
	virtual bool IsWorldStatic() const { return false; }

	/**
	 * For GPU solver (M2): If this collider is an analytic capsule, fill the world space segment (A-B) and radius and return true.
	 * Default is false (not supported) — RTTI is turned off, so the capsule is identified with this virtual accessor instead of dynamic_cast.
	 * SDF/other colliders are excluded from the GPU capsule path (processed separately as Texture3D SDF in M3).
	 */
	virtual bool GetGPUCapsule(FVector& OutA, FVector& OutB, float& OutRadius) const { return false; }

	/**
	 * Frame motion of GPU capsule: previous frame endpoint + 1/framedt. Only colliders with GetGPUCapsule=true are meaningful.
	 * Default is false (static) — caller falls back to prev=current endpoint, InvDt=0. Capsule does not have rigid transform.
	 * (Two joint points move separately) Pass the end point pair directly instead of GetFrameMotion. GPU supports surface velocity (drag) and
	 * substep Relative motion Write to CCD (SDF's PrevBoneToWorld/InvDeltaTime correspondence).
	 */
	virtual bool GetGPUCapsuleMotion(FVector& OutPrevA, FVector& OutPrevB, float& OutInvDeltaTime) const { return false; }

	/**
	 * For GPU solver (M3): If this collider is a per-bone SDF, fill the grid/transform view and return true. The default is false.
	 * Like capsule, this is a path that identifies the SDF collider without RTTI (mutually exclusive with GetGPUCapsule).
	 */
	virtual bool GetGPUSDF(FRopeSDFColliderView& OutView) const { return false; }

	/**
	 * For GPU solvers: If this collider is an analytic box (OBB), fill the world space center/rot/half-extents and return true.
	 * Default is false. Mutually exclusive with GetGPUCapsule/GetGPUSDF (same RTTI-Free identification pattern). static world
	 * Because it is for geometry, there is no frame motion — the GPU responds with surface velocity 0 (static).
	 */
	virtual bool GetGPUBox(FVector& OutCenter, FQuat& OutRot, FVector& OutHalfExtents) const { return false; }

	/**
	 * Frame motion of GPU box: previous frame center/rot + 1/framedt. Only colliders with GetGPUBox=true are meaningful.
	 * Default is false (static) — caller falls back to prev=current, InvDt=0. A moving static body (platform/door)
	 * Used to drag a rope (surface velocity) and prevent tunneling with a substep CCD (corresponds to capsule's GetGPUCapsuleMotion).
	 */
	virtual bool GetGPUBoxMotion(FVector& OutPrevCenter, FQuat& OutPrevRot, float& OutInvDeltaTime) const { return false; }

	/**
	 * For GPU solver: If this collider is an analytical convex, body-local plane set (unit normal·outer, PlaneDot(p)=dot(N,p)-W,
	 * scale reflection/rigid body transform not applied) and local AABB, and the body's rigid body transform (curr rot/trans + prev)
	 * fills 1/framedt and returns true. The default is false. world plane = local plane ∘ rigid body(rot, trans). The moving body
	 * prev Process surface velocity/substep CCD with rigid body (if static, prev=curr, InvDt=0). OutLocalPlanes collider
	 * Owned storage view (valid during that frame).
	 */
	virtual bool GetGPUConvex(TConstArrayView<FPlane>& OutLocalPlanes, FBox& OutLocalBounds,
		FQuat& OutRot, FVector& OutTrans, FQuat& OutPrevRot, FVector& OutPrevTrans, float& OutInvDeltaTime) const
	{
		return false;
	}

	/**
	 * Fills this frame's motion (prev->curr world transform) of the moving collider and sets it to true. The default is false (static/no motion).
	 * The solver is used to calculate the sub-pose for each substep in swept collision once outside the node loop (relative motion CCD hoisting).
	 * Must be const (read only) — collider is shared between ropes and solved in parallel.
	 */
	virtual bool GetFrameMotion(FTransform& OutPrev, FTransform& OutCurr) const { return false; }

	/**
	 * attribution for GPU contact detection (G3): Which bone/mesh this collider belongs to. GPU collider
	 * emits only the index, so the caller restores the index → (bone, mesh) with this. FRopeContact.Bone/SourceMesh
	 * Must be the same value (feed to the same check pipeline). The default is None/null.
	 */
	virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const
	{
		OutBone = NAME_None;
		OutMesh = nullptr;
	}
};

/**
 * Analytical capsule (swept-sphere segment). v1 bone collider — such as per-bone SDF (FRopeSDFCollider)
 * Selected on a per-provider basis behind the IRopeCollider interface (the solver does not know which one).
 */
class DYNAMICROPE_API FCapsuleCollider : public IRopeCollider
{
public:
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;
	FName   Bone = NAME_None;

	/**
	 * previous frame endpoint + 1/framedt. The provider caches and fills the prev endpoints for each bone (the SDF provider's
	 * PrevBoneToWorld counterpart). If InvDeltaTime=0 (default), static capsule — prev is ignored and the same behavior as before.
	 * The contact material point is identified by the segment parameter (t): prev position = Lerp(PrevA, PrevB, t). capsule axis itself
	 * Spin cannot be expressed, but it is a negligible component in bone capsule.
	 */
	FVector PrevA = FVector::ZeroVector;
	FVector PrevB = FVector::ZeroVector;
	float   InvDeltaTime = 0.0f;

	/**
	 * The mesh (skeletal) to which the bone of this capsule belongs. Even if it is delivered to a contact and the wrap exceeds the actor,
	 * Allows you to follow the *correct* mesh (the mesh that owns the captured bone). The type is USceneComponent.
	 * Generalization (compared to static opt-in) — capsule only passes skeletal.
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
