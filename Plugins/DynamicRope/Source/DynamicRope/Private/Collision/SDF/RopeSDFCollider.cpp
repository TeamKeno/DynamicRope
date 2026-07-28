// Copyright Epic Games, Inc. All Rights Reserved.
#include "Collision/SDF/RopeSDFCollider.h"
#include "Components/SceneComponent.h"
#include "DynamicRopeLog.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFSampler.h"
#include "HAL/IConsoleManager.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
#if !UE_BUILD_SHIPPING
	TAutoConsoleVariable<int32> CVarRopeLogSDFProjection(
		TEXT("r.DynamicRope.Debug.LogSDFProjection"),
		0,
		TEXT("Logs SDF surface projection diagnostics. 0=off, 1=failures, 2=failures and successful outside-bounds projections."));
#endif

	bool ShouldLogSDFProjection(int32 Level)
	{
#if !UE_BUILD_SHIPPING
		return CVarRopeLogSDFProjection.GetValueOnAnyThread() >= Level;
#else
		return false;
#endif
	}

	/**
	 * For negative scales (mirroring actors in the level): change the sign of the flipped axis before returning the local gradient to the world.
	 * Revert. For the position, FTransform::InverseTransformPosition divides the sign as is and gives mirror local coordinates.
	 * This is already true, but if you only rotate the normal (TransformVectorNoScale), its axis will point **inside** —
	 * In the FRopeContact contract, the normal is outward and the sign is load-bearing (the inward normal draws the rope into the body).
	 * On a positive scale, the multiplication value is exactly 1.0, so the existing result is bit-invariant. RopeScaleSign mirror on GPU.
	 */
	FVector MirrorLocalNormalForScale(const FVector& NLocal, const FTransform& Xform)
	{
		const FVector S = Xform.GetScale3D();
		return FVector(S.X < 0.0 ? -NLocal.X : NLocal.X,
			S.Y < 0.0 ? -NLocal.Y : NLocal.Y,
			S.Z < 0.0 ? -NLocal.Z : NLocal.Z);
	}
}

FRopeContact FRopeSDFCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	// point query(contact detection/wrap path). Solver collision uses QuerySwept.
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSDF_Query);
	FRopeContact Contact;

	if (!Volume || !Volume->IsBaked())
	{
		// Unbaked/invalid volume → No contact.
		return Contact;
	}

	// world → bone local space. The grid is baked into bone local (scale-Free ref pose).
	const FVector LocalPos = BoneToWorld.InverseTransformPosition(WorldPos);

	// local (baked) distance ↔ world distance conversion scale (#3). SDF distance is local cm without scale, NodeRadius/
	// Penetration/SurfacePoint is world cm — in the scaled mesh the contact band/push-out is offset by a multiple of the scale.
	// Assume uniform scale (non-uniformity matches GetScaledRadius of maximum component approximation — capsule provider). If scale is 1
	// LocalNodeRadius==NodeRadius·WorldDist==Dist, so the operation is unchanged.
	const float LocalToWorldScale = FMath::Max(KINDA_SMALL_NUMBER, static_cast<float>(BoneToWorld.GetScale3D().GetAbsMax()));
	const float LocalNodeRadius = NodeRadius / LocalToWorldScale;

	// If it is outside the narrow band (including node radius margin), quickly culling (converting world radius to local).
	if (!Volume->LocalBounds.ExpandBy(LocalNodeRadius).IsInsideOrOn(LocalPos))
	{
		return Contact;
	}

	// signed distance(outer +, local cm). Sampling is delegated to a single source of truth (RopeSDFSampler) shared with the visualization.
	// If the node sphere does not reach the surface, the gradient is dropped without even being calculated (Query is called every node×substep×iteration).
	const float Dist = RopeSDFSampler::SampleTrilinear(*Volume, LocalPos);
	if (Dist >= LocalNodeRadius)
	{
		return Contact;
	}

	// Outer unit normal (fallback to +Z when sampler degenerates). bone local → world (ignore scale, maintain units).
	const FVector NLocal = RopeSDFSampler::SampleGradient(*Volume, LocalPos);

	const float WorldDist = Dist * LocalToWorldScale;
	Contact.bHit = true;
	Contact.Normal = BoneToWorld.TransformVectorNoScale(MirrorLocalNormalForScale(NLocal, BoneToWorld))
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	// Penetration = Overlap depth based on query radius (positive number, world). SurfacePoint is the closest point on the surface (auxiliary/debug).
	Contact.Penetration = NodeRadius - WorldDist;
	Contact.SurfacePoint = WorldPos - Contact.Normal * WorldDist;
	// bone attribution (enter dominant bone in contact aggregation — non-None required) and the mesh that owns the bone (wrap follow between actors).
	Contact.Bone = Bone;
	Contact.SourceMesh = SourceMesh;

	// surface velocity (cm/s): The material point on the bone in WorldPos is the same as the PrevBoneToWorld standard in the previous frame.
	// was at local coordinates (LocalPos). (current - previous) / dt is the world velocity of that point. The solver's relative tangential velocity
	// Used to sweep left and right by dragging the rope with friction. If InvDeltaTime==0(first frame/stationary), 0 → existing operation.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevWorld = PrevBoneToWorld.TransformPosition(LocalPos);
		Contact.SurfaceVelocity = (WorldPos - PrevWorld) * InvDeltaTime;
	}
	return Contact;
}

FRopeSurfaceProjection FRopeSDFCollider::ProjectToSurface(const FVector& WorldPos, float MaxDistance) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSDF_ProjectToSurface);
	FRopeSurfaceProjection Projection;

	if (!Volume || !Volume->IsBaked())
	{
		return Projection;
	}

	const FVector LocalPos = BoneToWorld.InverseTransformPosition(WorldPos);
	const bool bOutsideBounds = !Volume->LocalBounds.IsInsideOrOn(LocalPos);
	FVector SurfaceLocal = Volume->LocalBounds.GetClosestPointTo(LocalPos);
	const float DistToBounds = static_cast<float>(FVector::Distance(LocalPos, SurfaceLocal));
	if (MaxDistance > 0.0f && DistToBounds > MaxDistance)
	{
		if (ShouldLogSDFProjection(1))
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("[RopeSDFProjection] fail=BoundsTooFar bone=%s mesh=%s world=%s local=%s distToBounds=%.2f maxDistance=%.2f"),
				*Bone.ToString(), *GetNameSafe(SourceMesh), *WorldPos.ToString(), *LocalPos.ToString(),
				DistToBounds, MaxDistance);
		}
		return Projection;
	}

	// Queries outside Bounds start at the nearest grid boundary. After a few trips along the SDF,
	// You can convergence with the actual surface points inside the narrow-band and then check the actual distance from the original query.
	constexpr int32 MaxProjectionIterations = 3;
	constexpr float ProjectionTolerance = 0.05f;
	for (int32 Iteration = 0; Iteration < MaxProjectionIterations; ++Iteration)
	{
		const float SignedDistance = RopeSDFSampler::SampleTrilinear(*Volume, SurfaceLocal);
		const FVector Gradient = RopeSDFSampler::SampleProjectionGradient(*Volume, SurfaceLocal);
		if (Gradient.IsNearlyZero())
		{
			if (ShouldLogSDFProjection(1))
			{
				UE_LOG(LogDynamicRope, Warning,
					TEXT("[RopeSDFProjection] fail=GradientZero bone=%s mesh=%s world=%s local=%s sampleLocal=%s iter=%d signedDistance=%.2f distToBounds=%.2f maxDistance=%.2f"),
					*Bone.ToString(), *GetNameSafe(SourceMesh), *WorldPos.ToString(), *LocalPos.ToString(),
					*SurfaceLocal.ToString(), Iteration, SignedDistance, DistToBounds, MaxDistance);
			}
			return Projection;
		}

		SurfaceLocal = Volume->LocalBounds.GetClosestPointTo(
			SurfaceLocal - Gradient * SignedDistance);
		if (FMath::Abs(SignedDistance) <= ProjectionTolerance)
		{
			break;
		}
	}

	const FVector SurfaceWorld = BoneToWorld.TransformPosition(SurfaceLocal);
	const float SurfaceDistance = static_cast<float>(FVector::Distance(WorldPos, SurfaceWorld));
	if (MaxDistance > 0.0f && SurfaceDistance > MaxDistance)
	{
		if (ShouldLogSDFProjection(1))
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("[RopeSDFProjection] fail=SurfaceTooFar bone=%s mesh=%s world=%s local=%s surfaceWorld=%s surfaceLocal=%s surfaceDistance=%.2f distToBounds=%.2f maxDistance=%.2f"),
				*Bone.ToString(), *GetNameSafe(SourceMesh), *WorldPos.ToString(), *LocalPos.ToString(),
				*SurfaceWorld.ToString(), *SurfaceLocal.ToString(), SurfaceDistance, DistToBounds, MaxDistance);
		}
		return Projection;
	}

	const FVector NLocal = RopeSDFSampler::SampleProjectionGradient(*Volume, SurfaceLocal);
	if (NLocal.IsNearlyZero())
	{
		if (ShouldLogSDFProjection(1))
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("[RopeSDFProjection] fail=SurfaceGradientZero bone=%s mesh=%s world=%s local=%s surfaceLocal=%s surfaceDistance=%.2f maxDistance=%.2f"),
				*Bone.ToString(), *GetNameSafe(SourceMesh), *WorldPos.ToString(), *LocalPos.ToString(),
				*SurfaceLocal.ToString(), SurfaceDistance, MaxDistance);
		}
		return Projection;
	}
	const FVector NormalWorld = BoneToWorld.TransformVectorNoScale(MirrorLocalNormalForScale(NLocal, BoneToWorld))
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

	Projection.bHit = true;
	Projection.SurfacePoint = SurfaceWorld;
	Projection.Normal = NormalWorld;
	Projection.Distance = SurfaceDistance;
	Projection.Bone = Bone;
	Projection.SourceMesh = SourceMesh;
	if (bOutsideBounds && ShouldLogSDFProjection(2))
	{
		UE_LOG(LogDynamicRope, Log,
			TEXT("[RopeSDFProjection] success=OutsideBounds bone=%s mesh=%s world=%s surfaceWorld=%s surfaceDistance=%.2f distToBounds=%.2f maxDistance=%.2f"),
			*Bone.ToString(), *GetNameSafe(SourceMesh), *WorldPos.ToString(), *SurfaceWorld.ToString(),
			SurfaceDistance, DistToBounds, MaxDistance);
	}
	return Projection;
}

FRopeContact FRopeSDFCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// Main cost point of solver collision (stationary rope + contact goes here). Cost per call = Pose Blend×2 +
	// Inversion + sample loop. The difference with RopeSDF_SweptSampleLoop below is the transform setup cost.
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSDF_QuerySwept);
	FRopeContact Contact;
	// Default value (prevents undefined use when uncontacted).
	OutHitWorldPos = Q.WorldEnd;
	if (!Volume || !Volume->IsBaked())
	{
		return Contact;
	}

	// substep The sub-pose is calculated and passed by the solver once per collider (hoisting outside the node loop → not blending for each node).
	// If stationary bone (bUseSubPose=false), it is boned in a single current pose without blending → Most bones of a standing character are cost-Free.
	const FTransform& PoseStart = Q.bUseSubPose ? Q.SubPoseStart : BoneToWorld;
	const FTransform& PoseEnd   = Q.bUseSubPose ? Q.SubPoseEnd   : BoneToWorld;

	// local (baked) distance ↔ world conversion scale (#3). L0/L1 are scale-Free bone local and Q.NodeRadius/Penetration/
	// SurfacePoint is the world. Use the scale of the contact frame (PoseEnd). Uniform scale assumption (behavior invariant at scale 1).
	const float LocalToWorldScale = FMath::Max(KINDA_SMALL_NUMBER, static_cast<float>(PoseEnd.GetScale3D().GetAbsMax()));
	const float LocalNodeRadius = Q.NodeRadius / LocalToWorldScale;

	// node substep path to collider local relative frame: start is based on the start sub-pose, end is based on the end sub-pose.
	// This one local segment contains both node motion + collider motion (relative motion) → the fast bone moves the node
	// Even if it overtakes, it is caught at the first contact (front) because the node crosses the surface locally.
	const FVector L0 = PoseStart.InverseTransformPosition(Q.WorldStart);
	const FVector L1 = PoseEnd.InverseTransformPosition(Q.WorldEnd);

	// Number of samples based on relative displacement (stationary rope + fast bond, enough samples → prevention of overtaking penetration).
	const double RelLen = FVector::Dist(L0, L1);
	const float  Step = FMath::Max(Q.SweepStep, 0.1f);
	const int32  NumSamples = FMath::Clamp(1 + FMath::FloorToInt(RelLen / Step), 1, FMath::Max(1, Q.MaxSamples));

	const FBox Band = Volume->LocalBounds.ExpandBy(LocalNodeRadius);

	// Separation: The node starts in contact with the surface (penetration within the L0 band) and moves *outside* the surface during the substep.
	// If it is exiting, let it go without re-pinning. Otherwise, the contact node will be pinned again to the starting point of every substep.
	// It cannot fall off (especially severe in end nodes with low tension). Approach (outside the L0 band) and stationary (L1 penetration) are not affected.
	//
	// BUGFIX: If you check "outside the surface" with only the endpoint L1, *penetration* (inside L0 → outside L1 on the other side of the body) is also misjudged as separation —
	// Since L1 is the Free space on the other side, dist(L1) >= NodeRadius is set, skipping the sweep entirely, and the rope retains the body as is.
	// Passes (occurs when tension is high). The only real separation is when the node moves in the normal direction outside the surface, so outside of L0.
	// Limited to cases where the relative displacement to the gradient is positive. If it is inside (through), the sweep below is not done early-out.
	// Grab it at the first contact and push it out of the surface (= blocking penetration).
	const bool bStartInContact = Band.IsInsideOrOn(L0) && RopeSDFSampler::SampleTrilinear(*Volume, L0) < LocalNodeRadius;
	if (bStartInContact)
	{
		const bool bEndOutside = !Band.IsInsideOrOn(L1) || RopeSDFSampler::SampleTrilinear(*Volume, L1) >= LocalNodeRadius;
		if (bEndOutside)
		{
			// L0 The displacement sign relative to the outer normal (gradient) determines “true separation vs. penetration.”
			const FVector OutwardLocal = RopeSDFSampler::SampleGradient(*Volume, L0);
			if (FVector::DotProduct(L1 - L0, OutwardLocal) > 0.0f)
			{
				// bHit=false — Omit re-pin only true disconnects moving outward.
				return Contact;
			}
		}
	}

	for (int32 k = 0; k < NumSamples; ++k)
	{
		const double T = (NumSamples <= 1) ? 1.0 : static_cast<double>(k) / static_cast<double>(NumSamples - 1);
		const FVector Lp = FMath::Lerp(L0, L1, T);
		if (!Band.IsInsideOrOn(Lp))
		{
			// Outside narrow band (volume + node radius) → No contact.
			continue;
		}

		const float Dist = RopeSDFSampler::SampleTrilinear(*Volume, Lp);
		if (Dist >= LocalNodeRadius)
		{
			// Still not up to the surface.
			continue;
		}

		// First contact. Normal/position is converted based on the substep end pose (current frame reached by the node).
		const FVector NLocal = RopeSDFSampler::SampleGradient(*Volume, Lp);
		const float WorldDist = Dist * LocalToWorldScale;
		Contact.bHit = true;
		Contact.Normal = PoseEnd.TransformVectorNoScale(MirrorLocalNormalForScale(NLocal, PoseEnd))
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		Contact.Penetration = Q.NodeRadius - WorldDist;
		// Node placement reference point (current pose world).
		OutHitWorldPos = PoseEnd.TransformPosition(Lp);
		Contact.SurfacePoint = OutHitWorldPos - Contact.Normal * WorldDist;
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;

		// surface velocity (drag): prev->curr frame displacement / dt of contact material point (Lp).
		if (InvDeltaTime > 0.0f)
		{
			const FVector WCurr = BoneToWorld.TransformPosition(Lp);
			const FVector WPrev = PrevBoneToWorld.TransformPosition(Lp);
			Contact.SurfaceVelocity = (WCurr - WPrev) * InvDeltaTime;
		}
		break;
	}
	return Contact;
}

FBox FRopeSDFCollider::GetWorldBounds() const
{
	if (!Volume)
	{
		return FBox(ForceInit);
	}
	return Volume->LocalBounds.TransformBy(BoneToWorld);
}

bool FRopeSDFCollider::GetGPUSDF(FRopeSDFColliderView& OutView) const
{
	if (!Volume || !Volume->IsBaked())
	{
		// Unbaked/invalid volumes are excluded from GPU collision (same guard as CPU Query).
		return false;
	}
	// Code byte blob (consumer dequantized into asymmetric bands).
	OutView.Distances    = Volume->Distances.GetData();
	OutView.BytesPerCode = Volume->BytesPerCode();
	OutView.NarrowBandInner = Volume->NarrowBandInner;
	OutView.NarrowBandOuter = Volume->NarrowBandOuter;
	OutView.ResX         = Volume->Resolution.X;
	OutView.ResY         = Volume->Resolution.Y;
	OutView.ResZ         = Volume->Resolution.Z;
	OutView.LocalMin     = Volume->LocalBounds.Min;
	OutView.LocalSize    = Volume->LocalBounds.GetSize();
	OutView.BoneToWorld  = BoneToWorld;
	// For GPU CCD/surfacevelocity dragging (same source as CPU QuerySwept).
	OutView.PrevBoneToWorld = PrevBoneToWorld;
	OutView.InvDeltaTime = InvDeltaTime;
	// Volume stable identifier (planted by provider) — GPU SDF cache key. No use of raw pointers (to prevent unload errors).
	OutView.VolumeKey    = VolumeKey;
	return true;
}
