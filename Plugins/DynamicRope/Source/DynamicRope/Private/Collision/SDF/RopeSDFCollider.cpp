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
	 * For a negative scale — a mirrored actor in the level — flip the sign of the mirrored axes when taking a
	 * local gradient back to world space.
	 * Position needs no such fix: FTransform::InverseTransformPosition already divides through by the signed
	 * scale and gives mirrored local coordinates. But rotating the normal alone (TransformVectorNoScale) would
	 * leave those axes pointing **inward**, and in FRopeContact's contract the normal points outward with the
	 * sign load-bearing — an inward normal sucks the rope into the body.
	 * On a positive scale the multiplier is exactly 1.0, so the result is bit-identical to before. The GPU
	 * mirrors this as RopeScaleSign.
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
	// Point query, used by contact detection and the wrap path. Solver collision goes through QuerySwept instead.
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSDF_Query);
	FRopeContact Contact;

	if (!Volume || !Volume->IsBaked())
	{
		// An unbaked or invalid volume means no contact.
		return Contact;
	}

	// World → bone-local. The grid is baked in bone-local space, against a scale-free reference pose.
	const FVector LocalPos = BoneToWorld.InverseTransformPosition(WorldPos);

	// Scale converting baked local distances to world distances. An SDF distance is in unscaled local cm while
	// NodeRadius, Penetration and SurfacePoint are world cm, so on a scaled mesh the contact band and the
	// push-out would be off by that scale factor.
	// It assumes uniform scale; a non-uniform one is approximated by the largest component, matching
	// GetScaledRadius in the capsule provider. At scale 1, LocalNodeRadius == NodeRadius and WorldDist == Dist,
	// so behaviour is unchanged.
	const float LocalToWorldScale = FMath::Max(KINDA_SMALL_NUMBER, static_cast<float>(BoneToWorld.GetScale3D().GetAbsMax()));
	const float LocalNodeRadius = NodeRadius / LocalToWorldScale;

	// Outside the narrow band, node radius margin included, cull immediately — converting the world radius into local units.
	if (!Volume->LocalBounds.ExpandBy(LocalNodeRadius).IsInsideOrOn(LocalPos))
	{
		return Contact;
	}

	// Signed distance, positive outside, in local cm. Sampling is delegated to RopeSDFSampler, the single
	// source of truth it shares with the visualization.
	// If the node's sphere does not reach the surface, bail before even computing the gradient — Query runs per
	// node × substep × iteration.
	const float Dist = RopeSDFSampler::SampleTrilinear(*Volume, LocalPos);
	if (Dist >= LocalNodeRadius)
	{
		return Contact;
	}

	// Outward unit normal, falling back to +Z where the sampler degenerates. Bone-local → world, ignoring scale so the units hold.
	const FVector NLocal = RopeSDFSampler::SampleGradient(*Volume, LocalPos);

	const float WorldDist = Dist * LocalToWorldScale;
	Contact.bHit = true;
	Contact.Normal = BoneToWorld.TransformVectorNoScale(MirrorLocalNormalForScale(NLocal, BoneToWorld))
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	// Penetration is the overlap depth against the query radius, positive, in world units. SurfacePoint is the nearest point on the surface, kept for diagnostics.
	Contact.Penetration = NodeRadius - WorldDist;
	Contact.SurfacePoint = WorldPos - Contact.Normal * WorldDist;
	// Bone attribution, which must be non-None so contact aggregation can pick a dominant bone, and the mesh owning that bone, which is what lets a wrap follow across actors.
	Contact.Bone = Bone;
	Contact.SourceMesh = SourceMesh;

	// Surface velocity (cm/s): the material point on the bone at WorldPos sat at the same local coordinates
	// (LocalPos) last frame, measured against PrevBoneToWorld, so (current − previous) / dt is that point's
	// world velocity. The solver uses it for relative tangential friction, so a moving bone drags and sweeps
	// the rope. 0 when InvDeltaTime is 0 — the first frame, or a still bone.
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

	// A query outside the bounds starts from the nearest point on the grid boundary. After a few steps along
	// the SDF it converges onto a real surface point inside the narrow band, and the actual distance is then
	// measured back to the original query point.
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
	// The main cost of solver collision — a still rope in contact spends its time here. Per call that is two
	// pose blends, an inversion and the sample loop. The only difference from RopeSDF_SweptSampleLoop below is
	// the transform setup cost.
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSDF_QuerySwept);
	FRopeContact Contact;
	// Defaults, so nothing is left undefined when there is no contact.
	OutHitWorldPos = Q.WorldEnd;
	if (!Volume || !Volume->IsBaked())
	{
		return Contact;
	}

	// The solver computes the substep sub-pose once per collider, hoisted outside the node loop so it is not
	// blended per node. For a still bone (bUseSubPose = false) there is a single current pose and no blending
	// at all, so most bones on a standing character cost nothing.
	const FTransform& PoseStart = Q.bUseSubPose ? Q.SubPoseStart : BoneToWorld;
	const FTransform& PoseEnd   = Q.bUseSubPose ? Q.SubPoseEnd   : BoneToWorld;

	// Scale converting baked local distances to world. L0 and L1 are scale-free bone-local, while Q.NodeRadius,
	// Penetration and SurfacePoint are world. It uses the contact frame's scale (PoseEnd) and assumes uniform
	// scale; at scale 1 the behaviour is unchanged.
	const float LocalToWorldScale = FMath::Max(KINDA_SMALL_NUMBER, static_cast<float>(PoseEnd.GetScale3D().GetAbsMax()));
	const float LocalNodeRadius = Q.NodeRadius / LocalToWorldScale;

	// Take the node's substep path into the collider's local relative frame: the start against the start
	// sub-pose, the end against the end sub-pose. That single local segment therefore contains both the node's
	// motion and the collider's — their relative motion — so even a fast bone overtaking the node still
	// crosses the surface in local space and is caught at the first contact, on the approaching side.
	const FVector L0 = PoseStart.InverseTransformPosition(Q.WorldStart);
	const FVector L1 = PoseEnd.InverseTransformPosition(Q.WorldEnd);

	// Sample count from the relative displacement, so a still rope against a fast bone gets enough samples to stop it being overtaken and penetrated.
	const double RelLen = FVector::Dist(L0, L1);
	const int32  NumSamples = RopeCollision::SweptSampleCount(RelLen, Q.SweepStep, Q.MaxSamples);

	const FBox Band = Volume->LocalBounds.ExpandBy(LocalNodeRadius);

	// Separation: when the node starts in contact with the surface, penetrating within the L0 band, and moves
	// *outward* during the substep, let it go rather than re-pinning. Otherwise a contact node is pinned back
	// to the start of every substep and can never fall away, which is worst on a low-tension end node.
	// Approaching (outside the L0 band) and resting (still penetrating at L1) are unaffected.
	//
	// Testing "outside the surface" with the end point L1 alone would misread *penetration* — inside at L0,
	// out the far side of the body at L1 — as separation: L1 is free space on the other side, so
	// dist(L1) >= NodeRadius holds, the sweep is skipped entirely, and the rope passes straight through the
	// body, which happens under high tension.
	// Real separation only means the node moved outward along the normal, so the test is restricted to the
	// case where the displacement relative to the gradient at L0 is positive. A node that went through is not
	// early-outed: the sweep below catches it at the first contact and pushes it out of the surface, which is
	// what blocks the penetration.
	const bool bStartInContact = Band.IsInsideOrOn(L0) && RopeSDFSampler::SampleTrilinear(*Volume, L0) < LocalNodeRadius;
	if (bStartInContact)
	{
		const bool bEndOutside = !Band.IsInsideOrOn(L1) || RopeSDFSampler::SampleTrilinear(*Volume, L1) >= LocalNodeRadius;
		if (bEndOutside)
		{
			// The sign of the displacement against the outward normal (the gradient) at L0 is what separates true separation from penetration.
			const FVector OutwardLocal = RopeSDFSampler::SampleGradient(*Volume, L0);
			if (FVector::DotProduct(L1 - L0, OutwardLocal) > 0.0f)
			{
			// bHit = false — the re-pin is skipped only for a genuine separation moving outward.
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
			// Outside the narrow band (the volume plus the node radius), so no contact.
			continue;
		}

		const float Dist = RopeSDFSampler::SampleTrilinear(*Volume, Lp);
		if (Dist >= LocalNodeRadius)
		{
			// Still short of the surface.
			continue;
		}

		// First contact. The normal and position are converted against the substep's end pose — the current frame the node reached.
		const FVector NLocal = RopeSDFSampler::SampleGradient(*Volume, Lp);
		const float WorldDist = Dist * LocalToWorldScale;
		Contact.bHit = true;
		Contact.Normal = PoseEnd.TransformVectorNoScale(MirrorLocalNormalForScale(NLocal, PoseEnd))
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		Contact.Penetration = Q.NodeRadius - WorldDist;
		// Reference point for placing the node, in the current pose's world space.
		OutHitWorldPos = PoseEnd.TransformPosition(Lp);
		Contact.SurfacePoint = OutHitWorldPos - Contact.Normal * WorldDist;
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;

		// Surface velocity, for drag: the contact material point's (Lp) prev → curr frame displacement over dt.
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
		// An unbaked or invalid volume is excluded from GPU collision, the same guard the CPU Query uses.
		return false;
	}
	// Code byte blob; the consumer dequantizes it across the asymmetric bands.
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
	// For GPU CCD and surface-velocity drag, from the same source the CPU QuerySwept uses.
	OutView.PrevBoneToWorld = PrevBoneToWorld;
	OutView.InvDeltaTime = InvDeltaTime;
	// Stable volume identifier, planted by the provider — the GPU SDF cache key. A raw pointer is deliberately not used, so an unload cannot alias.
	OutView.VolumeKey    = VolumeKey;
	return true;
}
