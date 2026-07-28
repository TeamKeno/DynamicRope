// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeCollider.h"
// RopeMath::ClosestSegmentParam (identifying contact material point)
#include "RopeMathHelpers.h"

FRopeContact FCapsuleCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	// The nearest point and distance from the node center to the capsule segment (A-B).
	const float   TSeg = RopeMath::ClosestSegmentParam(WorldPos, A, B);
	const FVector Closest = FMath::Lerp(A, B, TSeg);
	// ToNode = segment surface -> node (outward direction). If it is less than MinDist, check for overlap.
	const FVector ToNode = WorldPos - Closest;
	const float   Dist = ToNode.Size();
	const float   MinDist = Radius + NodeRadius;
	if (Dist >= MinDist)
	{
		// bHit = false: No overlap -> The remaining fields are meaningless (ignored by the caller).
		return Contact;
	}

	Contact.bHit = true;
	// Normal: Unit length, points to the node side (outside) on the surface = push-out direction.
	// sign is load-bearing (see FRopeContact contract comments): if flipped, the solver
	// Suck the rope into the capsule. Even when replacing with SDF, ∇ϕ (always points outward)
	// Written as is, it conforms to this convention — but pinned the bake as outside-positive.
	// Since direction is not defined in degenerate (node is on segment axis = Dist≈0), arbitrary
	// Fallback to the stability vector (+Z). SDF also requires the same fallback in the ∇ϕ≈0 section.
	Contact.Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToNode / Dist) : FVector::UpVector;
	// Penetration = Normal direction overlap depth (positive number). SurfacePoint is the closest point on the surface (auxiliary/debug use).
	Contact.Penetration = MinDist - Dist;
	Contact.SurfacePoint = Closest + Contact.Normal * Radius;
	// bone attribution (input to select dominant bone of contact aggregation) and the mesh that owns the bone (wrap follow between actors).
	Contact.Bone = Bone;
	Contact.SourceMesh = SourceMesh;

	// surface velocity (cm/s): (current - previous) / dt of contact material point (segment parameter TSeg). solver is relative tangent
	// Used to drag and sweep a rope by friction (same contract as SDF collider). 0 if InvDeltaTime==0(static/first frame).
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevClosest = FMath::Lerp(PrevA, PrevB, TSeg);
		Contact.SurfaceVelocity = (Closest - PrevClosest) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FCapsuleCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// stationary capsule (including first frame) is the same as the default (current pose static sweep) — early delegation.
	if (InvDeltaTime <= 0.0f || (PrevA.Equals(A) && PrevB.Equals(B)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	// Default value (prevents undefined use when uncontacted).
	OutHitWorldPos = Q.WorldEnd;

	// Capsule endpoint of this substep (interpolation of frame motion prev->curr into SubAlpha). Capsule does not have rigid transform.
	// (Two joint points move separately) Interpolate the end point itself instead of SubPose — Corresponds to SDF QuerySwept's local frame trick.
	const FVector CapAS = FMath::Lerp(PrevA, A, Q.SubAlpha0);
	const FVector CapBS = FMath::Lerp(PrevB, B, Q.SubAlpha0);
	const FVector CapAE = FMath::Lerp(PrevA, A, Q.SubAlpha1);
	const FVector CapBE = FMath::Lerp(PrevB, B, Q.SubAlpha1);

	// Number of samples reflecting relative motion: node movement + maximum of capsule end point movement (safe to overestimate — capped by MaxSamples).
	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd)
		+ FMath::Max(FVector::Dist(CapAS, CapAE), FVector::Dist(CapBS, CapBE));
	const float  Step = FMath::Max(Q.SweepStep, 0.1f);
	const int32  NumSamples = FMath::Clamp(1 + FMath::FloorToInt(RelLen / Step), 1, FMath::Max(1, Q.MaxSamples));
	const float  MinDist = Radius + Q.NodeRadius;

	// Separation Guard (RopeCollision::IsSweptSeparating): Omit re-pin if starting inside the contact skin and separating outside the surface.
	// The starting material point (TSeg0) is carried over to the start/end pose to check the movement of the contact point, and the end pose segment is checked for end contact.
	{
		const float   TSeg0 = RopeMath::ClosestSegmentParam(Q.WorldStart, CapAS, CapBS);
		const FVector Closest0 = FMath::Lerp(CapAS, CapBS, TSeg0);
		const FVector ToNode0 = Q.WorldStart - Closest0;
		const float   Dist0 = static_cast<float>(ToNode0.Size());
		const FVector Outward0 = (Dist0 > KINDA_SMALL_NUMBER) ? (ToNode0 / Dist0) : FVector::UpVector;
		const FVector Closest0End = FMath::Lerp(CapAE, CapBE, TSeg0);
		const float   TSeg1 = RopeMath::ClosestSegmentParam(Q.WorldEnd, CapAE, CapBE);
		const bool    bEndInContact = static_cast<float>(FVector::Dist(Q.WorldEnd, FMath::Lerp(CapAE, CapBE, TSeg1))) < MinDist;
		if (RopeCollision::IsSweptSeparating(Q.WorldStart, Q.WorldEnd, Closest0, Closest0End, Outward0,
			/*bStartInContact*/ Dist0 < MinDist, bEndInContact))
		{
			return Contact;
		}
	}

	for (int32 k = 0; k < NumSamples; ++k)
	{
		const float T = (NumSamples <= 1) ? 1.0f : static_cast<float>(k) / static_cast<float>(NumSamples - 1);
		const FVector Pt = FMath::Lerp(Q.WorldStart, Q.WorldEnd, T);
		const FVector CapAT = FMath::Lerp(CapAS, CapAE, T);
		const FVector CapBT = FMath::Lerp(CapBS, CapBE, T);
		const float   TSeg = RopeMath::ClosestSegmentParam(Pt, CapAT, CapBT);
		const FVector Closest = FMath::Lerp(CapAT, CapBT, TSeg);
		const FVector ToNode = Pt - Closest;
		const float   Dist = static_cast<float>(ToNode.Size());
		if (Dist >= MinDist)
		{
			continue;
		}

		// First contact. Carry over the contact material point (TSeg) to the end pose of the substep and move the node along with the surface by the remaining amount of motion.
		// (correspondence to SDF reconverting the local contact point to the end pose). The normal is a relative direction based on the material point, so it is valid as is.
		Contact.bHit = true;
		Contact.Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToNode / Dist) : FVector::UpVector;
		Contact.Penetration = MinDist - Dist;
		const FVector ClosestEnd = FMath::Lerp(CapAE, CapBE, TSeg);
		OutHitWorldPos = Pt + (ClosestEnd - Closest);
		Contact.SurfacePoint = ClosestEnd + Contact.Normal * Radius;
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;

		// surface velocity: Total frame (prev->curr) displacement of material point / dt (same contract as Query).
		const FVector WCurr = FMath::Lerp(A, B, TSeg);
		const FVector WPrev = FMath::Lerp(PrevA, PrevB, TSeg);
		Contact.SurfaceVelocity = (WCurr - WPrev) * InvDeltaTime;
		break;
	}
	return Contact;
}

FRopeSurfaceProjection FCapsuleCollider::ProjectToSurface(const FVector& WorldPos, float MaxDistance) const
{
	FRopeSurfaceProjection Projection;

	const FVector Closest = FMath::ClosestPointOnSegment(WorldPos, A, B);
	const FVector ToNode = WorldPos - Closest;
	const float DistToAxis = ToNode.Size();
	const FVector Normal = (DistToAxis > KINDA_SMALL_NUMBER) ? (ToNode / DistToAxis) : FVector::UpVector;
	const FVector SurfacePoint = Closest + Normal * Radius;
	const float SurfaceDistance = FMath::Abs(DistToAxis - Radius);
	if (MaxDistance > 0.0f && SurfaceDistance > MaxDistance)
	{
		return Projection;
	}

	Projection.bHit = true;
	Projection.SurfacePoint = SurfacePoint;
	Projection.Normal = Normal;
	Projection.Distance = SurfaceDistance;
	Projection.Bone = Bone;
	Projection.SourceMesh = SourceMesh;
	return Projection;
}

FBox FCapsuleCollider::GetWorldBounds() const
{
	FBox Box(ForceInit);
	Box += A;
	Box += B;
	return Box.ExpandBy(Radius);
}

bool FCapsuleCollider::GetGPUCapsule(FVector& OutA, FVector& OutB, float& OutRadius) const
{
	// Pass world space segment + radius as is. The GPU solver performs the same segment nearest push-out as the CPU Query.
	OutA = A;
	OutB = B;
	OutRadius = Radius;
	return true;
}

bool FCapsuleCollider::GetGPUCapsuleMotion(FVector& OutPrevA, FVector& OutPrevB, float& OutInvDeltaTime) const
{
	// previous frame endpoint + InvDt. If InvDeltaTime=0(static/first frame), false so that the caller falls back to prev=current.
	if (InvDeltaTime <= 0.0f)
	{
		return false;
	}
	OutPrevA = PrevA;
	OutPrevB = PrevB;
	OutInvDeltaTime = InvDeltaTime;
	return true;
}
