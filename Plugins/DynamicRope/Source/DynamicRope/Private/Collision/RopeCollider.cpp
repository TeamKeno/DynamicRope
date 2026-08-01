// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeCollider.h"
// RopeMath::ClosestSegmentParam (identifying contact material point)
#include "RopeMathHelpers.h"

FRopeContact FCapsuleCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	// Nearest point on the capsule segment (A-B) to the node centre, and the distance to it.
	const float   TSeg = RopeMath::ClosestSegmentParam(WorldPos, A, B);
	const FVector Closest = FMath::Lerp(A, B, TSeg);
	// ToNode points from the segment surface out to the node. Below MinDist the two overlap.
	const FVector ToNode = WorldPos - Closest;
	const float   Dist = ToNode.Size();
	const float   MinDist = Radius + NodeRadius;
	if (Dist >= MinDist)
	{
	// bHit = false: no overlap, so every other field is meaningless and the caller ignores it.
		return Contact;
	}

	Contact.bHit = true;
	// Normal: unit length, pointing from the surface out toward the node — the push-out direction.
	// The sign is load-bearing (see the FRopeContact contract): flipped, the solver sucks the rope into the
	// capsule. An SDF collider satisfies the same convention by using ∇ϕ directly, which always points
	// outward, and its bake pins outside as positive.
	// In the degenerate case, where the node sits on the segment axis and Dist ≈ 0, no direction is defined,
	// so it falls back to a fixed vector (+Z). An SDF needs the same fallback where ∇ϕ ≈ 0.
	Contact.Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToNode / Dist) : FVector::UpVector;
	// Penetration is the overlap depth along Normal, always positive. SurfacePoint is the nearest point on the surface, kept for diagnostics.
	Contact.Penetration = MinDist - Dist;
	Contact.SurfacePoint = Closest + Contact.Normal * Radius;
	// Bone attribution, which is what picks the dominant bone when contacts are aggregated, and the mesh owning that bone, which is what lets a wrap follow across actors.
	Contact.Bone = Bone;
	Contact.SourceMesh = SourceMesh;

	// Surface velocity (cm/s): (current − previous) / dt of the contact material point, identified by the
	// segment parameter TSeg. The solver uses it for relative tangential friction, so a moving capsule drags
	// and sweeps the rope — the same contract the SDF collider follows. 0 when InvDeltaTime is 0, meaning a
	// static capsule or the first frame.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevClosest = FMath::Lerp(PrevA, PrevB, TSeg);
		Contact.SurfaceVelocity = (Closest - PrevClosest) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FCapsuleCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// A still capsule, the first frame included, behaves exactly as the default static sweep against the current pose, so delegate early.
	if (InvDeltaTime <= 0.0f || (PrevA.Equals(A) && PrevB.Equals(B)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	// Defaults, so nothing is left undefined when there is no contact.
	OutHitWorldPos = Q.WorldEnd;

	// This substep's capsule endpoints, interpolating the frame motion prev → curr by SubAlpha. A capsule has
	// no rigid transform — its two joints move independently — so the endpoints themselves are interpolated
	// rather than a sub-pose, which is the counterpart to the SDF QuerySwept's local-frame trick.
	const FVector CapAS = FMath::Lerp(PrevA, A, Q.SubAlpha0);
	const FVector CapBS = FMath::Lerp(PrevB, B, Q.SubAlpha0);
	const FVector CapAE = FMath::Lerp(PrevA, A, Q.SubAlpha1);
	const FVector CapBE = FMath::Lerp(PrevB, B, Q.SubAlpha1);

	// Sample count from the relative motion: the larger of the node's travel and the capsule endpoints' travel. Overestimating is safe, and MaxSamples caps it.
	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd)
		+ FMath::Max(FVector::Dist(CapAS, CapAE), FVector::Dist(CapBS, CapBE));
	const int32  NumSamples = RopeCollision::SweptSampleCount(RelLen, Q.SweepStep, Q.MaxSamples);
	const float  MinDist = Radius + Q.NodeRadius;

	// Separation guard (RopeCollision::IsSweptSeparating): skip the re-pin when the node starts inside the
	// contact skin and is leaving the surface. The starting material point (TSeg0) is carried through both the
	// start and end poses to measure how the contact point moved, and the end contact is tested against the
	// end pose's segment.
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

		// First contact. Carry the contact material point (TSeg) into the substep's end pose and move the node
		// with the surface by the motion that remains — the counterpart to the SDF re-projecting its local
		// contact point into the end pose. The normal is relative to that material point, so it is already correct.
		Contact.bHit = true;
		Contact.Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToNode / Dist) : FVector::UpVector;
		Contact.Penetration = MinDist - Dist;
		const FVector ClosestEnd = FMath::Lerp(CapAE, CapBE, TSeg);
		OutHitWorldPos = Pt + (ClosestEnd - Closest);
		Contact.SurfacePoint = ClosestEnd + Contact.Normal * Radius;
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;

		// Surface velocity: the material point's whole-frame (prev → curr) displacement over dt, the same contract as Query.
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
	// Pass the world-space segment and radius straight through. The GPU solver runs the same nearest-point-on-segment push-out the CPU Query does.
	OutA = A;
	OutB = B;
	OutRadius = Radius;
	return true;
}

bool FCapsuleCollider::GetGPUCapsuleMotion(FVector& OutPrevA, FVector& OutPrevB, float& OutInvDeltaTime) const
{
	// Previous endpoints plus InvDt. With InvDeltaTime 0 — static, or the first frame — return false so the caller falls back to prev = current.
	if (InvDeltaTime <= 0.0f)
	{
		return false;
	}
	OutPrevA = PrevA;
	OutPrevB = PrevB;
	OutInvDeltaTime = InvDeltaTime;
	return true;
}
