// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Collision/RopeStaticCollider.h"

FRopeContact FRopeBoxCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	// Clamp into the box's local frame for the nearest point on the OBB. Near a corner or an edge the clamped
	// point lands on that corner or edge itself and the normal comes out along the correct diagonal — exactly
	// the information GDF voxel rounding destroys.
	const FVector P = Rot.UnrotateVector(WorldPos - Center);
	const FVector Clamped(
		FMath::Clamp(P.X, -HalfExtents.X, HalfExtents.X),
		FMath::Clamp(P.Y, -HalfExtents.Y, HalfExtents.Y),
		FMath::Clamp(P.Z, -HalfExtents.Z, HalfExtents.Z));

	FVector LocalNormal = FVector::UpVector;
	FVector LocalSurface = Clamped;
	// Signed distance to the surface: positive outside, negative inside.
	float   SignedDist = 0.0f;

	const FVector Delta = P - Clamped;
	const float DistOutside = static_cast<float>(Delta.Size());
	if (DistOutside > KINDA_SMALL_NUMBER)
	{
		// Outside: the clamped point is the nearest surface point, and the normal runs surface → node, outward.
		SignedDist = DistOutside;
		LocalNormal = Delta / DistOutside;
	}
	else
	{
		// Inside, or exactly on the surface: push out through whichever face is shallowest.
		// FaceDist is the distance to each axis face, all non-negative.
		const FVector FaceDist = HalfExtents - P.GetAbs();
		int32 MinAxis = 0;
		if (FaceDist.Y < FaceDist[MinAxis]) { MinAxis = 1; }
		if (FaceDist.Z < FaceDist[MinAxis]) { MinAxis = 2; }
		const float Sign = (P[MinAxis] >= 0.0) ? 1.0f : -1.0f;
		SignedDist = -static_cast<float>(FaceDist[MinAxis]);
		LocalNormal = FVector::ZeroVector;
		LocalNormal[MinAxis] = Sign;
		LocalSurface = P;
		LocalSurface[MinAxis] = Sign * HalfExtents[MinAxis];
	}

	if (SignedDist >= NodeRadius)
	{
		// bHit = false: no overlap. As with the capsule, touching exactly counts as no contact.
		return Contact;
	}

	Contact.bHit = true;
	// Normal: unit length, surface → node, outward — FRopeContact's frozen contract, where the sign is load-bearing.
	Contact.Normal = Rot.RotateVector(LocalNormal);
	// Inside, SignedDist is negative, so the depth is that plus the node radius.
	Contact.Penetration = NodeRadius - SignedDist;
	Contact.SurfacePoint = Rot.RotateVector(LocalSurface) + Center;
	// A wrappable box reports a virtual bone for detection attribution plus the target component; a static one reports None and null.
	Contact.Bone = Bone;
	Contact.SourceMesh = SourceMesh;
	// Surface velocity: (current pose − previous pose) / dt of the contact material point, held in local space
	// as LocalSurface. It is what lets a moving body drag the rope tangentially. 0 when InvDeltaTime is 0,
	// meaning static or the first frame.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevWorld = PrevRot.RotateVector(LocalSurface) + PrevCenter;
		Contact.SurfaceVelocity = (Contact.SurfacePoint - PrevWorld) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FRopeBoxCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// A still box, the first frame included, behaves exactly as the default static sweep against the current pose, so delegate early.
	if (InvDeltaTime <= 0.0f || (PrevCenter.Equals(Center) && PrevRot.Equals(Rot)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	OutHitWorldPos = Q.WorldEnd;

	// This substep's box sub-pose, interpolating the frame motion prev → curr by SubAlpha, so the node's path and the box's motion are swept together.
	const FVector CenterS = FMath::Lerp(PrevCenter, Center, Q.SubAlpha0);
	const FVector CenterE = FMath::Lerp(PrevCenter, Center, Q.SubAlpha1);
	const FQuat   RotS = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha0);
	const FQuat   RotE = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha1);

	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd) + FVector::Dist(CenterS, CenterE);
	const int32  NumSamples = RopeCollision::SweptSampleCount(RelLen, Q.SweepStep, Q.MaxSamples);

	// Separation guard (RopeCollision::IsSweptSeparating): skip the re-pin when the node starts inside the
	// contact skin and is leaving. The start contact and material point are tested against the start pose's
	// box, and the end contact against the end pose's.
	{
		const FRopeBoxCollider BoxS(CenterS, RotS, HalfExtents);
		const FRopeContact Start = BoxS.Query(Q.WorldStart, Q.NodeRadius);
		if (Start.bHit)
		{
			// Where the starting material point ends up in the end pose, in local space.
			const FVector Lp0 = RotS.UnrotateVector(Start.SurfacePoint - CenterS);
			const FVector Closest0End = RotE.RotateVector(Lp0) + CenterE;
			const FRopeBoxCollider BoxE(CenterE, RotE, HalfExtents);
			if (RopeCollision::IsSweptSeparating(Q.WorldStart, Q.WorldEnd, Start.SurfacePoint, Closest0End,
				Start.Normal, /*bStartInContact*/ true, /*bEndInContact*/ BoxE.Query(Q.WorldEnd, Q.NodeRadius).bHit))
			{
				return Contact;
			}
		}
	}

	for (int32 k = 0; k < NumSamples; ++k)
	{
		const float T = (NumSamples <= 1) ? 1.0f : static_cast<float>(k) / static_cast<float>(NumSamples - 1);
		const FVector Pt = FMath::Lerp(Q.WorldStart, Q.WorldEnd, T);
		// Point query against the sub-posed box (interpolated centre and rotation), building a temporary collider so the local clamp query can be reused.
		FRopeBoxCollider BoxT(FMath::Lerp(CenterS, CenterE, T), FQuat::Slerp(RotS, RotE, T), HalfExtents);
		// BoxT carries InvDt = 0, so its surface velocity is 0 — not used here.
		const FRopeContact C = BoxT.Query(Pt, Q.NodeRadius);
		if (!C.bHit)
		{
			continue;
		}
		// Carry the contact material point (local) into the substep's end pose, moving the node with the surface by the motion that remains.
		const FVector Lp = BoxT.Rot.UnrotateVector(C.SurfacePoint - BoxT.Center);
		const FVector ClosestEnd = RotE.RotateVector(Lp) + CenterE;
		Contact.bHit = true;
		Contact.Normal = C.Normal;
		Contact.Penetration = C.Penetration;
		OutHitWorldPos = Pt + (ClosestEnd - C.SurfacePoint);
		Contact.SurfacePoint = ClosestEnd;
		// A wrappable box reports a virtual bone for detection attribution plus the target component; a static one reports None and null.
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;
		// Surface velocity: the material point's whole-frame (prev → curr) displacement over dt.
		const FVector WCurr = Rot.RotateVector(Lp) + Center;
		const FVector WPrev = PrevRot.RotateVector(Lp) + PrevCenter;
		Contact.SurfaceVelocity = (WCurr - WPrev) * InvDeltaTime;
		break;
	}
	return Contact;
}

FBox FRopeBoxCollider::GetWorldBounds() const
{
	// OBB → AABB: sum of |rotation basis| · half-extents per axis.
	const FVector AxX = Rot.GetAxisX() * HalfExtents.X;
	const FVector AxY = Rot.GetAxisY() * HalfExtents.Y;
	const FVector AxZ = Rot.GetAxisZ() * HalfExtents.Z;
	const FVector Ext(
		FMath::Abs(AxX.X) + FMath::Abs(AxY.X) + FMath::Abs(AxZ.X),
		FMath::Abs(AxX.Y) + FMath::Abs(AxY.Y) + FMath::Abs(AxZ.Y),
		FMath::Abs(AxX.Z) + FMath::Abs(AxY.Z) + FMath::Abs(AxZ.Z));
	return FBox(Center - Ext, Center + Ext);
}

FRopeContact FRopeConvexCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;
	if (LocalPlanes.Num() == 0)
	{
		return Contact;
	}
	// World → body-local through the rigid transform: Lp = qInv·(p − Trans). Everything after this queries the local planes and local bounds.
	const FVector Lp = Rot.UnrotateVector(WorldPos - Trans);
	if (!LocalBounds.IsValid || !LocalBounds.ExpandBy(NodeRadius).IsInsideOrOn(Lp))
	{
		// Outside the local AABB expanded by NodeRadius, so certainly not in contact.
		return Contact;
	}

	// Max-plane, in local space: the plane the point violates most — the largest signed distance — approximates
	// the surface. It is exact inside and slightly underestimates near an outside edge, which is the
	// conservative direction. The push-out direction is that plane's local normal rotated into world space.
	double MaxD = -DBL_MAX;
	int32 Best = INDEX_NONE;
	for (int32 i = 0; i < LocalPlanes.Num(); ++i)
	{
		// dot(N, p) − W in local space, with N pointing outward.
		const double D = LocalPlanes[i].PlaneDot(Lp);
		if (D > MaxD)
		{
			MaxD = D;
			Best = i;
		}
	}
	if (Best == INDEX_NONE || MaxD >= NodeRadius)
	{
		// Any plane the point is more than NodeRadius outside of puts it certainly outside the convex, so there is no contact.
		return Contact;
	}

	const FVector LocalNormal(LocalPlanes[Best].X, LocalPlanes[Best].Y, LocalPlanes[Best].Z);
	// The surface point in local space — the material point.
	const FVector LocalSurface = Lp - LocalNormal * MaxD;
	Contact.bHit = true;
	// world outer normal (FROZEN contract, sign load-bearing).
	// A wrappable convex (virtual bone) carries its attribution so the contact rides the normal
	// contact-to-wrap decision path. The static default keeps both at None.
	Contact.Bone = Bone;
	Contact.SourceMesh = SourceMesh;
	Contact.Normal = Rot.RotateVector(LocalNormal);
	Contact.Penetration = NodeRadius - static_cast<float>(MaxD);
	Contact.SurfacePoint = Rot.RotateVector(LocalSurface) + Trans;
	// Surface velocity: (current pose − previous pose) / dt of the material point, held in local space. It is what lets a moving body drag the rope tangentially.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevWorld = PrevRot.RotateVector(LocalSurface) + PrevTrans;
		Contact.SurfaceVelocity = (Contact.SurfacePoint - PrevWorld) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FRopeConvexCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// A still convex, the first frame included, behaves exactly as the default static sweep against the current pose, so delegate early.
	if (InvDeltaTime <= 0.0f || (PrevTrans.Equals(Trans) && PrevRot.Equals(Rot)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	OutHitWorldPos = Q.WorldEnd;

	// The substep's sub-posed rigid transform, interpolated prev → curr.
	const FVector TransS = FMath::Lerp(PrevTrans, Trans, Q.SubAlpha0);
	const FVector TransE = FMath::Lerp(PrevTrans, Trans, Q.SubAlpha1);
	const FQuat   RotS = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha0);
	const FQuat   RotE = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha1);

	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd) + FVector::Dist(TransS, TransE);
	const int32  NumSamples = RopeCollision::SweptSampleCount(RelLen, Q.SweepStep, Q.MaxSamples);

	// Separation guard (RopeCollision::IsSweptSeparating): skip the re-pin when the node starts inside the
	// contact skin and is leaving. The local plane query is inlined here — the same logic as the sweep above —
	// against the start and end sub-posed rigid transforms, giving the start contact and material point and
	// the end contact.
	{
		const FVector Lp0 = RotS.UnrotateVector(Q.WorldStart - TransS);
		double MaxD0 = -DBL_MAX; int32 Best0 = INDEX_NONE;
		if (LocalBounds.ExpandBy(Q.NodeRadius).IsInsideOrOn(Lp0))
		{
			for (int32 pi = 0; pi < LocalPlanes.Num(); ++pi)
			{
				const double D = LocalPlanes[pi].PlaneDot(Lp0);
				if (D > MaxD0) { MaxD0 = D; Best0 = pi; }
			}
		}
		if (Best0 != INDEX_NONE && MaxD0 < Q.NodeRadius)
		{
			const FVector LocalNormal0(LocalPlanes[Best0].X, LocalPlanes[Best0].Y, LocalPlanes[Best0].Z);
			const FVector LocalSurface0 = Lp0 - LocalNormal0 * MaxD0;
			const FVector StartSurface = RotS.RotateVector(LocalSurface0) + TransS;
			const FVector Closest0End = RotE.RotateVector(LocalSurface0) + TransE; // material point end pose
			const FVector StartNormal = RotS.RotateVector(LocalNormal0);
			const FVector Lp1 = RotE.UnrotateVector(Q.WorldEnd - TransE);
			bool bEndInContact = false;
			if (LocalBounds.ExpandBy(Q.NodeRadius).IsInsideOrOn(Lp1))
			{
				double MaxD1 = -DBL_MAX;
				for (int32 pi = 0; pi < LocalPlanes.Num(); ++pi)
				{
					MaxD1 = FMath::Max(MaxD1, LocalPlanes[pi].PlaneDot(Lp1));
				}
				bEndInContact = MaxD1 < Q.NodeRadius;
			}
			if (RopeCollision::IsSweptSeparating(Q.WorldStart, Q.WorldEnd, StartSurface, Closest0End,
				StartNormal, /*bStartInContact*/ true, bEndInContact))
			{
				return Contact;
			}
		}
	}

	for (int32 k = 0; k < NumSamples; ++k)
	{
		const float T = (NumSamples <= 1) ? 1.0f : static_cast<float>(k) / static_cast<float>(NumSamples - 1);
		const FVector Pt = FMath::Lerp(Q.WorldStart, Q.WorldEnd, T);
		const FVector TransT = FMath::Lerp(TransS, TransE, T);
		const FQuat   RotT = FQuat::Slerp(RotS, RotE, T);
		// Point query in the sub-pose's local space, transformed by that rigid transform's RotT and TransT.
		const FVector Lp = RotT.UnrotateVector(Pt - TransT);
		if (!LocalBounds.ExpandBy(Q.NodeRadius).IsInsideOrOn(Lp))
		{
			continue;
		}
		double MaxD = -DBL_MAX; int32 Best = INDEX_NONE;
		for (int32 pi = 0; pi < LocalPlanes.Num(); ++pi)
		{
			const double D = LocalPlanes[pi].PlaneDot(Lp);
			if (D > MaxD) { MaxD = D; Best = pi; }
		}
		if (Best == INDEX_NONE || MaxD >= Q.NodeRadius)
		{
			continue;
		}
		const FVector LocalNormal(LocalPlanes[Best].X, LocalPlanes[Best].Y, LocalPlanes[Best].Z);
		const FVector LocalSurface = Lp - LocalNormal * MaxD;
		// Carry the material point into the end sub-pose.
		const FVector ClosestT = RotT.RotateVector(LocalSurface) + TransT;
		const FVector ClosestEnd = RotE.RotateVector(LocalSurface) + TransE;
		Contact.bHit = true;
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;
		Contact.Normal = RotT.RotateVector(LocalNormal);
		Contact.Penetration = Q.NodeRadius - static_cast<float>(MaxD);
		OutHitWorldPos = Pt + (ClosestEnd - ClosestT);
		Contact.SurfacePoint = ClosestEnd;
		const FVector WCurr = Rot.RotateVector(LocalSurface) + Trans;
		const FVector WPrev = PrevRot.RotateVector(LocalSurface) + PrevTrans;
		Contact.SurfaceVelocity = (WCurr - WPrev) * InvDeltaTime;
		break;
	}
	return Contact;
}

FBox FRopeConvexCollider::GetWorldBounds() const
{
	// Transform the local AABB by the rigid transform (Rot, Trans) into a world AABB for the broad phase. If it is invalid, leave it alone.
	if (!LocalBounds.IsValid)
	{
		return LocalBounds;
	}
	return LocalBounds.TransformBy(FTransform(Rot, Trans));
}
