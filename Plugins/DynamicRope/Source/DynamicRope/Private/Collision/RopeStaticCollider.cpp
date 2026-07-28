// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeStaticCollider.h"

FRopeContact FRopeBoxCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	// Convert box local to clamp — OBB closest point. Near corners/edges, the clamp result is closer to the corners/edges.
	// itself, and the normal comes out in the correct diagonal direction (the very information that GDF voxel rounding crushed).
	const FVector P = Rot.UnrotateVector(WorldPos - Center);
	const FVector Clamped(
		FMath::Clamp(P.X, -HalfExtents.X, HalfExtents.X),
		FMath::Clamp(P.Y, -HalfExtents.Y, HalfExtents.Y),
		FMath::Clamp(P.Z, -HalfExtents.Z, HalfExtents.Z));

	FVector LocalNormal = FVector::UpVector;
	FVector LocalSurface = Clamped;
	// Signed distance to surface (outer +, inner -).
	float   SignedDist = 0.0f;

	const FVector Delta = P - Clamped;
	const float DistOutside = static_cast<float>(Delta.Size());
	if (DistOutside > KINDA_SMALL_NUMBER)
	{
		// Outside: Clamp point is the nearest surface point, normal = surface -> node (outside) direction.
		SignedDist = DistOutside;
		LocalNormal = Delta / DistOutside;
	}
	else
	{
		// Inside (or on the surface): Push out in the outer direction of the surface with the shallowest penetration.
		// FaceDist = Distance to each axis face (all >= 0).
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
		// bHit = false: No overlap (same as capsule, boundary is treated as non-contact).
		return Contact;
	}

	Contact.bHit = true;
	// Normal: Unit, surface -> node (outer) — FRopeContact FROZEN contract (signed load-bearing).
	Contact.Normal = Rot.RotateVector(LocalNormal);
	// If inside, SignedDist<0, depth to side + node radius.
	Contact.Penetration = NodeRadius - SignedDist;
	Contact.SurfacePoint = Rot.RotateVector(LocalSurface) + Center;
	// If the box can be Wrapped, then virtual bone (detection attribution) + target component. If static, None/null.
	Contact.Bone = Bone;
	Contact.SourceMesh = SourceMesh;
	// surface velocity: (current pose - previous pose) / dt of contact material point (local LocalSurface). A moving body uses a rope
	// Used to drag in the tangential direction. 0 if InvDeltaTime==0(static/first frame).
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevWorld = PrevRot.RotateVector(LocalSurface) + PrevCenter;
		Contact.SurfaceVelocity = (Contact.SurfacePoint - PrevWorld) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FRopeBoxCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// stationary box (including first frame) is the same as the default (current pose static sweep) — early delegation.
	if (InvDeltaTime <= 0.0f || (PrevCenter.Equals(Center) && PrevRot.Equals(Rot)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	OutHitWorldPos = Q.WorldEnd;

	// Box sub-pose of this substep (interpolation of frame motion prev->curr with SubAlpha). Sweep node path and box motion together.
	const FVector CenterS = FMath::Lerp(PrevCenter, Center, Q.SubAlpha0);
	const FVector CenterE = FMath::Lerp(PrevCenter, Center, Q.SubAlpha1);
	const FQuat   RotS = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha0);
	const FQuat   RotE = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha1);

	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd) + FVector::Dist(CenterS, CenterE);
	const float  Step = FMath::Max(Q.SweepStep, 0.1f);
	const int32  NumSamples = FMath::Clamp(1 + FMath::FloorToInt(RelLen / Step), 1, FMath::Max(1, Q.MaxSamples));

	// Separation Guard (RopeCollision::IsSweptSeparating): Omit re-pin if starting inside the contact skin and separating outwards.
	// Checks the start contact/material point with the start pose box and the end contact with the end pose box.
	{
		const FRopeBoxCollider BoxS(CenterS, RotS, HalfExtents);
		const FRopeContact Start = BoxS.Query(Q.WorldStart, Q.NodeRadius);
		if (Start.bHit)
		{
			// End pose position of the starting material point (local).
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
		// Point question with sub-pose box (center/rot interpolation). Create a temporary collider and reuse the local clamp query.
		FRopeBoxCollider BoxT(FMath::Lerp(CenterS, CenterE, T), FQuat::Slerp(RotS, RotE, T), HalfExtents);
		// BoxT is InvDt=0 → surface velocity 0 (not used here).
		const FRopeContact C = BoxT.Query(Pt, Q.NodeRadius);
		if (!C.bHit)
		{
			continue;
		}
		// Carry over the contact material point (local) to the end pose of the substep — Move the node along with the surface by the remaining amount of motion.
		const FVector Lp = BoxT.Rot.UnrotateVector(C.SurfacePoint - BoxT.Center);
		const FVector ClosestEnd = RotE.RotateVector(Lp) + CenterE;
		Contact.bHit = true;
		Contact.Normal = C.Normal;
		Contact.Penetration = C.Penetration;
		OutHitWorldPos = Pt + (ClosestEnd - C.SurfacePoint);
		Contact.SurfacePoint = ClosestEnd;
		// If the box can be Wrapped, then virtual bone (detection attribution) + target component. If static, None/null.
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;
		// surface velocity: entire frame (prev->curr) displacement of material point / dt.
		const FVector WCurr = Rot.RotateVector(Lp) + Center;
		const FVector WPrev = PrevRot.RotateVector(Lp) + PrevCenter;
		Contact.SurfaceVelocity = (WCurr - WPrev) * InvDeltaTime;
		break;
	}
	return Contact;
}

FBox FRopeBoxCollider::GetWorldBounds() const
{
	// OBB -> AABB: per axis |rotation basis| · Half width sum.
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
	// world -> body-local (rigid body station): Lp = qInv*(p - Trans). Afterwards, query with local plane/local bounds.
	const FVector Lp = Rot.UnrotateVector(WorldPos - Trans);
	if (!LocalBounds.IsValid || !LocalBounds.ExpandBy(NodeRadius).IsInsideOrOn(Lp))
	{
		// outside local AABB(+NodeRadius) → definitely not in contact.
		return Contact;
	}

	// max-plane(local): The plane (maximum sign distance) that the point violates the most is the surface approximation. Accurate on the inside, near the outside edge
	// Underestimation (conservative). The push-out direction is the rotation of the local normal of the plane to the world.
	double MaxD = -DBL_MAX;
	int32 Best = INDEX_NONE;
	for (int32 i = 0; i < LocalPlanes.Num(); ++i)
	{
		// dot(N,p) - W (local), outside N.
		const double D = LocalPlanes[i].PlaneDot(Lp);
		if (D > MaxD)
		{
			MaxD = D;
			Best = i;
		}
	}
	if (Best == INDEX_NONE || MaxD >= NodeRadius)
	{
		// Some surface out of the NodeRadius is definitely out of convex → definitely out of contact.
		return Contact;
	}

	const FVector LocalNormal(LocalPlanes[Best].X, LocalPlanes[Best].Y, LocalPlanes[Best].Z);
	// local surface point (material point).
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
	// surface velocity: (current pose - previous pose) / dt of material point (local). A moving body pulls the rope in a tangential direction.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevWorld = PrevRot.RotateVector(LocalSurface) + PrevTrans;
		Contact.SurfaceVelocity = (Contact.SurfacePoint - PrevWorld) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FRopeConvexCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// stationary convex (with first frame) is the same as basic (current pose static sweep) — early delegation.
	if (InvDeltaTime <= 0.0f || (PrevTrans.Equals(Trans) && PrevRot.Equals(Rot)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	OutHitWorldPos = Q.WorldEnd;

	// substep sub-pose rigid body(prev->curr interpolation).
	const FVector TransS = FMath::Lerp(PrevTrans, Trans, Q.SubAlpha0);
	const FVector TransE = FMath::Lerp(PrevTrans, Trans, Q.SubAlpha1);
	const FQuat   RotS = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha0);
	const FQuat   RotE = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha1);

	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd) + FVector::Dist(TransS, TransE);
	const float  Step = FMath::Max(Q.SweepStep, 0.1f);
	const int32  NumSamples = FMath::Clamp(1 + FMath::FloorToInt(RelLen / Step), 1, FMath::Max(1, Q.MaxSamples));

	// Separation Guard (RopeCollision::IsSweptSeparating): Omit re-pin if starting inside the contact skin and separating outwards.
	// Inline the local plane query (same logic as sweep above) with the start/end sub-pose rigid body to obtain the start contact/material point and end contact.
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
		// Sub-pose local point query (local transformation to rigid body RotT/TransT).
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
		// Carry over the material point to the end sub-pose.
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
	// Convert local AABB to rigid body (Rot, Trans) → world AABB (broad phase). If invalid, stay as is.
	if (!LocalBounds.IsValid)
	{
		return LocalBounds;
	}
	return LocalBounds.TransformBy(FTransform(Rot, Trans));
}
