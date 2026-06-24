// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeCollider.h"

FRopeContact FCapsuleCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	const FVector Closest = FMath::ClosestPointOnSegment(WorldPos, A, B);
	const FVector ToNode = WorldPos - Closest;
	const float   Dist = ToNode.Size();
	const float   MinDist = Radius + NodeRadius;
	if (Dist >= MinDist)
	{
		return Contact; // bHit = false (히트 없음)
	}

	Contact.bHit = true;
	Contact.Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToNode / Dist) : FVector::UpVector;
	Contact.Penetration = MinDist - Dist;
	Contact.SurfacePoint = Closest + Contact.Normal * Radius;
	Contact.Bone = Bone;
	Contact.SourceMesh = SourceMesh;
	return Contact;
}

FBox FCapsuleCollider::GetWorldBounds() const
{
	FBox Box(ForceInit);
	Box += A;
	Box += B;
	return Box.ExpandBy(Radius);
}
