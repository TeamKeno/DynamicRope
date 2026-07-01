// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeCollider.h"

FRopeContact FCapsuleCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	// 노드 중심에서 캡슐 세그먼트(A-B)까지의 최근접점과 거리.
	const FVector Closest = FMath::ClosestPointOnSegment(WorldPos, A, B);
	const FVector ToNode = WorldPos - Closest;   // 세그먼트 표면 -> 노드 (바깥 방향)
	const float   Dist = ToNode.Size();
	const float   MinDist = Radius + NodeRadius; // 이 거리 미만이면 겹침으로 판정
	if (Dist >= MinDist)
	{
		return Contact; // bHit = false: 겹침 없음 -> 나머지 필드는 무의미(호출자가 무시)
	}

	Contact.bHit = true;
	// Normal: 단위 길이, 표면에서 노드 쪽(바깥)을 가리킨다 = push-out 방향.
	// 부호가 load-bearing이다(FRopeContact 계약 주석 참고): 뒤집으면 솔버가
	// 로프를 캡슐 안으로 빨아들인다. SDF로 교체할 때도 ∇φ(항상 바깥을 가리킴)를
	// 그대로 쓰면 이 규약과 일치한다 — 단 베이크를 outside-positive로 고정할 것.
	// 축퇴(노드가 세그먼트 축 위 = Dist≈0)에서는 방향이 정의되지 않으므로 임의의
	// 안정 벡터(+Z)로 폴백한다. SDF도 ∇φ≈0 구간에서 동일한 폴백이 필요하다.
	Contact.Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToNode / Dist) : FVector::UpVector;
	Contact.Penetration = MinDist - Dist;                    // 양수: Normal 방향 겹침 깊이
	Contact.SurfacePoint = Closest + Contact.Normal * Radius; // 표면 위 최근접점(보조/디버그용)
	Contact.Bone = Bone;                                     // 본 귀속: DecideWrap의 dominant bone 선택 입력
	Contact.SourceMesh = SourceMesh;                         // 본을 소유한 메시(액터 간 wrap follow)
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
	// 월드 공간 세그먼트 + 반지름을 그대로 넘긴다. GPU 솔버가 CPU Query와 동일한 segment 최근접 push-out을 수행한다.
	OutA = A;
	OutB = B;
	OutRadius = Radius;
	return true;
}
