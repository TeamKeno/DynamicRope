// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeStaticCollider.h"

FRopeContact FRopeBoxCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	// 박스 로컬로 변환해 클램프 — OBB 최근접점. 모서리/엣지 근방에서는 clamp 결과가 모서리/엣지
	// 자체가 되어 normal이 정확한 대각 방향으로 나온다(GDF 복셀 라운딩이 뭉개던 바로 그 정보).
	const FVector P = Rot.UnrotateVector(WorldPos - Center);
	const FVector Clamped(
		FMath::Clamp(P.X, -HalfExtents.X, HalfExtents.X),
		FMath::Clamp(P.Y, -HalfExtents.Y, HalfExtents.Y),
		FMath::Clamp(P.Z, -HalfExtents.Z, HalfExtents.Z));

	FVector LocalNormal = FVector::UpVector;
	FVector LocalSurface = Clamped;
	float   SignedDist = 0.0f; // 표면까지 부호 거리(바깥 +, 안쪽 -)

	const FVector Delta = P - Clamped;
	const float DistOutside = static_cast<float>(Delta.Size());
	if (DistOutside > KINDA_SMALL_NUMBER)
	{
		// 바깥: 클램프점이 최근접 표면점, normal = 표면 -> 노드(바깥) 방향.
		SignedDist = DistOutside;
		LocalNormal = Delta / DistOutside;
	}
	else
	{
		// 안쪽(또는 표면 위): 침투가 가장 얕은 면의 바깥 방향으로 밀어낸다.
		const FVector FaceDist = HalfExtents - P.GetAbs(); // 각 축 면까지 거리(전부 >= 0)
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
		return Contact; // bHit = false: 겹침 없음(캡슐과 동일하게 경계는 미접촉 취급)
	}

	Contact.bHit = true;
	// Normal: 단위, 표면 -> 노드(바깥) — FRopeContact FROZEN 계약(부호가 load-bearing).
	Contact.Normal = Rot.RotateVector(LocalNormal);
	Contact.Penetration = NodeRadius - SignedDist; // 안쪽이면 SignedDist<0이라 면까지 깊이 + 노드 반지름
	Contact.SurfacePoint = Rot.RotateVector(LocalSurface) + Center;
	// Bone/SourceMesh/SurfaceVelocity: 정적 월드 지오메트리 — 기본값(None/null/0) 그대로.
	return Contact;
}

FBox FRopeBoxCollider::GetWorldBounds() const
{
	// OBB -> AABB: 축별 |회전 basis| · 반폭 합.
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
	if (Planes.Num() == 0)
	{
		return Contact;
	}
	// 브로드/질의 컬: 월드 AABB(+NodeRadius 여유) 밖이면 확실히 미접촉. 평면 루프 진입 전 조기 컷.
	if (!Bounds.IsValid || !Bounds.ExpandBy(NodeRadius).IsInsideOrOn(WorldPos))
	{
		return Contact;
	}

	// max-plane: 점이 가장 많이 위반한 평면(부호 거리 최대)이 표면 근사. 내부는 정확(모든 PlaneDot<0 →
	// 최대값이 곧 가장 가까운 면), 외부 엣지 근방은 과소추정(보수적). 그 평면의 법선이 push-out 방향.
	double MaxD = -DBL_MAX;
	int32 Best = INDEX_NONE;
	for (int32 i = 0; i < Planes.Num(); ++i)
	{
		const double D = Planes[i].PlaneDot(WorldPos); // dot(N,p) - W, N 바깥
		if (D > MaxD)
		{
			MaxD = D;
			Best = i;
		}
	}
	if (Best == INDEX_NONE || MaxD >= NodeRadius)
	{
		return Contact; // 어떤 면 밖으로 NodeRadius 이상 → 확실히 컨벡스 밖(미접촉).
	}

	Contact.bHit = true;
	// 법선: 최대 위반 평면의 단위 바깥 법선(빌드 시 정규화). FRopeContact FROZEN 계약(부호 load-bearing).
	Contact.Normal = FVector(Planes[Best].X, Planes[Best].Y, Planes[Best].Z);
	Contact.Penetration = NodeRadius - static_cast<float>(MaxD); // 안쪽이면 MaxD<0이라 더 큼.
	Contact.SurfacePoint = WorldPos - Contact.Normal * MaxD;      // 그 평면 위 최근접점(보조/디버그).
	// Bone/SourceMesh/SurfaceVelocity: 정적 월드 지오메트리 — 기본값(None/null/0) 그대로.
	return Contact;
}
