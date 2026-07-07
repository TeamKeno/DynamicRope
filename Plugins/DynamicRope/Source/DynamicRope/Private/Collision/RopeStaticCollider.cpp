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
