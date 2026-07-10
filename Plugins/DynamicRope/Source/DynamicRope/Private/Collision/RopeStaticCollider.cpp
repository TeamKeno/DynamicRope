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
	Contact.Bone = Bone;             // 랩 가능 박스면 가상 본(감지 귀속). 정적이면 None.
	Contact.SourceMesh = SourceMesh; // 랩 가능 박스면 대상 컴포넌트. 정적이면 null.
	// 표면 속도: 접촉 재질점(로컬 LocalSurface)의 (현재 포즈 - 이전 포즈) / dt. 움직이는 바디가 로프를
	// 접선 방향으로 끄는 데 쓴다. InvDeltaTime==0(정적/첫 프레임)이면 0.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevWorld = PrevRot.RotateVector(LocalSurface) + PrevCenter;
		Contact.SurfaceVelocity = (Contact.SurfacePoint - PrevWorld) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FRopeBoxCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// 정지 박스(첫 프레임 포함)는 기본(현재 포즈 정적 스윕)과 동일 — 조기 위임.
	if (InvDeltaTime <= 0.0f || (PrevCenter.Equals(Center) && PrevRot.Equals(Rot)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	OutHitWorldPos = Q.WorldEnd;

	// 이 substep의 박스 sub-포즈(프레임 모션 prev->curr를 SubAlpha로 보간). 노드 경로와 박스 모션을 함께 스윕.
	const FVector CenterS = FMath::Lerp(PrevCenter, Center, Q.SubAlpha0);
	const FVector CenterE = FMath::Lerp(PrevCenter, Center, Q.SubAlpha1);
	const FQuat   RotS = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha0);
	const FQuat   RotE = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha1);

	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd) + FVector::Dist(CenterS, CenterE);
	const float  Step = FMath::Max(Q.SweepStep, 0.1f);
	const int32  NumSamples = FMath::Clamp(1 + FMath::FloorToInt(RelLen / Step), 1, FMath::Max(1, Q.MaxSamples));

	for (int32 k = 0; k < NumSamples; ++k)
	{
		const float T = (NumSamples <= 1) ? 1.0f : static_cast<float>(k) / static_cast<float>(NumSamples - 1);
		const FVector Pt = FMath::Lerp(Q.WorldStart, Q.WorldEnd, T);
		// sub-포즈 박스(center/rot 보간)로 점질의. 임시 collider를 만들어 로컬 clamp 질의를 재사용한다.
		FRopeBoxCollider BoxT(FMath::Lerp(CenterS, CenterE, T), FQuat::Slerp(RotS, RotE, T), HalfExtents);
		const FRopeContact C = BoxT.Query(Pt, Q.NodeRadius); // BoxT는 InvDt=0 → 표면 속도 0(여기선 미사용)
		if (!C.bHit)
		{
			continue;
		}
		// 접촉 재질점(로컬)을 substep 끝 포즈로 이월 — 노드를 표면과 함께 남은 모션만큼 옮긴다.
		const FVector Lp = BoxT.Rot.UnrotateVector(C.SurfacePoint - BoxT.Center);
		const FVector ClosestEnd = RotE.RotateVector(Lp) + CenterE;
		Contact.bHit = true;
		Contact.Normal = C.Normal;
		Contact.Penetration = C.Penetration;
		OutHitWorldPos = Pt + (ClosestEnd - C.SurfacePoint);
		Contact.SurfacePoint = ClosestEnd;
		Contact.Bone = Bone;             // 랩 가능 박스면 가상 본(감지 귀속). 정적이면 None.
		Contact.SourceMesh = SourceMesh; // 랩 가능 박스면 대상 컴포넌트. 정적이면 null.
		// 표면 속도: 재질점의 프레임 전체(prev->curr) 변위 / dt.
		const FVector WCurr = Rot.RotateVector(Lp) + Center;
		const FVector WPrev = PrevRot.RotateVector(Lp) + PrevCenter;
		Contact.SurfaceVelocity = (WCurr - WPrev) * InvDeltaTime;
		break;
	}
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
	if (LocalPlanes.Num() == 0)
	{
		return Contact;
	}
	// 월드 -> 바디-로컬(강체 역): Lp = qInv*(p - Trans). 이후 로컬 평면/로컬 bounds로 질의.
	const FVector Lp = Rot.UnrotateVector(WorldPos - Trans);
	if (!LocalBounds.IsValid || !LocalBounds.ExpandBy(NodeRadius).IsInsideOrOn(Lp))
	{
		return Contact; // 로컬 AABB(+NodeRadius) 밖 → 확실히 미접촉.
	}

	// max-plane(로컬): 점이 가장 많이 위반한 평면(부호 거리 최대)이 표면 근사. 내부는 정확, 외부 엣지 근방은
	// 과소추정(보수적). 그 평면의 로컬 법선을 월드로 회전한 것이 push-out 방향.
	double MaxD = -DBL_MAX;
	int32 Best = INDEX_NONE;
	for (int32 i = 0; i < LocalPlanes.Num(); ++i)
	{
		const double D = LocalPlanes[i].PlaneDot(Lp); // dot(N,p) - W (로컬), N 바깥
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

	const FVector LocalNormal(LocalPlanes[Best].X, LocalPlanes[Best].Y, LocalPlanes[Best].Z);
	const FVector LocalSurface = Lp - LocalNormal * MaxD; // 로컬 표면점(재질점).
	Contact.bHit = true;
	Contact.Normal = Rot.RotateVector(LocalNormal);           // 월드 바깥 법선(FROZEN 계약, 부호 load-bearing).
	Contact.Penetration = NodeRadius - static_cast<float>(MaxD);
	Contact.SurfacePoint = Rot.RotateVector(LocalSurface) + Trans;
	// 표면 속도: 재질점(로컬)의 (현재 포즈 - 이전 포즈) / dt. 움직이는 바디가 로프를 접선 방향으로 끈다.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevWorld = PrevRot.RotateVector(LocalSurface) + PrevTrans;
		Contact.SurfaceVelocity = (Contact.SurfacePoint - PrevWorld) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FRopeConvexCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// 정지 컨벡스(첫 프레임 포함)는 기본(현재 포즈 정적 스윕)과 동일 — 조기 위임.
	if (InvDeltaTime <= 0.0f || (PrevTrans.Equals(Trans) && PrevRot.Equals(Rot)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	OutHitWorldPos = Q.WorldEnd;

	// substep sub-포즈 강체(prev->curr 보간).
	const FVector TransS = FMath::Lerp(PrevTrans, Trans, Q.SubAlpha0);
	const FVector TransE = FMath::Lerp(PrevTrans, Trans, Q.SubAlpha1);
	const FQuat   RotS = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha0);
	const FQuat   RotE = FQuat::Slerp(PrevRot, Rot, Q.SubAlpha1);

	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd) + FVector::Dist(TransS, TransE);
	const float  Step = FMath::Max(Q.SweepStep, 0.1f);
	const int32  NumSamples = FMath::Clamp(1 + FMath::FloorToInt(RelLen / Step), 1, FMath::Max(1, Q.MaxSamples));

	for (int32 k = 0; k < NumSamples; ++k)
	{
		const float T = (NumSamples <= 1) ? 1.0f : static_cast<float>(k) / static_cast<float>(NumSamples - 1);
		const FVector Pt = FMath::Lerp(Q.WorldStart, Q.WorldEnd, T);
		const FVector TransT = FMath::Lerp(TransS, TransE, T);
		const FQuat   RotT = FQuat::Slerp(RotS, RotE, T);
		// sub-포즈 로컬 점질의(강체 RotT/TransT로 로컬 변환).
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
		// 재질점을 끝 sub-포즈로 이월.
		const FVector ClosestT = RotT.RotateVector(LocalSurface) + TransT;
		const FVector ClosestEnd = RotE.RotateVector(LocalSurface) + TransE;
		Contact.bHit = true;
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
	// 로컬 AABB를 강체(Rot,Trans)로 변환 → 월드 AABB(브로드페이즈). 무효면 그대로.
	if (!LocalBounds.IsValid)
	{
		return LocalBounds;
	}
	return LocalBounds.TransformBy(FTransform(Rot, Trans));
}
