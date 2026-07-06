// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeCollider.h"
#include "RopeMathHelpers.h" // RopeMath::ClosestSegmentParam (접촉 재질점 식별)

FRopeContact FCapsuleCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	// 노드 중심에서 캡슐 세그먼트(A-B)까지의 최근접점과 거리.
	const float   TSeg = RopeMath::ClosestSegmentParam(WorldPos, A, B);
	const FVector Closest = FMath::Lerp(A, B, TSeg);
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

	// 표면 속도(cm/s): 접촉 재질점(세그먼트 파라미터 TSeg)의 (현재 - 이전) / dt. solver가 상대 접선
	// 마찰로 로프를 끌어 쓸어내는 데 쓴다(SDF collider와 동일 계약). InvDeltaTime==0(정적/첫 프레임)이면 0.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevClosest = FMath::Lerp(PrevA, PrevB, TSeg);
		Contact.SurfaceVelocity = (Closest - PrevClosest) * InvDeltaTime;
	}
	return Contact;
}

FRopeContact FCapsuleCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// 정지 캡슐(첫 프레임 포함)은 기본(현재 포즈 정적 스윕)과 동일 — 조기 위임.
	if (InvDeltaTime <= 0.0f || (PrevA.Equals(A) && PrevB.Equals(B)))
	{
		return IRopeCollider::QuerySwept(Q, OutHitWorldPos);
	}

	FRopeContact Contact;
	OutHitWorldPos = Q.WorldEnd; // 기본값(미접촉 시 미정의 사용 방지).

	// 이 substep의 캡슐 끝점(프레임 모션 prev->curr를 SubAlpha로 보간). 캡슐은 리지드 트랜스폼이 없어
	// (두 관절점이 따로 움직임) SubPose 대신 끝점 자체를 보간한다 — SDF QuerySwept의 로컬 프레임 트릭 대응.
	const FVector CapAS = FMath::Lerp(PrevA, A, Q.SubAlpha0);
	const FVector CapBS = FMath::Lerp(PrevB, B, Q.SubAlpha0);
	const FVector CapAE = FMath::Lerp(PrevA, A, Q.SubAlpha1);
	const FVector CapBE = FMath::Lerp(PrevB, B, Q.SubAlpha1);

	// 상대 운동 반영 샘플 수: 노드 이동 + 캡슐 끝점 이동의 최대치(과대추정은 안전 — MaxSamples로 상한).
	const double RelLen = FVector::Dist(Q.WorldStart, Q.WorldEnd)
		+ FMath::Max(FVector::Dist(CapAS, CapAE), FVector::Dist(CapBS, CapBE));
	const float  Step = FMath::Max(Q.SweepStep, 0.1f);
	const int32  NumSamples = FMath::Clamp(1 + FMath::FloorToInt(RelLen / Step), 1, FMath::Max(1, Q.MaxSamples));
	const float  MinDist = Radius + Q.NodeRadius;

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

		// 첫 접촉. 접촉 재질점(TSeg)을 substep 끝 포즈로 이월해 노드를 표면과 함께 남은 모션만큼 옮긴다
		// (SDF가 로컬 접촉점을 끝 포즈로 재환산하는 것의 대응). 법선은 재질점 기준 상대 방향이라 그대로 유효.
		Contact.bHit = true;
		Contact.Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToNode / Dist) : FVector::UpVector;
		Contact.Penetration = MinDist - Dist;
		const FVector ClosestEnd = FMath::Lerp(CapAE, CapBE, TSeg);
		OutHitWorldPos = Pt + (ClosestEnd - Closest);
		Contact.SurfacePoint = ClosestEnd + Contact.Normal * Radius;
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;

		// 표면 속도: 재질점의 프레임 전체(prev->curr) 변위 / dt (Query와 동일 계약).
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
	// 월드 공간 세그먼트 + 반지름을 그대로 넘긴다. GPU 솔버가 CPU Query와 동일한 segment 최근접 push-out을 수행한다.
	OutA = A;
	OutB = B;
	OutRadius = Radius;
	return true;
}

bool FCapsuleCollider::GetGPUCapsuleMotion(FVector& OutPrevA, FVector& OutPrevB, float& OutInvDeltaTime) const
{
	// 이전 프레임 끝점 + InvDt. InvDeltaTime=0(정적/첫 프레임)이면 호출자가 prev=현재로 폴백하도록 false.
	if (InvDeltaTime <= 0.0f)
	{
		return false;
	}
	OutPrevA = PrevA;
	OutPrevB = PrevB;
	OutInvDeltaTime = InvDeltaTime;
	return true;
}
