// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFCollider.h"
#include "Collision/SDF/RopeSDFData.h"

FRopeContact FRopeSDFCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	if (!Volume || !Volume->IsBaked())
	{
		return Contact; // 미베이크 볼륨 → 컨택트 없음.
	}

	// 월드 → 본 로컬 공간. grid는 본 로컬에 구워져 있다.
	const FVector LocalPos = BoneToWorld.InverseTransformPosition(WorldPos);

	// 좁은밴드 밖이면(노드 반지름 여유 포함) 빠르게 컬링.
	if (!Volume->LocalBounds.ExpandBy(NodeRadius).IsInsideOrOn(LocalPos))
	{
		return Contact;
	}

	// TODO(B3): trilinear 샘플로 signed distance d를, central-difference로 gradient(=바깥 방향)를 구한다.
	//   d  = SampleTrilinear(*Volume, LocalPos)
	//   N  = normalize(gradient)                              // 축퇴 시 안정 폴백
	//   Contact.bHit        = d < NodeRadius
	//   Contact.Penetration = NodeRadius - d                  // query 반지름 기준 (FRopeContact 계약)
	//   Contact.Normal      = BoneToWorld.TransformVectorNoScale(N)  // 월드로, 바깥쪽 단위 유지
	//   Contact.Bone        = Bone;
	//   Contact.SourceMesh  = SourceMesh;                     // skeletal collider는 비-None 필수
	//   Contact.SurfacePoint = WorldPos - Contact.Normal * d; // 보조/디버그
	// 현재는 스캐폴딩 단계 → 컨택트 없음으로 반환(narrow-phase는 캡슐 경로가 담당).
	return Contact;
}

FBox FRopeSDFCollider::GetWorldBounds() const
{
	if (!Volume)
	{
		return FBox(ForceInit);
	}
	return Volume->LocalBounds.TransformBy(BoneToWorld);
}
