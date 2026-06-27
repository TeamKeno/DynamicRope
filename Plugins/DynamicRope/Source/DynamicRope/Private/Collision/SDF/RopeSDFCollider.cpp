// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFCollider.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFSampler.h"

FRopeContact FRopeSDFCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	FRopeContact Contact;

	if (!Volume || !Volume->IsBaked())
	{
		return Contact; // 미베이크/무효 볼륨 → 컨택트 없음.
	}

	// 월드 → 본 로컬 공간. grid는 본 로컬에 구워져 있다.
	const FVector LocalPos = BoneToWorld.InverseTransformPosition(WorldPos);

	// 좁은밴드 밖이면(노드 반지름 여유 포함) 빠르게 컬링.
	if (!Volume->LocalBounds.ExpandBy(NodeRadius).IsInsideOrOn(LocalPos))
	{
		return Contact;
	}

	// signed distance(바깥 +). 샘플링은 시각화와 공유하는 단일 진실 공급원(RopeSDFSampler)에 위임한다.
	// 노드 구체가 표면에 못 미치면 gradient는 계산조차 않고 빠진다(Query는 node×substep×iteration마다 호출).
	const float Dist = RopeSDFSampler::SampleTrilinear(*Volume, LocalPos);
	if (Dist >= NodeRadius)
	{
		return Contact;
	}

	// 바깥쪽 단위 법선(샘플러가 축퇴 시 +Z로 폴백). 본 로컬 → 월드(스케일 무시, 단위 유지).
	const FVector NLocal = RopeSDFSampler::SampleGradient(*Volume, LocalPos);

	Contact.bHit = true;
	Contact.Normal = BoneToWorld.TransformVectorNoScale(NLocal).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	Contact.Penetration = NodeRadius - Dist;                  // 양수: query 반지름 기준 겹침 깊이
	Contact.SurfacePoint = WorldPos - Contact.Normal * Dist;  // 표면 위 최근접점(보조/디버그)
	Contact.Bone = Bone;                                      // 본 귀속(DecideWrap dominant bone 입력, 비-None 필수)
	Contact.SourceMesh = SourceMesh;                          // 본을 소유한 메시(액터 간 wrap follow)

	// 표면 속도(cm/s): 지금 WorldPos에 있는 본 위의 물질점은 이전 프레임엔 PrevBoneToWorld 기준 같은
	// 로컬 좌표(LocalPos)에 있었다. (현재 - 이전) / dt 가 그 점의 월드 속도. solver가 상대 접선 속도
	// 마찰로 로프를 끌어 좌우로 쓸어내는 데 쓴다. InvDeltaTime==0(첫 프레임/정지)이면 0 → 기존 동작.
	if (InvDeltaTime > 0.0f)
	{
		const FVector PrevWorld = PrevBoneToWorld.TransformPosition(LocalPos);
		Contact.SurfaceVelocity = (WorldPos - PrevWorld) * InvDeltaTime;
	}
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

bool FRopeSDFCollider::GetGPUSDF(FRopeSDFColliderView& OutView) const
{
	if (!Volume || !Volume->IsBaked())
	{
		return false; // 미베이크/무효 볼륨은 GPU 충돌에서 제외(CPU Query와 동일 가드).
	}
	OutView.Distances    = Volume->Distances.GetData();
	OutView.ResX         = Volume->Resolution.X;
	OutView.ResY         = Volume->Resolution.Y;
	OutView.ResZ         = Volume->Resolution.Z;
	OutView.LocalMin     = Volume->LocalBounds.Min;
	OutView.LocalSize    = Volume->LocalBounds.GetSize();
	OutView.BoneToWorld  = BoneToWorld;
	OutView.VolumeKey    = Volume; // 프레임 내 동일 볼륨 업로드 dedup용 키.
	return true;
}
