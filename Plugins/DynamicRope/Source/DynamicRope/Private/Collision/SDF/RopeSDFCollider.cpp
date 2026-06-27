// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFCollider.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFSampler.h"
#include "ProfilingDebugging/CpuProfilerTrace.h" // TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)

FRopeContact FRopeSDFCollider::Query(const FVector& WorldPos, float NodeRadius) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSDF_Query); // 점 query(접촉 감지/wrap 경로). solver 충돌은 QuerySwept 사용.
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

FRopeContact FRopeSDFCollider::QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
{
	// solver 충돌의 주 비용 지점(정지 로프 + 접촉 시 여기로 몰린다). 호출당 비용 = 포즈 Blend×2 +
	// 역변환 + 샘플 루프. 아래 RopeSDF_SweptSampleLoop와의 차이가 transform 셋업 비용이다.
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSDF_QuerySwept);
	FRopeContact Contact;
	OutHitWorldPos = Q.WorldEnd; // 기본값(미접촉 시 미정의 사용 방지).
	if (!Volume || !Volume->IsBaked())
	{
		return Contact;
	}

	// 이 substep이 차지하는 collider sub-포즈(프레임 모션 prev->curr 를 substep에 분배).
	FTransform PoseStart; PoseStart.Blend(PrevBoneToWorld, BoneToWorld, FMath::Clamp(Q.SubAlpha0, 0.0f, 1.0f));
	FTransform PoseEnd;   PoseEnd.Blend(PrevBoneToWorld, BoneToWorld, FMath::Clamp(Q.SubAlpha1, 0.0f, 1.0f));

	// 노드 substep 경로를 collider 로컬 상대 프레임으로: 시작은 시작 sub-포즈, 끝은 끝 sub-포즈 기준.
	// 이 하나의 로컬 세그먼트가 노드 모션 + collider 모션(상대 운동)을 모두 담는다 → 빠른 본이 노드를
	// 추월해도 로컬에서는 노드가 표면을 가로지르므로 첫 접촉(앞면)에서 잡힌다.
	const FVector L0 = PoseStart.InverseTransformPosition(Q.WorldStart);
	const FVector L1 = PoseEnd.InverseTransformPosition(Q.WorldEnd);

	// 상대 변위 기반 샘플 수(정지 로프 + 빠른 본도 충분히 샘플 → 추월 관통 방지).
	const double RelLen = FVector::Dist(L0, L1);
	const float  Step = FMath::Max(Q.SweepStep, 0.1f);
	const int32  NumSamples = FMath::Clamp(1 + FMath::FloorToInt(RelLen / Step), 1, FMath::Max(1, Q.MaxSamples));

	const FBox Band = Volume->LocalBounds.ExpandBy(Q.NodeRadius);

	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSDF_SweptSampleLoop); // SDF 샘플(trilinear/gradient) 비용. QuerySwept과의 차이 = transform 셋업(Blend×2+역변환).
	for (int32 k = 0; k < NumSamples; ++k)
	{
		const double T = (NumSamples <= 1) ? 1.0 : static_cast<double>(k) / static_cast<double>(NumSamples - 1);
		const FVector Lp = FMath::Lerp(L0, L1, T);
		if (!Band.IsInsideOrOn(Lp))
		{
			continue; // 좁은밴드(볼륨 + 노드반경) 밖 → 접촉 없음.
		}

		const float Dist = RopeSDFSampler::SampleTrilinear(*Volume, Lp);
		if (Dist >= Q.NodeRadius)
		{
			continue; // 아직 표면에 못 미침.
		}

		// 첫 접촉. 법선/위치는 substep 끝 포즈(노드가 도달하는 현재 프레임) 기준으로 환산한다.
		const FVector NLocal = RopeSDFSampler::SampleGradient(*Volume, Lp);
		Contact.bHit = true;
		Contact.Normal = PoseEnd.TransformVectorNoScale(NLocal).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		Contact.Penetration = Q.NodeRadius - Dist;
		OutHitWorldPos = PoseEnd.TransformPosition(Lp);              // 노드 배치 기준점(현재 포즈 월드)
		Contact.SurfacePoint = OutHitWorldPos - Contact.Normal * Dist;
		Contact.Bone = Bone;
		Contact.SourceMesh = SourceMesh;

		// 표면 속도(드래그): 접촉 물질점(Lp)의 prev->curr 프레임 변위 / dt.
		if (InvDeltaTime > 0.0f)
		{
			const FVector WCurr = BoneToWorld.TransformPosition(Lp);
			const FVector WPrev = PrevBoneToWorld.TransformPosition(Lp);
			Contact.SurfaceVelocity = (WCurr - WPrev) * InvDeltaTime;
		}
		break;
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
