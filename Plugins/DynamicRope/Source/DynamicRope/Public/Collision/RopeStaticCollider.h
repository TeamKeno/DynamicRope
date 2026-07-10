// Copyright Epic Games, Inc. All Rights Reserved.
//
// 정적 월드 지오메트리(스태틱 바디의 심플 콜리전)용 해석적 collider들. GDF(Global Distance Field)는
// 복셀 클립맵이라 모서리가 복셀 크기만큼 둥글게 침식되어 로프가 박스 모서리를 타고 관통한다 —
// 여기의 박스(OBB) collider는 해석적 클램프 질의라 모서리/엣지에서 정확한 대각 normal을 준다.
// URopeStaticBodyProvider가 근접 정적 바디의 UBodySetup에서 추출해 서빙한다. GDF는 심플 콜리전이
// 없는 랜드스케이프/거대 메시용 far-field 폴백으로 강등된다.

#pragma once

#include "CoreMinimal.h"
#include "Collision/RopeCollider.h"

/**
 * 해석적 박스(OBB) collider. 기본은 정적 월드 지오메트리 — 프레임 모션 없음(SurfaceVelocity 0),
 * Bone=None/SourceMesh=null → IsWorldStatic()=true → 감지(detect) 제외(push-out 전용).
 * 랩 가능 박스(피드백 5번 박스 랩): Bone(가상 본)+SourceMesh(대상 컴포넌트)를 채우면 IsWorldStatic()=false가
 * 되어 감지에 참여하고, FRopeContact에 그 귀속을 실어 기존 DecideWrap 경로로 랩된다(FROZEN 계약 준용).
 */
class DYNAMICROPE_API FRopeBoxCollider : public IRopeCollider
{
public:
	FVector Center = FVector::ZeroVector;      // 월드 공간 박스 중심
	FQuat   Rot = FQuat::Identity;             // 월드 공간 박스 회전
	FVector HalfExtents = FVector::ZeroVector; // 로컬 반폭(스케일 반영 후)

	// 동적 바디 표면 속도용: 이전 프레임 center/rot + 1/프레임dt. provider가 움직이는 바디에 채운다.
	// InvDeltaTime=0(기본)이면 정적 — prev는 무시되고 기존 동작과 동일. 스케일은 프레임 간 불변 가정.
	FVector PrevCenter = FVector::ZeroVector;
	FQuat   PrevRot = FQuat::Identity;
	float   InvDeltaTime = 0.0f;

	// 랩 가능 박스: 비-None Bone(가상 본) + SourceMesh(대상 컴포넌트)면 랩 대상(감지 참여). 기본(None/null)이면
	// 정적 월드 push-out 전용(기존 URopeStaticBodyProvider 동작 — 감지 제외).
	FName Bone = NAME_None;
	const USceneComponent* SourceMesh = nullptr;

	FRopeBoxCollider() = default;
	FRopeBoxCollider(const FVector& InCenter, const FQuat& InRot, const FVector& InHalfExtents)
		: Center(InCenter), Rot(InRot), HalfExtents(InHalfExtents), PrevCenter(InCenter), PrevRot(InRot) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const override;
	virtual FBox GetWorldBounds() const override;
	// 가상 본이 있으면 랩 대상(감지 포함) → 비-정적. 없으면 정적 월드(push-out 전용, 감지 제외).
	virtual bool IsWorldStatic() const override { return Bone.IsNone(); }
	virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const override
	{
		OutBone = Bone;
		OutMesh = SourceMesh;
	}
	virtual bool GetGPUBox(FVector& OutCenter, FQuat& OutRot, FVector& OutHalfExtents) const override
	{
		OutCenter = Center;
		OutRot = Rot;
		OutHalfExtents = HalfExtents;
		return true;
	}
	virtual bool GetGPUBoxMotion(FVector& OutPrevCenter, FQuat& OutPrevRot, float& OutInvDeltaTime) const override
	{
		if (InvDeltaTime <= 0.0f)
		{
			return false; // 정적 — 호출자가 prev=현재, InvDt=0으로 폴백.
		}
		OutPrevCenter = PrevCenter;
		OutPrevRot = PrevRot;
		OutInvDeltaTime = InvDeltaTime;
		return true;
	}
	// ProjectToSurface: 기본 구현 그대로.
};

/**
 * 정적 캡슐/스피어(스피어 = A==B 축퇴 캡슐). FCapsuleCollider의 질의를 그대로 쓰되
 * IsWorldStatic()=true로 detect 제외 대상임을 표시한다. Bone/SourceMesh는 기본값(None/null),
 * InvDeltaTime=0(정적 — 표면 속도 0)을 유지할 것.
 */
class DYNAMICROPE_API FRopeStaticCapsuleCollider : public FCapsuleCollider
{
public:
	using FCapsuleCollider::FCapsuleCollider;

	virtual bool IsWorldStatic() const override { return true; }
};

/**
 * 해석적 컨벡스(평면 집합) collider. 정적 월드 지오메트리(스태틱 바디의 convex 심플 콜리전) 전용.
 * 질의는 max-plane: 점이 가장 많이 위반한 평면까지의 부호 거리를 침투 응답에 쓴다 — **내부에서는 정확**,
 * **외부 엣지/꼭짓점 근방에서는 거리를 과소추정**(무한 평면이 유한 엣지보다 가까우므로)해 접촉이 살짝
 * 이르게 걸린다(터널링 없는 보수적 동작이라 페널티 응답엔 충분). 박스처럼 정확한 최근접점(엣지/꼭짓점)
 * 계산은 인접 정보가 필요해 비싸므로, 정적 월드 충돌엔 이 근사가 표준(Obi 등과 동일).
 * FRopeContact FROZEN 계약: 비-스켈레탈이라 Bone=None, SourceMesh=null, SurfaceVelocity=0.
 */
class DYNAMICROPE_API FRopeConvexCollider : public IRopeCollider
{
public:
	// 바디-로컬 평면(단위 법선·바깥, 스케일 반영·강체 미적용). PlaneDot(p)=dot(N,p)-W: 로컬 내부는 모든 평면 <0.
	// 월드 평면 = 로컬 ∘ 강체(Rot,Trans). 강체만 프레임 간 움직이고 로컬 평면은 불변(스케일 불변 가정).
	TArray<FPlane> LocalPlanes;
	FBox LocalBounds = FBox(ForceInit); // 바디-로컬 AABB(질의 컬).

	// 바디의 강체 트랜스폼(컴포넌트 rot+trans). curr + 이전 프레임(동적 표면 속도/CCD). InvDeltaTime=0이면 정적.
	FQuat   Rot = FQuat::Identity;
	FVector Trans = FVector::ZeroVector;
	FQuat   PrevRot = FQuat::Identity;
	FVector PrevTrans = FVector::ZeroVector;
	float   InvDeltaTime = 0.0f;

	FRopeConvexCollider() = default;
	// 정적 편의 생성자: 로컬 평면 + 로컬 bounds + 강체(기본 identity → 월드=로컬). 테스트/정적 경로용.
	FRopeConvexCollider(TArray<FPlane>&& InLocalPlanes, const FBox& InLocalBounds,
		const FQuat& InRot = FQuat::Identity, const FVector& InTrans = FVector::ZeroVector)
		: LocalPlanes(MoveTemp(InLocalPlanes)), LocalBounds(InLocalBounds)
		, Rot(InRot), Trans(InTrans), PrevRot(InRot), PrevTrans(InTrans) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const override;
	virtual FBox GetWorldBounds() const override;
	virtual bool IsWorldStatic() const override { return true; }
	virtual bool GetGPUConvex(TConstArrayView<FPlane>& OutLocalPlanes, FBox& OutLocalBounds,
		FQuat& OutRot, FVector& OutTrans, FQuat& OutPrevRot, FVector& OutPrevTrans, float& OutInvDeltaTime) const override
	{
		OutLocalPlanes = LocalPlanes;
		OutLocalBounds = LocalBounds;
		OutRot = Rot; OutTrans = Trans;
		OutPrevRot = PrevRot; OutPrevTrans = PrevTrans;
		OutInvDeltaTime = InvDeltaTime;
		return true;
	}
	// ProjectToSurface: 기본 구현 그대로.
};
