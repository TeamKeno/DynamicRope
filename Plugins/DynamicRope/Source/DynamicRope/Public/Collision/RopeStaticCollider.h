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
 * 해석적 박스(OBB) collider. 정적 월드 지오메트리 전용 — 프레임 모션 없음(SurfaceVelocity 0).
 * FRopeContact FROZEN 계약: 비-스켈레탈이므로 Bone=NAME_None, SourceMesh=null.
 * IsWorldStatic()=true — 랩 대상이 아니므로 접촉 감지(detect) 파이프라인에서 제외된다.
 */
class DYNAMICROPE_API FRopeBoxCollider : public IRopeCollider
{
public:
	FVector Center = FVector::ZeroVector;      // 월드 공간 박스 중심
	FQuat   Rot = FQuat::Identity;             // 월드 공간 박스 회전
	FVector HalfExtents = FVector::ZeroVector; // 로컬 반폭(스케일 반영 후)

	FRopeBoxCollider() = default;
	FRopeBoxCollider(const FVector& InCenter, const FQuat& InRot, const FVector& InHalfExtents)
		: Center(InCenter), Rot(InRot), HalfExtents(InHalfExtents) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FBox GetWorldBounds() const override;
	virtual bool IsWorldStatic() const override { return true; }
	virtual bool GetGPUBox(FVector& OutCenter, FQuat& OutRot, FVector& OutHalfExtents) const override
	{
		OutCenter = Center;
		OutRot = Rot;
		OutHalfExtents = HalfExtents;
		return true;
	}
	// QuerySwept/ProjectToSurface: 기본 구현 그대로 — 정적이라 현재 포즈 라인 샘플 폴백이 정확하다.
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
	// 월드 공간 평면(단위 법선·바깥 방향). PlaneDot(p)=dot(N,p)-W: 내부는 모든 평면에서 <0.
	TArray<FPlane> Planes;
	FBox Bounds = FBox(ForceInit); // 월드 AABB(브로드페이즈 + 질의 컬).

	FRopeConvexCollider() = default;
	FRopeConvexCollider(TArray<FPlane>&& InPlanes, const FBox& InBounds)
		: Planes(MoveTemp(InPlanes)), Bounds(InBounds) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FBox GetWorldBounds() const override { return Bounds; }
	virtual bool IsWorldStatic() const override { return true; }
	virtual bool GetGPUConvex(TConstArrayView<FPlane>& OutPlanes, FBox& OutBounds) const override
	{
		OutPlanes = Planes;
		OutBounds = Bounds;
		return true;
	}
	// QuerySwept/ProjectToSurface: 기본 구현 그대로(정적 → 현재 포즈 라인 샘플 폴백).
};
