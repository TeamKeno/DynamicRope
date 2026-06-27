// Copyright Epic Games, Inc. All Rights Reserved.
//
// Collider 추상화. solver는 IRopeCollider를 query할 뿐, 그것이 capsule인지 per-bone SDF인지
// 아니면 world distance field인지 전혀 알지 못한다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class USkeletalMeshComponent;

/**
 * GPU 솔버(M3)용 SDF collider 뷰. 본 로컬 distance grid + 본→월드 트랜스폼을 런타임 타입 없이 노출한다.
 * Distances는 collider/asset 소유 포인터(해당 프레임 동안 유효). VolumeKey는 GPU 업로드 dedup용 식별자.
 */
struct FRopeSDFColliderView
{
	const float* Distances = nullptr; // 길이 ResX*ResY*ResZ, 행 우선(x + y*ResX + z*ResX*ResY), 바깥 +
	int32        ResX = 0;
	int32        ResY = 0;
	int32        ResZ = 0;
	FVector      LocalMin = FVector::ZeroVector;  // LocalBounds.Min
	FVector      LocalSize = FVector::ZeroVector; // LocalBounds 크기
	FTransform   BoneToWorld = FTransform::Identity;
	const void*  VolumeKey = nullptr; // 같은 볼륨 dedup 식별자(보통 FRopeBoneSDFVolume*)
};

/**
 * Swept(연속) collision 질의 파라미터. 노드가 이 substep에 WorldStart->WorldEnd로 움직이는 동안의
 * 첫 접촉을 찾는다. 움직이는 collider는 SubAlpha0/1로 프레임 모션(prev->curr)을 substep에 분배해
 * 노드-collider 상대 운동까지 본다(정지 collider는 알파 무시). 빠른 본이 정지한 로프를 추월할 때
 * 뒷면으로 밀어 관통시키는 대신 접근(앞)면에서 잡는 것이 목적.
 */
struct FRopeSweptQuery
{
	FVector WorldStart = FVector::ZeroVector; // substep 시작 노드 위치(PrevPos)
	FVector WorldEnd   = FVector::ZeroVector; // substep 끝 노드 위치(Pos)
	float   NodeRadius = 0.0f;                // 로프 두께(query 반지름)
	float   SweepStep  = 2.0f;                // 샘플 간격(cm)
	int32   MaxSamples = 16;                  // 구간당 샘플 상한

	// 움직이는 collider의 이 substep용 sub-포즈. solver가 GetFrameMotion으로 받은 prev/curr를 알파로 Blend해
	// 콜라이더당 1회 미리 계산한다(노드 루프 밖 호이스팅 → 노드마다 Blend 재계산 방지). bUseSubPose=false면
	// collider는 단일(현재) 포즈로 본다(정지 본/비-SDF). 공유 collider를 mutate하지 않으므로 병렬 솔브에 안전.
	bool       bUseSubPose  = false;
	FTransform SubPoseStart = FTransform::Identity;
	FTransform SubPoseEnd   = FTransform::Identity;
};

/** rope solver가 query하는 추상 collider. */
class DYNAMICROPE_API IRopeCollider
{
public:
	virtual ~IRopeCollider() = default;

	/**
	 * rope 노드 구체(center WorldPos, radius Radius)에 대한 최근접 표면 query.
	 * 해당 struct의 FROZEN contract에 따라 FRopeContact를 채운다. 반드시 const / 부작용 없음이어야 한다
	 * (node x substep x iteration마다 호출됨). Radius == 0 도 유효하다(solver push-out 경로).
	 */
	virtual FRopeContact Query(const FVector& WorldPos, float Radius) const = 0;

	/**
	 * Swept query: 노드의 substep 경로(+움직이는 collider의 상대 운동)를 따라 첫 접촉을 찾는다.
	 * 기본 구현은 정적 폴백 — collider 모션(알파)을 무시하고 WorldStart->WorldEnd 직선을 현재 포즈로
	 * 샘플한다(기존 solver 동작과 동일). 움직이는 SDF collider는 이를 override해 상대 운동을 반영한다.
	 * 첫 접촉 시 bHit=true, OutHitWorldPos = 그 접촉 지점의 노드 월드 위치. solver는
	 * OutHitWorldPos + Normal*Penetration 으로 노드를 배치한다.
	 */
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const
	{
		const double L = FVector::Dist(Q.WorldStart, Q.WorldEnd);
		const int32 NumSamples = FMath::Clamp(1 + FMath::FloorToInt(L / FMath::Max(Q.SweepStep, 0.1f)), 1, FMath::Max(1, Q.MaxSamples));
		for (int32 k = 0; k < NumSamples; ++k)
		{
			const double T = (NumSamples <= 1) ? 1.0 : static_cast<double>(k) / static_cast<double>(NumSamples - 1);
			const FVector P = FMath::Lerp(Q.WorldStart, Q.WorldEnd, T);
			const FRopeContact Contact = Query(P, Q.NodeRadius);
			if (Contact.bHit)
			{
				OutHitWorldPos = P;
				return Contact;
			}
		}
		return FRopeContact();
	}

	/** broad-phase culling용 월드 공간 bounds. */
	virtual FBox GetWorldBounds() const = 0;

	/**
	 * GPU 솔버(M2)용: 이 collider가 해석적 capsule이면 월드 공간 세그먼트(A-B)와 반지름을 채우고 true.
	 * 기본은 false(미지원) — RTTI가 꺼져 있어 dynamic_cast 대신 이 가상 accessor로 capsule을 식별한다.
	 * SDF/기타 collider는 GPU capsule 경로에서 제외된다(M3에서 Texture3D SDF로 별도 처리).
	 */
	virtual bool GetGPUCapsule(FVector& OutA, FVector& OutB, float& OutRadius) const { return false; }

	/**
	 * GPU 솔버(M3)용: 이 collider가 per-bone SDF면 grid/transform 뷰를 채우고 true. 기본은 false.
	 * 캡슐과 마찬가지로 RTTI 없이 SDF collider를 식별하는 경로다(GetGPUCapsule과 상호 배타적).
	 */
	virtual bool GetGPUSDF(FRopeSDFColliderView& OutView) const { return false; }

	/**
	 * 움직이는 collider의 이번 프레임 모션(prev->curr 월드 트랜스폼)을 채우고 true. 기본은 false(정적/모션없음).
	 * solver가 swept 충돌에서 substep별 sub-포즈를 노드 루프 밖에서 1회 계산하는 데 쓴다(상대 운동 CCD 호이스팅).
	 * 반드시 const(읽기 전용) — collider는 로프 간 공유되며 병렬 솔브된다.
	 */
	virtual bool GetFrameMotion(FTransform& OutPrev, FTransform& OutCurr) const { return false; }
};

/** 해석적 capsule(swept-sphere 세그먼트). v1 / fallback. 추후 per-bone SDF로 대체된다. */
class DYNAMICROPE_API FCapsuleCollider : public IRopeCollider
{
public:
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;
	FName   Bone = NAME_None;

	// 이 capsule의 bone이 속한 skeletal mesh. 컨택트로 전달되어 wrap이 액터를 넘어서도
	// *올바른* mesh(잡힌 bone을 소유한 mesh)를 따라갈 수 있게 한다.
	const USkeletalMeshComponent* SourceMesh = nullptr;

	FCapsuleCollider() = default;
	FCapsuleCollider(const FVector& InA, const FVector& InB, float InRadius, FName InBone = NAME_None,
		const USkeletalMeshComponent* InSourceMesh = nullptr)
		: A(InA), B(InB), Radius(InRadius), Bone(InBone), SourceMesh(InSourceMesh) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FBox GetWorldBounds() const override;
	virtual bool GetGPUCapsule(FVector& OutA, FVector& OutB, float& OutRadius) const override;
};
