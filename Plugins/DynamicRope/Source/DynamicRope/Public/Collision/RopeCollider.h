// Copyright Epic Games, Inc. All Rights Reserved.
//
// Collider 추상화. solver는 IRopeCollider를 query할 뿐, 그것이 capsule인지 per-bone SDF인지
// 아니면 world distance field인지 전혀 알지 못한다.
//
// [GPU 계약 — 커스텀 collider 주의] 런타임 솔브는 GPU 단일 경로다. CPU 계약(Query/QuerySwept)만
// 구현한 커스텀 collider는 유닛 테스트/CPU 폴백(쿡·-nullrhi·노드 수 초과)에서는 동작하지만, GPU
// 스텝에는 GetGPUCapsule / GetGPUSDF / GetGPUBox / GetGPUConvex 중 하나를 구현해야 실린다 — 넷 다
// false면 GPU 솔브에서 제외되고 서브시스템(PackStepColliders)이 세션당 1회 경고를 남긴다.
// 패킹 규칙: 비-정적(스켈레탈)은 capsule/SDF만, 정적(IsWorldStatic)은 capsule/box/convex만.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

// 랩 대상 추상화(Decision 0): SourceMesh/귀속 mesh를 USceneComponent로 일반화(정적 opt-in 대비).
class USceneComponent;

/**
 * GPU 솔버(M3)용 SDF collider 뷰. 본 로컬 distance grid + 본→월드 트랜스폼을 런타임 타입 없이 노출한다.
 * Distances는 collider/asset 소유 포인터(해당 프레임 동안 유효). VolumeKey는 GPU 업로드 dedup용 식별자.
 * Distances는 uint8 양자화 코드 — 소비자가 비대칭 밴드로 dequant(d = code*(range/255) - NBInner,
 * range = NBInner+NBOuter, 바깥 +). 안쪽/바깥 밴드가 달라 offset은 -NBInner.
 */
struct FRopeSDFColliderView
{
	/** 코드 바이트 블롭(복셀당 BytesPerCode, 행 우선). 리틀엔디안. */
	const uint8* Distances = nullptr;

	/** 복셀당 바이트(1=uint8 max255, 2=uint16 max65535). */
	int32        BytesPerCode = 1;

	/** 안쪽 dequant 밴드(cm). 코드 0 → -NarrowBandInner. */
	float        NarrowBandInner = 0.0f;

	/** 바깥 dequant 밴드(cm). 코드 max → +NarrowBandOuter. */
	float        NarrowBandOuter = 0.0f;

	int32        ResX = 0;
	int32        ResY = 0;
	int32        ResZ = 0;

	/** 본 로컬 그리드 원점(LocalBounds.Min)과 크기. */
	FVector      LocalMin = FVector::ZeroVector;
	FVector      LocalSize = FVector::ZeroVector;

	FTransform   BoneToWorld = FTransform::Identity;

	/** 이전 프레임 본 트랜스폼(GPU CCD/표면속도 드래그용). */
	FTransform   PrevBoneToWorld = FTransform::Identity;

	/** 1/프레임dt(표면 속도 = (curr-prev)*InvDeltaTime). 0이면 정적. */
	float        InvDeltaTime = 0.0f;

	/** 같은 볼륨 dedup 식별자(보통 FRopeBoneSDFVolume*). */
	const void*  VolumeKey = nullptr;
};

/**
 * Swept(연속) collision 질의 파라미터. 노드가 이 substep에 WorldStart->WorldEnd로 움직이는 동안의
 * 첫 접촉을 찾는다. 움직이는 collider는 SubAlpha0/1로 프레임 모션(prev->curr)을 substep에 분배해
 * 노드-collider 상대 운동까지 본다(정지 collider는 알파 무시). 빠른 본이 정지한 로프를 추월할 때
 * 뒷면으로 밀어 관통시키는 대신 접근(앞)면에서 잡는 것이 목적.
 */
struct FRopeSweptQuery
{
	/** substep 시작/끝 노드 위치(PrevPos → Pos). */
	FVector WorldStart = FVector::ZeroVector;
	FVector WorldEnd   = FVector::ZeroVector;

	/** 로프 두께(query 반지름). */
	float   NodeRadius = 0.0f;

	/** 샘플 간격(cm). */
	float   SweepStep  = 2.0f;

	/** 구간당 샘플 상한. */
	int32   MaxSamples = 16;

	/**
	 * 움직이는 collider의 이 substep용 sub-포즈. solver가 GetFrameMotion으로 받은 prev/curr를 알파로 Blend해
	 * 콜라이더당 1회 미리 계산한다(노드 루프 밖 호이스팅 → 노드마다 Blend 재계산 방지). bUseSubPose=false면
	 * collider는 단일(현재) 포즈로 본다(정지 본/비-SDF). 공유 collider를 mutate하지 않으므로 병렬 솔브에 안전.
	 */
	bool       bUseSubPose  = false;
	FTransform SubPoseStart = FTransform::Identity;
	FTransform SubPoseEnd   = FTransform::Identity;

	/**
	 * 이 substep의 프레임 모션 구간 비율(0=이전 프레임, 1=현재 프레임). solver가 항상 채운다.
	 * 리지드 트랜스폼이 없는 collider(캡슐 — 두 관절점이 따로 움직임)가 sub-포즈 대신 자체 prev 상태를
	 * 직접 보간하는 데 쓴다(SubPoseStart/End의 트랜스폼-프리 대응물).
	 */
	float SubAlpha0 = 0.0f;
	float SubAlpha1 = 1.0f;
};

/** collider 표면 projection 결과. 접촉 판정과 달리, 주어진 점에서 가장 가까운 표면점을 기술한다. */
struct FRopeSurfaceProjection
{
	bool bHit = false;
	FVector SurfacePoint = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	float Distance = 0.0f;
	FName Bone = NAME_None;
	const USceneComponent* SourceMesh = nullptr;
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
	 * 표면 projection 전용 query. collision Query와 달리 "겹쳤는가"가 아니라 WorldPos에서 가까운 표면점을 찾는다.
	 * MaxDistance보다 멀면 false를 반환할 수 있다. wrapping path 생성처럼 표면에 계속 붙이는 용도로 사용한다.
	 */
	virtual FRopeSurfaceProjection ProjectToSurface(const FVector& WorldPos, float MaxDistance) const
	{
		FRopeSurfaceProjection Projection;
		const FRopeContact Contact = Query(WorldPos, MaxDistance);
		if (!Contact.bHit)
		{
			return Projection;
		}

		Projection.bHit = true;
		Projection.SurfacePoint = Contact.SurfacePoint;
		Projection.Normal = Contact.Normal;
		Projection.Distance = FVector::Dist(WorldPos, Contact.SurfacePoint);
		Projection.Bone = Contact.Bone;
		Projection.SourceMesh = Contact.SourceMesh;
		return Projection;
	}

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
	 * 이 collider가 정적 월드 지오메트리(스태틱 바디)인지. 기본 false(스켈레탈/동적).
	 * 정적 collider는 랩 대상이 아니므로 접촉 감지(detect) 파이프라인에서 제외해야 한다 —
	 * detect는 노드당 가장 깊은 접촉 1개만 남기므로, 벽 접촉이 본 접촉을 가리면 그 노드의
	 * 랩 캡처가 조용히 실패한다(GPU는 PackStepColliders의 2-pass 패킹, CPU는 flight detector
	 * 입력 필터가 이 플래그를 본다). solve(push-out)에는 정상 참여한다.
	 */
	virtual bool IsWorldStatic() const { return false; }

	/**
	 * GPU 솔버(M2)용: 이 collider가 해석적 capsule이면 월드 공간 세그먼트(A-B)와 반지름을 채우고 true.
	 * 기본은 false(미지원) — RTTI가 꺼져 있어 dynamic_cast 대신 이 가상 accessor로 capsule을 식별한다.
	 * SDF/기타 collider는 GPU capsule 경로에서 제외된다(M3에서 Texture3D SDF로 별도 처리).
	 */
	virtual bool GetGPUCapsule(FVector& OutA, FVector& OutB, float& OutRadius) const { return false; }

	/**
	 * GPU 캡슐의 프레임 모션: 이전 프레임 끝점 + 1/프레임dt. GetGPUCapsule=true인 collider만 의미 있다.
	 * 기본은 false(정적) — 호출자는 prev=현재 끝점, InvDt=0으로 폴백한다. 캡슐은 리지드 트랜스폼이 없어
	 * (두 관절점이 따로 움직임) GetFrameMotion 대신 끝점 쌍을 직접 넘긴다. GPU가 표면 속도(드래그)와
	 * substep 상대 운동 CCD에 쓴다(SDF의 PrevBoneToWorld/InvDeltaTime 대응).
	 */
	virtual bool GetGPUCapsuleMotion(FVector& OutPrevA, FVector& OutPrevB, float& OutInvDeltaTime) const { return false; }

	/**
	 * GPU 솔버(M3)용: 이 collider가 per-bone SDF면 grid/transform 뷰를 채우고 true. 기본은 false.
	 * 캡슐과 마찬가지로 RTTI 없이 SDF collider를 식별하는 경로다(GetGPUCapsule과 상호 배타적).
	 */
	virtual bool GetGPUSDF(FRopeSDFColliderView& OutView) const { return false; }

	/**
	 * GPU 솔버용: 이 collider가 해석적 박스(OBB)면 월드 공간 center/rot/half-extents를 채우고 true.
	 * 기본은 false. GetGPUCapsule/GetGPUSDF와 상호 배타적(같은 RTTI-프리 식별 패턴). 정적 월드
	 * 지오메트리용이라 프레임 모션이 없다 — GPU는 표면 속도 0(정적)으로 응답한다.
	 */
	virtual bool GetGPUBox(FVector& OutCenter, FQuat& OutRot, FVector& OutHalfExtents) const { return false; }

	/**
	 * GPU 박스의 프레임 모션: 이전 프레임 center/rot + 1/프레임dt. GetGPUBox=true인 collider만 의미 있다.
	 * 기본은 false(정적) — 호출자는 prev=현재, InvDt=0으로 폴백한다. 움직이는 정적 바디(플랫폼/문)가
	 * 로프를 끌고(표면 속도) substep CCD로 터널링을 막는 데 쓴다(캡슐의 GetGPUCapsuleMotion 대응).
	 */
	virtual bool GetGPUBoxMotion(FVector& OutPrevCenter, FQuat& OutPrevRot, float& OutInvDeltaTime) const { return false; }

	/**
	 * GPU 솔버용: 이 collider가 해석적 컨벡스면 바디-로컬 평면 집합(단위 법선·바깥, PlaneDot(p)=dot(N,p)-W,
	 * 스케일 반영·강체 트랜스폼 미적용)과 로컬 AABB, 그리고 바디의 강체 트랜스폼(curr rot/trans + prev)과
	 * 1/프레임dt를 채우고 true. 기본은 false. 월드 평면 = 로컬 평면 ∘ 강체(rot,trans). 움직이는 바디는
	 * prev 강체로 표면 속도/substep CCD를 처리한다(정적이면 prev=curr, InvDt=0). OutLocalPlanes는 collider
	 * 소유 스토리지 뷰(해당 프레임 동안 유효).
	 */
	virtual bool GetGPUConvex(TConstArrayView<FPlane>& OutLocalPlanes, FBox& OutLocalBounds,
		FQuat& OutRot, FVector& OutTrans, FQuat& OutPrevRot, FVector& OutPrevTrans, float& OutInvDeltaTime) const
	{
		return false;
	}

	/**
	 * 움직이는 collider의 이번 프레임 모션(prev->curr 월드 트랜스폼)을 채우고 true. 기본은 false(정적/모션없음).
	 * solver가 swept 충돌에서 substep별 sub-포즈를 노드 루프 밖에서 1회 계산하는 데 쓴다(상대 운동 CCD 호이스팅).
	 * 반드시 const(읽기 전용) — collider는 로프 간 공유되며 병렬 솔브된다.
	 */
	virtual bool GetFrameMotion(FTransform& OutPrev, FTransform& OutCurr) const { return false; }

	/**
	 * GPU 접촉 감지(G3)용 귀속(attribution): 이 collider가 어느 bone/mesh에 속하는지. GPU는 콜라이더
	 * 인덱스만 emit하므로, 호출자가 인덱스 → (bone, mesh)를 이걸로 복원한다. FRopeContact.Bone/SourceMesh와
	 * 동일 값이어야 한다(같은 판정 파이프라인에 먹인다). 기본은 None/null.
	 */
	virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const
	{
		OutBone = NAME_None;
		OutMesh = nullptr;
	}
};

/**
 * 해석적 capsule(swept-sphere 세그먼트). v1 본 collider — per-bone SDF(FRopeSDFCollider)와 같은
 * IRopeCollider 인터페이스 뒤에서 provider 단위로 선택된다(solver는 어느 쪽인지 모른다).
 */
class DYNAMICROPE_API FCapsuleCollider : public IRopeCollider
{
public:
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;
	FName   Bone = NAME_None;

	/**
	 * 이전 프레임 끝점 + 1/프레임dt. provider가 본별 prev 끝점을 캐시해 채운다(SDF provider의
	 * PrevBoneToWorld 대응). InvDeltaTime=0(기본)이면 정적 캡슐 — prev는 무시되고 기존 동작과 동일.
	 * 접촉 재질점은 세그먼트 파라미터(t)로 식별: prev 위치 = Lerp(PrevA, PrevB, t). 캡슐 축 자체의
	 * 스핀(자전)은 표현 못 하지만 본 캡슐에서는 무시 가능한 성분이다.
	 */
	FVector PrevA = FVector::ZeroVector;
	FVector PrevB = FVector::ZeroVector;
	float   InvDeltaTime = 0.0f;

	/**
	 * 이 capsule의 bone이 속한 mesh(스켈레탈). 컨택트로 전달되어 wrap이 액터를 넘어서도
	 * *올바른* mesh(잡힌 bone을 소유한 mesh)를 따라갈 수 있게 한다. 타입은 USceneComponent로
	 * 일반화(정적 opt-in 대비) — 캡슐은 스켈레탈만 넘긴다.
	 */
	const USceneComponent* SourceMesh = nullptr;

	FCapsuleCollider() = default;
	FCapsuleCollider(const FVector& InA, const FVector& InB, float InRadius, FName InBone = NAME_None,
		const USceneComponent* InSourceMesh = nullptr)
		: A(InA), B(InB), Radius(InRadius), Bone(InBone), PrevA(InA), PrevB(InB), SourceMesh(InSourceMesh) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FRopeSurfaceProjection ProjectToSurface(const FVector& WorldPos, float MaxDistance) const override;
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const override;
	virtual FBox GetWorldBounds() const override;
	virtual bool GetGPUCapsule(FVector& OutA, FVector& OutB, float& OutRadius) const override;
	virtual bool GetGPUCapsuleMotion(FVector& OutPrevA, FVector& OutPrevB, float& OutInvDeltaTime) const override;
	virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const override
	{
		OutBone = Bone;
		OutMesh = SourceMesh;
	}
};
