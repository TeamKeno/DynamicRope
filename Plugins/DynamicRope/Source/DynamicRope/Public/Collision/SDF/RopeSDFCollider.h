// Copyright Epic Games, Inc. All Rights Reserved.
//
// 본별 SDF 볼륨 하나를 IRopeCollider로 감싸는 콜라이더. 본 로컬 grid를 현재 본 월드 트랜스폼으로
// 변환해 solver의 node query에 응답한다. FCapsuleCollider의 SDF 대응물이며, 동일한 FROZEN
// FRopeContact 계약을 따른다(Normal 바깥쪽 단위, Penetration은 query 반지름 기준, Bone 비-None).

#pragma once

#include "CoreMinimal.h"
#include "Collision/RopeCollider.h"

struct FRopeBoneSDFVolume;
class USkeletalMeshComponent;

/** 본 로컬 SDF 볼륨 1개에 대한 해석적 collider. v1 / 캡슐과 동일 인터페이스. */
class DYNAMICROPE_API FRopeSDFCollider : public IRopeCollider
{
public:
	// 본 로컬 distance grid. 해당 프레임 solve 동안 유효한 포인터(provider가 소유).
	const FRopeBoneSDFVolume* Volume = nullptr;

	// 본 → 월드 트랜스폼(grid를 월드에 배치). 매 프레임 메시에서 갱신.
	FTransform BoneToWorld = FTransform::Identity;

	// 이전 프레임의 본 → 월드 트랜스폼. 표면 속도(드래그) 산출용. 첫 프레임엔 BoneToWorld와 동일(속도 0).
	FTransform PrevBoneToWorld = FTransform::Identity;

	// 1/프레임dt. 표면 변위를 속도(cm/s)로 환산. 0이면 표면 속도 0(정적 취급).
	float InvDeltaTime = 0.0f;

	// 이 볼륨이 귀속된 본. FRopeContact.Bone으로 전파된다.
	FName Bone = NAME_None;

	// 본을 소유한 메시. cross-actor follow를 위해 contact로 전달된다.
	const USkeletalMeshComponent* SourceMesh = nullptr;

	FRopeSDFCollider() = default;
	FRopeSDFCollider(const FRopeBoneSDFVolume* InVolume, const FTransform& InBoneToWorld,
		const FTransform& InPrevBoneToWorld, float InInvDeltaTime,
		FName InBone, const USkeletalMeshComponent* InSourceMesh)
		: Volume(InVolume), BoneToWorld(InBoneToWorld), PrevBoneToWorld(InPrevBoneToWorld)
		, InvDeltaTime(InInvDeltaTime), Bone(InBone), SourceMesh(InSourceMesh) {}

	virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override;
	virtual FRopeSurfaceProjection ProjectToSurface(const FVector& WorldPos, float MaxDistance) const override;
	// 상대 운동 swept query: prev/curr 본 트랜스폼을 substep 알파로 보간해, 노드 경로를 collider
	// 로컬 상대 프레임에서 샘플한다(움직이는 본이 노드를 앞면에서 잡음). 표면 속도도 함께 채운다.
	virtual FRopeContact QuerySwept(const FRopeSweptQuery& Q, FVector& OutHitWorldPos) const override;
	virtual FBox GetWorldBounds() const override;
	virtual bool GetGPUSDF(FRopeSDFColliderView& OutView) const override;
	// 이번 프레임 본 모션(prev->curr). solver가 substep sub-포즈를 호이스팅하는 데 쓴다.
	virtual bool GetFrameMotion(FTransform& OutPrev, FTransform& OutCurr) const override
	{
		OutPrev = PrevBoneToWorld; OutCurr = BoneToWorld; return true;
	}
	virtual void GetGPUAttribution(FName& OutBone, const USkeletalMeshComponent*& OutMesh) const override
	{
		OutBone = Bone;
		OutMesh = SourceMesh;
	}
};
