// Copyright Epic Games, Inc. All Rights Reserved.
//
// 최소 구현 skeletal collider provider: 매 프레임 skeletal mesh로부터 본별 capsule(bone -> parent 세그먼트
// 또는 Physics Asset 셰이프)을 생성하는 v1 collider 소스. per-bone SDF provider(URopeSDFProvider)와
// 같은 베이스(URopeSkeletalColliderProvider) 뒤에 있어 대상별로 선택해 쓴다 — 해석적 캡슐이라 베이크가
// 필요 없어 가볍다. 등록/메시 해석/프레임 디둡/gather 파이프라인은 베이스가 소유하고, 여기서는 캡슐
// 빌드(RebuildColliders)와 포인터 append(AppendColliderPointers)만 채운다.

#pragma once

#include "CoreMinimal.h"
#include "Collision/RopeSkeletalColliderProvider.h"
#include "Collision/RopeCollider.h"
#include "RopeBoneCapsuleProvider.generated.h"

class USkeletalMeshComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeBoneCapsuleProvider : public URopeSkeletalColliderProvider
{
	GENERATED_BODY()

public:
	/**
	 * capsule로 노출할 본들. 각 capsule은 해당 본에서 그 parent까지를 잇는다(반지름 = CapsuleRadius).
	 * 비워두면 자동 모드: 메시의 Physics Asset 바디(capsule/sphere/box 셰이프, 본별 실제 치수 —
	 * box는 장축 캡슐 근사)로 캡슐을 만든다. Physics Asset이 없거나 쓸 수 있는 셰이프가 하나도
	 * 없으면(convex 전용 등) 레퍼런스 스켈레톤의 모든 본-부모 세그먼트로 폴백한다(AutoMinBoneLength
	 * 미만 제외 — 단 IK/트위스트 본의 가짜 세그먼트가 섞일 수 있으니 캐릭터는 Physics Asset 권장).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TArray<FName> Bones;

	/** 각 본 세그먼트를 감싸는 capsule 반지름(cm). 명시 Bones/스켈레톤 폴백 경로에서만 쓴다(Physics Asset은 셰이프 반지름). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision", meta = (ClampMin = "0.0", Units = "cm"))
	float CapsuleRadius = 8.0f;

	/** 자동 모드의 스켈레톤 폴백에서 이 길이(cm) 미만의 본 세그먼트는 제외한다(손가락/트위스트 잡음 컷). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision|Advanced", meta = (ClampMin = "0.0", Units = "cm"))
	float AutoMinBoneLength = 5.0f;

protected:
	//~ URopeSkeletalColliderProvider
	virtual void RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt) override;
	virtual void AppendColliderPointers(FRopeColliderGatherContext& Gather) override;

private:
	/** 프레임당 1회 재구성되는 백킹 스토리지. 넘겨준 포인터들은 해당 프레임 동안 유효하다. */
	TArray<FCapsuleCollider> Capsules;

	/**
	 * 캡슐별(빌드 순서 인덱스 정렬) 이전 프레임 끝점(A, B). 표면 속도(드래그)/상대 운동 CCD의 prev 소스 —
	 * SDF provider의 PrevBoneToWorld 대응. 빌드 순서는 소스(Bones 목록/Physics Asset/스켈레톤)가 프레임 간
	 * 동일해 인덱스로 안정 — 개수가 바뀌면(구성 변경) 리셋하고 그 프레임은 정적(속도 0) 취급.
	 */
	TArray<TPair<FVector, FVector>> PrevEndpoints;

	/**
	 * 이번 프레임 캡슐 목록(A/B/반지름/본)을 Capsules에 빌드한다. Bones 명시 목록 → Physics Asset 자동 →
	 * 스켈레톤 폴백 순. prev 끝점/InvDt는 RebuildColliders가 인덱스 정렬로 이어 붙인다.
	 */
	void BuildCapsules(USkeletalMeshComponent* Mesh);
};
