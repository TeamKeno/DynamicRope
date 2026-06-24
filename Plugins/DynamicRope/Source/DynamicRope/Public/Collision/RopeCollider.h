// Copyright Epic Games, Inc. All Rights Reserved.
//
// Collider 추상화. solver는 IRopeCollider를 query할 뿐, 그것이 capsule인지 per-bone SDF인지
// 아니면 world distance field인지 전혀 알지 못한다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class USkeletalMeshComponent;

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

	/** broad-phase culling용 월드 공간 bounds. */
	virtual FBox GetWorldBounds() const = 0;
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
};
