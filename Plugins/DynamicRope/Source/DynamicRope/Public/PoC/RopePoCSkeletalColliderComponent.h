// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — 실험용이며 출시 대상 아님. PoC/ 아래의 모든 것은 폐기 가능한 코드다.
// S3 (3-4주차): rope-collision capsule을 캐릭터의 애니메이션되는 limb에서 가져온다.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PoC/RopeCapsuleProvider.h"
#include "RopePoCSkeletalColliderComponent.generated.h"

class USkeletalMeshComponent;

/**
 * skeletal mesh의 bone에서 가져온 capsule을 rope solver에 공급한다.
 * mesh의 Physics Asset body(capsule/sphere)를 읽어 매 프레임 월드 공간으로 변환하므로,
 * 별도 비용 없이 애니메이션된다. 캐릭터 액터에 붙이면 rope(bAutoFindColliders)가
 * 자동으로 이를 탐색한다.
 */
UCLASS(ClassGroup = (DynamicRopePoC), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopePoCSkeletalColliderComponent : public UActorComponent, public IRopeCapsuleProvider
{
	GENERATED_BODY()

public:
	URopePoCSkeletalColliderComponent();

	/** bone이 capsule을 만들어내는 skeletal mesh. null이면 owner의 첫 SkeletalMeshComponent를 사용. */
	UPROPERTY(EditAnywhere, Category = "Rope Collider")
	TObjectPtr<USkeletalMeshComponent> TargetMesh = nullptr;

	/** 이 bone들만 capsule을 만든다. 비우면 physics asset의 모든 body. */
	UPROPERTY(EditAnywhere, Category = "Rope Collider")
	TArray<FName> Bones;

	/** mesh의 Physics Asset에서 capsule을 읽는다(권장). 없으면 수동 bone-to-child capsule로 fallback. */
	UPROPERTY(EditAnywhere, Category = "Rope Collider")
	bool bUsePhysicsAsset = true;

	/** physics asset이 없을 때 수동 fallback에 사용되는 radius(cm). */
	UPROPERTY(EditAnywhere, Category = "Rope Collider", meta = (ClampMin = "0.1", UIMin = "0.1", Units = "cm"))
	float ManualRadius = 8.0f;

	/** 생성된 capsule을 매 프레임 그린다. */
	UPROPERTY(EditAnywhere, Category = "Rope Collider")
	bool bDrawDebug = false;

	//~ IRopeCapsuleProvider
	virtual void GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const override;

private:
	USkeletalMeshComponent* ResolveMesh() const;
	void GatherFromPhysicsAsset(USkeletalMeshComponent* Mesh, TArray<FRopeCapsule>& OutCapsules) const;
	void GatherManual(USkeletalMeshComponent* Mesh, TArray<FRopeCapsule>& OutCapsules) const;
	void DrawCapsules(const TArray<FRopeCapsule>& Capsules, int32 FirstNew) const;
};
