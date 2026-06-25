// Copyright Epic Games, Inc. All Rights Reserved.
//
// 최소 구현 IRopeColliderProvider: 매 프레임 skeletal mesh로부터 나열된 본마다 capsule(bone -> parent 세그먼트)을
// 생성한다. contact/wrap 테스트용 v1 collider 소스이며, 이후 동일 인터페이스 뒤에서 per-bone
// SDF provider로 교체된다(M2-SDF).

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
#include "Collision/RopeCollider.h"
#include "RopeBoneCapsuleProvider.generated.h"

class USkeletalMeshComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeBoneCapsuleProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeBoneCapsuleProvider();

	/** 본들이 collider가 되는 mesh. null로 두면 owner로부터 자동으로 해석된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<USkeletalMeshComponent> SkeletalMesh = nullptr;

	/** capsule로 노출할 본들. 각 capsule은 해당 본에서 그 parent까지를 잇는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TArray<FName> Bones;

	/** 각 본 세그먼트를 감싸는 capsule 반지름(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision", meta = (ClampMin = "0.0", Units = "cm"))
	float CapsuleRadius = 8.0f;

	/** 생성된 본 capsule들을 매 프레임 그린다(녹색 = rope bounds와 겹침, 회색 = 컬링됨). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	bool bDrawDebug = false;

	//~ IRopeColliderProvider
	virtual void GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders) override;

private:
	// 프레임당 1회 재구성되는 백킹 스토리지. 넘겨준 포인터들은 해당 프레임 동안 유효하다.
	TArray<FCapsuleCollider> Capsules;

	// 마지막으로 capsule을 빌드한 GFrameCounter. 같은 프레임에 여러 로프가 호출해도 재빌드 안 함(디둡).
	uint64 BuiltFrame = static_cast<uint64>(-1);

	USkeletalMeshComponent* ResolveMesh();
};
