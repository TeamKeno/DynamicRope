// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeSDFData를 IRopeCollider로 공급하는 provider. 본별 볼륨을 현재 본 월드 트랜스폼으로 변환해
// 매 프레임 FRopeSDFCollider를 빌드한다. URopeBoneCapsuleProvider와 동일 인터페이스라 캡슐과
// 공존/대체 가능(비블로킹). 미베이크 볼륨은 건너뛰므로 데이터가 비어도 안전한 no-op.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
#include "Collision/SDF/RopeSDFCollider.h"
#include "RopeSDFProvider.generated.h"

class URopeSDFData;
class USkeletalMeshComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeSDFProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeSDFProvider();

	/** 본별 SDF 볼륨 에셋. 비어 있으면 collider를 공급하지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<URopeSDFData> SDFData = nullptr;

	/** 본 트랜스폼을 제공하는 메시. null로 두면 owner에서 자동 해석된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<USkeletalMeshComponent> SkeletalMesh = nullptr;

	/** 빌드된 볼륨의 월드 bounds를 매 프레임 그린다(녹색 = rope bounds와 겹침). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	bool bDrawDebug = false;

	//~ IRopeColliderProvider
	virtual void GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders) override;

private:
	// GatherColliders마다 재구성되는 백킹 스토리지. 넘겨준 포인터는 해당 프레임 동안 유효하다.
	TArray<FRopeSDFCollider> Colliders;

	USkeletalMeshComponent* ResolveMesh();
};
