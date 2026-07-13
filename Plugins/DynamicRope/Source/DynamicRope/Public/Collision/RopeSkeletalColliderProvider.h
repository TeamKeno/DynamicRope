// Copyright Epic Games, Inc. All Rights Reserved.
//
// 스켈레탈 메시에서 매 프레임 per-bone collider를 공급하는 provider의 추상 베이스. 캡슐
// (URopeBoneCapsuleProvider)과 SDF(URopeSDFProvider)가 공유하던 배선 — 서브시스템 등록/해제,
// 메시 해석, 프레임당 1회 빌드 디둡, gather 파이프라인(append + region 매핑) — 을 여기로 모은다.
// 서브클래스는 자기 스토리지(캡슐/SDF collider 배열)와 빌드 소스만 채운다: RebuildColliders +
// AppendColliderPointers 두 훅.
//
// **범위: 스켈레탈 전용.** 월드/정적 provider(GDF·StaticBody)는 ResolveMesh/본/prev-transform
// 기계를 공유하지 않고 ProvidesWorldStaticColliders 계약이 다르므로 이 베이스를 상속하지 않는다 —
// IRopeColliderProvider 인터페이스를 직접 구현한다.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
#include "RopeSkeletalColliderProvider.generated.h"

class USkeletalMeshComponent;

UCLASS(Abstract)
class DYNAMICROPE_API URopeSkeletalColliderProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeSkeletalColliderProvider();

	//~ UActorComponent — RopeSimSubsystem 중앙 레지스트리에 등록/해제(프레임당 1회 중앙 gather).
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** 본들이 collider가 되는 mesh. null로 두면 owner로부터 자동으로 해석된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TObjectPtr<USkeletalMeshComponent> SkeletalMesh = nullptr;

	//~ IRopeColliderProvider — 공용 gather 파이프라인(프레임 디둡 + 서브클래스 빌드 + region 매핑).
	virtual void GatherColliders(FRopeColliderGatherContext& Gather) override;

protected:
	/** SkeletalMesh(비면 owner의 첫 USkeletalMeshComponent)를 해석해 캐시한다. */
	USkeletalMeshComponent* ResolveMesh();

	/**
	 * 이번 프레임 collider를 서브클래스 스토리지에 (재)빌드한다. 프레임당 1회만 호출된다(BuiltFrame 디둡).
	 * 서브클래스가 스토리지 Reset + prev-state(표면속도용) 갱신을 소유한다.
	 * @param Mesh   해석된 스켈레탈 메시(non-null 보장).
	 * @param InvDt  표면속도 산출용 1/frameDt(첫 프레임/정지 프레임은 0 = 속도 0).
	 *
	 * UObject는 CDO 생성 때문에 C++ 순수 가상(=0)을 가질 수 없어(추상 클래스 인스턴스화 불가) UE 관용구
	 * PURE_VIRTUAL을 쓴다 — 본문을 제공하되 잘못 호출되면 fatal. 실 인스턴스는 항상 서브클래스라 미발화.
	 */
	virtual void RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt)
		PURE_VIRTUAL(URopeSkeletalColliderProvider::RebuildColliders, );

	/** 서브클래스 스토리지의 collider 포인터를 Gather.Colliders에 append한다(reserve 포함, 프레임당 로프마다 호출). */
	virtual void AppendColliderPointers(FRopeColliderGatherContext& Gather)
		PURE_VIRTUAL(URopeSkeletalColliderProvider::AppendColliderPointers, );

	/**
	 * 빌드 가능한 데이터가 있는지(mesh 외). 기본 true(캡슐은 mesh만 있으면 충분). SDF는 SDFData 유무로
	 * 게이트해, 데이터가 없으면 프레임 빌드에 진입하지 않고 no-op한다(빈 collider 공급).
	 */
	virtual bool HasColliderData() const { return true; }

private:
	/** 마지막으로 collider를 빌드한 GFrameCounter. 같은 프레임에 여러 로프가 호출해도 재빌드 안 함(디둡). */
	uint64 BuiltFrame = static_cast<uint64>(-1);
};
