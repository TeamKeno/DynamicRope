// Copyright Epic Games, Inc. All Rights Reserved.
//
// 정적 월드 지오메트리용 IRopeColliderProvider: 매 프레임 로프 활성 영역(전 로프 union AABB)을
// ECC_WorldStatic 오버랩으로 스캔해, 근접 정적 바디의 심플 콜리전(스피어/캡슐/박스)을 해석적
// collider로 추출한다. GDF(복셀 필드)가 뭉개던 박스 모서리를 해석적 질의로 정확히 처리하는 것이
// 목적 — GDF는 심플 콜리전이 없는 랜드스케이프/거대 메시용 far-field 폴백으로 남는다.
//
// 배치: 월드당 1개면 충분(어느 지속 액터든 무방 — 게임모드/레벨 액터/로프 소유 액터).
// ProvidesWorldStaticColliders()=true라 로프 소유 액터에 붙여도 소유자 제외에 걸리지 않는다.
// 컨벡스는 M2에서 평면 집합 collider로 추가 예정(현재는 스킵 + 로그). ISM/HISM은 v1 범위 밖.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
#include "Collision/RopeStaticCollider.h"
#include "RopeStaticBodyProvider.generated.h"

class UBodySetup;
class UPrimitiveComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeStaticBodyProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeStaticBodyProvider();

	//~ UActorComponent — RopeSimSubsystem 중앙 레지스트리에 등록/해제(프레임당 1회 중앙 gather).
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/**
	 * 프레임당 수집할 프리미티브 콜라이더 수 상한(예산). GPU solve 커널이 노드×substep마다 콜라이더
	 * 전량을 루프하므로 밀집 씬에서의 폭주를 막는다. 초과분은 버려지고 Verbose 로그를 남긴다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision", meta = (ClampMin = "1"))
	int32 MaxColliders = 128;

	/** 수집에서 제외할 컴포넌트(예: 로프가 의도적으로 통과해야 하는 지오메트리). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TArray<TObjectPtr<UPrimitiveComponent>> IgnoredComponents;

	//~ IRopeColliderProvider
	virtual void GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders) override;
	virtual bool ProvidesWorldStaticColliders() const override { return true; }

private:
	// 프레임당 1회 재구성되는 백킹 스토리지. 넘겨준 포인터들은 해당 프레임 동안 유효하다.
	TArray<FRopeBoxCollider> Boxes;
	TArray<FRopeStaticCapsuleCollider> Capsules;

	// 마지막으로 빌드한 GFrameCounter. 같은 프레임에 여러 로프가 호출해도 재빌드 안 함(디둡).
	// 단 이 provider는 RopeBounds(전 로프 union — 서브시스템이 프레임당 동일 값 전달)를 실제로 쓴다.
	uint64 BuiltFrame = static_cast<uint64>(-1);

	// RopeBounds 오버랩 → 근접 정적 바디의 AggGeom을 Boxes/Capsules로 추출한다.
	void BuildColliders(const FBox& RopeBounds);
	// 한 컴포넌트의 BodySetup 심플 콜리전을 월드 공간 콜라이더로 추가한다. 예산 소진 시 false.
	bool AppendBodyColliders(const UBodySetup& Setup, const FTransform& CompTM);
};
