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
class UInstancedStaticMeshComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeStaticBodyProvider : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeStaticBodyProvider();

	//~ UActorComponent — RopeSimSubsystem 중앙 레지스트리에 등록/해제(프레임당 1회 중앙 gather).
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// 콜라이더 예산(MaxColliders)과 컨벡스 평면 상한(MaxConvexPlanes)은 컴포넌트가 아니라 Project Settings
	// (UDynamicRopeSettings)에서 단일 관리한다 — 중복 방지 가드가 "월드당 프로바이더 1개"를 강제하므로
	// 컴포넌트별 숫자 예산은 전역 세팅 대비 실익이 없다. 아래 IgnoredComponents처럼 프로바이더별로만
	// 의미 있는 값만 컴포넌트에 남긴다. 프로바이더는 BuildColliders에서 세팅을 직접 읽는다.

	/** 수집에서 제외할 컴포넌트(예: 로프가 의도적으로 통과해야 하는 지오메트리). 프로바이더별 값이라 컴포넌트에 둔다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	TArray<TObjectPtr<UPrimitiveComponent>> IgnoredComponents;

	//~ IRopeColliderProvider
	virtual void GatherColliders(FRopeColliderGatherContext& Gather) override;
	virtual bool ProvidesWorldStaticColliders() const override { return true; }

private:
	// 프레임당 1회 재구성되는 백킹 스토리지. 넘겨준 포인터들은 해당 프레임 동안 유효하다.
	TArray<FRopeBoxCollider> Boxes;
	TArray<FRopeStaticCapsuleCollider> Capsules;
	TArray<FRopeConvexCollider> Convexes; // convex 심플 콜리전 + 전단 박스(6평면) 라우팅.

	// 추출 그룹: 컴포넌트(또는 ISM 호출) 1회가 추가한 콜라이더의 타입별 로컬 인덱스 range + 유니언 bounds.
	// gather의 region 오버랩이 이미 아는 "이 바디가 어느 로프 근처인가"를 그룹 단위로 보존해,
	// 서브시스템의 로프별 풀 전체 재-컬(O(로프×풀))을 "그룹 유니언 선-거절 → 히트 그룹만 콜라이더별
	// 배정"으로 대체한다(GatherColliders 끝의 매핑 단계). 겹치는 region은 그룹이 양쪽 모두에 배정된다.
	struct FExtractedGroup
	{
		int32 BoxStart = 0, BoxCount = 0;
		int32 CapStart = 0, CapCount = 0;
		int32 CvxStart = 0, CvxCount = 0;
		FBox Bounds = FBox(ForceInit);
	};
	TArray<FExtractedGroup> Groups;

	// 스냅샷(각 Start) 이후 Boxes/Capsules/Convexes에 추가된 분량을 유니언 bounds와 함께 그룹으로 기록.
	// 아무것도 추가되지 않았으면 무시.
	void RecordExtractedGroup(int32 BoxStart, int32 CapStart, int32 CvxStart);

	// 마지막으로 빌드한 GFrameCounter. 같은 프레임에 여러 로프가 호출해도 재빌드 안 함(디둡).
	// 단 이 provider는 RopeBounds(전 로프 union — 서브시스템이 프레임당 동일 값 전달)를 실제로 쓴다.
	uint64 BuiltFrame = static_cast<uint64>(-1);

	// 동적 바디 표면 속도용: 컴포넌트별 이전 프레임 월드 트랜스폼. 매 프레임 갱신 — 이번 프레임 (curr - prev)로
	// 표면 속도/substep CCD를 산출한다. weak 키라 파괴된 컴포넌트 항목은 다음 갱신에서 자연히 사라진다.
	TMap<TWeakObjectPtr<UPrimitiveComponent>, FTransform> PrevCompXforms;

	// 로프별 region마다 오버랩 → 근접 정적 바디의 AggGeom을 Boxes/Capsules로 추출한다. region 간 중복은
	// 컴포넌트/인스턴스 단위 디둡으로 프레임당 1회만 추출(빈 공간 union AABB의 낭비·예산 경합 제거).
	void BuildColliders(TArrayView<const FBox> RopeRegions);
	// 한 컴포넌트의 BodySetup 심플 콜리전을 월드 공간 콜라이더로 추가한다. 예산 소진 시 false.
	// 예산/컨벡스 평면 상한은 호출자(BuildColliders)가 Project Settings에서 읽어 전달한다(단일 소스).
	// PrevCompTM/InvDeltaTime: 동적 바디 표면 속도용(이전 프레임 트랜스폼 + 1/dt). 정적이면 PrevCompTM=CompTM,
	// InvDeltaTime=0을 넘긴다(표면 속도 0). 콜라이더 셰이프를 prev 트랜스폼으로도 만들어 prev 상태를 채운다.
	bool AppendBodyColliders(const UBodySetup& Setup, const FTransform& CompTM, const FTransform& PrevCompTM,
		float InvDeltaTime, int32 MaxColliders, int32 MaxConvexPlanes);

	// ISM/HISM(M3): region과 겹치는 인스턴스만 열거해 각 인스턴스 월드 트랜스폼으로 공유 BodySetup을 추출한다
	// (모든 인스턴스가 같은 메시 콜리전 공유). SeenIndices: 여러 region에 걸치는 ISM의 인스턴스를 인덱스 단위로
	// 디둡(이미 추출한 인덱스는 건너뜀) — 컴포넌트 단위 디둡은 다른 region의 다른 인스턴스를 놓치므로 부적합.
	// 예산 소진 시 false. 폴리지/모듈러 에셋 지원.
	bool AppendInstancedBodyColliders(UInstancedStaticMeshComponent& ISM, const FBox& Region,
		TSet<int32>& SeenIndices, int32 MaxColliders, int32 MaxConvexPlanes);
};
