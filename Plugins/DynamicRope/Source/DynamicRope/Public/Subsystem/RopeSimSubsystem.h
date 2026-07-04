// Copyright Epic Games, Inc. All Rights Reserved.
//
// 모든 활성 URopeComponent의 시뮬레이션을 한 곳에서 구동하는 world subsystem. 컴포넌트가 각자
// tick하던 것을 대체하는 단일 오케스트레이션 지점이다. 현재는 순차 구동만 — 추후 이 위에
// collider gather 디둡 / ParallelFor 병렬 솔브 / LOD·sleep / 프레임당 예산 상한을 얹는다.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "RopeGPUSolver.h" // FRopeGPUSolver (DynamicRopeShaders): 비동기 GPU 솔브 인스턴스
#include "RopeSimSubsystem.generated.h"

class URopeComponent;
class UActorComponent;
class IRopeCollider;
class AActor;

UCLASS()
class DYNAMICROPE_API URopeSimSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	/** 활성 로프를 시뮬레이션 목록에 등록/해제한다(컴포넌트 BeginPlay/EndPlay에서 호출). */
	void RegisterRope(URopeComponent* Rope);
	void UnregisterRope(URopeComponent* Rope);

	/**
	 * collider provider(IRopeColliderProvider를 구현한 UActorComponent)를 중앙 레지스트리에 등록/해제한다
	 * (provider BeginPlay/EndPlay에서 호출). 로프마다 월드를 스캔하던 것을 대체 — 프레임당 1회 중앙 빌드.
	 */
	void RegisterColliderProvider(UActorComponent* Provider);
	void UnregisterColliderProvider(UActorComponent* Provider);

	/** 월드의 rope sim subsystem(게임/PIE 월드에서 유효, 그 외엔 nullptr). */
	static URopeSimSubsystem* Get(const UWorld* World);

	/** GPU 상주 솔버 포인터(월드 수명). M5b: scene proxy가 resident PosBuf SRV를 가져오는 데 쓴다. */
	FRopeGPUSolver* GetGpuSolver() { return &GpuSolver; }

	/**
	 * wrap 핸드오프 정밀 동기(M5c): GPU 상주 로프의 CPU 미러(Sim)는 1~2프레임 낡으므로,
	 * Wrapping 진입 순간 1회 동기 리드백으로 최신 위치를 Sim에 반영한다(시드 정밀도 확보).
	 * GPU 솔버가 꺼져 있거나 상주 버퍼가 없거나 generation이 어긋나면 no-op(false).
	 * 블로킹(GPU idle 대기) — 이벤트당 1회 용도로만 호출할 것.
	 */
	bool SyncGpuPositionsForHandoff(URopeComponent& Rope);

	//~ UTickableWorldSubsystem
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
	//~ 씬→솔버 등록(GDF 통합 경로에서 뷰 확장이 솔버를 찾는 용도). 씬 생성 이후/해제 시점에 등록·해제.
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual void Deinitialize() override;

private:
	// 등록된 활성 로프(컴포넌트는 UObject → GC 추적).
	UPROPERTY(Transient)
	TArray<TObjectPtr<URopeComponent>> Ropes;

	// 등록된 collider provider(IRopeColliderProvider 구현 컴포넌트). GC 추적.
	UPROPERTY(Transient)
	TArray<TObjectPtr<UActorComponent>> ColliderProviders;

	// 프레임당 1회 중앙 빌드한 collider(provider별 소유 액터 + collider 포인터). 포인터는 provider 소유라 해당 프레임만 유효.
	struct FFrameProviderColliders
	{
		AActor* Owner = nullptr;          // 소스 필터링용(provider 컴포넌트의 owner 액터).
		TArray<IRopeCollider*> Colliders; // provider->GatherColliders가 채운 포인터(provider 백킹 스토리지를 가리킴).
	};
	TArray<FFrameProviderColliders> FrameProviders;

	// 등록된 provider 전부에서 1회 collider를 모은다(Prepare 이전). RopeBounds는 전 로프 bounds 합집합을 넘긴다.
	void BuildFrameColliders();
	// 한 로프의 collider를 중앙 빌드에서 모은다: 기본은 전체, 자기 owner provider만 제외(bIncludeOwnerColliders로 옵트인).
	void GatherCollidersForRope(const URopeComponent& Rope, TArray<IRopeCollider*>& OutColliders) const;

	// GPU 상주 솔버(M5). 영속 버퍼(로프별)를 매 프레임 in-place 전진. 인스턴스 상태라 월드별 1개.
	// G4: 렌더 가능 RHI면 이게 유일 런타임 경로. RHI 없으면(쿡/-nullrhi/서버) CPU 솔버로 자동 폴백.
	FRopeGPUSolver GpuSolver;

	// GetLatest로 회수한 RopeId별 최신(약간 지연) 위치 캐시. 매 프레임 갱신분을 각 Sim에 매핑한다.
	TMap<uint32, FRopeResidentLatest> GpuLatest;

	// GetLatestContacts로 회수한 RopeId별 최신(약간 지연) GPU 접촉 감지 결과(G3). Finalize 전에 귀속.
	TMap<uint32, FRopeResidentContacts> GpuLatestContacts;

	// GPU 감지 결과(콜라이더 인덱스)를 로프의 귀속 테이블로 FRopeContactCandidate로 복원해 컴포넌트에
	// 채운다(Finalize의 Flight 접촉 소스). 지연분이 현재 시드 generation과 맞을 때만 유효.
	void BuildGpuFlightCandidates(URopeComponent& Rope);
};
