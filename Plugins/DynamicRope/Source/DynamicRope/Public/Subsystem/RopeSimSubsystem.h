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

UCLASS()
class DYNAMICROPE_API URopeSimSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	/** 활성 로프를 시뮬레이션 목록에 등록/해제한다(컴포넌트 BeginPlay/EndPlay에서 호출). */
	void RegisterRope(URopeComponent* Rope);
	void UnregisterRope(URopeComponent* Rope);

	/** 월드의 rope sim subsystem(게임/PIE 월드에서 유효, 그 외엔 nullptr). */
	static URopeSimSubsystem* Get(const UWorld* World);

	//~ UTickableWorldSubsystem
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

private:
	// 등록된 활성 로프(컴포넌트는 UObject → GC 추적).
	UPROPERTY(Transient)
	TArray<TObjectPtr<URopeComponent>> Ropes;

	// GPU 상주 솔버(M5). 영속 버퍼(로프별)를 매 프레임 in-place 전진. 인스턴스 상태라 월드별 1개.
	// r.DynamicRope.GPUSolver로 켤 때만 사용. CPU 솔버는 ground-truth로 유지.
	FRopeGPUSolver GpuSolver;

	// GetLatest로 회수한 RopeId별 최신(약간 지연) 위치 캐시. 매 프레임 갱신분을 각 Sim에 매핑한다.
	TMap<uint32, FRopeResidentLatest> GpuLatest;
};
