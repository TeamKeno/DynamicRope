// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그의 단일 보관소. 모든 디버그 진입점은 FGameplayDebuggerCategory_Rope이며, 이 서브시스템은
// 그 카테고리와 sim tick 사이의 중계자다: 카테고리가 디버그 대상 액터를 등록(SetTarget)하면, sim
// tick(GT)이 그 액터의 로프만 캡처(ShouldCapture)해 스냅샷을 제출(SubmitSnapshot)하고, 카테고리가
// 다시 읽어(GetSnapshot) 그린다. 대상이 아닌 로프는 flight sweep 같은 캡처 비용을 아예 내지 않는다.
//
// 디버그 전용 기능이라 실질 동작은 WITH_GAMEPLAY_DEBUGGER에서만 컴파일된다(shipping에선 빈 셸).

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Debug/RopeDebugSnapshot.h"
#include "RopeDebugSubsystem.generated.h"

class URopeComponent;
class AActor;

UCLASS()
class DYNAMICROPE_API URopeDebugSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** 월드의 rope debug subsystem(게임/PIE에서만 유효, 그 외엔 nullptr). */
	static URopeDebugSubsystem* Get(const UWorld* World);

	/** 게이트플레이 디버거 카테고리가 그릴 때마다 호출: 현재 디버그 대상 액터 등록(+이번 프레임 활성 표시). */
	void SetTarget(AActor* InActor);

	/** 이 로프를 이번 프레임 디버그 캡처할지: 카테고리가 최근 활성이고 로프 owner가 대상 액터일 때만 true. */
	bool ShouldCapture(const URopeComponent* Rope) const;

	/** sim tick(GT)이 채운 스냅샷을 제출한다(대상 로프에 한해 호출). */
	void SubmitSnapshot(const URopeComponent* Rope, FRopeDebugSnapshot&& Snapshot);

	/** 카테고리가 그릴 때 읽는다. 스냅샷이 없거나 너무 오래됐으면(대상 해제 등) nullptr. */
	const FRopeDebugSnapshot* GetSnapshot(const URopeComponent* Rope) const;

	//~ UWorldSubsystem
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

private:
#if WITH_GAMEPLAY_DEBUGGER
	// 활성/스테일 판정 창(프레임). 카테고리 CollectData와 서브시스템 tick의 프레임 내 순서가 불확정이라
	// 한두 프레임 여유를 둬 캡처가 끊기지 않게 한다.
	static constexpr uint64 ActiveFrameWindow = 4;

	TWeakObjectPtr<AActor> TargetActor;
	uint64 LastActiveFrame = 0;
	TMap<TWeakObjectPtr<const URopeComponent>, FRopeDebugSnapshot> Snapshots;
#endif
};
