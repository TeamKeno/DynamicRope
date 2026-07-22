// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그의 단일 보관소. 모든 디버그 진입점은 FGameplayDebuggerCategory_Rope이며, 이 서브시스템은
// 그 카테고리와 sim tick 사이의 중계자다: 카테고리가 디버그 대상 액터를 등록(SetTarget)하면, sim
// tick(GT)이 그 액터의 로프만 캡처(ShouldCapture)해 스냅샷을 제출(SubmitSnapshot)하고, 카테고리가
// 다시 읽어(GetSnapshot) 그린다. 대상이 아닌 로프는 flight sweep 같은 캡처 비용을 아예 내지 않는다.
//
// 디버그 전용 기능이라 실질 동작은 WITH_GAMEPLAY_DEBUGGER에서만 컴파일된다(shipping에선 빈 셸).
//
// **모듈 내부 전용이라 Private에 둔다.** API 표면이 Private/Debug의 FRopeDebugSnapshot·ERopeDebugCapture를
// 그대로 노출하므로 Public에 두면 그 타입들까지 공개해야 하는데, 실제 사용처는 이 모듈의 Private 소스
// 셋(카테고리 / RopeComponent / 이 서브시스템 구현)뿐이다. 외부 모듈이 쓸 일이 생기면 그때 스냅샷 타입의
// 공개 여부부터 결정할 것 — 지금 공개하면 디버그 자료구조가 사실상 플러그인 API가 된다.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Debug/RopeDebugSnapshot.h"
#include "RopeDebugSubsystem.generated.h"

class URopeComponent;
class AActor;

UCLASS()
class URopeDebugSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	/** 월드의 rope debug subsystem(게임/PIE에서만 유효, 그 외엔 nullptr). */
	static URopeDebugSubsystem* Get(const UWorld* World);

	/** 게이트플레이 디버거 카테고리가 그릴 때마다 호출: 현재 디버그 대상 액터 + 켜진 보기의 캡처 범위를
	 *  등록한다(+이번 프레임 활성 표시). 마스크는 다음 sim tick이 읽으므로 토글 반영은 한 프레임 뒤다. */
	void SetTarget(AActor* InActor, ERopeDebugCapture InCaptureMask);

	/** 이 로프를 이번 프레임 디버그 캡처할지: 카테고리가 최근 활성이고 로프 owner가 대상 액터일 때만 true. */
	bool ShouldCapture(const URopeComponent* Rope) const;

	/** 이번 프레임 채울 섹션. 캡처 대상 로프가 어느 수집을 건너뛸지 판단하는 데 쓴다. */
	ERopeDebugCapture GetCaptureMask() const;

	/** sim tick(GT)이 채운 스냅샷을 제출한다(대상 로프에 한해 호출). */
	void SubmitSnapshot(const URopeComponent* Rope, FRopeDebugSnapshot&& Snapshot);

	/** 카테고리가 그릴 때 읽는다. 스냅샷이 없거나 너무 오래됐으면(대상 해제 등) nullptr. */
	const FRopeDebugSnapshot* GetSnapshot(const URopeComponent* Rope) const;

	/** hold된 마지막 flight 스냅샷(실시간 FlightHoldSeconds 창 내). 없으면 nullptr, 있으면 OutAgeSeconds에
	 *  경과초를 담는다. Flight를 벗어난 직후에도 flight 오버레이를 잔류시키는 데 쓴다. */
	const FRopeDebugSnapshot* GetHeldFlightSnapshot(const URopeComponent* Rope, float& OutAgeSeconds) const;

	//~ UWorldSubsystem
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

	//~ FTickableGameObject (UTickableWorldSubsystem) — 스냅샷 수명 관리를 프레임당 한 번 돌린다: 활성
	//   중엔 스테일/무효 항목만 정리(제출마다 전체 맵을 훑던 O(제출수×로프수) 제거), 카테고리가 비활성이면
	//   보관분을 통째로 비운다(제출이 멈춰도 마지막 배열이 월드 종료까지 남지 않게). 디버그 빌드에서만 틱한다.
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool IsTickable() const override;

private:
#if WITH_GAMEPLAY_DEBUGGER
	// 활성/스테일 판정 창(프레임). 카테고리 CollectData와 서브시스템 tick의 프레임 내 순서가 불확정이라
	// 한두 프레임 여유를 둬 캡처가 끊기지 않게 한다.
	static constexpr uint64 ActiveFrameWindow = 4;

	TWeakObjectPtr<AActor> TargetActor;
	uint64 LastActiveFrame = 0;
	ERopeDebugCapture CaptureMask = ERopeDebugCapture::None;
	TMap<TWeakObjectPtr<const URopeComponent>, FRopeDebugSnapshot> Snapshots;

	// 마지막 flight 스냅샷 보관: bHasFlight 프레임은 다음(Wrapping) 프레임 스냅샷에 덮여 사라지므로,
	// 실시간 FlightHoldSeconds 동안 별도로 들고 있어 flight 오버레이를 결정 순간 위치에 잔류시킨다.
	struct FHeldFlightSnapshot
	{
		FRopeDebugSnapshot Snapshot;
		// 캡처 시각 World->GetRealTimeSeconds() — slomo/pause와 무관한 실시간이라 hold 창이 벽시계 기준.
		double RealTimeSeconds = 0.0;
	};
	TMap<TWeakObjectPtr<const URopeComponent>, FHeldFlightSnapshot> HeldFlight;
	static constexpr double FlightHoldSeconds = 2.5;
#endif
};
