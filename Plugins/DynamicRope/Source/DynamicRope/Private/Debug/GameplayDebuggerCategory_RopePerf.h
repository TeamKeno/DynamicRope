// Copyright Epic Games, Inc. All Rights Reserved.
//
// 'RopePerf' 게임플레이 디버거 카테고리 — 월드 전역 로프 perf/스로틀 개관. 기존 'Rope' 카테고리가 디버그
// 액터 한 명의 로프를 깊게 그리는 데 반해, 이쪽은 월드의 **모든** 활성 로프를 훑어 프레임 부하를 한눈에
// 집계한다(안 보고 있는 로프 — 드래곤 등 — 의 perf 문제를 잡는 용도). 'stat DynamicRope' 그룹과 같은 값을
// per-rope 라인 + 상단 집계로 HUD에 얹는다.
//
// 로프 목록은 URopeSimSubsystem::GetRegisteredRopes() 하나에서 온다 — 이 화면이 집계하는 "이번 프레임
// 실제로 틱되는 로프"의 정의가 곧 그 등록 목록이라, 월드를 따로 훑으면 서브시스템이 안 도는 로프까지
// 섞여 프레임 부하와 어긋난다.
// 목록을 얻은 뒤의 값은 전부 URopeComponent의 public 게터(GetPhase/GetNodeCount/IsSleeping/
// GetSolverLODScale/IsGpuSteppedThisFrame/WasSolvedThisFrame/bUseWorldGDF/GetCenterlinePositions)에서
// 라이브로 읽는다 — 스냅샷 의존은 없다. 카메라 거리는 OwnerPC 카메라에서 직접 산출.
// WITH_GAMEPLAY_DEBUGGER가 꺼진 빌드(shipping 등)에서는 전체가 컴파일에서 제외된다.

#pragma once

#include "CoreMinimal.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "GameplayDebuggerCategory.h"

class APlayerController;
class AActor;

class FGameplayDebuggerCategory_RopePerf : public FGameplayDebuggerCategory
{
public:
	FGameplayDebuggerCategory_RopePerf();

	virtual void CollectData(APlayerController* OwnerPC, AActor* DebugActor) override;

	static TSharedRef<FGameplayDebuggerCategory> MakeInstance();
};

#endif // WITH_GAMEPLAY_DEBUGGER
