// Copyright Epic Games, Inc. All Rights Reserved.
//
// GDF 월드 충돌 지원 인프라 두 가지를 한 곳에 둔다:
//  1) 씬→솔버 레지스트리 — GPU 솔버는 월드별(URopeSimSubsystem 멤버)인데 GDF 뷰 확장은 전역이라,
//     렌더 스레드에서 렌더 중인 씬으로 해당 솔버를 찾아 dispatch해야 한다(멀티 PIE 월드 GDF 정합성).
//  2) GDF 소비자 게이트 — 커스텀 FFXSystem(FRopeGDFFXSystem)이 UsesGlobalDistanceField()에서 읽는
//     "이 씬에 GDF 로프가 활성인가" 플래그. 엔진이 GDF를 온디맨드로 빌드하게 만드는 신호.
// 키는 전부 FSceneInterface*(월드 Scene / View.Family->Scene / FFXSystem->GetSceneInterface() 동일 타입).

#pragma once

#include "CoreMinimal.h"

class FRopeGPUSolver;
class FSceneInterface;

namespace RopeGDF
{
	//~ 씬→솔버 레지스트리.
	/** 솔버를 씬에 등록한다(게임 스레드 — subsystem Initialize). */
	DYNAMICROPESHADERS_API void RegisterSolver(FSceneInterface* Scene, FRopeGPUSolver* Solver);
	/** 씬의 솔버 등록을 해제한다(게임 스레드 — subsystem Deinitialize). */
	DYNAMICROPESHADERS_API void UnregisterSolver(FSceneInterface* Scene);
	/** 씬으로 솔버를 찾는다(렌더 스레드 — 뷰 확장). 없으면 nullptr. */
	FRopeGPUSolver* FindSolver(FSceneInterface* Scene);

	//~ GDF 소비자 게이트.
	/** 이 씬의 활성 GDF 로프 수를 설정한다(게임 스레드 — subsystem Tick, 매 프레임 authoritative). */
	DYNAMICROPESHADERS_API void SetGDFActiveCount(FSceneInterface* Scene, int32 Count);
	/** 이 씬이 GDF를 필요로 하는가(활성 카운트>0 또는 강제 CVar). FFXSystem::UsesGlobalDistanceField()가 읽음. */
	bool IsGDFActive(FSceneInterface* Scene);
}
