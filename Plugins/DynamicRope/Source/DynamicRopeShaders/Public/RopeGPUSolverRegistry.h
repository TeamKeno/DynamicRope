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
class FRDGBuilder;

/**
 * Phase 2b: GDF 통합 경로에서 뷰 확장이 솔브 뒤에 프록시별 튜브를 씬 그래프로 빌드하기 위한 추상 인터페이스.
 * DynamicRopeShaders(하위 모듈)가 정의하고 런타임 FRopeSceneProxy가 구현한다(모듈 의존 방향 유지 — shaders는
 * 런타임 렌더 타입을 몰라도 이 인터페이스 포인터만 순회하면 된다).
 */
class IRopeGDFTubeProxy
{
public:
	virtual ~IRopeGDFTubeProxy() = default;
	/** 렌더 스레드. 전달받은 씬 렌더러 그래프에 이 프록시의 튜브 생성 패스를 얹는다(Solver로 resident PosBuf 획득). */
	virtual void BuildTubeInSceneGraph_RenderThread(FRDGBuilder& GraphBuilder, FRopeGPUSolver& Solver) = 0;
};

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

	//~ 경로 선택.
	/** GPU 솔브 dispatch를 씬 뷰 확장 경로(EnqueueSteps→DispatchPending)로 돌릴지 여부. r.DynamicRope.GDFDispatchInVE.
	    0(기본)=Step() 전용 그래프(현행), 1=뷰 확장에서 씬 그래프로 dispatch(GDF 월드 충돌 통합). */
	DYNAMICROPESHADERS_API bool IsDispatchInVE();

	//~ 튜브 프록시 레지스트리(Phase 2b, GDF 통합 경로에서 솔브 뒤 튜브를 씬 그래프로 빌드).
	/** 씬에 튜브 프록시를 등록한다(렌더 스레드 — 프록시 생성 시). */
	DYNAMICROPESHADERS_API void RegisterTubeProxy(FSceneInterface* Scene, IRopeGDFTubeProxy* Proxy);
	/** 씬에서 튜브 프록시 등록을 해제한다(렌더 스레드 — 프록시 소멸 시). */
	DYNAMICROPESHADERS_API void UnregisterTubeProxy(FSceneInterface* Scene, IRopeGDFTubeProxy* Proxy);
	/** 씬의 등록된 튜브 프록시를 순회한다(렌더 스레드 — 뷰 확장). */
	void ForEachTubeProxy(FSceneInterface* Scene, TFunctionRef<void(IRopeGDFTubeProxy*)> Fn);
}
