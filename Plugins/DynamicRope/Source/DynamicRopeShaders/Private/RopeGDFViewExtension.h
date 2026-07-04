// Copyright Epic Games, Inc. All Rights Reserved.
//
// GDF 월드 충돌용 씬 뷰 확장. GPU 솔브 dispatch를 씬 렌더러 그래프 안(PreRenderBasePass — GDF 빌드 이후·
// base pass 이전)으로 옮겨, GDF 파라미터가 유효한 타이밍에 돌린다. base pass 이전이라 GPU 튜브가 이번 프레임
// 결과를 봐 지연이 없다. r.DynamicRope.GDFDispatchInVE=1일 때만 동작(기본 0은 솔버 전용 그래프 경로).
//
// PreRenderBasePass 훅은 뷰 인자가 없으므로, PreRenderViewFamily에서 이번 패밀리를 캡처해 둔다.

#pragma once

#include "CoreMinimal.h"
#include "SceneViewExtension.h"

class FRopeGDFViewExtension : public FSceneViewExtensionBase
{
public:
	FRopeGDFViewExtension(const FAutoRegister& AutoReg)
		: FSceneViewExtensionBase(AutoReg)
	{
	}

	//~ ISceneViewExtension
	virtual void SetupViewFamily(FSceneViewFamily& /*InViewFamily*/) override {}
	virtual void SetupView(FSceneViewFamily& /*InViewFamily*/, FSceneView& /*InView*/) override {}
	virtual void BeginRenderViewFamily(FSceneViewFamily& /*InViewFamily*/) override {}
	/** 이번 패밀리를 캡처(다음 PreRenderBasePass에서 프라이머리 뷰/씬/GDF를 얻는 데 사용). */
	virtual void PreRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily) override;
	/** GDF 빌드 이후·base pass 이전. 캡처한 패밀리의 프라이머리 뷰로 GDF를 얻어 해당 씬의 솔버를 dispatch. */
	virtual void PreRenderBasePass_RenderThread(FRDGBuilder& GraphBuilder, bool bDepthBufferIsPopulated) override;

	//~ 등록/해제(OnPostEngineInit에서 생성, 모듈 shutdown에서 해제).
	static void EnsureRegistered();
	static void Shutdown();

private:
	// 직전 PreRenderViewFamily에서 캡처한 패밀리(PreRenderBasePass에서 소비 후 nullptr로 리셋). 렌더 스레드 전용.
	FSceneViewFamily* CurrentFamily = nullptr;
	// 씬별 마지막 dispatch 프레임(멀티 뷰/씬캡처에서 프레임당 1회로 dedup). 렌더 스레드 전용.
	TMap<class FSceneInterface*, uint32> LastDispatchedFrame;
};
