// Copyright Epic Games, Inc. All Rights Reserved.
//
// GDF 월드 충돌용 씬 뷰 확장. 씬 렌더 중(PrePostProcessPass_RenderThread — opaque 이후·GDF 유효·RDG 열림)에
// 해당 씬의 GPU 솔버를 찾아 GDF 파라미터와 함께 dispatch한다(솔브를 씬 렌더 타이밍으로 이전).
// Phase 1에서는 GDF 클립맵 수를 로그로 찍는 프로브만 수행한다(온디맨드 빌드 검증).

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
	virtual void PrePostProcessPass_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View, const FPostProcessingInputs& Inputs) override;

	//~ 등록/해제(OnPostEngineInit에서 생성, 모듈 shutdown에서 해제).
	static void EnsureRegistered();
	static void Shutdown();

private:
	// 씬별 마지막 dispatch 프레임(멀티 뷰/씬캡처에서 프레임당 1회로 dedup). 렌더 스레드 전용.
	TMap<class FSceneInterface*, uint32> LastDispatchedFrame;
};
