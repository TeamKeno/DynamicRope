// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeGDFViewExtension.h"
#include "RopeGPUSolverRegistry.h"
#include "DynamicRopeShadersLog.h"

#include "SceneView.h"
#include "SceneInterface.h"
#include "FXRenderingUtils.h"
#include "GlobalDistanceFieldParameters.h"
#include "Containers/StridedView.h"

static TSharedPtr<FRopeGDFViewExtension, ESPMode::ThreadSafe> GRopeGDFViewExtension;

void FRopeGDFViewExtension::EnsureRegistered()
{
	if (!GRopeGDFViewExtension.IsValid())
	{
		GRopeGDFViewExtension = FSceneViewExtensions::NewExtension<FRopeGDFViewExtension>();
		UE_LOG(LogDynamicRopeGPU, Log, TEXT("[GDF] 씬 뷰 확장 등록됨."));
	}
}

void FRopeGDFViewExtension::Shutdown()
{
	if (GRopeGDFViewExtension.IsValid())
	{
		FlushRenderingCommands();
		GRopeGDFViewExtension.Reset();
	}
}

void FRopeGDFViewExtension::PrePostProcessPass_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View, const FPostProcessingInputs& Inputs)
{
	check(IsInRenderingThread());

	FSceneInterface* Scene = View.Family ? View.Family->Scene : nullptr;
	if (!Scene)
	{
		return;
	}

	// 프라이머리 뷰만(분할화면 보조 뷰 제외), 씬/리플렉션 캡처 제외.
	if (View.Family->Views.Num() == 0 || View.Family->Views[0] != &View)
	{
		return;
	}
	if (View.bIsSceneCapture || View.bIsReflectionCapture)
	{
		return;
	}

	// 씬별 프레임당 1회로 dedup(같은 프레임에 여러 뷰 패밀리가 올 수 있음).
	const uint32 FrameNumber = View.Family->FrameNumber;
	if (const uint32* Last = LastDispatchedFrame.Find(Scene); Last && *Last == FrameNumber)
	{
		return;
	}
	LastDispatchedFrame.Add(Scene, FrameNumber);

	// 이 뷰의 GDF 파라미터(카메라 중심 clipmap). 소비자 신호가 없으면 null이거나 클립맵 0.
	const FGlobalDistanceFieldParameterData* GDF =
		UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData(MakeStridedView(sizeof(FSceneView), &View, 1));

	// Phase 1 프로브: 온디맨드 빌드가 실제로 일어나는지 확인(과다 로그 방지 위해 가끔만).
	static uint32 ProbeThrottle = 0;
	if ((ProbeThrottle++ % 60) == 0)
	{
		UE_LOG(LogDynamicRopeGPU, Log, TEXT("[GDF] probe: NumClipmaps=%d, PageAtlas=%s"),
			GDF ? GDF->NumGlobalSDFClipmaps : -1,
			(GDF && GDF->PageAtlasTexture) ? TEXT("valid") : TEXT("null"));
	}

	// Phase 2에서: FRopeGPUSolver* Solver = RopeGDF::FindSolver(Scene);
	//             Solver->DispatchPending_RenderThread(GraphBuilder, GDF, (FVector3f)View.ViewMatrices.GetPreViewTranslation());
}
