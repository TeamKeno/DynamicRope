// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeGDFViewExtension.h"
#include "RopeGPUSolverRegistry.h"
#include "RopeGPUSolver.h"
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

void FRopeGDFViewExtension::PreRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily)
{
	// GDF 빌드 이전 시점 — 여기선 dispatch하지 않고, base pass 훅에서 쓸 패밀리만 캡처한다.
	CurrentFamily = &InViewFamily;
}

void FRopeGDFViewExtension::PreRenderBasePass_RenderThread(FRDGBuilder& GraphBuilder, bool /*bDepthBufferIsPopulated*/)
{
	check(IsInRenderingThread());

	// 캡처한 패밀리는 이번 훅에서 1회만 소비(뒤따르는 base pass 없는 경로에서 stale 재사용 방지).
	FSceneViewFamily* Family = CurrentFamily;
	CurrentFamily = nullptr;

	if (!Family)
	{
		return;
	}

	FSceneInterface* Scene = Family->Scene;
	if (!Scene || Family->Views.Num() == 0)
	{
		return;
	}

	const FSceneView* View = Family->Views[0];
	if (!View || View->bIsSceneCapture || View->bIsReflectionCapture)
	{
		return;
	}

	// 씬별 프레임당 1회(한 프레임에 여러 패밀리/뷰가 올 수 있음).
	const uint32 FrameNumber = Family->FrameNumber;
	if (const uint32* Last = LastDispatchedFrame.Find(Scene); Last && *Last == FrameNumber)
	{
		return;
	}
	LastDispatchedFrame.Add(Scene, FrameNumber);

	FRopeGPUSolver* Solver = RopeGDF::FindSolver(Scene);
	if (!Solver)
	{
		return;
	}

	// 이 뷰의 GDF 파라미터(카메라 중심 clipmap; 미빌드면 null 또는 클립맵 0). 솔버가 null-체크해 스킵.
	const FGlobalDistanceFieldParameterData* GDF =
		UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData(MakeStridedView(sizeof(FSceneView), View, 1));

	// GDF 함수는 TranslatedWorld를 받으므로 월드→TranslatedWorld 오프셋을 넘긴다.
	const FVector3f PreViewTranslation = (FVector3f)View->ViewMatrices.GetPreViewTranslation();

	// 솔브를 씬 그래프에 얹는다. View 전달 — 솔브 CS가 bUseWorldGDF 로프에 GDF permutation을 골라 매 substep
	// 벽을 투영한다(정적 월드 GDF 충돌은 솔브 안에서 처리; 이 뷰 확장 경로에서만 유효).
	// 튜브는 여기서 다시 빌드하지 않는다: 솔브(=prepass 이후)에서 튜브를 덮어쓰면 depth prepass 지오메트리와
	// base pass 지오메트리가 어긋나 EQUAL 깊이 테스트에서 픽셀이 탈락한다(로프가 검게 탐). 튜브는 프록시가
	// 프레임 초 SetDynamicData에서 직전 프레임 PosBuf로 1회 빌드해 모든 패스에 일관되게 그린다(1프레임 렌더 지연).
	Solver->DispatchPending_RenderThread(GraphBuilder, View, GDF, PreViewTranslation);
}
