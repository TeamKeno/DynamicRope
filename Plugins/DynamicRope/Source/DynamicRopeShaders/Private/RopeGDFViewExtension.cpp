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

	if (!RopeGDF::IsDispatchInVE() || !Family)
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

	// 1) 솔브를 씬 그래프에 얹는다. View 전달 — GDF in-solver(r.DynamicRope.GDFInSolver) 경로가 GDF permutation
	//    선택 + View/GDF 바인딩에 사용(off면 무시). GDF는 이 뷰 확장 경로에서만 유효.
	Solver->DispatchPending_RenderThread(GraphBuilder, View, GDF, PreViewTranslation);

	// 2) 솔브 뒤: GDF 월드 밀어내기(정적 벽/바닥). PosBuf를 in-place 보정 → RDG가 solve→GDF 순서 보장.
	Solver->DispatchGDFCollision_RenderThread(GraphBuilder, *View, GDF, PreViewTranslation);

	// 3) GDF 뒤: 이 씬의 튜브 프록시들이 (GDF 보정된) PosBuf로 튜브를 (재)빌드한다(RDG가 GDF→tube 순서 보장).
	//    resident 프레임만 덮어쓰므로 지연이 없다(비-resident는 프록시가 스스로 스킵).
	RopeGDF::ForEachTubeProxy(Scene, [&GraphBuilder, Solver](IRopeGDFTubeProxy* Proxy)
	{
		Proxy->BuildTubeInSceneGraph_RenderThread(GraphBuilder, *Solver);
	});
}
