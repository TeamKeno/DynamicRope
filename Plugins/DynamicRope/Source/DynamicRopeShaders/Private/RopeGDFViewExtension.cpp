// Copyright 2026 TeamKeno. All Rights Reserved.

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
		UE_LOG(LogDynamicRopeGPU, Log, TEXT("[GDF] The scene view extension is registered."));
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
	// This runs before the distance field is built, so nothing is dispatched here; it only captures the family the base pass hook will use.
	CurrentFamily = &InViewFamily;
}

void FRopeGDFViewExtension::PreRenderBasePass_RenderThread(FRDGBuilder& GraphBuilder, bool /*bDepthBufferIsPopulated*/)
{
	check(IsInRenderingThread());

	// The captured family is consumed exactly once by this hook, which prevents a stale reuse on a path with no base pass following.
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

	// Once per frame per scene, since a frame can carry several families and views.
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

	// This view's global distance field parameters, being a camera-centred clipmap, which are null or an empty clipmap if it is not built. The solver null-checks and skips.
	const FGlobalDistanceFieldParameterData* GDF =
		UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData(MakeStridedView(sizeof(FSceneView), View, 1));

	// The distance field functions take translated world space, so the offset from world to translated world is passed in.
	const FVector3f PreViewTranslation = (FVector3f)View->ViewMatrices.GetPreViewTranslation();

	// Adds the solve to the scene graph. The view is passed through so the solve compute shader can select the
	// distance field permutation for ropes using the world field and project against walls every substep; static world
	// collision against the field is handled inside the solve and is valid on this view extension path alone.
	// The tube is not rebuilt here: overwriting it at solve time, meaning after the prepass, would leave the depth
	// prepass geometry and the base pass geometry disagreeing and the pixels would fail the equal depth test, leaving
	// the rope black. The tube is built once by the proxy at the start of the frame in SetDynamicData, from the
	// previous frame's position buffer, so every pass draws it consistently, at a one-frame render latency.
	Solver->DispatchPending_RenderThread(GraphBuilder, View, GDF, PreViewTranslation);
}
