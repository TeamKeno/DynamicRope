// Copyright Epic Games, Inc. All Rights Reserved.
//
// The scene view extension for world collision against the global distance field. It moves the GPU solve dispatch
// inside the scene renderer's graph, at PreRenderBasePass, meaning after the global distance field is built and
// before the base pass, so it runs at a point where the field's parameters are valid. This is the single dispatch
// path of the runtime GPU solve; the solver's own graph in Step() remains for the unit test harness alone.
// The tube is not built here: rebuilding it at solve time, meaning after the depth prepass, would leave the prepass
// and base pass geometry disagreeing and the pixels would fail the equal depth test, leaving the rope black.
// The tube is built once by the proxy at the start of the frame in SetDynamicData, from the previous frame's position
// buffer, which is a deliberate one-frame render latency.
//
// The PreRenderBasePass hook takes no view argument, so the family is captured in PreRenderViewFamily.

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
	/** Captures this family, used by the next PreRenderBasePass to obtain the primary view, the scene and the distance field. */
	virtual void PreRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily) override;
	/** After the distance field is built and before the base pass: obtains the field from the captured family's primary view and dispatches that scene's solver. */
	virtual void PreRenderBasePass_RenderThread(FRDGBuilder& GraphBuilder, bool bDepthBufferIsPopulated) override;

	//~ Registration and unregistration, created on OnPostEngineInit and released on module shutdown.
	static void EnsureRegistered();
	static void Shutdown();

private:
	// The family captured by the last PreRenderViewFamily, consumed by PreRenderBasePass and reset to null. Render thread only.
	FSceneViewFamily* CurrentFamily = nullptr;
	// The last dispatched frame per scene, which deduplicates to once per frame across multiple views and scene captures. Render thread only.
	TMap<class FSceneInterface*, uint32> LastDispatchedFrame;
};
