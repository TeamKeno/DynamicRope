// Copyright Epic Games, Inc. All Rights Reserved.
//
// A minimal custom FX system for building the global distance field on demand. Like Niagara it registers as a sibling
// in the scene's FFXSystemSet, so that the engine's build gate, ShouldPrepareGlobalDistanceField, which reads
// UsesGlobalDistanceField() as an OR, is told whether this scene has an active rope using the field, through
// RopeGDF::IsGDFActive. Every other pure virtual of FFXSystemInterface is a no-op stub: this system runs no
// simulation and no rendering and only conveys the signal that the field is needed.

#pragma once

#include "CoreMinimal.h"
#include "Containers/StridedView.h"
#include "FXSystem.h"

class FRopeGDFFXSystem final : public FFXSystemInterface
{
public:
	/** The unique name passed to RegisterCustomFXSystem. */
	static const FName Name;

	FRopeGDFFXSystem(ERHIFeatureLevel::Type InFeatureLevel, EShaderPlatform InShaderPlatform, FGPUSortManager* InGPUSortManager)
		: FeatureLevel(InFeatureLevel)
		, ShaderPlatform(InShaderPlatform)
		, GPUSortManager(InGPUSortManager)
	{
	}

	//~ The essential one: true when this scene has an active rope using the field, which makes the engine build it.
	virtual bool UsesGlobalDistanceField() const override;

	//~ The GPU sort manager must be returned exactly as it was given at construction, since the render loop uses it.
	virtual FGPUSortManager* GetGPUSortManager() const override { return GPUSortManager; }

	//~ The pure virtual stubs below; this system runs no particles, vector fields or GPU simulation.
	virtual void Tick(UWorld* /*World*/, float /*DeltaSeconds*/) override {}
#if WITH_EDITOR
	virtual void Suspend() override {}
	virtual void Resume() override {}
#endif
	virtual void DrawDebug(FCanvas* /*Canvas*/) override {}
	virtual void AddVectorField(UVectorFieldComponent* /*VectorFieldComponent*/) override {}
	virtual void RemoveVectorField(UVectorFieldComponent* /*VectorFieldComponent*/) override {}
	virtual void UpdateVectorField(UVectorFieldComponent* /*VectorFieldComponent*/) override {}
	virtual void PreInitViews(FRDGBuilder& /*GraphBuilder*/, bool /*bAllowGPUParticleUpdate*/, const TArrayView<const FSceneViewFamily*>& /*ViewFamilies*/, const FSceneViewFamily* /*CurrentFamily*/) override {}
	virtual void PostInitViews(FRDGBuilder& /*GraphBuilder*/, TConstStridedView<FSceneView> /*Views*/, bool /*bAllowGPUParticleUpdate*/) override {}
	virtual bool UsesDepthBuffer() const override { return false; }
	virtual bool RequiresEarlyViewUniformBuffer() const override { return false; }
	virtual bool RequiresRayTracingScene() const override { return false; }
	virtual void PreRender(FRDGBuilder& /*GraphBuilder*/, TConstStridedView<FSceneView> /*Views*/, FSceneUniformBuffer& /*SceneUniformBuffer*/, bool /*bAllowGPUParticleUpdate*/) override {}
	virtual void PostRenderOpaque(FRDGBuilder& /*GraphBuilder*/, TConstStridedView<FSceneView> /*Views*/, FSceneUniformBuffer& /*SceneUniformBuffer*/, bool /*bAllowGPUParticleUpdate*/) override {}

private:
	ERHIFeatureLevel::Type FeatureLevel;
	EShaderPlatform        ShaderPlatform;
	FGPUSortManager*       GPUSortManager = nullptr;
};

/** The RegisterCustomFXSystem factory, called by the engine when a scene is created to build the sibling FX system. */
FFXSystemInterface* CreateRopeGDFFXSystem(ERHIFeatureLevel::Type InFeatureLevel, EShaderPlatform InShaderPlatform, FGPUSortManager* InGPUSortManager);
