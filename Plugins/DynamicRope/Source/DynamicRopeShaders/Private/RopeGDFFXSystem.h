// Copyright Epic Games, Inc. All Rights Reserved.
//
// GDF 온디맨드 빌드를 위한 최소 커스텀 FX 시스템. Niagara처럼 씬의 FFXSystemSet에 sibling으로 등록되어,
// 엔진 GDF 빌드 게이트(ShouldPrepareGlobalDistanceField)가 OR로 읽는 UsesGlobalDistanceField()에
// "이 씬에 GDF 로프가 활성인가"(RopeGDF::IsGDFActive)를 실어준다. 나머지 FFXSystemInterface 순수가상은
// 전부 no-op 스텁 — 이 시스템은 실제 시뮬/렌더를 하지 않고 GDF 필요 신호만 전달한다.

#pragma once

#include "CoreMinimal.h"
#include "FXSystem.h"

class FRopeGDFFXSystem final : public FFXSystemInterface
{
public:
	/** RegisterCustomFXSystem에 넘길 고유 이름. */
	static const FName Name;

	FRopeGDFFXSystem(ERHIFeatureLevel::Type InFeatureLevel, EShaderPlatform InShaderPlatform, FGPUSortManager* InGPUSortManager)
		: FeatureLevel(InFeatureLevel)
		, ShaderPlatform(InShaderPlatform)
		, GPUSortManager(InGPUSortManager)
	{
	}

	//~ 핵심: 이 씬에 GDF 로프가 활성이면 true → 엔진이 GDF를 빌드한다.
	virtual bool UsesGlobalDistanceField() const override;

	//~ GPUSortManager는 생성 시 받은 것을 그대로 돌려줘야 한다(렌더 루프가 사용).
	virtual FGPUSortManager* GetGPUSortManager() const override { return GPUSortManager; }

	//~ 이하 순수가상 스텁(이 시스템은 파티클/벡터필드/GPU 시뮬을 하지 않음).
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

/** RegisterCustomFXSystem 팩토리. 씬 생성 시 엔진이 호출해 sibling FX 시스템을 만든다. */
FFXSystemInterface* CreateRopeGDFFXSystem(ERHIFeatureLevel::Type InFeatureLevel, EShaderPlatform InShaderPlatform, FGPUSortManager* InGPUSortManager);
