// Copyright 2026 TeamKeno. All Rights Reserved.

#include "RopeVertexFactory.h"

// GNullVertexBuffer, backing the loose parameters while the passthrough is inactive or has no data.
#include "GlobalRenderResources.h"
#include "MaterialShared.h"
// FMeshBatchElement, dereferenced in GetElementShaderBindings for VertexFactoryUserData. Reached only
// through a unity blob otherwise, so a non-unity build - which is what BuildPlugin runs - fails without it.
#include "MeshBatch.h"
#include "MeshDrawShaderBindings.h"
#include "MeshMaterialShader.h"
#include "RenderResource.h"

// Everything below implements the rope's own vertex factory type, which only exists from UE 5.6 on;
// see ROPE_WITH_VELOCITY_PASSTHROUGH in RopeVertexFactory.h. On 5.5 this file compiles to nothing and
// the header's fallback class renders through the engine's local vertex factory instead.
#if ROPE_WITH_VELOCITY_PASSTHROUGH
/**
 * The always-valid loose parameter binding for factories whose passthrough is disabled. The
 * compiled shader references the uniform buffer whenever the platform supports the passthrough
 * path, even though the runtime branch never takes it, so a binding must always exist.
 * Mirrors the engine's file-local null buffer in LocalVertexFactory.cpp.
 */
class FRopePassThroughNullUniformBuffer : public TUniformBuffer<FGPUSkinPassThroughFactoryLooseParameters>
{
	typedef TUniformBuffer<FGPUSkinPassThroughFactoryLooseParameters> Super;
public:
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override
	{
		FGPUSkinPassThroughFactoryLooseParameters Parameters;
		Parameters.FrameNumber = -1;
		Parameters.PositionBuffer = GNullVertexBuffer.VertexBufferSRV;
		Parameters.PreviousPositionBuffer = GNullVertexBuffer.VertexBufferSRV;
		Parameters.PreSkinnedTangentBuffer = GNullVertexBuffer.VertexBufferSRV;
		SetContentsNoUpdate(Parameters);
		Super::InitRHI(RHICmdList);
	}
};

static TGlobalResource<FRopePassThroughNullUniformBuffer> GRopePassThroughNullUniformBuffer;

/**
 * The parameter class for FRopeVertexFactory. It mirrors FLocalVertexFactoryShaderParameters with
 * one structural difference: the engine class reaches the passthrough loose parameters by casting
 * the factory to FGPUSkinPassthroughVertexFactory, which this factory is not, so the binding is
 * done here directly from FRopeVertexFactory's own state. Unlike the engine's passthrough, the
 * rope's position stream already holds the current positions, so the base stream and uniform-buffer
 * bindings are identical in both modes and only the flag and the loose parameters differ.
 */
class FRopeVertexFactoryShaderParameters : public FLocalVertexFactoryShaderParametersBase
{
	DECLARE_TYPE_LAYOUT(FRopeVertexFactoryShaderParameters, NonVirtual);

public:
	void Bind(const FShaderParameterMap& ParameterMap)
	{
		FLocalVertexFactoryShaderParametersBase::Bind(ParameterMap);
		IsGPUSkinPassThrough.Bind(ParameterMap, TEXT("bIsGPUSkinPassThrough"));
	}

	void GetElementShaderBindings(
		const FSceneInterface* Scene,
		const FSceneView* View,
		const FMeshMaterialShader* Shader,
		const EVertexInputStreamType InputStreamType,
		ERHIFeatureLevel::Type FeatureLevel,
		const FVertexFactory* VertexFactory,
		const FMeshBatchElement& BatchElement,
		FMeshDrawSingleShaderBindings& ShaderBindings,
		FVertexInputStreamArray& VertexStreams) const
	{
		const FRopeVertexFactory* RopeVertexFactory = static_cast<const FRopeVertexFactory*>(VertexFactory);
		const bool bPassThrough = RopeVertexFactory->IsVelocityPassThroughEnabled()
			&& RopeVertexFactory->GetLooseParametersUniformBuffer().IsValid();

		ShaderBindings.Add(IsGPUSkinPassThrough, bPassThrough ? 1u : 0u);

		// Decode VertexFactoryUserData as VertexFactoryUniformBuffer, as the engine parameter class
		// does on its non-passthrough path.
		FRHIUniformBuffer* VertexFactoryUniformBuffer = static_cast<FRHIUniformBuffer*>(BatchElement.VertexFactoryUserData);
		GetElementShaderBindingsBase(Scene, View, Shader, InputStreamType, FeatureLevel, VertexFactory,
			BatchElement, VertexFactoryUniformBuffer, ShaderBindings, VertexStreams);

		if (bPassThrough)
		{
			ShaderBindings.Add(Shader->GetUniformBufferParameter<FGPUSkinPassThroughFactoryLooseParameters>(),
				RopeVertexFactory->GetLooseParametersUniformBuffer());
		}
		else
		{
			ShaderBindings.Add(Shader->GetUniformBufferParameter<FGPUSkinPassThroughFactoryLooseParameters>(),
				GRopePassThroughNullUniformBuffer);
		}
	}

private:
	LAYOUT_FIELD(FShaderParameter, IsGPUSkinPassThrough);
};

IMPLEMENT_TYPE_LAYOUT(FRopeVertexFactoryShaderParameters);

FRopeVertexFactory::FRopeVertexFactory(ERHIFeatureLevel::Type InFeatureLevel, const char* InDebugName)
	: FLocalVertexFactory(InFeatureLevel, InDebugName)
{
}

bool FRopeVertexFactory::ShouldCompilePermutation(const FVertexFactoryShaderPermutationParameters& Parameters)
{
	return Parameters.MaterialParameters.bIsUsedWithSkeletalMesh || Parameters.MaterialParameters.bIsSpecialEngineMaterial;
}

void FRopeVertexFactory::SetVelocityPassThroughEnabled(bool bEnabled)
{
	checkf(!IsInitialized(), TEXT("The velocity passthrough is baked into cached draw bindings and must be decided before InitResource."));
	bVelocityPassThrough = bEnabled;
}

void FRopeVertexFactory::InitRHI(FRHICommandListBase& RHICmdList)
{
	FLocalVertexFactory::InitRHI(RHICmdList);

	if (bVelocityPassThrough)
	{
		// Created before the first mesh draw command is cached; a stale frame number keeps the
		// shader on zero deformation velocity until the proxy's first real update.
		FGPUSkinPassThroughFactoryLooseParameters Parameters;
		Parameters.FrameNumber = -1;
		Parameters.PositionBuffer = GNullVertexBuffer.VertexBufferSRV;
		Parameters.PreviousPositionBuffer = GNullVertexBuffer.VertexBufferSRV;
		Parameters.PreSkinnedTangentBuffer = GNullVertexBuffer.VertexBufferSRV;
		LooseParametersUniformBuffer = TUniformBufferRef<FGPUSkinPassThroughFactoryLooseParameters>::CreateUniformBufferImmediate(
			Parameters, UniformBuffer_MultiFrame);
	}
}

void FRopeVertexFactory::ReleaseRHI()
{
	LooseParametersUniformBuffer.SafeRelease();
	FLocalVertexFactory::ReleaseRHI();
}

void FRopeVertexFactory::UpdateLooseParameters(FRHICommandListBase& RHICmdList, uint32 FrameNumber,
	FRHIShaderResourceView* PositionSRV, FRHIShaderResourceView* PreviousPositionSRV,
	FRHIShaderResourceView* PreSkinnedTangentSRV)
{
	if (!LooseParametersUniformBuffer.IsValid())
	{
		return;
	}

	FGPUSkinPassThroughFactoryLooseParameters Parameters;
	Parameters.FrameNumber = FrameNumber;
	Parameters.PositionBuffer = PositionSRV ? PositionSRV : GNullVertexBuffer.VertexBufferSRV.GetReference();
	Parameters.PreviousPositionBuffer = PreviousPositionSRV ? PreviousPositionSRV : GNullVertexBuffer.VertexBufferSRV.GetReference();
	Parameters.PreSkinnedTangentBuffer = PreSkinnedTangentSRV ? PreSkinnedTangentSRV : GNullVertexBuffer.VertexBufferSRV.GetReference();
	LooseParametersUniformBuffer.UpdateUniformBufferImmediate(RHICmdList, Parameters);
}

IMPLEMENT_VERTEX_FACTORY_PARAMETER_TYPE(FRopeVertexFactory, SF_Vertex, FRopeVertexFactoryShaderParameters);
#if RHI_RAYTRACING
IMPLEMENT_VERTEX_FACTORY_PARAMETER_TYPE(FRopeVertexFactory, SF_RayHitGroup, FRopeVertexFactoryShaderParameters);
IMPLEMENT_VERTEX_FACTORY_PARAMETER_TYPE(FRopeVertexFactory, SF_Compute, FRopeVertexFactoryShaderParameters);
#endif

// The flag set matches FLocalVertexFactory's own registration one to one, which keeps every engine
// code path that inspects the flags behaving identically; SupportsGPUSkinPassThrough is the one
// that matters here, since it makes ModifyCompilationEnvironment compile the passthrough branch in.
IMPLEMENT_VERTEX_FACTORY_TYPE(FRopeVertexFactory, "/Engine/Private/LocalVertexFactory.ush",
	  EVertexFactoryFlags::UsedWithMaterials
	| EVertexFactoryFlags::SupportsStaticLighting
	| EVertexFactoryFlags::SupportsDynamicLighting
	| EVertexFactoryFlags::SupportsPrecisePrevWorldPos
	| EVertexFactoryFlags::SupportsPositionOnly
	| EVertexFactoryFlags::SupportsCachingMeshDrawCommands
	| EVertexFactoryFlags::SupportsPrimitiveIdStream
	| EVertexFactoryFlags::SupportsRayTracing
	| EVertexFactoryFlags::SupportsRayTracingDynamicGeometry
	| EVertexFactoryFlags::SupportsLightmapBaking
	| EVertexFactoryFlags::SupportsManualVertexFetch
	| EVertexFactoryFlags::SupportsPSOPrecaching
	| EVertexFactoryFlags::SupportsGPUSkinPassThrough
	| EVertexFactoryFlags::SupportsLumenMeshCards
	| EVertexFactoryFlags::SupportsTriangleSorting
);

#endif
