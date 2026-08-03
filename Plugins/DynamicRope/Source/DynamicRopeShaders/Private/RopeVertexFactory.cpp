// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeVertexFactory.h"

// FGPUSkinPassThroughFactoryLooseParameters, the engine-declared loose parameter struct the
// passthrough branch of LocalVertexFactory.ush reads. Declared ENGINE_API, so a plugin can create
// and bind uniform buffers of it even though the engine's passthrough vertex factory itself is not
// exported.
#include "GPUSkinVertexFactory.h"
// GNullVertexBuffer, backing the null loose parameters.
#include "GlobalRenderResources.h"
#include "MaterialShared.h"
#include "MeshDrawShaderBindings.h"
#include "MeshMaterialShader.h"
#include "RenderResource.h"

/**
 * The always-valid loose parameter binding for frames and platforms where the passthrough branch is
 * off. The compiled shader references the uniform buffer whenever the platform supports the
 * passthrough path, even though the runtime branch never takes it, so a binding must always exist.
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
 * done here directly. The passthrough flag is bound off and the loose parameters bound null, which
 * makes the factory render identically to FLocalVertexFactory.
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
		ShaderBindings.Add(IsGPUSkinPassThrough, 0u);

		// Decode VertexFactoryUserData as VertexFactoryUniformBuffer, as the engine parameter class
		// does on its non-passthrough path.
		FRHIUniformBuffer* VertexFactoryUniformBuffer = static_cast<FRHIUniformBuffer*>(BatchElement.VertexFactoryUserData);
		GetElementShaderBindingsBase(Scene, View, Shader, InputStreamType, FeatureLevel, VertexFactory,
			BatchElement, VertexFactoryUniformBuffer, ShaderBindings, VertexStreams);

		ShaderBindings.Add(Shader->GetUniformBufferParameter<FGPUSkinPassThroughFactoryLooseParameters>(),
			GRopePassThroughNullUniformBuffer);
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
