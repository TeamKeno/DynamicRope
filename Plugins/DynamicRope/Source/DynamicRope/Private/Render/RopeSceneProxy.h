// Copyright Epic Games, Inc. All Rights Reserved.
//
// Render proxy for the rope. Builds a tube around the centerline using parallel-transport
// frames (no Frenet twist-pop). Modeled on the engine's FCableSceneProxy: a persistent
// FLocalVertexFactory + real vertex/index buffers updated each frame, with a proper primitive
// uniform buffer. This is what integrates correctly with depth/occlusion/velocity (the
// FDynamicMeshBuilder one-shot path does not). Render-thread only; not module-exported.

#pragma once

#include "CoreMinimal.h"
#include "PrimitiveSceneProxy.h"
#include "MaterialShared.h"
#include "StaticMeshResources.h"
#include "LocalVertexFactory.h"
#include "RHIResources.h"

class URopeComponent;
class UMaterialInterface;

/** Dynamic data handed to the render thread: the centerline in component-local space. */
struct FRopeDynamicData
{
	TArray<FVector> Points;
};

/** Dynamic index buffer (topology is fixed for the proxy's lifetime). */
class FRopeIndexBuffer final : public FIndexBuffer
{
public:
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	int32 NumIndices = 0;
};

class FRopeSceneProxy final : public FPrimitiveSceneProxy
{
public:
	SIZE_T GetTypeHash() const override;

	explicit FRopeSceneProxy(URopeComponent* Component);
	virtual ~FRopeSceneProxy() override;

	/** Rebuild the tube from a new centerline (component-local). Takes ownership of NewData. */
	void SetDynamicData_RenderThread(FRHICommandListBase& RHICmdList, FRopeDynamicData* NewData);

	virtual void DrawStaticElements(FStaticPrimitiveDrawInterface* PDI) override;
	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily,
		uint32 VisibilityMap, FMeshElementCollector& Collector) const override;
	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override;
	virtual uint32 GetMemoryFootprint() const override { return sizeof(*this) + GetAllocatedSize(); }
	uint32 GetAllocatedSize() const { return FPrimitiveSceneProxy::GetAllocatedSize(); }

private:
	int32 GetRequiredVertexCount() const { return NumRings * (NumSides + 1); }
	int32 GetRequiredIndexCount() const { return (NumRings - 1) * NumSides * 2 * 3; }
	int32 GetVertIndex(int32 RingIdx, int32 SideIdx) const { return RingIdx * (NumSides + 1) + SideIdx; }

	/** Render-thread: regenerate the tube vertices/indices and upload to the GPU buffers. */
	void BuildTube(FRHICommandListBase& RHICmdList, const FRopeDynamicData& Data);

	UMaterialInterface* Material;
	FStaticMeshVertexBuffers VertexBuffers;
	FRopeIndexBuffer IndexBuffer;
	FLocalVertexFactory VertexFactory;
	FMaterialRelevance MaterialRelevance;

	int32 NumRings;
	int32 NumSides;
	float Radius;
	bool  bHasData = false;
};
