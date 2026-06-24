// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope의 렌더 proxy. parallel-transport frame을 사용해 centerline 주위에 tube를 생성한다
// (Frenet twist-pop 없음). 엔진의 FCableSceneProxy를 본떴다: 지속적인
// FLocalVertexFactory + 매 프레임 갱신되는 실제 vertex/index buffer, 그리고 제대로 된 primitive
// uniform buffer를 사용한다. 이것이 depth/occlusion/velocity와 올바르게 통합되는 방식이다
// (FDynamicMeshBuilder one-shot 경로는 그렇지 못하다). render-thread 전용이며 모듈 외부로 export되지 않는다.

#pragma once

#include "CoreMinimal.h"
#include "PrimitiveSceneProxy.h"
#include "MaterialShared.h"
#include "StaticMeshResources.h"
#include "LocalVertexFactory.h"
#include "RHIResources.h"

class URopeComponent;
class UMaterialInterface;

/** render thread로 넘기는 dynamic 데이터: component-local 공간의 centerline. */
struct FRopeDynamicData
{
	TArray<FVector> Points;
};

/** Dynamic index buffer (topology은 proxy의 수명 동안 고정된다). */
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

	/** 새 centerline(component-local)로부터 tube를 다시 만든다. NewData의 소유권을 가져간다. */
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

	/** Render-thread: tube의 vertex/index를 재생성해 GPU buffer로 업로드한다. */
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
