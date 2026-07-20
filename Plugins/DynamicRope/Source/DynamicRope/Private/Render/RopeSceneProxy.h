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
class FRopeGPUSolver;

/** render thread로 넘기는 dynamic 데이터: component-local 공간의 centerline. */
struct FRopeDynamicData
{
	TArray<FVector> Points;
	// M5b: 이 프레임에 로프가 GPU에서 step됐는가 → true면 GPU 튜브가 resident PosBuf를 직접 읽어도 됨(무지연).
	// false(whip/CPU-폴백/솔버 off)면 resident는 stale이므로 위 Points(CPU 미러)로 그린다.
	bool bGpuResident = false;
	// 이 프레임 GT 시드 generation. resident PosBuf가 같은 세대일 때만 직접 읽는다 — 같은 노드 수로
	// 재시드(재던지기)하면 버퍼는 아직 옛 세대라, 세대를 안 보면 직전 로프 포즈가 한 프레임 유령으로 뜬다.
	uint32 SimGeneration = 0;
	// resident 튜브(월드 PosBuf → component-local)용 변환 — Points를 로컬화한 것과 *같은* GT 프레임의
	// GetComponentTransform() 역행렬. 프록시의 GetLocalToWorld()를 쓰면 안 된다: SetDynamicData 렌더 커맨드는
	// 이번 프레임 트랜스폼이 프록시에 적용되는 UpdateAllPrimitiveSceneInfos보다 먼저 실행돼 한 프레임 이전
	// 값을 읽는다 → 빌드(N-1)와 드로우(N) 트랜스폼이 어긋나 월드 고정점(wrap 노드)이 컴포넌트 이동량만큼 떨린다.
	FMatrix44f WorldToLocal = FMatrix44f::Identity;
};

/** Dynamic index buffer (topology은 proxy의 수명 동안 고정된다). */
class FRopeIndexBuffer final : public FIndexBuffer
{
public:
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	int32 NumIndices = 0;
};

/**
 * M5b: 컴퓨트가 써넣는 UAV 가능 position vertex buffer(VF의 position stream). GPU write라 non-dynamic.
 * R32_FLOAT 타입 SRV(PositionComponentSRV) + UAV(컴퓨트 write). 정점 v 위치 = float[v*3..].
 */
class FRopeGpuPositionBuffer final : public FVertexBuffer
{
public:
	int32 NumVertices = 0;
	FShaderResourceViewRHIRef SRV;
	FUnorderedAccessViewRHIRef UAV;
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	virtual void ReleaseRHI() override;
};

/**
 * M5b(B1 임시): 매 프레임 CPU centerline(component-local)을 올려 튜브 컴퓨트가 읽는 버퍼. Dynamic + R32_FLOAT SRV.
 * (B2에서 센터라인 소스를 솔버 PosBuf로 바꾸면 제거된다.)
 */
class FRopeCenterlineBuffer final : public FVertexBuffer
{
public:
	int32 NumFloats = 0;
	FShaderResourceViewRHIRef SRV;
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	virtual void ReleaseRHI() override;
};

/**
 * B2-full: 컴퓨트가 써넣는 tangent basis 버퍼(VET_Short4N ×2 = TangentX/TangentZ, high-precision SNORM16).
 * v당 16바이트(TangentX@0, TangentZ@8). UAV는 R32_UINT(컴퓨트 v당 uint4), 매뉴얼 페치 SRV는 R16G16B16A16_SNORM.
 */
class FRopeGpuTangentBuffer final : public FVertexBuffer
{
public:
	int32 NumVertices = 0;
	// PF_R16G16B16A16_SNORM(매뉴얼 페치)
	FShaderResourceViewRHIRef SRV;
	// PF_R32_UINT(컴퓨트 write)
	FUnorderedAccessViewRHIRef UAV;
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	virtual void ReleaseRHI() override;
};

/** B2-full: 컴퓨트가 써넣는 UV 버퍼(VET_Float2). v당 2 float. UAV/SRV 모두 R32_FLOAT(SRV는 매뉴얼 페치용 G32R32F). */
class FRopeGpuTexCoordBuffer final : public FVertexBuffer
{
public:
	int32 NumVertices = 0;
	// PF_G32R32F(매뉴얼 페치)
	FShaderResourceViewRHIRef SRV;
	// PF_R32_FLOAT(컴퓨트 write)
	FUnorderedAccessViewRHIRef UAV;
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	virtual void ReleaseRHI() override;
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

	/** Render-thread: tube의 vertex/index를 재생성해 GPU buffer로 업로드한다(CPU 경로). */
	void BuildTube(FRHICommandListBase& RHICmdList, const FRopeDynamicData& Data);

	/**
	 * GPU 컴퓨트로 튜브 정점(position/tangent basis/UV)을 생성한다(B2-full). CPU BuildTube 불필요.
	 * 센터라인 소스: resident PosBuf(Subdiv=1 + GPU step 프레임) 또는 CPU 미러(Data.Points, Catmull-Rom
	 * 스무딩). GPU 튜브 상시화(RHI+링<=256) 시 이 경로가 기본이고, 아니면 CPU BuildTube로 폴백한다.
	 */
	void BuildTubeGPU(FRHICommandListBase& RHICmdList, const FRopeDynamicData& Data);

	/**
	 * 시뮬 노드(NumNodes)를 Catmull-Rom으로 세그먼트당 Subdiv회 서브분할해 렌더 센터라인(NumRings)을 만든다.
	 * 곡률은 이웃 노드로 추정(접선 ≈ (P[i+1]-P[i-1])/2) — 물리와 완전 분리된 렌더 전용 스무딩. Subdiv=1이면 1:1.
	 */
	void BuildSmoothedCenterline(const TArray<FVector>& Nodes, TArray<FVector>& Out) const;

	UMaterialInterface* Material;
	FStaticMeshVertexBuffers VertexBuffers;
	FRopeIndexBuffer IndexBuffer;
	FLocalVertexFactory VertexFactory;
	FMaterialRelevance MaterialRelevance;

	// GPU 튜브 경로 여부. 상시화: 렌더 가능 RHI + NumRings<=256이면 true(자동), 아니면 CPU BuildTube 폴백.
	// proxy 생성 시점에 한 번 결정(링 수는 proxy 수명 동안 고정).
	bool bUseGpuTube = false;
	FRopeGpuPositionBuffer GpuPositionBuffer;
	// B2-full
	FRopeGpuTangentBuffer  GpuTangentBuffer;
	// B2-full
	FRopeGpuTexCoordBuffer GpuTexCoordBuffer;
	FRopeCenterlineBuffer  CenterlineBuffer;
	// B2-full: index topology + white color를 GPU 경로에서 1회만 채운다.
	bool bGpuStaticsBuilt = false;

	/** B2-full: GPU 튜브 경로의 index topology + 상수 color(white)를 1회 채운다(매 프레임 CPU BuildTube 대체). */
	void BuildGpuStaticBuffers(FRHICommandListBase& RHICmdList);

	// M5b B2-lite: 솔버 resident PosBuf를 직접 읽어 렌더 지연 최소화. 솔버는 월드 수명이라 proxy 동안 유효(없으면 B1 폴백).
	// GDF 모드(솔브가 PreRenderBasePass)에서는 이 읽기가 직전 프레임 솔브 결과다 — 의도된 1프레임 지연.
	// 튜브를 솔브 뒤(prepass 이후)에 덮어쓰면 prepass/base pass 지오메트리가 어긋나 EQUAL 깊이 테스트에서
	// 픽셀이 탈락(로프가 검게 탐)하므로, 튜브는 프레임 초 1회 빌드로 모든 패스에 일관되게 그린다.
	FRopeGPUSolver* SolverPtr = nullptr;
	uint32 RopeId = 0;

	// 시뮬 센터라인 노드 수(= Component->NumParticles). Data.Points가 이 개수여야 한다.
	int32 NumNodes;
	// 렌더 튜브 세그먼트당 Catmull-Rom 서브분할(1=off). Component->TubeSmoothingSubdiv(링 상한 자동 하향).
	int32 Subdiv;
	// 렌더 링(스무딩된 센터라인) 수 = (NumNodes-1)*Subdiv+1. vertex/index 토폴로지 기준.
	int32 NumRings;
	int32 NumSides;
	float Radius;
	// Catmull-Rom knot α(0=uniform, 0.5=centripetal). Component->TubeSmoothingAlpha. 생성 시 1회.
	float SmoothParam;
	// velocity 버퍼 기록 여부(UDynamicRopeSettings::bWriteVelocity 스냅샷). GetViewRelevance가 읽음.
	bool  bWriteVelocity;
	bool  bHasData = false;
};
