// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The rope's render proxy. It builds a tube around the centreline using a parallel-transport frame,
// which avoids the twist pop of a Frenet frame. It follows the engine's FCableSceneProxy: a persistent
// vertex factory (FRopeVertexFactory, the plugin's FLocalVertexFactory subtype carrying the per-vertex
// velocity hooks), real vertex and index buffers refreshed each frame, and a proper primitive
// uniform buffer. That is what integrates correctly with depth, occlusion and velocity, which the
// one-shot FDynamicMeshBuilder path does not. It is render-thread only and is not exported outside the
// module.

#pragma once

#include "CoreMinimal.h"
#include "PrimitiveSceneProxy.h"
#include "MaterialShared.h"
#include "StaticMeshResources.h"
#include "RopeVertexFactory.h"
#include "RHIResources.h"

class URopeComponent;
class UMaterialInterface;
class FRopeGPUSolver;

/** The dynamic data handed to the render thread: the centreline in component-local space. */
struct FRopeDynamicData
{
	TArray<FVector> Points;
	// Whether the rope was stepped on the GPU this frame. When true the GPU tube may read the resident
	// position buffer directly, with no latency. When false, as during a whip, on the CPU fallback, or
	// with the solver off, the resident buffer is stale and the CPU mirror in Points above is drawn
	// instead.
	bool bGpuResident = false;
	// This frame's game-thread seed generation. The resident position buffer is read directly only while
	// the generations match: reseeding with the same node count, as a re-throw does, leaves the buffer on
	// the older generation, and ignoring that would show the previous rope pose as a one-frame ghost.
	uint32 SimGeneration = 0;
	// The transform for the resident tube, converting the world-space position buffer into component-local
	// space. It is the inverse of GetComponentTransform() from the same game-thread frame that localized
	// Points. The proxy's own GetLocalToWorld() must not be used: the SetDynamicData render command runs
	// before UpdateAllPrimitiveSceneInfos applies this frame's transform to the proxy, so it would read
	// the previous frame's value, and a build at frame N-1 drawn with the transform of frame N makes a
	// world-fixed point, such as a wrap node, jitter by exactly the component's movement.
	FMatrix44f WorldToLocal = FMatrix44f::Identity;
	// The game-thread GFrameCounter of the frame that produced this data. The vertex factory stamps it
	// into the velocity loose parameters, and the shader outputs deformation velocity only when it equals
	// the view's frame counter, which is captured from the same game-thread counter when the view family
	// is created. Any mismatch - a skipped update, a scene capture built on another frame - degrades to
	// zero deformation velocity instead of reading a wrong previous position.
	uint32 FrameNumber = 0;
};

/** The dynamic index buffer; the topology is fixed for the lifetime of the proxy. */
class FRopeIndexBuffer final : public FIndexBuffer
{
public:
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	int32 NumIndices = 0;
};

/**
 * The UAV-capable position vertex buffer the compute shader writes, which is the vertex factory's
 * position stream. It is written by the GPU and therefore not dynamic.
 * It has an R32_FLOAT typed SRV, used as the position component SRV, plus a UAV for the compute write.
 * Vertex v's position is at float indices v*3 onwards.
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
 * The buffer the tube compute shader reads, filled each frame by uploading the CPU centreline in
 * component-local space. Dynamic, with an R32_FLOAT SRV.
 * It becomes unnecessary once the centreline source is the solver's position buffer.
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
 * The tangent basis buffer the compute shader writes, as two VET_Short4N streams holding the tangent X
 * and Z axes in high-precision SNORM16.
 * That is 16 bytes per vertex, with tangent X at offset 0 and tangent Z at offset 8. The UAV is
 * R32_UINT, since the compute shader writes a uint4 per vertex, and the manual-fetch SRV is
 * R16G16B16A16_SNORM.
 */
class FRopeGpuTangentBuffer final : public FVertexBuffer
{
public:
	int32 NumVertices = 0;
	// PF_R16G16B16A16_SNORM, for the manual fetch.
	FShaderResourceViewRHIRef SRV;
	// PF_R32_UINT, for the compute write.
	FUnorderedAccessViewRHIRef UAV;
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	virtual void ReleaseRHI() override;
};

/** The UV buffer the compute shader writes, as VET_Float2 with two floats per vertex. Both the UAV and
 *  the SRV are R32_FLOAT, the latter as G32R32F for the manual fetch. */
class FRopeGpuTexCoordBuffer final : public FVertexBuffer
{
public:
	int32 NumVertices = 0;
	// PF_G32R32F, for the manual fetch.
	FShaderResourceViewRHIRef SRV;
	// PF_R32_FLOAT, for the compute write.
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

	/** Rebuilds the tube from a new component-local centreline, taking ownership of NewData. */
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

	/** Render thread: regenerates the tube's vertices and indices and uploads them to the GPU buffers.
	 *  This is the CPU path. */
	void BuildTube(FRHICommandListBase& RHICmdList, const FRopeDynamicData& Data);

	/**
	 * Generates the tube's vertices, meaning the positions, tangent basis and UVs, on the GPU, which
	 * makes the CPU tube build unnecessary.
	 * The centreline source is either the resident position buffer, on a GPU step frame with no
	 * subdivision, or the CPU mirror in Data.Points with Catmull-Rom smoothing. Wherever the GPU tube
	 * applies, meaning there is an RHI and the ring count is within the limit, this is the default path;
	 * otherwise it falls back to the CPU tube build.
	 */
	void BuildTubeGPU(FRHICommandListBase& RHICmdList, const FRopeDynamicData& Data);

	/**
	 * Subdivides the simulation nodes with Catmull-Rom interpolation, by the subdivision factor per
	 * segment, to produce the render centreline.
	 * The curvature is estimated from the neighbouring nodes, with the tangent approximated as half the
	 * difference between the next and previous nodes. It is render-only smoothing, entirely separate from
	 * the physics. A subdivision of 1 is one-to-one.
	 */
	void BuildSmoothedCenterline(const TArray<FVector>& Nodes, TArray<FVector>& Out) const;

	UMaterialInterface* Material;
	FStaticMeshVertexBuffers VertexBuffers;
	FRopeIndexBuffer IndexBuffer;
	FRopeVertexFactory VertexFactory;
	FMaterialRelevance MaterialRelevance;

	// Whether the GPU tube path is used: true automatically when there is a renderable RHI and the ring
	// count is within the maximum, and otherwise the CPU tube build is the fallback.
	// It is decided once when the proxy is created, since the ring count is fixed for its lifetime.
	bool bUseGpuTube = false;
	FRopeGpuPositionBuffer GpuPositionBuffer;
	// Tangent basis, generated alongside the positions by the GPU tube
	FRopeGpuTangentBuffer  GpuTangentBuffer;
	// UVs, generated alongside the positions by the GPU tube
	FRopeGpuTexCoordBuffer GpuTexCoordBuffer;
	FRopeCenterlineBuffer  CenterlineBuffer;
	// Last frame's tube positions, for per-vertex velocity: before each frame's positions are written,
	// the current buffer - GpuPositionBuffer on the GPU tube, the position vertex buffer on the CPU
	// fallback - is copied here, and the vertex factory's loose parameters point the shader at it as the
	// previous-position source. Same layout as the position stream, three floats per vertex, read through
	// the R32_FLOAT SRV; the UAV the shared buffer class also creates goes unused. Initialized only while
	// bVelocityActive.
	FRopeGpuPositionBuffer PrevPositionBuffer;
	// Fills the index topology and the constant white colour once on the GPU path.
	bool bGpuStaticsBuilt = false;

	/** Fills the index topology and the constant white colour of the GPU tube path once, which replaces
	 *  building the tube on the CPU every frame. */
	void BuildGpuStaticBuffers(FRHICommandListBase& RHICmdList);

	// Reads the solver's resident position buffer directly, which minimizes render latency. The solver
	// lives as long as the world, so it stays valid for the proxy's lifetime; without one it falls back to
	// uploading the CPU centreline.
	// On the global distance field path, where the solve runs during PreRenderBasePass, this read is the
	// previous frame's solve result, which is an intended one-frame delay.
	// Overwriting the tube after the solve, that is after the prepass, would leave the prepass and base
	// pass geometry inconsistent and the pixels would fail the equal-depth test, turning the rope black,
	// so the tube is built once at the start of the frame and drawn consistently in every pass.
	FRopeGPUSolver* SolverPtr = nullptr;
	uint32 RopeId = 0;

	// The number of simulation centreline nodes, which equals the component's particle count. Data.Points
	// has to hold exactly this many.
	int32 NumNodes;
	// Catmull-Rom subdivisions per render tube segment, where 1 disables it. It comes from the component's
	// tube smoothing subdivision, reduced automatically to fit the ring limit.
	int32 Subdiv;
	// The number of render rings, that is the smoothed centreline, which is (NumNodes - 1) * Subdiv + 1.
	// It is what the vertex and index topology is sized from.
	int32 NumRings;
	int32 NumSides;
	float Radius;
	// The Catmull-Rom knot parameter, where 0 is uniform and 0.5 is centripetal, taken from the component
	// once at creation.
	float SmoothParam;
	// Whether the per-vertex velocity path is active: the UDynamicRopeSettings::bWriteVelocity project
	// setting, snapshotted at creation, and the platform supporting the GPU-skin passthrough shader
	// branch. Drives the previous-position copy, the loose parameter updates and bVelocityRelevance;
	// when false the rope writes no velocity at all, as on unsupported platforms there is no correct
	// motion vector to write.
	bool  bVelocityActive;
	bool  bHasData = false;
	// The seed generation the previous-position buffer was copied under. After a reseed the buffer holds
	// the pre-reseed pose, so velocity is suppressed for that frame by stamping a stale frame number.
	uint32 LastVelocitySimGeneration = 0;

	/** Render thread, before the frame's tube build: copies the still-untouched current positions into
	 *  PrevPositionBuffer and refreshes the vertex factory's velocity loose parameters. */
	void UpdatePreviousPositions(FRHICommandListImmediate& RHICmdList, const FRopeDynamicData& Data);
};
