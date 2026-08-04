// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeTubeBuilder.h"
#include "DynamicRopeShadersLog.h"

#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RHICommandList.h"
// FComputeShaderUtils
#include "RenderGraphUtils.h"
// PipelineStateCache::PrecacheComputePipelineState — PrecacheTubeComputePSOs
#include "PipelineStateCache.h"
#include "DataDrivenShaderPlatformInfo.h"
// Tube build bandwidth instrumentation for 'stat DynamicRope'. The stat group is declared in the module's shared header.
#include "RenderingThread.h"
#include "Stats/Stats.h"
#include "RopeGPUStatGroup.h"

// The ring buckets, whose size is both the thread group size and the size of the groupshared frame array. One rope is
// one thread group, so the smallest bucket at least as large as the ring count is selected as a permutation to reduce
// idle threads. The largest, 512, is the GPU tube's ring limit, which covers roughly 170 nodes at a subdivision of 3;
// anything larger falls back to the CPU. The thread count and the groupshared array scale from ROPE_TUBE_MAX_RINGS in
// the shader, which the permutation sets to the bucket value. The groupshared budget is five float3 per ring, meaning
// 60 bytes, so 512 rings is 30 KB, under the 32 KB limit.
static constexpr int32 GRopeTubeRingBuckets[] = { 64, 128, 256, 512 };
// The largest bucket is the GPU tube's ring limit, beyond which it falls back to the CPU.
static constexpr int32 ROPE_TUBE_MAX_RINGS_CAP = 512;

int32 RopeGPU::TubeRingBucket(int32 NumRings)
{
	for (int32 Bucket : GRopeTubeRingBuckets)
	{
		if (NumRings <= Bucket) { return Bucket; }
	}
	// Beyond the limit, so the caller falls back to the CPU tube.
	return 0;
}

int32 RopeGPU::MaxTubeRings()
{
	return ROPE_TUBE_MAX_RINGS_CAP;
}

int32 RopeGPU::ComputeTubeSubdiv(int32 NumNodes, int32 WantedSubdiv)
{
	const int32 Wanted = FMath::Clamp(WantedSubdiv, 1, 8);
	if (NumNodes <= 2)
	{
		return Wanted;
	}
	const int32 MaxForGpu = FMath::Max(1, (ROPE_TUBE_MAX_RINGS_CAP - 1) / (NumNodes - 1));
	return FMath::Min(Wanted, MaxForGpu);
}

class FRopeBuildTubeCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeBuildTubeCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeBuildTubeCS, FGlobalShader);

	// The ring bucket is the thread count and the groupshared size. The value is passed to the shader as ROPE_TUBE_MAX_RINGS.
	class FRingBucket : SHADER_PERMUTATION_SPARSE_INT("ROPE_TUBE_MAX_RINGS", 64, 128, 256, 512);
	using FPermutationDomain = TShaderPermutationDomain<FRingBucket>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRings)
		SHADER_PARAMETER(uint32, NumSides)
		SHADER_PARAMETER(float, Radius)
		SHADER_PARAMETER_SRV(Buffer<float>, InCenterline)
		SHADER_PARAMETER_UAV(RWBuffer<float>, OutPositions)
		SHADER_PARAMETER_UAV(RWBuffer<uint>, OutTangents)
		SHADER_PARAMETER_UAV(RWBuffer<float>, OutTexCoords)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}
};

IMPLEMENT_GLOBAL_SHADER(FRopeBuildTubeCS, "/Plugin/DynamicRope/Private/RopeBuildTube.usf", "RopeBuildTubeCS", SF_Compute);

// Builds directly from the resident world-space position buffer, applying the world-to-local transform.
class FRopeBuildTubeResidentCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeBuildTubeResidentCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeBuildTubeResidentCS, FGlobalShader);

	class FRingBucket : SHADER_PERMUTATION_SPARSE_INT("ROPE_TUBE_MAX_RINGS", 64, 128, 256, 512);
	using FPermutationDomain = TShaderPermutationDomain<FRingBucket>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRings)
		SHADER_PARAMETER(uint32, NumSides)
		SHADER_PARAMETER(float, Radius)
		SHADER_PARAMETER(uint32, NumSrcNodes)
		SHADER_PARAMETER(uint32, Subdiv)
		SHADER_PARAMETER(float, SmoothParam)
		SHADER_PARAMETER(FMatrix44f, WorldToLocal)
		SHADER_PARAMETER_SRV(StructuredBuffer<float4>, InCenterline4)
		SHADER_PARAMETER_UAV(RWBuffer<float>, OutPositions)
		SHADER_PARAMETER_UAV(RWBuffer<uint>, OutTangents)
		SHADER_PARAMETER_UAV(RWBuffer<float>, OutTexCoords)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}
};

IMPLEMENT_GLOBAL_SHADER(FRopeBuildTubeResidentCS, "/Plugin/DynamicRope/Private/RopeBuildTube.usf", "RopeBuildTubeResidentCS", SF_Compute);

void RopeGPU::PrecacheTubeComputePSOs()
{
	FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
	if (!ShaderMap)
	{
		return;
	}

	// bForcePrecache: the PSO precache master switch (r.PSOPrecaching) is off in editor and uncooked
	// -game runs, where PrecacheComputePipelineState would otherwise no-op — exactly the runs where the
	// first-dispatch driver compile is felt. The compiles themselves run async on the thread pool.
	for (const int32 Bucket : GRopeTubeRingBuckets)
	{
		FRopeBuildTubeCS::FPermutationDomain TubePerm;
		TubePerm.Set<FRopeBuildTubeCS::FRingBucket>(Bucket);
		const TShaderRef<FRopeBuildTubeCS> TubeShader = ShaderMap->GetShader<FRopeBuildTubeCS>(TubePerm);
		if (TubeShader.IsValid())
		{
			PipelineStateCache::PrecacheComputePipelineState(
				TubeShader.GetComputeShader(), TEXT("RopeBuildTubeCS"), /*bForcePrecache*/ true);
		}

		FRopeBuildTubeResidentCS::FPermutationDomain ResidentPerm;
		ResidentPerm.Set<FRopeBuildTubeResidentCS::FRingBucket>(Bucket);
		const TShaderRef<FRopeBuildTubeResidentCS> ResidentShader = ShaderMap->GetShader<FRopeBuildTubeResidentCS>(ResidentPerm);
		if (ResidentShader.IsValid())
		{
			PipelineStateCache::PrecacheComputePipelineState(
				ResidentShader.GetComputeShader(), TEXT("RopeBuildTubeResidentCS"), /*bForcePrecache*/ true);
		}
	}
}

//~ 'stat DynamicRopeGPU' — GPU tube build bandwidth
// RopeGPUSolver.cpp instruments the solver and collision uploads under the same "DynamicRopeGPU" group, but the tube
// centreline upload is outside that path: it is a render command per proxy, in FRopeSceneProxy::BuildTubeGPU, and so
// falls outside it. It is therefore measured separately here. The group declaration is owned by RopeGPUStatGroup.h,
// shared across the module behind an include guard, while the individual stats are static and therefore local to this
// translation unit.
// The upload occurs on non-resident frames alone: the proxy uploads the CPU centreline mirror, one float3 per ring,
// into the centreline buffer every frame for the compute pass to read. Resident frames read the solver's resident
// position buffer directly and upload nothing, and that saving is read by comparing against the resident counter,
// since the higher the resident proportion the closer this bandwidth is to zero.
DECLARE_MEMORY_STAT(TEXT("GPU Tube Upload/Frame (Centerline)"), STAT_RopeGPU_TubeUpload, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Tube Builds/Frame"), STAT_RopeGPU_TubeBuilds, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Tube Resident Builds/Frame"), STAT_RopeGPU_TubeResidentBuilds, STATGROUP_DynamicRopeGPU);

// The GPU timeline stat, meaning the tube build kernel's actual GPU time as seen in 'stat gpu', ProfileGPU or the
// Insights GPU track. Unlike the bandwidth counters above it does not appear on this group's HUD. The tube build
// dispatches onto the immediate RHI command list rather than through RDG, being a render command per proxy, so it
// uses RHI_BREADCRUMB_EVENT_STAT rather than RDG_EVENT_SCOPE_STAT; SCOPED_GPU_STAT is a no-op here. The version
// contract is described in the matching comment block in RopeGPUSolver.cpp.
DECLARE_GPU_STAT_NAMED(RopeGPUTube, TEXT("DynamicRope Tube"));

#if STATS
// The tube build is a render command per rope, meaning per proxy, so it has no single frame entry point as the
// solver's RunSteps does. A per-frame value is produced by a deferred flush: the first build of a frame in which the
// render thread frame number changed pushes the previous frame's accumulation into the stats and resets it. As with
// the solver, a frame with no GPU tube build at all keeps the last value, which is the HUD convention.
static uint64 GRopeTubeUploadBytes = 0;    // Centreline upload bytes this frame, summed over non-resident builds.
static uint32 GRopeTubeBuildCount = 0;     // GPU tube builds this frame.
static uint32 GRopeTubeResidentCount = 0;  // How many of those were resident, meaning they uploaded nothing.
static uint32 GRopeTubeStatsFrame = 0;     // The render thread frame number at the last flush.

static void RopeTube_AccumBuild(bool bResident, uint64 CenterlineBytes)
{
	const uint32 Frame = GFrameNumberRenderThread;
	if (Frame != GRopeTubeStatsFrame)
	{
		// A frame boundary: publish the previous frame's accumulation and reset.
		SET_MEMORY_STAT(STAT_RopeGPU_TubeUpload, GRopeTubeUploadBytes);
		SET_DWORD_STAT(STAT_RopeGPU_TubeBuilds, GRopeTubeBuildCount);
		SET_DWORD_STAT(STAT_RopeGPU_TubeResidentBuilds, GRopeTubeResidentCount);
		GRopeTubeUploadBytes = 0;
		GRopeTubeBuildCount = 0;
		GRopeTubeResidentCount = 0;
		GRopeTubeStatsFrame = Frame;
	}
	++GRopeTubeBuildCount;
	if (bResident) { ++GRopeTubeResidentCount; }
	else           { GRopeTubeUploadBytes += CenterlineBytes; }
}
#endif

void RopeGPU::BuildTube_RenderThread(
	FRHICommandList& RHICmdList,
	FRHIShaderResourceView* InCenterlineSRV,
	FRHIUnorderedAccessView* OutPositionsUAV,
	FRHIUnorderedAccessView* OutTangentsUAV,
	FRHIUnorderedAccessView* OutTexCoordsUAV,
	int32 NumRings, int32 NumSides, float Radius)
{
	check(IsInRenderingThread());
	const int32 Bucket = RopeGPU::TubeRingBucket(NumRings);
	if (!InCenterlineSRV || !OutPositionsUAV || !OutTangentsUAV || !OutTexCoordsUAV
		|| NumRings < 2 || Bucket == 0 || NumSides < 3)
	{
		// A bucket of zero means the ring count is beyond the limit, so the caller falls back to the CPU.
		return;
	}

	FRopeBuildTubeCS::FPermutationDomain Perm;
	Perm.Set<FRopeBuildTubeCS::FRingBucket>(Bucket);
	TShaderMapRef<FRopeBuildTubeCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), Perm);

	FRopeBuildTubeCS::FParameters Params;
	Params.NumRings     = (uint32)NumRings;
	Params.NumSides     = (uint32)NumSides;
	Params.Radius       = Radius;
	Params.InCenterline = InCenterlineSRV;
	Params.OutPositions = OutPositionsUAV;
	Params.OutTangents  = OutTangentsUAV;
	Params.OutTexCoords = OutTexCoordsUAV;

	// One rope is one thread group, whose thread count is the bucket. The UAV barrier is the caller's, meaning the proxy's, responsibility.
	RHI_BREADCRUMB_EVENT_STAT(RHICmdList, RopeGPUTube, "DynamicRope Tube");
#if STATS
	// Non-resident: the centreline the proxy uploaded into the centreline buffer this frame, one float3 per ring, is this upload bandwidth.
	RopeTube_AccumBuild(/*bResident*/false, (uint64)NumRings * 3 * sizeof(float));
#endif

	FComputeShaderUtils::Dispatch(RHICmdList, ComputeShader, Params, FIntVector(1, 1, 1));
}

void RopeGPU::BuildTubeFromResident_RenderThread(
	FRHICommandList& RHICmdList,
	FRHIShaderResourceView* InResidentPositionsSRV,
	FRHIUnorderedAccessView* OutPositionsUAV,
	FRHIUnorderedAccessView* OutTangentsUAV,
	FRHIUnorderedAccessView* OutTexCoordsUAV,
	int32 NumRings, int32 NumSides, float Radius,
	int32 NumSrcNodes, int32 Subdiv, float SmoothParam,
	const FMatrix44f& WorldToLocal)
{
	check(IsInRenderingThread());
	const int32 Bucket = RopeGPU::TubeRingBucket(NumRings);
	if (!InResidentPositionsSRV || !OutPositionsUAV || !OutTangentsUAV || !OutTexCoordsUAV
		|| NumRings < 2 || Bucket == 0 || NumSides < 3 || NumSrcNodes < 2)
	{
		// A bucket of zero means the ring count is beyond the limit, so the caller falls back to the CPU.
		return;
	}

	FRopeBuildTubeResidentCS::FPermutationDomain Perm;
	Perm.Set<FRopeBuildTubeResidentCS::FRingBucket>(Bucket);
	TShaderMapRef<FRopeBuildTubeResidentCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), Perm);

	FRopeBuildTubeResidentCS::FParameters Params;
	Params.NumRings      = (uint32)NumRings;
	Params.NumSides      = (uint32)NumSides;
	Params.Radius        = Radius;
	Params.NumSrcNodes   = (uint32)NumSrcNodes;
	Params.Subdiv        = (uint32)FMath::Max(1, Subdiv);
	Params.SmoothParam   = SmoothParam;
	Params.WorldToLocal  = WorldToLocal;
	Params.InCenterline4 = InResidentPositionsSRV;
	Params.OutPositions  = OutPositionsUAV;
	Params.OutTangents   = OutTangentsUAV;
	Params.OutTexCoords  = OutTexCoordsUAV;

	RHI_BREADCRUMB_EVENT_STAT(RHICmdList, RopeGPUTube, "DynamicRope Tube");
#if STATS
	// Resident: it reads the solver's resident position buffer directly, so there is no centreline upload from the CPU and the upload bandwidth is zero.
	RopeTube_AccumBuild(/*bResident*/true, 0);
#endif

	FComputeShaderUtils::Dispatch(RHICmdList, ComputeShader, Params, FIntVector(1, 1, 1));
}
