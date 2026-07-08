// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeTubeBuilder.h"
#include "DynamicRopeShadersLog.h"

#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RHICommandList.h"
#include "RenderGraphUtils.h" // FComputeShaderUtils
#include "DataDrivenShaderPlatformInfo.h"

// 링 버킷(스레드그룹 크기 == groupshared frame 배열 크기). 로프 1개 = 스레드그룹 1개라, 예전엔 모든 로프가
// 고정 256 그룹을 잡아 링 수가 적은 로프는 스레드 대부분이 idle(배리어에는 참여)이었다. 이제 NumRings 이상인
// 가장 작은 버킷을 골라(퍼뮤테이션) 그 낭비를 없애고, 최상단 512로 GPU 튜브 링 상한을 끌어올린다
// (Subdiv=3 기준 노드 ~170까지; 그 이상만 CPU 폴백). numthreads/groupshared는 .usf에서 ROPE_TUBE_MAX_RINGS로
// 스케일 — 퍼뮤테이션이 그 define을 버킷 값으로 설정한다. groupshared 예산: 링당 5×float3=60B → 512링 = 30KB(<32KB).
static constexpr int32 GRopeTubeRingBuckets[] = { 64, 128, 256, 512 };
static constexpr int32 ROPE_TUBE_MAX_RINGS_CAP = 512; // 최상단 버킷 = GPU 튜브 링 상한(초과 시 CPU 폴백).

int32 RopeGPU::TubeRingBucket(int32 NumRings)
{
	for (int32 Bucket : GRopeTubeRingBuckets)
	{
		if (NumRings <= Bucket) { return Bucket; }
	}
	return 0; // 상한 초과 → 호출자가 CPU 튜브로 폴백.
}

int32 RopeGPU::MaxTubeRings()
{
	return ROPE_TUBE_MAX_RINGS_CAP;
}

class FRopeBuildTubeCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeBuildTubeCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeBuildTubeCS, FGlobalShader);

	// 링 버킷 = numthreads/groupshared 크기. 값이 곧 ROPE_TUBE_MAX_RINGS define으로 .usf에 전달된다.
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

// B2-lite: resident PosBuf(StructuredBuffer<float4>, 월드)에서 직접 생성 + WorldToLocal 변환.
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
		return; // Bucket==0 = NumRings가 상한 초과 → 호출자가 CPU 폴백.
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

	// 로프 1개 = 스레드그룹 1개(numthreads=버킷). UAV 배리어는 호출자(proxy)가 처리.
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
		return; // Bucket==0 = NumRings가 상한 초과 → 호출자가 CPU 폴백.
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

	FComputeShaderUtils::Dispatch(RHICmdList, ComputeShader, Params, FIntVector(1, 1, 1));
}
