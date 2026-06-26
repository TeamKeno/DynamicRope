// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeTubeBuilder.h"
#include "DynamicRopeShadersLog.h"

#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RHICommandList.h"
#include "RenderGraphUtils.h" // FComputeShaderUtils
#include "DataDrivenShaderPlatformInfo.h"

// groupshared frame 저장 / numthreads 크기 == 지원 최대 ring 수(= 솔버 MaxNodes). NumRings는 이 한도 내여야 한다.
static constexpr int32 ROPE_TUBE_MAX_RINGS = 256;

class FRopeBuildTubeCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeBuildTubeCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeBuildTubeCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRings)
		SHADER_PARAMETER(uint32, NumSides)
		SHADER_PARAMETER(float, Radius)
		SHADER_PARAMETER_SRV(Buffer<float>, InCenterline)
		SHADER_PARAMETER_UAV(RWBuffer<float>, OutPositions)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		OutEnvironment.SetDefine(TEXT("ROPE_TUBE_MAX_RINGS"), ROPE_TUBE_MAX_RINGS);
	}
};

IMPLEMENT_GLOBAL_SHADER(FRopeBuildTubeCS, "/Plugin/DynamicRope/Private/RopeBuildTube.usf", "RopeBuildTubeCS", SF_Compute);

// B2-lite: resident PosBuf(StructuredBuffer<float4>, 월드)에서 직접 생성 + WorldToLocal 변환.
class FRopeBuildTubeResidentCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeBuildTubeResidentCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeBuildTubeResidentCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRings)
		SHADER_PARAMETER(uint32, NumSides)
		SHADER_PARAMETER(float, Radius)
		SHADER_PARAMETER(FMatrix44f, WorldToLocal)
		SHADER_PARAMETER_SRV(StructuredBuffer<float4>, InCenterline4)
		SHADER_PARAMETER_UAV(RWBuffer<float>, OutPositions)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		OutEnvironment.SetDefine(TEXT("ROPE_TUBE_MAX_RINGS"), ROPE_TUBE_MAX_RINGS);
	}
};

IMPLEMENT_GLOBAL_SHADER(FRopeBuildTubeResidentCS, "/Plugin/DynamicRope/Private/RopeBuildTube.usf", "RopeBuildTubeResidentCS", SF_Compute);

void RopeGPU::BuildTube_RenderThread(
	FRHICommandList& RHICmdList,
	FRHIShaderResourceView* InCenterlineSRV,
	FRHIUnorderedAccessView* OutPositionsUAV,
	int32 NumRings, int32 NumSides, float Radius)
{
	check(IsInRenderingThread());
	if (!InCenterlineSRV || !OutPositionsUAV || NumRings < 2 || NumRings > ROPE_TUBE_MAX_RINGS || NumSides < 3)
	{
		return;
	}

	TShaderMapRef<FRopeBuildTubeCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));

	FRopeBuildTubeCS::FParameters Params;
	Params.NumRings     = (uint32)NumRings;
	Params.NumSides     = (uint32)NumSides;
	Params.Radius       = Radius;
	Params.InCenterline = InCenterlineSRV;
	Params.OutPositions = OutPositionsUAV;

	// 로프 1개 = 스레드그룹 1개(numthreads=ROPE_TUBE_MAX_RINGS). UAV 배리어는 호출자(proxy)가 처리.
	FComputeShaderUtils::Dispatch(RHICmdList, ComputeShader, Params, FIntVector(1, 1, 1));
}

void RopeGPU::BuildTubeFromResident_RenderThread(
	FRHICommandList& RHICmdList,
	FRHIShaderResourceView* InResidentPositionsSRV,
	FRHIUnorderedAccessView* OutPositionsUAV,
	int32 NumRings, int32 NumSides, float Radius,
	const FMatrix44f& WorldToLocal)
{
	check(IsInRenderingThread());
	if (!InResidentPositionsSRV || !OutPositionsUAV || NumRings < 2 || NumRings > ROPE_TUBE_MAX_RINGS || NumSides < 3)
	{
		return;
	}

	TShaderMapRef<FRopeBuildTubeResidentCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));

	FRopeBuildTubeResidentCS::FParameters Params;
	Params.NumRings      = (uint32)NumRings;
	Params.NumSides      = (uint32)NumSides;
	Params.Radius        = Radius;
	Params.WorldToLocal  = WorldToLocal;
	Params.InCenterline4 = InResidentPositionsSRV;
	Params.OutPositions  = OutPositionsUAV;

	FComputeShaderUtils::Dispatch(RHICmdList, ComputeShader, Params, FIntVector(1, 1, 1));
}
