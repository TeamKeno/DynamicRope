// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeTubeBuilder.h"
#include "DynamicRopeShadersLog.h"

#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RHICommandList.h"
// FComputeShaderUtils
#include "RenderGraphUtils.h"
#include "DataDrivenShaderPlatformInfo.h"
// 'stat DynamicRope' 튜브 빌드 대역폭 계측(GFrameNumberRenderThread / SET_*_STAT). 그룹 선언은 모듈 공용 헤더.
#include "RenderingThread.h"
#include "Stats/Stats.h"
#include "RopeGPUStatGroup.h"

// 링 버킷(스레드그룹 크기 == groupshared frame 배열 크기). 로프 1개 = 스레드그룹 1개라, 예전엔 모든 로프가
// 고정 256 그룹을 잡아 링 수가 적은 로프는 스레드 대부분이 idle(배리어에는 참여)이었다. 이제 NumRings 이상인
// 가장 작은 버킷을 골라(퍼뮤테이션) 그 낭비를 없애고, 최상단 512로 GPU 튜브 링 상한을 끌어올린다
// (Subdiv=3 기준 노드 ~170까지; 그 이상만 CPU 폴백). numthreads/groupshared는 .usf에서 ROPE_TUBE_MAX_RINGS로
// 스케일 — 퍼뮤테이션이 그 define을 버킷 값으로 설정한다. groupshared 예산: 링당 5×float3=60B → 512링 = 30KB(<32KB).
static constexpr int32 GRopeTubeRingBuckets[] = { 64, 128, 256, 512 };
// 최상단 버킷 = GPU 튜브 링 상한(초과 시 CPU 폴백).
static constexpr int32 ROPE_TUBE_MAX_RINGS_CAP = 512;

int32 RopeGPU::TubeRingBucket(int32 NumRings)
{
	for (int32 Bucket : GRopeTubeRingBuckets)
	{
		if (NumRings <= Bucket) { return Bucket; }
	}
	// 상한 초과 → 호출자가 CPU 튜브로 폴백.
	return 0;
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

// ── 'stat DynamicRopeGPU' — GPU 튜브 빌드 대역폭 ───────────────────────────────────────────────────
// RopeGPUSolver.cpp가 같은 "DynamicRopeGPU" 그룹으로 솔버/충돌 업로드(GPU Upload/Frame *)를 계측하지만, 튜브
// 중심선 업로드는 그 RunSteps 경로 밖 — 프록시(FRopeSceneProxy::BuildTubeGPU)별 렌더 커맨드라 거기서 빠진다.
// 그래서 여기서 따로 잡는다. 그룹 선언은 RopeGPUStatGroup.h(모듈 공용, include guard)가 소유 — 개별 stat은
// static이라 이 TU 로컬이다.
// 비-resident 프레임에만 발생: 프록시가 CPU 중심선 미러(NumRings×float3)를 매 프레임 CenterlineBuffer로 올려
// 컴퓨트가 읽는다(BuildTube_RenderThread). resident 프레임은 솔버 상주 PosBuf를 직접 읽어 업로드 0 — 그
// 절약분을 Resident 카운터로 대비해 본다(resident 비율이 높을수록 이 대역폭은 0에 수렴).
DECLARE_MEMORY_STAT(TEXT("GPU Tube Upload/Frame (Centerline)"), STAT_RopeGPU_TubeUpload, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Tube Builds/Frame"), STAT_RopeGPU_TubeBuilds, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Tube Resident Builds/Frame"), STAT_RopeGPU_TubeResidentBuilds, STATGROUP_DynamicRopeGPU);

// GPU 타임라인 stat — 튜브 빌드 커널의 **실제 GPU 시간**('stat gpu' / ProfileGPU / Insights GPU 트랙). 위
// 대역폭 카운터와 달리 이 그룹 HUD에는 안 나온다. 튜브 빌드는 RDG가 아니라 즉시 RHI 커맨드 리스트에 dispatch
// 하므로(프록시별 렌더 커맨드) RDG_EVENT_SCOPE_STAT이 아니라 RHI_BREADCRUMB_EVENT_STAT을 쓴다 — 5.7에서
// SCOPED_GPU_STAT은 no-op이다(자세한 버전 계약은 RopeGPUSolver.cpp의 같은 블록 주석 참조).
DECLARE_GPU_STAT_NAMED(RopeGPUTube, TEXT("DynamicRope Tube"));

#if STATS
// 튜브 빌드는 로프(프록시)별 렌더 커맨드라 솔버 RunSteps 같은 단일 프레임 진입점이 없다. RT 프레임 번호가
// 바뀌는 그 프레임 첫 빌드에서 직전 프레임 누산분을 stat에 밀어넣고 리셋하는 지연-플러시로 프레임당 값을
// 만든다(프레임에 GPU 튜브 빌드가 아예 없으면 마지막 값이 유지되는 것도 솔버와 동일 — HUD 관례).
static uint64 GRopeTubeUploadBytes = 0;    // 이번 프레임 중심선 업로드 바이트(비-resident 빌드 합)
static uint32 GRopeTubeBuildCount = 0;     // 이번 프레임 GPU 튜브 빌드 수
static uint32 GRopeTubeResidentCount = 0;  // 그중 resident(업로드 0) 빌드 수
static uint32 GRopeTubeStatsFrame = 0;     // 마지막 플러시 시점의 RT 프레임 번호

static void RopeTube_AccumBuild(bool bResident, uint64 CenterlineBytes)
{
	const uint32 Frame = GFrameNumberRenderThread;
	if (Frame != GRopeTubeStatsFrame)
	{
		// 프레임 경계 — 직전 프레임 누산분 publish 후 리셋.
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
		// Bucket==0 = NumRings가 상한 초과 → 호출자가 CPU 폴백.
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

	// 로프 1개 = 스레드그룹 1개(numthreads=버킷). UAV 배리어는 호출자(proxy)가 처리.
	RHI_BREADCRUMB_EVENT_STAT(RHICmdList, RopeGPUTube, "DynamicRope Tube");
#if STATS
	// 비-resident: 프록시가 이번 프레임 CenterlineBuffer에 올린 중심선(NumRings×float3)이 이 업로드 대역폭이다.
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
		// Bucket==0 = NumRings가 상한 초과 → 호출자가 CPU 폴백.
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
	// resident: 솔버 상주 PosBuf 직독 — CPU→GPU 중심선 업로드 없음(업로드 대역폭 0).
	RopeTube_AccumBuild(/*bResident*/true, 0);
#endif

	FComputeShaderUtils::Dispatch(RHICmdList, ComputeShader, Params, FIntVector(1, 1, 1));
}
