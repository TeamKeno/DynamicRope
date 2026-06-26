// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeGPUSolver.h"
#include "DynamicRopeShadersLog.h"

#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"
#include "DataDrivenShaderPlatformInfo.h"

// 스레드그룹 크기 == 지원하는 최대 노드 수. groupshared 정적 사이징과 numthreads에 함께 쓰인다.
static constexpr int32 ROPE_MAX_NODES = 256;

// HLSL FRopeGPUParams(RopeXPBD.usf)와 1:1 미러. 레이아웃 변경 시 .usf 동시 수정. 16바이트 정렬.
struct FRopeGPUParamsGPU
{
	int32     NodeOffset;
	int32     NumNodes;
	int32     NumSub;
	int32     Iters;
	float     FixedDt;
	float     SegmentLength;
	float     StretchCompliance;
	float     BendCompliance;
	float     Damping;
	int32     bStartPinned;
	float     Pad0 = 0.0f;
	float     Pad1 = 0.0f;
	FVector4f Gravity;
	FVector4f PinPrev;
	FVector4f PinTarget;
};
static_assert(sizeof(FRopeGPUParamsGPU) % 16 == 0, "FRopeGPUParamsGPU must be 16-byte aligned to match HLSL structured buffer.");

class FRopeXPBDSolveCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeXPBDSolveCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeXPBDSolveCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRopes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeGPUParams>, Params)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, InvMass)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, Positions)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, PrevPositions)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		OutEnvironment.SetDefine(TEXT("ROPE_MAX_NODES"), ROPE_MAX_NODES);
		OutEnvironment.SetDefine(TEXT("ROPE_THREADS"), ROPE_MAX_NODES);
	}
};

// 파일은 Shaders/Private/RopeXPBD.usf, 가상경로는 /Plugin/DynamicRope -> Shaders 이므로 /Private/ 포함.
IMPLEMENT_GLOBAL_SHADER(FRopeXPBDSolveCS, "/Plugin/DynamicRope/Private/RopeXPBD.usf", "RopeXPBDSolveCS", SF_Compute);

void FRopeGPUSolver::SolveBatch(TArrayView<const FRopeGPUJob> Jobs)
{
	// --- (1) GT에서 평탄화: 전 로프 노드를 하나의 글로벌 배열로, per-rope 파라미터를 별도 배열로.
	struct FFlatRope { FVector* Pos; FVector* Prev; int32 Offset; int32 NumNodes; };
	TArray<FVector4f>          Positions;
	TArray<FVector4f>          PrevPositions;
	TArray<float>              InvMass;
	TArray<FRopeGPUParamsGPU>  Params;
	TArray<FFlatRope>          Flat;

	for (const FRopeGPUJob& Job : Jobs)
	{
		if (!Job.Positions || !Job.PrevPositions || !Job.InvMass || Job.NumSub <= 0 || Job.NumNodes < 2)
		{
			continue;
		}
		const int32 N = Job.NumNodes;
		if (N > ROPE_MAX_NODES)
		{
			UE_LOG(LogDynamicRopeGPU, Warning, TEXT("GPU solve skipped a rope: %d nodes > ROPE_MAX_NODES(%d)."), N, ROPE_MAX_NODES);
			continue;
		}

		const int32 Offset = Positions.Num();
		for (int32 k = 0; k < N; ++k)
		{
			Positions.Add(FVector4f((float)Job.Positions[k].X, (float)Job.Positions[k].Y, (float)Job.Positions[k].Z, 0.0f));
			PrevPositions.Add(FVector4f((float)Job.PrevPositions[k].X, (float)Job.PrevPositions[k].Y, (float)Job.PrevPositions[k].Z, 0.0f));
			InvMass.Add(Job.InvMass[k]);
		}

		FRopeGPUParamsGPU P;
		P.NodeOffset        = Offset;
		P.NumNodes          = N;
		P.NumSub            = Job.NumSub;
		P.Iters             = FMath::Max(1, Job.Iterations);
		P.FixedDt           = Job.FixedDt;
		P.SegmentLength     = Job.SegmentLength;
		P.StretchCompliance = Job.StretchCompliance;
		P.BendCompliance    = Job.BendCompliance;
		P.Damping           = Job.Damping;
		P.bStartPinned      = Job.bStartPinned ? 1 : 0;
		P.Gravity           = FVector4f((float)Job.Gravity.X, (float)Job.Gravity.Y, (float)Job.Gravity.Z, 0.0f);
		P.PinPrev           = FVector4f((float)Job.StartPinPrev.X, (float)Job.StartPinPrev.Y, (float)Job.StartPinPrev.Z, 0.0f);
		P.PinTarget         = FVector4f((float)Job.StartPinTarget.X, (float)Job.StartPinTarget.Y, (float)Job.StartPinTarget.Z, 0.0f);
		Params.Add(P);

		Flat.Add({ Job.Positions, Job.PrevPositions, Offset, N });
	}

	if (Params.Num() == 0)
	{
		return;
	}

	const int32 TotalNodes = Positions.Num();
	const int32 NumRopes = Params.Num();

	// 리드백 결과를 받을 GT 스택 버퍼. FlushRenderingCommands가 커맨드 완료를 보장하므로 포인터 캡처가 안전하다.
	TArray<FVector4f> OutPos;
	TArray<FVector4f> OutPrev;
	OutPos.SetNumUninitialized(TotalNodes);
	OutPrev.SetNumUninitialized(TotalNodes);
	FVector4f* OutPosPtr  = OutPos.GetData();
	FVector4f* OutPrevPtr = OutPrev.GetData();

	ENQUEUE_RENDER_COMMAND(RopeGPUSolveBatch)(
		[Params = MoveTemp(Params), Positions = MoveTemp(Positions), PrevPositions = MoveTemp(PrevPositions),
		 InvMass = MoveTemp(InvMass), TotalNodes, NumRopes, OutPosPtr, OutPrevPtr](FRHICommandListImmediate& RHICmdList)
		{
			FRDGBuilder GraphBuilder(RHICmdList);

			FRDGBufferRef ParamsBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Params"),
				sizeof(FRopeGPUParamsGPU), NumRopes, Params.GetData(), (uint64)NumRopes * sizeof(FRopeGPUParamsGPU));
			FRDGBufferRef InvMassBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.InvMass"),
				sizeof(float), TotalNodes, InvMass.GetData(), (uint64)TotalNodes * sizeof(float));
			FRDGBufferRef PosBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Positions"),
				sizeof(FVector4f), TotalNodes, Positions.GetData(), (uint64)TotalNodes * sizeof(FVector4f));
			FRDGBufferRef PrevBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.PrevPositions"),
				sizeof(FVector4f), TotalNodes, PrevPositions.GetData(), (uint64)TotalNodes * sizeof(FVector4f));

			FRopeXPBDSolveCS::FParameters* PassParams = GraphBuilder.AllocParameters<FRopeXPBDSolveCS::FParameters>();
			PassParams->NumRopes      = (uint32)NumRopes;
			PassParams->Params        = GraphBuilder.CreateSRV(ParamsBuf);
			PassParams->InvMass       = GraphBuilder.CreateSRV(InvMassBuf);
			PassParams->Positions     = GraphBuilder.CreateUAV(PosBuf);
			PassParams->PrevPositions = GraphBuilder.CreateUAV(PrevBuf);

			TShaderMapRef<FRopeXPBDSolveCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
			// 로프 1개당 스레드그룹 1개.
			FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeXPBDSolve"),
				ComputeShader, PassParams, FIntVector(NumRopes, 1, 1));

			// --- 동기 리드백(M1): 결과 버퍼를 CPU로 복사하는 패스 추가 후, GPU idle 대기 + Lock.
			FRHIGPUBufferReadback PosReadback(TEXT("Rope.PosReadback"));
			FRHIGPUBufferReadback PrevReadback(TEXT("Rope.PrevReadback"));
			const uint32 NodeBytes = (uint32)TotalNodes * sizeof(FVector4f);
			AddEnqueueCopyPass(GraphBuilder, &PosReadback, PosBuf, NodeBytes);
			AddEnqueueCopyPass(GraphBuilder, &PrevReadback, PrevBuf, NodeBytes);

			GraphBuilder.Execute();
			RHICmdList.BlockUntilGPUIdle(); // 스파이크: 동기 완료 보장(스톨 — 후속에서 제거).

			{
				const void* Src = PosReadback.Lock(NodeBytes);
				FMemory::Memcpy(OutPosPtr, Src, NodeBytes);
				PosReadback.Unlock();
			}
			{
				const void* Src = PrevReadback.Lock(NodeBytes);
				FMemory::Memcpy(OutPrevPtr, Src, NodeBytes);
				PrevReadback.Unlock();
			}
		});

	// 동기: 위 커맨드(디스패치+리드백 Lock)가 끝날 때까지 GT 블록 → OutPos/OutPrev 채워짐 보장.
	FlushRenderingCommands();

	// --- (2) 결과를 각 로프 버퍼에 써넣는다(InvMass/SegmentLength 등 나머지는 GPU가 안 건드림).
	for (const FFlatRope& R : Flat)
	{
		for (int32 k = 0; k < R.NumNodes; ++k)
		{
			const FVector4f& Pn = OutPos[R.Offset + k];
			const FVector4f& Pp = OutPrev[R.Offset + k];
			R.Pos[k]  = FVector(Pn.X, Pn.Y, Pn.Z);
			R.Prev[k] = FVector(Pp.X, Pp.Y, Pp.Z);
		}
	}
}
