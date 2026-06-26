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
	int32     CapsuleOffset;   // M2: 이 로프의 capsule 글로벌 시작 인덱스
	int32     NumCapsules;     // M2: capsule 수(0이면 충돌 없음)
	float     CollisionRadius; // M2: 노드 두께
	float     Friction;        // M2: 접선 감쇠
	float     SweepStep;       // M2: swept 샘플 간격
	int32     MaxSweepSamples; // M2: 세그먼트당 샘플 상한
	int32     SDFColliderOffset; // M3: 이 로프의 SDF collider 글로벌 시작 인덱스
	int32     NumSDFColliders;   // M3: SDF collider 수(0이면 SDF 충돌 없음)
	int32     Pad0 = 0;
	int32     Pad1 = 0;
	FVector4f Gravity;
	FVector4f PinPrev;
	FVector4f PinTarget;
};
static_assert(sizeof(FRopeGPUParamsGPU) % 16 == 0, "FRopeGPUParamsGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeCapsule와 1:1 미러. xyz=세그먼트 끝점, B.w=반지름.
struct FRopeCapsuleGPU
{
	FVector4f A;
	FVector4f B; // w = Radius
};
static_assert(sizeof(FRopeCapsuleGPU) % 16 == 0, "FRopeCapsuleGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeSDFVolume와 1:1 미러. 본 로컬 grid 헤더(distance는 SDFDistances 버퍼에 DistOffset부터).
struct FRopeSDFVolumeGPU
{
	int32     DistOffset;
	int32     ResX;
	int32     ResY;
	int32     ResZ;
	FVector4f LocalMin;  // xyz
	FVector4f LocalSize; // xyz
};
static_assert(sizeof(FRopeSDFVolumeGPU) % 16 == 0, "FRopeSDFVolumeGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeSDFCollider와 1:1 미러. 볼륨 인덱스 + 본→월드 트랜스폼(quat/trans/scale, 행렬 레이아웃 회피).
struct FRopeSDFColliderGPU
{
	int32     VolumeIndex;
	int32     Pad0 = 0;
	int32     Pad1 = 0;
	int32     Pad2 = 0;
	FVector4f Rotation;    // quat (x,y,z,w)
	FVector4f Translation; // xyz
	FVector4f Scale;       // xyz
};
static_assert(sizeof(FRopeSDFColliderGPU) % 16 == 0, "FRopeSDFColliderGPU must be 16-byte aligned to match HLSL structured buffer.");

class FRopeXPBDSolveCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeXPBDSolveCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeXPBDSolveCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRopes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeGPUParams>, Params)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeCapsule>, Capsules)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, SDFDistances)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFVolume>, SDFVolumes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFCollider>, SDFColliders)
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
	TArray<FVector4f>           Positions;
	TArray<FVector4f>           PrevPositions;
	TArray<float>               InvMass;
	TArray<FRopeCapsuleGPU>     CapsulesFlat;
	TArray<float>               SDFDistances;   // M3: 전 볼륨 distance grid 연결(볼륨 dedup).
	TArray<FRopeSDFVolumeGPU>   SDFVolumes;     // M3: 볼륨 헤더.
	TArray<FRopeSDFColliderGPU> SDFCollidersFlat; // M3: 로프별 SDF collider 인스턴스.
	TMap<const void*, int32>    VolumeKeyToIndex; // 볼륨 dedup(VolumeKey -> SDFVolumes 인덱스).
	TArray<FRopeGPUParamsGPU>   Params;
	TArray<FFlatRope>           Flat;

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

		// 충돌(M2): 이 로프의 capsule을 글로벌 배열에 평탄화하고 per-rope offset/count 기록.
		const int32 CapOffset = CapsulesFlat.Num();
		const int32 CapCount = (Job.Capsules != nullptr) ? FMath::Max(0, Job.NumCapsules) : 0;
		for (int32 c = 0; c < CapCount; ++c)
		{
			const FRopeGPUCapsule& Cap = Job.Capsules[c];
			FRopeCapsuleGPU G;
			G.A = FVector4f((float)Cap.A.X, (float)Cap.A.Y, (float)Cap.A.Z, 0.0f);
			G.B = FVector4f((float)Cap.B.X, (float)Cap.B.Y, (float)Cap.B.Z, Cap.Radius);
			CapsulesFlat.Add(G);
		}

		// 충돌(M3): 이 로프의 SDF collider를 평탄화. 같은 볼륨(VolumeKey)은 distance grid를 1회만 업로드(dedup).
		const int32 SDFOffset = SDFCollidersFlat.Num();
		const int32 SDFInCount = (Job.SDFColliders != nullptr) ? FMath::Max(0, Job.NumSDFColliders) : 0;
		for (int32 c = 0; c < SDFInCount; ++c)
		{
			const FRopeGPUSDFCollider& S = Job.SDFColliders[c];
			const int64 Voxels = (int64)S.ResX * S.ResY * S.ResZ;
			if (!S.Distances || S.ResX < 2 || S.ResY < 2 || S.ResZ < 2 || Voxels <= 0)
			{
				continue; // 무효 볼륨 스킵.
			}

			int32 VolIdx;
			if (const int32* Found = VolumeKeyToIndex.Find(S.VolumeKey))
			{
				VolIdx = *Found;
			}
			else
			{
				VolIdx = SDFVolumes.Num();
				FRopeSDFVolumeGPU V;
				V.DistOffset = SDFDistances.Num();
				V.ResX = S.ResX;
				V.ResY = S.ResY;
				V.ResZ = S.ResZ;
				V.LocalMin  = FVector4f((float)S.LocalMin.X, (float)S.LocalMin.Y, (float)S.LocalMin.Z, 0.0f);
				V.LocalSize = FVector4f((float)S.LocalSize.X, (float)S.LocalSize.Y, (float)S.LocalSize.Z, 0.0f);
				SDFVolumes.Add(V);
				SDFDistances.Append(S.Distances, (int32)Voxels);
				VolumeKeyToIndex.Add(S.VolumeKey, VolIdx);
			}

			const FQuat   Q = S.BoneToWorld.GetRotation();
			const FVector T = S.BoneToWorld.GetTranslation();
			const FVector Sc = S.BoneToWorld.GetScale3D();
			FRopeSDFColliderGPU C;
			C.VolumeIndex = VolIdx;
			C.Rotation    = FVector4f((float)Q.X, (float)Q.Y, (float)Q.Z, (float)Q.W);
			C.Translation = FVector4f((float)T.X, (float)T.Y, (float)T.Z, 0.0f);
			C.Scale       = FVector4f((float)Sc.X, (float)Sc.Y, (float)Sc.Z, 0.0f);
			SDFCollidersFlat.Add(C);
		}
		const int32 SDFCount = SDFCollidersFlat.Num() - SDFOffset;

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
		P.CapsuleOffset     = CapOffset;
		P.NumCapsules       = CapCount;
		P.CollisionRadius   = Job.CollisionRadius;
		P.Friction          = Job.Friction;
		P.SweepStep         = Job.SweepStep;
		P.MaxSweepSamples   = FMath::Max(1, Job.MaxSweepSamples);
		P.SDFColliderOffset = SDFOffset;
		P.NumSDFColliders   = SDFCount;
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

	// 구조화 버퍼는 원소 >=1 이어야 한다 — 비면 더미 1개(어느 로프도 참조 안 함).
	if (CapsulesFlat.Num() == 0)     { CapsulesFlat.AddZeroed(1); }
	if (SDFDistances.Num() == 0)     { SDFDistances.AddZeroed(1); }
	if (SDFVolumes.Num() == 0)       { SDFVolumes.AddZeroed(1); }
	if (SDFCollidersFlat.Num() == 0) { SDFCollidersFlat.AddZeroed(1); }

	const int32 TotalNodes = Positions.Num();
	const int32 NumRopes = Params.Num();
	const int32 NumCapsulesTotal = CapsulesFlat.Num();
	const int32 NumSDFDistances = SDFDistances.Num();
	const int32 NumSDFVolumes = SDFVolumes.Num();
	const int32 NumSDFColliders = SDFCollidersFlat.Num();

	// 리드백 결과를 받을 GT 스택 버퍼. FlushRenderingCommands가 커맨드 완료를 보장하므로 포인터 캡처가 안전하다.
	TArray<FVector4f> OutPos;
	TArray<FVector4f> OutPrev;
	OutPos.SetNumUninitialized(TotalNodes);
	OutPrev.SetNumUninitialized(TotalNodes);
	FVector4f* OutPosPtr  = OutPos.GetData();
	FVector4f* OutPrevPtr = OutPrev.GetData();

	ENQUEUE_RENDER_COMMAND(RopeGPUSolveBatch)(
		[Params = MoveTemp(Params), Positions = MoveTemp(Positions), PrevPositions = MoveTemp(PrevPositions),
		 InvMass = MoveTemp(InvMass), CapsulesFlat = MoveTemp(CapsulesFlat), SDFDistances = MoveTemp(SDFDistances),
		 SDFVolumes = MoveTemp(SDFVolumes), SDFCollidersFlat = MoveTemp(SDFCollidersFlat),
		 TotalNodes, NumRopes, NumCapsulesTotal, NumSDFDistances, NumSDFVolumes, NumSDFColliders,
		 OutPosPtr, OutPrevPtr](FRHICommandListImmediate& RHICmdList)
		{
			FRDGBuilder GraphBuilder(RHICmdList);

			FRDGBufferRef ParamsBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Params"),
				sizeof(FRopeGPUParamsGPU), NumRopes, Params.GetData(), (uint64)NumRopes * sizeof(FRopeGPUParamsGPU));
			FRDGBufferRef CapsulesBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Capsules"),
				sizeof(FRopeCapsuleGPU), NumCapsulesTotal, CapsulesFlat.GetData(), (uint64)NumCapsulesTotal * sizeof(FRopeCapsuleGPU));
			FRDGBufferRef SDFDistBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFDistances"),
				sizeof(float), NumSDFDistances, SDFDistances.GetData(), (uint64)NumSDFDistances * sizeof(float));
			FRDGBufferRef SDFVolBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFVolumes"),
				sizeof(FRopeSDFVolumeGPU), NumSDFVolumes, SDFVolumes.GetData(), (uint64)NumSDFVolumes * sizeof(FRopeSDFVolumeGPU));
			FRDGBufferRef SDFColBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFColliders"),
				sizeof(FRopeSDFColliderGPU), NumSDFColliders, SDFCollidersFlat.GetData(), (uint64)NumSDFColliders * sizeof(FRopeSDFColliderGPU));
			FRDGBufferRef InvMassBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.InvMass"),
				sizeof(float), TotalNodes, InvMass.GetData(), (uint64)TotalNodes * sizeof(float));
			FRDGBufferRef PosBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Positions"),
				sizeof(FVector4f), TotalNodes, Positions.GetData(), (uint64)TotalNodes * sizeof(FVector4f));
			FRDGBufferRef PrevBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.PrevPositions"),
				sizeof(FVector4f), TotalNodes, PrevPositions.GetData(), (uint64)TotalNodes * sizeof(FVector4f));

			FRopeXPBDSolveCS::FParameters* PassParams = GraphBuilder.AllocParameters<FRopeXPBDSolveCS::FParameters>();
			PassParams->NumRopes      = (uint32)NumRopes;
			PassParams->Params        = GraphBuilder.CreateSRV(ParamsBuf);
			PassParams->Capsules      = GraphBuilder.CreateSRV(CapsulesBuf);
			PassParams->SDFDistances  = GraphBuilder.CreateSRV(SDFDistBuf);
			PassParams->SDFVolumes    = GraphBuilder.CreateSRV(SDFVolBuf);
			PassParams->SDFColliders  = GraphBuilder.CreateSRV(SDFColBuf);
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
