// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeGPUSolver.h"
#include "DynamicRopeShadersLog.h"

#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"
#include "RHICommandList.h"          // FRHICommandListExecutor, CreateShaderResourceView
#include "DataDrivenShaderPlatformInfo.h"
#include "Misc/ScopeLock.h"

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

// ---------------------------------------------------------------------------------------------------
// 상주 상태 정의
// ---------------------------------------------------------------------------------------------------

// 렌더 스레드 전용. 로프 1개의 영속 GPU 버퍼 + 리드백(단일 in-flight, consume 후 재무장).
struct FRopeResidentRope
{
	TRefCountPtr<FRDGPooledBuffer> PosBuf;
	TRefCountPtr<FRDGPooledBuffer> PrevBuf;
	TRefCountPtr<FRDGPooledBuffer> InvMassBuf;
	int32  NumNodes = 0;
	uint32 Generation = 0xFFFFFFFFu;     // 마지막으로 시드한 generation(다르면 재시드)
	FRHIGPUBufferReadback* PosReadback = nullptr;
	FRHIGPUBufferReadback* PrevReadback = nullptr;
	bool bReadbackArmed = false;          // 리드백 copy가 enqueue되어 결과 대기 중인가.
	FShaderResourceViewRHIRef PosSRV;     // M5b: PosBuf StructuredBuffer<float4> SRV(렌더용). 재시드 시 무효화.
};

// GT<->RT 공유 결과. RT가 채우고 GT GetLatest가 락 하에 읽는다.
struct FRopeResidentSharedResults
{
	FCriticalSection Lock;
	TMap<uint32, FRopeResidentLatest> Map;
};

// pimpl: 영속 버퍼 맵(RT 전용) + 공유 결과(GT<->RT). RDG/RHI 타입을 헤더에서 숨긴다.
struct FRopeGPUSolver::FImpl
{
	TMap<uint32, FRopeResidentRope>                          RtRopes;  // 렌더 스레드에서만 접근.
	TSharedRef<FRopeResidentSharedResults, ESPMode::ThreadSafe> Results
		= MakeShared<FRopeResidentSharedResults, ESPMode::ThreadSafe>();
};

FRopeGPUSolver::FRopeGPUSolver()
{
	Impl = MakeUnique<FImpl>();
}

FRopeGPUSolver::~FRopeGPUSolver()
{
	// 상주 버퍼/리드백을 만지기 전에 in-flight 렌더 커맨드를 모두 drain → 이후 GT에서 직접 정리 OK.
	FlushRenderingCommands();
	ReleaseAll_RenderThread();
}

void FRopeGPUSolver::ReleaseAll_RenderThread()
{
	for (TPair<uint32, FRopeResidentRope>& Pair : Impl->RtRopes)
	{
		delete Pair.Value.PosReadback;  Pair.Value.PosReadback = nullptr;
		delete Pair.Value.PrevReadback; Pair.Value.PrevReadback = nullptr;
	}
	Impl->RtRopes.Empty();
}

void FRopeGPUSolver::ReleaseRope(uint32 RopeId)
{
	// 공유 결과는 GT에서 즉시 제거.
	{
		FScopeLock SL(&Impl->Results->Lock);
		Impl->Results->Map.Remove(RopeId);
	}
	// 영속 버퍼/리드백은 렌더 스레드에서 해제(this 캡처 — destructor가 flush하므로 수명 안전).
	ENQUEUE_RENDER_COMMAND(RopeGPUReleaseRope)(
		[this, RopeId](FRHICommandListImmediate&)
		{
			if (FRopeResidentRope* R = Impl->RtRopes.Find(RopeId))
			{
				delete R->PosReadback;
				delete R->PrevReadback;
				Impl->RtRopes.Remove(RopeId);
			}
		});
}

void FRopeGPUSolver::GetLatest(TMap<uint32, FRopeResidentLatest>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	Out = Impl->Results->Map; // 작은 데이터 — 매 프레임 복사. (스왑 대신 복사로 호출자가 누적분 유지)
}

FRHIShaderResourceView* FRopeGPUSolver::GetResidentPositionSRV_RenderThread(uint32 RopeId, int32& OutNumNodes)
{
	check(IsInRenderingThread());
	OutNumNodes = 0;

	FRopeResidentRope* R = Impl->RtRopes.Find(RopeId);
	if (!R || !R->PosBuf.IsValid())
	{
		return nullptr;
	}
	OutNumNodes = R->NumNodes;

	if (!R->PosSRV.IsValid())
	{
		// PosBuf는 StructuredBuffer<float4>(stride 16) — structured SRV로 본다.
		FRHICommandListImmediate& RHICmdList = FRHICommandListExecutor::GetImmediateCommandList();
		R->PosSRV = RHICmdList.CreateShaderResourceView(R->PosBuf->GetRHI(),
			FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Structured));
	}
	return R->PosSRV.GetReference();
}

void FRopeGPUSolver::Step(TArray<FRopeGPUResidentStep>&& Steps)
{
	if (Steps.Num() == 0)
	{
		return;
	}

	ENQUEUE_RENDER_COMMAND(RopeResidentStep)(
		[this, Steps = MoveTemp(Steps)](FRHICommandListImmediate& RHICmdList)
		{
			// --- Loop 1: 직전 프레임 리드백 consume(immediate Lock — RDG 빌더 구성 *전*에 처리해 immediate RHI와
			// 열린 그래프의 인터리브를 피한다. 렌더 스레드라 Lock 합법, IsReady 게이트라 stall 없음).
			for (const FRopeGPUResidentStep& S : Steps)
			{
				FRopeResidentRope* Rp = Impl->RtRopes.Find(S.RopeId);
				if (!Rp)
				{
					continue;
				}
				FRopeResidentRope& R = *Rp;
				const bool bWillReseed = !R.PosBuf.IsValid() || R.NumNodes != S.NumNodes || R.Generation != S.Generation;
				if (bWillReseed || !R.bReadbackArmed || !R.PosReadback || !R.PrevReadback)
				{
					continue;
				}
				if (!R.PosReadback->IsReady() || !R.PrevReadback->IsReady())
				{
					continue; // 아직 복사 중 — 다음 프레임에 consume(레이턴시 1프레임 더 허용).
				}

				const int32 N = R.NumNodes;
				const uint32 Bytes = (uint32)N * sizeof(FVector4f);
				TArray<FVector> TmpPos, TmpPrev;
				TmpPos.SetNumUninitialized(N);
				TmpPrev.SetNumUninitialized(N);
				if (const FVector4f* Src = (const FVector4f*)R.PosReadback->Lock(Bytes))
				{
					for (int32 k = 0; k < N; ++k) { TmpPos[k] = FVector(Src[k].X, Src[k].Y, Src[k].Z); }
					R.PosReadback->Unlock();
				}
				if (const FVector4f* Src = (const FVector4f*)R.PrevReadback->Lock(Bytes))
				{
					for (int32 k = 0; k < N; ++k) { TmpPrev[k] = FVector(Src[k].X, Src[k].Y, Src[k].Z); }
					R.PrevReadback->Unlock();
				}
				R.bReadbackArmed = false; // 소비 완료 — 아래 dispatch 블록에서 재무장.

				// 락 구간은 맵 대입만(리드백 Lock은 위에서 끝냄) → GT GetLatest 블로킹 최소화.
				FScopeLock SL(&Impl->Results->Lock);
				FRopeResidentLatest& L = Impl->Results->Map.FindOrAdd(S.RopeId);
				L.Positions     = MoveTemp(TmpPos);
				L.PrevPositions = MoveTemp(TmpPrev);
				L.NumNodes      = N;
				L.Generation    = R.Generation;
			}

			// 이제부터 그래프 빌드(seed/register/dispatch/재무장). consume(immediate Lock)은 위에서 끝냈다.
			FRDGBuilder GraphBuilder(RHICmdList);

			// 업로드 버퍼는 Execute()까지 살아 있어야 한다(RDG가 실행 시 복사) → keep-alive 컨테이너에 보관.
			// Reserve로 외부 배열 재할당을 막아 내부 데이터 포인터를 안정화(CreateStructuredBuffer에 넘긴 GetData 유효).
			const int32 NumSteps = Steps.Num();
			TArray<TArray<FVector4f>>           KPos;     KPos.Reserve(NumSteps);
			TArray<TArray<FVector4f>>           KPrev;    KPrev.Reserve(NumSteps);
			TArray<TArray<float>>               KInv;     KInv.Reserve(NumSteps);
			TArray<TArray<FRopeGPUParamsGPU>>   KParams;  KParams.Reserve(NumSteps);
			TArray<TArray<FRopeCapsuleGPU>>     KCaps;    KCaps.Reserve(NumSteps);
			TArray<TArray<float>>               KSDFDist; KSDFDist.Reserve(NumSteps);
			TArray<TArray<FRopeSDFVolumeGPU>>   KSDFVol;  KSDFVol.Reserve(NumSteps);
			TArray<TArray<FRopeSDFColliderGPU>> KSDFCol;  KSDFCol.Reserve(NumSteps);

			// --- Loop 2: seed/register + dispatch + 리드백 재무장(graph 패스).
			for (const FRopeGPUResidentStep& S : Steps)
			{
				const int32 N = S.NumNodes;
				if (N < 2 || N > ROPE_MAX_NODES)
				{
					UE_LOG(LogDynamicRopeGPU, Warning, TEXT("GPU resident step skipped: %d nodes out of [2, %d]."), N, ROPE_MAX_NODES);
					continue;
				}

				FRopeResidentRope& R = Impl->RtRopes.FindOrAdd(S.RopeId);
				const bool bSeed = !R.PosBuf.IsValid() || R.NumNodes != N || R.Generation != S.Generation;

				FRDGBufferRef PosRDG = nullptr;
				FRDGBufferRef PrevRDG = nullptr;
				FRDGBufferRef InvMassRDG = nullptr;

				if (bSeed)
				{
					const bool bHaveSeed = S.SeedPositions.Num() == N && S.SeedPrevPositions.Num() == N && S.InvMass.Num() == N;
					TArray<FVector4f>& SeedPos  = KPos.AddDefaulted_GetRef();
					TArray<FVector4f>& SeedPrev = KPrev.AddDefaulted_GetRef();
					TArray<float>&     SeedInv  = KInv.AddDefaulted_GetRef();
					SeedPos.SetNumUninitialized(N);
					SeedPrev.SetNumUninitialized(N);
					SeedInv.SetNumUninitialized(N);
					for (int32 k = 0; k < N; ++k)
					{
						const FVector P  = bHaveSeed ? S.SeedPositions[k]     : FVector::ZeroVector;
						const FVector Pp = bHaveSeed ? S.SeedPrevPositions[k] : FVector::ZeroVector;
						SeedPos[k]  = FVector4f((float)P.X,  (float)P.Y,  (float)P.Z,  0.0f);
						SeedPrev[k] = FVector4f((float)Pp.X, (float)Pp.Y, (float)Pp.Z, 0.0f);
						SeedInv[k]  = bHaveSeed ? S.InvMass[k] : 1.0f;
					}
					PosRDG     = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Pos"),     sizeof(FVector4f), N, SeedPos.GetData(),  (uint64)N * sizeof(FVector4f));
					PrevRDG    = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Prev"),    sizeof(FVector4f), N, SeedPrev.GetData(), (uint64)N * sizeof(FVector4f));
					InvMassRDG = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.InvMass"), sizeof(float),     N, SeedInv.GetData(),  (uint64)N * sizeof(float));
					R.PosBuf     = GraphBuilder.ConvertToExternalBuffer(PosRDG);
					R.PrevBuf    = GraphBuilder.ConvertToExternalBuffer(PrevRDG);
					R.InvMassBuf = GraphBuilder.ConvertToExternalBuffer(InvMassRDG);
					R.NumNodes   = N;
					R.Generation = S.Generation;
					R.bReadbackArmed = false; // 재시드 후 직전 리드백은 stale.
					R.PosSRV.SafeRelease();   // PosBuf 새로 생성 → 캐시된 SRV 무효(렌더가 다음에 재생성).
				}
				else
				{
					PosRDG     = GraphBuilder.RegisterExternalBuffer(R.PosBuf);
					PrevRDG    = GraphBuilder.RegisterExternalBuffer(R.PrevBuf);
					InvMassRDG = GraphBuilder.RegisterExternalBuffer(R.InvMassBuf);
				}

				if (S.NumSub <= 0)
				{
					continue; // 이번 프레임 적분 없음 — 위치 불변, 리드백도 그대로 둠.
				}

				// --- per-rope 파라미터/충돌 버퍼(transient). NodeOffset/CapsuleOffset/SDFColliderOffset = 0(로프당 버퍼).
				TArray<FRopeCapsuleGPU>&     CapsFlat = KCaps.AddDefaulted_GetRef();
				for (const FRopeGPUCapsule& Cap : S.Capsules)
				{
					FRopeCapsuleGPU G;
					G.A = FVector4f((float)Cap.A.X, (float)Cap.A.Y, (float)Cap.A.Z, 0.0f);
					G.B = FVector4f((float)Cap.B.X, (float)Cap.B.Y, (float)Cap.B.Z, Cap.Radius);
					CapsFlat.Add(G);
				}

				TArray<float>&               SDFDist = KSDFDist.AddDefaulted_GetRef();
				TArray<FRopeSDFVolumeGPU>&   SDFVol  = KSDFVol.AddDefaulted_GetRef();
				TArray<FRopeSDFColliderGPU>& SDFCol  = KSDFCol.AddDefaulted_GetRef();
				TMap<const void*, int32>     VolumeKeyToIndex; // 이 로프 내 볼륨 dedup.
				for (const FRopeGPUSDFCollider& Src : S.SDFColliders)
				{
					const int64 Voxels = (int64)Src.ResX * Src.ResY * Src.ResZ;
					if (!Src.Distances || Src.ResX < 2 || Src.ResY < 2 || Src.ResZ < 2 || Voxels <= 0)
					{
						continue;
					}
					int32 VolIdx;
					if (const int32* Found = VolumeKeyToIndex.Find(Src.VolumeKey))
					{
						VolIdx = *Found;
					}
					else
					{
						VolIdx = SDFVol.Num();
						FRopeSDFVolumeGPU V;
						V.DistOffset = SDFDist.Num();
						V.ResX = Src.ResX;
						V.ResY = Src.ResY;
						V.ResZ = Src.ResZ;
						V.LocalMin  = FVector4f((float)Src.LocalMin.X,  (float)Src.LocalMin.Y,  (float)Src.LocalMin.Z,  0.0f);
						V.LocalSize = FVector4f((float)Src.LocalSize.X, (float)Src.LocalSize.Y, (float)Src.LocalSize.Z, 0.0f);
						SDFVol.Add(V);
						SDFDist.Append(Src.Distances, (int32)Voxels);
						VolumeKeyToIndex.Add(Src.VolumeKey, VolIdx);
					}
					const FQuat   Q  = Src.BoneToWorld.GetRotation();
					const FVector T  = Src.BoneToWorld.GetTranslation();
					const FVector Sc = Src.BoneToWorld.GetScale3D();
					FRopeSDFColliderGPU C;
					C.VolumeIndex = VolIdx;
					C.Rotation    = FVector4f((float)Q.X, (float)Q.Y, (float)Q.Z, (float)Q.W);
					C.Translation = FVector4f((float)T.X, (float)T.Y, (float)T.Z, 0.0f);
					C.Scale       = FVector4f((float)Sc.X, (float)Sc.Y, (float)Sc.Z, 0.0f);
					SDFCol.Add(C);
				}

				// 유효 개수(SDF 루프는 무효 볼륨을 skip하므로 S.SDFColliders.Num()와 다를 수 있음) — 더미 패딩 *전* 확정.
				const int32 NumValidCaps   = CapsFlat.Num();
				const int32 NumValidSDFCol = SDFCol.Num();

				// 구조화 버퍼는 원소 >=1 이어야 한다 — 비면 더미 1개(어느 노드도 참조 안 함; count는 위에서 0으로 고정).
				if (CapsFlat.Num() == 0) { CapsFlat.AddZeroed(1); }
				if (SDFDist.Num() == 0)  { SDFDist.AddZeroed(1); }
				if (SDFVol.Num() == 0)   { SDFVol.AddZeroed(1); }
				if (SDFCol.Num() == 0)   { SDFCol.AddZeroed(1); }

				TArray<FRopeGPUParamsGPU>& ParamsArr = KParams.AddDefaulted_GetRef();
				FRopeGPUParamsGPU P;
				P.NodeOffset        = 0;
				P.NumNodes          = N;
				P.NumSub            = S.NumSub;
				P.Iters             = FMath::Max(1, S.Iterations);
				P.FixedDt           = S.FixedDt;
				P.SegmentLength     = S.SegmentLength;
				P.StretchCompliance = S.StretchCompliance;
				P.BendCompliance    = S.BendCompliance;
				P.Damping           = S.Damping;
				P.bStartPinned      = S.bStartPinned ? 1 : 0;
				P.CapsuleOffset     = 0;
				P.NumCapsules       = NumValidCaps;
				P.CollisionRadius   = S.CollisionRadius;
				P.Friction          = S.Friction;
				P.SweepStep         = S.SweepStep;
				P.MaxSweepSamples   = FMath::Max(1, S.MaxSweepSamples);
				P.SDFColliderOffset = 0;
				P.NumSDFColliders   = NumValidSDFCol;
				P.Gravity           = FVector4f((float)S.Gravity.X, (float)S.Gravity.Y, (float)S.Gravity.Z, 0.0f);
				P.PinPrev           = FVector4f((float)S.StartPinPrev.X,   (float)S.StartPinPrev.Y,   (float)S.StartPinPrev.Z,   0.0f);
				P.PinTarget         = FVector4f((float)S.StartPinTarget.X, (float)S.StartPinTarget.Y, (float)S.StartPinTarget.Z, 0.0f);
				ParamsArr.Add(P);

				const int32 NumCapsulesTotal = CapsFlat.Num();
				const int32 NumSDFDistances  = SDFDist.Num();
				const int32 NumSDFVolumes    = SDFVol.Num();
				const int32 NumSDFColliders  = SDFCol.Num();

				FRDGBufferRef ParamsBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Params"),
					sizeof(FRopeGPUParamsGPU), 1, ParamsArr.GetData(), sizeof(FRopeGPUParamsGPU));
				FRDGBufferRef CapsulesBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Capsules"),
					sizeof(FRopeCapsuleGPU), NumCapsulesTotal, CapsFlat.GetData(), (uint64)NumCapsulesTotal * sizeof(FRopeCapsuleGPU));
				FRDGBufferRef SDFDistBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFDistances"),
					sizeof(float), NumSDFDistances, SDFDist.GetData(), (uint64)NumSDFDistances * sizeof(float));
				FRDGBufferRef SDFVolBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFVolumes"),
					sizeof(FRopeSDFVolumeGPU), NumSDFVolumes, SDFVol.GetData(), (uint64)NumSDFVolumes * sizeof(FRopeSDFVolumeGPU));
				FRDGBufferRef SDFColBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFColliders"),
					sizeof(FRopeSDFColliderGPU), NumSDFColliders, SDFCol.GetData(), (uint64)NumSDFColliders * sizeof(FRopeSDFColliderGPU));

				FRopeXPBDSolveCS::FParameters* PassParams = GraphBuilder.AllocParameters<FRopeXPBDSolveCS::FParameters>();
				PassParams->NumRopes      = 1;
				PassParams->Params        = GraphBuilder.CreateSRV(ParamsBuf);
				PassParams->Capsules      = GraphBuilder.CreateSRV(CapsulesBuf);
				PassParams->SDFDistances  = GraphBuilder.CreateSRV(SDFDistBuf);
				PassParams->SDFVolumes    = GraphBuilder.CreateSRV(SDFVolBuf);
				PassParams->SDFColliders  = GraphBuilder.CreateSRV(SDFColBuf);
				PassParams->InvMass       = GraphBuilder.CreateSRV(InvMassRDG);
				PassParams->Positions     = GraphBuilder.CreateUAV(PosRDG);
				PassParams->PrevPositions = GraphBuilder.CreateUAV(PrevRDG);

				TShaderMapRef<FRopeXPBDSolveCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
				FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeXPBDResident"),
					ComputeShader, PassParams, FIntVector(1, 1, 1)); // 로프 1개 = 스레드그룹 1개

				// --- 리드백 재무장: in-flight가 없을 때만(이번 프레임 stepped 위치를 비동기 copy). consume은 Loop1에서.
				if (!R.bReadbackArmed)
				{
					if (!R.PosReadback)  { R.PosReadback  = new FRHIGPUBufferReadback(TEXT("Rope.PosReadback")); }
					if (!R.PrevReadback) { R.PrevReadback = new FRHIGPUBufferReadback(TEXT("Rope.PrevReadback")); }
					const uint32 NodeBytes = (uint32)N * sizeof(FVector4f);
					AddEnqueueCopyPass(GraphBuilder, R.PosReadback,  PosRDG,  NodeBytes);
					AddEnqueueCopyPass(GraphBuilder, R.PrevReadback, PrevRDG, NodeBytes);
					R.bReadbackArmed = true;
				}
			}

			GraphBuilder.Execute();
		});
}
