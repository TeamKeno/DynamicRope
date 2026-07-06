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
#include "RHIStaticStates.h"         // TStaticSamplerState (GDF 샘플러) — Phase 2c
#include "GlobalDistanceFieldParameters.h" // FGlobalDistanceFieldParameters2 / _Minimal — Phase 2c
#include "GlobalRenderResources.h"   // GBlackVolumeTexture / GBlackUintVolumeTexture — Phase 2c
#include "SceneView.h"               // FSceneView / FViewUniformShaderParameters (GDF 패스 View UB) — Phase 2c
#include "DataDrivenShaderPlatformInfo.h"
#include "Misc/ScopeLock.h"
#include "HAL/IConsoleManager.h"     // TAutoConsoleVariable (GDFDebug 진단 토글)

// 스레드그룹 크기 == 지원하는 최대 노드 수. groupshared 정적 사이징과 numthreads에 함께 쓰인다.
static constexpr int32 ROPE_MAX_NODES = 256;

// 진단(GDFDebug): GDF 월드 충돌 패스가 노드별로 실제 읽은 거리/그라디언트-이동 정렬을 매 프레임 로그로 덤프한다.
// 0=off(기본). 얇은 벽 관통 원인 판별용 — minDist가 NodeR보다 크면 '과대보고', trig 노드의 dot>0이면 '부호-뒤집힘 사출'.
static TAutoConsoleVariable<int32> CVarRopeGDFDebug(
	TEXT("r.DynamicRope.GDFDebug"), 0,
	TEXT("로프 GDF 월드 충돌 패스의 노드별 진단값(거리/그라디언트·이동 정렬/침투/트리거)을 로그로 덤프. 0=off, 1=on."),
	ECVF_RenderThreadSafe);

// 진입-면 되밀기: 얇은 벽에서 gradient가 이동 방향을 향하는(중앙면 넘은) 노드를 gradient 대신 '온 길'로
// 되밀어 관통 사출을 막는다. 1=on(기본), 0=off(기존 gradient 밀어내기). GDFDebug로 A/B 비교용.
static TAutoConsoleVariable<int32> CVarRopeGDFEntrySidePush(
	TEXT("r.DynamicRope.GDFEntrySidePush"), 1,
	TEXT("GDF 월드 충돌에서 진입-면 되밀기(부호-뒤집힘 사출 방지). 0=off(기존), 1=on(기본)."),
	ECVF_RenderThreadSafe);

// 프레임 시작→끝 스윕: 끝점이 밴드 밖이어도 이동 경로가 얇은 벽을 통째로 건너뛴(터널링) 경우를,
// 진입점 쪽부터 마치해 진입 면에서 잡아 세운다. 1=on(기본), 0=off(끝점 점 쿼리만). GDFDebug로 A/B 비교용.
static TAutoConsoleVariable<int32> CVarRopeGDFSweep(
	TEXT("r.DynamicRope.GDFSweep"), 1,
	TEXT("GDF 월드 충돌에서 프레임 시작→끝 스윕(터널링 방지). 0=off(끝점만), 1=on(기본)."),
	ECVF_RenderThreadSafe);

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
	float     TipFrictionScale = 1.0f; // 자유단 마찰 배율(고정점=1, 끝=이 값). Pad0 슬롯 재사용.
	int32     CollisionPasses = 1;     // substep당 충돌 해소 패스 수(Iters로 상한). Pad1 슬롯 재사용.
	int32     bHasOverrides = 0;       // G0: 이 로프에 노드별 override(타깃/질량 주입)가 있는가.
	int32     Pad2 = 0;
	int32     Pad3 = 0;
	int32     Pad4 = 0;
	FVector4f Gravity;
	FVector4f PinPrev;
	FVector4f PinTarget;
};
static_assert(sizeof(FRopeGPUParamsGPU) % 16 == 0, "FRopeGPUParamsGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeCapsule와 1:1 미러. xyz=세그먼트 끝점, B.w=반지름, PrevB.w=InvDeltaTime(0이면 정적).
struct FRopeCapsuleGPU
{
	FVector4f A;
	FVector4f B;     // w = Radius
	FVector4f PrevA; // 이전 프레임 끝점(표면 속도 드래그/substep 상대 운동). 정적이면 패킹이 A/B로 채운다.
	FVector4f PrevB; // w = InvDeltaTime
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
	FVector4f Rotation;        // quat (x,y,z,w) — 현재 프레임
	FVector4f Translation;     // xyz
	FVector4f Scale;           // xyz
	FVector4f PrevRotation;    // quat (x,y,z,w) — 이전 프레임(CCD 상대 운동/표면속도용)
	FVector4f PrevTranslation; // xyz, w = InvDeltaTime(1/프레임dt; 0이면 정적)
};
static_assert(sizeof(FRopeSDFColliderGPU) % 16 == 0, "FRopeSDFColliderGPU must be 16-byte aligned to match HLSL structured buffer.");

class FRopeXPBDSolveCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeXPBDSolveCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeXPBDSolveCS, FGlobalShader);

	// GDF 월드 충돌을 substep 제약으로 통합하는 permutation. on일 때만 GDF 헤더 include + View/GDF 바인딩.
	// off(기본, View 없는 Step 경로 겸용)는 GDF 미참조 → View 없이 기존대로 컴파일된다.
	class FGDFDim : SHADER_PERMUTATION_BOOL("ROPE_USE_GDF");
	using FPermutationDomain = TShaderPermutationDomain<FGDFDim>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRopes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeGPUParams>, Params)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeCapsule>, Capsules)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, SDFDistances)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFVolume>, SDFVolumes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFCollider>, SDFColliders)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OverrideFlags)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, OverridePositions)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, OverridePrevPositions)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, OverrideInvMass)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, InvMass) // G0: override가 질량 마스크를 영속시키므로 RW.
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, Positions)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, PrevPositions)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutLambdaDist) // 장력 리드백(마지막 substep 세그먼트 λ).
		// --- GDF 통합 경로(FGDFDim on일 때만 셰이더가 참조; off면 미사용 → 언바운드 허용). FRopeGDFCollisionCS 레시피 미러.
		SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)
		SHADER_PARAMETER_STRUCT_INCLUDE(FGlobalDistanceFieldParameters2, GDF)
		SHADER_PARAMETER(FVector3f, GDFPreViewTranslation)
		SHADER_PARAMETER(uint32, bWorldGDFValid)
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

// HLSL FRopeGPUContact(RopeXPBD.usf)와 1:1 미러. 노드당 2슬롯(actual/predictive). 16바이트 정렬.
// Penetration은 WorldPoint.W에 팩(정렬 유지).
struct FRopeGPUContactGPU
{
	int32     bHit;
	int32     ColliderType;
	int32     ColliderIndex;
	int32     Source;
	FVector4f WorldPoint; // xyz 접촉점, w Penetration
	FVector4f Normal;
	FVector4f SurfaceVel;
};
static_assert(sizeof(FRopeGPUContactGPU) % 16 == 0, "FRopeGPUContactGPU must be 16-byte aligned to match HLSL structured buffer.");

// 접촉 감지 컴퓨트(G3). 솔브 후 상주 위치를 스윕해 노드당 최심 접촉을 OutContacts에 기록한다.
// 솔브 셰이더의 헬퍼/충돌 버퍼를 공유(같은 .usf)하되, 별도 엔트리라 자체 파라미터만 바인딩한다.
class FRopeContactDetectCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeContactDetectCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeContactDetectCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(int32, DetectNumNodes)
		SHADER_PARAMETER(int32, DetectNumCapsules)
		SHADER_PARAMETER(int32, DetectNumSDF)
		SHADER_PARAMETER(float, DetectContactRadius)
		SHADER_PARAMETER(float, DetectSegmentLength)
		SHADER_PARAMETER(float, DetectPredictionFrames)
		SHADER_PARAMETER(int32, DetectHasGuidedNodes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeCapsule>, Capsules)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, SDFDistances)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFVolume>, SDFVolumes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFCollider>, SDFColliders)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, DetectPositions)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, DetectPrevPositions)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, DetectGuidedMask)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, DetectWhipCur)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, DetectWhipPrev)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, DetectWhipNext)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FRopeGPUContact>, OutContacts)
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

IMPLEMENT_GLOBAL_SHADER(FRopeContactDetectCS, "/Plugin/DynamicRope/Private/RopeXPBD.usf", "RopeContactDetectCS", SF_Compute);

// Phase 2c: 엔진 GDF로 정적 월드에서 밀어내는 별도 post-solve 패스. View UB가 필요(GDF .ush의 ResolvedView) →
// 메인 솔브 CS(View 없는 Step 경로 겸용)와 분리한다. 뷰 확장이 솔브 뒤·튜브 앞에 로프별 dispatch한다.
class FRopeGDFCollisionCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeGDFCollisionCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeGDFCollisionCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)     // ResolvedView(GDF .ush LWC 오버로드).
		SHADER_PARAMETER_STRUCT_INCLUDE(FGlobalDistanceFieldParameters2, GDF)
		SHADER_PARAMETER(FVector3f, GDFPreViewTranslation)
		SHADER_PARAMETER(uint32, NumNodes)
		SHADER_PARAMETER(float, CollisionRadius)
		SHADER_PARAMETER(float, Friction)
		SHADER_PARAMETER(float, TipFrictionScale)
		SHADER_PARAMETER(uint32, bWorldGDFValid)
		SHADER_PARAMETER(uint32, bEntrySidePush) // 1=진입-면 되밀기(부호-뒤집힘 사출 방지).
		SHADER_PARAMETER(uint32, bSweep)         // 1=프레임 시작→끝 스윕(터널링 방지).
		SHADER_PARAMETER(uint32, SweepBackSteps) // = NumSub(솔버 Prev를 프레임 시작으로 역산할 배수).
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, InvMass)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, Positions)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, PrevPositions)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, DebugOut) // 진단(GDFDebug): 노드별 [Dist,dot,Pen,flag].
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		OutEnvironment.SetDefine(TEXT("ROPE_GDF_THREADS"), ROPE_MAX_NODES);
	}
};

IMPLEMENT_GLOBAL_SHADER(FRopeGDFCollisionCS, "/Plugin/DynamicRope/Private/RopeGDFCollision.usf", "RopeGDFCollisionCS", SF_Compute);

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
	// Phase 2c: 별도 post-solve GDF 밀어내기 패스가 순회 시 참조(매 step에서 갱신).
	bool  bUseWorldGDF = false;
	float GDFCollisionRadius = 0.0f;
	float GDFFriction = 0.0f;
	float GDFTipFrictionScale = 1.0f;
	int32 GDFNumSub = 1; // 이번 프레임 substep 수(스윕이 프레임 시작을 역산하는 데 사용).
	FRHIGPUBufferReadback* PosReadback = nullptr;
	FRHIGPUBufferReadback* PrevReadback = nullptr;
	bool bReadbackArmed = false;          // 리드백 copy가 enqueue되어 결과 대기 중인가.

	// 장력(λ) 리드백: 솔브(NumSub>0) 프레임에만 무장(override-only 프레임의 0을 안 내보내 직전 값 유지).
	// LambdaFixedDt = 무장 당시 substep dt — consume 시 F = max(0,-λ)/h² 변환에 쓴다.
	FRHIGPUBufferReadback* LambdaReadback = nullptr;
	bool  bLambdaArmed = false;
	float LambdaFixedDt = 0.0f;
	FShaderResourceViewRHIRef PosSRV;     // M5b: PosBuf StructuredBuffer<float4> SRV(렌더용). 재시드 시 무효화.

	// SDF 볼륨 그리드/헤더 resident(정적 베이크 데이터 — 볼륨 집합이 바뀔 때만 재업로드). 인스턴스(본
	// 트랜스폼)는 매 프레임 작은 버퍼로 따로 올린다. 이로써 매 프레임 multi-MB 그리드 재업로드를 없앤다.
	TRefCountPtr<FRDGPooledBuffer> SDFDistBuf;
	TRefCountPtr<FRDGPooledBuffer> SDFVolBuf;
	uint32 SDFSetSig = 0;                       // 볼륨 집합 시그니처(키+복셀수). 다르면 재빌드.
	TMap<const void*, int32> SDFVolKeyToIndex;  // VolumeKey -> SDFVol 인덱스(매 프레임 인스턴스 VolumeIndex 산정).

	// 접촉 감지(G3): 노드당 1슬롯 출력 버퍼(resident, N 변할 때만 재생성) + 리드백(위치와 같은 ring).
	TRefCountPtr<FRDGPooledBuffer> ContactBuf;
	FRHIGPUBufferReadback* ContactReadback = nullptr;
	bool bContactArmed = false;

	// 진단(GDFDebug): GDF 월드 충돌 패스 뒤 노드별 [Dist,dot,Pen,flag] 리드백(cvar on일 때만 무장). 공유 결과 아님 — 로그 전용.
	FRHIGPUBufferReadback* GDFDebugReadback = nullptr;
	bool bGDFDebugArmed = false;
};

// GT<->RT 공유 결과. RT가 채우고 GT GetLatest가 락 하에 읽는다.
struct FRopeResidentSharedResults
{
	FCriticalSection Lock;
	TMap<uint32, FRopeResidentLatest> Map;
	TMap<uint32, FRopeResidentContacts> Contacts; // G3: 접촉 감지 결과(GetLatestContacts).
};

// pimpl: 영속 버퍼 맵(RT 전용) + 공유 결과(GT<->RT). RDG/RHI 타입을 헤더에서 숨긴다.
struct FRopeGPUSolver::FImpl
{
	TMap<uint32, FRopeResidentRope>                          RtRopes;  // 렌더 스레드에서만 접근.
	TSharedRef<FRopeResidentSharedResults, ESPMode::ThreadSafe> Results
		= MakeShared<FRopeResidentSharedResults, ESPMode::ThreadSafe>();

	// GDF 경로(EnqueueSteps)로 쌓인 이번 프레임 step들. 뷰 확장이 DispatchPending_RenderThread에서 소비. RT 전용.
	TArray<FRopeGPUResidentStep> PendingSteps;
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
		delete Pair.Value.PosReadback;      Pair.Value.PosReadback = nullptr;
		delete Pair.Value.PrevReadback;     Pair.Value.PrevReadback = nullptr;
		delete Pair.Value.LambdaReadback;   Pair.Value.LambdaReadback = nullptr;
		delete Pair.Value.ContactReadback;  Pair.Value.ContactReadback = nullptr;
		delete Pair.Value.GDFDebugReadback; Pair.Value.GDFDebugReadback = nullptr;
	}
	Impl->RtRopes.Empty();
}

void FRopeGPUSolver::ReleaseRope(uint32 RopeId)
{
	// 공유 결과는 GT에서 즉시 제거.
	{
		FScopeLock SL(&Impl->Results->Lock);
		Impl->Results->Map.Remove(RopeId);
		Impl->Results->Contacts.Remove(RopeId);
	}
	// 영속 버퍼/리드백은 렌더 스레드에서 해제(this 캡처 — destructor가 flush하므로 수명 안전).
	ENQUEUE_RENDER_COMMAND(RopeGPUReleaseRope)(
		[this, RopeId](FRHICommandListImmediate&)
		{
			if (FRopeResidentRope* R = Impl->RtRopes.Find(RopeId))
			{
				delete R->PosReadback;
				delete R->PrevReadback;
				delete R->LambdaReadback;
				delete R->ContactReadback;
				delete R->GDFDebugReadback;
				Impl->RtRopes.Remove(RopeId);
			}
		});
}

void FRopeGPUSolver::GetLatest(TMap<uint32, FRopeResidentLatest>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	Out = Impl->Results->Map; // 작은 데이터 — 매 프레임 복사. (스왑 대신 복사로 호출자가 누적분 유지)
}

void FRopeGPUSolver::GetLatestContacts(TMap<uint32, FRopeResidentContacts>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	Out = Impl->Results->Contacts; // 노드당 최대 1건이라 작다 — 매 프레임 복사.
}

bool FRopeGPUSolver::ReadbackNow(uint32 RopeId, TArray<FVector>& OutPositions, TArray<FVector>& OutPrevPositions, uint32& OutGeneration)
{
	// GT 블로킹(M5c): RT에서 즉석 copy pass + GPU idle 대기 + Lock까지 끝내고, GT는 Flush로 그 완료를
	// 기다린다. 이벤트당 1회(wrap 핸드오프) 전용 — 상주 리드백(GetLatest)과 달리 지연이 없다.
	bool bOk = false;
	uint32 Generation = 0;
	ENQUEUE_RENDER_COMMAND(RopeGPUReadbackNow)(
		[this, RopeId, &OutPositions, &OutPrevPositions, &Generation, &bOk](FRHICommandListImmediate& RHICmdList)
		{
			FRopeResidentRope* Rp = Impl->RtRopes.Find(RopeId);
			if (!Rp || !Rp->PosBuf.IsValid() || !Rp->PrevBuf.IsValid() || Rp->NumNodes < 2)
			{
				return;
			}
			const int32 N = Rp->NumNodes;
			const uint32 Bytes = (uint32)N * sizeof(FVector4f);

			// 상주(in-flight) 리드백과 독립인 일회용 리드백 — RDG로 등록해 상태 전이를 맡긴다.
			FRHIGPUBufferReadback PosRb(TEXT("Rope.PosReadbackNow"));
			FRHIGPUBufferReadback PrevRb(TEXT("Rope.PrevReadbackNow"));
			{
				FRDGBuilder GraphBuilder(RHICmdList);
				FRDGBufferRef PosRDG  = GraphBuilder.RegisterExternalBuffer(Rp->PosBuf);
				FRDGBufferRef PrevRDG = GraphBuilder.RegisterExternalBuffer(Rp->PrevBuf);
				AddEnqueueCopyPass(GraphBuilder, &PosRb,  PosRDG,  Bytes);
				AddEnqueueCopyPass(GraphBuilder, &PrevRb, PrevRDG, Bytes);
				GraphBuilder.Execute();
			}
			RHICmdList.BlockUntilGPUIdle();

			OutPositions.SetNumUninitialized(N);
			OutPrevPositions.SetNumUninitialized(N);
			bool bLocked = false;
			if (const FVector4f* Src = (const FVector4f*)PosRb.Lock(Bytes))
			{
				for (int32 k = 0; k < N; ++k) { OutPositions[k] = FVector(Src[k].X, Src[k].Y, Src[k].Z); }
				PosRb.Unlock();
				bLocked = true;
			}
			if (const FVector4f* Src = (const FVector4f*)PrevRb.Lock(Bytes))
			{
				for (int32 k = 0; k < N; ++k) { OutPrevPositions[k] = FVector(Src[k].X, Src[k].Y, Src[k].Z); }
				PrevRb.Unlock();
			}
			else
			{
				bLocked = false;
			}
			Generation = Rp->Generation;
			bOk = bLocked;
		});
	FlushRenderingCommands(); // RT 커맨드 완료까지 GT 대기(참조 캡처 안전 + 결과 확정).
	OutGeneration = Generation;
	return bOk;
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

FRDGBufferRef FRopeGPUSolver::RegisterResidentPos_RenderThread(FRDGBuilder& GraphBuilder, uint32 RopeId, int32& OutNumNodes)
{
	check(IsInRenderingThread());
	OutNumNodes = 0;

	FRopeResidentRope* R = Impl->RtRopes.Find(RopeId);
	if (!R || !R->PosBuf.IsValid())
	{
		return nullptr;
	}
	OutNumNodes = R->NumNodes;
	// 같은 그래프에서 DispatchPending의 solve 패스가 이 PosBuf를 UAV로 등록했으면 RDG가 dedup해 동일 노드를
	// 돌려주고 solve→tube 의존성을 자동으로 건다.
	return GraphBuilder.RegisterExternalBuffer(R->PosBuf);
}

// Phase 2c: GDF 셰이더 파라미터 구성. 엔진 SetupGlobalDistanceFieldParameters(전체)는 RENDERER_API가 아니라
// 링크 불가 → inline _Minimal을 쓰고 그것이 빠뜨리는 CoverageAtlas 텍스처 + 샘플러 3개를 직접 보강한다.
// GDF가 null/클립맵 0이면 검은 볼륨 텍스처를 바인딩하고 OutValid=0(셰이더가 GDF 블록을 건너뛴다).
static void FillGDFShaderParams(const FGlobalDistanceFieldParameterData* GDF, FGlobalDistanceFieldParameters2& Out, uint32& OutValid)
{
	if (GDF && GDF->NumGlobalSDFClipmaps > 0)
	{
		Out = SetupGlobalDistanceFieldParameters_Minimal(*GDF);
		Out.GlobalDistanceFieldCoverageAtlasTexture = GDF->CoverageAtlasTexture
			? GDF->CoverageAtlasTexture : GBlackVolumeTexture->TextureRHI.GetReference();
		OutValid = 1;
	}
	else
	{
		Out = FGlobalDistanceFieldParameters2{};
		Out.GlobalDistanceFieldPageAtlasTexture     = GBlackVolumeTexture->TextureRHI.GetReference();
		Out.GlobalDistanceFieldCoverageAtlasTexture = GBlackVolumeTexture->TextureRHI.GetReference();
		Out.GlobalDistanceFieldPageTableTexture     = GBlackUintVolumeTexture->TextureRHI.GetReference();
		Out.GlobalDistanceFieldMipTexture           = GBlackVolumeTexture->TextureRHI.GetReference();
		OutValid = 0;
	}
	// 샘플러 3개는 _Minimal이 채우지 않는다(미세팅 시 검은 샘플). 항상 세팅.
	Out.GlobalDistanceFieldPageAtlasTextureSampler     = TStaticSamplerState<SF_Trilinear, AM_Wrap,  AM_Wrap,  AM_Wrap >::GetRHI();
	Out.GlobalDistanceFieldCoverageAtlasTextureSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap,  AM_Wrap,  AM_Wrap >::GetRHI();
	Out.GlobalDistanceFieldMipTextureSampler           = TStaticSamplerState<SF_Trilinear, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI();
}

// 상주 step들의 공용 실행부(RT). 전용 그래프(Step)든 씬 렌더러 그래프(DispatchPending_RenderThread)든
// 동일 본체를 전달받은 GraphBuilder에 얹는다(Execute는 호출자). GDF/PreViewTranslation은 GDF 월드 충돌(Phase 2c)에서 사용.
void FRopeGPUSolver::RunSteps_RenderThread(FRDGBuilder& GraphBuilder, TArray<FRopeGPUResidentStep>& Steps,
	const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation)
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
				// 재시드 프레임에는 직전 리드백이 stale이라 무시(위치/접촉 모두). 무장 해제해 다음 dispatch가 재무장.
				if (bWillReseed)
				{
					R.bReadbackArmed = false;
					R.bLambdaArmed = false;
					R.bContactArmed = false;
					R.bGDFDebugArmed = false; // 재시드 프레임 — 직전 GDF 진단 리드백도 stale.
					continue;
				}

				const int32 N = R.NumNodes;

				// 위치 리드백 consume(무장·준비됐을 때만) — 실패해도 접촉 consume은 독립 진행.
				TArray<FVector> TmpPos, TmpPrev;
				bool bHavePos = false;
				if (R.bReadbackArmed && R.PosReadback && R.PrevReadback
					&& R.PosReadback->IsReady() && R.PrevReadback->IsReady())
				{
					const uint32 Bytes = (uint32)N * sizeof(FVector4f);
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
					bHavePos = true;
				}

				// 장력(λ) 리드백: 위치와 독립 consume(솔브 프레임에만 무장). 무장 당시 dt로 힘 변환.
				TArray<float> TmpTension;
				bool bHaveTension = false;
				if (R.bLambdaArmed && R.LambdaReadback && R.LambdaReadback->IsReady())
				{
					const uint32 LBytes = (uint32)N * sizeof(float);
					if (const float* Src = (const float*)R.LambdaReadback->Lock(LBytes))
					{
						// 세그먼트 수 = N-1(슬롯 N-1은 커널이 항상 0). F = max(0,-λ)/h² — CPU Step과 동일 변환.
						const float InvDt2 = (R.LambdaFixedDt > 1e-6f) ? (1.0f / (R.LambdaFixedDt * R.LambdaFixedDt)) : 0.0f;
						TmpTension.SetNumUninitialized(N - 1);
						for (int32 k = 0; k < N - 1; ++k)
						{
							TmpTension[k] = FMath::Max(0.0f, -Src[k]) * InvDt2;
						}
						R.LambdaReadback->Unlock();
						bHaveTension = true;
					}
					R.bLambdaArmed = false; // 소비 완료 — 아래 dispatch 블록에서 재무장.
				}

				// 접촉 감지 리드백(G3): 위치와 독립 consume(감지는 Flight만 무장하므로 없을 수 있다).
				TArray<FRopeGPUContactResult> TmpContacts;
				bool bHaveContacts = false;
				if (R.bContactArmed && R.ContactReadback && R.ContactReadback->IsReady())
				{
					// 노드당 2슬롯: [0..N) actual, [N..2N) predictive. 슬롯 인덱스 % N = 노드 인덱스.
					const uint32 CBytes = (uint32)(2 * N) * sizeof(FRopeGPUContactGPU);
					if (const FRopeGPUContactGPU* Src = (const FRopeGPUContactGPU*)R.ContactReadback->Lock(CBytes))
					{
						for (int32 slot = 0; slot < 2 * N; ++slot)
						{
							if (Src[slot].bHit == 0)
							{
								continue;
							}
							FRopeGPUContactResult C;
							C.NodeIndex       = slot % N;
							C.ColliderType    = Src[slot].ColliderType;
							C.ColliderIndex   = Src[slot].ColliderIndex;
							C.Source          = (uint8)Src[slot].Source;
							C.Penetration     = Src[slot].WorldPoint.W; // w에 팩된 침투.
							C.WorldPoint      = FVector(Src[slot].WorldPoint.X, Src[slot].WorldPoint.Y, Src[slot].WorldPoint.Z);
							C.Normal          = FVector(Src[slot].Normal.X, Src[slot].Normal.Y, Src[slot].Normal.Z);
							C.SurfaceVelocity = FVector(Src[slot].SurfaceVel.X, Src[slot].SurfaceVel.Y, Src[slot].SurfaceVel.Z);
							TmpContacts.Add(C);
						}
						R.ContactReadback->Unlock();
						bHaveContacts = true;
					}
					R.bContactArmed = false; // 소비 완료 — 아래 dispatch 블록에서 재무장.
				}

				// 진단(GDFDebug): GDF 월드 충돌 패스가 남긴 노드별 [Dist, dot(Nrm,이동), Pen, flag] 회수 → 로그 덤프.
				// 공유 결과에 넣지 않고 로그만(cvar r.DynamicRope.GDFDebug로 무장·소비 게이트). 위 continue보다 앞에 둔다.
				if (R.bGDFDebugArmed && R.GDFDebugReadback && R.GDFDebugReadback->IsReady())
				{
					const uint32 DBytes = (uint32)(2 * N) * sizeof(FVector4f);
					if (const FVector4f* Src = (const FVector4f*)R.GDFDebugReadback->Lock(DBytes))
					{
						float MinDist = TNumericLimits<float>::Max(), MaxSeg = 0.0f;
						int32 MinIdx = -1, TrigCount = 0, RevCount = 0, SwpCount = 0, AttemptCount = 0, DetailCount = 0;
						FString TrigDetail;
						for (int32 k = 0; k < N; ++k)
						{
							const FVector4f& P0 = Src[2 * k + 0]; // [Dist, dot, Pen, flag]
							const FVector4f& P1 = Src[2 * k + 1]; // [SegLen, minSd, Move1, attempt]
							const float D = P0.X;
							if (D < 0.0f) { continue; } // sentinel(핀/미빌드로 조기 반환한 노드).
							if (D < MinDist) { MinDist = D; MinIdx = k; }
							MaxSeg = FMath::Max(MaxSeg, P1.X);
							if (P1.W > 0.5f) { ++AttemptCount; } // 스윕이 개입한 노드 수.
							const float F = P0.W; // 0=미접촉, 1=일반 push, 2=진입-면 되밀기, 3=스윕 캐치.
							if (F < 0.5f) { continue; }
							const TCHAR* Tag = TEXT("");
							if (F > 2.5f)      { ++SwpCount;  Tag = TEXT(" SWEEP"); } // flag==3
							else               { ++TrigCount; if (F > 1.5f) { ++RevCount; Tag = TEXT(" REV"); } } // 1/2
							if (DetailCount < 8)
							{
								++DetailCount;
								TrigDetail += FString::Printf(TEXT(" [n=%d d=%.2f dot=%+.2f pen=%.2f seg=%.1f minSd=%.1f%s]"),
									k, P0.X, P0.Y, P0.Z, P1.X, P1.Y, Tag);
							}
						}
						R.GDFDebugReadback->Unlock();
						UE_LOG(LogDynamicRopeGPU, Display,
							TEXT("[GDFDebug] Rope=%u N=%d NodeR=%.2f minDist=%.2f@%d trig=%d rev=%d swp=%d attempt=%d maxSeg=%.1f%s"),
							S.RopeId, N, R.GDFCollisionRadius,
							(MinIdx >= 0 ? MinDist : -1.0f), MinIdx, TrigCount, RevCount, SwpCount, AttemptCount, MaxSeg, *TrigDetail);
					}
					R.bGDFDebugArmed = false; // 소비 완료 — DispatchGDFCollision에서 cvar on이면 재무장.
				}

				if (!bHavePos && !bHaveContacts && !bHaveTension)
				{
					continue; // 이번 프레임 회수분 없음.
				}

				// 락 구간은 맵 대입만(리드백 Lock은 위에서 끝냄) → GT GetLatest 블로킹 최소화.
				FScopeLock SL(&Impl->Results->Lock);
				if (bHavePos || bHaveTension)
				{
					FRopeResidentLatest& L = Impl->Results->Map.FindOrAdd(S.RopeId);
					if (bHavePos)
					{
						L.Positions     = MoveTemp(TmpPos);
						L.PrevPositions = MoveTemp(TmpPrev);
						L.NumNodes      = N;
						L.Generation    = R.Generation; // generation 승격은 위치와 함께만(재시드 직후 stale 위치 승격 방지).
					}
					if (bHaveTension)
					{
						// 장력은 entry generation을 건드리지 않는다 — 재시드 직후 위치보다 먼저 도착하면
						// GT가 (구 generation으로) 한 프레임 거부하고, 위치가 따라잡으면 함께 소비된다.
						L.SegmentTension = MoveTemp(TmpTension);
					}
				}
				if (bHaveContacts)
				{
					FRopeResidentContacts& CL = Impl->Results->Contacts.FindOrAdd(S.RopeId);
					CL.Contacts   = MoveTemp(TmpContacts);
					CL.Generation = R.Generation;
				}
			}

			// 이제부터 그래프 빌드(seed/register/dispatch/재무장). consume은 위에서 끝냈다. GraphBuilder는 인자
			// (전용 그래프=Step, 씬 렌더러 그래프=DispatchPending_RenderThread). GDF 월드 충돌은 별도 post-solve
			// 패스(DispatchGDFCollision_RenderThread)에서 처리하므로 여기선 GDF/PreViewTranslation를 쓰지 않는다.
			(void)GDF; (void)PreViewTranslation;
			// 이 프레임 stepped 로프에 GDF 플래그/충돌 파라미터를 상주 상태에 기록(별도 GDF 패스가 순회에 사용).

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
			TArray<TArray<uint32>>              KOvFlags; KOvFlags.Reserve(NumSteps);
			TArray<TArray<FVector4f>>           KOvPos;   KOvPos.Reserve(NumSteps);
			TArray<TArray<FVector4f>>           KOvPrev;  KOvPrev.Reserve(NumSteps);
			TArray<TArray<float>>               KOvInv;   KOvInv.Reserve(NumSteps);
			// 감지 예측(G3b) whip 버퍼 keep-alive(전용 — 다른 K*와 공유 시 Reserve 초과 재할당 위험).
			TArray<TArray<uint32>>              KDetMask; KDetMask.Reserve(NumSteps);
			TArray<TArray<FVector4f>>           KDetCur;  KDetCur.Reserve(NumSteps);
			TArray<TArray<FVector4f>>           KDetPrev; KDetPrev.Reserve(NumSteps);
			TArray<TArray<FVector4f>>           KDetNext; KDetNext.Reserve(NumSteps);

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

				// Phase 2c: 별도 GDF 패스가 순회 시 쓸 플래그/충돌 파라미터를 상주 상태에 기록.
				R.bUseWorldGDF        = S.bUseWorldGDF;
				R.GDFCollisionRadius  = S.CollisionRadius;
				R.GDFFriction         = S.Friction;
				R.GDFTipFrictionScale = S.TipFrictionScale;
				R.GDFNumSub           = FMath::Max(1, S.NumSub); // 스윕 프레임-시작 역산용(최소 1).

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

				// G0: 오버라이드는 적분 없이도(NumSub=0) 기록해야 한다 — 로직 페이즈 프레임(Wrapping/Releasing 등).
				const bool bHasOverrides = S.HasOverrides() && S.OverrideFlags.Num() == N;
				if (S.HasOverrides() && !bHasOverrides)
				{
					UE_LOG(LogDynamicRopeGPU, Warning, TEXT("GPU override ignored: flags %d != nodes %d."),
						S.OverrideFlags.Num(), N);
				}

				if (S.NumSub <= 0 && !bHasOverrides && !S.bDetectContacts)
				{
					// 이번 프레임 적분/기록/감지 없음 — 위치 불변, 리드백도 그대로 둠. 상태만 외부 읽기(SRV)로
					// 확정한다(시드 업로드 직후 조기 종료 프레임 포함) — 아래 dispatch 경로의 호출과 동일 목적.
					GraphBuilder.UseExternalAccessMode(PosRDG, ERHIAccess::SRVMask);
					continue;
				}

				// --- per-rope 파라미터/충돌 버퍼(transient). NodeOffset/CapsuleOffset/SDFColliderOffset = 0(로프당 버퍼).
				TArray<FRopeCapsuleGPU>&     CapsFlat = KCaps.AddDefaulted_GetRef();
				for (const FRopeGPUCapsule& Cap : S.Capsules)
				{
					FRopeCapsuleGPU G;
					G.A = FVector4f((float)Cap.A.X, (float)Cap.A.Y, (float)Cap.A.Z, 0.0f);
					G.B = FVector4f((float)Cap.B.X, (float)Cap.B.Y, (float)Cap.B.Z, Cap.Radius);
					// 정적(InvDt 0)이면 prev=현재 — 커널이 prev 유효성 분기 없이 항상 lerp/변위 계산 가능.
					const bool bMoving = Cap.InvDeltaTime > 0.0f;
					const FVector& PA = bMoving ? Cap.PrevA : Cap.A;
					const FVector& PB = bMoving ? Cap.PrevB : Cap.B;
					G.PrevA = FVector4f((float)PA.X, (float)PA.Y, (float)PA.Z, 0.0f);
					G.PrevB = FVector4f((float)PB.X, (float)PB.Y, (float)PB.Z, Cap.InvDeltaTime);
					CapsFlat.Add(G);
				}

				// --- SDF 콜라이더(M3). 정적 베이크 그리드/헤더는 resident — 볼륨 집합 시그니처가 같으면
				// 재업로드하지 않는다(매 프레임 multi-MB 그리드 업로드 제거). 인스턴스(본 트랜스폼)만 매 프레임.
				TArray<FRopeSDFColliderGPU>& SDFCol = KSDFCol.AddDefaulted_GetRef();

				// 1) 고유 볼륨 dedup + 집합 시그니처(키/복셀수만 — distance 데이터는 만지지 않는다).
				TArray<const FRopeGPUSDFCollider*> UniqueVols; // 인덱스 = (재빌드 시) VolumeIndex
				TMap<const void*, int32>           FreshKeyToIndex;
				uint32 VolSig = 0;
				for (const FRopeGPUSDFCollider& Src : S.SDFColliders)
				{
					const int64 Voxels = (int64)Src.ResX * Src.ResY * Src.ResZ;
					if (!Src.Distances || Src.ResX < 2 || Src.ResY < 2 || Src.ResZ < 2 || Voxels <= 0)
					{
						continue;
					}
					if (!FreshKeyToIndex.Contains(Src.VolumeKey))
					{
						FreshKeyToIndex.Add(Src.VolumeKey, UniqueVols.Num());
						UniqueVols.Add(&Src);
						VolSig = HashCombine(VolSig, PointerHash(Src.VolumeKey));
						VolSig = HashCombine(VolSig, ::GetTypeHash((uint64)Voxels));
					}
				}

				// 2) 볼륨 distance/header 버퍼: 시그니처가 같으면 resident 재사용(업로드 0), 아니면 1회 재빌드.
				const bool bVolReuse = R.SDFDistBuf.IsValid() && R.SDFVolBuf.IsValid()
					&& R.SDFSetSig == VolSig && UniqueVols.Num() > 0;
				const TMap<const void*, int32>& KeyToIndex = bVolReuse ? R.SDFVolKeyToIndex : FreshKeyToIndex;

				FRDGBufferRef SDFDistBuf = nullptr;
				FRDGBufferRef SDFVolBuf  = nullptr;
				if (bVolReuse)
				{
					SDFDistBuf = GraphBuilder.RegisterExternalBuffer(R.SDFDistBuf);
					SDFVolBuf  = GraphBuilder.RegisterExternalBuffer(R.SDFVolBuf);
				}
				else if (UniqueVols.Num() > 0)
				{
					TArray<float>&             SDFDist = KSDFDist.AddDefaulted_GetRef();
					TArray<FRopeSDFVolumeGPU>& SDFVol  = KSDFVol.AddDefaulted_GetRef();
					SDFVol.Reserve(UniqueVols.Num());
					for (const FRopeGPUSDFCollider* Vp : UniqueVols)
					{
						FRopeSDFVolumeGPU V;
						V.DistOffset = SDFDist.Num();
						V.ResX = Vp->ResX; V.ResY = Vp->ResY; V.ResZ = Vp->ResZ;
						V.LocalMin  = FVector4f((float)Vp->LocalMin.X,  (float)Vp->LocalMin.Y,  (float)Vp->LocalMin.Z,  0.0f);
						V.LocalSize = FVector4f((float)Vp->LocalSize.X, (float)Vp->LocalSize.Y, (float)Vp->LocalSize.Z, 0.0f);
						SDFVol.Add(V);
						// 코드 → float(cm) dequant 후 업로드(셰이더 SDFDistances는 float 유지 → .usf 무변경).
						// 비대칭 밴드: d = code*(range/MaxCode) - NBIn, range = NBIn+NBOut. 코드는 복셀당
						// BytesPerCode 바이트(리틀엔디안): 1=uint8(max255), 2=uint16(max65535).
						const int32 VoxN = (int32)((int64)Vp->ResX * Vp->ResY * Vp->ResZ);
						const int32 Bpc = Vp->BytesPerCode;
						const float MaxCodeF = (Bpc >= 2) ? 65535.0f : 255.0f;
						const float NBIn = Vp->NarrowBandInner;
						const float Range = NBIn + Vp->NarrowBandOuter;
						const float DeqScale = (Range > 0.0f) ? (Range / MaxCodeF) : 0.0f;
						SDFDist.Reserve(SDFDist.Num() + VoxN);
						for (int32 Vi = 0; Vi < VoxN; ++Vi)
						{
							uint32 Code = Vp->Distances[Vi * Bpc];
							if (Bpc >= 2)
							{
								Code |= static_cast<uint32>(Vp->Distances[Vi * Bpc + 1]) << 8;
							}
							SDFDist.Add(static_cast<float>(Code) * DeqScale - NBIn); // 바깥 +
						}
					}
					SDFDistBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFDistances"),
						sizeof(float), SDFDist.Num(), SDFDist.GetData(), (uint64)SDFDist.Num() * sizeof(float));
					SDFVolBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFVolumes"),
						sizeof(FRopeSDFVolumeGPU), SDFVol.Num(), SDFVol.GetData(), (uint64)SDFVol.Num() * sizeof(FRopeSDFVolumeGPU));
					R.SDFDistBuf = GraphBuilder.ConvertToExternalBuffer(SDFDistBuf);
					R.SDFVolBuf  = GraphBuilder.ConvertToExternalBuffer(SDFVolBuf);
					R.SDFSetSig  = VolSig;
					R.SDFVolKeyToIndex = FreshKeyToIndex; // 복사(아래 인스턴스 루프가 KeyToIndex=FreshKeyToIndex를 계속 참조).
				}

				// 3) 인스턴스(매 프레임): VolumeIndex(캐시/신규 맵) + 현재 본 트랜스폼.
				for (const FRopeGPUSDFCollider& Src : S.SDFColliders)
				{
					const int32* VolIdx = KeyToIndex.Find(Src.VolumeKey);
					if (!VolIdx)
					{
						continue; // 무효 볼륨(위 dedup 조건과 일치).
					}
					const FQuat   Q  = Src.BoneToWorld.GetRotation();
					const FVector T  = Src.BoneToWorld.GetTranslation();
					const FVector Sc = Src.BoneToWorld.GetScale3D();
					const FQuat   PQ = Src.PrevBoneToWorld.GetRotation();
					const FVector PT = Src.PrevBoneToWorld.GetTranslation();
					FRopeSDFColliderGPU C;
					C.VolumeIndex = *VolIdx;
					C.Rotation        = FVector4f((float)Q.X, (float)Q.Y, (float)Q.Z, (float)Q.W);
					C.Translation     = FVector4f((float)T.X, (float)T.Y, (float)T.Z, 0.0f);
					C.Scale           = FVector4f((float)Sc.X, (float)Sc.Y, (float)Sc.Z, 0.0f);
					C.PrevRotation    = FVector4f((float)PQ.X, (float)PQ.Y, (float)PQ.Z, (float)PQ.W);
					C.PrevTranslation = FVector4f((float)PT.X, (float)PT.Y, (float)PT.Z, Src.InvDeltaTime); // w=InvDt
					SDFCol.Add(C);
				}

				// 유효 개수 — 더미 패딩 *전* 확정.
				const int32 NumValidCaps   = CapsFlat.Num();
				const int32 NumValidSDFCol = SDFCol.Num();

				// 구조화 버퍼는 원소 >=1 — 비면 더미 1개(어느 노드도 참조 안 함; count는 0으로 고정).
				if (CapsFlat.Num() == 0) { CapsFlat.AddZeroed(1); }
				if (SDFCol.Num() == 0)   { SDFCol.AddZeroed(1); }

				// 이 로프에 SDF 볼륨이 없으면 distance/header도 더미 1개 transient 생성(셰이더는 NumSDFColliders=0이라 미참조).
				if (!SDFDistBuf)
				{
					TArray<float>&             DummyDist = KSDFDist.AddDefaulted_GetRef(); DummyDist.AddZeroed(1);
					TArray<FRopeSDFVolumeGPU>& DummyVol  = KSDFVol.AddDefaulted_GetRef();  DummyVol.AddZeroed(1);
					SDFDistBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFDistances.Dummy"),
						sizeof(float), 1, DummyDist.GetData(), sizeof(float));
					SDFVolBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFVolumes.Dummy"),
						sizeof(FRopeSDFVolumeGPU), 1, DummyVol.GetData(), sizeof(FRopeSDFVolumeGPU));
				}

				// --- Override(G0) 업로드: 노드별 플래그/타깃/질량(transient, 오버라이드 프레임만 실데이터).
				// 없으면 더미 1개 + bHasOverrides=0 → 셰이더가 참조하지 않는다.
				TArray<uint32>&    OvFlags = KOvFlags.AddDefaulted_GetRef();
				TArray<FVector4f>& OvPos   = KOvPos.AddDefaulted_GetRef();
				TArray<FVector4f>& OvPrev  = KOvPrev.AddDefaulted_GetRef();
				TArray<float>&     OvInv   = KOvInv.AddDefaulted_GetRef();
				if (bHasOverrides)
				{
					const bool bHavePos  = S.OverridePositions.Num() == N;
					const bool bHavePrev = S.OverridePrevPositions.Num() == N;
					const bool bHaveInv  = S.OverrideInvMass.Num() == N;
					OvFlags.SetNumUninitialized(N);
					OvPos.SetNumUninitialized(N);
					OvPrev.SetNumUninitialized(N);
					OvInv.SetNumUninitialized(N);
					for (int32 k = 0; k < N; ++k)
					{
						OvFlags[k] = S.OverrideFlags[k];
						const FVector Pv  = bHavePos  ? S.OverridePositions[k]     : FVector::ZeroVector;
						const FVector Ppv = bHavePrev ? S.OverridePrevPositions[k] : FVector::ZeroVector;
						OvPos[k]  = FVector4f((float)Pv.X,  (float)Pv.Y,  (float)Pv.Z,  0.0f);
						OvPrev[k] = FVector4f((float)Ppv.X, (float)Ppv.Y, (float)Ppv.Z, 0.0f);
						OvInv[k]  = bHaveInv ? S.OverrideInvMass[k] : 1.0f;
					}
				}
				else
				{
					OvFlags.AddZeroed(1);
					OvPos.AddZeroed(1);
					OvPrev.AddZeroed(1);
					OvInv.AddZeroed(1);
				}
				FRDGBufferRef OvFlagsBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.OverrideFlags"),
					sizeof(uint32), OvFlags.Num(), OvFlags.GetData(), (uint64)OvFlags.Num() * sizeof(uint32));
				FRDGBufferRef OvPosBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.OverridePositions"),
					sizeof(FVector4f), OvPos.Num(), OvPos.GetData(), (uint64)OvPos.Num() * sizeof(FVector4f));
				FRDGBufferRef OvPrevBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.OverridePrevPositions"),
					sizeof(FVector4f), OvPrev.Num(), OvPrev.GetData(), (uint64)OvPrev.Num() * sizeof(FVector4f));
				FRDGBufferRef OvInvBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.OverrideInvMass"),
					sizeof(float), OvInv.Num(), OvInv.GetData(), (uint64)OvInv.Num() * sizeof(float));

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
				P.TipFrictionScale  = S.TipFrictionScale;
				P.CollisionPasses   = FMath::Clamp(S.CollisionPasses, 1, FMath::Max(1, S.Iterations));
				P.SweepStep         = S.SweepStep;
				P.MaxSweepSamples   = FMath::Max(1, S.MaxSweepSamples);
				P.SDFColliderOffset = 0;
				P.NumSDFColliders   = NumValidSDFCol;
				P.bHasOverrides     = bHasOverrides ? 1 : 0;
				P.Gravity           = FVector4f((float)S.Gravity.X, (float)S.Gravity.Y, (float)S.Gravity.Z, 0.0f);
				P.PinPrev           = FVector4f((float)S.StartPinPrev.X,   (float)S.StartPinPrev.Y,   (float)S.StartPinPrev.Z,   0.0f);
				P.PinTarget         = FVector4f((float)S.StartPinTarget.X, (float)S.StartPinTarget.Y, (float)S.StartPinTarget.Z, 0.0f);
				ParamsArr.Add(P);

				const int32 NumCapsulesTotal = CapsFlat.Num();

				FRDGBufferRef ParamsBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Params"),
					sizeof(FRopeGPUParamsGPU), 1, ParamsArr.GetData(), sizeof(FRopeGPUParamsGPU));
				FRDGBufferRef CapsulesBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Capsules"),
					sizeof(FRopeCapsuleGPU), NumCapsulesTotal, CapsFlat.GetData(), (uint64)NumCapsulesTotal * sizeof(FRopeCapsuleGPU));
				// SDFDistBuf/SDFVolBuf는 위에서 resident(또는 더미)로 준비됨. 매 프레임은 인스턴스 버퍼만 생성.
				FRDGBufferRef SDFColBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFColliders"),
					sizeof(FRopeSDFColliderGPU), SDFCol.Num(), SDFCol.GetData(), (uint64)SDFCol.Num() * sizeof(FRopeSDFColliderGPU));

				FRopeXPBDSolveCS::FParameters* PassParams = GraphBuilder.AllocParameters<FRopeXPBDSolveCS::FParameters>();
				PassParams->NumRopes      = 1;
				PassParams->Params        = GraphBuilder.CreateSRV(ParamsBuf);
				PassParams->Capsules      = GraphBuilder.CreateSRV(CapsulesBuf);
				PassParams->SDFDistances  = GraphBuilder.CreateSRV(SDFDistBuf);
				PassParams->SDFVolumes    = GraphBuilder.CreateSRV(SDFVolBuf);
				PassParams->SDFColliders  = GraphBuilder.CreateSRV(SDFColBuf);
				PassParams->OverrideFlags         = GraphBuilder.CreateSRV(OvFlagsBuf);
				PassParams->OverridePositions     = GraphBuilder.CreateSRV(OvPosBuf);
				PassParams->OverridePrevPositions = GraphBuilder.CreateSRV(OvPrevBuf);
				PassParams->OverrideInvMass       = GraphBuilder.CreateSRV(OvInvBuf);
				PassParams->InvMass       = GraphBuilder.CreateUAV(InvMassRDG);
				PassParams->Positions     = GraphBuilder.CreateUAV(PosRDG);
				PassParams->PrevPositions = GraphBuilder.CreateUAV(PrevRDG);
				// 장력(λ) 출력: 프레임 transient(N 슬롯, 커널이 매 dispatch 전체를 다시 쓴다 — 영속 불필요).
				FRDGBufferRef LambdaRDG = GraphBuilder.CreateBuffer(
					FRDGBufferDesc::CreateStructuredDesc(sizeof(float), N), TEXT("Rope.LambdaDist"));
				PassParams->OutLambdaDist = GraphBuilder.CreateUAV(LambdaRDG);

				// Phase 1: 항상 lean permutation(GDF off) — 기존 동작 유지. GDF 경로 선택은 Phase 3(View/GDF 주입 후).
				FRopeXPBDSolveCS::FPermutationDomain PermVec;
				PermVec.Set<FRopeXPBDSolveCS::FGDFDim>(false);
				TShaderMapRef<FRopeXPBDSolveCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), PermVec);
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

				// --- 장력(λ) 리드백 무장: 솔브 프레임(NumSub>0)에만 — override-only 프레임은 λ가 0이라
				// 무장하지 않고 직전 장력을 유지한다(GT는 갱신분이 있을 때만 덮어씀).
				if (!R.bLambdaArmed && S.NumSub > 0)
				{
					if (!R.LambdaReadback) { R.LambdaReadback = new FRHIGPUBufferReadback(TEXT("Rope.LambdaReadback")); }
					AddEnqueueCopyPass(GraphBuilder, R.LambdaReadback, LambdaRDG, (uint32)N * sizeof(float));
					R.LambdaFixedDt = S.FixedDt;
					R.bLambdaArmed = true;
				}

				// --- 접촉 감지(G3): 솔브 뒤 post-solve 위치를 스윕. RDG가 solve(UAV)→detect(SRV) 순서를 보장한다.
				// 노드당 2슬롯(actual+predictive) 출력. 감지가 있을 때만 ContactBuf(resident, 2N슬롯) 확보.
				if (S.bDetectContacts)
				{
					const bool bContactSeed = !R.ContactBuf.IsValid() || bSeed;
					FRDGBufferRef ContactRDG = nullptr;
					if (bContactSeed)
					{
						ContactRDG = GraphBuilder.CreateBuffer(
							FRDGBufferDesc::CreateStructuredDesc(sizeof(FRopeGPUContactGPU), 2 * N), TEXT("Rope.Contacts"));
						R.ContactBuf = GraphBuilder.ConvertToExternalBuffer(ContactRDG);
						R.bContactArmed = false; // 재생성 → 직전 접촉 리드백은 stale.
					}
					else
					{
						ContactRDG = GraphBuilder.RegisterExternalBuffer(R.ContactBuf);
					}

					// 예측 접촉용 whip 가이드 버퍼(G3b). whip 활성 시 노드별 마스크/타깃, 아니면 더미 1개.
					const bool bHasWhip = S.WhipGuidedMask.Num() == N && S.PredictionFrames > 0.0f;
					TArray<uint32>&    GMask = KDetMask.AddDefaulted_GetRef();
					TArray<FVector4f>& WCur  = KDetCur.AddDefaulted_GetRef();
					TArray<FVector4f>& WPrev = KDetPrev.AddDefaulted_GetRef();
					TArray<FVector4f>& WNext = KDetNext.AddDefaulted_GetRef();
					if (bHasWhip)
					{
						const bool bHaveCur  = S.WhipCurrentTargets.Num() == N;
						const bool bHavePrev = S.WhipPrevTargets.Num() == N;
						const bool bHaveNext = S.WhipNextTargets.Num() == N;
						GMask.SetNumUninitialized(N);
						WCur.SetNumUninitialized(N);
						WPrev.SetNumUninitialized(N);
						WNext.SetNumUninitialized(N);
						for (int32 k = 0; k < N; ++k)
						{
							GMask[k] = S.WhipGuidedMask[k];
							const FVector Cv = bHaveCur  ? S.WhipCurrentTargets[k] : FVector::ZeroVector;
							const FVector Pv = bHavePrev ? S.WhipPrevTargets[k]    : FVector::ZeroVector;
							const FVector Nv = bHaveNext ? S.WhipNextTargets[k]    : FVector::ZeroVector;
							WCur[k]  = FVector4f((float)Cv.X, (float)Cv.Y, (float)Cv.Z, 0.0f);
							WPrev[k] = FVector4f((float)Pv.X, (float)Pv.Y, (float)Pv.Z, 0.0f);
							WNext[k] = FVector4f((float)Nv.X, (float)Nv.Y, (float)Nv.Z, 0.0f);
						}
					}
					else
					{
						GMask.AddZeroed(1); WCur.AddZeroed(1); WPrev.AddZeroed(1); WNext.AddZeroed(1);
					}
					FRDGBufferRef GMaskBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.DetectGuidedMask"),
						sizeof(uint32), GMask.Num(), GMask.GetData(), (uint64)GMask.Num() * sizeof(uint32));
					FRDGBufferRef WCurBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.DetectWhipCur"),
						sizeof(FVector4f), WCur.Num(), WCur.GetData(), (uint64)WCur.Num() * sizeof(FVector4f));
					FRDGBufferRef WPrevBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.DetectWhipPrev"),
						sizeof(FVector4f), WPrev.Num(), WPrev.GetData(), (uint64)WPrev.Num() * sizeof(FVector4f));
					FRDGBufferRef WNextBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.DetectWhipNext"),
						sizeof(FVector4f), WNext.Num(), WNext.GetData(), (uint64)WNext.Num() * sizeof(FVector4f));

					FRopeContactDetectCS::FParameters* DetectParams = GraphBuilder.AllocParameters<FRopeContactDetectCS::FParameters>();
					DetectParams->DetectNumNodes       = N;
					DetectParams->DetectNumCapsules    = NumValidCaps;
					DetectParams->DetectNumSDF         = NumValidSDFCol;
					DetectParams->DetectContactRadius  = S.ContactRadius;
					DetectParams->DetectSegmentLength  = S.SegmentLength;
					DetectParams->DetectPredictionFrames = FMath::Max(0.0f, S.PredictionFrames);
					DetectParams->DetectHasGuidedNodes = bHasWhip ? 1 : 0;
					DetectParams->Capsules             = GraphBuilder.CreateSRV(CapsulesBuf);
					DetectParams->SDFDistances         = GraphBuilder.CreateSRV(SDFDistBuf);
					DetectParams->SDFVolumes           = GraphBuilder.CreateSRV(SDFVolBuf);
					DetectParams->SDFColliders         = GraphBuilder.CreateSRV(SDFColBuf);
					DetectParams->DetectPositions      = GraphBuilder.CreateSRV(PosRDG);
					DetectParams->DetectPrevPositions  = GraphBuilder.CreateSRV(PrevRDG);
					DetectParams->DetectGuidedMask     = GraphBuilder.CreateSRV(GMaskBuf);
					DetectParams->DetectWhipCur        = GraphBuilder.CreateSRV(WCurBuf);
					DetectParams->DetectWhipPrev       = GraphBuilder.CreateSRV(WPrevBuf);
					DetectParams->DetectWhipNext       = GraphBuilder.CreateSRV(WNextBuf);
					DetectParams->OutContacts          = GraphBuilder.CreateUAV(ContactRDG);

					TShaderMapRef<FRopeContactDetectCS> DetectShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
					FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeContactDetect"),
						DetectShader, DetectParams, FIntVector(1, 1, 1));

					if (!R.bContactArmed)
					{
						if (!R.ContactReadback) { R.ContactReadback = new FRHIGPUBufferReadback(TEXT("Rope.ContactReadback")); }
						AddEnqueueCopyPass(GraphBuilder, R.ContactReadback, ContactRDG, (uint32)(2 * N) * sizeof(FRopeGPUContactGPU));
						R.bContactArmed = true;
					}
				}

				// 렌더 raw 튜브 경로(M5b/B2)가 이 그래프 *밖에서* PosBuf를 SRV로 직독한다 — 외부 접근 모드로
				// 마지막 패스(solve/copy/detect) 뒤 SRV 전이(배리어)를 매 프레임 확정한다. 이게 없으면 그래프
				// 종료 상태가 리드백 copy 유무에 따라 UAVCompute/CopySrc로 오락가락해, 배리어 없는 프레임에
				// 튜브가 이전/미완성 위치를 읽어 wrap 노드가 떨린다(CL167 회귀). VE(GDF) 경로의 후속 쓰기는
				// DispatchGDFCollision이 UseInternalAccessMode로 재획득 후 다시 외부로 되돌린다.
				GraphBuilder.UseExternalAccessMode(PosRDG, ERHIAccess::SRVMask);
			}

}

void FRopeGPUSolver::Step(TArray<FRopeGPUResidentStep>&& Steps)
{
	if (Steps.Num() == 0)
	{
		return;
	}
	// 전용(자체) 그래프 경로 — 서브시스템 Tick 트리거(G4 기본). 씬 렌더 타이밍과 무관하게 즉시 실행.
	ENQUEUE_RENDER_COMMAND(RopeResidentStep)(
		[this, Steps = MoveTemp(Steps)](FRHICommandListImmediate& RHICmdList) mutable
		{
			FRDGBuilder GraphBuilder(RHICmdList);
			RunSteps_RenderThread(GraphBuilder, Steps, nullptr, FVector3f::ZeroVector);
			GraphBuilder.Execute();
		});
}

void FRopeGPUSolver::EnqueueSteps(TArray<FRopeGPUResidentStep>&& Steps)
{
	if (Steps.Num() == 0)
	{
		return;
	}
	// 씬 렌더러 그래프 경로(GDF 월드 충돌): dispatch는 안 하고 RT pending 큐에 쌓아둔다. 뷰 확장이 이번
	// 프레임 PreRenderBasePass에서 씬 그래프로 flush(GDF 파라미터가 유효한 타이밍 + 튜브 무지연).
	ENQUEUE_RENDER_COMMAND(RopeEnqueueSteps)(
		[this, Steps = MoveTemp(Steps)](FRHICommandListImmediate&) mutable
		{
			// 교체 시맨틱: 이번 프레임 step으로 대체한다(직전 프레임분이 뷰 확장에서 소비 안 됐어도 — 씬
			// 렌더가 없던 프레임 등 — 최신만 유효하므로 누적하지 않는다).
			Impl->PendingSteps = MoveTemp(Steps);
		});
}

void FRopeGPUSolver::DispatchPending_RenderThread(FRDGBuilder& GraphBuilder,
	const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation)
{
	check(IsInRenderingThread());
	if (Impl->PendingSteps.Num() == 0)
	{
		return;
	}
	RunSteps_RenderThread(GraphBuilder, Impl->PendingSteps, GDF, PreViewTranslation);
	Impl->PendingSteps.Reset();
}

void FRopeGPUSolver::DispatchGDFCollision_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View,
	const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation)
{
	check(IsInRenderingThread());

	// 프레임 공용 GDF 셰이더 파라미터(미빌드면 bWorldGDFValid=0 → 셰이더 no-op).
	FGlobalDistanceFieldParameters2 GDFShaderParams;
	uint32 bWorldGDFValid = 0;
	FillGDFShaderParams(GDF, GDFShaderParams, bWorldGDFValid);

	// 상주 로프 중 GDF 대상만 순회. PosBuf는 같은 그래프에서 솔브가 이미 등록(UAV)했으므로 dedup → solve→GDF
	// 순서 자동. 이후 튜브가 PosBuf를 읽으므로 GDF→tube 순서도 RDG가 보장한다.
	for (TPair<uint32, FRopeResidentRope>& Pair : Impl->RtRopes)
	{
		FRopeResidentRope& R = Pair.Value;
		if (!R.bUseWorldGDF || R.NumNodes < 2
			|| !R.PosBuf.IsValid() || !R.PrevBuf.IsValid() || !R.InvMassBuf.IsValid())
		{
			continue;
		}

		FRDGBufferRef PosRDG = GraphBuilder.RegisterExternalBuffer(R.PosBuf);
		FRDGBufferRef PrevRDG = GraphBuilder.RegisterExternalBuffer(R.PrevBuf);
		FRDGBufferRef InvRDG  = GraphBuilder.RegisterExternalBuffer(R.InvMassBuf);

		// RunSteps가 PosBuf를 외부 접근(SRVMask)으로 확정했으므로 UAV 쓰기 전에 내부 추적으로 재획득한다
		// (외부 접근 상태의 쓰기는 RDG validation 위반). 미확정 상태여도 no-op라 안전.
		GraphBuilder.UseInternalAccessMode(PosRDG);

		// 진단(GDFDebug): 노드당 2 슬롯([Dist,dot,Pen,flag] / [SegLen,minSd,Move1,attempt]) 출력 버퍼.
		// 렌더/다음 프레임에 안 쓰이므로 매 dispatch 임시(transient). UAV는 셰이더가 항상 쓰므로 cvar와
		// 무관하게 바인딩하고, CPU 리드백 copy만 cvar on일 때 아래에서 무장한다.
		FRDGBufferRef DebugRDG = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), FMath::Max(1, 2 * R.NumNodes)), TEXT("Rope.GDFDebug"));

		FRopeGDFCollisionCS::FParameters* P = GraphBuilder.AllocParameters<FRopeGDFCollisionCS::FParameters>();
		P->View                = View.ViewUniformBuffer;
		P->GDF                 = GDFShaderParams;
		P->GDFPreViewTranslation = PreViewTranslation;
		P->NumNodes            = (uint32)R.NumNodes;
		P->CollisionRadius     = R.GDFCollisionRadius;
		P->Friction            = R.GDFFriction;
		P->TipFrictionScale    = R.GDFTipFrictionScale;
		P->bWorldGDFValid      = bWorldGDFValid;
		P->bEntrySidePush      = (CVarRopeGDFEntrySidePush.GetValueOnRenderThread() != 0) ? 1u : 0u;
		P->bSweep              = (CVarRopeGDFSweep.GetValueOnRenderThread() != 0) ? 1u : 0u;
		P->SweepBackSteps      = (uint32)FMath::Max(1, R.GDFNumSub);
		P->InvMass             = GraphBuilder.CreateSRV(InvRDG);
		P->Positions           = GraphBuilder.CreateUAV(PosRDG);
		P->PrevPositions       = GraphBuilder.CreateUAV(PrevRDG);
		P->DebugOut            = GraphBuilder.CreateUAV(DebugRDG);

		TShaderMapRef<FRopeGDFCollisionCS> Shader(GetGlobalShaderMap(View.GetFeatureLevel()));
		FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeGDFCollision"),
			Shader, P, FIntVector(1, 1, 1)); // 로프 1개 = 스레드그룹 1개

		// 진단 리드백 무장(cvar on일 때만): GDF 패스가 방금 쓴 진단값을 비동기 copy. consume/로그는 RunSteps Loop1(다음 프레임).
		if (CVarRopeGDFDebug.GetValueOnRenderThread() != 0 && !R.bGDFDebugArmed)
		{
			if (!R.GDFDebugReadback) { R.GDFDebugReadback = new FRHIGPUBufferReadback(TEXT("Rope.GDFDebugReadback")); }
			AddEnqueueCopyPass(GraphBuilder, R.GDFDebugReadback, DebugRDG, (uint32)(2 * R.NumNodes) * sizeof(FVector4f));
			R.bGDFDebugArmed = true;
		}

		// GDF 쓰기 완료 — 다시 외부 읽기(SRV)로 확정: 같은 그래프의 튜브 SRV 읽기 + 다음 프레임 렌더 raw 직독 유효.
		GraphBuilder.UseExternalAccessMode(PosRDG, ERHIAccess::SRVMask);
	}
}
