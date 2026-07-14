// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeGPUSolver.h"
#include "DynamicRopeShadersLog.h"

#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"
// FRHICommandListExecutor, CreateShaderResourceView
#include "RHICommandList.h"
// TStaticSamplerState (GDF 샘플러) — Phase 2c
#include "RHIStaticStates.h"
// FGlobalDistanceFieldParameters2 / _Minimal — Phase 2c
#include "GlobalDistanceFieldParameters.h"
// GBlackVolumeTexture / GBlackUintVolumeTexture — Phase 2c
#include "GlobalRenderResources.h"
// FSceneView / FViewUniformShaderParameters (GDF 패스 View UB) — Phase 2c
#include "SceneView.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "Misc/ScopeLock.h"

// 노드 버킷(스레드그룹 크기 == groupshared/numthreads 크기). 로프 1개 = 스레드그룹 1개, 노드 = 스레드라,
// 예전엔 모든 로프가 고정 256 그룹을 잡아 노드 수가 적은 로프는 스레드 대부분이 idle(배리어에는 참여)이었다.
// 이제 NumNodes 이상인 가장 작은 버킷을 골라(퍼뮤테이션) 그 낭비를 없애고, 최상단 512로 지원 노드 상한을
// 올린다. numthreads(ROPE_THREADS)/groupshared(ROPE_MAX_NODES)는 버킷 값으로 스케일 — 퍼뮤테이션이
// ROPE_MAX_NODES를 버킷 값으로 설정하고 ModifyCompilationEnvironment가 ROPE_THREADS=버킷을 맞춘다.
// groupshared 예산: solve 9float/node → 512노드=18KB(<32KB). detect는 groupshared 없음(numthreads만).
static constexpr int32 GRopeNodeBuckets[] = { 64, 128, 256, 512 };
static_assert(GRopeNodeBuckets[UE_ARRAY_COUNT(GRopeNodeBuckets) - 1] == FRopeGPUSolver::MaxNodes,
	"최상단 노드 버킷이 FRopeGPUSolver::MaxNodes와 일치해야 한다(서브시스템 GPU 후보 게이트가 MaxNodes를 쓴다).");

// NumNodes 이상인 가장 작은 버킷. 없으면(> 상한) 0. 호출부는 MaxNodes 게이트 뒤라 항상 ≥64를 받는다.
static int32 RopeNodeBucket(int32 NumNodes)
{
	for (int32 Bucket : GRopeNodeBuckets)
	{
		if (NumNodes <= Bucket) { return Bucket; }
	}
	return 0;
}

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
	// M2: 이 로프의 capsule 글로벌 시작 인덱스
	int32     CapsuleOffset;
	// M2: capsule 수(0이면 충돌 없음)
	int32     NumCapsules;
	// M2: 노드 두께
	float     CollisionRadius;
	// M2: 접선 감쇠
	float     Friction;
	// M2: swept 샘플 간격
	float     SweepStep;
	// M2: 세그먼트당 샘플 상한
	int32     MaxSweepSamples;
	// M3: 이 로프의 SDF collider 글로벌 시작 인덱스
	int32     SDFColliderOffset;
	// M3: SDF collider 수(0이면 SDF 충돌 없음)
	int32     NumSDFColliders;
	// 자유단 마찰 배율(고정점=1, 끝=이 값). Pad0 슬롯 재사용.
	float     TipFrictionScale = 1.0f;
	// substep당 충돌 해소 패스 수(Iters로 상한). Pad1 슬롯 재사용.
	int32     CollisionPasses = 1;
	// G0: 이 로프에 노드별 override(타깃/질량 주입)가 있는가.
	int32     bHasOverrides = 0;
	// 정적 박스(OBB) 수(0이면 박스 충돌 없음). Pad2 슬롯 재사용.
	int32     NumBoxes = 0;
	// 정적 컨벡스(평면 집합) 수(0이면 컨벡스 충돌 없음). Pad3 슬롯 재사용.
	int32     NumConvexes = 0;
	// 각도-허용 벤딩: straightness ≤ 이 값이면 펴는 힘 0. Pad4 슬롯 재사용.
	float     BendReleaseRatio = 0.70f;
	// straightness ≥ 이 값이면 펴는 힘 100%.
	float     BendFullRatio    = 0.92f;
	// Strain limiting 최대 신장 배율(<1=비활성). Pad5 슬롯 재사용.
	float     MaxStretchRatio  = 1.5f;
	int32     Pad6 = 0;
	int32     Pad7 = 0;
	FVector4f Gravity;
	FVector4f PinPrev;
	FVector4f PinTarget;
};
static_assert(sizeof(FRopeGPUParamsGPU) % 16 == 0, "FRopeGPUParamsGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeCapsule와 1:1 미러. xyz=세그먼트 끝점, B.w=반지름, PrevB.w=InvDeltaTime(0이면 정적).
struct FRopeCapsuleGPU
{
	FVector4f A;
	// w = Radius
	FVector4f B;
	// 이전 프레임 끝점(표면 속도 드래그/substep 상대 운동). 정적이면 패킹이 A/B로 채운다.
	FVector4f PrevA;
	// w = InvDeltaTime
	FVector4f PrevB;
};
static_assert(sizeof(FRopeCapsuleGPU) % 16 == 0, "FRopeCapsuleGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeBox와 1:1 미러. 박스(OBB): 월드 center + quat + 반폭 + 이전 프레임 center/rot(동적 표면 속도).
struct FRopeBoxGPU
{
	// xyz, w = InvDeltaTime(1/프레임dt; 0이면 정적)
	FVector4f Center;
	// quat (x,y,z,w)
	FVector4f Rot;
	// xyz
	FVector4f HalfExtents;
	// xyz — 이전 프레임 중심(정적이면 패킹이 Center로 채움)
	FVector4f PrevCenter;
	// quat — 이전 프레임 회전
	FVector4f PrevRot;
};
static_assert(sizeof(FRopeBoxGPU) % 16 == 0, "FRopeBoxGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeConvex와 1:1 미러. 평면 풀 오프셋/개수 + 로컬 AABB + 강체(curr/prev). 평면은 ConvexPlanes(로컬)에 별도.
struct FRopeConvexGPU
{
	int32     PlaneOffset;
	int32     PlaneCount;
	int32     Pad0 = 0;
	int32     Pad1 = 0;
	// xyz
	FVector4f LocalBoundsCenter;
	// xyz, w = InvDeltaTime
	FVector4f LocalBoundsExtent;
	// quat (curr)
	FVector4f Rot;
	// xyz
	FVector4f Trans;
	// quat (prev)
	FVector4f PrevRot;
	// xyz
	FVector4f PrevTrans;
};
static_assert(sizeof(FRopeConvexGPU) % 16 == 0, "FRopeConvexGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeSDFVolume와 1:1 미러. 본 로컬 grid 헤더(distance는 SDFDistances 버퍼에 DistOffset부터).
struct FRopeSDFVolumeGPU
{
	int32     DistOffset;
	int32     ResX;
	int32     ResY;
	int32     ResZ;
	// xyz
	FVector4f LocalMin;
	// xyz
	FVector4f LocalSize;
};
static_assert(sizeof(FRopeSDFVolumeGPU) % 16 == 0, "FRopeSDFVolumeGPU must be 16-byte aligned to match HLSL structured buffer.");

// HLSL FRopeSDFCollider와 1:1 미러. 볼륨 인덱스 + 본→월드 트랜스폼(quat/trans/scale, 행렬 레이아웃 회피).
struct FRopeSDFColliderGPU
{
	int32     VolumeIndex;
	int32     Pad0 = 0;
	int32     Pad1 = 0;
	int32     Pad2 = 0;
	// quat (x,y,z,w) — 현재 프레임
	FVector4f Rotation;
	// xyz
	FVector4f Translation;
	// xyz
	FVector4f Scale;
	// quat (x,y,z,w) — 이전 프레임(CCD 상대 운동/표면속도용)
	FVector4f PrevRotation;
	// xyz, w = InvDeltaTime(1/프레임dt; 0이면 정적)
	FVector4f PrevTranslation;
};
static_assert(sizeof(FRopeSDFColliderGPU) % 16 == 0, "FRopeSDFColliderGPU must be 16-byte aligned to match HLSL structured buffer.");

class FRopeXPBDSolveCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeXPBDSolveCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeXPBDSolveCS, FGlobalShader);

	// 노드 버킷 = numthreads/groupshared 크기. 값이 곧 ROPE_MAX_NODES define으로 .usf에 전달된다.
	class FNodeBucket : SHADER_PERMUTATION_SPARSE_INT("ROPE_MAX_NODES", 64, 128, 256, 512);
	// GDF 월드 충돌을 substep 제약으로 통합하는 permutation. on일 때만 GDF 헤더 include + View/GDF 바인딩.
	// off(기본, View 없는 Step 경로 겸용)는 GDF 미참조 → View 없이 기존대로 컴파일된다.
	class FGDFDim : SHADER_PERMUTATION_BOOL("ROPE_USE_GDF");
	using FPermutationDomain = TShaderPermutationDomain<FNodeBucket, FGDFDim>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRopes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeGPUParams>, Params)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeCapsule>, Capsules)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, SDFDistances)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFVolume>, SDFVolumes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFCollider>, SDFColliders)
		// 정적 박스 — solve 전용(감지 CS는 미참조라 스트립).
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeBox>, Boxes)
		// 정적 컨벡스 — solve 전용.
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeConvex>, Convexes)
		// 컨벡스 평면 평탄 풀.
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, ConvexPlanes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OverrideFlags)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, OverridePositions)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, OverridePrevPositions)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, OverrideInvMass)
		// G0: override가 질량 마스크를 영속시키므로 RW.
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, InvMass)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, Positions)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, PrevPositions)
		// 장력 리드백(마지막 substep 세그먼트 λ).
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutLambdaDist)
		// --- GDF 통합 경로(FGDFDim on일 때만 셰이더가 참조; off면 미사용 → 언바운드 허용).
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
		// ROPE_MAX_NODES(groupshared)는 FNodeBucket 차원이 자동 설정. ROPE_THREADS(numthreads)를 같은 버킷으로 맞춘다.
		const FPermutationDomain PermutationVector(Parameters.PermutationId);
		OutEnvironment.SetDefine(TEXT("ROPE_THREADS"), PermutationVector.Get<FNodeBucket>());
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
	// xyz 접촉점, w Penetration
	FVector4f WorldPoint;
	FVector4f Normal;
	FVector4f SurfaceVel;
};
static_assert(sizeof(FRopeGPUContactGPU) % 16 == 0, "FRopeGPUContactGPU must be 16-byte aligned to match HLSL structured buffer.");

// 접촉 감지 컴퓨트(G3). 솔브 후 상주 위치를 스윕해 노드당 최심 접촉을 OutContacts에 기록한다.
// 별도 파일(RopeContactDetect.usf) — 솔브 셰이더와는 콜라이더 모델/질의(RopeColliderCommon.ush)만 공유한다.
class FRopeContactDetectCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeContactDetectCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeContactDetectCS, FGlobalShader);

	// 노드 버킷 = numthreads 크기(감지 커널은 groupshared 없음 — numthreads만 스케일). 솔브와 동일 버킷 집합.
	class FNodeBucket : SHADER_PERMUTATION_SPARSE_INT("ROPE_MAX_NODES", 64, 128, 256, 512);
	using FPermutationDomain = TShaderPermutationDomain<FNodeBucket>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(int32, DetectNumNodes)
		SHADER_PARAMETER(int32, DetectNumCapsules)
		SHADER_PARAMETER(int32, DetectNumSDF)
		SHADER_PARAMETER(int32, DetectNumBoxes)
		SHADER_PARAMETER(float, DetectContactRadius)
		SHADER_PARAMETER(float, DetectSegmentLength)
		SHADER_PARAMETER(float, DetectPredictionFrames)
		SHADER_PARAMETER(int32, DetectHasGuidedNodes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeCapsule>, Capsules)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, SDFDistances)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFVolume>, SDFVolumes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFCollider>, SDFColliders)
		// 랩 가능 박스 감지(정적 박스는 NumDetectBoxes로 자름).
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeBox>, Boxes)
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
		// ROPE_MAX_NODES는 FNodeBucket 차원이 자동 설정. ROPE_THREADS(numthreads)를 같은 버킷으로 맞춘다.
		const FPermutationDomain PermutationVector(Parameters.PermutationId);
		OutEnvironment.SetDefine(TEXT("ROPE_THREADS"), PermutationVector.Get<FNodeBucket>());
	}
};

IMPLEMENT_GLOBAL_SHADER(FRopeContactDetectCS, "/Plugin/DynamicRope/Private/RopeContactDetect.usf", "RopeContactDetectCS", SF_Compute);

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
	// 마지막으로 시드한 generation(다르면 재시드)
	uint32 Generation = 0xFFFFFFFFu;
	// GDF permutation 선택(뷰 확장 경로에서 solve-substep GDF 충돌)에 쓰는 플래그. 충돌 파라미터(반경/마찰)는
	// Params 버퍼(FRopeGPUParams)로 솔브 CS에 직접 전달하므로 여기 상주할 필요 없음.
	bool  bUseWorldGDF = false;
	FRHIGPUBufferReadback* PosReadback = nullptr;
	FRHIGPUBufferReadback* PrevReadback = nullptr;
	// 리드백 copy가 enqueue되어 결과 대기 중인가.
	bool bReadbackArmed = false;

	// 장력(λ) 리드백: 솔브(NumSub>0) 프레임에만 무장(override-only 프레임의 0을 안 내보내 직전 값 유지).
	// LambdaFixedDt = 무장 당시 substep dt — consume 시 F = max(0,-λ)/h² 변환에 쓴다.
	FRHIGPUBufferReadback* LambdaReadback = nullptr;
	bool  bLambdaArmed = false;
	float LambdaFixedDt = 0.0f;
	// M5b: PosBuf StructuredBuffer<float4> SRV(렌더용). 재시드 시 무효화.
	FShaderResourceViewRHIRef PosSRV;

	// SDF 볼륨 그리드/헤더 resident(정적 베이크 데이터 — 볼륨 집합이 바뀔 때만 재업로드). 인스턴스(본
	// 트랜스폼)는 매 프레임 작은 버퍼로 따로 올린다. 이로써 매 프레임 multi-MB 그리드 재업로드를 없앤다.
	TRefCountPtr<FRDGPooledBuffer> SDFDistBuf;
	TRefCountPtr<FRDGPooledBuffer> SDFVolBuf;
	// 볼륨 집합 시그니처(키+복셀수). 다르면 재빌드.
	uint32 SDFSetSig = 0;
	// VolumeKey -> SDFVol 인덱스(매 프레임 인스턴스 VolumeIndex 산정).
	TMap<const void*, int32> SDFVolKeyToIndex;

	// 접촉 감지(G3): 노드당 1슬롯 출력 버퍼(resident, N 변할 때만 재생성) + 리드백(위치와 같은 ring).
	TRefCountPtr<FRDGPooledBuffer> ContactBuf;
	FRHIGPUBufferReadback* ContactReadback = nullptr;
	bool bContactArmed = false;
};

// GT<->RT 공유 결과. RT가 채우고 GT GetLatest가 락 하에 읽는다.
struct FRopeResidentSharedResults
{
	FCriticalSection Lock;
	TMap<uint32, FRopeResidentLatest> Map;
	// G3: 접촉 감지 결과(GetLatestContacts).
	TMap<uint32, FRopeResidentContacts> Contacts;
};

// pimpl: 영속 버퍼 맵(RT 전용) + 공유 결과(GT<->RT). RDG/RHI 타입을 헤더에서 숨긴다.
struct FRopeGPUSolver::FImpl
{
	// 렌더 스레드에서만 접근.
	TMap<uint32, FRopeResidentRope>                          RtRopes;
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
				Impl->RtRopes.Remove(RopeId);
			}
		});
}

void FRopeGPUSolver::GetLatest(TMap<uint32, FRopeResidentLatest>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	// 작은 데이터 — 매 프레임 복사. (스왑 대신 복사로 호출자가 누적분 유지)
	Out = Impl->Results->Map;
}

void FRopeGPUSolver::GetLatestContacts(TMap<uint32, FRopeResidentContacts>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	// 노드당 최대 1건이라 작다 — 매 프레임 복사.
	Out = Impl->Results->Contacts;
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
	// RT 커맨드 완료까지 GT 대기(참조 캡처 안전 + 결과 확정).
	FlushRenderingCommands();
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

// ===== RunSteps_RenderThread 분해 =====================================================================
// 아래 헬퍼들은 RunSteps_RenderThread의 단계별 본체다(동작 동일 — 코드 이동). 업로드용 CPU 배열은 전부
// GraphBuilder.AllocObject<TArray<...>>()로 그래프 수명에 묶는다(Execute까지 생존 + 그래프 해체 시 소멸)
// — 함수 스코프 keep-alive 컨테이너(구 K* 배열)와 "Reserve로 재할당 방지" 규약이 필요 없다.

// 로프 1개의 그래프 빌드 중간 산출물(RDG 핸들 + 유효 개수). 오케스트레이션 루프가 단계 간에 전달한다.
struct FRopeStepBuild
{
	FRDGBufferRef PosRDG = nullptr;
	FRDGBufferRef PrevRDG = nullptr;
	FRDGBufferRef InvMassRDG = nullptr;
	// 이번 프레임 재시드(최초/노드수·generation 변화) 여부.
	bool bSeed = false;
	// G0 override 유효(플래그 길이 == 노드 수) 여부.
	bool bHasOverrides = false;

	FRDGBufferRef CapsulesBuf = nullptr;
	FRDGBufferRef SDFDistBuf = nullptr;
	FRDGBufferRef SDFVolBuf = nullptr;
	FRDGBufferRef SDFColBuf = nullptr;
	FRDGBufferRef BoxesBuf = nullptr;
	FRDGBufferRef ConvexBuf = nullptr;
	FRDGBufferRef ConvexPlanesBuf = nullptr;
	// 더미 패딩 *전* 유효 개수(셰이더 카운트용).
	int32 NumValidCaps = 0;
	int32 NumValidSDFCol = 0;
	int32 NumValidBoxes = 0;
	int32 NumValidConvexes = 0;

	FRDGBufferRef OvFlagsBuf = nullptr;
	FRDGBufferRef OvPosBuf = nullptr;
	FRDGBufferRef OvPrevBuf = nullptr;
	FRDGBufferRef OvInvBuf = nullptr;
};

// Loop 1: 직전 프레임 리드백 consume(immediate Lock — RDG 빌더 구성 *전*에 처리해 immediate RHI와
// 열린 그래프의 인터리브를 피한다. 렌더 스레드라 Lock 합법, IsReady 게이트라 stall 없음).
static void RopeConsumeReadbacks(TMap<uint32, FRopeResidentRope>& RtRopes,
	FRopeResidentSharedResults& Results, const TArray<FRopeGPUResidentStep>& Steps)
{
	for (const FRopeGPUResidentStep& S : Steps)
	{
		FRopeResidentRope* Rp = RtRopes.Find(S.RopeId);
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
			// 소비 완료 — dispatch 블록에서 재무장.
			R.bReadbackArmed = false;
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
			// 소비 완료 — dispatch 블록에서 재무장.
			R.bLambdaArmed = false;
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
					// w에 팩된 침투.
					C.Penetration     = Src[slot].WorldPoint.W;
					C.WorldPoint      = FVector(Src[slot].WorldPoint.X, Src[slot].WorldPoint.Y, Src[slot].WorldPoint.Z);
					C.Normal          = FVector(Src[slot].Normal.X, Src[slot].Normal.Y, Src[slot].Normal.Z);
					C.SurfaceVelocity = FVector(Src[slot].SurfaceVel.X, Src[slot].SurfaceVel.Y, Src[slot].SurfaceVel.Z);
					TmpContacts.Add(C);
				}
				R.ContactReadback->Unlock();
				bHaveContacts = true;
			}
			// 소비 완료 — dispatch 블록에서 재무장.
			R.bContactArmed = false;
		}

		if (!bHavePos && !bHaveContacts && !bHaveTension)
		{
			// 이번 프레임 회수분 없음.
			continue;
		}

		// 락 구간은 맵 대입만(리드백 Lock은 위에서 끝냄) → GT GetLatest 블로킹 최소화.
		FScopeLock SL(&Results.Lock);
		if (bHavePos || bHaveTension)
		{
			FRopeResidentLatest& L = Results.Map.FindOrAdd(S.RopeId);
			if (bHavePos)
			{
				L.Positions     = MoveTemp(TmpPos);
				L.PrevPositions = MoveTemp(TmpPrev);
				L.NumNodes      = N;
				// generation 승격은 위치와 함께만(재시드 직후 stale 위치 승격 방지).
				L.Generation    = R.Generation;
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
			FRopeResidentContacts& CL = Results.Contacts.FindOrAdd(S.RopeId);
			CL.Contacts   = MoveTemp(TmpContacts);
			CL.Generation = R.Generation;
		}
	}
}

// 상주 Pos/Prev/InvMass 확보: 재시드(최초/노드수·generation 변화)면 시드 업로드 + 외부 버퍼 변환,
// 아니면 기존 영속 버퍼를 그래프에 등록. B.PosRDG/PrevRDG/InvMassRDG/bSeed를 채운다.
static void RopeEnsureResidentBuffers(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	FRopeResidentRope& R, FRopeStepBuild& B)
{
	const int32 N = S.NumNodes;
	B.bSeed = !R.PosBuf.IsValid() || R.NumNodes != N || R.Generation != S.Generation;

	if (B.bSeed)
	{
		const bool bHaveSeed = S.SeedPositions.Num() == N && S.SeedPrevPositions.Num() == N && S.InvMass.Num() == N;
		TArray<FVector4f>& SeedPos  = *GraphBuilder.AllocObject<TArray<FVector4f>>();
		TArray<FVector4f>& SeedPrev = *GraphBuilder.AllocObject<TArray<FVector4f>>();
		TArray<float>&     SeedInv  = *GraphBuilder.AllocObject<TArray<float>>();
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
		B.PosRDG     = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Pos"),     sizeof(FVector4f), N, SeedPos.GetData(),  (uint64)N * sizeof(FVector4f));
		B.PrevRDG    = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Prev"),    sizeof(FVector4f), N, SeedPrev.GetData(), (uint64)N * sizeof(FVector4f));
		B.InvMassRDG = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.InvMass"), sizeof(float),     N, SeedInv.GetData(),  (uint64)N * sizeof(float));
		R.PosBuf     = GraphBuilder.ConvertToExternalBuffer(B.PosRDG);
		R.PrevBuf    = GraphBuilder.ConvertToExternalBuffer(B.PrevRDG);
		R.InvMassBuf = GraphBuilder.ConvertToExternalBuffer(B.InvMassRDG);
		R.NumNodes   = N;
		R.Generation = S.Generation;
		// 재시드 후 직전 리드백은 stale.
		R.bReadbackArmed = false;
		// PosBuf 새로 생성 → 캐시된 SRV 무효(렌더가 다음에 재생성).
		R.PosSRV.SafeRelease();
	}
	else
	{
		B.PosRDG     = GraphBuilder.RegisterExternalBuffer(R.PosBuf);
		B.PrevRDG    = GraphBuilder.RegisterExternalBuffer(R.PrevBuf);
		B.InvMassRDG = GraphBuilder.RegisterExternalBuffer(R.InvMassBuf);
	}
}

// 캡슐 패킹(M2): step의 월드 캡슐 → GPU 레이아웃 평탄화 + 업로드. B.CapsulesBuf/NumValidCaps를 채운다.
static void RopePackCapsules(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S, FRopeStepBuild& B)
{
	TArray<FRopeCapsuleGPU>& CapsFlat = *GraphBuilder.AllocObject<TArray<FRopeCapsuleGPU>>();
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

	// 유효 개수 — 더미 패딩 *전* 확정. 구조화 버퍼는 원소 >=1 — 비면 더미 1개(어느 노드도 참조 안 함).
	B.NumValidCaps = CapsFlat.Num();
	if (CapsFlat.Num() == 0) { CapsFlat.AddZeroed(1); }

	B.CapsulesBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Capsules"),
		sizeof(FRopeCapsuleGPU), CapsFlat.Num(), CapsFlat.GetData(), (uint64)CapsFlat.Num() * sizeof(FRopeCapsuleGPU));
}

// 박스 패킹: step의 정적 박스(OBB) → GPU 레이아웃 평탄화 + 업로드. B.BoxesBuf/NumValidBoxes를 채운다.
static void RopePackBoxes(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S, FRopeStepBuild& B)
{
	TArray<FRopeBoxGPU>& BoxesFlat = *GraphBuilder.AllocObject<TArray<FRopeBoxGPU>>();
	for (const FRopeGPUBox& Box : S.Boxes)
	{
		FRopeBoxGPU G;
		// w=InvDt
		G.Center      = FVector4f((float)Box.Center.X, (float)Box.Center.Y, (float)Box.Center.Z, Box.InvDeltaTime);
		G.Rot         = FVector4f((float)Box.Rot.X, (float)Box.Rot.Y, (float)Box.Rot.Z, (float)Box.Rot.W);
		G.HalfExtents = FVector4f((float)Box.HalfExtents.X, (float)Box.HalfExtents.Y, (float)Box.HalfExtents.Z, 0.0f);
		// 정적(InvDt 0)이면 prev=현재 — 커널이 prev 유효성 분기 없이 항상 보간 가능(캡슐 패킹과 동일).
		const bool bMoving = Box.InvDeltaTime > 0.0f;
		const FVector PC = bMoving ? Box.PrevCenter : Box.Center;
		const FQuat   PR = bMoving ? Box.PrevRot : Box.Rot;
		G.PrevCenter  = FVector4f((float)PC.X, (float)PC.Y, (float)PC.Z, 0.0f);
		G.PrevRot     = FVector4f((float)PR.X, (float)PR.Y, (float)PR.Z, (float)PR.W);
		BoxesFlat.Add(G);
	}

	// 유효 개수 — 더미 패딩 *전* 확정. 구조화 버퍼는 원소 >=1 — 비면 더미 1개(NumBoxes=0이라 미참조).
	B.NumValidBoxes = BoxesFlat.Num();
	if (BoxesFlat.Num() == 0) { BoxesFlat.AddZeroed(1); }

	B.BoxesBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Boxes"),
		sizeof(FRopeBoxGPU), BoxesFlat.Num(), BoxesFlat.GetData(), (uint64)BoxesFlat.Num() * sizeof(FRopeBoxGPU));
}

// 컨벡스 패킹: step의 정적 컨벡스 → 평면 평탄 풀(ConvexPlanes) + 헤더(Convexes) 업로드. 각 컨벡스의 평면을
// 풀에 이어붙이고 PlaneOffset/PlaneCount로 참조한다. B.ConvexBuf/ConvexPlanesBuf/NumValidConvexes를 채운다.
static void RopePackConvexes(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S, FRopeStepBuild& B)
{
	TArray<FRopeConvexGPU>& ConvFlat = *GraphBuilder.AllocObject<TArray<FRopeConvexGPU>>();
	TArray<FVector4f>&      PlaneFlat = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	for (const FRopeGPUConvex& Cv : S.Convexes)
	{
		// 정적(InvDt 0)이면 prev=현재 — 커널이 prev 유효성 분기 없이 항상 보간 가능(박스/캡슐 패킹과 동일).
		const bool bMoving = Cv.InvDeltaTime > 0.0f;
		const FQuat   PR = bMoving ? Cv.PrevRot : Cv.Rot;
		const FVector PT = bMoving ? Cv.PrevTrans : Cv.Trans;
		FRopeConvexGPU G;
		G.PlaneOffset       = PlaneFlat.Num();
		G.PlaneCount        = Cv.PlaneCount;
		G.LocalBoundsCenter = FVector4f((float)Cv.LocalBoundsCenter.X, (float)Cv.LocalBoundsCenter.Y, (float)Cv.LocalBoundsCenter.Z, 0.0f);
		// w=InvDt
		G.LocalBoundsExtent = FVector4f((float)Cv.LocalBoundsExtent.X, (float)Cv.LocalBoundsExtent.Y, (float)Cv.LocalBoundsExtent.Z, Cv.InvDeltaTime);
		G.Rot               = FVector4f((float)Cv.Rot.X, (float)Cv.Rot.Y, (float)Cv.Rot.Z, (float)Cv.Rot.W);
		G.Trans             = FVector4f((float)Cv.Trans.X, (float)Cv.Trans.Y, (float)Cv.Trans.Z, 0.0f);
		G.PrevRot           = FVector4f((float)PR.X, (float)PR.Y, (float)PR.Z, (float)PR.W);
		G.PrevTrans         = FVector4f((float)PT.X, (float)PT.Y, (float)PT.Z, 0.0f);
		ConvFlat.Add(G);
		const int32 Start = Cv.PlaneOffset;
		for (int32 p = 0; p < Cv.PlaneCount; ++p)
		{
			const FVector4& Pl = S.ConvexPlanes[Start + p];
			PlaneFlat.Add(FVector4f((float)Pl.X, (float)Pl.Y, (float)Pl.Z, (float)Pl.W));
		}
	}

	// 유효 개수 — 더미 패딩 *전* 확정. 구조화 버퍼는 원소 >=1 — 비면 더미 1개(NumConvexes=0이라 미참조).
	B.NumValidConvexes = ConvFlat.Num();
	if (ConvFlat.Num() == 0) { ConvFlat.AddZeroed(1); }
	if (PlaneFlat.Num() == 0) { PlaneFlat.AddZeroed(1); }

	B.ConvexBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Convexes"),
		sizeof(FRopeConvexGPU), ConvFlat.Num(), ConvFlat.GetData(), (uint64)ConvFlat.Num() * sizeof(FRopeConvexGPU));
	B.ConvexPlanesBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.ConvexPlanes"),
		sizeof(FVector4f), PlaneFlat.Num(), PlaneFlat.GetData(), (uint64)PlaneFlat.Num() * sizeof(FVector4f));
}

// SDF 콜라이더 패킹(M3): 볼륨 dedup + 상주 재사용(집합 시그니처 동일 시 업로드 0) 또는 dequant 재빌드
// + 인스턴스(본 트랜스폼, 매 프레임) 업로드. 볼륨 dedup 맵(FreshKeyToIndex)을 인스턴스 루프가 참조하므로
// 두 단계는 반드시 한 스코프에 있어야 한다(분리 금지 — dangling). B.SDF*Buf/NumValidSDFCol을 채운다.
static void RopePackSDFColliders(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	FRopeResidentRope& R, FRopeStepBuild& B)
{
	TArray<FRopeSDFColliderGPU>& SDFCol = *GraphBuilder.AllocObject<TArray<FRopeSDFColliderGPU>>();

	// 1) 고유 볼륨 dedup + 집합 시그니처(키/복셀수만 — distance 데이터는 만지지 않는다).
	// 인덱스 = (재빌드 시) VolumeIndex
	TArray<const FRopeGPUSDFCollider*> UniqueVols;
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

	if (bVolReuse)
	{
		B.SDFDistBuf = GraphBuilder.RegisterExternalBuffer(R.SDFDistBuf);
		B.SDFVolBuf  = GraphBuilder.RegisterExternalBuffer(R.SDFVolBuf);
	}
	else if (UniqueVols.Num() > 0)
	{
		TArray<float>&             SDFDist = *GraphBuilder.AllocObject<TArray<float>>();
		TArray<FRopeSDFVolumeGPU>& SDFVol  = *GraphBuilder.AllocObject<TArray<FRopeSDFVolumeGPU>>();
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
				// 바깥 +
				SDFDist.Add(static_cast<float>(Code) * DeqScale - NBIn);
			}
		}
		B.SDFDistBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFDistances"),
			sizeof(float), SDFDist.Num(), SDFDist.GetData(), (uint64)SDFDist.Num() * sizeof(float));
		B.SDFVolBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFVolumes"),
			sizeof(FRopeSDFVolumeGPU), SDFVol.Num(), SDFVol.GetData(), (uint64)SDFVol.Num() * sizeof(FRopeSDFVolumeGPU));
		R.SDFDistBuf = GraphBuilder.ConvertToExternalBuffer(B.SDFDistBuf);
		R.SDFVolBuf  = GraphBuilder.ConvertToExternalBuffer(B.SDFVolBuf);
		R.SDFSetSig  = VolSig;
		// 복사(아래 인스턴스 루프가 KeyToIndex=FreshKeyToIndex를 계속 참조).
		R.SDFVolKeyToIndex = FreshKeyToIndex;
	}

	// 3) 인스턴스(매 프레임): VolumeIndex(캐시/신규 맵) + 현재 본 트랜스폼.
	for (const FRopeGPUSDFCollider& Src : S.SDFColliders)
	{
		const int32* VolIdx = KeyToIndex.Find(Src.VolumeKey);
		if (!VolIdx)
		{
			// 무효 볼륨(위 dedup 조건과 일치).
			continue;
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
		// w=InvDt
		C.PrevTranslation = FVector4f((float)PT.X, (float)PT.Y, (float)PT.Z, Src.InvDeltaTime);
		SDFCol.Add(C);
	}

	// 유효 개수 — 더미 패딩 *전* 확정. 비면 더미 1개(셰이더는 NumSDFColliders=0이라 미참조).
	B.NumValidSDFCol = SDFCol.Num();
	if (SDFCol.Num() == 0) { SDFCol.AddZeroed(1); }

	// 이 로프에 SDF 볼륨이 없으면 distance/header도 더미 1개 transient 생성.
	if (!B.SDFDistBuf)
	{
		TArray<float>&             DummyDist = *GraphBuilder.AllocObject<TArray<float>>();
		TArray<FRopeSDFVolumeGPU>& DummyVol  = *GraphBuilder.AllocObject<TArray<FRopeSDFVolumeGPU>>();
		DummyDist.AddZeroed(1);
		DummyVol.AddZeroed(1);
		B.SDFDistBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFDistances.Dummy"),
			sizeof(float), 1, DummyDist.GetData(), sizeof(float));
		B.SDFVolBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFVolumes.Dummy"),
			sizeof(FRopeSDFVolumeGPU), 1, DummyVol.GetData(), sizeof(FRopeSDFVolumeGPU));
	}

	B.SDFColBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.SDFColliders"),
		sizeof(FRopeSDFColliderGPU), SDFCol.Num(), SDFCol.GetData(), (uint64)SDFCol.Num() * sizeof(FRopeSDFColliderGPU));
}

// Override(G0) 업로드: 노드별 플래그/타깃/질량(transient, 오버라이드 프레임만 실데이터).
// 없으면 더미 1개 + bHasOverrides=0 → 셰이더가 참조하지 않는다.
static void RopePackOverrides(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S, FRopeStepBuild& B)
{
	const int32 N = S.NumNodes;
	TArray<uint32>&    OvFlags = *GraphBuilder.AllocObject<TArray<uint32>>();
	TArray<FVector4f>& OvPos   = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	TArray<FVector4f>& OvPrev  = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	TArray<float>&     OvInv   = *GraphBuilder.AllocObject<TArray<float>>();
	if (B.bHasOverrides)
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
	B.OvFlagsBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.OverrideFlags"),
		sizeof(uint32), OvFlags.Num(), OvFlags.GetData(), (uint64)OvFlags.Num() * sizeof(uint32));
	B.OvPosBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.OverridePositions"),
		sizeof(FVector4f), OvPos.Num(), OvPos.GetData(), (uint64)OvPos.Num() * sizeof(FVector4f));
	B.OvPrevBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.OverridePrevPositions"),
		sizeof(FVector4f), OvPrev.Num(), OvPrev.GetData(), (uint64)OvPrev.Num() * sizeof(FVector4f));
	B.OvInvBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.OverrideInvMass"),
		sizeof(float), OvInv.Num(), OvInv.GetData(), (uint64)OvInv.Num() * sizeof(float));
}

// 솔브 패스: 파라미터 버퍼 구성 + XPBD CS dispatch(GDF permutation은 로프 단위 선택).
// 장력(λ) 출력 버퍼(프레임 transient)를 만들어 반환한다 — 리드백 무장(RopeArmReadbacks)이 소비.
static FRDGBufferRef RopeAddSolvePass(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	const FRopeResidentRope& R, const FRopeStepBuild& B, const FSceneView* View,
	const FGlobalDistanceFieldParameters2& GDFSolverParams, uint32 bGDFSolverValid,
	const FVector3f& PreViewTranslation)
{
	const int32 N = S.NumNodes;

	TArray<FRopeGPUParamsGPU>& ParamsArr = *GraphBuilder.AllocObject<TArray<FRopeGPUParamsGPU>>();
	FRopeGPUParamsGPU P;
	P.NodeOffset        = 0;
	P.NumNodes          = N;
	P.NumSub            = S.NumSub;
	P.Iters             = FMath::Max(1, S.Iterations);
	P.FixedDt           = S.FixedDt;
	P.SegmentLength     = S.SegmentLength;
	P.StretchCompliance = S.StretchCompliance;
	P.MaxStretchRatio   = S.MaxStretchRatio;
	P.BendCompliance    = S.BendCompliance;
	P.BendReleaseRatio  = S.BendReleaseRatio;
	P.BendFullRatio     = S.BendFullRatio;
	P.Damping           = S.Damping;
	P.bStartPinned      = S.bStartPinned ? 1 : 0;
	P.CapsuleOffset     = 0;
	// collision-free Aim Flight는 solve 커널의 형상 개수만 0으로 만든다. 업로드된 버퍼는 detect 커널이 계속 사용한다.
	P.NumCapsules       = S.bSolveCollisions ? B.NumValidCaps : 0;
	P.CollisionRadius   = S.CollisionRadius;
	P.Friction          = S.Friction;
	P.TipFrictionScale  = S.TipFrictionScale;
	P.CollisionPasses   = FMath::Clamp(S.CollisionPasses, 1, FMath::Max(1, S.Iterations));
	P.SweepStep         = S.SweepStep;
	P.MaxSweepSamples   = FMath::Max(1, S.MaxSweepSamples);
	P.SDFColliderOffset = 0;
	P.NumSDFColliders   = S.bSolveCollisions ? B.NumValidSDFCol : 0;
	P.bHasOverrides     = B.bHasOverrides ? 1 : 0;
	P.NumBoxes          = S.bSolveCollisions ? B.NumValidBoxes : 0;
	P.NumConvexes       = S.bSolveCollisions ? B.NumValidConvexes : 0;
	P.Gravity           = FVector4f((float)S.Gravity.X, (float)S.Gravity.Y, (float)S.Gravity.Z, 0.0f);
	P.PinPrev           = FVector4f((float)S.StartPinPrev.X,   (float)S.StartPinPrev.Y,   (float)S.StartPinPrev.Z,   0.0f);
	P.PinTarget         = FVector4f((float)S.StartPinTarget.X, (float)S.StartPinTarget.Y, (float)S.StartPinTarget.Z, 0.0f);
	ParamsArr.Add(P);

	FRDGBufferRef ParamsBuf = CreateStructuredBuffer(GraphBuilder, TEXT("Rope.Params"),
		sizeof(FRopeGPUParamsGPU), 1, ParamsArr.GetData(), sizeof(FRopeGPUParamsGPU));

	FRopeXPBDSolveCS::FParameters* PassParams = GraphBuilder.AllocParameters<FRopeXPBDSolveCS::FParameters>();
	PassParams->NumRopes      = 1;
	PassParams->Params        = GraphBuilder.CreateSRV(ParamsBuf);
	PassParams->Capsules      = GraphBuilder.CreateSRV(B.CapsulesBuf);
	PassParams->SDFDistances  = GraphBuilder.CreateSRV(B.SDFDistBuf);
	PassParams->SDFVolumes    = GraphBuilder.CreateSRV(B.SDFVolBuf);
	PassParams->SDFColliders  = GraphBuilder.CreateSRV(B.SDFColBuf);
	PassParams->Boxes         = GraphBuilder.CreateSRV(B.BoxesBuf);
	PassParams->Convexes      = GraphBuilder.CreateSRV(B.ConvexBuf);
	PassParams->ConvexPlanes  = GraphBuilder.CreateSRV(B.ConvexPlanesBuf);
	PassParams->OverrideFlags         = GraphBuilder.CreateSRV(B.OvFlagsBuf);
	PassParams->OverridePositions     = GraphBuilder.CreateSRV(B.OvPosBuf);
	PassParams->OverridePrevPositions = GraphBuilder.CreateSRV(B.OvPrevBuf);
	PassParams->OverrideInvMass       = GraphBuilder.CreateSRV(B.OvInvBuf);
	PassParams->InvMass       = GraphBuilder.CreateUAV(B.InvMassRDG);
	PassParams->Positions     = GraphBuilder.CreateUAV(B.PosRDG);
	PassParams->PrevPositions = GraphBuilder.CreateUAV(B.PrevRDG);
	// 장력(λ) 출력: 프레임 transient(N 슬롯, 커널이 매 dispatch 전체를 다시 쓴다 — 영속 불필요).
	FRDGBufferRef LambdaRDG = GraphBuilder.CreateBuffer(
		FRDGBufferDesc::CreateStructuredDesc(sizeof(float), N), TEXT("Rope.LambdaDist"));
	PassParams->OutLambdaDist = GraphBuilder.CreateUAV(LambdaRDG);

	// GDF in-solver(Phase 3): bUseWorldGDF 로프 + GDF 유효 시 GDF permutation 선택 + View/GDF 바인딩.
	// 아니면 lean(기존 동작). 로프당 개별 AddPass라 permutation을 로프 단위로 자유 선택한다.
	const bool bUseGDFPerm = (View != nullptr) && R.bUseWorldGDF && (bGDFSolverValid != 0);
	if (bUseGDFPerm)
	{
		PassParams->View                  = View->ViewUniformBuffer;
		PassParams->GDF                   = GDFSolverParams;
		PassParams->GDFPreViewTranslation = PreViewTranslation;
		PassParams->bWorldGDFValid        = bGDFSolverValid;
	}
	FRopeXPBDSolveCS::FPermutationDomain PermVec;
	// N ≤ MaxNodes(호출부 게이트) → 항상 ≥64.
	PermVec.Set<FRopeXPBDSolveCS::FNodeBucket>(RopeNodeBucket(N));
	PermVec.Set<FRopeXPBDSolveCS::FGDFDim>(bUseGDFPerm);
	TShaderMapRef<FRopeXPBDSolveCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), PermVec);
	FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeXPBDResident"),
		// 로프 1개 = 스레드그룹 1개
		ComputeShader, PassParams, FIntVector(1, 1, 1));

	return LambdaRDG;
}

// 리드백 재무장: in-flight가 없을 때만(이번 프레임 stepped 위치를 비동기 copy). consume은 RopeConsumeReadbacks.
static void RopeArmReadbacks(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	FRopeResidentRope& R, const FRopeStepBuild& B, FRDGBufferRef LambdaRDG)
{
	const int32 N = S.NumNodes;
	if (!R.bReadbackArmed)
	{
		if (!R.PosReadback)  { R.PosReadback  = new FRHIGPUBufferReadback(TEXT("Rope.PosReadback")); }
		if (!R.PrevReadback) { R.PrevReadback = new FRHIGPUBufferReadback(TEXT("Rope.PrevReadback")); }
		const uint32 NodeBytes = (uint32)N * sizeof(FVector4f);
		AddEnqueueCopyPass(GraphBuilder, R.PosReadback,  B.PosRDG,  NodeBytes);
		AddEnqueueCopyPass(GraphBuilder, R.PrevReadback, B.PrevRDG, NodeBytes);
		R.bReadbackArmed = true;
	}

	// 장력(λ) 리드백 무장: 솔브 프레임(NumSub>0)에만 — override-only 프레임은 λ가 0이라
	// 무장하지 않고 직전 장력을 유지한다(GT는 갱신분이 있을 때만 덮어씀).
	if (!R.bLambdaArmed && S.NumSub > 0)
	{
		if (!R.LambdaReadback) { R.LambdaReadback = new FRHIGPUBufferReadback(TEXT("Rope.LambdaReadback")); }
		AddEnqueueCopyPass(GraphBuilder, R.LambdaReadback, LambdaRDG, (uint32)N * sizeof(float));
		R.LambdaFixedDt = S.FixedDt;
		R.bLambdaArmed = true;
	}
}

// 접촉 감지(G3): 솔브 뒤 post-solve 위치를 스윕. RDG가 solve(UAV)→detect(SRV) 순서를 보장한다.
// 노드당 2슬롯(actual+predictive) 출력. 감지가 있을 때만 ContactBuf(resident, 2N슬롯) 확보.
static void RopeAddDetectPass(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	FRopeResidentRope& R, const FRopeStepBuild& B)
{
	const int32 N = S.NumNodes;

	const bool bContactSeed = !R.ContactBuf.IsValid() || B.bSeed;
	FRDGBufferRef ContactRDG = nullptr;
	if (bContactSeed)
	{
		ContactRDG = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(FRopeGPUContactGPU), 2 * N), TEXT("Rope.Contacts"));
		R.ContactBuf = GraphBuilder.ConvertToExternalBuffer(ContactRDG);
		// 재생성 → 직전 접촉 리드백은 stale.
		R.bContactArmed = false;
	}
	else
	{
		ContactRDG = GraphBuilder.RegisterExternalBuffer(R.ContactBuf);
	}

	// 예측 접촉용 whip 가이드 버퍼(G3b). whip 활성 시 노드별 마스크/타깃, 아니면 더미 1개.
	const bool bHasWhip = S.WhipGuidedMask.Num() == N && S.PredictionFrames > 0.0f;
	TArray<uint32>&    GMask = *GraphBuilder.AllocObject<TArray<uint32>>();
	TArray<FVector4f>& WCur  = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	TArray<FVector4f>& WPrev = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	TArray<FVector4f>& WNext = *GraphBuilder.AllocObject<TArray<FVector4f>>();
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
	// 감지는 비-정적 캡슐만: 호출자가 2-pass 패킹으로 정적(월드) 캡슐을 뒤에 붙이고 경계를
	// NumDetectCapsules로 알린다(-1=전부, 기존 동작). 정적 접촉이 최심-1건 슬롯에서 본 접촉을
	// 가리는 것을 막는다. 박스는 아예 감지 커널에 없다(같은 이유).
	DetectParams->DetectNumCapsules    = (S.NumDetectCapsules >= 0)
		? FMath::Min(S.NumDetectCapsules, B.NumValidCaps) : B.NumValidCaps;
	DetectParams->DetectNumSDF         = B.NumValidSDFCol;
	// 랩 가능 박스만 감지(정적 박스는 뒤라 제외). 박스도 노드당 최심 접촉 슬롯을 캡슐/SDF와 공유한다.
	DetectParams->DetectNumBoxes       = FMath::Clamp(S.NumDetectBoxes, 0, B.NumValidBoxes);
	DetectParams->DetectContactRadius  = S.ContactRadius;
	DetectParams->DetectSegmentLength  = S.SegmentLength;
	DetectParams->DetectPredictionFrames = FMath::Max(0.0f, S.PredictionFrames);
	DetectParams->DetectHasGuidedNodes = bHasWhip ? 1 : 0;
	DetectParams->Capsules             = GraphBuilder.CreateSRV(B.CapsulesBuf);
	DetectParams->SDFDistances         = GraphBuilder.CreateSRV(B.SDFDistBuf);
	DetectParams->SDFVolumes           = GraphBuilder.CreateSRV(B.SDFVolBuf);
	DetectParams->SDFColliders         = GraphBuilder.CreateSRV(B.SDFColBuf);
	DetectParams->Boxes                = GraphBuilder.CreateSRV(B.BoxesBuf);
	DetectParams->DetectPositions      = GraphBuilder.CreateSRV(B.PosRDG);
	DetectParams->DetectPrevPositions  = GraphBuilder.CreateSRV(B.PrevRDG);
	DetectParams->DetectGuidedMask     = GraphBuilder.CreateSRV(GMaskBuf);
	DetectParams->DetectWhipCur        = GraphBuilder.CreateSRV(WCurBuf);
	DetectParams->DetectWhipPrev       = GraphBuilder.CreateSRV(WPrevBuf);
	DetectParams->DetectWhipNext       = GraphBuilder.CreateSRV(WNextBuf);
	DetectParams->OutContacts          = GraphBuilder.CreateUAV(ContactRDG);

	FRopeContactDetectCS::FPermutationDomain DetectPerm;
	// N ≤ MaxNodes → 항상 ≥64.
	DetectPerm.Set<FRopeContactDetectCS::FNodeBucket>(RopeNodeBucket(N));
	TShaderMapRef<FRopeContactDetectCS> DetectShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), DetectPerm);
	FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeContactDetect"),
		DetectShader, DetectParams, FIntVector(1, 1, 1));

	if (!R.bContactArmed)
	{
		if (!R.ContactReadback) { R.ContactReadback = new FRHIGPUBufferReadback(TEXT("Rope.ContactReadback")); }
		AddEnqueueCopyPass(GraphBuilder, R.ContactReadback, ContactRDG, (uint32)(2 * N) * sizeof(FRopeGPUContactGPU));
		R.bContactArmed = true;
	}
}

// 상주 step들의 공용 실행부(RT). 전용 그래프(Step)든 씬 렌더러 그래프(DispatchPending_RenderThread)든
// 동일 본체를 전달받은 GraphBuilder에 얹는다(Execute는 호출자). GDF/PreViewTranslation은 GDF 월드 충돌(Phase 2c)에서 사용.
// 단계 본체는 위 헬퍼들(RopeConsumeReadbacks/RopeEnsureResidentBuffers/RopePack*/RopeAdd*Pass/RopeArmReadbacks)로
// 분해했고, 여기는 오케스트레이션만 남긴다 — early-out과 외부(SRV) 배리어 계약이 이 함수에서 한눈에 보인다.
void FRopeGPUSolver::RunSteps_RenderThread(FRDGBuilder& GraphBuilder, TArray<FRopeGPUResidentStep>& Steps,
	const FSceneView* View, const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation)
{
	// --- Loop 1: 직전 프레임 리드백 consume — 그래프 구성 *전*에 immediate Lock으로 처리.
	RopeConsumeReadbacks(Impl->RtRopes, *Impl->Results, Steps);

	// GDF in-solver: View가 있으면(=뷰 확장 경로) GDF permutation으로 솔브해 매 substep 벽을 투영한다.
	// 프레임 공용 GDF 셰이더 파라미터를 1회 산정(미빌드면 bGDFSolverValid=0 → 로프별 lean 폴백).
	// View 없는 Step 경로에선 GDF 미사용(정적 월드 충돌은 뷰 확장 경로 전용).
	const bool bGDFInSolver = (View != nullptr);
	FGlobalDistanceFieldParameters2 GDFSolverParams;
	uint32 bGDFSolverValid = 0;
	if (bGDFInSolver)
	{
		FillGDFShaderParams(GDF, GDFSolverParams, bGDFSolverValid);
	}

	// --- Loop 2: 로프별 seed/register → 패킹 → 솔브 → 리드백 재무장 → 감지 → 외부(SRV) 배리어.
	for (const FRopeGPUResidentStep& S : Steps)
	{
		const int32 N = S.NumNodes;
		if (N < 2 || N > FRopeGPUSolver::MaxNodes)
		{
			UE_LOG(LogDynamicRopeGPU, Warning, TEXT("GPU resident step skipped: %d nodes out of [2, %d]."), N, FRopeGPUSolver::MaxNodes);
			continue;
		}

		FRopeResidentRope& R = Impl->RtRopes.FindOrAdd(S.RopeId);
		// GDF permutation 선택에 쓰는 플래그를 상주 상태에 기록(충돌 반경/마찰은 Params 버퍼로 CS에 직접 전달).
		// Aim Flight에서 충돌 solve를 끌 때 GDF push-out도 함께 끄며, 별도 detect 커널에는 영향을 주지 않는다.
		R.bUseWorldGDF = S.bSolveCollisions && S.bUseWorldGDF;

		FRopeStepBuild B;
		RopeEnsureResidentBuffers(GraphBuilder, S, R, B);

		// G0: 오버라이드는 적분 없이도(NumSub=0) 기록해야 한다 — 로직 페이즈 프레임(Wrapping/Releasing 등).
		B.bHasOverrides = S.HasOverrides() && S.OverrideFlags.Num() == N;
		if (S.HasOverrides() && !B.bHasOverrides)
		{
			UE_LOG(LogDynamicRopeGPU, Warning, TEXT("GPU override ignored: flags %d != nodes %d."),
				S.OverrideFlags.Num(), N);
		}

		if (S.NumSub <= 0 && !B.bHasOverrides && !S.bDetectContacts)
		{
			// 이번 프레임 적분/기록/감지 없음 — 위치 불변, 리드백도 그대로 둠. 상태만 외부 읽기(SRV)로
			// 확정한다(시드 업로드 직후 조기 종료 프레임 포함) — 아래 dispatch 경로의 호출과 동일 목적.
			GraphBuilder.UseExternalAccessMode(B.PosRDG, ERHIAccess::SRVMask);
			continue;
		}

		RopePackCapsules(GraphBuilder, S, B);
		RopePackSDFColliders(GraphBuilder, S, R, B);
		RopePackBoxes(GraphBuilder, S, B);
		RopePackConvexes(GraphBuilder, S, B);
		RopePackOverrides(GraphBuilder, S, B);

		const FRDGBufferRef LambdaRDG = RopeAddSolvePass(GraphBuilder, S, R, B,
			bGDFInSolver ? View : nullptr, GDFSolverParams, bGDFSolverValid, PreViewTranslation);
		RopeArmReadbacks(GraphBuilder, S, R, B, LambdaRDG);

		if (S.bDetectContacts)
		{
			RopeAddDetectPass(GraphBuilder, S, R, B);
		}

		// 렌더 raw 튜브 경로(M5b/B2)가 이 그래프 *밖에서* PosBuf를 SRV로 직독한다 — 외부 접근 모드로
		// 마지막 패스(solve/copy/detect) 뒤 SRV 전이(배리어)를 매 프레임 확정한다. 이게 없으면 그래프
		// 종료 상태가 리드백 copy 유무에 따라 UAVCompute/CopySrc로 오락가락해, 배리어 없는 프레임에
		// 튜브가 이전/미완성 위치를 읽어 wrap 노드가 떨린다(CL167 회귀). GDF 충돌은 이제 솔브 CS 안(substep
		// 제약)에서 처리하므로 별도 post-solve 쓰기가 없고, 이 solve 패스가 PosBuf의 마지막 쓰기다.
		GraphBuilder.UseExternalAccessMode(B.PosRDG, ERHIAccess::SRVMask);
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
			// 전용 그래프 경로 — View/GDF 없음(GDF in-solver는 뷰 확장 경로 전용). View=nullptr → 항상 lean.
			RunSteps_RenderThread(GraphBuilder, Steps, nullptr, nullptr, FVector3f::ZeroVector);
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

void FRopeGPUSolver::DispatchPending_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView* View,
	const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation)
{
	check(IsInRenderingThread());
	if (Impl->PendingSteps.Num() == 0)
	{
		return;
	}
	RunSteps_RenderThread(GraphBuilder, Impl->PendingSteps, View, GDF, PreViewTranslation);
	Impl->PendingSteps.Reset();
}
