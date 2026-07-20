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
// TRACE_CPUPROFILER_EVENT_SCOPE — 렌더 스레드 dispatch 경로 실측(Unreal Insights CPU 타임라인).
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "ProfilingDebugging/RealtimeGPUProfiler.h"
#include "Stats/Stats.h"

// 'stat DynamicRopeGPU' RT 타이밍/메모리/대역폭 — 그룹 선언과 분리 이유는 RopeGPUStatGroup.h 참조(런타임 GT
// 대시보드 'stat DynamicRope'와 별개 그룹). 아래 CYCLE stat은 각 RopeRT_* 스코프(SCOPE_CYCLE_COUNTER)에서 RT
// 스레드 시간을 잡는다 — GPU 타임라인 시간이 아니라 RT CPU 시간이다. STATS 꺼진 빌드에선 매크로가 자동 no-op.
// RunSteps=RT 총량, 나머지는 그 하위 분해(PackSDF=SDF 재업로드 병목 지목용 — memory: sdf-global-volume-cache).
// HUD에는 프레임 예산에 실제로 잡히는 단계만 둔다 — 서브밀리초 부기(EnsureBuffers/Pack Boxes·Convexes·
// Overrides/Arm Readbacks)는 RopeRT_* Insights 스코프에 그대로 남아 있으니 필요할 때 거기서 본다.
#include "RopeGPUStatGroup.h"
DECLARE_CYCLE_STAT(TEXT("GPU RunSteps (RT total)"), STAT_RopeGPU_RunSteps, STATGROUP_DynamicRopeGPU);
DECLARE_CYCLE_STAT(TEXT("GPU Pack Capsules"), STAT_RopeGPU_PackCapsules, STATGROUP_DynamicRopeGPU);
DECLARE_CYCLE_STAT(TEXT("GPU Ensure Global SDF"), STAT_RopeGPU_EnsureGlobalSDF, STATGROUP_DynamicRopeGPU);
DECLARE_CYCLE_STAT(TEXT("GPU Pack SDF"), STAT_RopeGPU_PackSDF, STATGROUP_DynamicRopeGPU);
DECLARE_CYCLE_STAT(TEXT("GPU Add Solve Pass"), STAT_RopeGPU_AddSolvePass, STATGROUP_DynamicRopeGPU);
DECLARE_CYCLE_STAT(TEXT("GPU Add Detect Pass"), STAT_RopeGPU_AddDetectPass, STATGROUP_DynamicRopeGPU);
DECLARE_CYCLE_STAT(TEXT("GPU Graph Execute"), STAT_RopeGPU_GraphExecute, STATGROUP_DynamicRopeGPU);
DECLARE_CYCLE_STAT(TEXT("GPU Consume Readbacks"), STAT_RopeGPU_ConsumeReadbacks, STATGROUP_DynamicRopeGPU);
DECLARE_CYCLE_STAT(TEXT("GPU Dispatch Pending"), STAT_RopeGPU_DispatchPending, STATGROUP_DynamicRopeGPU);
// GPU 상주 VRAM(프레임 간 유지되는 영속 버퍼) — 로프별 Pos/Prev/InvMass/Contact + 전역 SDF 캐시(Dist/Vol).
// RunSteps 끝에서 GetSize()(바이트) 합산해 SET. 전송 대역폭이 아니라 상주 풋프린트다(SDF는 캐시 크기).
DECLARE_MEMORY_STAT(TEXT("GPU Mem: Rope Buffers"), STAT_RopeGPU_MemRopes, STATGROUP_DynamicRopeGPU);
DECLARE_MEMORY_STAT(TEXT("GPU Mem: Global SDF"), STAT_RopeGPU_MemGlobalSDF, STATGROUP_DynamicRopeGPU);
DECLARE_MEMORY_STAT(TEXT("GPU Mem: Resident Total"), STAT_RopeGPU_MemTotal, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Resident Ropes (count)"), STAT_RopeGPU_ResidentRopeCount, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU SDF Volumes"), STAT_RopeGPU_SDFVolumes, STATGROUP_DynamicRopeGPU);
// 프레임별 GPU 업로드 대역폭(실제 전송 바이트) — CreateStructuredBuffer 업로드를 RopeUploadBuffer로 감싸 누산.
// 상주 풋프린트(위 GPU Mem)와 달리 매 프레임 GPU로 올리는 양이다. 분해는 두 축만 남긴다: SDF(=CL389 캐시가
// 죽으면 매 프레임 MB 단위로 튄다 — 회귀 감시용)와 Colliders(콘텐츠 규모에 비례해 자라는 유일한 축). 나머지
// (Detect/Override/Seed)는 작고 산발적이라 total에 묻어 둔다.
DECLARE_MEMORY_STAT(TEXT("GPU Upload/Frame (total)"), STAT_RopeGPU_UploadTotal, STATGROUP_DynamicRopeGPU);
DECLARE_MEMORY_STAT(TEXT("GPU Upload/Frame (SDF Volume)"), STAT_RopeGPU_UploadSDF, STATGROUP_DynamicRopeGPU);
DECLARE_MEMORY_STAT(TEXT("GPU Upload/Frame (Colliders)"), STAT_RopeGPU_UploadColliders, STATGROUP_DynamicRopeGPU);
// GPU→CPU 다운로드 대역폭 + 프레임당 컴퓨트 dispatch/substep 수(솔버 작업량).
DECLARE_MEMORY_STAT(TEXT("GPU Readback/Frame (download)"), STAT_RopeGPU_ReadbackBytes, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Dispatches/Frame"), STAT_RopeGPU_Dispatches, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Substeps/Frame"), STAT_RopeGPU_Substeps, STATGROUP_DynamicRopeGPU);

// ── GPU 타임라인 stat — 위 CYCLE stat들과 재는 대상이 다르다 ────────────────────────────────────────
// 위쪽 'GPU *' CYCLE stat은 전부 **RT CPU 시간**(그래프를 짜는 데 든 시간)이다. 아래 두 개는 GPU가 실제로
// 커널을 돌린 시간으로, 엔진 GPU 그룹에 들어가 'stat gpu' / GPU Visualizer(ProfileGPU) / Insights GPU 트랙에
// 뜬다 — 이 그룹('stat DynamicRopeGPU')에는 안 나온다. 로프가 프레임 예산에서 몇 ms를 먹는지는 이쪽 숫자다.
//
// [매크로 선택 — 버전 계약] UE 5.7은 RHI_NEW_GPU_PROFILER=1이라 구형 RDG_GPU_STAT_SCOPE/SCOPED_GPU_STAT이
// **조용히 no-op**이 된다(5.8은 아예 deprecated). 반면 RDG_EVENT_SCOPE_STAT(RDG 경로)와
// RHI_BREADCRUMB_EVENT_STAT(즉시 RHI 경로)은 이 플러그인이 지원하는 5.5~5.8 전 버전에서 stat id를 실어
// 나른다. 그래서 RopeRHICompat 게이팅 없이 이 두 형태만 쓴다 — 새 GPU 스코프를 넣을 때도 이걸 따르라.
DECLARE_GPU_STAT_NAMED(RopeGPUSolve, TEXT("DynamicRope Solve"));
DECLARE_GPU_STAT_NAMED(RopeGPUDetect, TEXT("DynamicRope Detect"));

// RT 전용 프레임 업로드 누산기(RunSteps 시작에서 리셋, 끝에서 SET). RunSteps는 프레임당 1회 실행.
#if STATS
// 프레임 업로드 누산기(RunSteps 시작 리셋, 끝 SET). 카테고리별로 쪼개 MB 총합을 어디가 지배하는지 본다.
static uint64 GRopeUploadBytesTotal = 0;
static uint64 GRopeUploadBytesSDF = 0;        // Rope.GlobalSDF* — 복셀 볼륨 재업로드(정상 상태 ~0; 매 프레임 크면 캐시 미작동)
static uint64 GRopeUploadBytesColliders = 0;  // Capsules/Boxes/Convex/SDFColliders 인스턴스(매 프레임, 콜라이더 수 비례)
static uint64 GRopeReadbackBytes = 0;         // GPU→CPU 리드백(다운로드) 바이트 — Pos/Prev/Lambda/Contact
static uint32 GRopeDispatchCount = 0;         // 이번 프레임 컴퓨트 dispatch 수(솔브+감지)
static uint32 GRopeSubstepSum = 0;            // 이번 프레임 substep 합(솔버 작업량 프록시)

// 버퍼 이름으로 업로드를 카테고리 버킷에 분류. 감시 대상 두 축(SDF/Colliders)만 떼고 나머지(Detect/Override/
// 시드/Params)는 total에만 남긴다 — 작고 산발적이라 행을 쓸 값어치가 없다.
static void RopeAccumUploadBucket(const TCHAR* Name, uint64 Bytes)
{
	GRopeUploadBytesTotal += Bytes;
	if (FCString::Strifind(Name, TEXT("GlobalSDF")))
	{
		GRopeUploadBytesSDF += Bytes;
	}
	else if (FCString::Strifind(Name, TEXT("Capsules")) || FCString::Strifind(Name, TEXT("Boxes"))
		  || FCString::Strifind(Name, TEXT("Convex"))    || FCString::Strifind(Name, TEXT("SDFColliders")))
	{
		GRopeUploadBytesColliders += Bytes;
	}
}
#endif

// CreateStructuredBuffer 업로드 래퍼 — 초기데이터 바이트(InitialDataSize = 실제 GPU 전송량)를 카테고리별로
// 누산하고 엔진 헬퍼로 그대로 포워드한다. 모든 로프 GPU 업로드가 이 한 곳을 지나 프레임 대역폭이 자동 집계된다.
static FRDGBufferRef RopeUploadBuffer(FRDGBuilder& GraphBuilder, const TCHAR* Name, uint32 BytesPerElement,
	uint32 NumElements, const void* InitialData, uint64 InitialDataSize)
{
#if STATS
	RopeAccumUploadBucket(Name, InitialDataSize);
#endif
	return ::CreateStructuredBuffer(GraphBuilder, Name, BytesPerElement, NumElements, InitialData, InitialDataSize);
}

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
		SHADER_PARAMETER(float, DetectSweepStep)
		SHADER_PARAMETER(int32, DetectMaxSweepSamples)
		SHADER_PARAMETER(float, DetectPredictionFrames)
		SHADER_PARAMETER(float, DetectFrameToSubstepRatio)
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

	// (SDF 볼륨 그리드/헤더 resident는 per-rope에서 전역 FRopeGlobalSDFCache로 이동 — VolumeKey당 1회 상주.
	//  로프는 인스턴스(본 트랜스폼) 배열만 매 프레임 올린다. 전역화로 로프별 중복 업로드/집합 churn 재업로드 제거.)

	// 접촉 감지(G3): 노드당 1슬롯 출력 버퍼(resident, N 변할 때만 재생성) + 리드백(위치와 같은 ring).
	TRefCountPtr<FRDGPooledBuffer> ContactBuf;
	FRHIGPUBufferReadback* ContactReadback = nullptr;
	// 접촉 리드백을 무장한 dispatch의 귀속 서명 — 소비 시 결과에 실어 보낸다(오귀속 판정의 근거).
	uint32 ContactAttribSig = 0;
	bool bContactArmed = false;
};

// GT<->RT 공유 결과. RT가 채우고 GT GetLatest가 락 하에 읽는다.
struct FRopeResidentSharedResults
{
	FCriticalSection Lock;
	TMap<uint32, FRopeResidentLatest> Map;
	// G3: 접촉 감지 결과(GetLatestContacts).
	TMap<uint32, FRopeResidentContacts> Contacts;
	// 소비되지 못하고 교체된 pending step의 시뮬 시간(초). RT가 쌓고 GT가 DrainDroppedSimTime으로 비운다.
	TMap<uint32, float> DroppedSimTime;
	// wrap 핸드오프 동기 리드백의 프레임 스냅샷(ReadbackNow). 한 프레임에 여러 로프가 감겨도 GPU idle
	// 대기는 첫 요청 1회뿐이고, 나머지는 이 캐시를 락으로 읽는다(GT stall 없음).
	uint64 HandoffFrame = 0;
	TMap<uint32, FRopeResidentLatest> HandoffSnapshots;
};

// 전역 SDF 볼륨 캐시(RT 전용). 베이크된 복셀 데이터는 VolumeKey당 정적이라, 로프/프레임 무관하게 딱 한 번만
// dequant+업로드해 상주시킨다. 로프의 근접 볼륨 집합이 프레임마다 흔들려도(드래곤: 콜라이더 무상한+mesh 단위
// 브로드페이즈) distance 재업로드가 0이 된다 — 로프는 매 프레임 인스턴스(본 트랜스폼) 배열만 올린다. 인덱스/
// 오프셋은 프레임 내에서 안정적이라 기존 참조가 안 깨진다(신규 볼륨은 append). 무한 성장은 generational
// 재빌드로 막는다 — 오래 미참조 볼륨이 쌓이면 캐시를 live 집합만으로 압축 재구성한다(인스턴스 배열은 매
// 프레임 KeyToIndex 재조회라 인덱스 재배치 안전; 압축은 캐시된 CpuDist 슬라이스 복사라 소스 재-dequant 불필요).
struct FRopeGlobalSDFCache
{
	// VolumeKey(안정 식별자) -> 전역 볼륨 인덱스(= SDFVolumes 인덱스; 헤더가 CpuDist 오프셋을 가짐).
	TMap<uint64, int32> KeyToIndex;
	// VolumeKey -> 마지막으로 참조된 RT 프레임(재빌드 축출 판정용). KeyToIndex와 같은 키 집합.
	TMap<uint64, uint64> KeyLastUsedFrame;
	// CPU 원본(연결된 dequant float + 헤더). 신규 볼륨 append / 재빌드 시 live만 남기고 압축.
	TArray<float>             CpuDist;
	TArray<FRopeSDFVolumeGPU> CpuVol;
	// 상주 GPU 버퍼(external, 프레임 간 유지). dirty(신규 append 또는 재빌드)일 때만 재생성+업로드.
	TRefCountPtr<FRDGPooledBuffer> DistBuf;
	TRefCountPtr<FRDGPooledBuffer> VolBuf;
	bool bDirty = false;
	// RT 프레임 카운터(Ensure가 프레임당 1회 증가). KeyLastUsedFrame 스탬프/축출 판정의 시계.
	uint64 FrameCounter = 0;
};

// pimpl: 영속 버퍼 맵(RT 전용) + 공유 결과(GT<->RT). RDG/RHI 타입을 헤더에서 숨긴다.
struct FRopeGPUSolver::FImpl
{
	// 렌더 스레드에서만 접근.
	TMap<uint32, FRopeResidentRope>                          RtRopes;
	TSharedRef<FRopeResidentSharedResults, ESPMode::ThreadSafe> Results
		= MakeShared<FRopeResidentSharedResults, ESPMode::ThreadSafe>();

	// 전역 SDF 볼륨 상주(모든 로프 공유 — RopeEnsureGlobalSDFVolumes가 프레임당 1회 갱신).
	FRopeGlobalSDFCache GlobalSDF;

	// GDF 경로(EnqueueSteps)로 쌓인 이번 프레임 step들. 뷰 확장이 DispatchPending_RenderThread에서 소비. RT 전용.
	TArray<FRopeGPUResidentStep> PendingSteps;
};

bool RopeGPU::IsRuntimeSupported()
{
	// 커널이 SM5 가드로만 컴파일되므로(각 CS의 ShouldCompilePermutation) 그 아래 feature level에서는
	// 퍼뮤테이션 자체가 없다 — RHI 유무만 보던 종전 판정은 모바일에서 없는 셰이더를 요청하게 했다.
	// 판정 근거와 자세한 배경은 RopeGPUSolver.h 선언부 주석 참고.
	if (GDynamicRHI == nullptr || !FApp::CanEverRender())
	{
		return false;
	}
	if (GMaxRHIFeatureLevel < ERHIFeatureLevel::SM5)
	{
		// 조용한 CPU 폴백은 성능 이상으로 오해되기 쉬우므로 런타임에 1회 알린다.
		static bool bWarned = false;
		if (!bWarned)
		{
			bWarned = true;
			UE_LOG(LogDynamicRopeGPU, Warning,
				TEXT("GPU 로프 경로 비활성 — feature level이 SM5 미만이다(%s). CPU 솔버/튜브로 폴백한다."),
				*LexToString(GMaxRHIFeatureLevel));
		}
		return false;
	}
	return true;
}

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
	// 소비되지 않은 step도 함께 버린다 — 남으면 다음 dispatch가 방금 비운 상주 맵을 되살린다.
	Impl->PendingSteps.Reset();
	// 전역 SDF 상주 버퍼/캐시 해제(TRefCountPtr auto-release).
	Impl->GlobalSDF = FRopeGlobalSDFCache{};
}

void FRopeGPUSolver::ReleaseRope(uint32 RopeId)
{
	// 공유 결과는 GT에서 즉시 제거.
	{
		FScopeLock SL(&Impl->Results->Lock);
		Impl->Results->Map.Remove(RopeId);
		Impl->Results->Contacts.Remove(RopeId);
		Impl->Results->DroppedSimTime.Remove(RopeId);
		Impl->Results->HandoffSnapshots.Remove(RopeId);
	}
	// 영속 버퍼/리드백은 렌더 스레드에서 해제(this 캡처 — destructor가 flush하므로 수명 안전).
	ENQUEUE_RENDER_COMMAND(RopeGPUReleaseRope)(
		[this, RopeId](FRHICommandListImmediate&)
		{
			// 아직 소비되지 않은 pending step부터 걷어낸다. 남겨두면 뒤이은 뷰 확장 dispatch가
			// RunSteps_RenderThread의 FindOrAdd로 방금 지운 RopeId를 **되살려** 상주 버퍼/리드백을 다시
			// 만들고, 그 로프를 해제해 줄 주체는 이미 사라진 뒤라 월드 종료까지 VRAM에 남는다.
			// (EnqueueSteps는 교체 시맨틱이라 "다음 프레임이면 어차피 사라진다"가 성립하지 않는다 —
			// 씬 렌더가 없는 프레임에는 교체도 일어나지 않는다.)
			Impl->PendingSteps.RemoveAll(
				[RopeId](const FRopeGPUResidentStep& Step) { return Step.RopeId == RopeId; });

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

void FRopeGPUSolver::DrainDroppedSimTime(TMap<uint32, float>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	// 회수는 1회성이다(같은 시간을 두 번 돌려주면 오히려 앞서 나간다) — 옮기고 비운다.
	Out = MoveTemp(Impl->Results->DroppedSimTime);
	Impl->Results->DroppedSimTime.Reset();
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
	// GT 블로킹(M5c): 상주 미러(GetLatest)는 1~2프레임 낡아 wrap 시드 정밀도에 못 쓴다 — 핸드오프 순간만
	// 지금 값을 받는다. 비용은 GPU idle 대기 + 렌더 커맨드 flush라 프레임당 여러 번 내면 그대로 hitch다.
	// 그래서 **프레임 단위로 한 번만** 뜬다: 첫 요청이 상주 로프 전체를 한 그래프에서 복사하고 idle을
	// 1회 기다려 스냅샷을 만들고, 같은 프레임의 나머지 로프는 락만 잡고 그 스냅샷을 읽는다(대기 0).
	// 한 프레임 안에서는 dispatch가 아직 없어(솔브는 Prepare 뒤) 모든 로프에 같은 스냅샷이 유효하다.
	// 여분 복사(안 감기는 로프까지)는 로프당 수십 KB로, stall 한 번보다 훨씬 싸다.
	const uint64 FrameId = GFrameCounter;

	auto CopyOutFromCache = [&]() -> bool
	{
		const FRopeResidentLatest* Snap = Impl->Results->HandoffSnapshots.Find(RopeId);
		if (!Snap || Snap->NumNodes < 2)
		{
			return false;
		}
		OutPositions = Snap->Positions;
		OutPrevPositions = Snap->PrevPositions;
		OutGeneration = Snap->Generation;
		return true;
	};

	{
		FScopeLock SL(&Impl->Results->Lock);
		if (Impl->Results->HandoffFrame == FrameId)
		{
			return CopyOutFromCache();
		}
	}

	ENQUEUE_RENDER_COMMAND(RopeGPUReadbackNow)(
		[this, FrameId](FRHICommandListImmediate& RHICmdList)
		{
			// 상주 로프 전체의 Pos/Prev를 한 그래프에 모아 복사한다(패스는 많아도 idle 대기는 1회).
			TArray<uint32> Ids;
			TArray<TUniquePtr<FRHIGPUBufferReadback>> PosRbs;
			TArray<TUniquePtr<FRHIGPUBufferReadback>> PrevRbs;
			TArray<int32> Nodes;
			{
				FRDGBuilder GraphBuilder(RHICmdList);
				for (TPair<uint32, FRopeResidentRope>& Pair : Impl->RtRopes)
				{
					FRopeResidentRope& Rp = Pair.Value;
					if (!Rp.PosBuf.IsValid() || !Rp.PrevBuf.IsValid() || Rp.NumNodes < 2)
					{
						continue;
					}
					const uint32 Bytes = (uint32)Rp.NumNodes * sizeof(FVector4f);
					TUniquePtr<FRHIGPUBufferReadback> PosRb =
						MakeUnique<FRHIGPUBufferReadback>(TEXT("Rope.PosReadbackNow"));
					TUniquePtr<FRHIGPUBufferReadback> PrevRb =
						MakeUnique<FRHIGPUBufferReadback>(TEXT("Rope.PrevReadbackNow"));
					AddEnqueueCopyPass(GraphBuilder, PosRb.Get(),
						GraphBuilder.RegisterExternalBuffer(Rp.PosBuf), Bytes);
					AddEnqueueCopyPass(GraphBuilder, PrevRb.Get(),
						GraphBuilder.RegisterExternalBuffer(Rp.PrevBuf), Bytes);
					Ids.Add(Pair.Key);
					Nodes.Add(Rp.NumNodes);
					PosRbs.Add(MoveTemp(PosRb));
					PrevRbs.Add(MoveTemp(PrevRb));
				}
				GraphBuilder.Execute();
			}
			RHICmdList.BlockUntilGPUIdle();

			TMap<uint32, FRopeResidentLatest> Snapshots;
			Snapshots.Reserve(Ids.Num());
			for (int32 i = 0; i < Ids.Num(); ++i)
			{
				const int32 N = Nodes[i];
				const uint32 Bytes = (uint32)N * sizeof(FVector4f);
				const FVector4f* SrcPos = (const FVector4f*)PosRbs[i]->Lock(Bytes);
				const FVector4f* SrcPrev = SrcPos ? (const FVector4f*)PrevRbs[i]->Lock(Bytes) : nullptr;
				if (SrcPos && SrcPrev)
				{
					FRopeResidentLatest Snap;
					Snap.NumNodes = N;
					Snap.Positions.SetNumUninitialized(N);
					Snap.PrevPositions.SetNumUninitialized(N);
					for (int32 k = 0; k < N; ++k)
					{
						Snap.Positions[k] = FVector(SrcPos[k].X, SrcPos[k].Y, SrcPos[k].Z);
						Snap.PrevPositions[k] = FVector(SrcPrev[k].X, SrcPrev[k].Y, SrcPrev[k].Z);
					}
					if (const FRopeResidentRope* Rp = Impl->RtRopes.Find(Ids[i]))
					{
						Snap.Generation = Rp->Generation;
					}
					Snapshots.Add(Ids[i], MoveTemp(Snap));
				}
				if (SrcPos) { PosRbs[i]->Unlock(); }
				if (SrcPrev) { PrevRbs[i]->Unlock(); }
			}

			FScopeLock SL(&Impl->Results->Lock);
			Impl->Results->HandoffSnapshots = MoveTemp(Snapshots);
			Impl->Results->HandoffFrame = FrameId;
		});
	// RT 커맨드 완료까지 GT 대기(스냅샷 확정).
	FlushRenderingCommands();

	FScopeLock SL(&Impl->Results->Lock);
	return CopyOutFromCache();
}

FRHIShaderResourceView* FRopeGPUSolver::GetResidentPositionSRV_RenderThread(uint32 RopeId, int32& OutNumNodes,
	uint32& OutGeneration)
{
	check(IsInRenderingThread());
	OutNumNodes = 0;
	OutGeneration = 0;

	FRopeResidentRope* R = Impl->RtRopes.Find(RopeId);
	if (!R || !R->PosBuf.IsValid())
	{
		return nullptr;
	}
	OutNumNodes = R->NumNodes;
	OutGeneration = R->Generation;

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
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_ConsumeReadbacks);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_ConsumeReadbacks);
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
			CL.AttribSig  = R.ContactAttribSig;
		}
	}
}

// 상주 Pos/Prev/InvMass 확보: 재시드(최초/노드수·generation 변화)면 시드 업로드 + 외부 버퍼 변환,
// 아니면 기존 영속 버퍼를 그래프에 등록. B.PosRDG/PrevRDG/InvMassRDG/bSeed를 채운다.
static void RopeEnsureResidentBuffers(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	FRopeResidentRope& R, FRopeStepBuild& B)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_EnsureBuffers);
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
		B.PosRDG     = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Pos"),     sizeof(FVector4f), N, SeedPos.GetData(),  (uint64)N * sizeof(FVector4f));
		B.PrevRDG    = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Prev"),    sizeof(FVector4f), N, SeedPrev.GetData(), (uint64)N * sizeof(FVector4f));
		B.InvMassRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.InvMass"), sizeof(float),     N, SeedInv.GetData(),  (uint64)N * sizeof(float));
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
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackCapsules);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_PackCapsules);
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

	B.CapsulesBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Capsules"),
		sizeof(FRopeCapsuleGPU), CapsFlat.Num(), CapsFlat.GetData(), (uint64)CapsFlat.Num() * sizeof(FRopeCapsuleGPU));
}

// 박스 패킹: step의 정적 박스(OBB) → GPU 레이아웃 평탄화 + 업로드. B.BoxesBuf/NumValidBoxes를 채운다.
static void RopePackBoxes(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S, FRopeStepBuild& B)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackBoxes);
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

	B.BoxesBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Boxes"),
		sizeof(FRopeBoxGPU), BoxesFlat.Num(), BoxesFlat.GetData(), (uint64)BoxesFlat.Num() * sizeof(FRopeBoxGPU));
}

// 컨벡스 패킹: step의 정적 컨벡스 → 평면 평탄 풀(ConvexPlanes) + 헤더(Convexes) 업로드. 각 컨벡스의 평면을
// 풀에 이어붙이고 PlaneOffset/PlaneCount로 참조한다. B.ConvexBuf/ConvexPlanesBuf/NumValidConvexes를 채운다.
static void RopePackConvexes(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S, FRopeStepBuild& B)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackConvexes);
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

	B.ConvexBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Convexes"),
		sizeof(FRopeConvexGPU), ConvFlat.Num(), ConvFlat.GetData(), (uint64)ConvFlat.Num() * sizeof(FRopeConvexGPU));
	B.ConvexPlanesBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.ConvexPlanes"),
		sizeof(FVector4f), PlaneFlat.Num(), PlaneFlat.GetData(), (uint64)PlaneFlat.Num() * sizeof(FVector4f));
}

// 전역 SDF 볼륨 상주 보장(프레임당 1회, 로프 루프 *전*). 이번 프레임 모든 Step의 SDF 콜라이더에서 아직
// 캐시에 없는 VolumeKey만 dequant해 전역 배열에 append하고, 신규 볼륨이 생긴 프레임에만(dirty) 상주 버퍼를
// 재업로드한다(세션당 볼륨 수만큼, 이후 0). 반환: 이번 프레임 바인딩할 전역 distance/header RDG 버퍼(어느
// 로프도 SDF가 없으면 더미 1개). 이걸로 로프별·집합-churn 재업로드(구 per-rope VolSig 게이트의 50ms)를 없앤다.
// generational 재빌드 파라미터.
//  - EvictAfterFrames: 이만큼 연속 미참조면 축출 대상(히스테리시스 — 컬링 경계 깜빡임에 재빌드 안 터지게).
//  - RebuildReclaimFrac: 죽은 dist 바이트 비율이 이 이상일 때만 재빌드(자잘한 회수로 MB 재업로드 방지).
static constexpr uint64 GRopeSDFEvictAfterFrames = 600;   // ~10s @ 60fps
static constexpr float  GRopeSDFRebuildReclaimFrac = 0.25f;

// 캐시를 live(최근 EvictAfterFrames 내 참조) 볼륨만으로 압축 재구성. dequant된 CpuDist 슬라이스를 그대로
// 복사(소스 불필요)하고 인덱스/오프셋을 새로 부여 → bDirty로 상주 버퍼 1회 재업로드. 인스턴스 배열이 매
// 프레임 KeyToIndex를 재조회하므로 인덱스 재배치는 다음 팩 단계가 자동 반영(참조 무손상).
static void RopeRebuildGlobalSDFCache(FRopeGlobalSDFCache& Cache, uint64 Frame)
{
	TMap<uint64, int32>   NewKeyToIndex;
	TMap<uint64, uint64>  NewLastUsed;
	TArray<float>              NewDist;
	TArray<FRopeSDFVolumeGPU>  NewVol;
	NewKeyToIndex.Reserve(Cache.KeyToIndex.Num());
	NewVol.Reserve(Cache.CpuVol.Num());
	NewDist.Reserve(Cache.CpuDist.Num());

	for (const TPair<uint64, int32>& KV : Cache.KeyToIndex)
	{
		const uint64* Last = Cache.KeyLastUsedFrame.Find(KV.Key);
		if (!Last || (Frame - *Last) >= GRopeSDFEvictAfterFrames)
		{
			continue;   // 오래 미참조 — 드롭.
		}
		const FRopeSDFVolumeGPU& Old = Cache.CpuVol[KV.Value];
		const int32 VoxN = Old.ResX * Old.ResY * Old.ResZ;

		FRopeSDFVolumeGPU NewV = Old;
		NewV.DistOffset = NewDist.Num();
		NewKeyToIndex.Add(KV.Key, NewVol.Num());
		NewVol.Add(NewV);
		NewDist.Append(Cache.CpuDist.GetData() + Old.DistOffset, VoxN);
		NewLastUsed.Add(KV.Key, *Last);
	}

	Cache.KeyToIndex       = MoveTemp(NewKeyToIndex);
	Cache.KeyLastUsedFrame = MoveTemp(NewLastUsed);
	Cache.CpuVol           = MoveTemp(NewVol);
	Cache.CpuDist          = MoveTemp(NewDist);
	Cache.bDirty           = true;   // 아래 업로드 경로가 압축된 버퍼를 1회 재업로드.
}

static void RopeEnsureGlobalSDFVolumes(FRDGBuilder& GraphBuilder, const TArray<FRopeGPUResidentStep>& Steps,
	FRopeGlobalSDFCache& Cache, FRDGBufferRef& OutDistRDG, FRDGBufferRef& OutVolRDG)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_EnsureGlobalSDF);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_EnsureGlobalSDF);
	const uint64 Frame = ++Cache.FrameCounter;   // 프레임당 1회(Ensure는 RunSteps당 1회).
	for (const FRopeGPUResidentStep& S : Steps)
	{
		for (const FRopeGPUSDFCollider& Src : S.SDFColliders)
		{
			const int64 Voxels = (int64)Src.ResX * Src.ResY * Src.ResZ;
			if (!Src.Distances || Src.ResX < 2 || Src.ResY < 2 || Src.ResZ < 2 || Voxels <= 0)
			{
				continue;
			}
			// 유효 볼륨 — 마지막 사용 프레임 스탬프(신규/기존 공통; 재빌드 축출 판정의 기준).
			Cache.KeyLastUsedFrame.FindOrAdd(Src.VolumeKey) = Frame;
			if (Cache.KeyToIndex.Contains(Src.VolumeKey))
			{
				// 이미 상주 — dequant/업로드 없음(정적 베이크 데이터라 재-dequant 불필요).
				continue;
			}
			// 신규 볼륨: 전역 배열에 append(인덱스/오프셋 stable — 기존 참조 불변).
			Cache.KeyToIndex.Add(Src.VolumeKey, Cache.CpuVol.Num());

			FRopeSDFVolumeGPU V;
			V.DistOffset = Cache.CpuDist.Num();
			V.ResX = Src.ResX; V.ResY = Src.ResY; V.ResZ = Src.ResZ;
			V.LocalMin  = FVector4f((float)Src.LocalMin.X,  (float)Src.LocalMin.Y,  (float)Src.LocalMin.Z,  0.0f);
			V.LocalSize = FVector4f((float)Src.LocalSize.X, (float)Src.LocalSize.Y, (float)Src.LocalSize.Z, 0.0f);
			Cache.CpuVol.Add(V);

			// 코드 → float(cm) dequant. 비대칭 밴드: d = code*(range/MaxCode) - NBIn. 코드는 복셀당 BytesPerCode
			// 바이트(리틀엔디안): 1=uint8(max255), 2=uint16(max65535). 셰이더 SDFDistances는 float 유지(.usf 무변경).
			const int32 VoxN = (int32)Voxels;
			const int32 Bpc = Src.BytesPerCode;
			const float MaxCodeF = (Bpc >= 2) ? 65535.0f : 255.0f;
			const float NBIn = Src.NarrowBandInner;
			const float Range = NBIn + Src.NarrowBandOuter;
			const float DeqScale = (Range > 0.0f) ? (Range / MaxCodeF) : 0.0f;
			Cache.CpuDist.Reserve(Cache.CpuDist.Num() + VoxN);
			for (int32 Vi = 0; Vi < VoxN; ++Vi)
			{
				uint32 Code = Src.Distances[Vi * Bpc];
				if (Bpc >= 2)
				{
					Code |= static_cast<uint32>(Src.Distances[Vi * Bpc + 1]) << 8;
				}
				// 바깥 +
				Cache.CpuDist.Add(static_cast<float>(Code) * DeqScale - NBIn);
			}
			Cache.bDirty = true;
		}
	}

	// generational 축출: 오래 미참조 볼륨이 회수 문턱 이상 쌓였으면 live 집합만으로 캐시 압축 재구성.
	if (Cache.CpuVol.Num() > 0)
	{
		int64 DeadFloats = 0;
		for (const TPair<uint64, int32>& KV : Cache.KeyToIndex)
		{
			const uint64* Last = Cache.KeyLastUsedFrame.Find(KV.Key);
			if (!Last || (Frame - *Last) >= GRopeSDFEvictAfterFrames)
			{
				const FRopeSDFVolumeGPU& H = Cache.CpuVol[KV.Value];
				DeadFloats += (int64)H.ResX * H.ResY * H.ResZ;
			}
		}
		if (DeadFloats > 0 && Cache.CpuDist.Num() > 0 &&
			(float)DeadFloats / (float)Cache.CpuDist.Num() >= GRopeSDFRebuildReclaimFrac)
		{
			RopeRebuildGlobalSDFCache(Cache, Frame);
		}
	}

	if (Cache.CpuVol.Num() == 0)
	{
		// 볼륨이 하나도 없음(최초, 또는 재빌드로 전부 축출) — 상주 버퍼 해제(VRAM 회수) 후 더미 바인딩.
		Cache.DistBuf.SafeRelease();
		Cache.VolBuf.SafeRelease();
		Cache.bDirty = false;
		// 더미 1개(바인딩 유효성; 셰이더는 NumSDFColliders=0이라 미참조).
		TArray<float>&             DummyDist = *GraphBuilder.AllocObject<TArray<float>>();
		TArray<FRopeSDFVolumeGPU>& DummyVol  = *GraphBuilder.AllocObject<TArray<FRopeSDFVolumeGPU>>();
		DummyDist.AddZeroed(1);
		DummyVol.AddZeroed(1);
		OutDistRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.GlobalSDFDist.Dummy"),
			sizeof(float), 1, DummyDist.GetData(), sizeof(float));
		OutVolRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.GlobalSDFVol.Dummy"),
			sizeof(FRopeSDFVolumeGPU), 1, DummyVol.GetData(), sizeof(FRopeSDFVolumeGPU));
		return;
	}

	if (Cache.DistBuf.IsValid() && Cache.VolBuf.IsValid() && !Cache.bDirty)
	{
		// 신규 볼륨 없음 → 재사용(업로드 0). 정상 상태 — 로프의 근접 볼륨 집합/순서가 흔들려도 여기로 온다.
		OutDistRDG = GraphBuilder.RegisterExternalBuffer(Cache.DistBuf);
		OutVolRDG  = GraphBuilder.RegisterExternalBuffer(Cache.VolBuf);
		return;
	}

	// 최초 또는 신규 볼륨(dirty) — 전역 버퍼 재생성+업로드(append-only라 grow, 세션당 볼륨 수만큼만 발생).
	OutDistRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.GlobalSDFDist"),
		sizeof(float), Cache.CpuDist.Num(), Cache.CpuDist.GetData(), (uint64)Cache.CpuDist.Num() * sizeof(float));
	OutVolRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.GlobalSDFVol"),
		sizeof(FRopeSDFVolumeGPU), Cache.CpuVol.Num(), Cache.CpuVol.GetData(), (uint64)Cache.CpuVol.Num() * sizeof(FRopeSDFVolumeGPU));
	Cache.DistBuf = GraphBuilder.ConvertToExternalBuffer(OutDistRDG);
	Cache.VolBuf  = GraphBuilder.ConvertToExternalBuffer(OutVolRDG);
	Cache.bDirty = false;
}

// SDF 콜라이더 패킹(M3): 전역 볼륨 캐시(RopeEnsureGlobalSDFVolumes가 상주 보장)를 공유 바인딩하고, 이 로프의
// 인스턴스(전역 VolumeIndex + 현재/직전 본 트랜스폼) 배열만 매 프레임 올린다. distance dequant/업로드는 여기서
// 하지 않는다 — 전역 캐시가 VolumeKey당 1회만 수행. B.SDF*Buf/NumValidSDFCol을 채운다.
static void RopePackSDFColliders(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S, FRopeStepBuild& B,
	const TMap<uint64, int32>& GlobalKeyToIndex, FRDGBufferRef GlobalDistRDG, FRDGBufferRef GlobalVolRDG)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackSDF);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_PackSDF);
	// 전역 distance/header 버퍼를 공유 바인딩(로프별 복사 없음).
	B.SDFDistBuf = GlobalDistRDG;
	B.SDFVolBuf  = GlobalVolRDG;

	TArray<FRopeSDFColliderGPU>& SDFCol = *GraphBuilder.AllocObject<TArray<FRopeSDFColliderGPU>>();
	for (const FRopeGPUSDFCollider& Src : S.SDFColliders)
	{
		const int32* VolIdx = GlobalKeyToIndex.Find(Src.VolumeKey);
		if (!VolIdx)
		{
			// 무효 볼륨(전역 캐시가 dequant 조건으로 걸러 미등록) — 스킵.
			continue;
		}
		const FQuat   Q  = Src.BoneToWorld.GetRotation();
		const FVector T  = Src.BoneToWorld.GetTranslation();
		const FVector Sc = Src.BoneToWorld.GetScale3D();
		const FQuat   PQ = Src.PrevBoneToWorld.GetRotation();
		const FVector PT = Src.PrevBoneToWorld.GetTranslation();
		FRopeSDFColliderGPU C;
		C.VolumeIndex     = *VolIdx;
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
	B.SDFColBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.SDFColliders"),
		sizeof(FRopeSDFColliderGPU), SDFCol.Num(), SDFCol.GetData(), (uint64)SDFCol.Num() * sizeof(FRopeSDFColliderGPU));
}

// Override(G0) 업로드: 노드별 플래그/타깃/질량(transient, 오버라이드 프레임만 실데이터).
// 없으면 더미 1개 + bHasOverrides=0 → 셰이더가 참조하지 않는다.
static void RopePackOverrides(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S, FRopeStepBuild& B)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackOverrides);
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
	B.OvFlagsBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.OverrideFlags"),
		sizeof(uint32), OvFlags.Num(), OvFlags.GetData(), (uint64)OvFlags.Num() * sizeof(uint32));
	B.OvPosBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.OverridePositions"),
		sizeof(FVector4f), OvPos.Num(), OvPos.GetData(), (uint64)OvPos.Num() * sizeof(FVector4f));
	B.OvPrevBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.OverridePrevPositions"),
		sizeof(FVector4f), OvPrev.Num(), OvPrev.GetData(), (uint64)OvPrev.Num() * sizeof(FVector4f));
	B.OvInvBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.OverrideInvMass"),
		sizeof(float), OvInv.Num(), OvInv.GetData(), (uint64)OvInv.Num() * sizeof(float));
}

// 솔브 패스: 파라미터 버퍼 구성 + XPBD CS dispatch(GDF permutation은 로프 단위 선택).
// 장력(λ) 출력 버퍼(프레임 transient)를 만들어 반환한다 — 리드백 무장(RopeArmReadbacks)이 소비.
static FRDGBufferRef RopeAddSolvePass(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	const FRopeResidentRope& R, const FRopeStepBuild& B, const FSceneView* View,
	const FGlobalDistanceFieldParameters2& GDFSolverParams, uint32 bGDFSolverValid,
	const FVector3f& PreViewTranslation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_AddSolvePass);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_AddSolvePass);
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

	FRDGBufferRef ParamsBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Params"),
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
#if STATS
	++GRopeDispatchCount;
#endif
	FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeXPBDResident"),
		// 로프 1개 = 스레드그룹 1개
		ComputeShader, PassParams, FIntVector(1, 1, 1));

	return LambdaRDG;
}

// 리드백 재무장: in-flight가 없을 때만(이번 프레임 stepped 위치를 비동기 copy). consume은 RopeConsumeReadbacks.
static void RopeArmReadbacks(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	FRopeResidentRope& R, const FRopeStepBuild& B, FRDGBufferRef LambdaRDG)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_ArmReadbacks);
	const int32 N = S.NumNodes;
	if (!R.bReadbackArmed)
	{
		if (!R.PosReadback)  { R.PosReadback  = new FRHIGPUBufferReadback(TEXT("Rope.PosReadback")); }
		if (!R.PrevReadback) { R.PrevReadback = new FRHIGPUBufferReadback(TEXT("Rope.PrevReadback")); }
		const uint32 NodeBytes = (uint32)N * sizeof(FVector4f);
		AddEnqueueCopyPass(GraphBuilder, R.PosReadback,  B.PosRDG,  NodeBytes);
		AddEnqueueCopyPass(GraphBuilder, R.PrevReadback, B.PrevRDG, NodeBytes);
#if STATS
		GRopeReadbackBytes += 2ull * NodeBytes;
#endif
		R.bReadbackArmed = true;
	}

	// 장력(λ) 리드백 무장: 솔브 프레임(NumSub>0)에만 — override-only 프레임은 λ가 0이라
	// 무장하지 않고 직전 장력을 유지한다(GT는 갱신분이 있을 때만 덮어씀).
	if (!R.bLambdaArmed && S.NumSub > 0)
	{
		if (!R.LambdaReadback) { R.LambdaReadback = new FRHIGPUBufferReadback(TEXT("Rope.LambdaReadback")); }
		AddEnqueueCopyPass(GraphBuilder, R.LambdaReadback, LambdaRDG, (uint32)N * sizeof(float));
#if STATS
		GRopeReadbackBytes += (uint64)N * sizeof(float);
#endif
		R.LambdaFixedDt = S.FixedDt;
		R.bLambdaArmed = true;
	}
}

// 접촉 감지(G3): 솔브 뒤 post-solve 위치를 스윕. RDG가 solve(UAV)→detect(SRV) 순서를 보장한다.
// 노드당 2슬롯(actual+predictive) 출력. 감지가 있을 때만 ContactBuf(resident, 2N슬롯) 확보.
static void RopeAddDetectPass(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& S,
	FRopeResidentRope& R, const FRopeStepBuild& B)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_AddDetectPass);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_AddDetectPass);
	// 솔브 스코프 안에 중첩 — 감지 커널 시간은 Solve가 아니라 이쪽으로 귀속된다.
	RDG_EVENT_SCOPE_STAT(GraphBuilder, RopeGPUDetect, "DynamicRope Detect");
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
	FRDGBufferRef GMaskBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.DetectGuidedMask"),
		sizeof(uint32), GMask.Num(), GMask.GetData(), (uint64)GMask.Num() * sizeof(uint32));
	FRDGBufferRef WCurBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.DetectWhipCur"),
		sizeof(FVector4f), WCur.Num(), WCur.GetData(), (uint64)WCur.Num() * sizeof(FVector4f));
	FRDGBufferRef WPrevBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.DetectWhipPrev"),
		sizeof(FVector4f), WPrev.Num(), WPrev.GetData(), (uint64)WPrev.Num() * sizeof(FVector4f));
	FRDGBufferRef WNextBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.DetectWhipNext"),
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
	DetectParams->DetectSweepStep      = FMath::Max(S.ContactSweepStep, 0.1f);
	DetectParams->DetectMaxSweepSamples = FMath::Max(S.ContactMaxSweepSamples, 1);
	DetectParams->DetectPredictionFrames = FMath::Max(0.0f, S.PredictionFrames);
	DetectParams->DetectFrameToSubstepRatio = S.ContactFrameToSubstepRatio;
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
#if STATS
	++GRopeDispatchCount;
#endif
	FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeContactDetect"),
		DetectShader, DetectParams, FIntVector(1, 1, 1));

	if (!R.bContactArmed)
	{
		if (!R.ContactReadback) { R.ContactReadback = new FRHIGPUBufferReadback(TEXT("Rope.ContactReadback")); }
		AddEnqueueCopyPass(GraphBuilder, R.ContactReadback, ContactRDG, (uint32)(2 * N) * sizeof(FRopeGPUContactGPU));
		// ColliderIndex가 가리키는 집합은 *이* dispatch의 것이다 — 그 서명을 결과까지 들고 간다.
		R.ContactAttribSig = S.AttribSig;
#if STATS
		GRopeReadbackBytes += (uint64)(2 * N) * sizeof(FRopeGPUContactGPU);
#endif
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
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_RunSteps);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_RunSteps);
	// GPU 타임라인 귀속: 이 스코프 안에서 GraphBuilder에 추가되는 모든 패스가 'DynamicRope Solve'로 잡힌다
	// (감지 패스는 RopeAddDetectPass가 자체 스코프로 다시 떼어간다).
	RDG_EVENT_SCOPE_STAT(GraphBuilder, RopeGPUSolve, "DynamicRope Solve");
#if STATS
	// 이번 프레임 업로드 누산 리셋(아래 RopeUploadBuffer들이 카테고리별로 더한다). RunSteps는 프레임당 1회.
	GRopeUploadBytesTotal = 0;
	GRopeUploadBytesSDF = 0;
	GRopeUploadBytesColliders = 0;
	GRopeReadbackBytes = 0;
	GRopeDispatchCount = 0;
	GRopeSubstepSum = 0;
#endif
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

	// 전역 SDF 볼륨 상주 보장(프레임당 1회, 로프 루프 *전*). 신규 볼륨만 dequant/업로드 → 정상 상태 재업로드 0.
	// 아래 로프별 RopePackSDFColliders는 이 전역 버퍼를 공유 바인딩하고 인스턴스(본 트랜스폼) 배열만 올린다.
	FRDGBufferRef GlobalSDFDistRDG = nullptr;
	FRDGBufferRef GlobalSDFVolRDG = nullptr;
	RopeEnsureGlobalSDFVolumes(GraphBuilder, Steps, Impl->GlobalSDF, GlobalSDFDistRDG, GlobalSDFVolRDG);

	// --- Loop 2: 로프별 seed/register → 패킹 → 솔브 → 리드백 재무장 → 감지 → 외부(SRV) 배리어.
	for (const FRopeGPUResidentStep& S : Steps)
	{
		const int32 N = S.NumNodes;
#if STATS
		GRopeSubstepSum += (uint32)FMath::Max(0, S.NumSub);
#endif
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
		RopePackSDFColliders(GraphBuilder, S, B, Impl->GlobalSDF.KeyToIndex, GlobalSDFDistRDG, GlobalSDFVolRDG);
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

#if STATS
	// GPU 상주 VRAM 계측. 영속 버퍼(ConvertToExternalBuffer로 배정, 프레임 간 유지)만 합산 — 프레임 transient
	// (colliders/params/detect RDG)는 풀 재사용이라 상주 풋프린트가 아니다. GetSize()=Desc 바이트.
	{
		uint64 RopeBytes = 0;
		for (const TPair<uint32, FRopeResidentRope>& Pair : Impl->RtRopes)
		{
			const FRopeResidentRope& RR = Pair.Value;
			if (RR.PosBuf.IsValid())     { RopeBytes += RR.PosBuf->GetSize(); }
			if (RR.PrevBuf.IsValid())    { RopeBytes += RR.PrevBuf->GetSize(); }
			if (RR.InvMassBuf.IsValid()) { RopeBytes += RR.InvMassBuf->GetSize(); }
			if (RR.ContactBuf.IsValid()) { RopeBytes += RR.ContactBuf->GetSize(); }
		}
		uint64 SdfBytes = 0;
		if (Impl->GlobalSDF.DistBuf.IsValid()) { SdfBytes += Impl->GlobalSDF.DistBuf->GetSize(); }
		if (Impl->GlobalSDF.VolBuf.IsValid())  { SdfBytes += Impl->GlobalSDF.VolBuf->GetSize(); }

		SET_MEMORY_STAT(STAT_RopeGPU_MemRopes, RopeBytes);
		SET_MEMORY_STAT(STAT_RopeGPU_MemGlobalSDF, SdfBytes);
		SET_MEMORY_STAT(STAT_RopeGPU_MemTotal, RopeBytes + SdfBytes);
		SET_DWORD_STAT(STAT_RopeGPU_ResidentRopeCount, Impl->RtRopes.Num());
		SET_DWORD_STAT(STAT_RopeGPU_SDFVolumes, Impl->GlobalSDF.CpuVol.Num());
		SET_MEMORY_STAT(STAT_RopeGPU_UploadTotal, GRopeUploadBytesTotal);
		SET_MEMORY_STAT(STAT_RopeGPU_UploadSDF, GRopeUploadBytesSDF);
		SET_MEMORY_STAT(STAT_RopeGPU_UploadColliders, GRopeUploadBytesColliders);
		SET_MEMORY_STAT(STAT_RopeGPU_ReadbackBytes, GRopeReadbackBytes);
		SET_DWORD_STAT(STAT_RopeGPU_Dispatches, GRopeDispatchCount);
		SET_DWORD_STAT(STAT_RopeGPU_Substeps, GRopeSubstepSum);
	}
#endif
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
			{
				// RDG 컴파일 + RHI 커맨드 기록(렌더 스레드 CPU 비용의 큰 부분일 수 있음).
				TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_GraphExecute);
				SCOPE_CYCLE_COUNTER(STAT_RopeGPU_GraphExecute);
				GraphBuilder.Execute();
			}
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
			// 렌더가 없던 프레임 등 — 최신만 유효하므로 누적하지 않는다). 다만 **시간은 버리지 않는다**:
			// 덮어쓰는 step이 들고 있던 substep 분량을 장부에 적어 GT가 accumulator로 되돌리게 한다
			// (그냥 버리면 그 시간만큼 시뮬이 영구히 뒤처진다 — DrainDroppedSimTime 주석).
			if (Impl->PendingSteps.Num() > 0)
			{
				FScopeLock SL(&Impl->Results->Lock);
				for (const FRopeGPUResidentStep& Dropped : Impl->PendingSteps)
				{
					if (Dropped.NumSub > 0 && Dropped.FixedDt > 0.0f)
					{
						Impl->Results->DroppedSimTime.FindOrAdd(Dropped.RopeId) +=
							static_cast<float>(Dropped.NumSub) * Dropped.FixedDt;
					}
				}
			}
			Impl->PendingSteps = MoveTemp(Steps);
		});
}

void FRopeGPUSolver::DispatchPending_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView* View,
	const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_DispatchPending);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_DispatchPending);
	check(IsInRenderingThread());
	if (Impl->PendingSteps.Num() == 0)
	{
		return;
	}
	RunSteps_RenderThread(GraphBuilder, Impl->PendingSteps, View, GDF, PreViewTranslation);
	Impl->PendingSteps.Reset();
}
