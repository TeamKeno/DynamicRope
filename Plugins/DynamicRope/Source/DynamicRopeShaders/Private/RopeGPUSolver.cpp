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
// TStaticSamplerState (GDF sampler) — Phase 2c
#include "RHIStaticStates.h"
// FGlobalDistanceFieldParameters2 / _Minimal — Phase 2c
#include "GlobalDistanceFieldParameters.h"
// GBlackVolumeTexture / GBlackUintVolumeTexture — Phase 2c
#include "GlobalRenderResources.h"
// FSceneView / FViewUniformShaderParameters (GDF pass View UB) — Phase 2c
#include "SceneView.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "Misc/ScopeLock.h"
// TRACE_CPUPROFILER_EVENT_SCOPE — render thread dispatch path ground truth (Unreal Insights CPU timeline).
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "ProfilingDebugging/RealtimeGPUProfiler.h"
#include "Stats/Stats.h"

// 'stat DynamicRopeGPU' RT timing/memory/bandwidth — see RopeGPUStatGroup.h for group declaration and separation reasons (runtime GT
// Separate group from dashboard 'stat DynamicRope'). The CYCLE stat below shows RT in each RopeRT_* scope (SCOPE_CYCLE_COUNTER)
// Capture thread time — RT CPU time, not GPU timeline time. In builds where STATS is turned off, the macro is an automatic no-op.
// RunSteps=RT total volume, the rest is its sub-decomposition (PackSDF=For pointing out SDF reupload bottlenecks — memory: sdf-global-volume-cache).
// The HUD only has steps that are actually captured in the frame budget — sub-millisecond bookkeeping (EnsureBuffers/Pack Boxes·Convexes·
// Overrides/Arm Readbacks) remain in the RopeRT_* Insights scope, so you can bone them there when needed.
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
// GPU resident VRAM (persistent buffer maintained between frames) — Pos/Prev/InvMass/Contact per rope + global SDF cache (Dist/Vol).
// Add GetSize() (bytes) at the end of RunSteps and set. It's not the transmission bandwidth, it's the resident footprint (SDF is the cache size).
DECLARE_MEMORY_STAT(TEXT("GPU Mem: Rope Buffers"), STAT_RopeGPU_MemRopes, STATGROUP_DynamicRopeGPU);
DECLARE_MEMORY_STAT(TEXT("GPU Mem: Global SDF"), STAT_RopeGPU_MemGlobalSDF, STATGROUP_DynamicRopeGPU);
DECLARE_MEMORY_STAT(TEXT("GPU Mem: Resident Total"), STAT_RopeGPU_MemTotal, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Resident Ropes (count)"), STAT_RopeGPU_ResidentRopeCount, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU SDF Volumes"), STAT_RopeGPU_SDFVolumes, STATGROUP_DynamicRopeGPU);
// GPU upload bandwidth per frame, in bytes actually transferred, accumulated by the CreateStructuredBuffer
// uploads inside RopeUploadBuffer. Unlike the resident footprint (GPU Mem above) this is what goes up to the
// GPU every frame. Breaking it down leaves only two axes worth watching: SDF, which spikes into the megabytes
// every frame if the global volume cache stops working, so it is the regression signal; and Colliders, the
// one axis that scales with the content. The rest — detect, override, seed — is small and sporadic and stays
// folded into the total.
DECLARE_MEMORY_STAT(TEXT("GPU Upload/Frame (total)"), STAT_RopeGPU_UploadTotal, STATGROUP_DynamicRopeGPU);
DECLARE_MEMORY_STAT(TEXT("GPU Upload/Frame (SDF Volume)"), STAT_RopeGPU_UploadSDF, STATGROUP_DynamicRopeGPU);
DECLARE_MEMORY_STAT(TEXT("GPU Upload/Frame (Colliders)"), STAT_RopeGPU_UploadColliders, STATGROUP_DynamicRopeGPU);
// GPU→CPU download bandwidth + per-frame compute dispatch/substep number (solver workload).
DECLARE_MEMORY_STAT(TEXT("GPU Readback/Frame (download)"), STAT_RopeGPU_ReadbackBytes, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Dispatches/Frame"), STAT_RopeGPU_Dispatches, STATGROUP_DynamicRopeGPU);
DECLARE_DWORD_COUNTER_STAT(TEXT("GPU Substeps/Frame"), STAT_RopeGPU_Substeps, STATGROUP_DynamicRopeGPU);

// ── GPU timeline stat — The measurement target is different from the above CYCLE stat ─────────────────────────────────────────
// The 'GPU *' CYCLE stat at the top is all **RT CPU time** (the time taken to create the graph). The two below are actually GPUs.
// The time the kernel was run, go to the engine GPU group and go to 'stat gpu' / GPU Visualizer (ProfileGPU) / Insights GPU track.
// appears — it does not appear in this group ('stat DynamicRopeGPU'). This number shows how many ms rope takes from the frame budget.
//
// [Macro Selection — Version Agreement] UE 5.7 has RHI_NEW_GPU_PROFILER=1 so the older RDG_GPU_STAT_SCOPE/SCOPED_GPU_STAT
// **Quietly becomes a no-op** (5.8 was completely deprecated). On the other hand, RDG_EVENT_SCOPE_STAT(RDG path) and
// RHI_BREADCRUMB_EVENT_STAT (immediate RHI path) carries the stat id from versions 5.5 to 5.8 supported by this plugin.
// Carry. So we only use these two forms without RopeRHICompat gating — follow this when adding a new GPU scope.
DECLARE_GPU_STAT_NAMED(RopeGPUSolve, TEXT("DynamicRope Solve"));
DECLARE_GPU_STAT_NAMED(RopeGPUDetect, TEXT("DynamicRope Detect"));

// RT-only frame upload accumulator (Reset at the start of RunSteps, SET at the end). RunSteps runs once per frame.
#if STATS
// frame upload accumulator (RunSteps start reset, end SET). Breaking it down by category, it's the bone that dominates the total MB.
static uint64 GRopeUploadBytesTotal = 0;
static uint64 GRopeUploadBytesSDF = 0;        // Rope.GlobalSDF* — Reupload voxel volume (normal state ~0; cache does not work if each frame is large)
static uint64 GRopeUploadBytesColliders = 0;  // Capsules/Boxes/Convex/SDFColliders instance (every frame, proportional to the number of colliders)
static uint64 GRopeReadbackBytes = 0;         // GPU→CPU readback (download) bytes — Pos/Prev/Lambda/Contact
static uint32 GRopeDispatchCount = 0;         // Number of compute dispatches for this frame (solve+detection)
static uint32 GRopeSubstepSum = 0;            // This frame substep sum (solver workload proxy)

// Classifies uploads into category buckets using the buffer name. Remove only the two axes to be monitored (SDF/Colliders) and the rest (Detect/Override/
// Seed/Params) are only left in total — they are small and sporadic, so it is not worth using the row.
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

// CreateStructuredBuffer upload wrapper — Initial data bytes (InitialDataSize = actual GPU transfer amount) by category
// Accumulate and forward as is to the engine helper. All rope GPU uploads pass through this one place and frame bandwidth is automatically calculated.
static FRDGBufferRef RopeUploadBuffer(FRDGBuilder& GraphBuilder, const TCHAR* Name, uint32 BytesPerElement,
	uint32 NumElements, const void* InitialData, uint64 InitialDataSize)
{
#if STATS
	RopeAccumUploadBucket(Name, InitialDataSize);
#endif
	return ::CreateStructuredBuffer(GraphBuilder, Name, BytesPerElement, NumElements, InitialData, InitialDataSize);
}

// node bucket (thread group size == groupshared/numthreads size). 1 rope = 1 thread group, node = thread,
// In the past, all ropes held pinned 256 groups, and most of the threads of ropes with a small number of nodes were idle (participating in the barrier).
// Now, select the smallest bucket (permutation) that is more than NumNodes, eliminate that waste, and set the support node cap to 512 at the top.
// Raise. numthreads(ROPE_THREADS)/groupshared(ROPE_MAX_NODES) scale by bucket value — permutation
// Set ROPE_MAX_NODES to the bucket value and ModifyCompilationEnvironment adjusts ROPE_THREADS=bucket.
// groupshared budget: solve 9float/node → 512node=18KB (<32KB). Detect no groupshared (numthreads only).
static constexpr int32 GRopeNodeBuckets[] = { 64, 128, 256, 512 };
static_assert(GRopeNodeBuckets[UE_ARRAY_COUNT(GRopeNodeBuckets) - 1] == FRopeGPUSolver::MaxNodes,
	"The top node bucket must equal FRopeGPUSolver::MaxNodes (the subsystem's GPU eligibility gate uses MaxNodes).");

// Smallest bucket with more than NumNodes. If not (> cap) 0. The caller always receives ≥64 after the MaxNodes gate.
static int32 RopeNodeBucket(int32 NumNodes)
{
	for (int32 Bucket : GRopeNodeBuckets)
	{
		if (NumNodes <= Bucket) { return Bucket; }
	}
	return 0;
}

// 1:1 mirror with HLSL FRopeGPUParams (RopeXPBD.usf). Simultaneous modification of .usf when changing layout. 16 byte alignment.
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
	// Global start index of this rope's capsules
	int32     CapsuleOffset;
	// Capsule count (0 = no capsule collision)
	int32     NumCapsules;
	// Node thickness
	float     CollisionRadius;
	// Tangential damping
	float     Friction;
	// Swept sample spacing
	float     SweepStep;
	// Sample cap per segment
	int32     MaxSweepSamples;
	// Global start index of this rope's SDF colliders
	int32     SDFColliderOffset;
	// SDF collider count (0 = no SDF collision)
	int32     NumSDFColliders;
	// Free end friction multiplier (pinned point=1, end=this value). Pad0 slot reuse.
	float     TipFrictionScale = 1.0f;
	// Number of collision resolution passes per substep (cap in Iters). Reuse Pad1 slot.
	int32     CollisionPasses = 1;
	// G0: Is there per-node override (target/mass injection) on this rope?
	int32     bHasOverrides = 0;
	// Number of box(OBB) (if 0, there is no box collision). Pad2 slot reuse.
	int32     NumBoxes = 0;
	// Number of convex (plane set) (if 0, no convex collision). Pad3 slot reuse.
	int32     NumConvexes = 0;
	// Angle-allowed bending: If straightness ≤ this value, straightening force is 0. Reuse Pad4 slot.
	float     BendReleaseRatio = 0.70f;
	// If straightness ≥ this value, the straightening force is 100%.
	float     BendFullRatio    = 0.92f;
	// Strain limiting Maximum stretch multiplier (<1=disabled). Pad5 slot reuse.
	float     MaxStretchRatio  = 1.5f;
	int32     Pad6 = 0;
	int32     Pad7 = 0;
	FVector4f Gravity;
	FVector4f PinPrev;
	FVector4f PinTarget;
};
static_assert(sizeof(FRopeGPUParamsGPU) % 16 == 0, "FRopeGPUParamsGPU must be 16-byte aligned to match HLSL structured buffer.");

// 1:1 mirror with HLSL FRopeCapsule. xyz=segment endpoint, B.w=radius, PrevB.w=InvDeltaTime (static if 0).
struct FRopeCapsuleGPU
{
	FVector4f A;
	// w = Radius
	FVector4f B;
	// previous frame endpoint (surface velocity drag/substep relative motion). If static, packing is filled with A/B.
	FVector4f PrevA;
	// w = InvDeltaTime
	FVector4f PrevB;
};
static_assert(sizeof(FRopeCapsuleGPU) % 16 == 0, "FRopeCapsuleGPU must be 16-byte aligned to match HLSL structured buffer.");

// 1:1 mirror with HLSL FRopeBox. box(OBB): world center + quat + half width + previous frame center/rot(dynamic surface velocity).
struct FRopeBoxGPU
{
	// xyz, w = InvDeltaTime(1/framedt; static if 0)
	FVector4f Center;
	// quat (x,y,z,w)
	FVector4f Rot;
	// xyz
	FVector4f HalfExtents;
	// xyz — Center of previous frame (if static, packing is filled with Center)
	FVector4f PrevCenter;
	// quat — rotate previous frame
	FVector4f PrevRot;
};
static_assert(sizeof(FRopeBoxGPU) % 16 == 0, "FRopeBoxGPU must be 16-byte aligned to match HLSL structured buffer.");

// 1:1 mirror with HLSL FRopeConvex. Flat full offset/count + local AABB + rigid body(curr/prev). Planes are separate in ConvexPlanes(local).
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

// 1:1 mirror with HLSL FRopeSDFVolume. bone local grid header (distance from DistOffset in SDFDistances buffer).
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

// 1:1 mirror with HLSL FRopeSDFCollider. Volume index + bone → world transform (quat/trans/scale, avoiding matrix layout).
struct FRopeSDFColliderGPU
{
	int32     VolumeIndex;
	int32     Pad0 = 0;
	int32     Pad1 = 0;
	int32     Pad2 = 0;
	// quat (x,y,z,w) — current frame
	FVector4f Rotation;
	// xyz
	FVector4f Translation;
	// xyz
	FVector4f Scale;
	// quat (x,y,z,w) — previous frame (for CCD relative motion/surfacevelocity)
	FVector4f PrevRotation;
	// xyz, w = InvDeltaTime(1/framedt; static if 0)
	FVector4f PrevTranslation;
};
static_assert(sizeof(FRopeSDFColliderGPU) % 16 == 0, "FRopeSDFColliderGPU must be 16-byte aligned to match HLSL structured buffer.");

class FRopeXPBDSolveCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeXPBDSolveCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeXPBDSolveCS, FGlobalShader);

	// node bucket = numthreads/groupshared size. The value is soon passed to .usf as ROPE_MAX_NODES define.
	class FNodeBucket : SHADER_PERMUTATION_SPARSE_INT("ROPE_MAX_NODES", 64, 128, 256, 512);
	// A permutation that incorporates GDF world collision as a substep constraint. GDF header include + View/GDF binding only when on.
	// off (default, step path without view) does not reference GDF → is compiled as before without view.
	class FGDFDim : SHADER_PERMUTATION_BOOL("ROPE_USE_GDF");
	using FPermutationDomain = TShaderPermutationDomain<FNodeBucket, FGDFDim>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(uint32, NumRopes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeGPUParams>, Params)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeCapsule>, Capsules)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, SDFDistances)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFVolume>, SDFVolumes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeSDFCollider>, SDFColliders)
		// box solve. detection CS uses the same buffer format in a separate parameter structure.
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeBox>, Boxes)
		// convex — solve only.
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeConvex>, Convexes)
		// convex plane flat pool.
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, ConvexPlanes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OverrideFlags)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, OverridePositions)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, OverridePrevPositions)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, OverrideInvMass)
		// G0: RW because override persists mass mask.
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, InvMass)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, Positions)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, PrevPositions)
		// tension readback (last substep segment λ).
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutLambdaDist)
		// --- GDF integration path (only referenced by shader when FGDFDim on; unused when off → unbound allowed).
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
		// ROPE_MAX_NODES(groupshared) automatically sets the FNodeBucket dimension. Set ROPE_THREADS(numthreads) to the same bucket.
		const FPermutationDomain PermutationVector(Parameters.PermutationId);
		OutEnvironment.SetDefine(TEXT("ROPE_THREADS"), PermutationVector.Get<FNodeBucket>());
	}
};

// The file is Shaders/Private/RopeXPBD.usf, and the virtual path is /Plugin/DynamicRope -> Shaders, so /Private/ is included.
IMPLEMENT_GLOBAL_SHADER(FRopeXPBDSolveCS, "/Plugin/DynamicRope/Private/RopeXPBD.usf", "RopeXPBDSolveCS", SF_Compute);

// 1:1 mirror with HLSL FRopeGPUContact (RopeXPBD.usf). 2 slots per node (actual/predictive). 16 byte alignment.
// Penetration packs into WorldPoint.W (maintains alignment).
struct FRopeGPUContactGPU
{
	int32     bHit;
	int32     ColliderType;
	int32     ColliderIndex;
	int32     Source;
	// xyz contact point, w Penetration
	FVector4f WorldPoint;
	FVector4f Normal;
	FVector4f SurfaceVel;
};
static_assert(sizeof(FRopeGPUContactGPU) % 16 == 0, "FRopeGPUContactGPU must be 16-byte aligned to match HLSL structured buffer.");

// contact detection compute (G3). After solving, sweep the resident location and record the deepest contact per node in OutContacts.
// Separate file (RopeContactDetect.usf) — Only the collider model/query (RopeColliderCommon.ush) is shared with the solve shader.
class FRopeContactDetectCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FRopeContactDetectCS);
	SHADER_USE_PARAMETER_STRUCT(FRopeContactDetectCS, FGlobalShader);

	// node bucket = size numthreads (detection kernel is not groupshared — only scales numthreads). Same bucket set as solve.
	class FNodeBucket : SHADER_PERMUTATION_SPARSE_INT("ROPE_MAX_NODES", 64, 128, 256, 512);
	using FPermutationDomain = TShaderPermutationDomain<FNodeBucket>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(int32, DetectNumNodes)
		SHADER_PARAMETER(int32, DetectNumCapsules)
		SHADER_PARAMETER(int32, DetectNumSDF)
		SHADER_PARAMETER(int32, DetectNumBoxes)
		SHADER_PARAMETER(int32, DetectNumConvexes)
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
		// wrapable box detection (static boxes are truncated with NumDetectBoxes).
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeBox>, Boxes)
		// wrapable convex detection (static convex truncated by DetectNumConvexes).
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FRopeConvex>, Convexes)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, ConvexPlanes)
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
		// ROPE_MAX_NODES is automatically set to FNodeBucket dimension. Set ROPE_THREADS(numthreads) to the same bucket.
		const FPermutationDomain PermutationVector(Parameters.PermutationId);
		OutEnvironment.SetDefine(TEXT("ROPE_THREADS"), PermutationVector.Get<FNodeBucket>());
	}
};

IMPLEMENT_GLOBAL_SHADER(FRopeContactDetectCS, "/Plugin/DynamicRope/Private/RopeContactDetect.usf", "RopeContactDetectCS", SF_Compute);

// ---------------------------------------------------------------------------------------------------
// Resident State Definitions
// ---------------------------------------------------------------------------------------------------

// render thread only. rope 1 persistent GPU buffer + readback (single in-Flight, consume and then rearm).
struct FRopeResidentRope
{
	TRefCountPtr<FRDGPooledBuffer> PosBuf;
	TRefCountPtr<FRDGPooledBuffer> PrevBuf;
	TRefCountPtr<FRDGPooledBuffer> InvMassBuf;
	int32  NumNodes = 0;
	// Last seeded generation (reseeded if different)
	uint32 Generation = 0xFFFFFFFFu;
	// Flag used for GDF permutation selection (solve-substep GDF collision in view expansion path). The collision parameter (radius/friction) is
	// It is passed directly to solve CS as a Params buffer (FRopeGPUParams), so it does not need to reside here.
	bool  bUseWorldGDF = false;
	FRHIGPUBufferReadback* PosReadback = nullptr;
	FRHIGPUBufferReadback* PrevReadback = nullptr;
	// Is the readback copy enqueued and waiting for the result?
	bool bReadbackArmed = false;

	// tension(λ) readback: solve(NumSub>0) Armed only in the frame (does not emit 0 in the override-only frame and maintains the previous value).
	// LambdaFixedDt = substep dt at the time of arming — used for F = max(0,-λ)/h² conversion when consumed.
	FRHIGPUBufferReadback* LambdaReadback = nullptr;
	bool  bLambdaArmed = false;
	float LambdaFixedDt = 0.0f;
	// PosBuf StructuredBuffer<float4> SRV, for the render tube. Invalidated on a reseed.
	FShaderResourceViewRHIRef PosSRV;

	// (SDF volume grid/header resident moved from per-rope to global FRopeGlobalSDFCache — resident once per VolumeKey.
	//  rope only uploads the instance (bone transform) array every frame. Globalization eliminates duplicate upload/collection churn re-upload for each rope.)

	// contact detection (G3): actual/predictive 2 slot output per node buffer (resident, regenerated only when the number of nodes changes) + readback.
	TRefCountPtr<FRDGPooledBuffer> ContactBuf;
	FRHIGPUBufferReadback* ContactReadback = nullptr;
	// Attribution signature of dispatch armed with contact readback — sent in results upon consumption (basis for attribution check).
	uint32 ContactAttribSig = 0;
	bool bContactArmed = false;
};

// GT<->RT shared results. RT fills and GT GetLatest reads under lock.
struct FRopeResidentSharedResults
{
	FCriticalSection Lock;
	TMap<uint32, FRopeResidentLatest> Map;
	// G3: contact detection result (GetLatestContacts).
	TMap<uint32, FRopeResidentContacts> Contacts;
	// Simulation time (seconds) of the pending step that could not be consumed and was replaced. RT stacks and GT drains with DrainDroppedSimTime.
	TMap<uint32, float> DroppedSimTime;
	// wrap Last authoritative snapshot of handoff synchronous readback. ReadbackNow starts from RT to the previous pending step.
	// After consumption, this map and Map are promoted together. The frame cache can obscure new pending steps in the same frame.
	// Do not use (handoff is a rare event and accuracy is a priority).
	TMap<uint32, FRopeResidentLatest> HandoffSnapshots;
};

// global SDF volume cache (RT only). Baked voxel data is static per VolumeKey, so it is used only once regardless of rope/frame.
// dequant+Upload and reside. Even if the rope's close volume set shakes every frame (Dragon: collider no cap+mesh unit
// broad phase) distance reupload becomes 0 — rope only uploads the array of instances (bone transform) every frame. index/
// The offset is not static within the frame, so existing references are not broken (new volumes are appended). Infinite growth is generational
// Prevent with rebuild — If unreferenced volumes accumulate for a long time, the cache is compressed and rebuilt with only the live set (the instance array is
// frame KeyToIndex recheck index relocation safe; Compression is a copy of the cached CpuDist slice, so source re-dequant is not required).
struct FRopeGlobalSDFCache
{
	// VolumeKey (stable identifier) ​​-> global volume index (= SDFVolumes index; header has CpuDist offset).
	TMap<uint64, int32> KeyToIndex;
	// VolumeKey -> Last referenced RT frame (for rebuild eviction check). A set of keys, such as KeyToIndex.
	TMap<uint64, uint64> KeyLastUsedFrame;
	// CPU source (concatenated dequant float + header). When appending/rebuilding a new volume, it is compressed leaving only the live.
	TArray<float>             CpuDist;
	TArray<FRopeSDFVolumeGPU> CpuVol;
	// Resident GPU buffer (external, maintained between frames). Recreate+upload only when dirty (new append or rebuild).
	TRefCountPtr<FRDGPooledBuffer> DistBuf;
	TRefCountPtr<FRDGPooledBuffer> VolBuf;
	bool bDirty = false;
	// RT frame counter (Ensure increases by 1 time per frame). KeyLastUsedFrame stamp/eviction check's clock.
	uint64 FrameCounter = 0;
};

// pimpl: persistent buffer map (RT only) + shared result (GT<->RT). Hide the RDG/RHI type from the header.
struct FRopeGPUSolver::FImpl
{
	// Accessed only on the render thread.
	TMap<uint32, FRopeResidentRope>                          RtRopes;
	TSharedRef<FRopeResidentSharedResults, ESPMode::ThreadSafe> Results
		= MakeShared<FRopeResidentSharedResults, ESPMode::ThreadSafe>();

	// Global SDF volume resident (shared by all ropes — RopeEnsureGlobalSDFVolumes are updated once per-frame).
	FRopeGlobalSDFCache GlobalSDF;

	// This frame steps accumulated as GDF path (EnqueueSteps). View extension consumes on DispatchPending_RenderThread. RT only.
	TArray<FRopeGPUResidentStep> PendingSteps;
	// rope held because ReadbackNow cannot run without View/GDF. of other ropes until the next scene is dispatched.
	// EnqueueSteps replacement semantics protect the step from being discarded.
	TSet<uint32> HandoffGDFBlockedRopes;
};

bool RopeGPU::IsRuntimeSupported()
{
	// Since the kernel is compiled only with SM5 guard (ShouldCompilePermutation of each CS), the feature level below it is
	// There is no permutation itself — the previous check, which only looked at the presence or absence of RHI, required a shader that was not present on mobile.
	// For check basis and detailed background, refer to the comments in the RopeGPUSolver.h declaration.
	if (GDynamicRHI == nullptr || !FApp::CanEverRender())
	{
		return false;
	}
	if (GMaxRHIFeatureLevel < ERHIFeatureLevel::SM5)
	{
		// Since quiet CPU fallback can easily be misunderstood as a performance issue, it is notified once at runtime.
		static bool bWarned = false;
		if (!bWarned)
		{
			bWarned = true;
			UE_LOG(LogDynamicRopeGPU, Warning,
				TEXT("GPU rope path disabled — feature level is below SM5 (%s). Falling back to the CPU solver and tube."),
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
	// Drain all in-Flight render commands before touching the resident buffer/readback → Clean up directly in GT afterwards. OK.
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
	// Any unconsumed steps are also discarded — if any, the next dispatch revives the resident map that was just emptied.
	Impl->PendingSteps.Reset();
	Impl->HandoffGDFBlockedRopes.Reset();
	// global SDF resident buffer/cache release (TRefCountPtr auto-release).
	Impl->GlobalSDF = FRopeGlobalSDFCache{};
}

void FRopeGPUSolver::ReleaseRope(uint32 RopeId)
{
	// Shared results are immediately removed from GT.
	{
		FScopeLock SL(&Impl->Results->Lock);
		Impl->Results->Map.Remove(RopeId);
		Impl->Results->Contacts.Remove(RopeId);
		Impl->Results->DroppedSimTime.Remove(RopeId);
		Impl->Results->HandoffSnapshots.Remove(RopeId);
	}
	// Free persistent buffer/readback on render thread (capture this — destructor flushes, so lifetime safe).
	ENQUEUE_RENDER_COMMAND(RopeGPUReleaseRope)(
		[this, RopeId](FRHICommandListImmediate&)
		{
			// Removes pending steps that have not yet been consumed. If left, the subsequent view expansion dispatch will occur.
			// **Revive** the RopeId just cleared with FindOrAdd in RunSteps_RenderThread and re-create the resident buffer/readback.
			// The subject who created and released the rope has already disappeared, so it remains in VRAM until the end of the world.
			// (EnqueueSteps has replacement semantics, so “it disappears anyway in the next frame” does not hold —
			// Replacement does not occur in frames where there is no scene render.)
			Impl->PendingSteps.RemoveAll(
				[RopeId](const FRopeGPUResidentStep& Step) { return Step.RopeId == RopeId; });
			Impl->HandoffGDFBlockedRopes.Remove(RopeId);

			if (FRopeResidentRope* Resident = Impl->RtRopes.Find(RopeId))
			{
				delete Resident->PosReadback;
				delete Resident->PrevReadback;
				delete Resident->LambdaReadback;
				delete Resident->ContactReadback;
				Impl->RtRopes.Remove(RopeId);
			}
		});
}

void FRopeGPUSolver::DrainDroppedSimTime(TMap<uint32, float>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	// Retrieval is a one-time use (if you do the same amount of time twice, you'll get ahead) — move and empty.
	Out = MoveTemp(Impl->Results->DroppedSimTime);
	Impl->Results->DroppedSimTime.Reset();
}

void FRopeGPUSolver::GetLatest(TMap<uint32, FRopeResidentLatest>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	// Small data — copy every frame. (The caller maintains the accumulated amount by copying instead of swapping)
	Out = Impl->Results->Map;
}

void FRopeGPUSolver::GetLatestContacts(TMap<uint32, FRopeResidentContacts>& Out)
{
	FScopeLock SL(&Impl->Results->Lock);
	// Maximum actual/predictive per node is 2, so it is small — copied every frame.
	Out = Impl->Results->Contacts;
}

bool FRopeGPUSolver::ReadbackNow(uint32 RopeId, TArray<FVector>& OutPositions, TArray<FVector>& OutPrevPositions, uint32& OutGeneration)
{
	// Game-thread blocking readback. EnqueueSteps only fills the render thread's pending queue, so a plain
	// resident copy would return a pose older than the Flight step that just preceded it. Using the render
	// command ordering, this executes that RopeId's pending non-GDF step first in a dedicated graph, then
	// copies and reads back the result in the same command.
	// Because the GDF step requires a valid Scene View/GDF, it is not executed with lean permutation here but is suspended.
	struct FImmediateReadbackResult
	{
		bool bSuccess = false;
		FRopeResidentLatest Snapshot;
	};
	const TSharedRef<FImmediateReadbackResult, ESPMode::ThreadSafe> Result =
		MakeShared<FImmediateReadbackResult, ESPMode::ThreadSafe>();

	ENQUEUE_RENDER_COMMAND(RopeGPUReadbackNow)(
		[this, RopeId, Result](FRHICommandListImmediate& RHICmdList)
		{
			check(IsInRenderingThread());

			bool bHasPendingForRope = false;
			bool bPendingNeedsSceneGDF = false;
			for (const FRopeGPUResidentStep& PendingStep : Impl->PendingSteps)
			{
				if (PendingStep.RopeId == RopeId)
				{
					bHasPendingForRope = true;
					bPendingNeedsSceneGDF |= PendingStep.bUseWorldGDF;
				}
			}
			if (bHasPendingForRope)
			{
				if (bPendingNeedsSceneGDF)
				{
					// Protect every queued temporal step for this rope until a scene graph can supply the GDF.
					Impl->HandoffGDFBlockedRopes.Add(RopeId);
					FScopeLock SL(&Impl->Results->Lock);
					Impl->Results->HandoffSnapshots.Remove(RopeId);
					return;
				}

				TArray<FRopeGPUResidentStep> ImmediateSteps;
				TArray<FRopeGPUResidentStep> RemainingSteps;
				ImmediateSteps.Reserve(Impl->PendingSteps.Num());
				RemainingSteps.Reserve(Impl->PendingSteps.Num());
				for (FRopeGPUResidentStep& PendingStep : Impl->PendingSteps)
				{
					if (PendingStep.RopeId == RopeId)
					{
						ImmediateSteps.Add(MoveTemp(PendingStep));
					}
					else
					{
						RemainingSteps.Add(MoveTemp(PendingStep));
					}
				}
				Impl->PendingSteps = MoveTemp(RemainingSteps);
				Impl->HandoffGDFBlockedRopes.Remove(RopeId);

				FRDGBuilder SolveGraph(RHICmdList);
				RunSteps_RenderThread(SolveGraph, ImmediateSteps, nullptr, nullptr, FVector3f::ZeroVector);
				SolveGraph.Execute();
			}

			FRopeResidentRope* Resident = Impl->RtRopes.Find(RopeId);
			if (!Resident || !Resident->PosBuf.IsValid() || !Resident->PrevBuf.IsValid() || Resident->NumNodes < 2)
			{
				return;
			}

			const int32 NumNodes = Resident->NumNodes;
			const uint32 NodeBytes = static_cast<uint32>(NumNodes) * sizeof(FVector4f);
			TUniquePtr<FRHIGPUBufferReadback> PosRb =
				MakeUnique<FRHIGPUBufferReadback>(TEXT("Rope.PosReadbackNow"));
			TUniquePtr<FRHIGPUBufferReadback> PrevRb =
				MakeUnique<FRHIGPUBufferReadback>(TEXT("Rope.PrevReadbackNow"));
			{
				// SolveGraph confirms it as an external buffer and registers/copies it in a new graph.
				FRDGBuilder CopyGraph(RHICmdList);
				AddEnqueueCopyPass(CopyGraph, PosRb.Get(), CopyGraph.RegisterExternalBuffer(Resident->PosBuf), NodeBytes);
				AddEnqueueCopyPass(CopyGraph, PrevRb.Get(), CopyGraph.RegisterExternalBuffer(Resident->PrevBuf), NodeBytes);
				CopyGraph.Execute();
			}
			RHICmdList.SubmitAndBlockUntilGPUIdle();

			const FVector4f* SrcPos = static_cast<const FVector4f*>(PosRb->Lock(NodeBytes));
			const FVector4f* SrcPrev = static_cast<const FVector4f*>(PrevRb->Lock(NodeBytes));
			if (!SrcPos || !SrcPrev)
			{
				if (SrcPos) { PosRb->Unlock(); }
				if (SrcPrev) { PrevRb->Unlock(); }
				return;
			}

			FRopeResidentLatest Snapshot;
			Snapshot.NumNodes = NumNodes;
			Snapshot.Generation = Resident->Generation;
			Snapshot.Positions.SetNumUninitialized(NumNodes);
			Snapshot.PrevPositions.SetNumUninitialized(NumNodes);
			for (int32 NodeIndex = 0; NodeIndex < NumNodes; ++NodeIndex)
			{
				Snapshot.Positions[NodeIndex] = FVector(SrcPos[NodeIndex].X, SrcPos[NodeIndex].Y, SrcPos[NodeIndex].Z);
				Snapshot.PrevPositions[NodeIndex] = FVector(SrcPrev[NodeIndex].X, SrcPrev[NodeIndex].Y, SrcPrev[NodeIndex].Z);
			}
			PosRb->Unlock();
			PrevRb->Unlock();

			// Even if RunSteps failed to consume the existing async copy or rearmed a new copy, after the above idle
			// All can be drained safely. If you leave a flag, the next step will be faster than this authoritative snapshot.
			// You can promote old poses/contacts back to Results.
			auto DrainReadback = [](FRHIGPUBufferReadback* Readback, uint32 Bytes)
			{
				if (Readback && Readback->IsReady())
				{
					if (Readback->Lock(Bytes))
					{
						Readback->Unlock();
					}
				}
			};
			if (Resident->bReadbackArmed)
			{
				DrainReadback(Resident->PosReadback, NodeBytes);
				DrainReadback(Resident->PrevReadback, NodeBytes);
			}
			if (Resident->bLambdaArmed)
			{
				DrainReadback(Resident->LambdaReadback, static_cast<uint32>(NumNodes) * sizeof(float));
			}
			if (Resident->bContactArmed)
			{
				DrainReadback(Resident->ContactReadback,
					static_cast<uint32>(2 * NumNodes) * sizeof(FRopeGPUContactGPU));
			}
			Resident->bReadbackArmed = false;
			Resident->bLambdaArmed = false;
			Resident->bContactArmed = false;

			{
				FScopeLock SL(&Impl->Results->Lock);
				if (const FRopeResidentLatest* Existing = Impl->Results->Map.Find(RopeId))
				{
					if (Existing->Generation == Snapshot.Generation && Existing->NumNodes == Snapshot.NumNodes)
					{
						Snapshot.SegmentTension = Existing->SegmentTension;
					}
				}
				// Make GetLatest see the same authority pose so it doesn't roll back to the stale async mirror on the next GT Tick.
				Impl->Results->Map.Add(RopeId, Snapshot);
				Impl->Results->HandoffSnapshots.Add(RopeId, Snapshot);
				Impl->Results->Contacts.Remove(RopeId);
			}
			Result->Snapshot = MoveTemp(Snapshot);
			Result->bSuccess = true;
		});
	// GT waits until RT command completes (snapshot confirmed).
	FlushRenderingCommands();

	if (!Result->bSuccess || Result->Snapshot.NumNodes < 2)
	{
		return false;
	}
	OutPositions = Result->Snapshot.Positions;
	OutPrevPositions = Result->Snapshot.PrevPositions;
	OutGeneration = Result->Snapshot.Generation;
	return true;
}

FRHIShaderResourceView* FRopeGPUSolver::GetResidentPositionSRV_RenderThread(uint32 RopeId, int32& OutNumNodes,
	uint32& OutGeneration)
{
	check(IsInRenderingThread());
	OutNumNodes = 0;
	OutGeneration = 0;

	FRopeResidentRope* Resident = Impl->RtRopes.Find(RopeId);
	if (!Resident || !Resident->PosBuf.IsValid())
	{
		return nullptr;
	}
	OutNumNodes = Resident->NumNodes;
	OutGeneration = Resident->Generation;

	if (!Resident->PosSRV.IsValid())
	{
		// PosBuf is boned as StructuredBuffer<float4>(stride 16) — structured SRV.
		FRHICommandListImmediate& RHICmdList = FRHICommandListExecutor::GetImmediateCommandList();
		Resident->PosSRV = RHICmdList.CreateShaderResourceView(Resident->PosBuf->GetRHI(),
			FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Structured));
	}
	return Resident->PosSRV.GetReference();
}

// Phase 2c: GDF shader parameter configuration. Engine SetupGlobalDistanceFieldParameters (all) is not RENDERER_API
// Link not possible → Use inline _Minimal and directly reinforce the 3 CoverageAtlas textures + samplers it omits.
// If GDF is null/clipmap 0, bind black volume texture and OutValid=0 (shader skips GDF block).
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
	// The 3 samplers are not filled with _Minimal (black samples when not set). Always set.
	Out.GlobalDistanceFieldPageAtlasTextureSampler     = TStaticSamplerState<SF_Trilinear, AM_Wrap,  AM_Wrap,  AM_Wrap >::GetRHI();
	Out.GlobalDistanceFieldCoverageAtlasTextureSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap,  AM_Wrap,  AM_Wrap >::GetRHI();
	Out.GlobalDistanceFieldMipTextureSampler           = TStaticSamplerState<SF_Trilinear, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI();
}

// ===== RunSteps_RenderThread disassembly =========================================================================
// The helpers below are the step-by-step body of RunSteps_RenderThread (same operation — code movement). CPU arrangement for upload is all
// Bind to graph lifetime with GraphBuilder.AllocObject<TArray<...>>() (survives until Execute + disappears when graph is torn down)
// — No need for function-scoped keep-alive containers (formerly K* arrays) and the "prevent reallocation to Reserve" convention.

// rope 1 graph build intermediate output (RDG handle + valid count). The orchestration loop passes between stages.
struct FRopeStepBuild
{
	FRDGBufferRef PosRDG = nullptr;
	FRDGBufferRef PrevRDG = nullptr;
	FRDGBufferRef InvMassRDG = nullptr;
	// Whether to reseed this frame (initial/node number/generation change).
	bool bSeed = false;
	// Whether G0 override is valid (flag length == number of nodes).
	bool bHasOverrides = false;

	FRDGBufferRef CapsulesBuf = nullptr;
	FRDGBufferRef SDFDistBuf = nullptr;
	FRDGBufferRef SDFVolBuf = nullptr;
	FRDGBufferRef SDFColBuf = nullptr;
	FRDGBufferRef BoxesBuf = nullptr;
	FRDGBufferRef ConvexBuf = nullptr;
	FRDGBufferRef ConvexPlanesBuf = nullptr;
	// Dummy padding *before* valid count (for shader count).
	int32 NumValidCaps = 0;
	int32 NumValidSDFCol = 0;
	int32 NumValidBoxes = 0;
	int32 NumValidConvexes = 0;

	FRDGBufferRef OvFlagsBuf = nullptr;
	FRDGBufferRef OvPosBuf = nullptr;
	FRDGBufferRef OvPrevBuf = nullptr;
	FRDGBufferRef OvInvBuf = nullptr;
};

// Loop 1: Just before frame readback consume(immediate Lock — Process *before* RDG builder configuration to create immediate RHI and
// Avoid interleaving of open graphs. Lock is legal because it is a render thread, and there is no stall because it is an IsReady gate).
static void RopeConsumeReadbacks(TMap<uint32, FRopeResidentRope>& RtRopes,
	FRopeResidentSharedResults& Results, const TArray<FRopeGPUResidentStep>& Steps)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_ConsumeReadbacks);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_ConsumeReadbacks);
	for (const FRopeGPUResidentStep& Step : Steps)
	{
		FRopeResidentRope* ResidentPtr = RtRopes.Find(Step.RopeId);
		if (!ResidentPtr)
		{
			continue;
		}
		FRopeResidentRope& Resident = *ResidentPtr;
		const bool bWillReseed = !Resident.PosBuf.IsValid() || Resident.NumNodes != Step.NumNodes
			|| Resident.Generation != Step.Generation;
		// In the reseeding frame, the previous readback is stale, so it is ignored (both location/contact). Disarm and re-arm the next dispatch.
		if (bWillReseed)
		{
			Resident.bReadbackArmed = false;
			Resident.bLambdaArmed = false;
			Resident.bContactArmed = false;
			continue;
		}

		const int32 NumNodes = Resident.NumNodes;

		// Location readback consume (only when armed/ready) — Even if it fails, contact consume proceeds independently.
		TArray<FVector> TmpPos, TmpPrev;
		bool bHavePos = false;
		if (Resident.bReadbackArmed && Resident.PosReadback && Resident.PrevReadback
			&& Resident.PosReadback->IsReady() && Resident.PrevReadback->IsReady())
		{
			const uint32 Bytes = (uint32)NumNodes * sizeof(FVector4f);
			TmpPos.SetNumUninitialized(NumNodes);
			TmpPrev.SetNumUninitialized(NumNodes);
			if (const FVector4f* Src = (const FVector4f*)Resident.PosReadback->Lock(Bytes))
			{
				for (int32 k = 0; k < NumNodes; ++k) { TmpPos[k] = FVector(Src[k].X, Src[k].Y, Src[k].Z); }
				Resident.PosReadback->Unlock();
			}
			if (const FVector4f* Src = (const FVector4f*)Resident.PrevReadback->Lock(Bytes))
			{
				for (int32 k = 0; k < NumNodes; ++k) { TmpPrev[k] = FVector(Src[k].X, Src[k].Y, Src[k].Z); }
				Resident.PrevReadback->Unlock();
			}
			// Consumed — rearmed in dispatch block.
			Resident.bReadbackArmed = false;
			bHavePos = true;
		}

		// tension(λ) readback: independent of position consume (armed only in solve frame). Force converted to dt when armed.
		TArray<float> TmpTension;
		bool bHaveTension = false;
		if (Resident.bLambdaArmed && Resident.LambdaReadback && Resident.LambdaReadback->IsReady())
		{
			const uint32 LambdaBytes = (uint32)NumNodes * sizeof(float);
			if (const float* Src = (const float*)Resident.LambdaReadback->Lock(LambdaBytes))
			{
				// Number of segments = NumNodes-1 (the last slot in the kernel is always 0). F = max(0,-λ)/h² — Same conversion as CPU Step.
				const float InvDt2 = (Resident.LambdaFixedDt > 1e-6f)
					? (1.0f / (Resident.LambdaFixedDt * Resident.LambdaFixedDt)) : 0.0f;
				TmpTension.SetNumUninitialized(NumNodes - 1);
				for (int32 k = 0; k < NumNodes - 1; ++k)
				{
					TmpTension[k] = FMath::Max(0.0f, -Src[k]) * InvDt2;
				}
				Resident.LambdaReadback->Unlock();
				bHaveTension = true;
			}
			// Consumed — rearmed in dispatch block.
			Resident.bLambdaArmed = false;
		}

		// contact detection readback(G3): independent of location consume (detection may not be present since only Flight is armed).
		TArray<FRopeGPUContactResult> TmpContacts;
		bool bHaveContacts = false;
		if (Resident.bContactArmed && Resident.ContactReadback && Resident.ContactReadback->IsReady())
		{
			// 2 slots per node: [0..NumNodes) actual, [NumNodes..2*NumNodes) predictive.
			// slot index % NumNodes = node index.
			const uint32 ContactBytes = (uint32)(2 * NumNodes) * sizeof(FRopeGPUContactGPU);
			if (const FRopeGPUContactGPU* Src =
				(const FRopeGPUContactGPU*)Resident.ContactReadback->Lock(ContactBytes))
			{
				for (int32 Slot = 0; Slot < 2 * NumNodes; ++Slot)
				{
					if (Src[Slot].bHit == 0)
					{
						continue;
					}
					FRopeGPUContactResult Contact;
					Contact.NodeIndex       = Slot % NumNodes;
					Contact.ColliderType    = Src[Slot].ColliderType;
					Contact.ColliderIndex   = Src[Slot].ColliderIndex;
					Contact.Source          = (uint8)Src[Slot].Source;
					// penetration packed in w.
					Contact.Penetration     = Src[Slot].WorldPoint.W;
					Contact.WorldPoint      = FVector(Src[Slot].WorldPoint.X, Src[Slot].WorldPoint.Y, Src[Slot].WorldPoint.Z);
					Contact.Normal          = FVector(Src[Slot].Normal.X, Src[Slot].Normal.Y, Src[Slot].Normal.Z);
					Contact.SurfaceVelocity = FVector(Src[Slot].SurfaceVel.X, Src[Slot].SurfaceVel.Y, Src[Slot].SurfaceVel.Z);
					TmpContacts.Add(Contact);
				}
				Resident.ContactReadback->Unlock();
				bHaveContacts = true;
			}
			// Consumed — rearmed in dispatch block.
			Resident.bContactArmed = false;
		}

		if (!bHavePos && !bHaveContacts && !bHaveTension)
		{
			// No number of frames this time.
			continue;
		}

		// The lock section is only map assignment (readback lock ends at the top) → GT GetLatest blocking is minimized.
		FScopeLock SL(&Results.Lock);
		if (bHavePos || bHaveTension)
		{
			FRopeResidentLatest& Latest = Results.Map.FindOrAdd(Step.RopeId);
			if (bHavePos)
			{
				Latest.Positions     = MoveTemp(TmpPos);
				Latest.PrevPositions = MoveTemp(TmpPrev);
				Latest.NumNodes      = NumNodes;
				// generation promotion only with position (preventing stale position promotion immediately after reseeding).
				Latest.Generation    = Resident.Generation;
			}
			if (bHaveTension)
			{
				// tension does not touch entry generation — if it arrives before the position immediately after reseeding,
				// GT rejects one frame (with the old generation), and when the position catches up, it is consumed together.
				Latest.SegmentTension = MoveTemp(TmpTension);
			}
		}
		if (bHaveContacts)
		{
			FRopeResidentContacts& LatestContacts = Results.Contacts.FindOrAdd(Step.RopeId);
			LatestContacts.Contacts   = MoveTemp(TmpContacts);
			LatestContacts.Generation = Resident.Generation;
			LatestContacts.AttribSig  = Resident.ContactAttribSig;
		}
	}
}

// Securing resident Pos/Prev/InvMass: If reseeding (initial/node number/generation change), seed upload + external buffer conversion,
// Otherwise, register the existing persistent buffer in the graph. Fill in Build.PosRDG/PrevRDG/InvMassRDG/bSeed.
static void RopeEnsureResidentBuffers(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step,
	FRopeResidentRope& Resident, FRopeStepBuild& Build)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_EnsureBuffers);
	const int32 NumNodes = Step.NumNodes;
	Build.bSeed = !Resident.PosBuf.IsValid() || Resident.NumNodes != NumNodes
		|| Resident.Generation != Step.Generation;

	if (Build.bSeed)
	{
		const bool bHaveSeed = Step.SeedPositions.Num() == NumNodes
			&& Step.SeedPrevPositions.Num() == NumNodes && Step.InvMass.Num() == NumNodes;
		TArray<FVector4f>& SeedPos  = *GraphBuilder.AllocObject<TArray<FVector4f>>();
		TArray<FVector4f>& SeedPrev = *GraphBuilder.AllocObject<TArray<FVector4f>>();
		TArray<float>&     SeedInv  = *GraphBuilder.AllocObject<TArray<float>>();
		SeedPos.SetNumUninitialized(NumNodes);
		SeedPrev.SetNumUninitialized(NumNodes);
		SeedInv.SetNumUninitialized(NumNodes);
		for (int32 k = 0; k < NumNodes; ++k)
		{
			const FVector Position = bHaveSeed ? Step.SeedPositions[k] : FVector::ZeroVector;
			const FVector PrevPosition = bHaveSeed ? Step.SeedPrevPositions[k] : FVector::ZeroVector;
			SeedPos[k] = FVector4f((float)Position.X, (float)Position.Y, (float)Position.Z, 0.0f);
			SeedPrev[k] = FVector4f((float)PrevPosition.X, (float)PrevPosition.Y, (float)PrevPosition.Z, 0.0f);
			SeedInv[k] = bHaveSeed ? Step.InvMass[k] : 1.0f;
		}
		Build.PosRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Pos"), sizeof(FVector4f), NumNodes,
			SeedPos.GetData(), (uint64)NumNodes * sizeof(FVector4f));
		Build.PrevRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Prev"), sizeof(FVector4f), NumNodes,
			SeedPrev.GetData(), (uint64)NumNodes * sizeof(FVector4f));
		Build.InvMassRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.InvMass"), sizeof(float), NumNodes,
			SeedInv.GetData(), (uint64)NumNodes * sizeof(float));
		Resident.PosBuf = GraphBuilder.ConvertToExternalBuffer(Build.PosRDG);
		Resident.PrevBuf = GraphBuilder.ConvertToExternalBuffer(Build.PrevRDG);
		Resident.InvMassBuf = GraphBuilder.ConvertToExternalBuffer(Build.InvMassRDG);
		Resident.NumNodes = NumNodes;
		Resident.Generation = Step.Generation;
		// The readback immediately before reseeding is stale.
		Resident.bReadbackArmed = false;
		// New PosBuf creation → Cached SRV invalid (render will be regenerated next time).
		Resident.PosSRV.SafeRelease();
	}
	else
	{
		Build.PosRDG = GraphBuilder.RegisterExternalBuffer(Resident.PosBuf);
		Build.PrevRDG = GraphBuilder.RegisterExternalBuffer(Resident.PrevBuf);
		Build.InvMassRDG = GraphBuilder.RegisterExternalBuffer(Resident.InvMassBuf);
	}
}

// Capsule packing: flatten the step's world capsules into the GPU layout and upload them, filling Build.CapsulesBuf and NumValidCaps.
static void RopePackCapsules(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step, FRopeStepBuild& Build)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackCapsules);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_PackCapsules);
	TArray<FRopeCapsuleGPU>& CapsFlat = *GraphBuilder.AllocObject<TArray<FRopeCapsuleGPU>>();
	for (const FRopeGPUCapsule& Capsule : Step.Capsules)
	{
		FRopeCapsuleGPU GpuCapsule;
		GpuCapsule.A = FVector4f((float)Capsule.A.X, (float)Capsule.A.Y, (float)Capsule.A.Z, 0.0f);
		GpuCapsule.B = FVector4f((float)Capsule.B.X, (float)Capsule.B.Y, (float)Capsule.B.Z, Capsule.Radius);
		// If static(InvDt 0), prev=current — kernel can always compute lerp/displacement without prev validation branch.
		const bool bMoving = Capsule.InvDeltaTime > 0.0f;
		const FVector& PrevA = bMoving ? Capsule.PrevA : Capsule.A;
		const FVector& PrevB = bMoving ? Capsule.PrevB : Capsule.B;
		GpuCapsule.PrevA = FVector4f((float)PrevA.X, (float)PrevA.Y, (float)PrevA.Z, 0.0f);
		GpuCapsule.PrevB = FVector4f((float)PrevB.X, (float)PrevB.Y, (float)PrevB.Z, Capsule.InvDeltaTime);
		CapsFlat.Add(GpuCapsule);
	}

	// Valid count — confirmed *before* dummy padding. Structured buffer has elements >=1 — 1 empty dummy (not referenced by any node).
	Build.NumValidCaps = CapsFlat.Num();
	if (CapsFlat.Num() == 0) { CapsFlat.AddZeroed(1); }

	Build.CapsulesBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Capsules"),
		sizeof(FRopeCapsuleGPU), CapsFlat.Num(), CapsFlat.GetData(), (uint64)CapsFlat.Num() * sizeof(FRopeCapsuleGPU));
}

// box packing: step's box (OBB) → GPU layout flattening + upload. Fills Build.BoxesBuf/NumValidBoxes.
static void RopePackBoxes(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step, FRopeStepBuild& Build)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackBoxes);
	TArray<FRopeBoxGPU>& BoxesFlat = *GraphBuilder.AllocObject<TArray<FRopeBoxGPU>>();
	for (const FRopeGPUBox& Box : Step.Boxes)
	{
		FRopeBoxGPU GpuBox;
		// w=InvDt
		GpuBox.Center      = FVector4f((float)Box.Center.X, (float)Box.Center.Y, (float)Box.Center.Z, Box.InvDeltaTime);
		GpuBox.Rot         = FVector4f((float)Box.Rot.X, (float)Box.Rot.Y, (float)Box.Rot.Z, (float)Box.Rot.W);
		GpuBox.HalfExtents = FVector4f((float)Box.HalfExtents.X, (float)Box.HalfExtents.Y, (float)Box.HalfExtents.Z, 0.0f);
		// If static(InvDt 0), prev=current — kernel can always interpolate without prev validation branch (same as capsule packing).
		const bool bMoving = Box.InvDeltaTime > 0.0f;
		const FVector PrevCenter = bMoving ? Box.PrevCenter : Box.Center;
		const FQuat PrevRotation = bMoving ? Box.PrevRot : Box.Rot;
		GpuBox.PrevCenter = FVector4f((float)PrevCenter.X, (float)PrevCenter.Y, (float)PrevCenter.Z, 0.0f);
		GpuBox.PrevRot = FVector4f((float)PrevRotation.X, (float)PrevRotation.Y,
			(float)PrevRotation.Z, (float)PrevRotation.W);
		BoxesFlat.Add(GpuBox);
	}

	// Valid Count — Confirmed *before* dummy padding. Structured buffer has elements >=1 — 1 empty dummy (not referenced because NumBoxes=0).
	Build.NumValidBoxes = BoxesFlat.Num();
	if (BoxesFlat.Num() == 0) { BoxesFlat.AddZeroed(1); }

	Build.BoxesBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Boxes"),
		sizeof(FRopeBoxGPU), BoxesFlat.Num(), BoxesFlat.GetData(), (uint64)BoxesFlat.Num() * sizeof(FRopeBoxGPU));
}

// convex packing: step's convex → plane flat pool (ConvexPlanes) + upload header (Convexes). The plane of each convex is
// is attached to the pool and referenced with PlaneOffset/PlaneCount. Fill in Build.ConvexBuf/ConvexPlanesBuf/NumValidConvexes.
static void RopePackConvexes(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step, FRopeStepBuild& Build)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackConvexes);
	TArray<FRopeConvexGPU>& ConvFlat = *GraphBuilder.AllocObject<TArray<FRopeConvexGPU>>();
	TArray<FVector4f>&      PlaneFlat = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	for (const FRopeGPUConvex& Convex : Step.Convexes)
	{
		// If static(InvDt 0), prev=current — kernel can always interpolate without prev validation branch (same as box/capsule packing).
		const bool bMoving = Convex.InvDeltaTime > 0.0f;
		const FQuat PrevRotation = bMoving ? Convex.PrevRot : Convex.Rot;
		const FVector PrevTranslation = bMoving ? Convex.PrevTrans : Convex.Trans;
		FRopeConvexGPU GpuConvex;
		GpuConvex.PlaneOffset = PlaneFlat.Num();
		GpuConvex.PlaneCount = Convex.PlaneCount;
		GpuConvex.LocalBoundsCenter = FVector4f((float)Convex.LocalBoundsCenter.X,
			(float)Convex.LocalBoundsCenter.Y, (float)Convex.LocalBoundsCenter.Z, 0.0f);
		// w=InvDt
		GpuConvex.LocalBoundsExtent = FVector4f((float)Convex.LocalBoundsExtent.X,
			(float)Convex.LocalBoundsExtent.Y, (float)Convex.LocalBoundsExtent.Z, Convex.InvDeltaTime);
		GpuConvex.Rot = FVector4f((float)Convex.Rot.X, (float)Convex.Rot.Y,
			(float)Convex.Rot.Z, (float)Convex.Rot.W);
		GpuConvex.Trans = FVector4f((float)Convex.Trans.X, (float)Convex.Trans.Y, (float)Convex.Trans.Z, 0.0f);
		GpuConvex.PrevRot = FVector4f((float)PrevRotation.X, (float)PrevRotation.Y,
			(float)PrevRotation.Z, (float)PrevRotation.W);
		GpuConvex.PrevTrans = FVector4f((float)PrevTranslation.X, (float)PrevTranslation.Y,
			(float)PrevTranslation.Z, 0.0f);
		ConvFlat.Add(GpuConvex);
		const int32 Start = Convex.PlaneOffset;
		for (int32 PlaneIndex = 0; PlaneIndex < Convex.PlaneCount; ++PlaneIndex)
		{
			const FVector4& Plane = Step.ConvexPlanes[Start + PlaneIndex];
			PlaneFlat.Add(FVector4f((float)Plane.X, (float)Plane.Y, (float)Plane.Z, (float)Plane.W));
		}
	}

	// Valid count — confirmed *before* dummy padding. Structured buffer has elements >=1 — 1 empty dummy (not referenced because NumConvexes=0).
	Build.NumValidConvexes = ConvFlat.Num();
	if (ConvFlat.Num() == 0) { ConvFlat.AddZeroed(1); }
	if (PlaneFlat.Num() == 0) { PlaneFlat.AddZeroed(1); }

	Build.ConvexBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Convexes"),
		sizeof(FRopeConvexGPU), ConvFlat.Num(), ConvFlat.GetData(), (uint64)ConvFlat.Num() * sizeof(FRopeConvexGPU));
	Build.ConvexPlanesBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.ConvexPlanes"),
		sizeof(FVector4f), PlaneFlat.Num(), PlaneFlat.GetData(), (uint64)PlaneFlat.Num() * sizeof(FVector4f));
}

// Guaranteed global SDF volume residency (once per-frame, *before* the rope loop). In this frame, the SDF collider of all steps is still
// Only VolumeKeys not in the cache are dequantized and appended to the global array, and the resident buffer is stored only in frames where a new volume is created (dirty).
// Reupload (as many volumes per session, then 0). Return: global distance/header RDG buffer to bind to this frame (which
// If there is neither rope nor SDF, 1 dummy). This eliminates the per-rope/set-churn reupload (formerly 50ms of per-rope VolSig gate).
// generational rebuild parameters.
//  - EvictAfterFrames: If this number of consecutive unreferenced objects are evicted (hysteresis — no rebuild due to culling boundary blinking).
//  - RebuildReclaimFrac: Rebuild only when dead dist byte ratio is above this (prevent MB reupload with minor recall).
static constexpr uint64 GRopeSDFEvictAfterFrames = 600;   // ~10s @ 60fps
static constexpr float  GRopeSDFRebuildReclaimFrac = 0.25f;

// Rebuild cache compressed with only live (see recent EvictAfterFrames) volumes. The dequantized CpuDist slice is intact.
// Copy (source not required) and give a new index/offset → Re-upload the resident buffer once as bDirty. The instance array is
// Because frame KeyToIndex is re-queried, index relocation automatically reflects the next pack step (no reference damage).
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
			continue;   // long unreferenced — drop.
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
	Cache.bDirty           = true;   // Re-upload the compressed buffer in the upload path below once.
}

static void RopeEnsureGlobalSDFVolumes(FRDGBuilder& GraphBuilder, const TArray<FRopeGPUResidentStep>& Steps,
	FRopeGlobalSDFCache& Cache, FRDGBufferRef& OutDistRDG, FRDGBufferRef& OutVolRDG)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_EnsureGlobalSDF);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_EnsureGlobalSDF);
	const uint64 Frame = ++Cache.FrameCounter;   // Once per-frame (Ensure once per RunSteps).
	for (const FRopeGPUResidentStep& Step : Steps)
	{
		for (const FRopeGPUSDFCollider& Source : Step.SDFColliders)
		{
			const int64 Voxels = (int64)Source.ResX * Source.ResY * Source.ResZ;
			if (!Source.Distances || Source.ResX < 2 || Source.ResY < 2 || Source.ResZ < 2 || Voxels <= 0)
			{
				continue;
			}
			// Effective Volume — Last used frame stamp (common for new/existing; basis for rebuild eviction check).
			Cache.KeyLastUsedFrame.FindOrAdd(Source.VolumeKey) = Frame;
			if (Cache.KeyToIndex.Contains(Source.VolumeKey))
			{
				// Already resident — no dequant/upload (static bake data, no need to re-dequant).
				continue;
			}
			// New volume: append to global array (index/offset stable — existing references invariant).
			Cache.KeyToIndex.Add(Source.VolumeKey, Cache.CpuVol.Num());

			FRopeSDFVolumeGPU Volume;
			Volume.DistOffset = Cache.CpuDist.Num();
			Volume.ResX = Source.ResX; Volume.ResY = Source.ResY; Volume.ResZ = Source.ResZ;
			Volume.LocalMin = FVector4f((float)Source.LocalMin.X, (float)Source.LocalMin.Y,
				(float)Source.LocalMin.Z, 0.0f);
			Volume.LocalSize = FVector4f((float)Source.LocalSize.X, (float)Source.LocalSize.Y,
				(float)Source.LocalSize.Z, 0.0f);
			Cache.CpuVol.Add(Volume);

			// code → float(cm) dequant. Asymmetric band: d = code*(range/MaxCode) - NBIn. Code is BytesPerCode per voxel
			// Bytes (little-endian): 1=uint8(max255), 2=uint16(max65535). shader SDFDistances remain float (.usf unchanged).
			const int32 VoxN = (int32)Voxels;
			const int32 BytesPerCode = Source.BytesPerCode;
			const float MaxCodeF = (BytesPerCode >= 2) ? 65535.0f : 255.0f;
			const float NarrowBandInner = Source.NarrowBandInner;
			const float Range = NarrowBandInner + Source.NarrowBandOuter;
			const float DeqScale = (Range > 0.0f) ? (Range / MaxCodeF) : 0.0f;
			Cache.CpuDist.Reserve(Cache.CpuDist.Num() + VoxN);
			for (int32 Vi = 0; Vi < VoxN; ++Vi)
			{
				uint32 Code = Source.Distances[Vi * BytesPerCode];
				if (BytesPerCode >= 2)
				{
					Code |= static_cast<uint32>(Source.Distances[Vi * BytesPerCode + 1]) << 8;
				}
				// Outside +
				Cache.CpuDist.Add(static_cast<float>(Code) * DeqScale - NarrowBandInner);
			}
			Cache.bDirty = true;
		}
	}

	// generational eviction: If unreferenced volumes have accumulated beyond the recall threshold for a long time, reconfigure cache compression with only the live set.
	if (Cache.CpuVol.Num() > 0)
	{
		int64 DeadFloats = 0;
		for (const TPair<uint64, int32>& KV : Cache.KeyToIndex)
		{
			const uint64* Last = Cache.KeyLastUsedFrame.Find(KV.Key);
			if (!Last || (Frame - *Last) >= GRopeSDFEvictAfterFrames)
			{
				const FRopeSDFVolumeGPU& Volume = Cache.CpuVol[KV.Value];
				DeadFloats += (int64)Volume.ResX * Volume.ResY * Volume.ResZ;
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
		// No volumes (first time, or all evicted by rebuild) — Dummy binding after freeing resident buffer (reclaiming VRAM).
		Cache.DistBuf.SafeRelease();
		Cache.VolBuf.SafeRelease();
		Cache.bDirty = false;
		// 1 dummy (binding validity; shader does not reference NumSDFColliders=0).
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
		// No new volume → reuse (upload 0). Steady state — comes here even if the set/order of the rope's adjacent volumes is disturbed.
		OutDistRDG = GraphBuilder.RegisterExternalBuffer(Cache.DistBuf);
		OutVolRDG  = GraphBuilder.RegisterExternalBuffer(Cache.VolBuf);
		return;
	}

	// First or new volume (dirty) — global buffer regeneration + upload (append-only grow, occurs only as many volumes per session).
	OutDistRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.GlobalSDFDist"),
		sizeof(float), Cache.CpuDist.Num(), Cache.CpuDist.GetData(), (uint64)Cache.CpuDist.Num() * sizeof(float));
	OutVolRDG = RopeUploadBuffer(GraphBuilder, TEXT("Rope.GlobalSDFVol"),
		sizeof(FRopeSDFVolumeGPU), Cache.CpuVol.Num(), Cache.CpuVol.GetData(), (uint64)Cache.CpuVol.Num() * sizeof(FRopeSDFVolumeGPU));
	Cache.DistBuf = GraphBuilder.ConvertToExternalBuffer(OutDistRDG);
	Cache.VolBuf  = GraphBuilder.ConvertToExternalBuffer(OutVolRDG);
	Cache.bDirty = false;
}

// SDF collider packing: bind the shared global volume cache — where RopeEnsureGlobalSDFVolumes guarantees the
// volumes are resident — and upload only the per-instance array each frame, the global VolumeIndex plus the
// current and previous bone transforms. No distance dequantization or upload happens here; the global cache
// does that once per VolumeKey. Fills Build.SDF*Buf and NumValidSDFCol.
static void RopePackSDFColliders(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step, FRopeStepBuild& Build,
	const TMap<uint64, int32>& GlobalKeyToIndex, FRDGBufferRef GlobalDistRDG, FRDGBufferRef GlobalVolRDG)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackSDF);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_PackSDF);
	// Shared binding of global distance/header buffer (no rope-specific copying).
	Build.SDFDistBuf = GlobalDistRDG;
	Build.SDFVolBuf = GlobalVolRDG;

	TArray<FRopeSDFColliderGPU>& SDFCol = *GraphBuilder.AllocObject<TArray<FRopeSDFColliderGPU>>();
	for (const FRopeGPUSDFCollider& Source : Step.SDFColliders)
	{
		const int32* VolumeIndex = GlobalKeyToIndex.Find(Source.VolumeKey);
		if (!VolumeIndex)
		{
			// invalid volume (unregistered because the global cache filtered it out with dequant conditions) — Skip.
			continue;
		}
		const FQuat Rotation = Source.BoneToWorld.GetRotation();
		const FVector Translation = Source.BoneToWorld.GetTranslation();
		const FVector Scale = Source.BoneToWorld.GetScale3D();
		const FQuat PrevRotation = Source.PrevBoneToWorld.GetRotation();
		const FVector PrevTranslation = Source.PrevBoneToWorld.GetTranslation();
		FRopeSDFColliderGPU GpuCollider;
		GpuCollider.VolumeIndex = *VolumeIndex;
		GpuCollider.Rotation = FVector4f((float)Rotation.X, (float)Rotation.Y,
			(float)Rotation.Z, (float)Rotation.W);
		GpuCollider.Translation = FVector4f((float)Translation.X, (float)Translation.Y,
			(float)Translation.Z, 0.0f);
		GpuCollider.Scale = FVector4f((float)Scale.X, (float)Scale.Y, (float)Scale.Z, 0.0f);
		GpuCollider.PrevRotation = FVector4f((float)PrevRotation.X, (float)PrevRotation.Y,
			(float)PrevRotation.Z, (float)PrevRotation.W);
		// w=InvDt
		GpuCollider.PrevTranslation = FVector4f((float)PrevTranslation.X, (float)PrevTranslation.Y,
			(float)PrevTranslation.Z, Source.InvDeltaTime);
		SDFCol.Add(GpuCollider);
	}

	// Valid count — confirmed *before* dummy padding. 1 empty dummy (shader is not referenced because NumSDFColliders=0).
	Build.NumValidSDFCol = SDFCol.Num();
	if (SDFCol.Num() == 0) { SDFCol.AddZeroed(1); }
	Build.SDFColBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.SDFColliders"),
		sizeof(FRopeSDFColliderGPU), SDFCol.Num(), SDFCol.GetData(), (uint64)SDFCol.Num() * sizeof(FRopeSDFColliderGPU));
}

// Override(G0) upload: per-node flag/target/mass (transient, only override frame is real data).
// If not, 1 dummy + bHasOverrides=0 → shader does not refer to it.
static void RopePackOverrides(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step, FRopeStepBuild& Build)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_PackOverrides);
	const int32 NumNodes = Step.NumNodes;
	TArray<uint32>&    OvFlags = *GraphBuilder.AllocObject<TArray<uint32>>();
	TArray<FVector4f>& OvPos   = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	TArray<FVector4f>& OvPrev  = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	TArray<float>&     OvInv   = *GraphBuilder.AllocObject<TArray<float>>();
	if (Build.bHasOverrides)
	{
		const bool bHavePositions = Step.OverridePositions.Num() == NumNodes;
		const bool bHavePrevPositions = Step.OverridePrevPositions.Num() == NumNodes;
		const bool bHaveInvMass = Step.OverrideInvMass.Num() == NumNodes;
		OvFlags.SetNumUninitialized(NumNodes);
		OvPos.SetNumUninitialized(NumNodes);
		OvPrev.SetNumUninitialized(NumNodes);
		OvInv.SetNumUninitialized(NumNodes);
		for (int32 k = 0; k < NumNodes; ++k)
		{
			OvFlags[k] = Step.OverrideFlags[k];
			const FVector Position = bHavePositions ? Step.OverridePositions[k] : FVector::ZeroVector;
			const FVector PrevPosition = bHavePrevPositions
				? Step.OverridePrevPositions[k] : FVector::ZeroVector;
			OvPos[k] = FVector4f((float)Position.X, (float)Position.Y, (float)Position.Z, 0.0f);
			OvPrev[k] = FVector4f((float)PrevPosition.X, (float)PrevPosition.Y, (float)PrevPosition.Z, 0.0f);
			OvInv[k] = bHaveInvMass ? Step.OverrideInvMass[k] : 1.0f;
		}
	}
	else
	{
		OvFlags.AddZeroed(1);
		OvPos.AddZeroed(1);
		OvPrev.AddZeroed(1);
		OvInv.AddZeroed(1);
	}
	Build.OvFlagsBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.OverrideFlags"),
		sizeof(uint32), OvFlags.Num(), OvFlags.GetData(), (uint64)OvFlags.Num() * sizeof(uint32));
	Build.OvPosBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.OverridePositions"),
		sizeof(FVector4f), OvPos.Num(), OvPos.GetData(), (uint64)OvPos.Num() * sizeof(FVector4f));
	Build.OvPrevBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.OverridePrevPositions"),
		sizeof(FVector4f), OvPrev.Num(), OvPrev.GetData(), (uint64)OvPrev.Num() * sizeof(FVector4f));
	Build.OvInvBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.OverrideInvMass"),
		sizeof(float), OvInv.Num(), OvInv.GetData(), (uint64)OvInv.Num() * sizeof(float));
}

// solve pass: parameter buffer configuration + XPBD CS dispatch (GDF permutation selects rope units).
// tension(λ) Creates and returns an output buffer (frame transient) — consumed by readback arm (RopeArmReadbacks).
static FRDGBufferRef RopeAddSolvePass(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step,
	const FRopeStepBuild& Build, const FSceneView* View,
	const FGlobalDistanceFieldParameters2& GDFSolverParams, uint32 bGDFSolverValid,
	const FVector3f& PreViewTranslation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_AddSolvePass);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_AddSolvePass);
	const int32 NumNodes = Step.NumNodes;

	TArray<FRopeGPUParamsGPU>& ParamsArr = *GraphBuilder.AllocObject<TArray<FRopeGPUParamsGPU>>();
	FRopeGPUParamsGPU GpuParams;
	GpuParams.NodeOffset = 0;
	GpuParams.NumNodes = NumNodes;
	GpuParams.NumSub = Step.NumSub;
	GpuParams.Iters = FMath::Max(1, Step.Iterations);
	GpuParams.FixedDt = Step.FixedDt;
	GpuParams.SegmentLength = Step.SegmentLength;
	GpuParams.StretchCompliance = Step.StretchCompliance;
	GpuParams.MaxStretchRatio = Step.MaxStretchRatio;
	GpuParams.BendCompliance = Step.BendCompliance;
	GpuParams.BendReleaseRatio = Step.BendReleaseRatio;
	GpuParams.BendFullRatio = Step.BendFullRatio;
	GpuParams.Damping = Step.Damping;
	GpuParams.bStartPinned = Step.bStartPinned ? 1 : 0;
	GpuParams.CapsuleOffset = 0;
	// collision-Free aim Flight only sets the number of shapes in the solve kernel to 0. The uploaded buffer continues to be used by the detect kernel.
	GpuParams.NumCapsules = Step.bSolveCollisions ? Build.NumValidCaps : 0;
	GpuParams.CollisionRadius = Step.CollisionRadius;
	GpuParams.Friction = Step.Friction;
	GpuParams.TipFrictionScale = Step.TipFrictionScale;
	GpuParams.CollisionPasses = FMath::Clamp(Step.CollisionPasses, 1, FMath::Max(1, Step.Iterations));
	GpuParams.SweepStep = Step.SweepStep;
	GpuParams.MaxSweepSamples = FMath::Max(1, Step.MaxSweepSamples);
	GpuParams.SDFColliderOffset = 0;
	GpuParams.NumSDFColliders = Step.bSolveCollisions ? Build.NumValidSDFCol : 0;
	GpuParams.bHasOverrides = Build.bHasOverrides ? 1 : 0;
	GpuParams.NumBoxes = Step.bSolveCollisions ? Build.NumValidBoxes : 0;
	GpuParams.NumConvexes = Step.bSolveCollisions ? Build.NumValidConvexes : 0;
	GpuParams.Gravity = FVector4f((float)Step.Gravity.X, (float)Step.Gravity.Y, (float)Step.Gravity.Z, 0.0f);
	GpuParams.PinPrev = FVector4f((float)Step.StartPinPrev.X, (float)Step.StartPinPrev.Y,
		(float)Step.StartPinPrev.Z, 0.0f);
	GpuParams.PinTarget = FVector4f((float)Step.StartPinTarget.X, (float)Step.StartPinTarget.Y,
		(float)Step.StartPinTarget.Z, 0.0f);
	ParamsArr.Add(GpuParams);

	FRDGBufferRef ParamsBuf = RopeUploadBuffer(GraphBuilder, TEXT("Rope.Params"),
		sizeof(FRopeGPUParamsGPU), 1, ParamsArr.GetData(), sizeof(FRopeGPUParamsGPU));

	FRopeXPBDSolveCS::FParameters* PassParams = GraphBuilder.AllocParameters<FRopeXPBDSolveCS::FParameters>();
	PassParams->NumRopes      = 1;
	PassParams->Params        = GraphBuilder.CreateSRV(ParamsBuf);
	PassParams->Capsules      = GraphBuilder.CreateSRV(Build.CapsulesBuf);
	PassParams->SDFDistances  = GraphBuilder.CreateSRV(Build.SDFDistBuf);
	PassParams->SDFVolumes    = GraphBuilder.CreateSRV(Build.SDFVolBuf);
	PassParams->SDFColliders  = GraphBuilder.CreateSRV(Build.SDFColBuf);
	PassParams->Boxes         = GraphBuilder.CreateSRV(Build.BoxesBuf);
	PassParams->Convexes      = GraphBuilder.CreateSRV(Build.ConvexBuf);
	PassParams->ConvexPlanes  = GraphBuilder.CreateSRV(Build.ConvexPlanesBuf);
	PassParams->OverrideFlags         = GraphBuilder.CreateSRV(Build.OvFlagsBuf);
	PassParams->OverridePositions     = GraphBuilder.CreateSRV(Build.OvPosBuf);
	PassParams->OverridePrevPositions = GraphBuilder.CreateSRV(Build.OvPrevBuf);
	PassParams->OverrideInvMass       = GraphBuilder.CreateSRV(Build.OvInvBuf);
	PassParams->InvMass       = GraphBuilder.CreateUAV(Build.InvMassRDG);
	PassParams->Positions     = GraphBuilder.CreateUAV(Build.PosRDG);
	PassParams->PrevPositions = GraphBuilder.CreateUAV(Build.PrevRDG);
	// output tension(λ): frame transient (N slots, kernel rewrites entire dispatch every time - no persistence required).
	FRDGBufferRef LambdaRDG = GraphBuilder.CreateBuffer(
		FRDGBufferDesc::CreateStructuredDesc(sizeof(float), NumNodes), TEXT("Rope.LambdaDist"));
	PassParams->OutLambdaDist = GraphBuilder.CreateUAV(LambdaRDG);

	// GDF inside the solver: for a rope with bUseWorldGDF and a valid GDF, select the GDF permutation and bind
	// the view and GDF parameters; otherwise stay lean, as before. Each rope gets its own AddPass, so the
	// permutation can be chosen per rope.
	const bool bUseGDFPerm = (View != nullptr) && Step.bSolveCollisions && Step.bUseWorldGDF
		&& (bGDFSolverValid != 0);
	if (bUseGDFPerm)
	{
		PassParams->View                  = View->ViewUniformBuffer;
		PassParams->GDF                   = GDFSolverParams;
		PassParams->GDFPreViewTranslation = PreViewTranslation;
		PassParams->bWorldGDFValid        = bGDFSolverValid;
	}
	FRopeXPBDSolveCS::FPermutationDomain Permutation;
	// NumNodes ≤ MaxNodes(caller gate) → always ≥64.
	Permutation.Set<FRopeXPBDSolveCS::FNodeBucket>(RopeNodeBucket(NumNodes));
	Permutation.Set<FRopeXPBDSolveCS::FGDFDim>(bUseGDFPerm);
	TShaderMapRef<FRopeXPBDSolveCS> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), Permutation);
#if STATS
	++GRopeDispatchCount;
#endif
	FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeXPBDResident"),
		// 1 rope = 1 thread group
		ComputeShader, PassParams, FIntVector(1, 1, 1));

	return LambdaRDG;
}

// readback rearmament: only when there is no in-Flight (asynchronous copy of this frame stepped position). consume is RopeConsumeReadbacks.
static void RopeArmReadbacks(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step,
	FRopeResidentRope& Resident, const FRopeStepBuild& Build, FRDGBufferRef LambdaRDG)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_ArmReadbacks);
	const int32 NumNodes = Step.NumNodes;
	if (!Resident.bReadbackArmed)
	{
		if (!Resident.PosReadback)  { Resident.PosReadback  = new FRHIGPUBufferReadback(TEXT("Rope.PosReadback")); }
		if (!Resident.PrevReadback) { Resident.PrevReadback = new FRHIGPUBufferReadback(TEXT("Rope.PrevReadback")); }
		const uint32 NodeBytes = (uint32)NumNodes * sizeof(FVector4f);
		AddEnqueueCopyPass(GraphBuilder, Resident.PosReadback, Build.PosRDG, NodeBytes);
		AddEnqueueCopyPass(GraphBuilder, Resident.PrevReadback, Build.PrevRDG, NodeBytes);
#if STATS
		GRopeReadbackBytes += 2ull * NodeBytes;
#endif
		Resident.bReadbackArmed = true;
	}

	// Armed with tension(λ) readback: Only in solve frame (NumSub>0) — in override-only frame, λ is 0
	// Maintains the previous tension without arming (GT is overwritten only when updated).
	if (!Resident.bLambdaArmed && Step.NumSub > 0)
	{
		if (!Resident.LambdaReadback)
		{
			Resident.LambdaReadback = new FRHIGPUBufferReadback(TEXT("Rope.LambdaReadback"));
		}
		AddEnqueueCopyPass(GraphBuilder, Resident.LambdaReadback, LambdaRDG,
			(uint32)NumNodes * sizeof(float));
#if STATS
		GRopeReadbackBytes += (uint64)NumNodes * sizeof(float);
#endif
		Resident.LambdaFixedDt = Step.FixedDt;
		Resident.bLambdaArmed = true;
	}
}

// contact detection (G3): Sweep the post-solve position after solving. RDG guarantees the solve(UAV)→detect(SRV) order.
// 2 slots (actual+predictive) output per node. ContactBuf (resident, 2N slot) is secured only when there is detection.
static void RopeAddDetectPass(FRDGBuilder& GraphBuilder, const FRopeGPUResidentStep& Step,
	FRopeResidentRope& Resident, const FRopeStepBuild& Build)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_AddDetectPass);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_AddDetectPass);
	// Nested within solve scope — detection kernel time is attributed here, not Solve.
	RDG_EVENT_SCOPE_STAT(GraphBuilder, RopeGPUDetect, "DynamicRope Detect");
	const int32 NumNodes = Step.NumNodes;

	const bool bContactSeed = !Resident.ContactBuf.IsValid() || Build.bSeed;
	FRDGBufferRef ContactRDG = nullptr;
	if (bContactSeed)
	{
		ContactRDG = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(FRopeGPUContactGPU), 2 * NumNodes), TEXT("Rope.Contacts"));
		Resident.ContactBuf = GraphBuilder.ConvertToExternalBuffer(ContactRDG);
		// Regeneration → The previous contact readback is stale.
		Resident.bContactArmed = false;
	}
	else
	{
		ContactRDG = GraphBuilder.RegisterExternalBuffer(Resident.ContactBuf);
	}

	// Whip guide buffer (G3b) for predictive contact. Per-node mask/target when whip active, otherwise 1 dummy.
	const bool bHasWhip = Step.WhipGuidedMask.Num() == NumNodes && Step.PredictionFrames > 0.0f;
	TArray<uint32>&    GMask = *GraphBuilder.AllocObject<TArray<uint32>>();
	TArray<FVector4f>& WCur  = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	TArray<FVector4f>& WPrev = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	TArray<FVector4f>& WNext = *GraphBuilder.AllocObject<TArray<FVector4f>>();
	if (bHasWhip)
	{
		const bool bHaveCurrentTargets = Step.WhipCurrentTargets.Num() == NumNodes;
		const bool bHavePrevTargets = Step.WhipPrevTargets.Num() == NumNodes;
		const bool bHaveNextTargets = Step.WhipNextTargets.Num() == NumNodes;
		GMask.SetNumUninitialized(NumNodes);
		WCur.SetNumUninitialized(NumNodes);
		WPrev.SetNumUninitialized(NumNodes);
		WNext.SetNumUninitialized(NumNodes);
		for (int32 k = 0; k < NumNodes; ++k)
		{
			GMask[k] = Step.WhipGuidedMask[k];
			const FVector CurrentTarget = bHaveCurrentTargets
				? Step.WhipCurrentTargets[k] : FVector::ZeroVector;
			const FVector PrevTarget = bHavePrevTargets
				? Step.WhipPrevTargets[k] : FVector::ZeroVector;
			const FVector NextTarget = bHaveNextTargets
				? Step.WhipNextTargets[k] : FVector::ZeroVector;
			WCur[k] = FVector4f((float)CurrentTarget.X, (float)CurrentTarget.Y, (float)CurrentTarget.Z, 0.0f);
			WPrev[k] = FVector4f((float)PrevTarget.X, (float)PrevTarget.Y, (float)PrevTarget.Z, 0.0f);
			WNext[k] = FVector4f((float)NextTarget.X, (float)NextTarget.Y, (float)NextTarget.Z, 0.0f);
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
	DetectParams->DetectNumNodes       = NumNodes;
	// Detection uses only the front wrappable capsule/box. The caller appends the static world shape to the end.
	// Pass boundaries to NumDetectCapsules/NumDetectBoxes to ensure that static contacts do not obscure wrap candidates.
	// NumDetectCapsules=-1 participates in the entire capsule (existing behavior).
	DetectParams->DetectNumCapsules    = (Step.NumDetectCapsules >= 0)
		? FMath::Min(Step.NumDetectCapsules, Build.NumValidCaps) : Build.NumValidCaps;
	DetectParams->DetectNumSDF         = Build.NumValidSDFCol;
	// Detects only boxes that can be Wrapped (excluding static boxes). box also shares the deepest contact slot per node with capsule/SDF.
	DetectParams->DetectNumBoxes       = FMath::Clamp(Step.NumDetectBoxes, 0, Build.NumValidBoxes);
	// Only detects wrap-capable convexes (excluding static convexes) — Same contract as box.
	DetectParams->DetectNumConvexes    = FMath::Clamp(Step.NumDetectConvexes, 0, Build.NumValidConvexes);
	DetectParams->DetectContactRadius  = Step.ContactRadius;
	DetectParams->DetectSegmentLength  = Step.SegmentLength;
	DetectParams->DetectSweepStep      = FMath::Max(Step.ContactSweepStep, 0.1f);
	DetectParams->DetectMaxSweepSamples = FMath::Max(Step.ContactMaxSweepSamples, 1);
	DetectParams->DetectPredictionFrames = FMath::Max(0.0f, Step.PredictionFrames);
	DetectParams->DetectFrameToSubstepRatio = Step.ContactFrameToSubstepRatio;
	DetectParams->DetectHasGuidedNodes = bHasWhip ? 1 : 0;
	DetectParams->Capsules             = GraphBuilder.CreateSRV(Build.CapsulesBuf);
	DetectParams->SDFDistances         = GraphBuilder.CreateSRV(Build.SDFDistBuf);
	DetectParams->SDFVolumes           = GraphBuilder.CreateSRV(Build.SDFVolBuf);
	DetectParams->SDFColliders         = GraphBuilder.CreateSRV(Build.SDFColBuf);
	DetectParams->Boxes                = GraphBuilder.CreateSRV(Build.BoxesBuf);
	DetectParams->Convexes             = GraphBuilder.CreateSRV(Build.ConvexBuf);
	DetectParams->ConvexPlanes         = GraphBuilder.CreateSRV(Build.ConvexPlanesBuf);
	DetectParams->DetectPositions      = GraphBuilder.CreateSRV(Build.PosRDG);
	DetectParams->DetectPrevPositions  = GraphBuilder.CreateSRV(Build.PrevRDG);
	DetectParams->DetectGuidedMask     = GraphBuilder.CreateSRV(GMaskBuf);
	DetectParams->DetectWhipCur        = GraphBuilder.CreateSRV(WCurBuf);
	DetectParams->DetectWhipPrev       = GraphBuilder.CreateSRV(WPrevBuf);
	DetectParams->DetectWhipNext       = GraphBuilder.CreateSRV(WNextBuf);
	DetectParams->OutContacts          = GraphBuilder.CreateUAV(ContactRDG);

	FRopeContactDetectCS::FPermutationDomain Permutation;
	// NumNodes ≤ MaxNodes → always ≥64.
	Permutation.Set<FRopeContactDetectCS::FNodeBucket>(RopeNodeBucket(NumNodes));
	TShaderMapRef<FRopeContactDetectCS> DetectShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), Permutation);
#if STATS
	++GRopeDispatchCount;
#endif
	FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("RopeContactDetect"),
		DetectShader, DetectParams, FIntVector(1, 1, 1));

	if (!Resident.bContactArmed)
	{
		if (!Resident.ContactReadback)
		{
			Resident.ContactReadback = new FRHIGPUBufferReadback(TEXT("Rope.ContactReadback"));
		}
		AddEnqueueCopyPass(GraphBuilder, Resident.ContactReadback, ContactRDG,
			(uint32)(2 * NumNodes) * sizeof(FRopeGPUContactGPU));
		// The set pointed to by ColliderIndex belongs to *this* dispatch — it carries its signature with it to the results.
		Resident.ContactAttribSig = Step.AttribSig;
#if STATS
		GRopeReadbackBytes += (uint64)(2 * NumNodes) * sizeof(FRopeGPUContactGPU);
#endif
		Resident.bContactArmed = true;
	}
}

// Shared execution unit (RT) of resident steps. Whether it is a dedicated graph (Step) or a scene renderer graph (DispatchPending_RenderThread)
// The same body is Loaded on the received GraphBuilder (Execute is caller). GDF/PreViewTranslation is used in GDF world collision (Phase 2c).
// The main body of the step is the above helpers (RopeConsumeReadbacks/RopeEnsureResidentBuffers/RopePack*/RopeAdd*Pass/RopeArmReadbacks).
// We've disassembled it, leaving only the orchestration here — the early-out and external (SRV) barrier contracts are visible at a glance in this function.
void FRopeGPUSolver::RunSteps_RenderThread(FRDGBuilder& GraphBuilder, TArray<FRopeGPUResidentStep>& Steps,
	const FSceneView* View, const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeRT_RunSteps);
	SCOPE_CYCLE_COUNTER(STAT_RopeGPU_RunSteps);
	// GPU Timeline attribution: All passes added to GraphBuilder within this scope are captured as 'DynamicRope Solve'
	// (detection pass is taken back by RopeAddDetectPass with its own scope).
	RDG_EVENT_SCOPE_STAT(GraphBuilder, RopeGPUSolve, "DynamicRope Solve");
#if STATS
	// Reset the upload accumulation for this frame (RopeUploadBuffers below are added by category). RunSteps once per frame.
	GRopeUploadBytesTotal = 0;
	GRopeUploadBytesSDF = 0;
	GRopeUploadBytesColliders = 0;
	GRopeReadbackBytes = 0;
	GRopeDispatchCount = 0;
	GRopeSubstepSum = 0;
#endif
	// --- Loop 1: Readback consume the previous frame — Processed as immediate Lock *before* graph construction.
	RopeConsumeReadbacks(Impl->RtRopes, *Impl->Results, Steps);

	// GDF in-solver: If there is a view (=view expansion path), solve it with GDF permutation and project the wall at every substep.
	// Calculate frame shared GDF shader parameters once (if not built, bGDFSolverValid=0 → lean fallback for each rope).
	// GDF is not used in step paths without views (static world collision is only for view expansion paths).
	const bool bGDFInSolver = (View != nullptr);
	FGlobalDistanceFieldParameters2 GDFSolverParams;
	uint32 bGDFSolverValid = 0;
	if (bGDFInSolver)
	{
		FillGDFShaderParams(GDF, GDFSolverParams, bGDFSolverValid);
	}

	// Guaranteed global SDF volume residency (once per-frame, *before* the rope loop). Dequant/upload only new volumes → Steady state reupload 0.
	// The RopePackSDFColliders for each rope below share-bind this global buffer and upload only the instance (bone transform) array.
	FRDGBufferRef GlobalSDFDistRDG = nullptr;
	FRDGBufferRef GlobalSDFVolRDG = nullptr;
	RopeEnsureGlobalSDFVolumes(GraphBuilder, Steps, Impl->GlobalSDF, GlobalSDFDistRDG, GlobalSDFVolRDG);

	// --- Loop 2 (phase batching): preparation (2a) → solve dispatch continuation (2b) → readback arming (2c) → detection (2d) → external barrier (2e).
	// Previously, [solve → readback copy → detection → SRV transition] was interleaved for each rope, so the copy/transition barrier of rope N was
	// N+1 solve was issued only after waiting for N solves to be completed — 64 threads (1 group) dispatched as many ropes on the GPU
	// Serialized so that the wall clock grows to "delay per group x number of ropes" ('DynamicRope Solve' is proportional to the number of ropes). If you tie it down step by step
	// There is no barrier between solve dispatch (ropes are buffer independent, global SDF is read-read), so the GPU executes groups in parallel.
	// → Wall clock ≈ 1 slowest rope. Per-rope semantics (solve→copy→detection→SRV transition) remains the same on a rope basis,
	// Only the global order is rearranged — the upload is executed in batches in the RDG prologue anyway, so it remains in 2a.
	struct FRopePreparedStep
	{
		const FRopeGPUResidentStep* Step = nullptr;
		FRopeStepBuild Build;
		// tension(λ) transient buffer created by solve pass — consumed by readback arming (2c).
		FRDGBufferRef LambdaRDG = nullptr;
		// Whether to perform GDF permutation (2b alignment key — minimize switches by placing the same PSOs in a row).
		bool bGDFPerm = false;
	};
	TArray<FRopePreparedStep> Prepared;
	Prepared.Reserve(Steps.Num());
	TSet<uint32> PreparedRopeIds;
	bool bHasSequentialSameRopeSteps = false;

	// --- 2a: seed/register + collider/override packing per rope.
	for (const FRopeGPUResidentStep& Step : Steps)
	{
		const int32 NumNodes = Step.NumNodes;
#if STATS
		GRopeSubstepSum += (uint32)FMath::Max(0, Step.NumSub);
#endif
		if (NumNodes < 2 || NumNodes > FRopeGPUSolver::MaxNodes)
		{
			UE_LOG(LogDynamicRopeGPU, Warning, TEXT("GPU resident step skipped: %d nodes out of [2, %d]."),
				NumNodes, FRopeGPUSolver::MaxNodes);
			continue;
		}

		FRopeResidentRope& Resident = Impl->RtRopes.FindOrAdd(Step.RopeId);
		// Record the flag used for GDF permutation selection in the resident state (collision radius/friction is directly passed to CS as Params buffer).
		// When you turn off collision solve in Aim Flight, GDF push-out is also turned off, and it does not affect the separate detect kernel.
		Resident.bUseWorldGDF = Step.bSolveCollisions && Step.bUseWorldGDF;

		FRopePreparedStep Entry;
		Entry.Step = &Step;
		bHasSequentialSameRopeSteps |= PreparedRopeIds.Contains(Step.RopeId);
		PreparedRopeIds.Add(Step.RopeId);
		RopeEnsureResidentBuffers(GraphBuilder, Step, Resident, Entry.Build);

		// G0: Override must be recorded without integration (NumSub=0) — logic phase frame (Wrapping/Releasing, etc.).
		Entry.Build.bHasOverrides = Step.HasOverrides() && Step.OverrideFlags.Num() == NumNodes;
		if (Step.HasOverrides() && !Entry.Build.bHasOverrides)
		{
			UE_LOG(LogDynamicRopeGPU, Warning, TEXT("GPU override ignored: flags %d != nodes %d."),
				Step.OverrideFlags.Num(), NumNodes);
		}

		if (Step.NumSub <= 0 && !Entry.Build.bHasOverrides && !Step.bDetectContacts)
		{
			// No integration/recording/detection this frame — position unchanged, readback also left as is. External read status only (SRV)
			// Confirm (including early termination frame immediately after seed upload) — Same purpose as call in 2e.
			GraphBuilder.UseExternalAccessMode(Entry.Build.PosRDG, ERHIAccess::SRVMask);
			continue;
		}

		RopePackCapsules(GraphBuilder, Step, Entry.Build);
		RopePackSDFColliders(GraphBuilder, Step, Entry.Build, Impl->GlobalSDF.KeyToIndex,
			GlobalSDFDistRDG, GlobalSDFVolRDG);
		RopePackBoxes(GraphBuilder, Step, Entry.Build);
		RopePackConvexes(GraphBuilder, Step, Entry.Build);
		RopePackOverrides(GraphBuilder, Step, Entry.Build);

		// Same as the bUseGDFPerm check in RopeAddSolvePass (here only the alignment key is used — the actual selection is the single source of truth).
		Entry.bGDFPerm = bGDFInSolver && Step.bSolveCollisions && Step.bUseWorldGDF
			&& (bGDFSolverValid != 0);
		Prepared.Add(MoveTemp(Entry));
	}

	// Continuous placement of same permutations (node ​​buckets, GDF) — Minimize PSO switches between solve dispatches.
	// Because it is a stable alignment, the step order (= registration order) is maintained within the same key.
	// A catch-up queue may contain more than one temporal step for a rope. Those steps touch the same
	// resident UAV and must remain in their original frame order. Ordinary frames still take the PSO sort.
	if (!bHasSequentialSameRopeSteps)
	{
		Prepared.StableSort([](const FRopePreparedStep& A, const FRopePreparedStep& B)
		{
			const int32 BucketA = RopeNodeBucket(A.Step->NumNodes);
			const int32 BucketB = RopeNodeBucket(B.Step->NumNodes);
			if (BucketA != BucketB) { return BucketA < BucketB; }
			return !A.bGDFPerm && B.bGDFPerm;
		});
	}

	TMap<uint32, int32> LastPreparedIndexByRope;
	for (int32 PreparedIndex = 0; PreparedIndex < Prepared.Num(); ++PreparedIndex)
	{
		LastPreparedIndexByRope.Add(Prepared[PreparedIndex].Step->RopeId, PreparedIndex);
	}

	// --- 2b: Issue solve dispatches continuously before any copy/detection. Different ropes remain independent;
	// queued steps for the same rope share a UAV, so RDG preserves their original dispatch order.
	for (FRopePreparedStep& Entry : Prepared)
	{
		Entry.LambdaRDG = RopeAddSolvePass(GraphBuilder, *Entry.Step, Entry.Build,
			bGDFInSolver ? View : nullptr, GDFSolverParams, bGDFSolverValid, PreViewTranslation);
	}

	// --- 2c: Readback rearmament (asynchronous copy). Before detection (2d) — Pos transitions in only one direction: UAV → CopySrc → SRV.
	for (int32 PreparedIndex = 0; PreparedIndex < Prepared.Num(); ++PreparedIndex)
	{
		FRopePreparedStep& Entry = Prepared[PreparedIndex];
		if (LastPreparedIndexByRope.FindRef(Entry.Step->RopeId) != PreparedIndex)
		{
			continue;
		}
		FRopeResidentRope& Resident = Impl->RtRopes.FindChecked(Entry.Step->RopeId);
		RopeArmReadbacks(GraphBuilder, *Entry.Step, Resident, Entry.Build, Entry.LambdaRDG);
	}

	// --- 2d: contact detection (Flight rope only). Solve results are read as SRV — order is guaranteed to be RDG dependent.
	for (int32 PreparedIndex = 0; PreparedIndex < Prepared.Num(); ++PreparedIndex)
	{
		FRopePreparedStep& Entry = Prepared[PreparedIndex];
		if (LastPreparedIndexByRope.FindRef(Entry.Step->RopeId) == PreparedIndex
			&& Entry.Step->bDetectContacts)
		{
			FRopeResidentRope& Resident = Impl->RtRopes.FindChecked(Entry.Step->RopeId);
			RopeAddDetectPass(GraphBuilder, *Entry.Step, Resident, Entry.Build);
		}
	}

	// --- 2e: batch the external (SRV) access transitions. The raw render tube path reads PosBuf as an SRV
	// *outside* this graph, so after the last pass — solve, copy or detect — the SRV transition (barrier) is
	// pinned every frame. Without it the graph's end state alternates between UAVCompute and CopySrc depending
	// on whether a readback copy ran, and on a frame with no barrier the tube reads a previous or partial
	// position and the wrap nodes visibly shake.
	// GDF collision is resolved inside the solve CS as a substep constraint, so there is no separate post-solve
	// write, and for a rope without detection the solve pass is the last thing that writes PosBuf.
	for (int32 PreparedIndex = 0; PreparedIndex < Prepared.Num(); ++PreparedIndex)
	{
		FRopePreparedStep& Entry = Prepared[PreparedIndex];
		if (LastPreparedIndexByRope.FindRef(Entry.Step->RopeId) == PreparedIndex)
		{
			GraphBuilder.UseExternalAccessMode(Entry.Build.PosRDG, ERHIAccess::SRVMask);
		}
	}

#if STATS
	// GPU-resident VRAM instrumentation. Sum only persistent buffers (assigned with ConvertToExternalBuffer, maintained between frames) — frame transient
	// (colliders/params/detect RDG) is fully reused, so it is not a resident footprint. GetSize()=Desc bytes.
	{
		uint64 RopeBytes = 0;
		for (const TPair<uint32, FRopeResidentRope>& Pair : Impl->RtRopes)
		{
			const FRopeResidentRope& Resident = Pair.Value;
			if (Resident.PosBuf.IsValid())     { RopeBytes += Resident.PosBuf->GetSize(); }
			if (Resident.PrevBuf.IsValid())    { RopeBytes += Resident.PrevBuf->GetSize(); }
			if (Resident.InvMassBuf.IsValid()) { RopeBytes += Resident.InvMassBuf->GetSize(); }
			if (Resident.ContactBuf.IsValid()) { RopeBytes += Resident.ContactBuf->GetSize(); }
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
	// Private (self) graph path — Subsystem Tick trigger (G4 default). Executes immediately regardless of scene render timing.
	ENQUEUE_RENDER_COMMAND(RopeResidentStep)(
		[this, Steps = MoveTemp(Steps)](FRHICommandListImmediate& RHICmdList) mutable
		{
			FRDGBuilder GraphBuilder(RHICmdList);
			// Dedicated graph path — No View/GDF (GDF in-solver is for view expansion path only). View=nullptr → always lean.
			RunSteps_RenderThread(GraphBuilder, Steps, nullptr, nullptr, FVector3f::ZeroVector);
			{
				// RDG compilation + RHI command recording (can be a large portion of render thread CPU cost).
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
	// Scene renderer graph path (GDF world collision): does not dispatch immediately, but accumulates in the RT
	// temporal queue. PreRenderBasePass flushes it with valid GDF parameters and no additional tube delay.
	ENQUEUE_RENDER_COMMAND(RopeEnqueueSteps)(
		[this, Steps = MoveTemp(Steps)](FRHICommandListImmediate&) mutable
		{
			constexpr int32 MaxQueuedFramesPerRope = 4;
			constexpr int32 MaxQueuedSubstepsPerRope = 32;
			TMap<uint32, int32> IncomingStepIndices;
			TMap<uint32, int32> QueuedFrameCounts;
			TMap<uint32, int32> QueuedSubstepCounts;
			for (int32 StepIndex = 0; StepIndex < Steps.Num(); ++StepIndex)
			{
				IncomingStepIndices.Add(Steps[StepIndex].RopeId, StepIndex);
				QueuedFrameCounts.FindOrAdd(Steps[StepIndex].RopeId) += 1;
				QueuedSubstepCounts.FindOrAdd(Steps[StepIndex].RopeId) +=
					FMath::Max(0, Steps[StepIndex].NumSub);
			}

			// Select the newest compatible history that fits the bounded catch-up budget. A queued frame keeps
			// its own guide/pin/override payload and its own substep schedule; combining only NumSub would apply
			// the newest payload once and extrapolate it across several frames, which overshoots during Flight.
			TSet<int32> PreservedTemporalIndices;
			for (int32 PendingIndex = Impl->PendingSteps.Num() - 1; PendingIndex >= 0; --PendingIndex)
			{
				const FRopeGPUResidentStep& Pending = Impl->PendingSteps[PendingIndex];
				const int32* IncomingIndex = IncomingStepIndices.Find(Pending.RopeId);
				if (!IncomingIndex || !Pending.CanExecuteBefore(Steps[*IncomingIndex]))
				{
					continue;
				}

				int32& FrameCount = QueuedFrameCounts.FindOrAdd(Pending.RopeId);
				int32& SubstepCount = QueuedSubstepCounts.FindOrAdd(Pending.RopeId);
				const int32 PendingSubsteps = FMath::Max(0, Pending.NumSub);
				if (FrameCount >= MaxQueuedFramesPerRope ||
					SubstepCount + PendingSubsteps > MaxQueuedSubstepsPerRope)
				{
					continue;
				}

				PreservedTemporalIndices.Add(PendingIndex);
				++FrameCount;
				SubstepCount += PendingSubsteps;
			}

			TArray<FRopeGPUResidentStep> PreservedTemporalSteps;
			TArray<FRopeGPUResidentStep> PreservedHandoffGDFSteps;
			PreservedTemporalSteps.Reserve(PreservedTemporalIndices.Num());
			// A reseed/topology boundary or a queue beyond the bounded catch-up budget keeps the existing refund
			// path. A GDF handoff step with no newer step for that rope remains preserved until a scene view can
			// consume it.
			if (Impl->PendingSteps.Num() > 0)
			{
				FScopeLock SL(&Impl->Results->Lock);
				for (int32 PendingIndex = 0; PendingIndex < Impl->PendingSteps.Num(); ++PendingIndex)
				{
					FRopeGPUResidentStep& Dropped = Impl->PendingSteps[PendingIndex];
					if (Impl->HandoffGDFBlockedRopes.Contains(Dropped.RopeId)
						&& !IncomingStepIndices.Contains(Dropped.RopeId))
					{
						PreservedHandoffGDFSteps.Add(MoveTemp(Dropped));
						continue;
					}
					Impl->HandoffGDFBlockedRopes.Remove(Dropped.RopeId);
					if (PreservedTemporalIndices.Contains(PendingIndex))
					{
						// Only the newest frame publishes contact/readback observations. The older frame still solves
						// with its own colliders and overrides before the newer one.
						Dropped.bDetectContacts = false;
						PreservedTemporalSteps.Add(MoveTemp(Dropped));
						continue;
					}
					if (Dropped.NumSub > 0 && Dropped.FixedDt > 0.0f)
					{
						Impl->Results->DroppedSimTime.FindOrAdd(Dropped.RopeId) +=
							static_cast<float>(Dropped.NumSub) * Dropped.FixedDt;
					}
				}
			}
			Impl->PendingSteps = MoveTemp(PreservedTemporalSteps);
			Impl->PendingSteps.Append(MoveTemp(Steps));
			Impl->PendingSteps.Append(MoveTemp(PreservedHandoffGDFSteps));
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
	for (const FRopeGPUResidentStep& Step : Impl->PendingSteps)
	{
		Impl->HandoffGDFBlockedRopes.Remove(Step.RopeId);
	}
	RunSteps_RenderThread(GraphBuilder, Impl->PendingSteps, View, GDF, PreViewTranslation);
	Impl->PendingSteps.Reset();
}
