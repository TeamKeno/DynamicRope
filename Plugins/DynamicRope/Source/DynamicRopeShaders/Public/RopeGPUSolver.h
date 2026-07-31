// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU (compute) implementation of the XPBD rope solver, with the centerline resident on the GPU.
// The position buffer persists between frames and is advanced in place on the GPU each frame, so the
// sequential dependency between substeps is satisfied entirely on the GPU and there is no round-trip stall
// or slow-motion. Every readback — for render and for collision — happens on the render thread, where each
// frame's step command locks and consumes the previous readback before re-arming it, so the game thread
// never stalls.
// The formulation matches the CPU FRopeXPBDSolver, except that constraints are solved with red-black and
// stride-3 colouring for parallel safety.
// This module (DynamicRopeShaders) does not depend on the DynamicRope runtime types, so a step is
// self-contained POD; the runtime subsystem fills it from FRopeSimState and the config.

#pragma once

#include "CoreMinimal.h"

/** Analytic capsule for GPU collision: a world-space segment (A-B) plus a radius, extracted by the caller from the collider. */
struct FRopeGPUCapsule
{
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;

	/**
	 * Previous endpoints plus 1 / frame dt, used for surface-velocity drag and substep relative-motion CCD —
	 * the counterpart to the SDF's PrevBoneToWorld. InvDeltaTime 0, the default, means static: packing falls
	 * back to prev = current, so leaving these unset behaves exactly as before.
	 */
	FVector PrevA = FVector::ZeroVector;
	FVector PrevB = FVector::ZeroVector;
	float   InvDeltaTime = 0.0f;
};

/**
 * Analytic box (OBB) for GPU collision. Static world geometry — a static body's simple collision — is the
 * common case; a moving or wrappable box takes part in surface velocity and CCD through the frame motion
 * below (PrevCenter, PrevRot and InvDeltaTime).
 * It exists because an analytic query gives exact diagonal normals at corners and edges, where GDF voxel
 * rounding lets the rope sink in. The caller extracts it with IRopeCollider::GetGPUBox and GetGPUBoxMotion.
 */
struct FRopeGPUBox
{
	/** World-space box centre. */
	FVector Center = FVector::ZeroVector;
	/** World-space box rotation. */
	FQuat   Rot = FQuat::Identity;
	/** Local half-extents, with scale already applied. */
	FVector HalfExtents = FVector::ZeroVector;

	/**
	 * Previous centre and rotation plus 1 / frame dt, for a moving body's surface velocity and substep CCD.
	 * InvDeltaTime 0 means static: packing fills prev = current, so leaving these unset behaves as before.
	 * The counterpart to the capsule's Prev* fields.
	 */
	FVector PrevCenter = FVector::ZeroVector;
	FQuat   PrevRot = FQuat::Identity;
	float   InvDeltaTime = 0.0f;
};

/**
 * Analytic convex (plane set) for GPU collision, covering static and dynamic world geometry — convex simple
 * collision and sheared boxes.
 * The planes live in the step's flat ConvexPlanes pool over [PlaneOffset, PlaneOffset + PlaneCount): they are
 * body-local, with unit outward normals and PlaneDot(p) = dot(N, p) - W, before the rigid transform. A world
 * plane is the local plane composed with the rigid transform (Rot, Trans). LocalBounds is the local AABB, used
 * to cull. A moving body gets surface velocity and CCD from its prev transform and InvDeltaTime. The caller
 * extracts all of it with GetGPUConvex.
 */
struct FRopeGPUConvex
{
	/** Start index into the ConvexPlanes pool. */
	int32   PlaneOffset = 0;
	/** Number of planes. */
	int32   PlaneCount = 0;
	/** Body-local AABB centre. */
	FVector LocalBoundsCenter = FVector::ZeroVector;
	/** Body-local AABB half-extents. */
	FVector LocalBoundsExtent = FVector::ZeroVector;
	/** Current rigid rotation. */
	FQuat   Rot = FQuat::Identity;
	/** Current rigid translation. */
	FVector Trans = FVector::ZeroVector;
	/** Previous rigid rotation. */
	FQuat   PrevRot = FQuat::Identity;
	/** Previous rigid translation. */
	FVector PrevTrans = FVector::ZeroVector;
	/** 1 / frame dt. 0 means static. */
	float   InvDeltaTime = 0.0f;
};

/**
 * Per-bone SDF collider for GPU collision: a bone-local distance grid plus the bone-to-world transform.
 * Distances points into storage the caller (the asset) owns and is valid for the duration of the Step call —
 * it is copied on the game thread before the render command is issued.
 * The bytes are uint8 quantization codes; the game-thread flattening dequantizes them across the asymmetric
 * bands and uploads floats. Colliders sharing a VolumeKey share one GPU upload within a step.
 */
struct FRopeGPUSDFCollider
{
	/** Blob of code bytes, BytesPerCode per voxel, row-major, little-endian. Outside is positive. */
	const uint8* Distances = nullptr;
	/** Bytes per voxel (1 = uint8, max 255; 2 = uint16, max 65535). */
	int32        BytesPerCode = 1;
	/** Inner dequantization band (cm). Code 0 maps to -NarrowBandInner. */
	float        NarrowBandInner = 0.0f;
	/** Outer dequantization band (cm). The maximum code maps to +NarrowBandOuter. */
	float        NarrowBandOuter = 0.0f;
	int32        ResX = 0;
	int32        ResY = 0;
	int32        ResZ = 0;
	FVector      LocalMin = FVector::ZeroVector;
	FVector      LocalSize = FVector::ZeroVector;
	FTransform   BoneToWorld = FTransform::Identity;
	/** Previous frame's bone transform, for CCD and surface-velocity drag. */
	FTransform   PrevBoneToWorld = FTransform::Identity;
	/** 1 / frame dt, for surface velocity. 0 means static. */
	float        InvDeltaTime = 0.0f;
	uint64       VolumeKey = 0;
};

/**
 * Per-node bits in FRopeGPUResidentStep::OverrideFlags, 1:1 with the override stage of RopeXPBD.usf.
 * "Targets are computed on the game thread, applied on the GPU": this is how a logic phase (whip, wrapping,
 * hold, releasing) writes per-node targets and masses straight into the resident buffer without a reseed.
 * Application order is Position → Prev → PrevFromPosition → InvMass, where PrevFromPosition copies Pos
 * *after* Position has been applied.
 */
enum class ERopeGPUOverride : uint8
{
	None             = 0,
	// Pos[i]  = OverridePositions[i]
	Position         = 1 << 0,
	// Prev[i] = OverridePrevPositions[i] — the gap from Pos becomes the Verlet velocity, as the whip uses
	Prev             = 1 << 1,
	// Prev[i] = Pos[i] — pinned with zero velocity (wrapping, hold). Reads the GPU's current Pos, not the CPU mirror
	PrevFromPosition = 1 << 2,
	// InvMass[i] = OverrideInvMass[i] — persisted into the resident InvMass buffer (mass mask and restore)
	InvMass          = 1 << 3,
};
ENUM_CLASS_FLAGS(ERopeGPUOverride)

/**
 * One frame's step input for one resident rope. Self-contained — values and TArrays only — filled on the game
 * thread and moved to the render thread.
 * RopeId is the stable key identifying the persistent buffer, such as the component's UniqueID. Generation
 * rises whenever the CPU changes the sim out of band, by throwing or resizing, and the render thread reseeds
 * the GPU buffer when it sees a generation change, a node count change, or the rope for the first time.
 * SeedPositions, SeedPrevPositions and InvMass are supplied every frame but only uploaded on a reseed;
 * otherwise they are ignored.
 */
struct FRopeGPUResidentStep
{
	uint32 RopeId = 0;
	uint32 Generation = 0;
	int32  NumNodes = 0;

	/** Seed data, supplied every frame but uploaded only on a reseed. */
	TArray<FVector> SeedPositions;
	TArray<FVector> SeedPrevPositions;
	TArray<float>   InvMass;

	/** Sim and config scalars. */
	float   SegmentLength = 0.0f;
	bool    bStartPinned = false;
	FVector StartPinPrev = FVector::ZeroVector;
	FVector StartPinTarget = FVector::ZeroVector;
	float   StretchCompliance = 0.0f;
	/** Strain limiting: after each substep solve, hard-project every segment to at most this multiple of
	 *  SegmentLength (1.5 default; below 1 disables it). It is what stops the segments beside an anchor pin
	 *  over-stretching and jittering when a long chain hangs off one and the iterations cannot reach them. */
	float   MaxStretchRatio = 1.5f;
	float   BendCompliance = 0.0f;
	/** Bend tolerance: at or below this straightness the straightening force is 0, which relaxes the angle at a corner or a wrap boundary. */
	float   BendReleaseRatio = 0.70f;
	/** At or above this straightness the straightening force is full, so a gentle bend straightens as before. */
	float   BendFullRatio = 0.92f;
	float   Damping = 0.0f;
	int32   Iterations = 1;
	/** Collision resolve passes per substep, capped at Iterations. 1 resolves once at the end of the substep. */
	int32   CollisionPasses = 1;
	FVector Gravity = FVector::ZeroVector;

	/**
	 * Colliders to apply to this rope, copied by value so they stay valid for the life of the step.
	 * With bSolveCollisions false the solve kernel ignores the colliders and the GDF, but the detect kernel
	 * still uses the lists below unchanged.
	 */
	bool  bSolveCollisions = true;
	/** Rope node thickness (= FRopeSolverConfig::CollisionRadius). */
	float CollisionRadius = 0.0f;
	/** Tangential damping [0..1], the Coulomb μ. */
	float Friction = 0.0f;
	/** Friction multiplier toward the free end (1 at the pinned end), which lets the end node slide rather than grip. */
	float TipFrictionScale = 1.0f;
	/** Swept sample spacing (cm). */
	float SweepStep = 2.0f;
	/** Cap on samples per segment. */
	int32 MaxSweepSamples = 16;
	/** Push off static world geometry with the engine's Global Distance Field. Only valid on the scene-graph dispatch path. */
	bool  bUseWorldGDF = false;
	TArray<FRopeGPUCapsule>     Capsules;
	TArray<FRopeGPUSDFCollider> SDFColliders;
	/** Analytic boxes (OBB). All of them are solved against; the first NumDetectBoxes also take part in contact detection. */
	TArray<FRopeGPUBox>         Boxes;
	/** Analytic convexes (plane sets). Solve only. */
	TArray<FRopeGPUConvex>      Convexes;
	/** Flat body-local pool of every convex's planes, as (nx, ny, nz, w) with outward normals. */
	TArray<FVector4>            ConvexPlanes;

	/**
	 * How many capsules the contact detection kernel sees: only [0, NumDetectCapsules) at the front of
	 * Capsules take part. The caller (PackStepColliders) packs non-static capsules first and static world
	 * capsules after, in two passes, because detection keeps only the single deepest contact per node — a wall
	 * contact masking a bone contact would silently cost that node its wrap capture.
	 * -1, the default, lets every capsule take part, which is what the existing behaviour and tests expect.
	 */
	int32 NumDetectCapsules = -1;

	/**
	 * How many boxes the detection kernel sees: only [0, NumDetectBoxes) at the front of Boxes take part, under
	 * the same two-pass packing as capsules — wrappable boxes first, static ones after. 0, the default, keeps
	 * boxes out of detection entirely, which is the static-box-only legacy behaviour.
	 */
	int32 NumDetectBoxes = 0;

	/**
	 * How many convexes the detection kernel sees: only [0, NumDetectConvexes) at the front of Convexes take
	 * part, under the same contract as boxes — static convexes are appended after and so excluded automatically.
	 */
	int32 NumDetectConvexes = 0;

	/** Detection sweep spacing (cm) and sample cap — mirrors the CPU FParams::ContactSweepStep and ContactMaxSweepSamples. */
	float ContactSweepStep = 2.0f;
	int32 ContactMaxSweepSamples = 16;

	/**
	 * Signature of the collider attribution set this dispatch used, computed by the caller. It comes back
	 * unchanged in the detection readback, and comparing it is how the consumer knows a returned ColliderIndex
	 * can still be resolved (FRopeResidentContacts::AttribSig).
	 */
	uint32 AttribSig = 0;

	/** This frame's substep schedule, computed by the caller through RopeSolverSubsteps. NumSub <= 0 holds position without integrating. */
	int32 NumSub = 0;
	float FixedDt = 0.0f;

	/**
	 * --- Contact detection. After the solve, sweep PosBuf against PrevBuf to find the deepest contact per node.
	 * bDetectContacts runs the detection kernel after the solve dispatch and reads the results back
	 * (GetLatestContacts). ContactRadius is the detection query radius (= FRopeWrapConfig::ContactQueryRadius),
	 * separate from the solver's CollisionRadius.
	 */
	bool  bDetectContacts = false;
	float ContactRadius = 0.0f;

	/**
	 * --- Predicted contact. The path extrapolating each node's next position is swept too, so a contact about
	 * to happen is detected early. That gives two slots per node, actual and predicted. PredictionFrames <= 0
	 * disables prediction. On a frame where the whip is active, a guided node extrapolates through its
	 * previous, current and next targets (PredictiveGuided); every other node extrapolates its frame
	 * displacement (PredictiveFree).
	 * The WhipGuided* arrays are filled to NumNodes only while the whip is active, and are otherwise empty,
	 * leaving only the free prediction.
	 */
	float           PredictionFrames = 0.0f;
	/**
	 * Substep-to-frame displacement conversion factor (= DeltaTime / FixedDt). A free node's prediction uses the
	 * rope's Verlet displacement, which is the last substep's delta, so this converts it to a frame
	 * displacement; leaving it at 1 shrinks the prediction to substep scale. A guided node's prediction is a
	 * frame-level target difference and does not use it. Same meaning and value as the CPU
	 * FParams::FrameDeltaTime path.
	 */
	float           ContactFrameToSubstepRatio = 1.0f;
	/** Per-node guided flag (1 = guided). */
	TArray<uint8>   WhipGuidedMask;
	TArray<FVector> WhipCurrentTargets;
	TArray<FVector> WhipPrevTargets;
	TArray<FVector> WhipNextTargets;

	/**
	 * --- Override. Write per-node targets computed by a logic phase on the game thread straight into the
	 * resident buffer, in place of a reseed. Empty means no override.
	 * When filled, OverrideFlags must be exactly NumNodes long — a mismatch is ignored with a warning — and a
	 * value array only needs to be NumNodes long when some node actually sets the matching bit.
	 * An override dispatches even with NumSub = 0, recording the values without integrating, which is what a
	 * Wrapping or Releasing frame does.
	 * Holds the per-node OR of ERopeGPUOverride bits.
	 */
	TArray<uint8>   OverrideFlags;
	/** Valid only on nodes with the Position bit. */
	TArray<FVector> OverridePositions;
	/** Valid only on nodes with the Prev bit. */
	TArray<FVector> OverridePrevPositions;
	/** Valid only on nodes with the InvMass bit. */
	TArray<float>   OverrideInvMass;

	bool HasOverrides() const { return OverrideFlags.Num() > 0; }

	/**
	 * Carries the simulation time of a same-rope step that the render graph did not consume into this newer
	 * step. The newest logic payload remains authoritative, but its next render dispatch integrates the whole
	 * elapsed interval instead of returning the old interval to the game thread one frame later. Returns false
	 * across a reseed/topology/timestep boundary, where the caller must keep the ordinary refund path.
	 */
	bool MergeReplacedSimulationTime(const FRopeGPUResidentStep& Replaced)
	{
		if (RopeId != Replaced.RopeId || Generation != Replaced.Generation ||
			NumNodes != Replaced.NumNodes || FixedDt <= KINDA_SMALL_NUMBER ||
			Replaced.FixedDt <= KINDA_SMALL_NUMBER ||
			!FMath::IsNearlyEqual(FixedDt, Replaced.FixedDt))
		{
			return false;
		}

		// Each game-thread step already obeys the solver's per-frame workload cap. A pending replacement
		// commonly combines two otherwise valid 12-substep frames, so applying that same 18-step cap again
		// discards 25% of elapsed time and produces a small visible hitch. The shader supports 32 substeps;
		// use that separate resident catch-up bound for merged render work.
		constexpr int32 MaxResidentCatchUpSubsteps = 32;
		NumSub = FMath::Min(MaxResidentCatchUpSubsteps,
			FMath::Max(0, NumSub) + FMath::Max(0, Replaced.NumSub));
		return true;
	}
};

/** The resident rope's latest, slightly delayed, positions as read by the game thread. The render-thread readback fills it and the game thread copies under lock. */
struct FRopeResidentLatest
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	/**
	 * Per-segment tension (NumNodes - 1 entries, F = max(0, -λ)/h², the same units and meaning as
	 * FRopeSimState::SegmentTension). It is only armed and read back on a solve frame (NumSub > 0), so it can
	 * update less often than the positions; empty means it was never read back.
	 */
	TArray<float>   SegmentTension;
	/** The seed generation these positions belong to, so a reseed boundary cannot apply stale data. */
	uint32 Generation = 0;
	int32  NumNodes = 0;
};

/**
 * One GPU contact detection result. There can be at most one actual and one predicted contact per node.
 * The GPU cannot produce a bone or a mesh — an FName and a pointer are game-thread concepts — so it emits a
 * collider index instead, and the runtime caller turns that index back into a (bone, mesh) pair through the
 * attribution table. A 1:1 mirror of the HLSL FRopeGPUContact, in layout and in meaning.
 */
struct FRopeGPUContactResult
{
	int32   NodeIndex = INDEX_NONE;
	/** 0 = capsule, 1 = SDF, 2 = box, naming which of the step's Capsules, SDFColliders or Boxes arrays applies. */
	int32   ColliderType = 0;
	/** Index within that array — the key for restoring attribution. */
	int32   ColliderIndex = 0;
	/** ERopeContactCandidateSource: 1=Actual, 2=PredictiveFree, 4=PredictiveGuided. */
	uint8   Source = 1;
	float   Penetration = 0.0f;
	/** Contact point on the surface (the counterpart to FRopeContact::SurfacePoint). */
	FVector WorldPoint = FVector::ZeroVector;
	/** Unit outward normal, from the collider toward the node. */
	FVector Normal = FVector::UpVector;
	/** Surface velocity at the contact point (cm/s; 0 when static). */
	FVector SurfaceVelocity = FVector::ZeroVector;
};

/** The resident rope's latest, slightly delayed, contact detection results as read by the game thread, copied by GetLatestContacts. */
struct FRopeResidentContacts
{
	/** Only the slots the GPU filled with a valid contact. */
	TArray<FRopeGPUContactResult> Contacts;
	/** The seed generation these results belong to, so stale data cannot be applied. */
	uint32 Generation = 0;
	/**
	 * The collider attribution signature (FRopeGPUResidentStep::AttribSig) of the **dispatch** that produced
	 * this result. ColliderIndex refers to that dispatch's ordering, so a consumer compares this against its
	 * current signature to know whether the index still means the same thing — an exact correspondence, not the
	 * approximation "nothing has changed in the last N frames".
	 */
	uint32 AttribSig = 0;
};

/**
 * GPU (compute) implementation of the XPBD rope solver, with the centerline resident on the GPU.
 *  - Step: advances each rope's persistent GPU buffer one frame in place, with no round trip and no stall,
 *          reseeding from the CPU sim when it must (first sight, node count change, generation change), and
 *          consuming then re-arming the readback on the render thread.
 *  - GetLatest: copies, under lock, the latest positions the render-thread readback filled in (one to two
 *          frames behind). Used for render and collision.
 *  - ReleaseRope: frees a rope's persistent buffer and readback, on EndPlay and the like.
 * The instance state — the persistent buffer map — belongs to the render thread; the game-thread methods
 * either enqueue render commands or read the shared results. One instance per world. The CPU solver remains
 * the ground truth.
 */
class FRHIShaderResourceView;
class FRDGBuilder;
class FGlobalDistanceFieldParameterData;
class FSceneView;

namespace RopeGPU
{
	/**
	 * Can the GPU path — solver, detection and tube — run in this runtime? Beyond needing a renderable RHI,
	 * **the feature level must be SM5 or above**: every kernel is guarded by SM5 in its
	 * ShouldCompilePermutation, so no permutation exists on ES3.1 or mobile, and going only by "is there an
	 * RHI" would have the renderer ask for a shader that does not exist — an assert, a crash, or nothing drawn.
	 *
	 * The test reads the global GMaxRHIFeatureLevel, the highest this device can actually produce. The editor's
	 * mobile preview only lowers the *scene* feature level while the real RHI stays SM6, so the GPU path keeps
	 * running under preview. That is deliberate: changing the simulation path to follow the preview would make
	 * what you see stop matching what runs.
	 *
	 * Both the solver (URopeSimSubsystem) and the tube (FRopeSceneProxy) call it. Were the two gates to
	 * disagree, the rope could end up solving on the GPU while its tube built on the CPU, so the decision is
	 * kept to this single source of truth.
	 */
	DYNAMICROPESHADERS_API bool IsRuntimeSupported();
}

class DYNAMICROPESHADERS_API FRopeGPUSolver
{
public:
	/** Maximum nodes per rope, which is the largest compute thread group bucket. Callers must only pass steps
	 *  within this limit. It must match the top of the node bucket array in RopeGPUSolver.cpp, where a
	 *  static_assert enforces it. */
	static constexpr int32 MaxNodes = 512;

	FRopeGPUSolver();
	~FRopeGPUSolver();

	/**
	 * Render thread. Returns the SRV of the rope's resident PosBuf (StructuredBuffer<float4> of world
	 * positions), or null if there is none. The scene proxy reads this SRV directly and builds the tube on the
	 * GPU, so the tube carries no position lag and needs no render readback.
	 * If the solver has never stepped this rope — the GPU solver is off — it is null and the caller falls back
	 * to the CPU path.
	 *
	 * OutGeneration is the seed generation the buffer currently holds, and the caller must compare it against
	 * its own. Going by node count alone shows a one-frame ghost of the previous rope's pose on a frame that
	 * **reseeded** with the same node count, such as a re-throw, because the buffer still holds the old
	 * generation until that frame's dispatch runs.
	 */
	FRHIShaderResourceView* GetResidentPositionSRV_RenderThread(uint32 RopeId, int32& OutNumNodes,
		uint32& OutGeneration);

	/** Hand this frame's resident steps to the render thread and advance them in place on the GPU, without blocking.
	    The steps are consumed (moved). This executes immediately on its own RDG graph, which is the unit-test
	    harness path with no scene renderer; at runtime, use EnqueueSteps. */
	void Step(TArray<FRopeGPUResidentStep>&& Steps);

	/**
	 * The runtime dispatch path: stack the steps on the render thread's pending queue without dispatching.
	 * This frame's view flushes them onto the scene renderer's graph in PreRenderBasePass through
	 * DispatchPending_RenderThread, which is when the GDF parameters are valid.
	 */
	void EnqueueSteps(TArray<FRopeGPUResidentStep>&& Steps);

	/** Render thread. Records the accumulated pending steps onto the GraphBuilder it is given, without executing
	    it. GDF is this view's Global Distance Field parameters and may be null; PreViewTranslation is the
	    world-to-translated-world offset. */
	void DispatchPending_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView* View,
		const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation);

	/**
	 * Drain, per RopeId, the simulation time (in seconds) belonging to pending steps that were replaced before
	 * being consumed. Cleared as it is read, under lock.
	 *
	 * EnqueueSteps has replace semantics, so a frame whose view never reached the base pass has its steps
	 * overwritten by the next frame's and they vanish. Their substep time was already drawn out of the
	 * accumulator on the game thread, so left alone that **simulation time is permanently lost** — not a
	 * skipped frame that catches up later, but time gone for good.
	 * The caller returns this value to the accumulator before computing the next schedule, which keeps the
	 * accumulator the single truth of "time simulated" and lets RopeSolverSubsteps' existing cap absorb any
	 * resulting catch-up.
	 */
	void DrainDroppedSimTime(TMap<uint32, float>& Out);

	/** Copy, under lock, the latest positions the render-thread readback filled per RopeId. With nothing new arrived, it can return the previous values unchanged. */
	void GetLatest(TMap<uint32, FRopeResidentLatest>& Out);

	/** Copy, under lock, the latest contact detection results per RopeId. Same latency as GetLatest, roughly one to two frames. */
	void GetLatestContacts(TMap<uint32, FRopeResidentContacts>& Out);

	/**
	 * Game-thread blocking synchronous readback: run this rope's pending non-GDF render-thread steps first, then
	 * fetch the resident Pos and Prev *now*, waiting for the GPU to go idle.
	 * A pending step that needs the scene GDF has no valid view, so it is not executed: the call returns false
	 * and the step is preserved for the next scene dispatch.
	 * Only for the rare case where the very latest positions are genuinely required once per event, such as the
	 * wrap handoff — never call it per frame.
	 * OutGeneration is the seed generation the buffer holds, and the caller rejects stale data by comparing it
	 * against its own.
	 * @return true when a resident buffer exists and the fetch succeeded.
	 */
	bool ReadbackNow(uint32 RopeId, TArray<FVector>& OutPositions, TArray<FVector>& OutPrevPositions, uint32& OutGeneration);

	/** Free a rope's persistent buffer and readback, on the render thread. Called from the component's EndPlay and Unregister. */
	void ReleaseRope(uint32 RopeId);

private:
	/**
	 * Hide the resident state — the render-thread-only persistent buffer map, plus the results shared between
	 * game and render threads — behind a pimpl, so no RDG or RHI type appears in this header and no member has
	 * to be a by-value incomplete type.
	 */
	struct FImpl;
	TUniquePtr<FImpl> Impl;

	void ReleaseAll_RenderThread();

	/** Shared execution body of Step and DispatchPending_RenderThread: record the resident seeding, registration,
	    dispatch and readback onto the GraphBuilder. Executing it is the caller's job, and the caller empties the
	    steps once they are consumed. */
	void RunSteps_RenderThread(FRDGBuilder& GraphBuilder, TArray<FRopeGPUResidentStep>& Steps,
		const FSceneView* View, const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation);
};
