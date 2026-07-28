// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU (compute) implementation of the XPBD rope solver — M5: Centerline GPU resident.
// The position buffer is persisted between frames and advanced in-place on the GPU every frame → Sequential dependency is maintained within the GPU.
// is satisfied and there is no round-trip stall/slomo (resolving the limitation of M4 asynchronous readback). All readbacks (for render/collision) are
// Processed in the render thread (every frame step command locks and consumes the previous readback and then rearranges it) → No GT stall.
// Same formula as CPU FRopeXPBDSolver, but constraints are solved with red-black/stride-3 coloring (parallel safety).
// This module (DynamicRopeShaders) does not depend on the DynamicRope runtime type → step is a self-contained POD.
// The caller (runtime subsystem) fills in the steps from FRopeSimState/Config.

#pragma once

#include "CoreMinimal.h"

/** Analytical capsule for GPU collision (M2). world space segment(A-B) + radius. The caller extracts and fills the collider.*/
struct FRopeGPUCapsule
{
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;

	/**
	 * previous frame endpoint + 1/framedt (surface velocity drag/substep for relative motion CCD — PrevBoneToWorld counterpart in SDF).
	 * If InvDeltaTime=0 (default), static — Packing falls back to prev=current, so even if not filled, it is the same as the existing operation.
	 */
	FVector PrevA = FVector::ZeroVector;
	FVector PrevB = FVector::ZeroVector;
	float   InvDeltaTime = 0.0f;
};

/**
 * Analytical box (OBB) for GPU collision. Static world geometry (static body simple collision) is the default, and moving
 * Body/wrapable box participates in surface velocity·CCD with the frame motion below (PrevCenter/PrevRot + InvDeltaTime).
 * Reason for the existence of analytical queries that give exact diagonal normals at corners/edges (replaces GDF voxel rounding penetration).
 * caller extracts and fills with IRopeCollider::GetGPUBox(+GetGPUBoxMotion).
 */
struct FRopeGPUBox
{
	/** Center of world space box.*/
	FVector Center = FVector::ZeroVector;
	/** world space box rotation.*/
	FQuat   Rot = FQuat::Identity;
	/** local half-width (after scale reflection).*/
	FVector HalfExtents = FVector::ZeroVector;

	/**
	 * previous frame center/rot + 1/framedt (moving body surface velocity/substep CCD). If InvDeltaTime=0, static —
	 * The packing is filled with prev=current, so even if it is not filled, it is the same as the existing operation (corresponding to Prev* of the capsule).
	 */
	FVector PrevCenter = FVector::ZeroVector;
	FQuat   PrevRot = FQuat::Identity;
	float   InvDeltaTime = 0.0f;
};

/**
 * Analytical convex (plane set) for GPU collision. static/dynamic world geometry (convex simple collision + shear box).
 * The plane is stored in Step's ConvexPlanes flat pool [PlaneOffset, PlaneOffset+PlaneCount) (body-local, unit
 * Normal·Outside, PlaneDot(p)=dot(N,p)-W, rigid body not applied). world = local ∘ rigid body(Rot,Trans). LocalBounds is local
 * AABB (vaginal curl). The moving body is processed with surface velocity/CCD using prev rigid body + InvDeltaTime. The caller is extracted with GetGPUConvex.
 */
struct FRopeGPUConvex
{
	/** Starting index within the ConvexPlanes pool.*/
	int32   PlaneOffset = 0;
	/** Number of planes.*/
	int32   PlaneCount = 0;
	/** Body-local AABB center.*/
	FVector LocalBoundsCenter = FVector::ZeroVector;
	/** Body-local AABB half size.*/
	FVector LocalBoundsExtent = FVector::ZeroVector;
	/** rigid body rotation (curr).*/
	FQuat   Rot = FQuat::Identity;
	/** rigid body translation (curr).*/
	FVector Trans = FVector::ZeroVector;
	/** rigid body rotation (prev).*/
	FQuat   PrevRot = FQuat::Identity;
	/** rigid body translation (prev).*/
	FVector PrevTrans = FVector::ZeroVector;
	/** 1/framedt (static if 0).*/
	float   InvDeltaTime = 0.0f;
};

/**
 * per-bone SDF collider for GPU collision (M3). bone local distance grid + bone → world transform.
 * Distances is a pointer owned by the caller (asset) (valid during Step calls — copied from GT before moving to the render command).
 * Distances are uint8 quantization codes — during GT flattening, they are dequantized into asymmetric bands and uploaded to the float buffer.
 * If the VolumeKey is the same, GPU upload is shared (dedup) within the same step.
 */
struct FRopeGPUSDFCollider
{
	/** Blob of code bytes (BytesPerCode per voxel, row-first, little-endian). Outside +.*/
	const uint8* Distances = nullptr;
	/** Bytes per voxel (1=uint8 max255, 2=uint16 max65535).*/
	int32        BytesPerCode = 1;
	/** Inner dequant band (cm). Code 0 → -NarrowBandInner.*/
	float        NarrowBandInner = 0.0f;
	/** Outer dequant band (cm). Code max → +NarrowBandOuter.*/
	float        NarrowBandOuter = 0.0f;
	int32        ResX = 0;
	int32        ResY = 0;
	int32        ResZ = 0;
	FVector      LocalMin = FVector::ZeroVector;
	FVector      LocalSize = FVector::ZeroVector;
	FTransform   BoneToWorld = FTransform::Identity;
	/** previous frame bone transform (CCD/surfacevelocity drag).*/
	FTransform   PrevBoneToWorld = FTransform::Identity;
	/** 1/framedt (for surface velocity). If 0, static.*/
	float        InvDeltaTime = 0.0f;
	uint64       VolumeKey = 0;
};

/**
 * per-node bit (G0) in FRopeGPUResidentStep::OverrideFlags. 1:1 with the override stage of RopeXPBD.usf.
 * "target calculation is GT, application is GPU" — per-node calculated by logic phase (whip/Wrapping/hold/Releasing)
 * This is a passage that records target/mass directly into the resident buffer without reseeding. Application order: Position → Prev →
 * PrevFromPosition → InvMass (PrevFromPosition copies Pos *after* applying Position).
 */
enum class ERopeGPUOverride : uint8
{
	None             = 0,
	// Pos[i]  = OverridePositions[i]
	Position         = 1 << 0,
	// Prev[i] = OverridePrevPositions[i] (Difference from Pos becomes Verlet velocity — whip)
	Prev             = 1 << 1,
	// Prev[i] = Pos[i] — velocity 0 pinned(Wrapping/hold). Based on current Pos on GPU side (not CPU mirror).
	PrevFromPosition = 1 << 2,
	// InvMass[i] = OverrideInvMass[i] — persist (mass mask/restore) to resident InvMass buffer
	InvMass          = 1 << 3,
};
ENUM_CLASS_FLAGS(ERopeGPUOverride)

/**
 * One frame step input from one resident rope. self-contained (all values/TArray) — Filled from GT and MoveTemp to render thread.
 * RopeId is a stable key that identifies the persistent buffer (e.g. component UniqueID). Generation is a CPU that throws/resizes the Sim.
 * Increases when changed to out-of-band → RT detects generation change/node number change/first and reseeds the GPU buffer.
 * SeedPositions/PrevPositions/InvMass are provided every frame, but RT is actually uploaded only when reseed is needed (ignored in normal times).
 */
struct FRopeGPUResidentStep
{
	uint32 RopeId = 0;
	uint32 Generation = 0;
	int32  NumNodes = 0;

	/** Seed data (provided every frame; RT is uploaded to GPU only when reseeding).*/
	TArray<FVector> SeedPositions;
	TArray<FVector> SeedPrevPositions;
	TArray<float>   InvMass;

	/** sim/config scalar.*/
	float   SegmentLength = 0.0f;
	bool    bStartPinned = false;
	FVector StartPinPrev = FVector::ZeroVector;
	FVector StartPinTarget = FVector::ZeroVector;
	float   StretchCompliance = 0.0f;
	/** Strain limiting: Hard projection of each segment after substep solve with ≤ this scale × SegmentLength (1.5=default,
	 *  <1=disabled). Prevents anchor-adjacent overstretching/jitter caused by lack of iteration when a long chain hangs on an anchor pin.*/
	float   MaxStretchRatio = 1.5f;
	float   BendCompliance = 0.0f;
	/** Angle-allowed bending: If straightness ≤ this value, straightening force is 0 (relaxes corner/wrap boundary angles).*/
	float   BendReleaseRatio = 0.70f;
	/** straightness ≥ If this value, the straightening force is 100% (gentle bending straightens as before).*/
	float   BendFullRatio = 0.92f;
	float   Damping = 0.0f;
	int32   Iterations = 1;
	/** Number of collision resolution passes per substep (cap in Iterations). 1=End of substep once (existing).*/
	int32   CollisionPasses = 1;
	FVector Gravity = FVector::ZeroVector;

	/**
	 * collision(M2/M3). List of colliders to apply to this rope (value copy so valid for the life of the step).
	 * If false, the solve kernel ignores the collider/GDF, but the detect kernel can use the list below as is.
	 */
	bool  bSolveCollisions = true;
	/** rope node thickness (= FRopeSolverConfig::CollisionRadius).*/
	float CollisionRadius = 0.0f;
	/** Tangential damping [0..1](Coulomb μ).*/
	float Friction = 0.0f;
	/** Free end friction multiplier (pinned point=1, end=this value). Be sure to leave the end node well.*/
	float TipFrictionScale = 1.0f;
	/** Swept sample spacing (cm).*/
	float SweepStep = 2.0f;
	/** sample cap per segment.*/
	int32 MaxSweepSamples = 16;
	/** Phase 2c: Push static world with engine GDF (only valid for scene graph dispatch).*/
	bool  bUseWorldGDF = false;
	TArray<FRopeGPUCapsule>     Capsules;
	TArray<FRopeGPUSDFCollider> SDFColliders;
	/** Analytical box (OBB). It is used for solving, and the front NumDetectBoxes also participate in contact detection.*/
	TArray<FRopeGPUBox>         Boxes;
	/** Analytical convex (plane set). solve only.*/
	TArray<FRopeGPUConvex>      Convexes;
	/** Body-local flat pool of the entire convex ((nx,ny,nz,w), outer direction normal).*/
	TArray<FVector4>            ConvexPlanes;

	/**
	 * Number of capsules that the contact detection(detect) kernel will see. Only [0, NumDetectCapsules) in front of Capsules participates in detection —
	 * caller(PackStepColliders) packs the non-static capsule at the front and the static(world) capsule at the back in 2-pass.
	 * detection leaves only one deepest contact per node, so the wall (static) contact covers the bone (skeletal) contact, so wrap capture is not possible.
	 * Prevents silent failure. -1 (default) = Participate fully (compatible with existing behavior/tests).
	 */
	int32 NumDetectCapsules = -1;

	/**
	 * Number of wrapable boxes (OBB) seen by the detection kernel. Only [0, NumDetectBoxes) in front of the Boxes participate in detection (same as capsule)
	 * 2-pass packing: in front of the wrapable box, behind the static box). 0 (default) = No participation in detection (static box only — legacy behavior).
	 */
	int32 NumDetectBoxes = 0;

	/**
	 * Number of convexes that can be Wrapped by the detection kernel. Only the front of the convexes [0, NumDetectConvexes) participates in detection.
	 * (same contract as box — static convex is appended after and automatically excluded).
	 */
	int32 NumDetectConvexes = 0;

	/** detection sweep sample interval (cm) and sample number cap — CPU FParams::ContactSweepStep/ContactMaxSweepSamples mirror.*/
	float ContactSweepStep = 2.0f;
	int32 ContactMaxSweepSamples = 16;

	/**
	 * Signature (computed by the caller) of the collider attribution set used by this dispatch. It comes back as is in the detection readback,
	 * This is the basis for checking whether ColliderIndex can be interpreted at the time of consumption (FRopeResidentContacts::AttribSig).
	 */
	uint32 AttribSig = 0;

	/** This frame substep schedule (calculated and delivered by caller to RopeSolverSubsteps). If NumSub<=0, keep without integration.*/
	int32 NumSub = 0;
	float FixedDt = 0.0f;

	/**
	 * --- contact detection (G3): After solving in Flight, sweep PosBuf/PrevBuf to detect the deepest contact per node.
	 * bDetectContacts runs the detection kernel after solve dispatch and reads back the results (GetLatestContacts).
	 * ContactRadius is the detection query radius (= FRopeWrapConfig::ContactQueryRadius; separate from the solver's CollisionRadius).
	 */
	bool  bDetectContacts = false;
	float ContactRadius = 0.0f;

	/**
	 * --- Predicted contact (G3b): The path that extrapolates the next location of the node is also swept to detect the contact that will soon be reached. 2 slots per node
	 * (actual + predictive) output. If PredictionFrames<=0, no predictions. In the whip active frame, the guide node
	 * Extrapolates to the current/previous/next target (PredictiveGuided), and otherwise extrapolates to frame displacement (PredictiveFree).
	 * WhipGuided* is filled with NumNodes length only when whip is active (otherwise it is empty → only Free prediction).
	 */
	float           PredictionFrames = 0.0f;
	/**
	 * substep→frame displacement conversion factor (= DeltaTime / FixedDt). Free node prediction is rope verlet displacement (last
	 * Used to increase substep delta) to frame displacement — if 1, no conversion (substep unit reduction bug). guide node
	 * This coefficient is not used because prediction is a frame-level target difference. Same meaning/value as CPU FParams::FrameDeltaTime path.
	 */
	float           ContactFrameToSubstepRatio = 1.0f;
	/** Whether to guide per-node (1=guided).*/
	TArray<uint8>   WhipGuidedMask;
	TArray<FVector> WhipCurrentTargets;
	TArray<FVector> WhipPrevTargets;
	TArray<FVector> WhipNextTargets;

	/**
	 * --- Override(G0): Write the per-node target calculated by the logic phase (GT) directly to the resident buffer (replaces reseeding).
	 * If empty, no override. When populating, OverrideFlags is set to exactly NumNodes length (ignore all + warn if mismatch),
	 * The value array needs to be provided as NumNodes length only when there is a node that writes the corresponding bit.
	 * Even if NumSub=0, if there is an override, it is dispatched and only recorded without integration (e.g. Wrapping/Releasing frame).
	 * per-node ERopeGPUOverride bit OR
	 */
	TArray<uint8>   OverrideFlags;
	/** Only Position bit node is valid.*/
	TArray<FVector> OverridePositions;
	/** Only Prev bit node is valid.*/
	TArray<FVector> OverridePrevPositions;
	/** Only InvMass bit node is valid.*/
	TArray<float>   OverrideInvMass;

	bool HasOverrides() const { return OverrideFlags.Num() > 0; }
};

/** The latest (slightly delayed) location of the resident rope as retrieved by GT. RT readback fills in and GT copies under lock.*/
struct FRopeResidentLatest
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	/**
	 * Tension per segment (NumNodes - 1, F = max(0,-λ)/h² — Same units/meaning as FRopeSimState::SegmentTension).
	 * Since it is armed and recovered only in the solve(NumSub>0) frame, it may be updated more rarely than the position (if empty, not recovered).
	 */
	TArray<float>   SegmentTension;
	/** The seed generation that this location corresponds to (preventing stale application of reseed boundaries).*/
	uint32 Generation = 0;
	int32  NumNodes = 0;
};

/**
 * 1 GPU contact detection (G3) result. There can be a maximum of one actual contact and one predicted contact per node.
 * Since the GPU cannot create bone/mesh (FName/pointer, GT concept), it emits only the collider index,
 * caller (runtime) index → (bone, mesh)
 * Restore to the attribution table. 1:1 mirror of HLSL FRopeGPUContact (same layout/semantics).
 */
struct FRopeGPUContactResult
{
	int32   NodeIndex = INDEX_NONE;
	/** 0=capsule, 1=SDF, 2=box (step's Capsules/SDFColliders/Boxes array distinction).*/
	int32   ColliderType = 0;
	/** Index within the corresponding array (attribution restoration key).*/
	int32   ColliderIndex = 0;
	/** ERopeContactCandidateSource: 1=Actual, 2=PredictiveFree, 4=PredictiveGuided. */
	uint8   Source = 1;
	float   Penetration = 0.0f;
	/** surface contact point (corresponding to FRopeContact.SurfacePoint).*/
	FVector WorldPoint = FVector::ZeroVector;
	/** Outer (collider→node) unit normal.*/
	FVector Normal = FVector::UpVector;
	/** Contact point surface velocity (cm/s; 0 if static).*/
	FVector SurfaceVelocity = FVector::ZeroVector;
};

/** The latest (slightly delayed) contact detection result of the resident rope retrieved by GT. Copy to GetLatestContacts.*/
struct FRopeResidentContacts
{
	/** bHit slots only (valid contacts filled by GPU).*/
	TArray<FRopeGPUContactResult> Contacts;
	/** Corresponding seed generation (preventing stale application).*/
	uint32 Generation = 0;
	/**
	 * Collider attribution signature (as FRopeGPUResidentStep::AttribSig) at the **dispatch point** that created this result.
	 * ColliderIndex points to the current collation order, so the consumer can compare this value directly with his current signature.
	 * Check whether the index still has the same meaning — “Nframe has not changed recently” is not an approximation, but an exact correspondence.
	 */
	uint32 AttribSig = 0;
};

/**
 * GPU (compute) implementation of the XPBD rope solver. Centerline GPU resident (M5a).
 *  - Step: Advances the persistent GPU buffer for each rope by one frame in-place every frame (no round trip/stall).
 *                Reseed in CPU Sim when necessary (initial/node number/generation change). Readback consume+rearm at RT.
 *  - GetLatest: Copy the latest position filled by RT readback (about 1-2 frame delay) under lock. For render/collision.
 *  - ReleaseRope: Rope Releases persistent buffer/readback (EndPlay, etc.).
 * The instance state (persistent buffer map) is owned by the render thread — GT methods enqueue render commands or read shared results.
 * Owns 1 world star. CPU solver remains ground-truth.
 */
class FRHIShaderResourceView;
class FRDGBuilder;
class FGlobalDistanceFieldParameterData;
class FSceneView;

namespace RopeGPU
{
	/**
	 * Can GPU path (solver·detection·tube) be used in this runtime? In addition to having a renderable RHI
	 * It is important to note that **the feature level is SM5 or higher** — the kernel is all compiled with SM5 guard (each CS
	 * ShouldCompilePermutation) Permutation does not exist in ES3.1/mobile. Just looking at the presence or absence of RHI
	 * Render requests a shader that does not exist on the mobile and goes to assert/crash/no output.
	 *
	 * The check standard is global GMaxRHIFeatureLevel (= the maximum value that this device can actually produce). editor's
	 * The mobile preview only lowers the scene feature level, and the actual RHI remains SM6, so the GPU path is maintained.
	 * This is the intended behavior (if the preview to simulation path is changed, the reproduction is out of sync).
	 *
	 * The caller is both the solver (URopeSimSubsystem) and the tube (FRopeSceneProxy) — if the two gates are misaligned,
	 * The solver is a GPU, but the tube is in a half-state like a CPU, so the check is set to a single source of truth.
	 */
	DYNAMICROPESHADERS_API bool IsRuntimeSupported();
}

class DYNAMICROPESHADERS_API FRopeGPUSolver
{
public:
	/** Maximum number of nodes per rope (= top compute thread group bucket). The caller must only pass steps within this limit.
	 *  Must match the top of the node bucket array in RopeGPUSolver.cpp (static_assert there is mandatory).*/
	static constexpr int32 MaxNodes = 512;

	FRopeGPUSolver();
	~FRopeGPUSolver();

	/**
	 * render thread. Rope's resident PosBuf (StructuredBuffer<float4>, world position) Returns SRV (null if not present).
	 * M5b B2-lite: Scene proxy reads this SRV directly and creates a tube on the GPU → no position delay (no render readback).
	 * If the solver has never stepped on this rope (= GPU solver off), null → the caller falls back to the CPU path.
	 *
	 * OutGeneration is the seed generation contained in this buffer. The caller must compare its generation —
	 * If you look at the number of nodes, the pose of the previous rope is read as is in the frame that was **reseeded** (rethrowing, etc.) with the same number of nodes.
	 * One frame ghost appears (the buffer is holding the old generation until that frame is dispatched).
	 */
	FRHIShaderResourceView* GetResidentPositionSRV_RenderThread(uint32 RopeId, int32& OutNumNodes,
		uint32& OutGeneration);

	/** Pass this frame's resident steps to the render thread and advance in-place on the GPU (no blocks). The step is consumed (MoveTemp).
	    Execute immediately on a dedicated (self) RDG graph — unit test harness path without a scene renderer (EnqueueSteps at runtime).*/
	void Step(TArray<FRopeGPUResidentStep>&& Steps);

	/**
	 * Runtime dispatch path: Just stacks steps in the render thread pending queue (does not dispatch). View expansion this frame
	 * In PreRenderBasePass, flush the scene renderer graph with DispatchPending_RenderThread (GDF parameter valid timing).
	 */
	void EnqueueSteps(TArray<FRopeGPUResidentStep>&& Steps);

	/** render thread. The accumulated pending steps are Loaded on the received (scene renderer) GraphBuilder (does not execute itself).
	    GDF is the Global Distance Field parameter of this view (can be null), PreViewTranslation is world→TranslatedWorld offset.*/
	void DispatchPending_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView* View,
		const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation);

	/**
	 * Recovers the simulation time of the pending step that was not consumed and replaced in seconds per RopeId (cleared immediately upon call, lock).
	 *
	 * EnqueueSteps are replacement semantics, so the steps of a frame that did not have a base pass for view expansion are transferred to the next frame step.
	 * It is covered and disappears. However, the substep time is already made by cutting the accumulator in GT, so if you leave it alone,
	 * **Simulation time is permanently lost** (It is not a frame being skipped, but time is lost, so it will not be made up later).
	 * The caller returns this value to the accumulator before the next schedule calculation — the accumulator then returns to
	 * becomes the single truth of "simulated time", and the existing cap of RopeSolverSubsteps automatically blocks the rush.
	 */
	void DrainDroppedSimTime(TMap<uint32, float>& Out);

	/** Copy (lock) the latest position filled by RT readback by RopeId. If nothing new arrives, it can be returned with the previous value maintained.*/
	void GetLatest(TMap<uint32, FRopeResidentLatest>& Out);

	/** Copy (lock) the latest contact detection results filled by RT readback by RopeId. Same latency as GetLatest (approximately 1-2 frames).*/
	void GetLatestContacts(TMap<uint32, FRopeResidentContacts>& Out);

	/**
	 * GT blocking synchronous readback (M5c): This rope's RT pending non-GDF step is executed first, then resident Pos/Prev.
	 * Get the value *now* (including waiting for GPU idle). Pending steps that require Scene GDF do not have a valid View.
	 * Does not execute, returns false, and preserves until the next scene dispatch.
	 * wrap Only for applications where "up-to-date location is absolutely necessary, once per event" such as handoffs — do not call every frame.
	 * OutGeneration is the seed generation that the buffer corresponds to (caller rejects stale by comparing it with its own generation).
	 * @return true if there is a resident buffer and retrieval is successful.
	 */
	bool ReadbackNow(uint32 RopeId, TArray<FVector>& OutPositions, TArray<FVector>& OutPrevPositions, uint32& OutGeneration);

	/** Release rope's persistent buffer/readback (in the render thread). Called from component EndPlay/Unregister.*/
	void ReleaseRope(uint32 RopeId);

private:
	/**
	 * Hide resident state (render thread-only persistent buffer map + GT<->RT shared results) with pimpl — RDG/RHI type in header
	 * is not exposed, and sizeof requirements are also avoided when storing incomplete types by-value as members (pointer members).
	 */
	struct FImpl;
	TUniquePtr<FImpl> Impl;

	void ReleaseAll_RenderThread();

	/** Step()/DispatchPending_RenderThread shared Execution unit: received resident seed/register/dispatch/readback
	    Loaded on GraphBuilder (Execute is the caller's responsibility). Steps are emptied by the caller after consumption.*/
	void RunSteps_RenderThread(FRDGBuilder& GraphBuilder, TArray<FRopeGPUResidentStep>& Steps,
		const FSceneView* View, const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation);
};
