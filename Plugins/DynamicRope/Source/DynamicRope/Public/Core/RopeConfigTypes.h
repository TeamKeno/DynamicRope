// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RopeConfigTypes.generated.h"

/**
 * Where the wrapping axis is placed (FRopeWrapConfig::WrappingAxisSource).
 * Both modes take the axis direction from the normal of the rope's travel plane; they differ in where the
 * axis origin sits and what the wrap is measured against.
 * The fallback chain behind them is shared — bone → parent → component base → bone-local X, in
 * FRopeWrappingPhase::ResolveWrappingAxis.
 */
UENUM(BlueprintType)
enum class ERopeWrappingAxisSource : uint8
{
	/**
	 * Bone-centred guide plane: the axis passes through the latch bone, and the wrap is measured against the
	 * latch tangent. Suited to paths that re-resolve the axis per bone, such as a single-bone wrap under
	 * AssistedJudged.
	 */
	BoneCenteredGuidePlane = 0 UMETA(DisplayName = "Bone-Centered Guide Plane"),

	/**
	 * Capture travel plane: the axis direction is the normal of the plane the rope swung through, and the
	 * origin is corrected onto the centre of the contacted collider cluster. Measuring the wrap against the
	 * velocity at capture suits a composite wrap. If no travel plane can be built, it falls back to the
	 * shared bone and component chain above.
	 */
	CaptureTravelPlane = 1 UMETA(DisplayName = "Capture Travel Plane")
};

/** XPBD solver tuning, for designers. What is saved here is what the runtime uses. */
USTRUCT(BlueprintType)
struct FRopeSolverConfig
{
	GENERATED_BODY()

	/** Physics substeps per frame. More substeps beat more iterations against tunnelling. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1", ClampMax = "16"))
	int32 Substeps = 12;

	/** Constraint iterations per substep. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1"))
	int32 Iterations = 4;

	// 1 means collisions are resolved once at the end of the substep. Where a bend is sharp enough that a
	// single pass cannot overcome the inward pull of the distance and bending constraints, splitting the
	// iterations into this many groups and resolving collisions between them defends steeper angles, at
	// proportionally higher cost. Capped at Iterations.

	/** Collision resolve passes per substep. Raise it only for very sharp bends. */
	int32 CollisionPassesPerSubstep = 1;

	// **Whatever the value, contacts are always solved on the *last* iteration of every collision pass.**
	// That is the load-bearing half of the contract: without a solve at the end, the distance and bending
	// constraints get the final word and can push a node back inside the surface, so the substep ends
	// penetrating. Setting N to Iterations or more therefore means "exactly once per pass" — the same
	// cadence the GPU kernel uses.
	// Solving on every iteration holds sharper included angles, because collision competes with distance and
	// bending each time instead of being overruled. Raising N cuts the number of those contests, reducing
	// cost linearly and the penetration margin with it. It is the one handle that directly trades CPU
	// fallback cost against quality when the colliders are expensive SDFs.
	// Not exposed to the Details panel: FRopeXPBDSolver is the only consumer, so it **only affects the CPU
	// fallback**. The GPU is the single runtime path in normal play (cook, -nullrhi, server and oversized
	// ropes are the CPU cases), so this has no effect there.

	/** How often contacts are solved: 1 = every iteration, N = every Nth. CPU fallback only. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1"))
	int32 ContactSolveInterval = 1;

	/** Stretch compliance — the inverse of stiffness. 0 is inextensible. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", DisplayName = "Stretch Softness"))
	float StretchCompliance = 0.0f;

	// A hard projection that walks the chain from the pinned and anchored nodes after each substep solve and
	// clamps every segment to at most this multiple of SegmentLength, moving Prev with it so the correction
	// injects no velocity. It exists because the XPBD distance constraint is Gauss-Seidel: with few
	// iterations the correction never propagates to the far end of a long chain hanging off an anchor pin,
	// and the segments next to the anchor run away — six times their length and more — with enormous tension
	// and a tangential whip jitter. A sequential sweep propagates along the whole chain in one go and
	// confines that runaway under the cap, which is the standard PBD long-range constraint remedy.

	/** Longest a segment may stretch, as a multiple of its rest length. 1.0 is fully inextensible; 0 or below 1 disables the clamp. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", DisplayName = "Max Stretch"))
	float MaxStretchRatio = 1.5f;

	/** Bending compliance. Higher is limper. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", DisplayName = "Bend Softness"))
	float BendCompliance = 0.02f;

	// Releases the straightening force as a bend gets steeper, which takes the angular bounce out of a free
	// node at a corner or a wrap boundary. The measure is r = (distance from node i to i+2) / (2 ×
	// SegmentLength) = cos(half the turn angle): 1 is straight, and smaller is a steeper bend.
	// At r ≤ BendReleaseRatio the straightening force is 0, at r ≥ BendFullRatio it is full, and it
	// smoothsteps between them. Raise BendReleaseRatio if corners are still too stiff; set both to 0 to
	// straighten always and switch the tolerance off. Lower BendFullRatio if a free rope hangs too limply.
	// The solver guarantees BendFullRatio stays above BendReleaseRatio.

	/** Bend tolerance floor: at or below this ratio (≈91° turn) the rope is not straightened at all. */
	float BendReleaseRatio = 0.70f;

	/** Bend tolerance ceiling: at or above this ratio (≈46° turn) the rope is straightened fully. */
	float BendFullRatio = 0.92f;

	/** Tangential friction against colliders, the Coulomb coefficient μ [0..1]. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Friction = 0.5f;

	// The end node carries the least tension, so friction grips it most easily and it stops sliding; easing
	// the grip toward the end lets it run.

	/** Friction multiplier toward the free end (1 at the pinned end). 1.0 means uniform friction. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Tip Grip Falloff"))
	float TipFrictionScale = 1.0f;

	// Resolved once at the component boundary (GetEffectiveCollisionRadius); the solver and the GPU step only
	// ever see the resolved value.
	// NOTE: a consumer that uses this struct without a component — a unit test, or a direct solver call — has
	// no auto resolution, and 0 there really does mean radius 0. Set an explicit value in that case.

	/** How far the solver holds nodes off a contact surface (cm). 0 = auto, matching the render tube radius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Collision Radius (0=Auto)"))
	float CollisionRadius = 0.0f;


	/** Collision sweep sample spacing (cm) along the path a node travelled in one substep. Lower resists
	 *  tunnelling better and costs more queries; move it together with MaxSweepSamples. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning",
		meta = (ClampMin = "0.1", Units = "cm"))
	float SweepStep = 2.0f;

	/** Cap on sweep samples per segment. A very fast node hits this cap and its samples spread out, so raise
	 *  it too when lowering SweepStep alone stops helping. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning",
		meta = (ClampMin = "1", ClampMax = "64"))
	int32 MaxSweepSamples = 16;

	/** Gravity applied to the rope (cm/s²). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	// The unit is the per-frame velocity loss at 60fps and is independent of the substep count, because
	// Integrate compensates exponentially. At 0.02 the survival rate over a second is 0.98^60 ≈ 0.30, an
	// effective drag of k ≈ 1.2/s, which puts free-fall terminal velocity near 8 m/s.
	// Raising it bleeds speed and kills momentum quickly, which reads as a light ribbon; lower it for weight.

	/** Air resistance. Higher damping makes the rope feel lighter and lose momentum faster. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Motion Damping"))
	float Damping = 0.02f;

	//~ Scaling (sleep and LOD) — cuts the cost of idle and distant ropes when many are active -----------
	// Sleep: while Free or Wrapped, if every node's speed stays under SleepVelocityThreshold for SleepDelay,
	// the solve is skipped. Free then dispatches nothing at all; Wrapped keeps running its logic — bone
	// follow (Hold), traction and auto-release — and only the free span's solve pauses, so the GPU sees an
	// override-only dispatch. It wakes on a moving pin, on reeling, on a collider moving nearby, on the
	// wrapped bone moving the nodes, on an active pull arriving, and on any phase transition.
	// The other phases (Flight, Contacting, Wrapping, Releasing) never sleep.

	/** Let an idle rope stop solving. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling")
	bool bAllowSleep = true;

	/** Speed below which a node counts as idle (cm/s), measured as its per-frame displacement over dt. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.1", DisplayName = "Sleep Speed Threshold"))
	float SleepVelocityThreshold = 3.0f;

	/** How long every node must stay slow before the rope sleeps (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "s"))
	float SleepDelay = 0.5f;

	/**
	 * Beyond LODStartDistance from the player camera, constraint iterations fall off linearly, reaching
	 * LODMinIterationScale at LODEndDistance — convergence error is invisible at that range. With no camera,
	 * as on a dedicated server, iterations always stay full.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling")
	bool bEnableDistanceLOD = true;

	/** Distance at which iteration fall-off begins (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "LOD Start"))
	float LODStartDistance = 3000.0f;

	/** Distance at which fall-off reaches its floor (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "LOD End"))
	float LODEndDistance = 8000.0f;

	/** Iteration multiplier at maximum distance (1 = no reduction). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.05", ClampMax = "1.0", DisplayName = "LOD Min Iterations"))
	float LODMinIterationScale = 0.25f;
};

/**
 * Wrap detection tuning: when does a rope draped over a limb count as wrapped?
 * The fields shown by default are the balance-level knobs; the AdvancedDisplay ones are for detection
 * infrastructure, which is mostly specific to AssistedJudged combined with a bare wrap. GuaranteedWrap runs
 * no detection and builds no path, so none of the advanced fields apply to it.
 */
USTRUCT(BlueprintType)
struct FRopeWrapConfig
{
	GENERATED_BODY()

	/**
	 * Whether a SurfaceVectorField path point may move to a neighbouring candidate bone instead of staying
	 * pinned to the one latch bone. With it off the candidate graph has zero depth and cost, only the current
	 * bone is evaluated, and the behaviour reduces to a single-bone wrap.
	 *
	 * Not exposed to the Details panel: switching it off only degrades the wrap, so there is no reason to
	 * beyond debugging.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap")
	bool bEnableMultiBoneWrapping = true;

	/**
	 * The probe radius for surface queries. It belongs to no single step — detection (Flight and Contacting)
	 * and establishment (path build projection, the snap cap, DecideWrap) all share it.
	 * Resolved at the component boundary (GetEffectiveContactQueryRadius), so consumers see only the resolved
	 * value. A consumer without a component must set an explicit value.
	 *
	 * Not exposed to the Details panel: auto follows the render tube radius, so setting the radius alone
	 * keeps a preset coherent — and entering an explicit value is exactly what breaks that.
	 */
	/** Surface query probe radius (cm). 0 = auto, the render tube radius × 1.5. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", Units = "cm"))
	float ContactQueryRadius = 0.0f;

	//~ Capture detection -----------------------------------------------------------
	// The thresholds that decide a Flight has caught something, and the contact sweep those thresholds read.
	// The three thresholds are on the FullSimulation and AssistedJudged detection path only; GuaranteedWrap
	// resolves through GuidedThrow and never consults them. The sweep is Flight detection itself, so it
	// applies whatever the mode.

	// A default of 1 is as lenient as it goes. Keeping poor wraps out is the job of the commit-time angle
	// gates (FailedWrapMinAngleDeg, CommitMin*), which measure it far more accurately. What this threshold is
	// still worth is stopping a hopeless wrap from starting at all, so no path build is wasted and no false
	// start is visible.
	// Not exposed to the Details panel for that reason.

	/** Rope nodes that must touch one bone before it counts as a catch rather than a graze. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "1"))
	int32 MinLatchNodes = 1;

	/**
	 * Legacy serialized value retained for asset and Blueprint compatibility. Wrapping now commits
	 * immediately after MinLatchNodes real contacts produce a valid seed, so this value has no runtime
	 * effect.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", Units = "s"))
	float WrapDecisionTime = 0.016f;

	// ContactSweepStep is the more direct lever on the same symptom, a fast throw passing through its target;
	// its comment gives the order to try things in. Not exposed to the Details panel for that reason.

	/** Flight prediction lookahead, in multiples of the frame displacement, so thin limbs are not missed. 0 disables it. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", ClampMax = "4.0"))
	float PredictiveContactFrames = 1.0f;

	/**
	 * Spacing at which the path a node travelled this frame is sampled for its deepest contact.
	 * **This is the anti-tunnelling value**, so keep it under the thickness of the thinnest thing worth
	 * catching — a forearm, a handrail. Lower it when a fast throw passes through its target without
	 * capturing. Same idea as FRopeSolverConfig::SweepStep, but detection runs during Flight only and has its
	 * own budget.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.1", Units = "cm", DisplayName = "Sweep Step"))
	float ContactSweepStep = 2.0f;

	/** Cap on samples for the detection sweep above. A very fast node hits this cap and its samples spread
	 *  out, so raise it too when lowering ContactSweepStep alone stops helping. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "1", ClampMax = "64", DisplayName = "Max Sweep Samples"))
	int32 ContactMaxSweepSamples = 16;

	/**
	 * How many wrap seeds — contact targets — one capture may adopt. 1, the default, is a single seed.
	 * At 2 or more, alongside the dominant target, contacts that dwell on a *different* (mesh, bone) further
	 * toward the tail than the dominant latch are wrapped as secondary seeds. Wrapping both legs is the
	 * motivating case: one leg is wrapped, and the contact nodes on the other are pinned to that bone too.
	 * Only the dominant seed gets a wrapping path — a secondary seed holds its bone with the contact nodes it
	 * already has, and no path is built around it.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "1", ClampMax = "8"))
	int32 MaxWrapSeeds = 1;

	/**
	 * Where the wrapping axis comes from. BoneCenteredGuidePlane, the default, puts the axis on the latch
	 * bone, which suits an AssistedJudged single-bone wrap; CaptureTravelPlane centres it on the contacted
	 * cluster, which is what a composite wrap needs.
	 *
	 * Exposed because the right value differs per preset — a bola wants CaptureTravelPlane, a single-bone
	 * catch wants BoneCenteredGuidePlane — though setting it through a preset beats setting it per instance.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (DisplayName = "Axis Source"))
	ERopeWrappingAxisSource WrappingAxisSource = ERopeWrappingAxisSource::BoneCenteredGuidePlane;

	/**
	 * How far a SurfaceVectorField path may cross open space on a tangent chord, with no surface under it.
	 * 0, the default, is off: an interrupted projection fails the path build outright.
	 * Switching it on changes two things, and together they are what makes it possible to wrap something
	 * forked, such as a pair of legs:
	 *  - a projection that would snap to a surface more than one segment from the predicted point is refused,
	 *    and the path stays on the chord. (Off, the query radius keeps its lenient snap.)
	 *  - path points on a chord raise no anchors, so after commit those nodes stay free rope. The solver
	 *    holds them slung straight, and they take tension when the target's legs spread — which is what
	 *    binding actually feels like.
	 * Failing to regain a surface within this distance falls back to the same failure handling as before.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Max Gap Bridge"))
	float WrappingMaxGapBridgeDistance = 0.0f;

	// A long rope spirals around its target several times — 3000° to 4400° in practice — and reseeding onto a
	// new bone spreads it to the neck and head, an "octopus wrap". Rope left outside the path when the cap
	// closes it is frozen for the rest of the Wrapping phase and becomes free span after commit; the front
	// motion does not drag nodes beyond the path. For a two-leg bola, 400° to 540° — one turn plus margin —
	// reads naturally.
	// **It does not apply to a composite analytic helix.** The strategy is not chosen by hand: the *target
	// geometry* decides it at runtime, and two or more bones bound to the same pose-space island make the
	// wrap composite (bPathUsesPoseSpaceIsland in the path build). A single forearm is sequential and takes
	// this cap; a pair of legs is composite and does not. Which means the multi-bone case most likely to
	// produce the octopus wrap this value guards against is exactly the case it cannot reach — filter that
	// one with CommitMinWrapAngleDeg and CommitMinWrapCoverageDeg instead.

	/** Total wrap angle at which the path build stops early and *succeeds* (deg). 0 = unlimited. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.0", Units = "deg", DisplayName = "Max Wrap Angle"))
	float WrappingMaxWrapAngleDeg = 0.0f;

	/** How long the accumulated latch span must hold steady before Wrapping commits (s). */
	float WrappingStableTime = 0.10f;

	/**
	 * **Floor** on how long the Wrapping phase lasts — not how long wrapping takes. The two paths use it
	 * differently.
	 *
	 * On the normal path, where the wrap angle maps to time, WrappingAngularSpeedDegPerSec sets the speed and
	 * the duration falls out of it. This value is only a floor: even if the front arrives early, the commit
	 * waits until it has elapsed, and during that wait the rope **sits still in its finished wrap pose**,
	 * because Wrapping is logic-driven and the solver is off. So what this value really sets is the gap
	 * between "the wrap finished" and "tension begins" at the entry into Wrapped — put it below the real
	 * wrapping time to remove that pause.
	 *
	 * On the DistanceFallback path, where no angle mapping is possible, it is the other way round: this value
	 * sets the actual duration, and the front speed is FullDistance / (this value + the tail delay). The
	 * angular speed setting is unused there.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap",
		meta = (ClampMin = "0.01", Units = "s", DisplayName = "Min Wrap Duration"))
	float WrappingMotionDuration = 0.20f;

	/**
	 * Nominal wrapping speed (deg/s), before the animation's easing is applied. Both the single and composite
	 * paths advance FrontWrapAngleRad at this rate. The default of 1100 deg/s matches the measured single-bone
	 * behaviour it replaced, a base speed of roughly 203 cm/s over 10.57 cm/rad.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap",
		meta = (ClampMin = "1.0", ClampMax = "7200.0", Units = "deg/s", DisplayName = "Wrap Speed"))
	float WrappingAngularSpeedDegPerSec = 1100.0f;

	// A policy constant settled by measurement, so there is no basis for picking a different one — which is
	// why it is not exposed to the Details panel. Wrapping speed and duration are
	// WrappingAngularSpeedDegPerSec and WrappingMotionDuration.

	/** Settle time between the front reaching its target and the Wrapped commit (s), which absorbs the pop as the last node hands over to its anchor. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.0", ClampMax = "1.0", Units = "s"))
	float WrappingPostFrontSettleTime = 0.08f;

	/** Extra delay per segment so a tail node settles onto the surface path. DistanceFallback path only,
	 *  where the commit deadline rules out angle mapping. */
	float WrappingTailDelayPerSegment = 0.024f;

	// The real budget is a size-proportional reference figure multiplied by this (FRopeWrappingPhase::
	// ComputePathStepBudget, whose base is 8 — so 8 is 1×). Raising it finishes the wrapping path sooner at a
	// higher per-frame cost.
	// Not exposed to the Details panel: the reason anyone would raise it, "path building is slow on a big
	// rope", is already handled by scaling the budget with rope size.

	/** Multiplier on the surface path integration budget per frame — a multiple, not a step count. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "1", ClampMax = "128"))
	int32 WrappingPathBuildStepsPerFrame = 8;

	/**
	 * Axial distance gained per unit of circumferential travel for a sequential SurfaceVectorField wrap.
	 * A composite analytic helix ignores it and derives the pitch from the tail's slope at contact.
	 *
	 * Not exposed to the Details panel: which strategy runs is decided at runtime by the target's geometry
	 * (see the pose-space island note on WrappingMaxWrapAngleDeg), so changing the value and watching the
	 * result teaches nothing about cause and effect.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "-2.0", ClampMax = "2.0"))
	float WrappingHelixPitchScale = 0.25f;

	// The multi-bone projection scoring constants below — depth, cost, weights, bonuses and hysteresis — were
	// settled by tuning and are developer constants, adjusted in code rather than in the Details panel. The
	// on/off switch for the whole thing is bEnableMultiBoneWrapping above. Each value is documented in place.

	/**
	 * How many edges from the current bone stay in the candidate set. Only skeleton parent and child edges
	 * are used for now; a designer-authored transition added later would pass the same depth limit, which is
	 * the first safeguard against a distant bridge opening in one step.
	 */
	int32 MaxBoneTransitionDepth = 3;

	/**
	 * Cap on a candidate's accumulated graph cost. Two candidates at the same depth can cost differently once
	 * edges carry different penalties. Only parent and child edge costs accumulate today; the cap earns its
	 * keep once designer-authored or near-forbidden edges join them, by cutting candidates before projection.
	 */
	float MaxBoneTransitionCost = 5.0f;

	/**
	 * Cost of traversing one automatic parent/child edge. Higher makes staying on the current bone easier;
	 * lower makes the rope work its way along the parent/child chain more readily.
	 */
	float AutoParentChildTransitionPenalty = 1.0f;

	/** Weight on projection distance. The further the surface is from the predicted point, the worse the score. */
	float ProjectionDistanceWeight = 0.35f;

	/** Weight on the distance from the rope node itself to the projected surface point, favouring the bone the rope is actually beside. */
	float RopeNodeDistanceWeight = 0.25f;

	/** Weight on the bend between the old and new tangents. Higher prefers smoother progress. */
	float TangentContinuityWeight = 8.0f;

	/** Weight on the bend between the old and new normals. Higher prefers continuous surface normals. */
	float NormalContinuityWeight = 5.0f;

	/** Weight on graph cost, penalizing a candidate that crosses more parent/child edges. */
	float BoneTransitionPenaltyWeight = 1.0f;

	/** Bonus for staying on the current bone, which steadies the bone choice near the tie point. */
	float CurrentBoneBonus = 0.35f;

	/** How much better a new bone must score before the wrap moves to it — transition hysteresis. */
	float BoneTransitionHysteresis = 0.75f;

	/** Penalty on a candidate that returns straight to the previous bone, discouraging A → B → A. */
	float ImmediateBoneReturnPenalty = 1.5f;

	/** Path distance that must pass after a bone transition before the next is allowed (cm). 0 disables it. */
	float MinBoneTransitionPathDistance = 8.0f;

	/** Longest the physics-driven wrapping settle may run before the best accumulated anchors are committed (s). */
	float WrappingMaxSettleTime = 0.90f;

	// Why an angle rather than the turn count it replaced: one turn costs 2πr of rope, so the bigger the
	// target the more absolute length it demands — a body of radius 100 cm needs 628 cm per turn, which a
	// default 200 cm rope simply cannot pay, and wrapping large targets broke structurally as a result. The
	// wrapped angle measures hook quality independently of target size (120° is a third of a turn).

	/** Below this wrap angle, a wrap whose path build failed is released instead of committed (deg). 0 disables the guard. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning|Quality",
		meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg", DisplayName = "Min Angle On Path Failure"))
	float FailedWrapMinAngleDeg = 120.0f;

	// How it differs from FailedWrapMinAngleDeg: that one aborts early, and only where the path build failed.
	// This is the last gate before commit and applies to *every* wrap — path completed, path failed, or
	// settle timed out alike. A path can complete normally and still commit a poor wrap, if the latch landed
	// near the tip and the path is therefore short; a settle-timeout commit can go through on a single
	// anchor. Opt in when a game rule should filter those out, and leave it at 0 for a design where a light
	// hook on the tip is fine.

	/** Below this wrap angle, no wrap commits at all (deg). 0 = off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning|Quality",
		meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg", DisplayName = "Min Commit Angle"))
	float CommitMinWrapAngleDeg = 0.0f;

	// How it differs from CommitMinWrapAngleDeg, which accumulates: an accumulated angle sums every bit of
	// rotation, so oscillating back and forth across a surface inflates it, and it can take several turns to
	// pass 360°. Coverage instead asks which directions around the axis the rope actually occupies — a purely
	// geometric enclosure measure from 0 to 360°, immune to oscillation. It is what tells you the target is
	// genuinely trapped, as a two-leg bola is, by checking there is no gap left to slip out through.
	// It is exact under a CaptureTravelPlane wrap, where the axis is pinned at capture; under
	// BoneCenteredGuidePlane it approximates, using the last rolling axis.
	// Around 300° locks a pair of legs; 0 allows a loose hook.

	/** Below this angular coverage around the wrap axis, no wrap commits (deg). 0 = off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning|Quality",
		meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg", DisplayName = "Min Commit Coverage"))
	float CommitMinWrapCoverageDeg = 0.0f;

	// NOTE: WrappingContactGraceTime was removed — it was never wired to anything. Restore the setting
	// alongside the logic if grace handling is ever implemented.

};

/**
 * Tuning for what happens *after* the wrap — hold, pull and release. It is a separate domain from wrap
 * detection (FRopeWrapConfig), and applies the same way whatever the resolve mode or latching model.
 * Consumed by URopeComponent's wrapping and movement constraints, and by the four Wrapped steps: hold, pull
 * sample, tether and pull application, then auto-release.
 */
USTRUCT(BlueprintType)
struct FRopeHoldConfig
{
	GENERATED_BODY()

	/**
	 * From the moment wrapping begins, keep the free span between the wielder's hand and the rope within its
	 * material rest length.
	 * With it on, the Wielder projects CharacterMovement's final movement — input, root motion and sliding
	 * included — onto a spherical constraint in the same PrePhysics frame, and an ordinary Pawn gets the same
	 * safety net immediately after its movement tick.
	 *
	 * This constraint is gameplay authority and owes nothing to SegmentTension or GPU readback, which is what
	 * stops a rope with TetherCompliance = 0 being stretched by kinematic pawn movement and only recovering a
	 * frame later. With TetherCompliance > 0 the elasticity is deliberate, so hard projection switches itself
	 * off. Custom movement without a Wielder should call URopeComponent::ConstrainWielderLocation before
	 * applying its move.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (DisplayName = "Enforce Rope Length"))
	bool bEnforceWielderLengthConstraint = true;

	/**
	 * The **target-side mirror** of the hard projection above. When the wrapped target is a CMC-driven
	 * character — a kinematic capsule — and the wielder's end has infinite mass, as an anchored or kinematic
	 * carrier such as a helicopter does, the target's capsule is swept into a sphere centred on the hand with
	 * the leg's rest length as its radius, in that same frame.
	 * It works only in that combination. When both ends can move, the λ pair already distributes the
	 * correction by inverse mass, and adding a projection would apply it twice.
	 *
	 * Without it, the only way to track a moving carrier is positional recovery, capped by TetherMaxBiasSpeed,
	 * so a carrier faster than that cap stretches the rope without limit. Switches itself off when
	 * TetherCompliance > 0, where the elasticity is deliberate.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (DisplayName = "Enforce Rope Length (Target)"))
	bool bEnforceTargetLengthConstraint = true;

	// The traction boundary is max(this value, MaxDistance × TautSlackRatio × hysteresis), and for a rope of
	// any real length the second term always wins — about 18 cm against 0.5 cm on a 600 cm rope. The designer
	// knob that moves the boundary is TautSensitivity; this is the numerical stability floor beneath it,
	// which is why it is not exposed to the Details panel.

	/** Numerical tolerance band for activating the length constraint (cm). It never adds rope length, so an
	 *  inextensible rope cannot grow through it. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Hold|Tuning",
		meta = (ClampMin = "0.0", Units = "cm"))
	float LengthConstraintActivationSlop = 0.5f;

	/**
	 * Auto-release (ERopeReleaseReason::Tension) once the authoritative constraint tension stays above this
	 * value for TensionReleaseTime. 0 disables it.
	 * Same units as GetConstraintTension and MaxTetherTension; unrelated to XPBD SegmentTension.
	 * Auto-release applies to FullSimulation and AssistedJudged only — a GuaranteedWrap rope is released
	 * explicitly.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release", meta = (ClampMin = "0.0", DisplayName = "Release At Tension"))
	float TensionReleaseForce = 0.0f;

	/** How long tension must stay over the threshold before releasing (s), so an impact spike does not let go.
	 *  Greyed out when tension release is off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release",
		meta = (ClampMin = "0.0", Units = "s", EditCondition = "TensionReleaseForce > 0.0", DisplayName = "Release Delay"))
	float TensionReleaseTime = 0.05f;

	// A λ impulse produces only the component along the rope axis, so when the direction swings sharply the
	// old perpendicular inertia survives and the target flies off. This ratio — 0 keeps it, 1 removes it
	// entirely — is how much of that residual is subtracted each frame. The rate is per-frame at 60fps and is
	// corrected by dt when applied, so it is frame-rate independent.
	// The only receivers are endpoints that go through ApplySimBody: a wielder configured as a physics actor
	// with a simulating root, and a simulating target in elastic mode with TetherCompliance > 0. An
	// inextensible simulating target (TetherCompliance = 0) is held by the exclusive Chaos physics constraint
	// instead and never sees this damping, and neither does a CMC character.

	/** How much sideways inertia is removed from the tether each frame [0..1], which stops a target being flung when the pull direction swings. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Tether Sideways Damping"))
	float TetherPerpDamping = 0.3f;

	/**
	 * Safety cap on the velocity the tether injects, clamped after the fact so the direction is preserved
	 * (ClampInjectedVelocity). 0 removes the clamp, which is not recommended. The cap on λ's positional
	 * recovery command is a separate knob, TetherMaxBiasSpeed.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "cm/s", DisplayName = "Tether Speed Limit"))
	float TetherMaxSpeed = 1500.0f;

	// This is SolveTetherLambda's MaxBiasSpeed. Once the gap closes and the separating speed is gone, this
	// term is the only momentum left — the constraint is one-directional, so nothing brakes the target after
	// it goes slack, and this value is the cap on that coasting speed. Reusing TetherMaxSpeed (1500) for it
	// used to accelerate a light target to 15 m/s within a frame or two, which then went slack and flung it
	// away; a value in the low hundreds is plenty to reel one in. Since the tether's job is to stop the gap
	// growing, and recovery is the only thing that ever takes an existing gap back, this cap is what decides
	// how much of a winch the tether is. 0 means no recovery at all — the tether seals the length and any
	// existing overshoot comes back only through reeling or the target walking closer.

	/** Cap on the speed at which the tether reels an overshoot back in (cm/s). 0 disables recovery. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "cm/s", DisplayName = "Tether Recovery Speed"))
	float TetherMaxBiasSpeed = 150.0f;

	// It is the one knob that sets where "how heavy must something be to drag me while I am braced" falls,
	// and it is usually left alone. The same value feeds the λ distribution (as an effective inverse mass)
	// and the pullability check behind climb-in.

	/** How much friction a grounded character can brace with, as a multiple of its mass. Higher holds firmer and pulls heavier things; lower is dragged more easily. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ClampMin = "1.0"))
	float GroundBraceFactor = 1.5f;

	// (The hysteresis on the pullability check is an internal constant — the mass knob is unified into
	//  GroundBraceFactor alone. See PullMassHysteresis in RopeComponentTraction.cpp.)

	/**
	 * Auto-release (ERopeReleaseReason::Distance) once the authoritative material-length violation exceeds
	 * this much (cm). 0, the default, disables it.
	 * With a tether it reads as "the tether holds on, but past this it loses" — and a strong enough tether
	 * never lets the overshoot accumulate that far. Without a tether it is a plain distance limit.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Release At Overstretch"))
	float DistanceReleaseSlack = 0.0f;

	// The pull direction is not the straight anchor-to-hand chord. It walks from the anchor toward the hand
	// and stops at the end node of the first straight leg it finds, so a rope bent around a wall or a corner
	// pulls along that first leg instead of through the obstacle, which is what a chord would do. While
	// walking, a segment that bends more than this angle away from the accumulated leg direction counts as a
	// corner and ends the walk; on a straight rope the walk reaches the hand (node 0) and the direction is
	// exactly the chord.
	// Larger ignores gentle bends and stays closer to a chord; smaller reacts to slight ones. The sag and
	// node jitter of a taut rope sit below this threshold and a wall edge sits above it, and SmoothTime
	// absorbs whatever jitter is left.

	/** Bend angle at which the pull direction treats the rope as turning a corner (deg). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "1.0", ClampMax = "179.0", Units = "deg", DisplayName = "Pull Corner Angle"))
	float PullBendThresholdDeg = 30.0f;

	/**
	 * Time constant of the EMA that smooths the pull direction (s), absorbing frame-to-frame jitter in the
	 * look-ahead direction and the GPU mirror's lag. Alpha is 1 - exp(-dt/this), so it is frame-rate
	 * independent. Larger is smoother and slower to respond; 0 is no smoothing. Seeded from the measured
	 * value when the wrap begins.
	 */
	float PullDirSmoothTime = 0.08f;

	/**
	 * Time constant of the EMA that smooths the pull aim node (s). The aim node the walk picks is an integer
	 * index, so on a shaking rope it jumps discretely each frame — the whole direction jumps with it and the
	 * tether overshoot goes discontinuous, cutting the traction — and a direction EMA cannot catch that.
	 * Smoothing the index as a float and interpolating between nodes keeps both direction and tether
	 * continuous. Alpha is 1 - exp(-dt/this), so it is frame-rate independent. Larger is smoother and slower;
	 * 0 is no smoothing. Seeded from the measured value when the wrap begins.
	 */
	float PullAimSmoothTime = 0.08f;

	/**
	 * Tension cap on the active pull — the default SetActivePull passes. Together with the target speed
	 * (ActivePullMaxLinearSpeed) it gives realistic mass dependence: a light target reaches the target speed
	 * at once within this tension, and one too heavy to pull there lags behind. So the target speed says how
	 * fast, and this says up to what weight.
	 * Force magnitude is a rope-physics concern, which is why it lives here; the Wielder's pull action uses
	 * this value. It is only ever authorized while Wrapped and taut — see URopeComponent::SetActivePull and
	 * the bActivePullRequiresTaut / ActivePullTautTension gate below.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ClampMin = "0.0", DisplayName = "Pull Strength"))
	float PullForce = 100000.0f;

	/**
	 * Whether the active pull applies only while the rope is taut. True, the default, applies force only on a
	 * taut frame, which is what pulling a slack rope should physically do — nothing. False ignores tension and
	 * authorizes the pull whenever the rope is Wrapped with a valid pull sample, for a presentation or a
	 * special-case mechanic. URopeComponent::IsPullTaut() reports the taut state either way, independently of
	 * this switch, so an animation pull window can still gate on it.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (DisplayName = "Pull Requires Taut"))
	bool bActivePullRequiresTaut = true;

	/**
	 * Optional load threshold on the taut check. 0, the default, lets a geometrically taut rope start an
	 * active pull; above 0, the authoritative GetConstraintTension() must also exceed this value before the
	 * rope counts as loaded. XPBD SegmentTension takes no part.
	 * Greyed out with bActivePullRequiresTaut off, where the taut check is not consulted at all.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning",
		meta = (ClampMin = "0.0", EditCondition = "bActivePullRequiresTaut", DisplayName = "Pull Load Threshold"))
	float ActivePullTautTension = 0.0f;

	// It scales the slack tolerance ratio and the maximum sag together from one number
	// (URopeComponent::GetEffectiveTautSlackRatio and GetEffectiveTautMaxSag, geometrically interpolated).
	// 0.5, the default, is 3% slack and 20 cm of sag; 0 gives 9% and 80 cm, 1 gives 1% and 5 cm. The
	// hysteresis and the release grace period behind the check are already-tuned internal constants
	// (RopeComponentTraction.cpp).

	/** How taut the whole chain must look before traction starts [0..1]. 0 pulls on a slacker rope; 1 demands it be visibly straight. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Taut Sensitivity"))
	float TautSensitivity = 0.5f;

	/**
	 * Floor that keeps the legacy particle-chord analytic fallback from firing on noise. It is the path taken
	 * with no live material geometry, and it takes no part in the taut or tension decisions of the ordinary
	 * hard-constraint or Chaos paths. 0, the default, is off; the hysteresis around it is an internal
	 * constant (RopeComponentTraction.cpp).
	 *
	 * Not exposed to the Details panel: reaching it needs no Chaos backend, no live constraint and no hard
	 * wielder attempt at once, so a Wielder setup with bEnforceWielderLengthConstraint on never gets there.
	 * It only matters when writing a custom mover that uses this fallback directly.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0"))
	float TautMinTension = 0.0f;

	/**
	 * Time constant (s) for recovering the overshoot C on an inextensible tether (TetherCompliance = 0).
	 * Each frame it commands an approach speed that closes C by β = 1 - exp(-dt/this), so smaller is firmer
	 * and lands immediately, larger tracks more softly, and 0 takes the whole gap in one frame (β = 1). It is
	 * frame-rate independent. The absolute cap on that command is TetherMaxBiasSpeed (SolveTetherLambda's
	 * MaxBiasSpeed), which keeps the frame right after commit from spiking on a large C and caps the coasting
	 * left in any residual slack.
	 * Elastic mode ignores this value and recovers the overshoot through TetherCompliance's restoring force.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "s"))
	float TetherSettleTime = 0.08f;

	/**
	 * Tension limit, and the overload threshold (kg·cm/s²; 0 is unlimited).
	 * In elastic mode (TetherCompliance > 0), λ ≤ this × dt is a real force cap.
	 * In inextensible mode (TetherCompliance = 0) a finite force cap and an exact length cannot both hold, so
	 * length wins and the full reaction is reported. The value is then only the debugger's overload baseline —
	 * actual release is TensionReleaseForce's job, or a game rule's — which is why it is greyed out for an
	 * inextensible rope, the default.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning",
		meta = (ClampMin = "0.0", EditCondition = "TetherCompliance > 0.0", DisplayName = "Tether Tension Limit"))
	float MaxTetherTension = 500000.0f;

	/**
	 * Material compliance α (s²/kg, the inverse of stiffness). 0, the default, is an inextensible rope.
	 * Above 0 it is an implicit spring of stiffness k = 1/α with generalized critical damping, giving
	 * deliberate elasticity — a bungee — that stays stable across frame rates. For example 0.0005 gives
	 * k = 2000 kg/s².
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", DisplayName = "Rope Elasticity"))
	float TetherCompliance = 0.0f;

	/**
	 * Target speed the active pull drives toward. The pull moves the target along the pull direction at this
	 * speed under the PullForce tension cap, clamping the impulse so the target speed is reached without
	 * overshoot — which is what removes the juddering and drifting a constant force (a = F/m) produces by
	 * blowing past the target within one frame. A heavy target cannot be pulled to this speed under the
	 * tension limit and lags behind, giving realistic mass dependence. 0 means no pull.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold",
		meta = (ClampMin = "0.0", Units = "cm/s", DisplayName = "Pull Speed"))
	float ActivePullMaxLinearSpeed = 300.0f;

	/**
	 * Spin cap (deg/s; 0 is unlimited) on a body under active pull. Applying force at the centre of mass adds
	 * no torque, so most sources of spin are already gone; this catches what ragdoll joint dynamics still
	 * produce.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "deg/s", DisplayName = "Pull Spin Limit"))
	float ActivePullMaxAngularSpeed = 720.0f;
};
