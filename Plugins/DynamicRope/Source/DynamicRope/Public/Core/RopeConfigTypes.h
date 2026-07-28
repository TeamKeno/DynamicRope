// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RopeConfigTypes.generated.h"

/**
 * Based on which standard to place the Wrapping axis (FRopeWrapConfig::WrappingAxisSource).
 * In both modes, the normal of the rope progress plane is used as the axis direction, but the axis origin and winding standard are different.
 * The back fallback (bone → parent → component base → bone local X) is common — FRopeWrappingPhase::ResolveWrappingAxis.
 */
UENUM(BlueprintType)
enum class ERopeWrappingAxisSource : uint8
{
	/**
	 * Bone center guide plane: The axis origin is based on the latch bone location, and the winding is based on the latch tangent.
	 * It is suitable for paths that require reinterpretation of the axis for each bone, such as single bone Wrapping in Assisted resolve.
	 */
	BoneCenteredGuidePlane = 0 UMETA(DisplayName = "Bone-Centered Guide Plane"),

	/**
	 * Capture progress plane: The normal of the swing plane where the rope flew is used as the axis direction, and the axis origin is the capture contact area and
	 * Calibrate to the center of the collider cluster. Winding is suitable for Composite Wrapping based on the velocity at the moment of capture.
	 * If the capture progress plane cannot be created, it falls back to the common bone/component axis fallback.
	 */
	CaptureTravelPlane = 1 UMETA(DisplayName = "Capture Travel Plane")
};

/** XPBD solver tuning (for designers). The saved value is the runtime applied value.*/
USTRUCT(BlueprintType)
struct FRopeSolverConfig
{
	GENERATED_BODY()

	/** Number of per-frame physics substeps (anti-tunneling; "small steps" are better than increasing iterations).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1", ClampMax = "16"))
	int32 Substeps = 12;

	/** Number of constraint iterations per substep.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1"))
	int32 Iterations = 4;

	/** Number of collision resolution passes per substep. 1=Once at the end of the substep (existing behavior, perf no regression). In sharp bends
	 *  When a single collision cannot overcome the inward pulling force of distance/bending, constraint iteration is performed by this number.
	 *  Divide and insert collisions in between → Defense up to sharper angles (>1 is stronger, but costs ↑). cap as Iterations.*/
	int32 CollisionPassesPerSubstep = 1;

	/** How many iterations will the contact constraint be solved (CPU fallback only — GPU kernel normally does once per collision pass).
	 *  1 = Every iteration (default, existing behavior). N = once every N iteration.
	 *
	 *  **No matter what value is given, it must be solved in the *last* iteration of each collision pass.** This is the core of the contract —
	 *  If you don't solve it at the end, there is no chance for distance/bending to push the node back into the surface, so the substep
	 *  Ends in penetration state. So, if N is given as Iterations or more, “exactly once per pass” = cadence like GPU.
	 *
	 *  Solving each iteration is stronger at sharp included angles (collision competes with distance/bending every time, so tension
	 *  is not pushed). Increasing N reduces the number of competitions, reducing the cost linearly, but reducing the penetration margin.
	 *  Query is the only handle that directly divides the CPU fallback cost from the expensive SDF collider.
	 *
	 *  Non-exposed (BP only): Since there is only one consumer, FRopeXPBDSolver, **only works in CPU fallback**. Single GPU
	 *  Because it is a runtime path (Cook/-nullrhi/Server/Excess size only CPU), it has no effect in normal play.*/
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1"))
	int32 ContactSolveInterval = 1;

	/** XPBD stretch compliance (reciprocal of stiffness). 0 = inextensible.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", DisplayName = "Stretch Softness"))
	float StretchCompliance = 0.0f;

	/** Strain limiting (maximum elongation clamp): After substep solve, walk along the chain at pinned/anchor pinned nodes.
	 *  Hard projects the segment length to ≤ this multiplier × SegmentLength (velocity neutral — prev is also moved). XPBD Distance
	 *  constraint is Gauss-Seidel, so if the number of iterations is small, when a long chain (several dozen nodes) hangs on an anchor pin, correction is required until the end.
	 *  Due to failure to propagate, runaway elongation (6 times+), huge tension, and tangential whip jitter occur in segments adjacent to the anchor. Sequential sweep is
	 *  propagates throughout the chain at once, confining this runaway into a cap (PBD long-range constraint standard solution).
	 *  1.5 = Allow up to 50% elongation (default). 1.0 = Fully unstretched (tightest). **0 or <1 = disabled** (strain limit off).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", DisplayName = "Max Stretch"))
	float MaxStretchRatio = 1.5f;

	/** XPBD bending compliance. The bigger it is, the more flaccid it is.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", DisplayName = "Bend Softness"))
	float BendCompliance = 0.02f;

	/** Angle-allowed bending: The steeper the bend, the more the stretching force is released (alleviating the angular bounce of the Free node at the corner/wrap boundary).
	 *  decision value r = (i↔i+2 distance)/(2*SegmentLength) = cos(turn angle/2): 1=straight line, the smaller the steeper bend.
	 *  If r ≤ BendReleaseRatio, the unfolding force is 0 (completely released), if r ≥ BendFullRatio, it is 100% (existing operation), and smoothstep in between.
	 *  Default 0.70 (≈turn angle 91°). If the corner is still sharp, raise it (allows for sharp bends), and if you set both values ​​to 0, it will always be straightened (turns off angle tolerance).*/
	float BendReleaseRatio = 0.70f;

	/** Angle-allowed bending: If r ≥ this value, the straightening force is 100% (gentle bending is straightened as before). Default 0.92 (≈turn angle 46°).
	 *  If the Free rope is too flabby, lower it, and it should always be above BendReleaseRatio (internally guaranteed by the solver).*/
	float BendFullRatio = 0.92f;

	/** Tangential direction friction to the collider [0..1] (Coulomb coefficient μ).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Friction = 0.5f;

	/** A multiplier that weakens friction toward the Free end (pinned point=1, end=this value). The end node has the lowest tension.
	 *  It is easily caught by friction, so lower the grip at the end to let it go. If 1.0, there is no taper (uniform friction).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Tip Grip Falloff"))
	float TipFrictionScale = 1.0f;

	/** collision query radius(cm) — The solver lifts the node this far from the contact surface. **Default 0 = auto: render tube
	 *  Use Radius as is** (Automatic matching of 3 types of radius — 2026-07-13 surface audit B-2; If you enter a specified value, that value).
	 *  Analysis is performed once at the component boundary (GetEffectiveCollisionRadius) — The solver/GPU step receives only the interpreted value.
	 *  NOTE: Auto interpretation is disabled for consumers (unit tests/custom solver calls) who use this structure directly without a component.
	 *  None — If it is 0, it operates as radius 0, so be sure to enter a specific value.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Collision Radius (0=Auto)"))
	float CollisionRadius = 0.0f;

	// NOTE: bUseWorldGDF moved directly under URopeComponent ("Rope|Collision" category)
	// (2026-07-13 surface audit CL-4 — collision domain agglomeration: one digit with bIncludeOwnerColliders).

	/** Collision sweep sample interval (cm) — The path the node moved in one substep is queried at this interval.
	 *  The lower it is, the stronger it is against tunneling and the higher the query cost. It moves in pairs with MaxSweepSamples below.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning",
		meta = (ClampMin = "0.1", Units = "cm"))
	float SweepStep = 2.0f;

	/** Number of sweep samples per section cap (cost limit). In very fast nodes, the gap is widened by being pressed against this cap,
	 *  If lowering the SweepStep has no effect, you must also increase this value.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning",
		meta = (ClampMin = "1", ClampMax = "64"))
	int32 MaxSweepSamples = 16;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	/** Air resistance. Units are **per-frame velocity reduction rate based on 60fps** (independent of number of substeps — Integrate compensates exponentially).
	 *  If 0.02, residual rate per second is 0.98^60 ≈ 0.30, effective drag k ≈ 1.2/s → Free fall terminal velocity ≈ g/k ≈ 8m/s.
	 *  As you raise it, the longitudinal velocity decreases and momentum dies quickly, making it look like a “light ribbon” — lower it if you need more weight.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Motion Damping"))
	float Damping = 0.02f;

	//~ Scaling (Sleep/LOD) — Reduces idle/far costs when multiple ropes exist ------------------

	/**
	 * Sleep: In Free/Wrapped phase, all node velocities are below SleepVelocityThreshold during SleepDelay.
	 * If held, solve is skipped — Free has no dispatch itself, and Wrapped has bone following(Hold)·traction·automatic release.
	 * The logic continues to run and only the Free span solve is paused (GPU is override-only dispatch). Pin moving/Wrapping/moving
	 * Wakes up from collider proximity/Wrapped bone movement (node ​​drift)/Active Pull Loaded/phase transition.
	 * Other phases (Flight/Contacting/Wrapping/Releasing) are always active.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling")
	bool bAllowSleep = true;

	/** Slip entry check velocity (cm/s) — Maximum node displacement / dt between frames must be less than this value.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.1", DisplayName = "Sleep Speed Threshold"))
	float SleepVelocityThreshold = 3.0f;

	/** Time (in seconds) that low speed must be maintained before entering slip.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "s"))
	float SleepDelay = 0.5f;

	/**
	 * Distance LOD: If the distance to the player camera exceeds LODStartDistance, reduce constraint iteration.
	 * and decreases linearly from LODEndDistance to LODMinIterationScale (convergence error is not visible from a distance).
	 * If there is no camera (Dedi server), always full iteration.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling")
	bool bEnableDistanceLOD = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "LOD Start"))
	float LODStartDistance = 3000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "LOD End"))
	float LODEndDistance = 8000.0f;

	/** Iteration multiplier at the farthest distance (1=no reduction).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.05", ClampMax = "1.0", DisplayName = "LOD Min Iterations"))
	float LODMinIterationScale = 0.25f;
};

/**
 * Contact Decision Tuning: When is a draped rope considered “Wrapped” by a limb?
 * Parameter Hierarchy (2026-07-13 Meeting Decision F): Basic Display field = T2 (Balance), AdvancedDisplay field = T3
 * (Advanced — ②AssistedJudged × BareWrap check Mostly dedicated to infrastructure. ③Guaranteed checks/path builds
 * is not used, so T3 is all meaningless). For details, see Docs/PoC/02_WrapResolveModes.md §5~6.
 */
USTRUCT(BlueprintType)
struct FRopeWrapConfig
{
	GENERATED_BODY()

	/**
	 * Whether the SurfaceVectorField path point will be passed to the graph candidate bone without being pinned to one latch bone.
	 * If false, candidate graph depth/cost becomes 0 and only the current bone is evaluated, returning close to the existing single bone operation.
	 *
	 * Non-exposed (BP only): This is a fallback switch that deteriorates into a single bone operation when turned off, so there is no reason to turn it off other than debugging.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap")
	bool bEnableMultiBoneWrapping = true;

	/**
	 * contact *query* radius(cm) — As the name suggests, it does not belong to a specific step, but **detection(Flight/Contacting) and
	 * is the surface query probe radius** shared by the establishment (path build projection/snap cap/DecideWrap).
	 * **Default 0 = auto: render tube Radius × 1.5** (Automatic matching of 3 types of radius; If you enter a specified value, it will be the value). The interpretation is
	 * At the component boundary (GetEffectiveContactQueryRadius) — the consumer receives the interpreted value. Note: component
	 * — you must enter an explicit value.
	 *
	 * Non-exposed (BP only): Auto follows the render tube radius, so the preset fits together just by setting the radius.
	 * The moment you enter the specified value, the automatic matching is broken.*/
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", Units = "cm"))
	float ContactQueryRadius = 0.0f;

	//~ Capture check(detection) --------------------------------------------------------
	// The threshold for Flight to be considered “caught,” and the contact detection sweep beyond which that check passes. Threshold three is
	// ①FullSimulation/②AssistedJudged check path only, ③GuaranteedWrap is confirmed by GuidedThrow.
	// It is only established as an anchor, so it is not viewed (sweep is Flight detection itself, so it has nothing to do with the mode).

	/** Minimum number of rope nodes that must touch a bone to be considered a catch rather than a grazing contact.
	 *
	 *  Non-exposure (BP only): The default of 1 is as lenient as possible, and the goal of "not catching stale things" is the angle at the time of commit.
	 *  gateway(FailedWrapMinAngleDeg/CommitMin*) achieves more accuracy. The only reason left to post is
	 *  The purpose is to prevent the useless operation of trying to wrap from even starting (to prevent wasted path builds/visual false starts).*/
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "1"))
	int32 MinLatchNodes = 1;

	/** Contact must last this long (in seconds) on the same bone before confirming the wrap.
	 *
	 *  Non-exposure (BP only): Default 0.016 = 60fps 1 frame, so there is virtually no dwell. Such as MinLatchNodes
	 *  This is the second levera redundancy for the question ("How sure must it be to reach to catch it?").*/
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", Units = "s"))
	float WrapDecisionTime = 0.016f;

	/** Flight prediction lookahead (frame displacement multiple) to avoid missing thin limb/SDF candidates. If 0, prediction is off.
	 *
	 *  Non-exposure (BP only): ContactSweepStep is a more direct solution to the same symptom of "fast throwing passes target".
	 *  It's a lever, and its comments guide the response sequence.*/
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", ClampMax = "4.0"))
	float PredictiveContactFrames = 1.0f;

	/**
	 * Sample interval (cm) of contact detection sweep. The path taken by the node in one frame is interrogated at this interval to determine the deepest contact.
	 * Look for — **value to prevent tunneling**, so it should be less than the thickness of the thinner side (forearm/handrail) of the object you are trying to grab.
	 * If a fast throw passes through the target and misses the capture, lower this value. solver collision
	 * Same idea as FRopeSolverConfig::SweepStep, but detection only runs on Flight and budget is set aside.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.1", Units = "cm", DisplayName = "Sweep Step"))
	float ContactSweepStep = 2.0f;

	/** Sample number cap (cost limit) of the above detection sweep. In very fast nodes, the gap is widened by being pressed against this cap,
	 *  If lowering ContactSweepStep has no effect, you must also increase this value.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "1", ClampMax = "64", DisplayName = "Max Sweep Samples"))
	int32 ContactMaxSweepSamples = 16;

	/**
	 * Maximum number of wrap seeds (contact targets) that can be adopted in one capture. 1 (default) = Traditional single seed behavior.
	 * If it is 2 or more, in addition to the dominant target, dwell on a *different* (mesh, bone) on the tail side than the dominant latch.
	 * The filled contacts are wound together as auxiliary seeds (e.g. both legs – one leg is wound and the contact node on the other leg is also wound together)
	 * pinned to that bone). A Wrapping path (spiral) is created only for the dominant seed, and the secondary seed uses the contact node as its own.
	 * It is a method of holding on to the bone — it does not create a path that goes around the auxiliary object.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "1", ClampMax = "8"))
	int32 MaxWrapSeeds = 1;

	/**
	 * Wrapping axis guidance source. BoneCenteredGuidePlane (default) uses latch bone for Assisted single bone Wrapping.
	 * is used as the axis origin. CaptureTravelPlane supports Composite Wrapping around the contact area/cluster center axis.
	 *
	 * Maintain exposure because the value must be different for each preset (bola = CaptureTravelPlane, single bone capture =
	 * BoneCenteredGuidePlane). It is better for users to set presets rather than individually.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (DisplayName = "Axis Source"))
	ERopeWrappingAxisSource WrappingAxisSource = ERopeWrappingAxisSource::BoneCenteredGuidePlane;

	/**
	 * Maximum distance (cm) that a SurfaceVectorField path can cross in a tangent straight line (chord) through empty space without a surface.
	 * 0 (default) = Off — If projection is interrupted, the path build fails as before.
	 * is turned on, two things change (4 stages of wrap based on progress direction, premise of wrap where the object is split in two like a leg of lamb):
	 *  ① If the projection attempts to pull to a surface that is more than one segment away from the prediction point, it refuses to snap and returns to the chord.
	 *     goes (if turned off, QueryRadius retains my lenient snap — default behavior unchanged).
	 *  ② Path points in the chord section do not create anchors — after commit, those nodes remain as Free ropes and the solver
	 *     Holds a suspended/straight form, and tension is applied when the object is spread (actual physics of binding).
	 * If it fails to re-enter the surface even after passing this distance, it falls into the same failure processing as before.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Max Gap Bridge"))
	float WrappingMaxGapBridgeDistance = 0.0f;

	/**
	 * Rolling amount cap (degrees): Cumulative Wrapping angle of SurfaceVectorField path build (rolling axis integral —
	 * FRopeWrappingState::PathAccumulatedAngleRad) reaches this value, the path is closed early with *success*.
	 * 0 (default) = Unlimited — Proceed until all remaining rope is wound (existing behavior).
	 * A long rope spirally wraps around the object several times (actual measurement 3000~4400°), and bone conversion reseed is used for the neck/head.
	 * Prevents “octopus wrap” from spreading to other areas. The remaining rope outside the path finished at the cap is during Wrapping.
	 * It is frozen and then stretches to Free span after commit (front motion does not drag nodes outside the path).
	 * If you do a double leg bola, 400~540° (one turn + margin) is natural.
	 *
	 * **[Plot] Does not apply to Composite AnalyticHelix** — and it's up to you to decide which strategy to use.
	 * , but the *target geometry* is determined at runtime: if two or more bones are bound to the same pose-space column,
	 * Composite (bPathUsesPoseSpaceIsland in path build). One forearm is sequential, so this cap is applied,
	 * The legs are composite, so they don't get stuck. In other words, multiple bones are most likely to produce the "octopus wrap" that this value is intended to prevent.
	 * target is actually invalid — in that case overwrapping is CommitMinWrapAngleDeg/CommitMinWrapCoverageDeg
	 * Must be filtered through the gateway.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.0", Units = "deg", DisplayName = "Max Wrap Angle"))
	float WrappingMaxWrapAngleDeg = 0.0f;

	/** Wrapping phase must keep the same accumulated latch span stable this long before committing. */
	float WrappingStableTime = 0.10f;

	/**
	 * **Minimum length** (seconds) of the Wrapping phase — This is not the time taken for Wrapping. The roles are different in the two paths.
	 *
	 * ① Normal (angle mapping) path — Wrapping velocity is determined by WrappingAngularSpeedDegPerSec, and the time required is determined by WrappingAngularSpeedDegPerSec.
	 *    This is the result. This value only works as a floor: even if the front reaches the target early, it starts Wrapping from the beginning.
	 *    Do not commit before the time has elapsed. During that waiting period, the rope is **stationary in the already completed wrap position.
	 *    Yes** (Wrapping is logic-driven, so the solver does not run). In other words, what this value determines is “Wrapping completed” and
	 *    This is the interval between “tension start (Wrapped entry)” — to eliminate moxibustion, lower it below the actual Wrapping time.
	 * ② DistanceFallback (angle mapping not possible) path — reversed. This value determines the actual time taken.
	 *    Calculate front velocity as FullDistance / (this value + tail delay). Each velocity setting is not used here.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap",
		meta = (ClampMin = "0.01", Units = "s", DisplayName = "Min Wrap Duration"))
	float WrappingMotionDuration = 0.20f;

	/**
	 * Standard velocity (deg/s) before applying easing of Wrapping animation.
	 * For both Single/Composite, FrontWrapAngleRad is performed with this value. 1100deg/s is 3 steps ago
	 * This is a common default value tailored to the Single actual measurement start policy (baseSpeed approximately 203cm/s / 10.57cm/rad).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap",
		meta = (ClampMin = "1.0", ClampMax = "7200.0", Units = "deg/s", DisplayName = "Wrap Speed"))
	float WrappingAngularSpeedDegPerSec = 1100.0f;

	/** After the angle mapping front reaches the angle+distance target, there is a short path to be secured before the Wrapped commit.
	 *  Stabilization time. Prevents frame popping that may occur during the last kinematic node/anchor transition.
	 *
	 *  Non-exposed (BP only): This is a policy constant determined by actual measurements, so there is no basis for choosing a different value. The velocity/length of Wrapping is
	 *  WrappingAngularSpeedDegPerSec and WrappingMotionDuration are determined.*/
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.0", ClampMax = "1.0", Units = "s"))
	float WrappingPostFrontSettleTime = 0.08f;

	/** Delay time added to each segment to ensure that the tail node settles on the surface path.
	 *  Applies only to the DistanceFallback path where angle mapping cannot be used due to the commit time limit from step 4.*/
	float WrappingTailDelayPerSegment = 0.024f;

	/** **Multiple** of the surface path integration step budget to be performed in one frame during Wrapping (not the number of steps itself).
	 *  The actual budget is a reference value proportional to the rope size multiplied by this value.
	 *  (FRopeWrappingPhase::ComputePathStepBudget — base 8, so 8 = 1x).
	 *  If you increase it, the Wrapping path will be completed faster, but the instantaneous cost will increase.
	 *
	 *  Not exposed (BP only): The main reason for changing, "path completion on large ropes is slow", is already scaled proportional to size.
	 *  Process.*/
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "1", ClampMax = "128"))
	int32 WrappingPathBuildStepsPerFrame = 8;

	/**
	 * Axis distance advanced per circumference distance for Sequential SurfaceVectorField wrapping.
	 * Composite Analytic Helix does not use this value but automatically calculates it from the tail slope at the moment of contact.
	 *
	 * Non-exposed (BP only): The target geometry determines at runtime which strategy to use (see below).
	 * Refer to the pose-space island description in the WrappingMaxWrapAngleDeg annotation), and even if you change the value, the response depends on the target.
	 * You cannot learn cause and effect.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "-2.0", ClampMax = "2.0"))
	float WrappingHelixPitchScale = 0.25f;

	// NOTE: The multibone projection scoring details below (12 types of depth/cost/weight/bonus/hysteresis) are based on actual tuning.
	// Internalized as developer constant (2026-07-13 surface audit B-2 — UPROPERTY removed, adjusted only in code).
	// The on/off switch is the bEnableMultiBoneWrapping above. The meaning of each value is annotated by field.

	/**
	 * How many edges in the current bone are considered candidates.
	 * For now, only the skeleton parent/child edges are used. Even if you add a designer-specified transition later,
	 * Since it passes the same depth limit, it is a primary safety device that prevents bridges that are too far away from opening at once.
	 */
	int32 MaxBoneTransitionDepth = 3;

	/**
	 * candidate graph cumulative cost cap.
	 * Even if the depth is the same, the cost may vary if the penalty for each edge is different. For now, only parent/child edge costs
	 * It accumulates, but later plays the role of cutting out candidates before projection when mixing designer edges / edges close to prohibition.
	 */
	float MaxBoneTransitionCost = 5.0f;

	/**
	 * Cost of passing one automatic parent/child edge.
	 * The larger the value, the greater the graph cost, making it easier to maintain the same bone, and lowering it makes the parent/child chain more active.
	 */
	float AutoParentChildTransitionPenalty = 1.0f;

	/** projection distance score weight. The farther from the predicted location to the surface, the more disadvantageous it is.*/
	float ProjectionDistanceWeight = 0.35f;

	/** Distance weight between the actual rope node location and the projection surface point. Prefer the bone on the side where the rope is actually located.*/
	float RopeNodeDistanceWeight = 0.25f;

	/** Weight of the degree to which the old tangent and the new tangent are bent. The larger the value, the smoother progress is preferred.*/
	float TangentContinuityWeight = 8.0f;

	/** Weight of the degree to which the old normal and the new normal are bent. The larger the value, the more surface normal continuity is preferred.*/
	float NormalContinuityWeight = 5.0f;

	/** graph cost weight. A candidate who crosses the parent/child line more often is at a disadvantage.*/
	float BoneTransitionPenaltyWeight = 1.0f;

	/** Current bone maintenance bonus. Reduces bone shaking near the tie point.*/
	float CurrentBoneBonus = 0.35f;

	/** The new bone must be better than the current bone by this score to switch. Transition hysteresis.*/
	float BoneTransitionHysteresis = 0.75f;

	/** Penalty added to candidates that return directly to the previous bone. Reduce the round trip from A->B->A.*/
	float ImmediateBoneReturnPenalty = 1.5f;

	/** After the last bone transition, more than this distance (cm) must be advanced before the next transition is allowed. If 0, disabled.*/
	float MinBoneTransitionPathDistance = 8.0f;

	/** Upper bound for physics-based wrapping settle before committing the best accumulated anchors. */
	float WrappingMaxSettleTime = 0.90f;

	/**
	 * Minimum wrap angle (degrees) of the wrap for which path creation failed. If the angle of winding to failure is less than this value, “Slightly
	 * Release the commit instead of the “sticky” commit. 0 = Guard off.
	 * Why it is angle-based (replaces the previous constexpr "at least 1 turn" criterion): One turn requires rope 2πr, so the target
	 * The bigger it is, the more the absolute length increases — radius 100cm The body is 628cm per turn, which is equivalent to a basic rope (200cm).
	 * It was physically impossible and the large target wrap was structurally destroyed. Wrapped angle is independent of target size
	 * This is a measure of “hook quality” (120° = 1/3 turn hook).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning|Quality",
		meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg", DisplayName = "Min Angle On Path Failure"))
	float FailedWrapMinAngleDeg = 120.0f;

	/**
	 * Commit quality floor: wrap of *all* wraps (regardless of path completion/failure/settle timeout) that are committed as Wrapped.
	 * If the angle is less than this value, release instead of commit. 0 (default) = Off — Retain existing behavior.
	 * Difference from FailedWrapMinAngleDeg: that is an early abort for wrap only where "path creation failed", this is a commit.
	 * The final gateway just before. Even if the path is completed normally, if the latch is near the tip, the path is short (winding angle is small) and it sticks.
	 * commits can come out, and the settle timeout commit passes even with one anchor — such a poor wrap in the game
	 * Turn on by opt-in when you want to filter by rule (keep it at 0 if it is a design that allows slight tip overlapping).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning|Quality",
		meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg", DisplayName = "Min Commit Angle"))
	float CommitMinWrapAngleDeg = 0.0f;

	/**
	 * Shape-based binding gateway (degrees): Angular coverage around the Wrapping axis of the committed wrap path (aligning path point angles)
	 * 360° - maximum space) is less than this value, release instead of commit. 0 (default) = Off — Retain existing behavior.
	 * Difference from CommitMinWrapAngleDeg (accumulated angle): The accumulated angle is the sum of the amount of rotation, so vibration/reciprocation on the surface occurs.
	 * The value can be inflated and takes several turns to exceed 360°. Coverage refers to “what direction does the rope actually travel around the axis?”
	 * It is a pure geometric measure of "enclosure" (0-360°) and is immune to vibration — it determines whether the object is truly trapped (like a two-legged bola).
	 * check to see if there are any spaces to escape. In the CaptureTravelPlane Wrapping the axes are pinned at capture time.
	 * The meaning is correct (approximation based on the last axis in BoneCenteredGuidePlane's rolling axis).
	 * Around 300° for double leg locking, or 0 to allow for a loose hook.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning|Quality",
		meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg", DisplayName = "Min Commit Coverage"))
	float CommitMinWrapCoverageDeg = 0.0f;

	// NOTE: Previously [unwired] WrappingContactGraceTime has been removed (2026-07-13 surface audit B-2 — consumption code
	// is dead without any settings). When actually wiring the grace logic, restore the settings in the CL as well.

};

/**
 * Tuning *after* Wrapped (retention/pull/release) — domain of Design Note 01 (Post-Wrap model), and
 * Consistent with the boundary “Common regardless of arrival mode/latching model” (02 document §3). Previously, FRopeWrapConfig (establishment check)
 * The mixed one was separated (2026-07-13 surface audit B-1 — existing BP tuning non-transferred clean break).
 * Consumer: Wrapping/Movement constraint + Wrapped step 4 of URopeComponent
 * (Hold → Pull sample → Tether/Pull application → automatic release).
 */
USTRUCT(BlueprintType)
struct FRopeHoldConfig
{
	GENERATED_BODY()

	/**
	 * From the moment Wrapping begins, the Free span of the wielder's hand is forced within the material rest length.
	 * If true, RopeWielder will determine the final movement of the CharacterMovement (including input/root motion/slide) using the same PrePhysics
	 * is projected onto the frame as a spherical constraint, and the same safety net is applied to the general Pawn immediately after the movement tick.
	 *
	 * This constraint is a gameplay authority unrelated to SegmentTension/GPU readback. Therefore
	 * Prevents a rope with TetherCompliance=0 from being stretched first and then recovered later due to kinematic pawn movement.
	 * If TetherCompliance>0, intentional elasticity is allowed, so hard projection is automatically disabled.
	 * For custom movement without a Wielder, call URopeComponent::ConstrainWielderLocation before applying the movement.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (DisplayName = "Enforce Rope Length"))
	bool bEnforceWielderLengthConstraint = true;

	/**
	 * **Target-side mirror** of the above wielder hard projection: The wound target is a CMC driven character (kinematic capsule) and the wielder ends
	 * When there is infinite mass (Anchor — helicopter/kinematic carrier), the target capsule is placed within the sphere of the radius of the center of the hand and leg rest length.
	 * Project the sweep to the same frame. Only works in this combination — if both ends can move, the λ pair is already
	 * Since it is distributed in the inverse mass ratio, double correction occurs when projection intervenes.
	 *
	 * Without this, the only means of tracking carrier movement is location recovery (bias, TetherMaxBiasSpeed cap), so the carrier
	 * If it is faster than that, the rope will stretch infinitely. Automatically disabled if TetherCompliance>0 (intentional elasticity).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (DisplayName = "Enforce Rope Length (Target)"))
	bool bEnforceTargetLengthConstraint = true;

	/**
	 * Material-length constraint activation tolerance(cm). This is a numerical boundary band:
	 * it allows an outward attempt within this distance to produce a stable reaction, but it is
	 * never added to rope length and therefore cannot make an inextensible rope longer.
	 *
	 * Unexposed (BP only): traction start boundary is `max(this value, MaxDistance × TautSlackRatio × hysteresis)`.
	 * For ropes of actual length, the latter term always wins (≈18cm vs. 0.5cm for a 600cm rope). moving the border
	 * The designer knob is TautSensitivity, which is the numerical stability floor underneath it.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Hold|Tuning",
		meta = (ClampMin = "0.0", Units = "cm"))
	float LengthConstraintActivationSlop = 0.5f;

	/**
	 * The authoritative material-constraint tension (kg·cm/s²) during Wrapped sets this value during TensionReleaseTime.
	 * If it is continuously exceeded, it is automatically released (ERopeReleaseReason::Tension). 0 = disabled.
	 * It is the same unit as GetConstraintTension/MaxTetherTension and is not used together with XPBD SegmentTension.
	 * (Automatic release is only valid in arrival mode ①② — ③ Guaranteed is only for explicit release.)
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release", meta = (ClampMin = "0.0", DisplayName = "Release At Tension"))
	float TensionReleaseForce = 0.0f;

	/** Duration (in seconds) of tension release check. It prevents loosening with an instantaneous spike (impact frame).
	 *  If TensionReleaseForce = 0 (tension release off), there is no check and it is grayed out.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release",
		meta = (ClampMin = "0.0", Units = "s", EditCondition = "TensionReleaseForce > 0.0", DisplayName = "Release Delay"))
	float TensionReleaseTime = 0.05f;

	/**
	 * (limited to the simulation body of the analytic λ path) Since λ impulse creates only the rope axis component, if the direction changes suddenly, the old direction
	 * The inertia remains perpendicular and flies (“excessive inertia”). This ratio (0 = preservation, 1 = complete removal) determines how much of that residual inertia is.
	 * Suppress fling by subtracting across frames. The value is a per-frame rate based on 60fps and is corrected by dt when applied.
	 * (frame rate independent).
	 *
	 * The only recipient is an endpoint that passes ApplySimBody — **when the wielder is a physical actor configuration (sim root)**, and
	 * **Simulate target for elastic mode with TetherCompliance > 0**. The simulation target is non-stretchable (TetherCompliance = 0).
	 * Since the Chaos physical constraint is exclusive (bUseChaosBackend branch of UpdateConstraintTether), it does not take advantage of this damping,
	 * Does not apply to CMC characters.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Tether Sideways Damping"))
	float TetherPerpDamping = 0.3f;

	/**
	 * Tether velocity safety cap (cm/s) — Secondary clamp of injected resulting velocity (ClampInjectedVelocity —
	 * movement is preserved). 0 = No clamp (not recommended). The position retrieval command cap of λ is a separate knob (TetherMaxBiasSpeed —
	 * Previously, this value was reused and recovery was virtually cap-Free).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "cm/s", DisplayName = "Tether Speed Limit"))
	float TetherMaxSpeed = 1500.0f;

	/**
	 * λ position recall (bias) command velocity cap (cm/s) — SolveTetherLambda's MaxBiasSpeed. With gap offset (SepSpeed)
	 * Otherwise, only this term remains as the momentum (since it is a direction constraint, there is no braking after slack conversion — this value is slack coasting)
	 * cap of velocity). Previously, by reusing TetherMaxSpeed (1500), a light target was accelerated to 15m/s in one or two frames and then
	 * It flew away (“swish”) with a slack transition — this value is sufficient for the number of landings. Tether is "preventing the spread"
	 * Since this is the main function and the only thing that dynamically rewinds the excess is the recovery port, this cap determines the winchability of the tether.
	 * 0 = No retrieval (sealing only - excess is given to rewrapping/natural access only).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "cm/s", DisplayName = "Tether Recovery Speed"))
	float TetherMaxBiasSpeed = 150.0f;

	/**
	 * How many times the grounded character can withstand friction up to its mass (effective mass = Mass × this value). The bigger the tighter
	 * Holds It's good at pulling heavy objects, and the smaller it is, the easier it is to be dragged. “How heavy does an object have to be to drag me when I’m grounded?”
	 * The only tuning knob that determines the intersection point of the “start” — mostly unset by default.
	 * λ distribution (effective inverse mass) and climbable check (climb-in) are written as shared.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ClampMin = "1.0"))
	float GroundBraceFactor = 1.5f;

	// (the hysteresis of the draggable check is an unexposed internal constant — the mass knob is the GroundBraceFactor
	//  Unified into one. (See PullMassHysteresis in RopeComponent.cpp UpdateTargetPullable.)

	/**
	 * Distance release: The amount of authoritative material-length violation during Wrapped is this value (cm).
	 * is exceeded, it is automatically released (ERopeReleaseReason::Distance).
	 * 0 = disabled (default). When used with Tether
	 * becomes "the tether holds on, but if it exceeds this limit, it loses" — if the tether is strong enough, the excess won't build up.
	 * If not activated and used without a tether, it operates purely as a distance limit.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Release At Overstretch"))
	float DistanceReleaseSlack = 0.0f;

	/**
	 * Pull direction corner check threshold (degrees). The pulling direction is not from anchor → hand straight (chord), but from the anchor to the hand.
	 * Hold it so that it faces the end node of the “first straight bridge” you find while walking along → If the rope gets caught on a wall/corner and breaks,
	 * Stop just before and pull along the first leg (a straight chord penetrates the obstacle). While walking, the next segment so far is
	 * If it bends more than this angle in the cumulative leg direction, it sees it as a corner and stops — if it is straight, it walks to the hand (node 0) exactly.
	 * becomes a chord. If you hold it large (ignoring gentle bends), it is closer to a chord, and if you hold it small, it is sensitive to even slight bends.
	 * The sag/node jitter when tight is below this threshold, and the wall edge is above this threshold (residual jitter is absorbed by SmoothTime).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "1.0", ClampMax = "179.0", Units = "deg", DisplayName = "Pull Corner Angle"))
	float PullBendThresholdDeg = 30.0f;

	/**
	 * Pull direction time smoothing constant (seconds, EMA time constant). Inter-frame jitter in look-ahead direction + GPU mirror delay
	 * Noise is absorbed with an exponential moving average (alpha = 1-exp(-dt/this value), frame rate independent). The bigger the smoother it is
	 * Slow response, 0 means no smoothing (circle look-ahead). When wrap starts, it is initialized to the measured value.
	 */
	float PullDirSmoothTime = 0.08f;

	/**
	 * Pull aiming node time smoothing constant (seconds, EMA time constant). The integer aiming node (AimNode) chosen by walk is rope.
	 * If shaken, it jumps discretely every frame (the entire direction jumps + tether excess discontinuity = traction is cut off) with direction EMA
	 * I can't catch it. If you EMA the aiming index as a float and interpolate between nodes, the direction·tether becomes continuous (alpha=1-exp(-dt/
	 * value), frame rate independent). The larger it is, the smoother it is, but the response is slower, and if it is 0, there is no smoothing. Initializes with measured values ​​when wrap starts.
	 */
	float PullAimSmoothTime = 0.08f;

	/**
	 * **Maximum tension** (cap of traction force) for Active Pull (input hold) — Default value in SetActivePull. target velocity
	 * (ActivePullMaxLinearSpeed): Light targets will immediately (overshoot) target velocity within this tension.
	 * , and heavy objects that cannot be pulled to the target with this tension lag behind (depending on realistic mass — this value starts from "how many kg")
	 * determines "how hard it is"). The force size is the rope physics domain, so it lives here (Wielder PullAction uses this value).
	 * Wrapped + Only actually authorized when taut (URopeComponent::SetActivePull contract — tautology check below
	 * bActivePullRequiresTaut/ActivePullTautTension gate).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ClampMin = "0.0", DisplayName = "Pull Strength"))
	float PullForce = 100000.0f;

	/**
	 * Whether to apply only when the active pull is taut. true (default) = Force is applied only to the frame where the rope is taut (the stretched rope is
	 * No response when pulled — physically natural). false = Ignore tension: Always approved if Wrapped + valid pull sample
	 * (for presentation/special gameplay). The pull check itself can always be queried with URopeComponent::IsPullTaut() (this switch and
	 * Update regardless — for external check such as animation pull window).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (DisplayName = "Pull Requires Taut"))
	bool bActivePullRequiresTaut = true;

	/**
	 * Optional load threshold for taut check. 0 (default) = If pure geometry taut, an active Pull can be initiated.
	 * > 0, the authoritative GetConstraintTension() must exceed this value to be considered load-bearing.
	 * XPBD SegmentTension is not used.
	 * If you turn off bActivePullRequiresTaut, the pull check itself is not visible and is grayed out.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning",
		meta = (ClampMin = "0.0", EditCondition = "bActivePullRequiresTaut", DisplayName = "Pull Load Threshold"))
	float ActivePullTautTension = 0.0f;

	/**
	 * Sensitivity of entire chain taut check [0..1] — 0=loose (traction starts even with less taut), 1=strict (must be stretched more clearly)
	 * traction). Scale the slack allowable ratio and maximum allowable sag (cm) together to one value (URopeComponent's
	 * GetEffectiveTautSlackRatio / GetEffectiveTautMaxSag — geometric interpolation). 0.5 (default) = existing tuning
	 * (slack 3%, sag 20cm). 0 → slack 9%·sag 80cm, 1 → slack 1%·sag 5cm. check hysteresis·release grace period
	 * This is an internal constant that has already been tuned (RopeComponentTraction.cpp). A single handle that “only attracts when visually unfolded.”
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Taut Sensitivity"))
	float TautSensitivity = 0.5f;

	/**
	 * Contamination prevention threshold for legacy particle-chord analytic fallback. Without field/live material geometry
	 * path.
	 * Does not participate in the taut·tension of the normal Pawn hard-constraint/Chaos path. 0 (default) = off. check
	 * hysteresis is an internal constant (RopeComponentTraction.cpp).
	 *
	 * Non-exposed (BP only): Reach condition is "Not Chaos backend ∧ No live constraint ∧ hard wielder attempt
	 * None", so it will not run in Wielder configurations with bEnforceWielderLengthConstraint turned on.
	 * This is only meaningful if you are writing a custom mover that directly uses this fallback.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0"))
	float TautMinTension = 0.0f;

	/**
	 * Position recovery time constant (seconds) of excess C in non-stretch (TetherCompliance=0) constraint.
	 * Closing every frame C by β = 1−exp(−dt/this value)
	 * Commands the approach velocity — smaller means firmer (immediate landing), larger means softer tracking. 0 = entire amount of one frame (β=1).
	 * Frame rate independent. The absolute cap of the recovery command velocity is TetherMaxBiasSpeed ​​(SolveTetherLambda's
	 * MaxBiasSpeed ​​— Prevents spikes in frames with large C immediately after commit and caps residual slack coasting.
	 * Elastic mode does not use this value and recovers excess length using TetherCompliance's kC resilience.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "s"))
	float TetherSettleTime = 0.08f;

	/**
	 * tension limit/overload criterion (kg·cm/s², 0 = unlimited).
	 * In elastic mode with TetherCompliance>0, λ ≤ this value × dt is the actual force cap.
	 * In non-stretchable mode with TetherCompliance=0, finite force cap and exact length cannot be satisfied at the same time.
	 * Prioritize length and report full reaction. At this time, this value is only the debugger's overload baseline,
	 * The actual release/release is specified by TensionReleaseForce or a separate game rule — so non-stretchable (default)
	 * , it is grayed out.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning",
		meta = (ClampMin = "0.0", EditCondition = "TetherCompliance > 0.0", DisplayName = "Tether Tension Limit"))
	float MaxTetherTension = 500000.0f;

	/**
	 * Material compliance α (s²/kg = inverse stiffness). 0 (default) = non-stretchable rope.
	 * > 0, it is a common game with k=1/α implicit spring + generalized critical damping
	 * Creates intentional elasticity (bungee, etc.) that is not static even at the frame rate. Example: 0.0005 → k=2000 kg/s².
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", DisplayName = "Rope Elasticity"))
	float TetherCompliance = 0.0f;

	/**
	 * **traction target velocity**(cm/s) of active pull. Active Pull drives the object along the pulling direction at this velocity (tension cap PullForce
	 * ), clamp the impulse to the target arrival (mass
	 * a=F/m eliminates the problem of bouncing (dust/jaw) by overshooting the target in one frame. Heavy objects reach this velocity due to the tension limit.
	 * I can't pull it off and fall behind (depending on realistic mass). 0 = no traction. Applies only during traction.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold",
		meta = (ClampMin = "0.0", Units = "cm/s", DisplayName = "Pull Speed"))
	float ActivePullMaxLinearSpeed = 300.0f;

	/**
	 * Velocity cap (deg/s, 0 = unlimited) of the physical body being actively pulled. If force is applied to the center of gravity (AddForce), there is no torque.
	 * Although most of the causes of spin disappear, this cap suppresses the remaining spin created by ragdoll joint dynamics.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "deg/s", DisplayName = "Pull Spin Limit"))
	float ActivePullMaxAngularSpeed = 720.0f;
};
