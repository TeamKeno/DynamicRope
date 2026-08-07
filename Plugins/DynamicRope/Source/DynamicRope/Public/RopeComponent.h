// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The single UE integration point (Facade). Owns the sim state (FRopeSimState), the solver and the
// per-phase logic classes (WhipGuide / WrappingPhase / WrapController) by value, and runs the phase
// state machine (ERopePhase) that decides whether physics or logic governs the rope. The real work of
// each phase lives in the Logic/ classes; what stays here is the orchestration that drives transitions
// and broadcasts events. Attach it to an actor and call Throw().

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "Components/MeshComponent.h"
#include "Core/RopeLengthConstraintState.h"
#include "Core/RopeMovementConstraint.h"
#include "Core/RopeTypes.h"
#include "Core/RopeSimFrameIO.h"
#include "Core/RopePullDriveState.h"
// FRopeAimRayHitResult/FRopeAimRayThrowRequest + aiming logic/state.
#include "Logic/RopeAimTargeting.h"
// Sleep + distance LOD (solve throttling).
#include "Logic/RopeSolverThrottle.h"
// Deadband + smoothing for the tip mesh's segment-follow placement.
#include "Logic/RopeTipStabilizer.h"
#include "Solver/RopeXPBDSolver.h"
#include "Logic/RopeWrapController.h"
#include "Logic/RopeWhipGuide.h"
#include "Logic/RopeFlightContactDetector.h"
#include "Logic/RopeWrappingPhase.h"
#include "RopeComponent.generated.h"

class AActor;
class IRopeCollider;
class IRopeColliderProvider;
class UMaterialInterface;
class URopePreset;
class USkeletalMeshComponent;
class UStaticMesh;
class UStaticMeshComponent;
// Wrap targets are generalized to USceneComponent, so a rope can wrap a static prop as well as a bone.
class USceneComponent;
class FRegisterComponentContext;
struct FRopeDebugSnapshot;
// Debugger per-node Flight visualization item (Debug/RopeDebugSnapshot.h).
struct FRopeFlightNodeDebug;
// Debugger capture scope bits (Debug/RopeDebugSnapshot.h) — passed to the capture side so only the
// views that are switched on get collected.
enum class ERopeDebugCapture : uint8;

// The wrap event carries a struct payload rather than a bare bone name, so the latch set, the decision
// values and multi-bone wraps all arrive with it.
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnWrapped, const FRopeWrappedEventInfo&, Info);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnCaptured, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnReleased, FName, Bone, ERopeReleaseReason, Reason);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnPhaseChanged, ERopePhase, OldPhase, ERopePhase, NewPhase);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnPresetApplied, const URopePreset*, Preset);

// FRopeAimRayHitResult / FRopeAimRayThrowRequest live in Logic/RopeAimTargeting.h, re-exposed by the include above.

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeComponent : public UMeshComponent
{
	GENERATED_BODY()

	// The subsystem reaches straight into Sim / SolverConfig / Phase / WhipGuide and into SimFrame, the
	// per-frame contract bundle (see FRopeSimFrameIO) — both for the GPU step and the CPU SolveSimFrame path.
	friend class URopeSimSubsystem;
	// PrePhysics movement authority: the Wielder consumes the CPU constraint and primes the
	// physical target tether before Chaos without exposing the mutation API to general callers.
	friend class URopeWielderComponent;

#if WITH_DEV_AUTOMATION_TESTS
	// Test seam: forces SetPhase so RopePresetTests can exercise the ApplyPreset phase gate's reject path.
	friend struct FRopePresetTestSeam;
	// Test seam: checks the Contacting seed's anchor invariant and the synthetic latch fallback entry.
	friend struct FRopeWrappingFallbackTestSeam;
	// Test seam: checks virtual bridge lifetime and the shared GuidedThrow entry state.
	friend struct FRopeComponentRefactorTestSeam;
	// Test seam: reproduces the Wielder input/pull lifecycle and the self-wrap gate without a world.
	friend struct FRopeWielderComponentTestSeam;
	// Test seam: drives the open-space guided throw against a moving hand pin without a world.
	friend struct FRopeFreeGuidedThrowTestSeam;
#endif

public:
	URopeComponent();

	//~ Setup -------------------------------------------------------

	// The rope's top-level contract: this one value decides what a throw guarantees, whether aiming and
	// preview run, and whether the wrap goes through the detection gate. **The rope is the source of
	// truth** — the Wielder derives its aiming and throw behaviour from here, and a Blueprint or AI
	// caller works off this value alone with no Wielder at all. The per-mode contracts are documented on
	// the ERopeWrapResolveMode enumerators.

	/** How a throw resolves into a wrap — what the rope guarantees between leaving the hand and latching. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (DisplayName = "Wrap Mode"))
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	//~ Tip (attachment on the free end — spearhead, harpoon, weight) ------------------------
	// A display-only static mesh pinned to the rope's free end (GetNodeCount()-1). It carries no mass and
	// no collision by default: the solver never reads a tip mass. Latching works the same way in every
	// resolve mode, so a single bUseTipMesh switch governs the tip from BeginPlay to EndPlay. On EndPlay
	// only a tip we spawned is destroyed; a component adopted by tag is left alone.
	//
	// The one mode-specific piece is socket alignment (bUseTipMeshSockets): aligning the head with the
	// point it pierces only means something in GuaranteedWrap. IsTipSocketPlacementActive() is the single
	// source of truth for that condition, and HasTipSocket / ReadTipSocketLocal are the only socket readers.
	//
	// Fallback placement (socket alignment off, another mode, or no head socket): the mesh origin sits on
	// the rope's end node, oriented along the last segment. A tip buried in the target is left buried —
	// that is intended, not a missing correction. It keeps following through Wrapped as well, because the
	// end node is a bone-local anchor, so the mesh tracks the animation instead of freezing in place.
	// With a head socket but no tail socket, the rope meets the mesh at its origin.
	//
	// NOTE: the /** */ lines below become editor tooltips — keep them to one short line and put the
	// reasoning in blocks like this one.

	/** Attach a mesh to the rope's free end. Off: every Tip setting below is ignored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip")
	bool bUseTipMesh = false;

	/** Mesh to spawn on the tip. Ignored when a component is adopted by tag instead. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Mesh"))
	TObjectPtr<UStaticMesh> TipMesh = nullptr;

	/** Adopt an existing static mesh component on the owner with this tag instead of spawning one. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Component Tag"))
	FName TipMeshComponentTag = NAME_None;

	/** Tip offset, in the frame of the rope's end node. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Relative Transform"))
	FTransform TipMeshRelativeTransform = FTransform::Identity;

	// The tip follows the free end every frame, so leaving its collision on lets a display-only mesh bump
	// the character capsule or feed back into the rope's own collision queries — hence **off by default**.
	// Switching it on gives a spawned tip full QueryAndPhysics; a component adopted by tag gets its
	// authored setting back instead (captured when we adopted it). Applied when the tip is acquired
	// (EnsureTipMesh) and when the value is edited in the editor or PIE (PostEditChangeProperty).

	/** Give the tip mesh collision. Off by default so a display-only tip cannot disturb the rope or its owner. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Enable Collision"))
	bool bTipMeshCollision = false;

	// Off is an extension point: game code drives the tip itself through GetTipMeshComponent() while the
	// rope is Free, and the rope leaves its transform alone. Every other phase follows regardless.

	/** While Free, keep the tip on the rope's end each frame. Off: game code places the tip itself. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Sync While Free"))
	bool bSyncTipMeshOnFree = true;

	// XPBD leaves millimetre residual motion on a resting rope — the hand pin keeps moving with the
	// wielder's idle animation, so whole-rope sleep cannot be relied on — and the socket follow
	// multiplies angular noise by the tail-to-head lever arm, so a raw per-frame follow shivers on the
	// ground. The stabilizer (FRopeTipStabilizer) deadbands and smooths the follow sample; both fade
	// out with tip speed, so a flying or dragged tip passes through raw. The placement-contract
	// branches (Loaded, the wrapped embed, the aimed guided throw) bypass it entirely.
	// Deliberately not designer-exposed (and absent from URopePreset): one proven tuning covers every
	// rope, and per-preset knobs here only invite the deadband/smoothing mistuning artefacts (visible
	// rope-to-tip separation, snap pops). Code that drives the tip itself can still toggle the switch.

	/** Smooth and deadband the tip mesh while it follows the rope's end. Off: raw per-frame follow. */
	bool bStabilizeTipFollow = true;

	/** Hold the tip against position changes smaller than this while nearly at rest (cm). */
	float TipStabilizePositionDeadband = 5.0f;

	/** Hold the tip against direction changes smaller than this while nearly at rest (degrees). */
	float TipStabilizeAngleDeadband = 3.0f;

	/** Half-life of the converge toward a moved tip target (s). 0: snap once past the deadband. */
	float TipStabilizeHalfLife = 0.03f;

	/** Tip speed (cm/s) above which stabilization is fully bypassed. */
	float TipStabilizeFadeOutSpeed = 120.0f;

	/** End segments averaged to derive the tip direction. 1: the raw last segment. */
	int32 TipDirectionSampleCount = 3;

	// GetLoadedTipTransform() implements this; override that virtual to change the placement convention.

	/** Owner skeletal mesh socket the tip is held at while Loaded. Empty: this component's transform. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	FName LoadedHandSocket = NAME_None;

	// While Loaded this offset is composed onto the hand socket frame by MakeLoadedTipBaseWorld(), which
	// drives *both* the tip mesh and the pinned free-end node - route every Loaded placement through that
	// helper or the spear and the rope end drift apart. Distinct from TipMeshRelativeTransform, which
	// applies in every phase and cancels out along the socket/pierce paths, so tuning the grip here
	// cannot disturb the embed alignment.

	/** Tip offset while Loaded, in the hand socket's frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh", DisplayName = "Loaded Relative Transform"))
	FTransform LoadedTipRelativeTransform = FTransform::Identity;

	// On: the mesh is flipped so its tail meets the rope end and its head sits at the pierce point, then
	// frozen in bone-local space so it follows the target's animation. Off: sockets are not read at all
	// and the fallback placement above applies.

	/** Place the tip precisely by its Head/Tail sockets. Off: the mesh origin sits on the rope's end node. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Use Sockets"))
	bool bUseTipMeshSockets = false;

	/** Head socket — the tip's point, embedded at the aim hit. Empty: socket placement is disabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Tip Socket"))
	FName TipSocketName = NAME_None;

	/** Tail socket — where the rope's end attaches. Empty: the rope attaches at the mesh origin. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Rope Socket"))
	FName TipRopeSocketName = NAME_None;

	// The three values below are read once by InitRope, so writing them at runtime does nothing until the
	// rope is reinitialized — BlueprintReadOnly keeps callers out of that trap. To change length while
	// playing, use SetRopeLength / SetReelRate.

	// ClampMax 512 is the GPU solver's node cap (FRopeGPUSolver::MaxNodes, one thread group). Above it the
	// rope silently drops to the CPU solve and CPU tube, which is a performance cliff with no authoring
	// signal, so the editor blocks it outright; the Blueprint and C++ paths are hard-clamped by InitRope.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "2", ClampMax = "512", DisplayName = "Node Count"))
	int32 NumParticles = 64;

	/** Initial, and maximum, rope length (cm). The live length is GetCurrentRopeLength / SetRopeLength. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 600.0f;

	/** Shortest length reel-in can reach (cm). RopeLength is the upper bound. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "10.0", Units = "cm"))
	float MinRopeLength = 100.0f;

	/** Default reel speed (cm/s) for reel-in and reel-out input. Length is a rope-domain concern, so it lives
	 *  here; the Wielder's reel action passes this value to SetReelRate. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "0.0", Units = "cm/s"))
	float ReelSpeed = 300.0f;

	// Whether the rope tube is visible while Loaded. The default OnEnterLoaded() implementation is what
	// reads it, so a subclass that overrides that hook with its own presentation ignores this value.
	// Off shows only the tip held in the hand socket; on also draws the rope between hand and tip, which
	// keeps solving and therefore sags.
	// Visibility is applied on the edge into Loaded, so assigning this while already Loaded would not take
	// effect — BlueprintReadOnly plus SetShowRopeWhenLoaded / ToggleShowRopeWhenLoaded, same as RopeMaterial.

	/** Show the rope tube while Loaded. GuaranteedWrap only. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope",
		meta = (EditCondition = "ResolveMode == ERopeWrapResolveMode::GuaranteedWrap"))
	bool bShowRopeWhenLoaded = false;

	// The config structs below use ShowOnlyInnerProperties, so the Details panel lists their fields
	// directly under the category header (Rope|Solver, Rope|Throw, …) instead of behind a second
	// expander for the struct name. Blueprint and serialization are unaffected — each struct is still one
	// BlueprintReadWrite variable — and the per-field subcategories (Rope|Solver|Scaling and so on) still apply.

	/** XPBD solver tuning (Free and Flight physics). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ShowOnlyInnerProperties))
	FRopeSolverConfig SolverConfig;

	/** Throw and launch parameters (entry into Flight). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ShowOnlyInnerProperties))
	FRopeThrowParams ThrowParams;

	// GuaranteedWrap resolves off a preview, so it runs no wrap detection and builds no path — WrapConfig
	// is greyed out in that mode. Putting the EditCondition on the struct *member* (where ResolveMode is
	// visible) propagates edit-const to the inlined children, so they grey out together. The values are
	// preserved and only editing is blocked (EditConditionHides defaults to false, so nothing is hidden).

	/** Physics-to-logic handoff — capture thresholds and wrap establishment (path build, decision, commit). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap",
		meta = (ShowOnlyInnerProperties, EditCondition = "ResolveMode != ERopeWrapResolveMode::GuaranteedWrap"))
	FRopeWrapConfig WrapConfig;

	/** Tuning for what happens *after* the wrap: hold, pull and release. A separate domain from detection
	 *  (WrapConfig), and shared by every resolve mode and latching model. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ShowOnlyInnerProperties))
	FRopeHoldConfig HoldConfig;

	//~ Collision -----------------------------------------------------------------
	// The collision switches live together here. The radii themselves are SolverConfig.CollisionRadius and
	// WrapConfig.ContactQueryRadius, where 0 (the default) means auto — the GetEffective* helpers below
	// derive them from the render Radius so all three radii stay in step.

	/**
	 * By default the rope collides with every collider provider in the world except its owner's, so a rope
	 * in flight does not tangle in the thrower's own limbs. Cross-actor wrap still works, because another
	 * actor's provider is part of "every provider".
	 *
	 * The exclusion works at two granularities:
	 *  - A skeletal or wrap-target provider is skipped whole (the owner's provider never contributes).
	 *  - The static world provider (URopeStaticBodyProvider) is filtered per body — only shapes whose
	 *    source actor is the owner drop out, which is what keeps the tether proxy, the tip mesh and a held
	 *    weapon from shoving the rope around. Floors, pillars and other actors' geometry stay.
	 *
	 * **Turn this on when the rope is mounted on a prop** — a URopeComponent attached to a pillar, crane or
	 * anchor actor. Leaving it off excludes the mount's own collision too, and the rope falls through it.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision",
		meta = (ToolTip = "Off (default) excludes the owner's own colliders, down to individual owner-owned shapes in the static world provider such as the tether proxy, tip or held weapon. Turn it on when the rope is mounted on a prop actor, or it falls through its own mount.", DisplayName = "Collide With Owner"))
	bool bIncludeOwnerColliders = false;

	/**
	 * Push the rope off static world geometry — walls and floors — using the engine's Global Distance
	 * Field, evaluated on the GPU inside the solve dispatch. The project needs Generate Mesh Distance
	 * Fields; without a valid field this is silently a no-op. **On by default**, since wall and floor
	 * penetration is the more expensive failure; switch it off to avoid on-demand GDF builds.
	 * It reports no bone and no surface velocity, so it complements per-bone SDF colliders for broad
	 * static geometry rather than replacing them. Push radius and friction come from CollisionRadius,
	 * Friction and TipFrictionScale.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision", meta = (DisplayName = "Use World Distance Field"))
	bool bUseWorldGDF = true;


	/** Resolved solver collision radius: SolverConfig.CollisionRadius, or the render Radius when it is 0. */
	float GetEffectiveCollisionRadius() const
	{
		return SolverConfig.CollisionRadius > 0.0f ? SolverConfig.CollisionRadius : Radius;
	}

	/** Maximum stretch ratio for Flight, Wrapping and commit frames in FullSimulation and AssistedJudged.
	 *  A rigid Wrapped hold (TetherCompliance = 0) also stays at 1.0; only a compliant hold uses the setting. */
	float GetEffectiveMaxStretchRatio() const;

	/** Resolved contact query radius: WrapConfig.ContactQueryRadius, or the render Radius × 1.5 when it is 0. */
	float GetEffectiveContactQueryRadius() const
	{
		return WrapConfig.ContactQueryRadius > 0.0f ? WrapConfig.ContactQueryRadius : Radius * 1.5f;
	}

	/** Resolved taut slack tolerance, as a ratio: HoldConfig.TautSensitivity interpolated geometrically
	 *  from 0.09 (lax) through 0.03 to 0.01 (strict). Consumed by the chord gate in RopeComponentTraction.cpp. */
	float GetEffectiveTautSlackRatio() const
	{
		return 0.09f * FMath::Pow(0.01f / 0.09f, FMath::Clamp(HoldConfig.TautSensitivity, 0.0f, 1.0f));
	}

	/** Resolved maximum sag a taut rope may carry (cm): HoldConfig.TautSensitivity interpolated
	 *  geometrically from 80 (lax) through 20 to 5 (strict). Consumed by the sag gate in RopeComponentTraction.cpp. */
	float GetEffectiveTautMaxSag() const
	{
		return 80.0f * FMath::Pow(5.0f / 80.0f, FMath::Clamp(HoldConfig.TautSensitivity, 0.0f, 1.0f));
	}

	//~ Whip (throw swing) --------------------------------------------------------
	/** Tuning for the whip swing at the start of a throw. The runtime state belongs to WhipGuide, which
	 *  receives a snapshot built by MakeWhipGuideConfig(). */
	// GuaranteedWrap throws a guided arc rather than a whip Flight, so whip tuning means nothing there and
	// is greyed out in that mode (same struct-member EditCondition trick as WrapConfig above).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip",
		meta = (ShowOnlyInnerProperties, EditCondition = "ResolveMode != ERopeWrapResolveMode::GuaranteedWrap"))
	FRopeWhipConfig WhipConfig;

	/** Time elapsed in the current whip swing (s). 0 when no swing is running. */
	UFUNCTION(BlueprintPure, Category = "Rope|Whip")
	float GetWhipElapsed() const { return WhipGuide.GetElapsed(); }

	//~ Render --------------------------------------------------------------------
	// The render values below (Radius, NumSides, TubeSmoothing*) are read once when the scene proxy is
	// built, so a runtime write does nothing until the proxy is rebuilt — hence BlueprintReadOnly. Editor
	// edits do apply, because changing them recreates the render state.

	/** Visual tube radius (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (ClampMin = "0.1", Units = "cm", DisplayName = "Rope Radius"))
	float Radius = 2.0f;

	/** Sides in the tube's cross-section. Higher is rounder. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "3", ClampMax = "32", DisplayName = "Sides"))
	int32 NumSides = 8;

	/** Tube smoothing: Catmull-Rom subdivisions per segment (1 = off). Simulation nodes are left alone; only
	 *  the render centerline is resampled, estimating curvature from neighbouring nodes. Default 1 because
	 *  an interpolated ring can bulge outside the node polyline — most visibly where the rope lies against a
	 *  wall — so with closely spaced nodes a straight connection is the more accurate one. Raise it for
	 *  sparse ropes that need to look round, and pull the overshoot back in with TubeSmoothingAlpha.
	 *  If NumRings = (NumParticles-1) × Subdiv + 1 exceeds the GPU tube's ring cap, the proxy lowers it. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "1", ClampMax = "8", DisplayName = "Smoothing Subdivisions"))
	int32 TubeSmoothingSubdiv = 1;

	/** Catmull-Rom knot parameter for tube smoothing: 0 = uniform, 0.5 = centripetal (less tangential
	 *  overshoot at sharp corners, so the middle ring against a wall bulges through it less), 1 = chordal.
	 *  The CPU and GPU smoothing paths share this value so they render alike. No effect when Subdiv is 1. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Smoothing Strength"))
	float TubeSmoothingAlpha = 0.5f;

	// The taut presentation values below are read every frame (not proxy-captured), so they are
	// BlueprintReadWrite and a runtime change applies immediately.

	/** Render-only shaping of a taut wrapped hold: the hand-side free span is straightened toward its
	 *  chord and a short thrum plays when the chain snaps taut, so the tension the tether applies is
	 *  visible on camera. The simulation, tension and gameplay are unaffected. A span bent over a
	 *  corner or pivot fades the effect out automatically and renders as solved. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render|Tuning", meta = (DisplayName = "Taut Presentation"))
	bool bTautPresentation = true;

	/** How far the taut free span is straightened toward its chord, from 0 (as solved) to 1 (dead
	 *  straight). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render|Tuning",
		meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bTautPresentation", DisplayName = "Taut Straightening"))
	float TautStraightening = 1.0f;

	/** Peak displacement, in centimetres, of the decaying string thrum played the moment the chain
	 *  snaps taut. 0 disables the thrum and keeps the straightening. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render|Tuning",
		meta = (ClampMin = "0.0", Units = "cm", EditCondition = "bTautPresentation", DisplayName = "Taut Thrum Amplitude"))
	float TautThrumAmplitude = 2.5f;

	/** Material for the rope tube. Falls back to the engine default material when unset.
	 *  Replace it at runtime with SetMaterial(0, M): the scene proxy captures the material when it is built,
	 *  and a direct assignment does not mark the render state dirty, so it would not show until the proxy is
	 *  next rebuilt. Blueprint's direct Set cannot be hooked, which is why this is not BlueprintReadWrite —
	 *  the same reason RopeLength is read-only with a setter. Editor edits go through PostEditChangeProperty. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (DisplayName = "Material"))
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	//~ API ---------------------------------------------------------------

	/**
	 * Stamp a whole preset (URopePreset) onto this rope: copy its values and reinitialize.
	 * **Only accepted in Free or Loaded** — in any other phase (mid-flight, mid-wrap) it changes nothing and
	 * returns false.
	 * Applying one reseeds the sim (InitRope), rebuilds the render state and material instance, reacquires
	 * the tip, and realigns mode and phase: a GuaranteedWrap rope ends up Loaded, and a rope that was Loaded
	 * returns to Free if the preset switches it to another mode.
	 * Instance wiring such as TipMeshComponentTag is outside the preset and survives, so a tip adopted by tag
	 * stays adopted. A preset with bUseTipMesh = false does not hide that tip; that is the instance's business.
	 * Not replicated — this is a local stamp.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool ApplyPreset(const URopePreset* Preset);

	/** Throw the rope. FullSimulation and AssistedJudged launch the tip at the throw speed and enter a
	 *  physical Flight; GuaranteedWrap is accepted only from Loaded and follows a resolved path through
	 *  GuidedThrow. Direction comes from ThrowParams.FrameMode, the single source of truth for the throw
	 *  frame. To pass a direction explicitly, use ThrowWithContext(FRopeThrowContext). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw();

	/** Extended throw entry point: the Wielder computes the origin, frame and velocity and passes them in. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowWithContext(const FRopeThrowContext& ThrowContext);

	/** **Actual sweep radius** for an aim query: resolves a requested radius of 0 to this rope's fallback.
	 *  The aiming visualization draws the same figure, and since 0 is the default the fallback is the
	 *  usual case. */
	float GetAimRayEffectiveQueryRadius(float RequestedRadius) const;

	/** Register the world span the aim ray inspects as the collider subsystem's aiming gather region. */
	void SetAimRayColliderQueryBounds(const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius);
	/** Leaving aim mode clears the aiming gather region, its snapshot and any pending or cached result. */
	void ClearAimRayColliderQueryBounds();
	/** Queue a HUD or preview query. It resolves right after the subsystem's normal collider gather rather
	 *  than forcing an immediate re-gather. */
	void QueueAimRayQuery(const FRopeAimRayThrowRequest& Request);
	/** Latest HUD or preview result, resolved at the last gather. The Wielder consumes it on its next tick,
	 *  so it can be up to one frame old. */
	bool GetLatestAimRayQueryResult(FRopeAimRayQueryResult& OutResult) const;
	/** Compatibility shim. It no longer re-gathers providers: it forwards to QueueAimRayQuery and returns false. */
	UE_DEPRECATED(5.7, "Use QueueAimRayQuery/GetLatestAimRayQueryResult. Immediate collider refresh was removed.")
	bool RefreshAimRayQueryColliders(const FRopeAimRayThrowRequest& Request);
	/** Resolve an aim request against the current aiming collider list. With no hit, OutContext is the
	 *  base-context fallback. */
	bool ResolveAimRayThrowContext(const FRopeAimRayThrowRequest& Request, FRopeThrowContext& OutContext,
		FRopeAimRayHitResult* OutHit = nullptr, FRopeAimRayHitResult* OutBlockedHit = nullptr) const;
	/** Queue a request that becomes a real throw as soon as the next collider gather completes. */
	void QueueAimRayThrow(const FRopeAimRayThrowRequest& Request);

	//~ Wielder contract (C++ only) -----------------------------------------------
	// Entry points for the Wielder's aiming and GuaranteedWrap preview flow. Not a general user API and not
	// exposed to Blueprint — game code normally does not call these, only a Wielder or a reimplementation of
	// the same contract. The preview target comes solely from the aim hit; nothing searches around the throw
	// direction for an alternative.

	/** Build the GuaranteedWrap preview. It returns the contacts and anchors that GuidedThrow and Wrapped
	 *  need to start, not just a render centerline. */
	bool BuildPreparedWrappingPreview(const FRopeThrowContext& ThrowContext, FRopePreparedThrowPreview& OutPrepared,
		FString* OutFailureReason = nullptr) const;

	/** Throw along a prepared preview as the authoritative path — no Flight, no Contacting, no re-search. */
	bool ThrowWithPreparedPreview(const FRopePreparedThrowPreview& Prepared);

	/**
	 * Resolve a GuaranteedWrap aim request into a prepared path right after the next normal collider gather.
	 * With no montage, pass bExecuteWhenReady = true to throw as soon as it resolves; pass false to hold it
	 * for the montage notify, which then calls RequestExecuteQueuedGuaranteedAimThrow.
	 * Callback order is OnPrepared → the throw itself → OnResolved.
	 */
	bool QueueGuaranteedAimThrow(const FRopeAimRayThrowRequest& Request, bool bExecuteWhenReady);
	/** Execute the queued request if it has resolved; otherwise mark it to fire the moment it does. */
	bool RequestExecuteQueuedGuaranteedAimThrow();
	/** Drop a queued request that never fired — montage cancelled, mode changed, or EndPlay. */
	void CancelQueuedGuaranteedAimThrow();

	/**
	 * Enter the Loaded ready state: the tip is held in the hand socket, with the rope tube shown or hidden
	 * per bShowRopeWhenLoaded (hidden by default). **GuaranteedWrap only.**
	 * **Accepted only from Free or Loaded** — a rope in flight or already wrapped cannot be loaded, and the
	 * call is a no-op.
	 * A GuaranteedWrap rope starts Loaded at BeginPlay, and Loaded is the only phase it can throw from
	 * (CanThrowNow). Binding this call to input is the caller's job.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void EnterLoaded();

	/** Set whether the rope tube is drawn while Loaded. Applied at once if already Loaded, otherwise from the
	 *  next entry into Loaded — a deployed rope's visibility is untouched. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetShowRopeWhenLoaded(bool bShow);

	/** Flip the Loaded rope display, for binding to a key. Returns the value after flipping. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool ToggleShowRopeWhenLoaded();

	/** Is the rope tube set to be drawn while Loaded? */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsShowRopeWhenLoaded() const { return bShowRopeWhenLoaded; }

	/** Can this rope be thrown right now (mode × current phase)? GuaranteedWrap only from Loaded; the other
	 *  modes always. Throw entry and the aiming HUD share this gate. Game rules such as stamina are separate —
	 *  see the Wielder's CanThrow(). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool CanThrowNow() const { return RopeWrapModes::CanThrowInPhase(ResolveMode, Phase); }

	/** Manually release whatever grab or wrap is in progress (Contacting, Wrapping or Wrapped). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ReleaseWrap();

	/**
	 * Cut the rope from outside gameplay — a sword swing, damage, anything. Force-releases the grab or wrap
	 * in progress with ERopeReleaseReason::Cut. The flow matches ReleaseWrap; only the reason differs, so the
	 * game can react to it separately (a snapping presentation, say). It does not physically sever the rope
	 * into two pieces; that would need runtime length change and splitting.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void CutRope();

	UFUNCTION(BlueprintPure, Category = "Rope")
	ERopePhase GetPhase() const { return Phase; }

	/**
	 * Builds the current gameplay-authoritative hand-side length constraint.
	 *
	 * Valid from Wrapping through Wrapped, independent of tension/taut state and GPU readback.
	 * The pivot is resolved from the live target binding and MaxDistance is the material length
	 * from node 0 to the first hand-side anchor. A node-0 contact is a valid zero-radius constraint;
	 * self-wrap and invalid bindings return false.
	 */
	bool BuildWielderMovementConstraint(
		FRopeWielderMovementConstraint& OutConstraint,
		float PendingReelDeltaTime = 0.0f) const;

	/**
	 * Projects a proposed rope-pin world position into the current hard length boundary.
	 * Custom Pawn/Mover implementations can call this immediately before applying their final move.
	 *
	 * @return true when a valid Wrapping/Wrapped constraint exists. bOutWasConstrained tells whether
	 *         DesiredPinWorld was outside and actually projected.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope|Hold")
	bool ConstrainWielderLocation(
		FVector DesiredPinWorld,
		FVector& OutConstrainedPinWorld,
		FVector& OutBoundaryNormal,
		bool& bOutWasConstrained) const;

	/**
	 * Is the Wielder end — this rope's own owner — a simulating physics body rather than a kinematic movement
	 * adapter? A Chaos vehicle or a physics prop is one; a Character is not, because its authority is the
	 * capsule its movement component sweeps.
	 *
	 * Such an owner must not be position-projected from the game thread: the solver owns its transform, so the
	 * move is discarded by the next physics sync (and warns in the editor), and UMovementComponent::Velocity is
	 * an output mirror it never reads back. The hand-side length boundary is enforced for it by the physical
	 * tether instead, which binds that body directly as one side of the Chaos constraint. Custom movement
	 * should consult this before calling ConstrainWielderLocation.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope|Hold")
	bool IsWielderPhysicallySimulated() const;

	/**
	 * Tension in one segment (SegmentIndex spans node i to i+1), derived from the solver's XPBD distance λ
	 * as F = max(0, -λ)/h². The units are relative to a unit-mass node, so one node hanging under gravity
	 * reads about 980. Only stretch is positive; slack and compression read 0.
	 * A GPU-resident rope's mirror lags one to two frames, and phases that do not solve (Contacting,
	 * Releasing) hold the previous value.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetSegmentTension(int32 SegmentIndex) const;

	/**
	 * Highest tension across all XPBD segments, for diagnostics. This is a solver and debug reading — for
	 * gameplay load, pull activation and auto-release, use GetConstraintTension.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetMaxTension() const;

	/**
	 * This frame's pull sample: the direction to the first hand-side anchor, and the authoritative
	 * constraint tension. Recomputed every Wrapping and Wrapped frame — observed from the wrapping
	 * state's anchors before the commit, and from the wrap controller after it.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool GetPullSample(FVector& OutDirection, float& OutTension) const
	{
		OutDirection = PullDrive.LastPullSample.Direction;
		OutTension = GetConstraintTension();
		return PullDrive.LastPullSample.bValid;
	}

	/**
	 * Is the rope taut this frame? At the default threshold (0) this is a pure material-length geometry
	 * check; only when ActivePullTautTension > 0 does the authoritative constraint tension act as an extra
	 * load gate. XPBD SegmentTension plays no part in it.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsPullTaut() const { return PullDrive.bPullTaut; }

	/**
	 * Was the chain geometrically taut entering this frame? A live hand-to-anchor material boundary is the
	 * source of truth when one exists, and only the legacy path falls back to sag plus chord hysteresis.
	 * SegmentTension is a solver diagnostic and is not an input to this gameplay state.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsChainTaut() const { return PullDrive.bChainTaut; }

	/** This frame's tether overshoot (cm): straight-line hand-to-anchor distance minus the available rope
	 *  length, clamped at 0. Computed every Wrapping and Wrapped frame even with the tether off. Useful
	 *  for game reactions such as pulling the wielder or detecting that they have left the ground. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetTetherOvershoot() const { return LengthConstraintState.LastViolation; }

	/** Share of the tether correction the target actually took this frame, 0..1. The final value for both
	 *  automatic (mass-based) and manual splits: 1 means the wielder took none, 0 means it took all of it.
	 *  Useful for gating wielder traction and for debugging. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetEffectiveTetherTargetShare() const { return PullDrive.LastTargetShare; }

	/**
	 * This frame's gameplay-authoritative material constraint tension, λ/dt in kg·cm/s².
	 * The backends are mutually exclusive: a physical target reports the Chaos constraint force, a
	 * hard-projected pawn reports the reaction of the motion rejected before projection, and the legacy or
	 * custom path uses the analytic solve.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetConstraintTension() const { return LengthConstraintState.GetTension(); }

	/** Backward-compatible name. New gameplay code should use GetConstraintTension. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetTetherTension() const { return GetConstraintTension(); }

	/**
	 * Can the target be dragged? This is the source of truth for the active pull's climb-in direction, and a
	 * pure function (no UObject, unit-testable). The target is draggable when its effective mass is at most
	 * the wielder's. bPrev carries the previous answer for hysteresis: to flip it, the other side must be
	 * MarginRatio (≥ 1) times heavier, which keeps the decision from flapping at the boundary.
	 * Infinite mass — an anchor — comes in as +BIG_NUMBER, so an infinite target cannot be dragged and an
	 * infinite wielder can drag anything.
	 */
	static bool DecideTargetPullable(float EffMassTarget, float EffMassWielder, bool bPrev, float MarginRatio);

	/**
	 * Set the active pull force. While Wrapped and taut, a *constant* force of this magnitude is applied to
	 * the target each frame. It does not scale with tension, so there is no feedback runaway. 0 stops it.
	 * Intended to be switched on while an input is held and off on release (the Wielder's PullAction does
	 * exactly that). Against a character target the force is divided by mass and competes with ground
	 * friction, so useful magnitudes run from tens of thousands upward.
	 * The taut gate comes from HoldConfig (bActivePullRequiresTaut, ActivePullTautTension) and is readable
	 * through IsPullTaut(). Passing bIgnoreTautGate = true bypasses that gate for this call alone, which is
	 * how an animation pull window marks its "ignore tension" span (UAnimNotifyState_RopePull).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetActivePull(float Force, bool bIgnoreTautGate = false);

	/** Current rope length (cm). Reeling changes it; RopeLength is both the default and the cap. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetCurrentRopeLength() const { return Sim.RopeLength; }

	/**
	 * Set the rope length directly — the immediate form of reeling. Clamped to [MinRopeLength, RopeLength].
	 * The node count stays fixed and the segment rest lengths shrink uniformly, so it reaches the solver
	 * (CPU and GPU alike) from the next frame without a reseed.
	 * Shortening it while Wrapped reduces the available length, so the tether pulls the target in; with no
	 * tether the tension rises instead, which can be combined with TensionRelease.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetRopeLength(float NewLength);

	/**
	 * Set the reel speed (cm/s). Positive reels in (shorter), negative reels out toward the initial length,
	 * 0 stops. Applied every frame in Free, Flight and Wrapped; held during Contacting, Wrapping and
	 * Releasing, because path building depends on the segment length. Intended for held input (the
	 * Wielder's reel-in and reel-out actions).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetReelRate(float CmPerSecond);

	/** Is the rope asleep — at rest in Free, with the solve skipped? */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsSleeping() const { return Throttle.IsAsleep(); }

	/** Iteration multiplier from the current distance LOD (1 = full quality). For debugging and profiling. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetSolverLODScale() const { return Throttle.GetSolverLODScale(); }

	/** Was this rope dispatched to the GPU this frame? **This does not mean it solved physics** — the
	 *  subsystem sends solve frames and override-only frames (Wrapping, Releasing, GuidedThrow) to the GPU
	 *  alike. Read it together with the two getters below rather than as "the GPU solved". For debugging. */
	bool IsGpuSteppedThisFrame() const { return SimFrame.bGpuSteppedThisFrame; }

	/** Did this rope actually run physics solve steps this frame, on either CPU or GPU? Sleep, Contacting,
	 *  Releasing and logic-override-only frames are false. Combined with IsGpuSteppedThisFrame() it isolates
	 *  the CPU fallback solve: WasSolvedThisFrame() && !IsGpuSteppedThisFrame(). For debugging and profiling. */
	bool WasSolvedThisFrame() const { return SimFrame.bSolveThisFrame; }

	/** Did a logic phase (Wrapping, Wrapped, Releasing, GuidedThrow, …) override node positions this frame —
	 *  that is, positions moved without a solve? Together with the two getters above this splits the frame
	 *  into six paths: SLEEP / GPU_SOLVE / GPU_OVERRIDE / CPU_SOLVE / CPU_OVERRIDE / IDLE. On the GPU path
	 *  IsGpuSteppedThisFrame() alone cannot tell an override apart from a solve; on the CPU path this value
	 *  is what separates override from idle. For debugging and profiling. */
	bool HadLogicOverrideThisFrame() const { return SimFrame.OverrideFrame.HasAny(); }

	/** Name of the bone currently wrapped (valid while Wrapped, None otherwise), readable without the event. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	FName GetWrappedBoneName() const { return WrapController.State.BoneName; }

	/** The component currently wrapped, skeletal or static. Null when not Wrapped or the target is gone. */
	const USceneComponent* GetWrappedComponent() const { return WrapController.State.Mesh.Get(); }

	/** The component a Flight capture latched onto — the target the wrap attempt is heading for. Set from
	 *  the moment OnRopeCaptured fires, through Contacting and Wrapping; null otherwise. The
	 *  GuaranteedWrap path skips Contacting and commits before its Captured fires, so there the target
	 *  reads through GetWrappedComponent instead. */
	const USceneComponent* GetContactCandidateMesh() const { return ContactTracker.CandidateMesh; }

	/** The skeletal mesh currently wrapped (valid while Wrapped, null otherwise); GetOwner() on it reaches the
	 *  target actor. Storage is a weak const USceneComponent now that static targets can be wrapped, so this
	 *  returns null for a static target. The cast lives in the .cpp to keep a heavy include out of the header. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	USkeletalMeshComponent* GetWrappedMesh() const;

	/** Number of centerline nodes (= NumParticles once the sim is initialized). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	int32 GetNodeCount() const { return Sim.Num(); }

	/** World position of a centerline node (0 = hand or anchor, GetNodeCount()-1 = free end). An out-of-range
	 *  index gives ZeroVector. For Blueprint use such as attaching an effect or sound to the rope's end —
	 *  in C++, GetCenterlinePositions() avoids the copy. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetNodePosition(int32 NodeIndex) const
	{
		return Sim.Positions.IsValidIndex(NodeIndex) ? Sim.Positions[NodeIndex] : FVector::ZeroVector;
	}

	const TArray<FVector>& GetCenterlinePositions() const { return Sim.Positions; }

	/**
	 * Applies this frame's taut-hold presentation shaping (straightening and thrum, see
	 * RopeTautPresentation.h) to a world-space copy of the centerline, exactly as the render push
	 * applies it, and reports whether anything moved. A no-op outside a taut Wrapped hold.
	 * Anything placed on the *visible* rope — hand IK targets, attached effects — must run its copy
	 * through this before sampling, or it will sit on the solved pose a few centimetres off the tube
	 * the player actually sees.
	 * bIncludeThrum = false shapes with the straightening only: right for a *grip* target, since a
	 * gripping hand pins the rope rather than riding its vibration, and a 12 Hz wave fed into an IK
	 * effector reads as the arm shaking.
	 */
	bool ApplyTautPresentationShaping(TArray<FVector>& WorldPoints, bool bIncludeThrum = true) const;

	/**
	 * Pins one interior centerline node to a scene component's socket every Wrapped frame — the hang
	 * grip: the rope follows the animated hand, instead of the hand chasing simulated nodes that swing
	 * inertia carries behind the character. The node is held by the same per-frame override the wrapped
	 * hold uses (position plus zero inverse mass, CPU and GPU alike), and the taut presentation
	 * straightens from the pinned node upward rather than from the lower hand.
	 * The interiors between the two hands are draped kinematically too (chord plus slack-derived sag):
	 * a nearly taut sub-arm's-length chain pinched between two animation-driven ends has no resolvable
	 * dynamics at rope node density, so simulating it only renders as trembling.
	 * NodeIndex is clamped each frame to stay strictly between the hand and the first wrapped node.
	 * Clearing restores the whole span to the solver without a fling. Wrapped only; other phases
	 * ignore the pin. URopeWielderComponent drives this from its hang regrip.
	 */
	void SetHangGripPin(USceneComponent* Target, FName Socket, int32 NodeIndex);
	void ClearHangGripPin();

	/** The node the hang grip pinned this frame, INDEX_NONE while inactive. */
	int32 GetHangGripPinNode() const { return AppliedHangGripPinNode; }

	// Extension point for game code that drives the tip itself while Free (bSyncTipMeshOnFree = false) — in
	// that case the rope does not touch this component's transform during Free.

	/** The tip attachment component. Null when the tip is unused or was never acquired. */
	UFUNCTION(BlueprintPure, Category = "Rope|Tip")
	UStaticMeshComponent* GetTipMeshComponent() const { return TipMeshComponent; }

	//~ Events ----------------------------------------------------
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnWrapped OnRopeWrapped;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnCaptured OnRopeCaptured;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnReleased OnRopeReleased;

	/** Every phase transition, excluding same-phase resets. Finer-grained than Wrapped/Captured/Released for
	 *  driving UI and audio. It is broadcast from inside the transition (SetPhase), so a handler must not
	 *  change the rope's state — calls like ReleaseWrap are not supported here. Bind those to OnRopeWrapped
	 *  or OnRopeReleased, which fire once the transition has finished. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnPhaseChanged OnRopePhaseChanged;

	/** Fires right after a successful ApplyPreset, and not at all if the preset was rejected. The Wielder
	 *  subscribes to resync its mode-driven state (preview, tick), and game code can use it for its own
	 *  reactions such as a UI refresh. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnPresetApplied OnPresetApplied;

private:
	/**
	 * One simulation frame runs in three stages, driven by URopeSimSubsystem (hence the friend declaration:
	 * the component does not tick itself and game code never calls these, so they are private).
	 *
	 * The split is not arbitrary — the middle stage fans out across ropes in parallel on the CPU, or becomes
	 * a GPU dispatch, so the only question that decides where code belongs is whether it needs the solve
	 * result.
	 *  Prepare (game thread): produces solve *input* — advance the pinned target, compute whip targets, run
	 *                 the logic phases (Contacting, Wrapping, Wrapped, Releasing), fill OverrideFrame and
	 *                 decide bSolveThisFrame. Anything that does not need the solve result belongs here; it
	 *                 is the last point before the solve where UObjects and events may be touched. The
	 *                 collider snapshot (FrameColliders) is gathered centrally by the subsystem beforehand.
	 *  Solve (parallel): POD (Sim) and const colliders only — steps the solver when bSolveThisFrame (Free,
	 *                 Flight, Wrapping, Wrapped). No UObjects, events or transitions; this is the
	 *                 thread-safety boundary. Under Wrapped the latch nodes have InvMass = 0, so only the
	 *                 free span moves.
	 *  Finalize (game thread): consumes solve *output* — Flight contact detection reads the node motion path
	 *                 (Prev → Pos), which is solve output, so it has nowhere else to live. That asymmetry is
	 *                 why logic sits in Prepare while detection sits here. Transitions, event broadcasts, the
	 *                 render push and observation (stats, debugger snapshot) also happen here.
	 */
	void PrepareSimFrame(float DeltaTime, const TOptional<FVector>& LODCameraLocation);
	void SolveSimFrame(float DeltaTime);
	void FinalizeSimFrame(float DeltaTime);

public:
	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void SendRenderDynamicData_Concurrent() override;
	// The editor does not tick the subsystem, and a freshly spawned rope should still be visible: initialize
	// the sim on register, and push the centerline once right after the render state is created so BuildTube
	// has data without a tick.
	virtual void OnRegister() override;
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
#if WITH_EDITOR
	// Changing NumParticles or RopeLength in the editor rebuilds the sim with the new values so the proxy topology matches.
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	//~ UPrimitiveComponent / UMeshComponent
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

protected:
	//~ Extension hooks (for subclasses) -------------------------------------------
	// All of these are called on the game thread on a cold path — there is no hook inside the parallel Solve
	// stage. The node-level hot loop (the solver and the logic classes) is the POD and GPU parity reference,
	// so it is deliberately not a virtual extension point.
	// When adding a hook, stating its calling thread, phase and frequency in the comment is part of the contract.

	/** Called right after a phase transition and right before OnRopePhaseChanged, once per transition (same-phase resets excluded). */
	virtual void OnPhaseChanged(ERopePhase OldPhase, ERopePhase NewPhase) {}

	//~ Loaded presentation hooks — game thread, cold path. Override the defaults to customize the presentation.

	/** Where the tip is held while Loaded, in world space. Default: the LoadedHandSocket socket on the owner's
	 *  skeletal mesh, or this component's transform when there is none.
	 *  ⚠ Called **twice per frame** while Loaded (once for the pinned node, once for the tip mesh), not once
	 *  per transition — cache anything expensive. */
	virtual FTransform GetLoadedTipTransform() const;

	/** Called once **on the edge** into Loaded: calling EnterLoaded() again while already Loaded does not fire
	 *  it, which matters because applying a preset to a GuaranteedWrap rope always calls EnterLoaded().
	 *  Default: show or hide the rope tube per bShowRopeWhenLoaded. Pairs 1:1 with OnDeployFromLoaded. */
	virtual void OnEnterLoaded();

	/** Called once when leaving Loaded — a throw taking hold, or a preset switching the rope to another mode.
	 *  Default: show the rope tube again and restore the full RopeLength.
	 *  ⚠ GetPhase() is **still Loaded** at this point; the transition happens after the hook returns. */
	virtual void OnDeployFromLoaded();

	/**
	 * Wrap target gate. Returning false makes that (Mesh, Bone) candidate invisible to the rope — override it
	 * to restrict what may be wrapped by team, tag or any other game rule. Default true (allow everything).
	 *
	 * **Every path that picks a target goes through this one gate.** Were aiming, preview and detection to
	 * ask separately, they would disagree — aiming refusing a target the preview had already chosen. The call
	 * sites are:
	 *   - Flight candidate selection, per frame and per candidate, plus Contacting re-gather — RemoveNonWrappableCandidates
	 *   - the aim ray query — FRopeAimTargeting::FindAimRayBoneHit (a barred target reads as no hit)
	 *   - preview arc search — injected as FRopeThrowPreviewBuilder::FInput::CanWrapTarget (the builder is
	 *     UObject-free and cannot call the virtual, so the caller passes it as a lambda)
	 *   - prepared throw entry — ThrowWithPreparedPreview, the last line of defence
	 * A new target-selection path must route through this gate too.
	 */
	virtual bool CanWrapTarget(const USceneComponent* Mesh, FName Bone) const { return true; }

	//~ Native event hooks: called right before each delegate broadcast, following the engine's Notify
	//  convention, so a C++ subclass can react without binding to its own delegate.
	virtual void NotifyCaptured(FName Bone) {}
	virtual void NotifyWrapped(const FRopeWrappedEventInfo& Info) {}
	virtual void NotifyReleased(FName Bone, ERopeReleaseReason Reason) {}
	/** Called right after a successful ApplyPreset and right before OnPresetApplied (game thread, cold path, once per apply). */
	virtual void NotifyPresetApplied(const URopePreset* Preset) {}

	/**
	 * Abort gate for a GuaranteedWrap presentation, polled every frame while GuidedThrow runs (game thread,
	 * cold path — the presentation lasts roughly 0.2 s).
	 * The default always returns false, meaning "the guarantee still holds". Override it and return true when
	 * a game rule should break the guarantee — the target dying or teleporting away — and the presentation is
	 * cut short with OnRopeReleased(ThrowAborted). An internal failure is distinguishable as Broken. Losing
	 * the target mesh always stops the throw, hook or no hook.
	 * **Polled only for an aimed throw** — a throw into open space has no target, so there is no guarantee to
	 * break and the prepared preview is a stub. It is a policy hook and therefore C++ only; Blueprint reacts
	 * through OnRopeReleased.
	 */
	virtual bool ShouldAbortGuaranteedThrow(const FRopePreparedThrowPreview& Prepared) const { return false; }

	/**
	 * Final interpretation of a throw context — **the only extension hook that may touch it**, called once per
	 * throw. It rebuilds the frame into an orthonormal right-handed basis (Forward is authoritative, Up is
	 * orthogonalized, Right is re-derived as Up × Forward and any incoming Right is ignored) and fills in the
	 * velocity and origin fallbacks. Override it for aim assist and similar, and the preview and the real
	 * throw stay in agreement automatically.
	 *
	 * **Every throw passes through this gateway**, whichever producer built the context — the Throw()
	 * convenience path's FRopeThrowContext::MakeDefault, URopeWielderComponent::BuildThrowContext, or a
	 * direct Blueprint call. A GuaranteedWrap throw passes through it once when the preview is built, and
	 * reuses that result.
	 *
	 * **An override must be pure** — same input, same output, no state changes. Because a GuaranteedWrap throw
	 * reuses the context resolved at preview time, randomness here (aim spread, say) makes repeated preview
	 * queries disagree with one another, and makes the previewed trajectory diverge from the actual throw.
	 */
	virtual FRopeThrowContext ResolveThrowContext(const FRopeThrowContext& ThrowContext) const;

	/**
	 * Apply the active pull force, called every frame while Wrapped, taut and pulling. The default receiver
	 * chain is: simulating bone → CharacterMovement → simulating root. Override it for a custom movement
	 * system (Mover and the like), a vehicle, or any special target.
	 * Force is the pull direction × the tension cap, so |Force| is that cap. A physical body is driven by a
	 * tension-capped velocity drive (ApplyPullVelocityDrive), and DeltaTime is what turns the cap into an
	 * impulse limit (tension × dt).
	 *
	 * This is the *policy* hook for the active pull — what to pull, and how hard. To intercept per receiver
	 * instead, use ApplyTractionToReceiver; even this default implementation goes through that gate before
	 * anything is applied.
	 */
	virtual void ApplyPullForce(const FVector& Force, const FRopePullSample& Pull, float DeltaTime);

	/**
	 * **The single gateway** the rope passes through before applying traction to any receiver (game thread,
	 * at most a few calls per frame, cold path). Return true to claim the request as handled by the subclass
	 * and skip the built-in application; the default returns false and the built-in path runs.
	 *
	 * **Every force and velocity the rope injects comes through here** — the automatic tether at both ends,
	 * the active pull on the target, climb-in on the wielder, and the slack break. So overriding this one
	 * function is enough to route all rope traction into a custom movement system, a vehicle, or a special
	 * receiver.
	 *
	 * The unit of Request.Amount depends on Request.Source; see the FRopeTractionRequest comment.
	 * Returning true still leaves the rope's own observation and state updates in place, on purpose — the
	 * rope's state must not fork based on whether a subclass handled the request.
	 */
	virtual bool ApplyTractionToReceiver(const FRopeTractionRequest& Request) { return false; }

	// Read-only access to the sim state, for subclasses. Changes go through the public API (Throw, the Set* family).
	const FRopeSimState& GetSimState() const { return Sim; }

private:
	//~ Tip attachment runtime state -----------------------------------------------
	// This one is a UObject, unlike the value-type sim members, so it needs GC tracking (Transient UPROPERTY).
	// EnsureTipMesh acquires it on the way into a throw, and FinalizeSimFrame keeps it on the free end each frame.
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> TipMeshComponent = nullptr;

	// Did we spawn it? Only a tip we spawned is destroyed in EndPlay; an adopted component is left alone.
	bool bTipMeshSpawnedByUs = false;

	// Authored world scale of an adopted tip, so overwriting its transform preserves the visual size.
	FVector TipMeshAuthoredScale = FVector::OneVector;

	// Authored relative transform of an adopted tip, captured when we adopted it. Teardown restores it, so
	// re-acquiring (on a preset switch) reads the authored baseline instead of the transform we overwrote
	// each frame, which would otherwise accumulate scale.
	FTransform TipMeshAuthoredRelative = FTransform::Identity;

	// Authored collision setting of an adopted tip. With bTipMeshCollision = false we switch collision off and
	// teardown puts this back, respecting the external component's ownership. Meaningless for a tip we spawned.
	TEnumAsByte<ECollisionEnabled::Type> TipMeshAuthoredCollision = ECollisionEnabled::QueryAndPhysics;

	// Split out so branches that only need existence do not pay for reading the socket transform.
	bool HasTipSocket(FName Socket) const;

	// Acquire, destroy and follow the tip attachment across BeginPlay..EndPlay (only while bUseTipMesh is on).
	void EnsureTipMesh();
	void TeardownSpawnedTipMesh();
	void UpdateTipMeshTransform(float DeltaTime);

	// Presentation-side filter for the segment-follow fallback. Reset on phase transitions and sim
	// reseeds; the placement-contract branches (Loaded, the wrapped embed, the aimed guided throw)
	// never touch it.
	FRopeTipStabilizer TipStabilizer;

	// Bundles the Rope|Tip stabilizer properties into the stabilizer's per-call params.
	FRopeTipStabilizer::FParams MakeTipStabilizerParams() const;

	// Push bTipMeshCollision onto the current tip (no-op without one). Called on acquisition and on edit.
	void ApplyTipMeshCollision();

	// Single source for where the tip sits while Loaded: LoadedTipRelativeTransform composed onto the
	// (overridable) GetLoadedTipTransform() socket frame. Both Loaded consumers - the tip mesh placement
	// and the pinned free-end node - must read this, never the raw socket transform.
	FTransform MakeLoadedTipBaseWorld() const;

	//~ Pierce embed (socket-based) helpers ----------------------------------------
	// **The single source of truth** for whether socket placement is active: tip in use, socket opt-in, and
	// GuaranteedWrap. Aligning the head with a pierce point only means anything in that mode, so the other
	// modes ignore the socket names even when they are filled in. HasTipSocket goes through this predicate,
	// which switches off the whole socket path with it.
	bool IsTipSocketPlacementActive() const
	{
		return bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap;
	}
	// Read a tip static mesh socket as a component-local transform. Returns false when socket placement is off
	// or the socket does not exist, and the caller falls back. This is the only socket read.
	bool ReadTipSocketLocal(FName Socket, FTransform& OutLocal) const;
	// Tip placement in local space, including an adopted component's authored scale and TipMeshRelativeTransform.
	FTransform MakeTipPlacementTransform() const;
	// The "placement reference" socket-local transform. The actual SetWorldTransform uses MakeTipWorldTransform(BaseWorld).
	FTransform MakeTipPlacementSocketLocal(const FTransform& SocketLocal) const;
	FTransform MakeTipWorldTransform(const FTransform& BaseWorld) const;
	// Invert the tip mesh transform so the rope attachment point — the tail socket, or the mesh origin — lands on RopeAttachWorld.
	void ComputeTipFollowTransform(const FVector& RopeAttachWorld, const FVector& ForwardDir,
		FTransform& OutComponentWorld) const;
	// World position where the rope should attach (tail socket, or mesh origin) for a given component transform.
	FVector ResolveTipRopeAttachWorld(const FTransform& ComponentWorld) const;
	// Resolve the owner-local prepared path into a world snapshot at throw time and write it into the pierce socket targets.
	void ApplyPierceSocketTargetsToPrepared(FRopePreparedThrowPreview& InOutPrepared) const;
	// Recover the current world hit point from the single pierce anchor in Prepared, bone-local when possible.
	bool ResolvePreparedPierceHitPoint(const FRopePreparedThrowPreview& Prepared, FVector& OutHitPoint) const;
	// Build the mesh-origin (component) world transform that puts the head socket at HitPoint with the
	// Tail → Head socket vector along PierceDir. With a tail socket it also reports the rope attachment point
	// in world space; without one, the mesh origin. Returns false when TipSocketName names no socket (pierce
	// embed disabled). The pure placement maths belongs to FRopeTipPlacement.
	bool ComputePierceEmbed(const FVector& HitPoint, const FVector& PierceDir,
		FTransform& OutComponentWorld, FVector& OutTailWorld) const;

	//~ Phase state machine --------------------------------------------------------
	ERopePhase Phase = ERopePhase::Free;
	// A frame that returned to Flight during Prepare (from Contacting, say) never ran Flight's advance and
	// solve, so contact detection in Finalize is delayed one frame — otherwise a stale guide candidate would
	// recapture immediately.
	bool bEnteredFlightDuringPrepareThisFrame = false;

#if WITH_GAMEPLAY_DEBUGGER
	// The phase at the start of the frame. Transitions happen throughout the frame and the end-of-frame phase
	// alone cannot say what moved to what, so the debug snapshot keeps both sides of the transition.
	ERopePhase DebugPhaseAtFrameStart = ERopePhase::Free;
	// The frame (GFrameCounter) the value above was recorded in, so it is written once per frame. The
	// subsystem can run ResolvePendingAimThrow **before** Prepare and transition into Flight through
	// StartFreshThrow, so capturing only in Prepare would already have missed it.
	uint64 DebugPhaseFrameStamp = 0;
	// Did the active pull actually clear the tension gate this Wrapped frame? The value stored by SetActivePull
	// cannot tell a gated frame apart from one with no input at all.
	bool DebugActivePullPassedGate = false;

public:
	/** Pin the frame-start phase at the first contact point of the frame (once per frame; later calls are
	 *  no-ops). Called before the subsystem touches the rope, and again in Prepare for ropes that skip that path. */
	void CaptureDebugFrameStartPhase()
	{
		if (DebugPhaseFrameStamp != GFrameCounter)
		{
			DebugPhaseFrameStamp = GFrameCounter;
			DebugPhaseAtFrameStart = Phase;
		}
	}

private:
#endif

	/**
	 * The single point where Phase is assigned, so every transition logs the same way
	 * ("[Name] Old -> New (Reason)"). Reason is extra log context and may be null.
	 * Event broadcasts and transition cleanup differ per transition and stay with the caller — nothing
	 * implicit happens here.
	 */
	void SetPhase(ERopePhase NewPhase, const TCHAR* Reason = nullptr);

	/**
	 * Drop the "attempt in progress" transient state that must not outlive a phase transition:
	 * ContactTracker, PendingWrapSeed, CaptureTravelFrame, WrappingPhase.State, ContactingElapsed,
	 * FlightNoContactElapsed and TensionOverTime.
	 * Only a successful wrap commit passes bPreservePhysicalTether = true, which keeps the existing Chaos
	 * constraint identity alive.
	 * Members already idle are left alone, so this is safe to call on any transition.
	 * (ReleaseCooldown differs per transition and is set by the caller.)
	 */
	void ResetTransientPhaseState(bool bPreservePhysicalTether = false);

	// Aim-ray targeting logic and state live in FRopeAimTargeting (the AimTargeting member). What stays here
	// is the frame contract with the subsystem: the StartFreshThrow transition, which is orchestration, and
	// SimFrame access, which the component owns.
	// Resolve the pending HUD/preview query into the result cache, right after the subsystem fills AimFrameColliders.
	void ResolvePendingAimQuery();
	// Resolve a queued GuaranteedWrap request into a prepared path against the same aiming list, and throw at once if it was armed.
	void ResolvePendingGuaranteedAimThrow();
	// Resolve a pending request into a hit or fallback context, right after the subsystem fills FrameColliders.
	void ResolvePendingAimThrow();
	// Keep only the mesh and bone an aim-ray throw named as contact and wrap candidates.
	// A collision-free aim flight has the solver ignore this list on purpose, but the wrapping path keeps
	// using the filtered one. An ordinary Flight solve uses the same list.
	void FilterFrameCollidersForAimWrapTarget();
	/** Snapshot of the context an aim query needs: the collider list plus the fallback dimensions. */
	FRopeAimTargeting::FQueryContext MakeAimQueryContext() const;
	/**
	 * Colliders an aiming query may use — the aim ray hit, and the GuaranteedWrap preview arc search.
	 * While aiming, that is the aiming-specific snapshot (the rope's AABB ∪ the ray's span), since a distant
	 * target is not in the physics list. When not aiming it is the physics snapshot, which covers the
	 * Blueprint and AI path where Throw() is called directly with no Wielder aiming flow — there is no ray
	 * span then, so the list around the rope is the only source.
	 */
	const TArray<IRopeCollider*>& GetAimQueryColliders() const;

	//~ Sim state and per-phase logic ----------------------------------------------
	// None of these are UObjects — held by value, outside GC, carrying only POD and weak references.
	// The four logic members below follow the rope's life in order: Throw/Flight → Contacting → Wrapping → Wrapped.
	/** The single source of truth: the particle chain shared by solver, logic and render. */
	FRopeSimState       Sim;
	/** Current-frame view used for Wrapped observation only: a game-thread scratch copy of Sim with the latest start pin and anchor overrides applied. */
	FRopeSimState       PullObservationSim;

	/** XPBD physics. Owns Free and Flight outright, and the free span while Wrapping and Wrapped. */
	FRopeXPBDSolver     Solver;

	/** Throw and Flight: the whip swing (computes guide targets, then applies them). */
	FRopeWhipGuide      WhipGuide;

	/** Flight and Contacting: tracks which bone dominates the contact candidates. */
	FRopeContactTracker ContactTracker;

	// Candidate storage shared by the CPU Flight fallback and the Contacting re-gather. The detector appends,
	// so each path resets it immediately before use. GPU Flight consumes SimFrame.GpuFlightCandidates directly.
	TArray<FRopeContactCandidate> ContactCandidateScratch;

	// Next frame's whip targets for CPU Flight predictive contact. The GPU path carries its own array in the
	// subsystem's dispatch payload and does not use this scratch.
	TArray<FVector> NextGuideTargetScratch;

	/** Contacting: the wrap seed built at capture, which is what the Wrapping phase starts from. */
	FRopeWrapState      PendingWrapSeed;
	/** Set right after a GPU Flight capture, while the pending render-thread step and the CPU Sim have not yet
	 *  been reconciled. Contacting's dwell, dismiss and seed checks are all held off until
	 *  SyncGpuPositionsForHandoff succeeds. */
	bool bPendingGpuCaptureHandoff = false;

	/** Contacting through Wrapping: the rope's travel frame captured at the moment of capture — velocity,
	 *  lay direction and travel plane normal. From Contacting onward the nodes are still, so this is the only
	 *  moment it can be measured. It is the fallback axis for a CaptureTravelPlane guide plane. */
	FRopeCaptureTravelFrame CaptureTravelFrame;

	/** Wrapping: incremental path build, front motion and mass mask (the working state is .State). */
	FRopeWrappingPhase  WrappingPhase;

	/** Wrapped: holds and releases the bone-local latch. */
	FRopeWrapController WrapController;

	/**
	 * Holds an anchorless span of a composite analytic helix as a straight line between the two real anchors
	 * on either side of it. Component-only runtime state: rather than attributing the span to one bone, both
	 * surface bindings are re-resolved together every frame.
	 */
	struct FKinematicVirtualBridge
	{
		/** Interior nodes left without an anchor because the SDF projection failed between the two real surface points. */
		TArray<int32> NodeIndices;
		/** The real surface anchors on each side, re-resolved from their bone-local bindings every frame. */
		FRopeSurfaceAnchor LeftAnchor;
		FRopeSurfaceAnchor RightAnchor;
		/** Original segment count across the span, including both end nodes, × SegmentLength. The reference for diagnosing an over-stretched bridge. */
		float RestSpanLength = 0.0f;
		/** Path distance at which the bridge switches on — when the wrapping front reaches the real anchor on the right. */
		float ActivationFrontDistance = 0.0f;
		/** A bridge registered incrementally before commit waits as false, and turns true the moment both real anchors are pinned. */
		bool bActive = true;
		/** One-shot log latch so one bridge's over-stretch warning does not repeat every frame. */
		bool bLoggedStretchWarning = false;
	};

	/** Spans bridged straight between two real anchors. Pinned straight from the moment the wrapping front reaches them through Wrapped. */
	TArray<FKinematicVirtualBridge> KinematicVirtualBridges;
	/** How far along the runs WrappingPhase has already computed, so the component's bridges stay in step. */
	int32 KinematicVirtualBridgeRunCursor = 0;

	/** GuidedThrow state: either the resolved preview path (aimed) or an arc to the ray's endpoint (open space, bFreeThrow). */
	FRopeGuidedThrowState GuidedThrowState;

	// Normal of the whip guide's spline plane, fixed at the start of the Flight. Entering Wrapping through
	// Contacting reuses it as the axis of the virtual wrap plane placed at the bone.
	bool bHasFlightGuidePlaneNormal = false;
	FVector FlightGuidePlaneNormal = FVector::RightVector;

	// Aim-ray targeting state: the per-throw wrap target lock, the pending HUD/preview query and its result,
	// and the aim throw cue. The query and lock logic lives with it — see FRopeAimTargeting
	// (Logic/RopeAimTargeting.h).
	FRopeAimTargeting AimTargeting;

	/** GuaranteedWrap only: the input ray is resolved once at the next normal gather, then either thrown at once or held until the montage notify. */
	struct FPendingGuaranteedAimThrow
	{
		FRopeAimRayThrowRequest Request;
		FRopeThrowContext ResolvedContext;
		FRopePreparedThrowPreview Prepared;
		bool bQueued = false;
		bool bResolved = false;
		bool bExecuteWhenReady = false;

		void Reset() { *this = FPendingGuaranteedAimThrow(); }
	};
	FPendingGuaranteedAimThrow PendingGuaranteedAimThrow;

	/** Throw the resolved request along its prepared path, or as an open-space arc, with no further queries, and fire the completion or rejection callback. */
	bool ExecutePendingGuaranteedAimThrow();

	//~ Phase timers ---------------------------------------------------------------
	/** Time spent in Contacting, against the WrapDecisionTime threshold. */
	float ContactingElapsed = 0.0f;

	/** Time spent in Flight since the whip ended, with no capture. */
	float FlightNoContactElapsed = 0.0f;

	/** Time left before Releasing returns to Free. */
	float ReleaseCooldown = 0.0f;

	/** How long the peak tension has stayed above TensionReleaseForce while Wrapped. */
	float TensionOverTime = 0.0f;

	// Wrapped traction and smoothing state: the pull sample, three EMAs, the active pull, the tether overshoot
	// and the warning latches. For what each member means and what survives a transition, see
	// FRopePullDriveState (Core/RopePullDriveState.h).
	FRopePullDriveState PullDrive;
	// Passive material-length authority: the rejected movement, the constraint λ and tension, and the live
	// material and anchor history. Never sourced from XPBD SegmentTension.
	FRopeLengthConstraintState LengthConstraintState;
	FRopeResolvedWrappedEndpoints WrappedEndpointCache;

	// Reel speed (cm/s; positive reels in, negative reels out, 0 stops). SetReelRate writes it and UpdateReel
	// applies it each frame.
	float ReelRate = 0.0f;

	// Apply one frame of reeling, at the start of Prepare: change the length by ReelRate × dt in the phases that allow it.
	void UpdateReel(float DeltaTime);

	//~ Sleep and LOD --------------------------------------------------------------
	// The state and the decision live in FRopeSolverThrottle (Logic/RopeSolverThrottle.h); what stays in the
	// component is the camera access, which is game-thread, and the sleep transition log.
	FRopeSolverThrottle Throttle;

	// Distance LOD scale (Prepare, game thread): the subsystem hands over the camera position it fetches once
	// per frame, this turns it into a distance and delegates to Throttle.
	void ComputeSolverLOD(const TOptional<FVector>& CameraLocation);
	// LOD-scaled iteration count, shared by the CPU solve and the GPU step, called by the subsystem.
	int32 GetLODScaledIterations() const { return Throttle.LODScaledIterations(SolverConfig.Iterations); }

	// Automatic traction, half one — the tether. Observe the whole chain's chord and opening speed, solve for
	// λ, then apply an equal and opposite impulse pair at the two ends. Called every Wrapped frame from
	// ApplyWrappedTraction, and the result is recorded in one place, LengthConstraintState.
	void UpdateConstraintTether(float DeltaTime);

	// Automatic traction, half two — the ragdoll target. An engine physics constraint ties the hand side at the
	// corner to an anchor point on the wrapped bone, with a spherical limit at the leg's rest length.
	// This covers **every simulating body**: skeletal bones and component bodies alike.
	//
	// The hand side is one of two carriers. Normally it is a kinematic proxy re-placed at the corner every
	// frame, which is an infinite mass: the target is drawn in and the wielder feels nothing, correct for a
	// character whose movement adapter owns its own reaction, and correct for a corner that models an external
	// pulley. When the rope's owner is itself a simulating body — a Chaos vehicle, a physics prop — that is
	// wrong: nothing absorbs the reaction and the rope can never drag the owner. Such an owner is bound
	// **directly** as Frame1 (bCornerIsOwnerAttachPoint), with no proxy, so Chaos splits the reaction across
	// both bodies by inverse mass inside the same substep.
	// Direct binding requires the corner to be a point genuinely fixed in the owner's body, which is the rope's
	// own attachment point (node 0). The pull-sample fallback puts the corner at a look-ahead node on the rope
	// instead, so it passes false and keeps the proxy.
	// A per-frame game-thread velocity impulse cannot serve them. On a jointed body it is caught between a
	// whole-body-sized kick, which runs away, and a bone-sized λ, which collapses the traction force; and it
	// loses structurally under an airborne load such as a suspended prop, where gravity and swing act during
	// the physics substep while the game thread only corrects a beat late, producing floating, pendulum
	// pumping and a dependence on orthogonal damping. A Chaos constraint is solved inside the substep
	// together with gravity, the joints and ground contact, which is what makes it hold.
	// With a Wielder present, PrePhysics drives it once and PostPhysics only observes the force. A custom
	// mover without a Wielder falls back to UpdateConstraintTether's drive. It is torn down on abort or
	// release, on a target or bone change, and in EndPlay.
	void UpdatePhysicalTether(class UPrimitiveComponent* TargetPrim, FName Bone,
		const FVector& AnchorWorld, const FVector& CornerWorld, float LegRestLen, float DeltaTime,
		bool bCornerIsOwnerAttachPoint);
	/** Reads the current Chaos constraint force only. The proxy transform and the limit are left alone. */
	void SamplePhysicalTetherForce(float DeltaTime);
	/**
	 * Called by the Wielder right after movement and right before Chaos. The attempted move and the velocity
	 * actually rejected by the authoritative projection are recorded as the reaction, and for a physical
	 * target the same attempt is forwarded to the Chaos proxy.
	 */
	void PrepareWielderLengthConstraint(
		const FRopeWielderMovementConstraint& Constraint,
		const FVector& OutwardNormal,
		float RejectedSeparatingSpeed,
		float PositionViolation,
		bool bAtLimit,
		bool bHardProjectionApplied,
		float DeltaTime);
	/** Generalized-mass share of the hard velocity reaction already applied to the Wielder. */
	float ComputeWielderLengthReactionShare(
		const FRopeWielderMovementConstraint& Constraint) const;
	/**
	 * Position-correction share for a hard Wielder projection. Only a simulated target has
	 * a same-frame Chaos receiver for the complementary position; all other targets require
	 * the Wielder to take the full correction.
	 */
	float ComputeWielderLengthPositionCorrectionShare(
		const FRopeWielderMovementConstraint& Constraint) const;
	void TeardownPhysicalTether();

	/** The physical tether's kinematic proxy, which follows the corner, and its constraint. Runtime only, and
	 *  only for a simulating-body target. The proxy is absent while the owner's own body carries Frame1. */
	UPROPERTY(Transient)
	TObjectPtr<class USphereComponent> PhysicalTetherProxy;
	UPROPERTY(Transient)
	TObjectPtr<class UPhysicsConstraintComponent> PhysicalTetherConstraint;
	// Target and bone the constraint is bound to (a change regenerates it) and the current limit (cm; < 0 means unset, and it skips the update).
	TWeakObjectPtr<class UPrimitiveComponent> PhysicalTetherTarget;
	FName PhysicalTetherBone = NAME_None;
	float PhysicalTetherLimit = -1.0f;
	// Frame1's carrier when the owner's own simulating body is bound directly, and null while the kinematic
	// proxy carries it. A change either way regenerates the constraint, which is what a runtime
	// SimulatePhysics toggle on the owner looks like from here.
	TWeakObjectPtr<class UPrimitiveComponent> PhysicalTetherWielder;
	FName PhysicalTetherWielderBone = NAME_None;
	// The attachment point pinned at creation, in the owner actor's space. Unlike the target anchor below this
	// reads the component transform rather than the lagging Sim mirror, and the rope is rigidly attached to its
	// owner, so it is constant and an instantaneous guard is enough. It still guards, because re-attaching the
	// rope to a different socket mid-wrap does relocate it for real and would leave Frame1 stale.
	FVector PhysicalTetherWielderActorLocal = FVector::ZeroVector;
	/** GFrameCounter of the frame in which the Wielder already drove the proxy and limit authoritatively in PrePhysics. */
	uint64 PhysicalTetherPrePhysicsFrame = MAX_uint64;
	// Body-local anchor pinned at creation (constraint Frame2). The reference the drift guard compares against
	// to notice that the wrap anchor moved within the same (target, bone) and regenerate.
	FVector PhysicalTetherAnchorLocal = FVector::ZeroVector;
	// EMA of the observed local anchor, so the drift guard does not regenerate every frame. AnchorWorld comes
	// from the rope's Sim, which on the GPU path mirrors one to two frames late, while the body transform
	// comes from the bone as it is now; on a fast ragdoll bone that lag difference alone shakes the local
	// anchor every frame without any real relocation. Comparing the smoothed value instead of the instant one
	// filters the lag noise out and leaves only sustained relocations — a seed joining, or a promotion.
	FVector PhysicalTetherSmoothedAnchorLocal = FVector::ZeroVector;

	// Tether: compute the wielder's pull direction — hand (node 0) toward the rope's first leg, that is toward
	// the anchor — and return it smoothed into PullDrive.SmoothedWielderPullDir with an EMA (PullDirSmoothTime),
	// so direction jitter cannot make the input axis bounce (a degenerate 180° flip reseeds it raw instead).
	// With bInstantaneous the EMA is skipped for raw geometry: in an airborne swing the EMA cannot keep up with
	// the orbit, and the tangential error of the lagging direction brakes and accelerates the swing frame by
	// frame, fighting the player. The state keeps seeding meanwhile, so re-entering the EMA on landing is continuous.
	FVector ComputeSmoothedWielderDir(const FVector& Aim, const FVector& DirToAim, float DeltaTime, bool bInstantaneous);

	// Update this Wrapped frame's pullability decision, independent of overshoot — the active pull's climb-in
	// direction and the distribution observation (the binary LastTargetShare) share PullDrive.bTargetPullable.
	// It compares the effective mass at each end, with hysteresis.
	void UpdateTargetPullable();

	// Resolve target and wielder once per Wrapped traction pass, then share it with that frame's gate, tether and pull.
	const FRopeResolvedWrappedEndpoints* GetOrResolveWrappedEndpoints();

	// Climb-in (target not pullable): apply the active pull force to the wielder, the rope's owner, so the
	// wielder is drawn toward the anchor instead. Mirrors ApplyPullForce's owner side (simulating root → CharacterMovement).
	void ApplyPullForceToWielder(const FVector& Force, float DeltaTime);

	// Active pull as a tension-capped velocity drive: pull the target body along Dir toward the target speed
	// (ActivePullMaxLinearSpeed), clamping the impulse to J = min(mass × ΔV, MaxTension × dt). A light target
	// reaches the target speed at once with no overshoot, and a heavy one lags behind under the tension limit,
	// which is the mass dependence you want. It removes the overshoot, drift and juddering of a constant force (a = F/m).
	void ApplyPullVelocityDrive(UPrimitiveComponent* Prim, FName BoneName, const FVector& Dir, float MaxTension, float DeltaTime) const;

	// Clamp the pulled body's angular velocity with the HoldConfig cap — a safety net for residual ragdoll spin,
	// since the force is applied at the centre of mass and carries no torque of its own. Called after the force
	// is applied. With BoneName None it acts on the whole component.
	void ClampPulledBodyVelocity(UPrimitiveComponent* Prim, FName BoneName) const;

	// Shared finalization for every release trigger: phase transition, handing nodes back, dropping transient state, cooldown and event.
	void FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog);

	// Shared finalization for leaving before the wrap was established (Captured through Wrapping): transition
	// back to Flight, drop the transient state, then send the per-instance release notification — after the
	// state is clear, per DispatchReleased's reentrancy contract. Shared by four call sites: dismiss, stall and
	// two wrapping aborts. Bone is captured by value before the reset. Before commit bWasWrapped is false, so
	// this is per-instance only and raises no central signal.
	void FinishPreCommitReleaseToFlight(FName Bone, const TCHAR* PhaseLog);

	// Shared body of ReleaseWrap and CutRope: release the grab or wrap in progress for the given reason,
	// including working out which bone to report.
	void ReleaseWrapAs(ERopeReleaseReason Reason);

	//~ Subsystem frame contract (written and read by RopeSimSubsystem) -------------
	// Per-frame simulation input/output bundle. For each member's meaning and lifetime see FRopeSimFrameIO
	// (Core/RopeSimFrameIO.h).
	FRopeSimFrameIO SimFrame;

	// State of the last render push. A rope at rest skips the dynamic-data and transform dirty flags, but a
	// switch to or from GPU residency and a change in the component transform both have to be noticed so the
	// new WorldToLocal and local centerline get pushed.
	FTransform LastRenderDataComponentTransform = FTransform::Identity;
	bool bHasLastRenderDataComponentTransform = false;
	bool bLastRenderDataGpuResident = false;

	//~ Taut hold presentation (render only) ----------------------------------------
	// Oscillator and blend state for the taut straightening/thrum (see RopeTautPresentation.h). Updated
	// once per frame in FinalizeSimFrame and consumed by SendRenderDynamicData_Concurrent, which shapes
	// only the copy sent to the renderer — the simulation never sees it. A frame that shapes the
	// centerline also drops the GPU-resident flag on its dynamic data, so the tube reads the shaped CPU
	// upload instead of the solver's untouched position buffer.
	/** Smoothed 0..1 activation, so the shaping fades in and out instead of popping on the taut edge. */
	float TautPresentationBlend = 0.0f;
	/** Decaying thrum amplitude (cm), reset to TautThrumAmplitude on the frame the chain snaps taut. */
	float TautThrumLevel = 0.0f;
	/** Thrum oscillator phase, radians. */
	float TautThrumPhase = 0.0f;
	bool bWasTautPresentationActive = false;
	/** Advances the blend and the thrum oscillator; called every frame from FinalizeSimFrame. */
	void UpdateTautPresentation(float DeltaTime);
	/** The lowest wrapped node index (surface anchors and legacy latches alike), INDEX_NONE without one.
	 *  The hand-side free span the presentation shapes ends here. */
	int32 GetFirstWrappedNodeIndex() const;

	//~ Hang grip pin (see SetHangGripPin) -------------------------------------------
	/** Writes this frame's grip override into SimFrame.OverrideFrame, and the one-shot restore after a
	 *  clear. Called from the Wrapped branch of PrepareSimFrame, after the hold's own overrides. */
	void ApplyHangGripPinOverride();
	TWeakObjectPtr<USceneComponent> HangGripPinTarget;
	FName HangGripPinSocket = NAME_None;
	/** The requested node; clamped each frame against the current first wrapped node. */
	int32 HangGripPinNode = INDEX_NONE;
	/** The node actually pinned this frame (the clamped index), INDEX_NONE while inactive. It is also
	 *  where the taut presentation span starts, so the drawn rope keeps passing through the grip. */
	int32 AppliedHangGripPinNode = INDEX_NONE;
	/** A node whose mass must be handed back to the solver next frame, after a clear mid-Wrapped. */
	int32 PendingHangGripUnpinNode = INDEX_NONE;

	//~ Initialization and utilities ------------------------------------------------
	void InitRope();

	/** Initialize the sim once if it is empty — the safety guard at the top of OnRegister, Throw and Prepare. */
	void EnsureRopeInitialized();

#if WITH_GAMEPLAY_DEBUGGER
	// Fill the centerline, Wrapped and collider fields shared by every view into the snapshot, when this rope
	// is the debug capture target (called from FinalizeSimFrame).
	/** The header summary is always filled; the remaining sections only when CaptureMask asks for them, so a disabled view costs nothing to collect. */
	void FillDebugSnapshot(FRopeDebugSnapshot& Snapshot, ERopeDebugCapture CaptureMask) const;
#endif

	//~ Throw -----------------------------------------------------------------------
	/** Build a GuaranteedWrap preview from a context that has already been through ResolveThrowContext.
	 *  The internal entry point that lets ThrowWithContext's success and failure paths share one resolution. */
	bool BuildPreparedWrappingPreviewFromResolvedContext(const FRopeThrowContext& ResolvedThrowContext,
		FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason) const;

	// A throw starts by running the four helpers below in the order they are listed; StartFreshThrow is
	// orchestration and nothing else.
	void StartFreshThrow(const FRopeThrowContext& ThrowContext);

	/** 1) Clear the way for any throw: release an active wrap with a normal release notification, drop the
	 *  bridges and the phase transients, and start fresh without waiting out a cooldown. */
	void ResetStateForNewThrow();

	/** Set up the GuidedThrow state and the start node pin, shared by the prepared and open-space paths. */
	bool BeginGuidedThrowState(FRopePreparedThrowPreview&& Prepared, bool bFreeThrow);

	/** 2) Reset the chain: pin the hand (node 0) at the origin, zero every node's velocity (Prev = Pos), and bump the GPU resident buffer's reseed generation. */
	void ResetChainForThrow(const FVector& HandOrigin);

	/** 3) Start the whip swing: assemble the swing basis and inherited velocity from the resolved throw, then activate WhipGuide and snap it to T = 0. */
	void BeginWhipSwingFromThrow(const FRopeThrowContext& ResolvedThrow);

	/** 4) Inject the throw velocity, Verlet-style, by pushing PrevPositions back along the aim direction.
	 *  (Verlet velocity is (Pos - Prev)/dt, so moving Prev alone injects velocity without moving anything.)
	 *  It reads the resolved aim direction from WhipGuide.GetAimDir, so it must run after step 3. */
	void InjectThrowVelocityIntoVerlet(const FRopeThrowContext& ResolvedThrow);

	/** Advance GuidedThrow by one frame: move the nodes onto the preview centerline with the solver off. */
	void UpdateGuidedThrow(float DeltaTime);

	/** On GuidedThrow completing, turn the prepared anchors into an FRopeWrapState and commit straight to Wrapped. */
	void FinishGuidedThrow();

	/** The single point that raises Captured (native hook, then Blueprint delegate). Shared by the Flight
	 *  capture and the GuaranteedWrap reach — an inline broadcast per path leaves one side silent without
	 *  anyone noticing, which is exactly what happened to the guaranteed path once. */
	void DispatchCaptured(FName Bone);

	/** Throw into open space with no target: start an arc GuidedThrow toward EndpointWorld, returning to Free if it lands without catching. */
	bool StartFreeGuidedThrow(const FRopeThrowContext& ThrowContext, const FVector& EndpointWorld);

	/** The endpoint for a throw into open space: the ray end at rope length, pulled back to just in front
	 *  of the surface when world geometry blocks the path. The clamp is what keeps the rope above the
	 *  floor — GuidedThrow runs no solver, so an endpoint below it is replayed as a straight pass through.
	 *  Both open-space throw sites, the direct Blueprint one and the aim-miss one, resolve it here so they
	 *  cannot drift apart. */
	FVector ResolveFreeThrowEndpoint(const FRopeThrowContext& ResolvedThrow) const;

	/**
	 * Whether opaque world geometry, meaning a wall or a floor, blocks the segment, and the single place
	 * the engine trace behind it is issued. It settles two things:
	 *  - A target behind the blocking point is not aimable, because what cannot be seen cannot be aimed at.
	 *    The aiming HUD shows it as blocked instead of as a target.
	 *  - A throw with no target, that is into open space, has its endpoint pulled back to just in front of
	 *    the blocking surface. GuidedThrow replays node positions with the solver switched off, so without
	 *    the clamp the rope travels straight through the floor to a point underneath it.
	 * It uses an engine trace rather than the rope's own collider pool, because the static world provider
	 * extracts simple collision only and therefore never sees landscapes or floors that carry complex
	 * collision alone. At most one trace per aim resolve.
	 * Returns false when there is no world, as in a unit test, or nothing blocks. The rope's own actor is
	 * ignored, so the thrower's body and its tip mesh never count as a wall.
	 */
	bool TraceWorldAimBlocker(const FVector& Start, const FVector& End,
		FVector& OutBlockPoint, float& OutDistance) const;

	FVector ComputeThrowInheritedVelocity(const FRopeThrowContext& ThrowContext) const;

	/** Throw-strength gate: resolve Context.ThrowSpeed, falling back to ThrowParams.ThrowSpeed when it is 0.
	 *  A positive result (≥ 1 cm/s) is written to OutThrowSpeed and returns true; anything else logs a warning
	 *  and returns false. **Called before any state changes.** */
	bool TryResolveValidThrowSpeed(const FRopeThrowContext& Context, float& OutThrowSpeed) const;

	/** Build the config snapshot WhipGuide receives, from the Rope|Whip properties. */
	FRopeWhipGuide::FConfig MakeWhipGuideConfig() const;

	/** Weight of the throw impulse along the tail (smoothly 0 → 1 from FirstTailNode to the last node). */
	float TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const;

	//~ Flight ----------------------------------------------------------------------
	// The detection pipeline itself is FRopeFlightContactDetector (static, no UObject dependency). What stays
	// here is the assembly that needs UObject context.

	/** Parameter snapshot for the detector: WrapConfig, the tube radius, the component's forward, the substep dt
	 *  and the frame dt. SubstepDeltaTime is the FixedDt derived from SolverConfig.Substeps, used to turn
	 *  SurfaceVelocity (cm/s) into a displacement for the relative-motion test; FrameDeltaTime converts a
	 *  substep displacement into a frame one when extrapolating a predicted contact. */
	FRopeFlightContactDetector::FParams MakeFlightDetectParams(float DeltaTime) const;

	// Finalize's Flight block runs the helpers below in the order they are listed:
	// 1) build candidates → 2) evaluate the capture and transition → 3) observe (stats and debugger, read-only
	// consumption kept apart from the decision).

	/** Apply the CanWrapTarget gate to a candidate list, dropping barred targets. Flight's build and the
	 *  Contacting re-gather share this one filter, so tightening the condition moves both phases together. */
	void RemoveNonWrappableCandidates(TArray<FRopeContactCandidate>& Candidates) const;

	/** 1) Select or build this frame's Flight candidates. GPU results are consumed straight out of the SimFrame
	 *  array without a copy; the CPU fallback fills ContactCandidateScratch in actual → predicted →
	 *  relative-motion order. */
	TArray<FRopeContactCandidate>& GetOrBuildFlightContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams);

	/** Build candidates for the CPU Flight fallback only. The caller resets OutCandidates and NextGuideTargetScratch first. */
	void BuildCpuFlightContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeContactCandidate>& OutCandidates);

	/**
	 * Synchronous top-up for an AssistedJudged aim lock: query the bone collider the aim locked onto directly,
	 * along the CPU whip's real path, independent of the asynchronous GPU result. That covers a readback lost
	 * on an intermediate frame, and a nearer bone on the same mesh occluding the locked one. In Flight it
	 * includes the predicted path as well; in Contacting it stays actual-only.
	 * An ordinary full-simulation or unaimed flight pays nothing extra for this.
	 */
	void AddSynchronousAssistedAimContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeContactCandidate>& InOutCandidates);

	/**
	 * Synchronous top-up for an ordinary whip-guided flight: sweep the guide path the CPU already holds
	 * (previous -> current targets) against every wrappable collider, so a crossing that lasted less than one
	 * frame still lands as a real, same-frame Actual contact. Without it, a low frame rate lets the guided tip
	 * cross a thin isolated target — a pillar, a lever — entirely between two end-of-frame poses: the GPU
	 * detector's post-solve difference sees only the last substep, prediction alone is barred from capturing,
	 * and the throw tunnels. The assisted aim keeps its narrowed exact-primary probe; this is its
	 * full-simulation counterpart.
	 */
	void AddSynchronousWhipGuidedContactCandidates(
		const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeContactCandidate>& InOutCandidates);

	/** 2a) Fold the candidates into one frame-local evaluation, shared by the capture decision and the observation. */
	FRopeFlightCaptureEvaluation EvaluateFlightCapture(const TArray<FRopeContactCandidate>& Candidates,
		const FRopeFlightContactDetector::FParams& DetectParams) const;

	/** 2b) Apply the evaluation to game state: on a capture, move the tracker into ContactTracker and go to
	 *  Contacting; otherwise end the whip and advance the no-contact timer. Returns whether it captured. */
	bool ApplyFlightCaptureEvaluation(float DeltaTime, const TArray<FRopeContactCandidate>& Candidates,
		FRopeFlightCaptureEvaluation& Evaluation);

	/** 3) Observation: stat counters, collected only when stats are on, plus the debugger snapshot when
	 *  OutSnapshot is non-null, which only happens for the rope the debugger is targeting. Every read-only
	 *  consumer that takes no part in the decision belongs here, which is what keeps debug and stat code out
	 *  of FinalizeSimFrame's body. */
	void RecordFlightObservation(const FRopeFlightContactDetector::FParams& DetectParams,
		const TArray<FRopeContactCandidate>& Candidates, const FRopeContactTracker& FrameTracker,
		bool bShouldCapture, FRopeDebugSnapshot* OutSnapshot);

#if WITH_GAMEPLAY_DEBUGGER
	/** 3, debugger only: per-node detection input and decision visualization for the targeted rope. It queries
	 *  the detector separately from the bone pipeline — deliberate duplication, paid for by one rope. */
	void GatherFlightNodeDebug(const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeFlightNodeDebug>& OutNodeDebug) const;
#endif

	/** On a confirmed capture, take ownership of the evaluated tracker and set up Contacting
	 *  (PendingWrapSeed, CaptureTravelFrame, timers). */
	void BuildContactingState(FRopeContactTracker&& EvaluatedTracker,
		const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);

	//~ Contacting ------------------------------------------------------------------
	// Re-gather actual contacts each frame and advance the tracker's dwell: sustained contact goes to Wrapping,
	// lost contact dismisses back to Flight, and a dwell that never reaches the threshold times out to Flight
	// as a safety net. The decision reads total progress, not the tracker dwell, which resets whenever the
	// dominant bone changes.
	void UpdateContacting(float DeltaTime);

	bool ShouldDismissContacting() const;

	bool ShouldStartWrapping() const;

	FRopeWrapState BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const;

	/**
	 * Build the seed latch and anchor for one (Bone, Mesh) target — shared by the dominant target and, when the
	 * seed is multiplexed, the secondary ones.
	 * OutLatch is always filled. It returns true when a surface frame was obtained from the contact candidates
	 * and an anchor was built. OutMesh falls back to the candidate's mesh when the tracker has none, and is
	 * null when neither does.
	 */
	bool BuildSeedLatchForTarget(const TArray<FRopeContactCandidate>& Candidates,
		FName Bone, const USceneComponent* TrackedMesh, int32 NodeIndex, float RopeDistance,
		FRopeLatchNode& OutLatch, FRopeSurfaceAnchor& OutAnchor, const USceneComponent*& OutMesh) const;

	//~ Wrapping --------------------------------------------------------------------
	// The Wrapping phase's real work — path build, front motion, mass mask — is FRopeWrappingPhase
	// (the WrappingPhase member). What stays here is the orchestration that drives transitions and events.

	void StartWrappingFromContacting();

	void UpdateWrapping(float DeltaTime);

	/** The calling context handed to WrappingPhase: WrapConfig, the collider snapshot, the tube radius and the log name. */
	FRopeWrappingPhase::FContext MakeWrappingContext() const;

	/**
	 * Storage behind the collider list MakeWrappingContext hands over — only those that passed the
	 * CanWrapTarget gate. (FContext holds the array *by reference*, so it needs storage that outlives the call.)
	 * This is the gate that keeps a barred target from surfacing as a surface or attribution candidate during
	 * the wrap path build. On a rope that does not override the gate its contents match FrameColliders, so
	 * behaviour is unchanged.
	 */
	mutable TArray<IRopeCollider*> WrappableColliders;

	void CommitWrapping();

	/** Assemble the wrap event payload from the commit seed and the decision values, shared by NotifyWrapped and OnRopeWrapped. */
	FRopeWrappedEventInfo MakeWrappedEventInfo(const FRopeWrapState& Seed, float AngleDeg, float CoverageDeg) const;

	/** The single broadcast for a wrap taking hold: native hook, per-instance Blueprint delegate, and the subsystem's central signal. */
	void DispatchWrapped(const FRopeWrappedEventInfo& Info);

	/** The single broadcast for a release. The per-instance pair (NotifyReleased, then OnRopeReleased) always
	 *  fires, so every engagement is closed — including a Contacting or Wrapping abort, and a target being
	 *  destroyed. An engagement is opened by Captured, by Wrapped, **or by an aimed GuaranteedWrap throw**
	 *  (a successful ThrowWithPreparedPreview, which raises no start event of its own). **A GuaranteedWrap
	 *  throw into open space (bFreeThrow) has no target, opens nothing, and therefore closes nothing** — it
	 *  simply lands in Free. The central OnAnyRopeReleased fires **only for a committed wrap** (bWasWrapped):
	 *  raising it on an abort before commit would restore a ragdolled target that another rope still has
	 *  wrapped. WrappedMesh is the central signal's payload, and is null before commit. */
	void DispatchReleased(const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason, bool bWasWrapped);

	/**
	 * Release notifications received while a Wrapped notification is still running are queued here, to **keep
	 * them in order**. If a handler calls ReleaseWrap() from inside the notification, the release notification
	 * would nest and finish first, so subscribers would see Released before Wrapped — and a ragdoll target
	 * would discard the recovery, then take the Wrapped and stay down for good. Deferring the release
	 * notification until the Wrapped one has finished guarantees the order is **always Wrapped, then Released**.
	 * State changes (ReleaseWrap itself) are not deferred; only the notification is.
	 */
	struct FDeferredReleaseNotice
	{
		TWeakObjectPtr<USceneComponent> WrappedMesh;
		FName Bone = NAME_None;
		ERopeReleaseReason Reason = ERopeReleaseReason::Manual;
		bool bWasWrapped = false;
	};

	/** Nesting depth of DispatchWrapped; above 0, release notifications are queued. */
	int32 WrappedDispatchDepth = 0;
	TArray<FDeferredReleaseNotice> DeferredReleaseNotices;

	/** Send the queued release notifications in order (called only once the Wrapped notification is fully finished). */
	void FlushDeferredReleaseNotices();

	void AbortWrapping(ERopeReleaseReason Reason);

	/** Shared finalization for an interrupted GuaranteedWrap presentation: transition to Releasing, drop the
	 *  transient state, start the cooldown and raise the release event. Only an aimed throw raises an event —
	 *  an open-space throw (bFreeThrow) opened no engagement, so there is nothing to close. */
	void AbortGuidedThrow(ERopeReleaseReason Reason, const TCHAR* ReasonLog);

	//~ Wrapped ---------------------------------------------------------------------
	// Prepare's Wrapped case runs the four helpers below in the order they are listed.

	/** 1) Follow the bone: Hold (reposition on the skinned bone, with no velocity injected) plus the mass mask.
	 *  Returns false once the target mesh is lost and the rope has been released as Broken — the caller ends
	 *  the frame there. */
	bool HoldWrappedNodesToBone(float DeltaTime);

	/** Composes PullObservationSim: this frame's hand pin plus the override frame applied on top of Sim,
	 *  so the observations below read the current frame's boundary rather than the previous solved pose.
	 *  Shared by the Wrapping and Wrapped branches of Prepare. */
	void ComposePullObservationSim();

	/** The hand-side (minimum-node) anchor of the in-progress wrap, chosen across FRopeWrappingState's
	 *  anchor sets; null while nothing is bound. The movement constraint and the Wrapping-phase pull
	 *  observation share it so both resolve the same material boundary. */
	const FRopeSurfaceAnchor* FindWrappingHandSideAnchor() const;

	/** 2) Compute the observations: the authoritative constraint tension, the pull sample (ComputePull), and
	 *  two stages of smoothing (a scalar EMA, then a direction EMA). Traction (3), the release check (4), the
	 *  debugger and Blueprint all read this shared result. Despite the name it also runs every Wrapping
	 *  frame — sample, taut latch and overshoot come from the wrapping-state hand-side anchor there, so the
	 *  wielder's tether consumers engage before the commit; steps 3 and 4 stay Wrapped-only. */
	void UpdateWrappedPullSample(float DeltaTime, const FRopeSimState& ObservationSim);

	/** 3) Apply traction: the tether (λ impulse constraint plus, for a ragdoll, the physics constraint) and the active pull (constant force while taut, or climb-in). */
	void ApplyWrappedTraction(float DeltaTime);

	/** 4) Auto-release check: tension over threshold (TensionRelease*) or distance over it (DistanceReleaseSlack,
	 *  reading the overshoot the tether updated in step 3). Returns true if it released, and the caller then
	 *  skips the solve. */
	bool CheckWrappedAutoRelease(float DeltaTime);

	/** Mass mask for a wrap: InvMass = 0 on latch and anchor nodes, 1 elsewhere, so the solver moves only the free span. */
	void ApplyWrappedMassMask(bool bResetDynamicNodeVelocity = false);
	// The whole mask is rebuilt only when the topology or the bindings change. Each frame's hold updates just the pinned nodes' positions and InvMass.
	bool bWrappedMassMaskDirty = true;

	/** Register each run WrappingPhase has newly computed as a bridge, once, and activate it when the front reaches it. */
	void UpdateWrappingKinematicVirtualBridges(const TArray<FRopeVirtualBridgeRun>& Runs,
		const TArray<FRopeSurfaceAnchor>& Anchors, float FrontDistance);

	/** Re-verify the existing bridges against the final commit anchors and activate them. Does not rebuild them. */
	bool FinalizeKinematicVirtualBridges(const TArray<FRopeVirtualBridgeRun>& Runs,
		const TArray<FRopeSurfaceAnchor>& CommitAnchors);

	/** Space the bridge nodes evenly between their two anchors' current world positions, as a hard kinematic override. */
	void HoldKinematicVirtualBridges();

	/** Drop the previous bridge bindings on release, on a re-throw, or when entering a non-composite path. */
	void ResetKinematicVirtualBridges();

	/** Hand the active bridge nodes back to the solver: restore their mass, zero their velocity, and drop the bindings and scan state. */
	void ReleaseKinematicVirtualBridgesToSolver();

};
