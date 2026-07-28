// Copyright Epic Games, Inc. All Rights Reserved.
//
// Single UE integration point (Facade). Sim state (FRopeSimState) and solver, logic F-classes for each phase
// (WhipGuide/WrappingPhase/WrapController) as the value, branching physics and logic
// Roll the phase state machine (ERopePhase). The actual operation of each phase is in the Logic/ class,
// In this component, only the orchestration that determines transition and event broadcasting remains.
// Attach it to the character and use it with Throw().

#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "Core/RopeLengthConstraintState.h"
#include "Core/RopeMovementConstraint.h"
#include "Core/RopeTypes.h"
#include "Core/RopeSimFrameIO.h"
#include "Core/RopePullDriveState.h"
// FRopeAimRayHitResult/FRopeAimRayThrowRequest + aiming logic/state.
#include "Logic/RopeAimTargeting.h"
// Slip + Distance LOD (solve throttling).
#include "Logic/RopeSolverThrottle.h"
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
// Wrap target abstraction (Decision 0): Generalize the wrap target mesh to USceneComponent.
class USceneComponent;
class FRegisterComponentContext;
struct FRopeDebugSnapshot;
// Debugger per-node Flight visualization item (Debug/RopeDebugSnapshot.h).
struct FRopeFlightNodeDebug;
// Debugger capture scope bit (Debug/RopeDebugSnapshot.h) — Passed to the capture side to only collect views that are turned on.
enum class ERopeDebugCapture : uint8;

// The Wrapped establishment event was expanded from a single bone name to a structure payload (decision G at the meeting on 2026-07-13 —
// latching/decision value/multiple bones. Existing BP bindings require reconnection, clean break approved).
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnWrapped, const FRopeWrappedEventInfo&, Info);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnCaptured, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnReleased, FName, Bone, ERopeReleaseReason, Reason);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnPhaseChanged, ERopePhase, OldPhase, ERopePhase, NewPhase);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnPresetApplied, const URopePreset*, Preset);

// (FRopeAimRayHitResult / FRopeAimRayThrowRequest moved to Logic/RopeAimTargeting.h — still exposed as include above.)

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeComponent : public UMeshComponent
{
	GENERATED_BODY()

	// The subsystem sets Sim/SolverConfig/Phase/WhipGuide + SimFrame (frame contract bundle —
	// (see FRopeSimFrameIO comment) is accessed directly (including GPU placement solve; CPU path uses SolveSimFrame).
	friend class URopeSimSubsystem;
	// PrePhysics movement authority: the Wielder consumes the CPU constraint and primes the
	// physical target tether before Chaos without exposing the mutation API to general callers.
	friend class URopeWielderComponent;

#if WITH_DEV_AUTOMATION_TESTS
	// test seam: Minimum approach for ApplyPreset phase gate negative test (RopePresetTests) to force SetPhase.
	friend struct FRopePresetTestSeam;
	// test seam: Minimal approach to verify anchor invariant of Contacting seed and synthetic latch fallback entry.
	friend struct FRopeWrappingFallbackTestSeam;
	// test seam: Minimal approach to verify virtual bridge lifetime and GuidedThrow common entry state.
	friend struct FRopeComponentRefactorTestSeam;
	// test seam: A minimal approach to reproducing the Wielder input/pull lifecycle and self-wrap check without worlds.
	friend struct FRopeWielderComponentTestSeam;
#endif

public:
	URopeComponent();

	//~ Setup -------------------------------------------------------

	// This rope's top contract — this value determines what it guarantees, its aiming/preview status, and whether it uses a check gateway.
	// One decides. **source of truth is rope**: Wielder's aiming/throwing method is derived from here, and BP direct/AI
	// Completed with this value alone, without Wielder. See the ERopeWrapResolveMode enumerator comment for mode-specific contracts.

	/** Wrapping resolution (reaching) mode — what is guaranteed from throwing to latching.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (DisplayName = "Wrap Mode"))
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	//~ Tip(tip attachment — spearhead/harpoon/chu) ----------------------------------------
	// A display-only StaticMesh attached to the rope Free end(GetNodeCount()-1). No mass·collision (reflect tip mass solver
	// Not — confirmed 2026-07-14). **latching is a common function regardless of model** (2026-07-17): bUseTipMesh with one
	// On, lifespan is unified for all modes BeginPlay to EndPlay — destroy on EndPlay (only if we spawn),
	// External components (found by tags) are not destroyed.
	// The only exception is socket correction (bUseTipMeshSockets) — the concept of aligning the head to the insertion point exists only in ③.
	// The single source of truth for the active condition is IsTipSocketPlacementActive(), and existence confirmation/reading is handled by HasTipSocket/ReadTipSocketLocal.
	//
	// fallback (when socket compensation is off, not in ③, or there is no head socket): The mesh origin is at the end node of the rope, and the
	// Placed in the last segment direction — does not compensate if the tip is buried in the target (intended no compensation). Even in Wrapped
	// Since the end node is a bone-local anchor, the animation continues to follow, and the segment is guided rather than just a rotation-only frozen position.
	// If there is only a head and no tail, the rope is connected to the mesh origin.
	//
	// NOTE: The /** */ below becomes an editor tooltip — keep it short, one line, and write the details in this block.

	/** Use the tip attachment. When turned off, all Tip settings below are ignored and it becomes a regular rope without a tip.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip")
	bool bUseTipMesh = false;

	/** StaticMesh to spawn on the tip. If it's empty and you can't find it with tags, no tip.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Mesh"))
	TObjectPtr<UStaticMesh> TipMesh = nullptr;

	/** Reuse the StaticMeshComponent of this tag already attached to the Owner as a tip (takes precedence over spawn, does not destroy).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Component Tag"))
	FName TipMeshComponentTag = NAME_None;

	/** Tip placement offset (based on tip node frame).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Relative Transform"))
	FTransform TipMeshRelativeTransform = FTransform::Identity;

	// Tip (spearhead/harpoon/weight) is a display-only mesh that follows the Free end every frame, so if the collision body is turned on, the character
	// Rope behavior bounces when it collides with a capsule/world or interferes with the rope's collision query — **Default off**. If we turn it on, we
	// The spawned tip captures the entire collision (QueryAndPhysics), and the external component reused as a tag captures the authored collision settings (value at the time of acquisition).
	// has. The point of application is securing tips (EnsureTipMesh) and editor/PIE editing (PostEditChangeProperty).

	/** Tip Turn on mesh collision. **Default off** — Prevents collisions of display-only tips from interfering with the rope/character.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Enable Collision"))
	bool bTipMeshCollision = false;

	// OFF = An extension point that tells the game code to directly drive Free placement with GetTipMeshComponent() (ropes are left untouched).
	// Phases other than Free (Flight/GuidedThrow/Wrapping/Wrapped/Releasing/Loaded) always follow regardless of this value.

	/** In Free, align the tip with the end of the rope every frame. When turned off, the tip will not be touched during Free.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Sync While Free"))
	bool bSyncTipMeshOnFree = true;

	// Use the default GetLoadedTipTransform() implementation — override its virtual to change the placement convention.

	/** Owner skeletal mesh socket to attach tip to in Loaded(Loaded). If not, component (hand) transform.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	FName LoadedHandSocket = NAME_None;

	// While Loaded this offset is composed onto the hand socket frame by MakeLoadedTipBaseWorld(), which
	// drives *both* the tip mesh and the pinned free-end node - route every Loaded placement through that
	// helper or the spear and the rope end drift apart. Distinct from TipMeshRelativeTransform, which
	// applies in every phase and cancels out along the socket/pierce paths, so tuning the grip here
	// cannot disturb the embed alignment.

	/** Offset applied to the tip while Loaded, expressed in the hand socket's frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh", DisplayName = "Loaded Relative Transform"))
	FTransform LoadedTipRelativeTransform = FTransform::Identity;

	// On = Invert the mesh origin so that the tail is at the end of the rope and the head is at the insertion point, and freeze the posture as bone-local.
	// Follows the target animation. Off = Do not read the socket at all (fallback above). ①② is meaningless.

	/** ③(Guaranteed) only — Precisely place the tip with the Head/Tail socket. When turned off, the mesh origin is placed at the end of the rope.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Use Sockets"))
	bool bUseTipMeshSockets = false;

	/** Head socket — The pointed end of the tip. This socket is embedded in the aiming hit point. If not, the above correction is disabled.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Tip Socket"))
	FName TipSocketName = NAME_None;

	/** Tail socket — The point where the rope Free end will be connected. If not, connect to mesh origin.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Rope Socket"))
	FName TipRopeSocketName = NAME_None;

	// The initialization-only values below (NumParticles/RopeLength/MinRopeLength) are only consumed at InitRope time —
	// Runtime writes are invalid until reinitialization, so BlueprintReadOnly (trap avoidance). Runtime length change is
	// Use SetRopeLength/SetReelRate.

	// ClampMax 512 = FRopeGPUSolver::MaxNodes(GPU solver thread group cap). CPU solve+tube quietly when exceeded
	// becomes a fallback and is blocked in the editor due to a performance cliff + no authoring signal (BP/code path is hard clamped by InitRope).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "2", ClampMax = "512", DisplayName = "Node Count"))
	int32 NumParticles = 72;

	/** Initial (maximum) rope length (cm). Runtime current length is GetCurrentRopeLength/SetRopeLength.*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 600.0f;

	/** Minimum length (cm) that can be reduced by Wrapping(reel-in). RopeLength (initial) is cap.*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "10.0", Units = "cm"))
	float MinRopeLength = 100.0f;

	/** Default reel velocity (cm/s) used by rewrapping/unwrapping input. The length change is in the rope domain, so it lives here.
	 *  (2026-07-13 surface Audit A-2 — Moved from Wielder; Wielder Reel action calls SetReelRate with this value).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "0.0", Units = "cm/s"))
	float ReelSpeed = 150.0f;

	// rope tube visibility during Loaded(Loaded). This is the value consumed by the default OnEnterLoaded() implementation, so override that hook.
	// If you enter your own presentation, this value is ignored. Off = Presentation with only the window visible in the hand socket, On = Presentation between hand and window
	// The rope appears as is (even in Loaded, the solve turns and the rope sags).
	// Direct assignment is not reflected when Loaded (visibility is applied at the Loaded entry edge), so BlueprintReadOnly +
	// Use SetShowRopeWhenLoaded/ToggleShowRopeWhenLoaded setters (same reason as RopeMaterial).

	/** In Loaded (Loaded) state, the rope tube is shown. ③(GuaranteedWrap) Dedicated presentation switch.*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope",
		meta = (EditCondition = "ResolveMode == ERopeWrapResolveMode::GuaranteedWrap"))
	bool bShowRopeWhenLoaded = false;

	// All domain-specific setting structures below are exposed as ShowOnlyInnerProperties — Category in the Details panel
	// The fields are expanded immediately below the header (Rope|Solver / Rope|Throw / …), so "Category → Structure name →
	// is edited in one step without double expansion of the field. BP/serialization is not affected (the structure remains as a single
	// BlueprintReadWrite variable). The detailed categories of each field (Rope|Solver|Scaling, etc.) are maintained inside the structure.

	/** XPBD solver tuning (Free/Flight physics).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ShowOnlyInnerProperties))
	FRopeSolverConfig SolverConfig;

	/** throwing/launch parameters (Flight entry).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ShowOnlyInnerProperties))
	FRopeThrowParams ThrowParams;

	// ③ (GuaranteedWrap) is preview-based, so there is no Wrapping check/path build → WrapConfig is grayed out in ③.
	// When EditCondition is placed on a struct *member* (directly under component, you can see ResolveMode), it is set to ShowOnlyInnerProperties.
	// edit-const is propagated to promoted inline children and they become gray together (unlike category hiding, in the property node tree
	// Behavior — independent of display promotion). The value is preserved and only edits are prevented (EditConditionHides default false = not hidden).

	/** physics → logic (wrap) handoff — capture check threshold and *establish* (path build/check/commit) tuning.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap",
		meta = (ShowOnlyInnerProperties, EditCondition = "ResolveMode != ERopeWrapResolveMode::GuaranteedWrap"))
	FRopeWrapConfig WrapConfig;

	/** Wrapped *after* (retain/pull/unwind) tuning — Post-Wrap domain separate from check(WrapConfig)
	 *  (2026-07-13 surface audit B-1; Design Note 01 Common regardless of domain, reach mode, and latching model).*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ShowOnlyInnerProperties))
	FRopeHoldConfig HoldConfig;

	//~ Collision (collision domain) -----------------------------------------------
	// Clustered the scattered collision-related switches into one place (2026-07-13 surface audit CL-4). The radius itself is
	// in SolverConfig.CollisionRadius / WrapConfig.ContactQueryRadius, 0 (default)=auto — below
	// GetEffective* helper is derived from render Radius (automatically matching 3 types of radii).

	/**
	 * By default, the rope collides with all collider providers in the world, excluding its owner's.
	 * — To prevent the stretched rope from becoming tangled in the thrower's limbs when throwing. cross-actor wrap (grabbing another actor's body)
	 * It operates automatically because the actor is included in the “whole”.
	 *
	 * The exclusion scope is two-pronged:
	 *  - Skeleton/wrap target provider is **provider unit** (skip the owner's provider entirely).
	 *  - static world provider (URopeStaticBodyProvider) is **body unit** — source from shapes caught while browsing the world
	 *    Only removes the actor that is the owner (prevents tether proxy, tip mesh, held weapon, etc. from following the rope and pushing its own rope).
	 *    The world geometry of other actors such as floors/pillars remains as is.
	 *
	 * **Configurations where a rope is placed on a prop (when a URopeComponent is attached to a column, crane, or anchor actor) must have this value turned on** —
	 * If you turn it off, the collision of the pedestal is also excluded because it is owned by the owner, and the rope passes through the pedestal.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision",
		meta = (ToolTip = "끄면(기본) 자기 owner의 콜라이더를 제외합니다 — 정적 월드 provider가 잡은 owner 소유 셰이프(테더 프록시/팁/무기)도 바디 단위로 빠집니다. 로프를 기둥 등 프롭 액터에 붙였다면 켜세요(안 켜면 받침대를 통과).", DisplayName = "Collide With Owner"))
	bool bIncludeOwnerColliders = false;

	/**
	 * Push the rope on static world geometry (wall/floor) using the engine's Global Distance Field. GPU path (scene graph
	 * dispatch). Project requires Generate Mesh Distance Fields — silently no-op if GDF is invalid
	 * **Default On** (Wall/Floor penetration protection is safer; turn off to save on GDF on-demand builds).
	 * No bone attribution·surfacevelocity (static world wide-area push complement) — Not a replacement for per-bone SDF. While on
	 * The engine builds the GDF on demand. Push radius/friction shares CollisionRadius/Friction/TipFrictionScale.
	 * (Moved directly from SolverConfig to component — collision domain aggregation.)
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision", meta = (DisplayName = "Use World Distance Field"))
	bool bUseWorldGDF = true;


	/** solved solver collision radius: SolverConfig.CollisionRadius(0=auto → render Radius). Consumed at solve/GPU step boundaries.*/
	float GetEffectiveCollisionRadius() const
	{
		return SolverConfig.CollisionRadius > 0.0f ? SolverConfig.CollisionRadius : Radius;
	}

	/** Maximum elongation multiplier to use for Flight/Wrapping and commit frames in FullSimulation/Assisted.
	 *  rigid authoritative Wrapped(TetherCompliance=0) also maintains 1.0, and only compliant hold returns to the setting value.*/
	float GetEffectiveMaxStretchRatio() const;

	/** Interpreted contact query radius: WrapConfig.ContactQueryRadius(0=auto → render Radius × 1.5). Consume at detection/wrap path boundary.*/
	float GetEffectiveContactQueryRadius() const
	{
		return WrapConfig.ContactQueryRadius > 0.0f ? WrapConfig.ContactQueryRadius : Radius * 1.5f;
	}

	/** Interpreted taut slack tolerance ratio: geometric interpolation of HoldConfig.TautSensitivity (0=lax~1=strict)
	 *  (0→0.09, 0.5→0.03 (old default), 1→0.01). Consumed by chord gate in RopeComponentTraction.cpp.*/
	float GetEffectiveTautSlackRatio() const
	{
		return 0.09f * FMath::Pow(0.01f / 0.09f, FMath::Clamp(HoldConfig.TautSensitivity, 0.0f, 1.0f));
	}

	/** Analyzed tautology maximum allowable sag (cm): Geometric interpolation of HoldConfig.TautSensitivity
	 *  (0→80, 0.5→20 (old default), 1→5). Consumed by sag gate in RopeComponentTraction.cpp.*/
	float GetEffectiveTautMaxSag() const
	{
		return 80.0f * FMath::Pow(5.0f / 80.0f, FMath::Clamp(HoldConfig.TautSensitivity, 0.0f, 1.0f));
	}

	//~ Whip (throwing swing settings) -----------------------------------------------
	/** Tuning of the whip swing in the early stages of throwing. Runtime state is owned by WhipGuide, and when called
	 *  Create and pass a snapshot with MakeWhipGuideConfig().*/
	// ③ (GuaranteedWrap) uses GuidedThrow arch instead of whip Flight, so whip tuning is meaningless → Grayed out in ③
	// (struct-member EditCondition method like WrapConfig — see WrapConfig comment above).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip",
		meta = (ShowOnlyInnerProperties, EditCondition = "ResolveMode != ERopeWrapResolveMode::GuaranteedWrap"))
	FRopeWhipConfig WhipConfig;

	/** Current whip swing elapsed time (s). 0 when swing is disabled.*/
	UFUNCTION(BlueprintPure, Category = "Rope|Whip")
	float GetWhipElapsed() const { return WhipGuide.GetElapsed(); }

	//~ Render(render) -------------------------------------------------------
	// The render values below (Radius/NumSides/TubeSmoothing*) are read once and solidified when creating a scene proxy — runtime writing
	// BlueprintReadOnly because it is not reflected until proxy regeneration (editor changes are reflected through render state regeneration).

	/** Visual tube radius (cm).*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (ClampMin = "0.1", Units = "cm", DisplayName = "Rope Radius"))
	float Radius = 2.0f;

	/** Number of sides of tube cross section. The higher it is, the more rounded it is.*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "3", ClampMax = "32", DisplayName = "Sides"))
	int32 NumSides = 8;

	/** render Tube smoothing: Number of Catmull-Rom subdivisions per segment (1=off). Leave the simulation node as is and only render the center line
	 *  Estimates curvature with neighboring nodes and smoothes it out (separated from physics — render only). Reason for default 1: interpolation ring is node
	 *  It can swell outside the polyline (especially the section that touches the wall), so if the nodes are tight, a straight line connection is more accurate.
	 *  Raise only the sparse ropes to make it look round, and reduce the overshoot with TubeSmoothingAlpha (centripetal).
	 *  If NumRings=(NumParticles-1)*Subdiv+1 exceeds the GPU tube ring cap, the proxy automatically lowers.*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "1", ClampMax = "8", DisplayName = "Smoothing Subdivisions"))
	int32 TubeSmoothingSubdiv = 1;

	/** render Catmull-Rom knot α for tube smoothing: 0=uniform, 0.5=centripetal (tangential overshoot in sharp corners↓ —
	 *  The middle ring of the section against the wall bulges less outside the wall), 1=chordal. CPU smoothing and GPU resident smoothing
	 *  The render is consistent by using the same value. If TubeSmoothingSubdiv=1, no effect.*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Smoothing Strength"))
	float TubeSmoothingAlpha = 0.5f;

	/** Material applied to rope tube. If not set, the engine default material is used.
	 *  Runtime replacement is with SetMaterial(0, M) — the scene proxy captures the material at creation time, so directly assigning
	 *  There is no MarkRenderStateDirty, so the replacement is not reflected until the next proxy regeneration.
	 *  BP's direct Set cannot be hooked, so it is not a BlueprintReadWrite — RopeLength is BlueprintReadOnly +
	 *  Same reason as SetRopeLength. Editor detail panel editing is handled by PostEditChangeProperty.*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (DisplayName = "Material"))
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	//~ API ---------------------------------------------------------------

	/**
	 * Apply the entire preset (URopePreset) — Copy (stamp) the value and reinitialize the rope. **Only in Free/Loaded**
	 * is established, other phases (flying or winding) return false and do not change anything.
	 * When applied: Sim reseed (InitRope) + render/MID reconfiguration + tip reacquisition + mode-phase matching (if ③, Loaded
	 * , was Loaded, but returns to Free when ①② is reached).
	 * Instance wiring values, such as TipMeshComponentTag, are maintained because they are outside the preset — the external tip captured by the tag is maintained.
	 * bUseTipMesh=false The preset does not hide it (instance responsibility). No replication (local stamp).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool ApplyPreset(const URopePreset* Preset);

	/** Fires a rope. ①② receives the initial tip velocity and becomes a physical Flight, and ③ is established only in Loaded and establishes a confirmed path.
	 *  Enters GuidedThrow following (the mode determines the path). The actual direction is ThrowParams.FrameMode.
	 *  Forward is the single source of truth. To specify direction directly, use ThrowWithContext(FRopeThrowContext).*/
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw();

	/** Extended throw entry point where Wielder calculates and passes origin/frame/velocity.*/
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowWithContext(const FRopeThrowContext& ThrowContext);

	/** **Actual sweep radius**, which interprets the request radius (0=unspecified) as the fallback protocol of this rope. Aiming visualization is a query and
	 *  Used to draw the same dimensions — 0 is the default, so in practice, a fallback almost always occurs.*/
	float GetAimRayEffectiveQueryRadius(float RequestedRadius) const;

	/** Register the world section to be inspected by the aim ray as the aiming collection region of the collider subsystem.*/
	void SetAimRayColliderQueryBounds(const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius);
	/** When the aim ray mode ends, the aiming collection region, snapshot, and pending/cache results are cleared together.*/
	void ClearAimRayColliderQueryBounds();
	/** Register HUD/preview request. It is interpreted immediately after normal subsystem collider gather and does not re-collect immediately.*/
	void QueueAimRayQuery(const FRopeAimRayThrowRequest& Request);
	/** Recent HUD/preview results confirmed at normal gather. Since the next Wielder tick is consumed, there is a delay of up to 1 frame.*/
	bool GetLatestAimRayQueryResult(FRopeAimRayQueryResult& OutResult) const;
	/** Legacy API for compatibility. It does not immediately recollect the provider, but switches to QueueAimRayQuery and always returns false.*/
	UE_DEPRECATED(5.7, "Use QueueAimRayQuery/GetLatestAimRayQueryResult. Immediate collider refresh was removed.")
	bool RefreshAimRayQueryColliders(const FRopeAimRayThrowRequest& Request);
	/** Aim request is interpreted as the current aiming collider list. If there is no hit, OutContext is a BaseContext fallback.*/
	bool ResolveAimRayThrowContext(const FRopeAimRayThrowRequest& Request, FRopeThrowContext& OutContext,
		FRopeAimRayHitResult* OutHit = nullptr, FRopeAimRayHitResult* OutBlockedHit = nullptr) const;
	/** Queue the request to confirm the actual throw immediately after collecting the latest collider.*/
	void QueueAimRayThrow(const FRopeAimRayThrowRequest& Request);

	//~ Wielder contract (C++ only) -----------------------------------------------
	// Entry point used by the URopeWielderComponent's aiming/GuaranteedWrap preview constraint flow. Not a general user API
	// BP Not Exposed — Usually not called directly in game code (only when attaching a Wielder or reimplementing the same contract).
	// The preview target is determined only by the aiming result (aim hit) — there is no alternative search around the throwing direction.

	/** Preview build for GuaranteedWrap. It returns not only the render centerline but also the contact/anchor required for actual GuidedThrow/Wrapped entry.*/
	bool BuildPreparedWrappingPreview(const FRopeThrowContext& ThrowContext, FRopePreparedThrowPreview& OutPrepared,
		FString* OutFailureReason = nullptr) const;

	/** Throw using Prepared preview as the authoritative path. Flight/Contacting Enter GuidedThrow without re-searching.*/
	bool ThrowWithPreparedPreview(const FRopePreparedThrowPreview& Prepared);

	/**
	 * Guaranteed The aim request at the moment of input is confirmed as a prepared path immediately after normal collider gather. Without a montage
	 * Execute immediately with bExecuteWhenReady=true, queue the montage path with false, then notify.
	 * Call RequestExecuteQueuedGuaranteedAimThrow. The order is OnPrepared → actual execution → OnResolved.
	 */
	bool QueueGuaranteedAimThrow(const FRopeAimRayThrowRequest& Request, bool bExecuteWhenReady);
	/** If the queue result is ready, it is executed immediately. If it is not yet gathered, it is displayed to be executed immediately after preparation.*/
	bool RequestExecuteQueuedGuaranteedAimThrow();
	/** Cancel Montage/Change Mode/EndPlay discards Guaranteed requests that have not yet been executed.*/
	void CancelQueuedGuaranteedAimThrow();

	/**
	 * Enters the throwing ready (Loaded/Loaded) state — the spear (tip) is held in the hand socket (rope tube symbol indicates
	 * bShowRopeWhenLoaded, default hidden). **③ Only**
	 * **Only valid in Free/Loaded** (otherwise no-op — cannot be Loaded while flying or plugged in).
	 * ③ Rope starts with Loaded in BeginPlay. Throwing is only possible in this state (CanThrowNow).
	 * Loaded Input binding is up to the user (calling this API).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void EnterLoaded();

	/** Sets the rope tube display during Loaded(Loaded). If it is Loaded, it is reflected immediately; in other phases,
	 *  Applies from the next Loaded entry (the visibility of the deployed state is not affected).*/
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetShowRopeWhenLoaded(bool bShow);

	/** Flips the Loaded rope display (to be attached to the entered key). Return value = value after flipping.*/
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool ToggleShowRopeWhenLoaded();

	/** Is it set to show the rope tube during Loaded?*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsShowRopeWhenLoaded() const { return bShowRopeWhenLoaded; }

	/** Does a throw occur on this rope now (mode × current phase). ③ is only true when Loaded, and ①② is always true.
	 *  This is a gate shared by throwing entry and aiming HUD. Game rules (stamina, etc.) are separate — Wielder's CanThrow().*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool CanThrowNow() const { return RopeWrapModes::CanThrowInPhase(ResolveMode, Phase); }

	/** Manually release the Contacting/Wrapping (Contacting/Wrapping/Wrapped) currently in progress (Releasing phase).*/
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ReleaseWrap();

	/**
	 * Rope cutting (external gameplay — slashing/damaging, etc.): In-progress grab/Wrapping with ERopeReleaseReason::Cut
	 * Forced release. The flow is the same as ReleaseWrap, but the reason is different so the game can react differently (rope destruction presentation, etc.)
	 * . Followed by a physical cut to split the rope itself into two pieces (requires simulating length change/splitting).
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
	 * segment(SegmentIndex = node i~i+1) tension. Force derived from the solver's XPBD distance λ (F=max(0,-λ)/h²,
	 * Relative units based on mass 1 node — gravity load of 1 hanging node ≈ 980). Only stretch is positive, slack/compression = 0.
	 * GPU-resident rope mirrors 1 to 2 frames of delay. Phases without solve (Contacting/Releasing) maintain the previous value.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetSegmentTension(int32 SegmentIndex) const;

	/**
	 * Maximum diagnostic tension among all XPBD segments. Visual solver/debug only, gameplay load,
	 * Use GetConstraintTension for Pull activation and automatic release.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetMaxTension() const;

	/**
	 * This frame pull data: first anchor direction and authoritative constraint tension on the hand side.
	 * Calculated every frame while Wrapped.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool GetPullSample(FVector& OutDirection, float& OutTension) const
	{
		OutDirection = PullDrive.LastPullSample.Direction;
		OutTension = GetConstraintTension();
		return PullDrive.LastPullSample.bValid;
	}

	/**
	 * Is the rope taut this frame? At the default threshold(0), it is a pure material-length geometry check,
	 * Only when ActivePullTautTension > 0, authoritative constraint tension is used as an additional load gate.
	 * XPBD SegmentTension does not participate in this check.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsPullTaut() const { return PullDrive.bPullTaut; }

	/**
	 * Is the chain geometrically tight before this frame? If there is a live hand/anchor material boundary, use it
	 * is used as the source of truth, and falls back to sag + chord hysteresis only in the legacy path. SegmentTension is visual
	 * This is a solver diagnostic value and is not a prerequisite for this gameplay state.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsChainTaut() const { return PullDrive.bChainTaut; }

	/** This frame's tether excess (cm): Straight line distance between hand and anchor - available rope length (0 if less than 0). While Wrapped
	 *  Calculated every frame (calculated even if tether is off). For game reactions such as wielder traction/ground departure check.*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetTetherOvershoot() const { return LengthConstraintState.LastViolation; }

	/** Share of tether target actually used this frame (shareT) [0..1]. Automatic (mass-based)/manual common final value —
	 *  If it is 1, the wielder's share is 0 (all targets), if it is 0, the entire amount is wielder. For wielder traction active check/debug.*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetEffectiveTetherTargetShare() const { return PullDrive.LastTargetShare; }

	/**
	 * This frame gameplay-authoritative material constraint tension = λ/dt(kg·cm/s²).
	 * Backends are mutually exclusive: the physical target is a Chaos constraint force, and the hard-projected Pawn is a Chaos constraint force.
	 * Reaction force of rejection motion before projection, legacy/custom path uses analytic solve.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetConstraintTension() const { return LengthConstraintState.GetTension(); }

	/** Backward-compatible name. New gameplay code should use GetConstraintTension. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetTetherTension() const { return GetConstraintTension(); }

	/**
	 * pullable check(source of truth of active Pull climb-in direction) — Pure function (UObject independent, unit testable).
	 * “Can be dragged” if target effectivemass EffMassTarget ≤ wielder effectivemass EffMassWielder. bPrev(just before sticky
	 * check, the mass on the other side must be MarginRatio(≥1) times larger (to prevent boundary flapping).
	 * Infinite mass (anchor) is transferred to +BIG_NUMBER (infinite target = cannot be attracted, infinite wielder = target can be attracted).
	 */
	static bool DecideTargetPullable(float EffMassTarget, float EffMassWielder, bool bPrev, float MarginRatio);

	/**
	 * Active Pull force setting — Wrapped + When the rope is taut, a *constant* force of this magnitude is applied every frame.
	 * is applied to the target (unrelated to tension → no feedback surge). 0 = stationary. Turns on during input hold and turns off when released
	 * Purpose (URopeWielderComponent's PullAction calls this). The character target is CharacterMovement.
	 * It is divided into mass and competes with ground friction, so the range felt is tens to hundreds of thousands.
	 * Check/gate tension with HoldConfig(bActivePullRequiresTaut/ActivePullTautTension) — IsPullTaut().
	 * If bIgnoreTautGate=true, this Pull is approved regardless of the config, ignoring tautology (per-call bypass —
	 * For the "Ignore tension" section of the animation pull window, passed by UAnimNotifyState_RopePull).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetActivePull(float Force, bool bIgnoreTautGate = false);

	/** Current (runtime) rope length (cm). Changes to rewrapping/unwrapping — default/cap is RopeLength.*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetCurrentRopeLength() const { return Sim.RopeLength; }

	/**
	 * Sets the rope length directly (immediate form of Wrapping/unwrapping). Clamp with [MinRopeLength, RopeLength(initial)].
	 * The number of nodes is maintained and the segment rest length changes uniformly — to the solver (same CPU/GPU) without reseeding.
	 * It is reflected from the next frame. If you shorten it while Wrapped, the available rope length decreases and the tether pulls the target.
	 * If there is no tether, tension rises (can be combined with TensionRelease).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetRopeLength(float NewLength);

	/**
	 * Wrapping velocity setting (cm/s). Positive = Wrapping (shortening), negative = unwinding (lengthening, to initial length), 0 = stationary.
	 * Applies to every frame in Free/Flight/Wrapped (Contacting/Wrapping/Releasing is temporarily on hold —
	 * path creation depends on SegmentLength). For input hold purposes (URopeWielderComponent's ReelIn/Out actions).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetReelRate(float CmPerSecond);

	/** Is it in sleep (stationary check — solve skip state in Free phase)?*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsSleeping() const { return Throttle.IsAsleep(); }

	/** The iteration multiplier for the current distance LOD (1=full quality). For debug/profile verification purposes.*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetSolverLODScale() const { return Throttle.GetSolverLODScale(); }

	/** Was this rope dispatched to the GPU in this frame? **Does not just mean physics solve** — subsystems
	 *  Load the solve frame and override-only frame (Wrapping/Releasing/GuidedThrow) equally on the GPU.
	 *  (bSolveThisFrame in TryBuildResidentStep || OverrideFrame.HasAny()). Therefore, with this value
	 *  It should not be read as “Solving GPU”, but should be combined with the two getters below. For debug verification.*/
	bool IsGpuSteppedThisFrame() const { return SimFrame.bGpuSteppedThisFrame; }

	/** Did this rope (regardless of CPU/GPU) actually perform physics solving steps this frame? Sleep·Contacting·Releasing·
	 *  Logic override-only frame is false. Combined with IsGpuSteppedThisFrame(), it selects the CPU fallback solve.
	 *  (WasSolvedThisFrame() && !IsGpuSteppedThisFrame()). For debug/profile.*/
	bool WasSolvedThisFrame() const { return SimFrame.bSolveThisFrame; }

	/** Did this frame's logic phase (Wrapping/Wrapped/Releasing/GuidedThrow, etc.) result in a node override?
	 *  = Frame where solve was not performed but the position was updated. Combined with the above two getters, the solve path is divided into 6 types:
	 *  SLEEP / GPU_SOLVE / GPU_OVERRIDE / CPU_SOLVE / CPU_OVERRIDE / IDLE. GPU path is
	 *  IsGpuSteppedThisFrame() alone reveals override, but to distinguish between override and idle in the CPU path,
	 *  This value is required. For debug/profile.*/
	bool HadLogicOverrideThisFrame() const { return SimFrame.OverrideFrame.HasAny(); }

	/** Name of the currently Wrapped bone (valid while Wrapped, otherwise None). Displayed so that it can be viewed without event parameters.*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	FName GetWrappedBoneName() const { return WrapController.State.BoneName; }

	/** The original component (skeletal/static shared) currently being wound. null if not Wrapped or the target is lost.*/
	const USceneComponent* GetWrappedComponent() const { return WrapController.State.Mesh.Get(); }

	/** The currently Wrapped skeletal mesh (valid while Wrapped, otherwise null). The target actor response continues with GetOwner().
	 *  Internal storage is now const USceneComponent weak (generalized over static wrap) — here it only returns a skeletal,
	 *  If it is a static target, it is null. The body needs Cast and is defined in .cpp (avoiding heavy header include).*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	USkeletalMeshComponent* GetWrappedMesh() const;

	/** Centerline node number (= NumParticles, after simulation initialization).*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	int32 GetNodeCount() const { return Sim.Num(); }

	/** World location of the centerline node (0=hand/anchor, GetNodeCount()-1=end). Out-of-range index is ZeroVector.
	 *  For BP consumption, such as attaching effects/sounds to the end of a rope — In C++, GetCenterlinePositions() is copy-Free.*/
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetNodePosition(int32 NodeIndex) const
	{
		return Sim.Positions.IsValidIndex(NodeIndex) ? Sim.Positions[NodeIndex] : FVector::ZeroVector;
	}

	const TArray<FVector>& GetCenterlinePositions() const { return Sim.Positions; }

	// Extension point for game code to directly drive Free placement when bSyncTipMeshOnFree=false — In that case
	// rope does not touch the transform of this component while it is Free.

	/** Tip attachment component (null if the tip is not used or is not secured).*/
	UFUNCTION(BlueprintPure, Category = "Rope|Tip")
	UStaticMeshComponent* GetTipMeshComponent() const { return TipMeshComponent; }

	//~ Events ----------------------------------------------------
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnWrapped OnRopeWrapped;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnCaptured OnRopeCaptured;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnReleased OnRopeReleased;

	/** All phase transition notifications (excluding same phase resets). For more detailed state integration (UI/SFX) than Wrapped/Captured/Released.
	 *  is broadcast during transition processing (inside SetPhase), so calls that change the rope state in the handler (ReleaseWrap, etc.)
	 *  Not supported — bind such responses to OnRopeWrapped/OnRopeReleased (firing after the transition has finished).*/
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnPhaseChanged OnRopePhaseChanged;

	/** Fired immediately after ApplyPreset success (not fired if rejected). Wielder on mode driven state (preview/tick) resynchronization
	 *  You can subscribe, and the game code can also be used for preset conversion reactions (UI updates, etc.).*/
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnPresetApplied OnPresetApplied;

private:
	/**
	 * One simulation frame is divided into three stages and URopeSimSubsystem runs (friend approach;
	 * component is not ticked directly and is not called by external game code, so it is private).
	 *
	 * Separation contract — Solve in the middle is divided into rope-to-rope parallel (CPU) or GPU dispatch, and is not an arbitrary classification.
	 * There is only one criterion for checking “which is which”: is the solution result required?
	 *  Prepare(GT): solve *input* production — advance pin target, calculate whip target, logic phase (Contacting/Wrapping/
	 *                 Wrapped/Releasing) processing + OverrideFrame calculation, bSolveThisFrame decision. The solve result is
	 *                 All unnecessary logic goes here (the last point where UObjects/events can be touched before solving).
	 *                 collider snapshots (FrameColliders) are collected centrally by the subsystem before this call.
	 *  Solve (Parallel): POD(Sim) + const collider only — when bSolveThisFrame(Free/Flight/Wrapping/Wrapped)
	 *                 Solver. Step. Prohibit UObject/Event/Transition (thread safe boundary). Wrapped means the latch node is
	 *                 Since InvMass=0, only the Free span moves physically.
	 *  Finalize(GT): Solve *output* consumption — Flight contact detection is node movement path (Prev→Pos), i.e. solve output is
	 *                 Since it is an input, it has no choice but to be here (the basis for the asymmetry of "Prepare logic, Finalize only detection").
	 *                 Transition/event broadcast + render push + observation (stats/debugger snapshot) also here.
	 */
	void PrepareSimFrame(float DeltaTime, const TOptional<FVector>& LODCameraLocation);
	void SolveSimFrame(float DeltaTime);
	void FinalizeSimFrame(float DeltaTime);

public:
	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void SendRenderDynamicData_Concurrent() override;
	// Editor (no subsystem tick) · Make the rope visible even immediately after spawning: Initialize the Sim upon registration;
	// Push the center line once immediately after creating the render state (BuildTube runs without a tick, so bHasData=true).
	virtual void OnRegister() override;
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
#if WITH_EDITOR
	// Changing NumParticles/RopeLength in the editor reconfigures the Sim with the new values ​​(matching proxy topology).
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	//~ UPrimitiveComponent / UMeshComponent
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

protected:
	//~ Extension hook (for subclass) -------------------------------------------------
	// They are all called frame by frame (cold pass) in the game thread — there is no hook in the Solve phase that runs in parallel.
	// The node-level hot loop (solver/logic F-class) is a POD/GPU parity reference point, so it is not a virtual expansion point.
	// When adding a hook, specifying the calling thread/phase/frequency in the comment is part of the contract.

	/** Called immediately after a phase transition, immediately before broadcasting OnRopePhaseChanged (once per transition, excluding same phase reset).*/
	virtual void OnPhaseChanged(ERopePhase OldPhase, ERopePhase NewPhase) {}

	//~ Loaded(Loaded) presentation hook — all game threads (cold pass). Customize the presentation by overriding the default implementation.

	/** World transform where to place the window (tip) during Loaded. Default: LoadedHandSocket socket on the Owner skeletal mesh (or component transform if not present).
	 *  ⚠ Called **twice per frame** while on reel, not per transition (node ​​operation + tip mesh placement) — heavy calculations should be cached.*/
	virtual FTransform GetLoadedTipTransform() const;

	/** Enters Loaded **only at the edge** once (if it is already in Reel, it does not fire again even if EnterLoaded() is called again —
	 *  This is because applying the preset unconditionally calls EnterLoaded() on ③ rope). Default: Based on bShowRopeWhenLoaded
	 *  rope Turns tube render on or off. It is paired 1:1 with OnDeployFromLoaded.*/
	virtual void OnEnterLoaded();

	/** Once when leaving the reel (when the throw is established or ①② is set to preset). Default: Redisplay the rope tube and
	 *  Restore the entire length(RopeLength).
	 *  ⚠ At the time of call, GetPhase() is **still Loaded** (phase transition occurs after this hook).*/
	virtual void OnDeployFromLoaded();

	/**
	 * wrap target gate. If false, that (Mesh, Bone) candidate is treated as not existing — according to game rules such as team/tag, etc.
	 * override when limiting what can be wound. Default true (allow all).
	 *
	 * **All paths that select a target share this one gate** — If aiming/preview/check are different, "aiming is
	 * It was rejected, but the preview was "selected", resulting in a mismatch. Call point:
	 *   - Flight candidate calculation (per frame/candidate) + Contacting recollection — RemoveNonWrappableCandidates
	 *   - aiming ray query — FRopeAimTargeting::FindAimRayBoneHit
	 *     (Ban target is blocked = no aim hit)
	 *   - preview arc navigation — FRopeThrowPreviewBuilder::FInput::CanWrapTarget injection
	 *     (Because the builder is UObject-Free, virtual cannot be called directly, so the caller puts it as a lambda)
	 *   - prepared throw entry — ThrowWithPreparedPreview (last line of defense)
	 * If you add a new target selection path, this gate will also be burned.
	 */
	virtual bool CanWrapTarget(const USceneComponent* Mesh, FName Bone) const { return true; }

	//~ Event native hook: Called immediately before each delegate broadcast (engine Notify convention). C++ subclass
	//  You can react without bypassing binding to your delegate.
	virtual void NotifyCaptured(FName Bone) {}
	virtual void NotifyWrapped(const FRopeWrappedEventInfo& Info) {}
	virtual void NotifyReleased(FName Bone, ERopeReleaseReason Reason) {}
	/** Called immediately after ApplyPreset success, immediately before broadcasting OnPresetApplied (GT, cold pass — once per application).*/
	virtual void NotifyPresetApplied(const URopePreset* Preset) {}

	/**
	 * ③ Interrupt check hook (GT, cold pass — presentation is ~0.2 seconds) called every frame during presentation (GuidedThrow).
	 * Default is always false = "still guaranteed" (Decision G of meeting 2026-07-13). With game rules like target death/teleportation
	 * If the guarantee needs to be broken, override it and return true — presentation is aborted and OnRopeReleased(ThrowAborted) is fired.
	 * (Internal failure can be distinguished by the consumer as Broken). Loss of the target mesh always stops regardless of the hook.
	 * **Polled only on aiming throwing** — Throwing in the air (no target) is not guaranteed to break and Prepared is a stub.
	 * Do not call. It is a policy hook, so it is for C++ only (response is OnRopeReleased in BP).
	 */
	virtual bool ShouldAbortGuaranteedThrow(const FRopePreparedThrowPreview& Prepared) const { return false; }

	/**
	 * throwing context final interpretation — **The only extension hook that can touch the throwing context** (once per throw).
	 * Reconstruct the frame into orthogonal (right-hand system) (Forward standard, Up orthogonalization, Right = Up×Forward re-induction —
	 * input Right ignored), velocity·origin fallback. Custom points such as aim assist (if overridden, preview and actual
	 * throwing is automatically matched).
	 *
	 * **All throwing passes through this gateway** — whatever the context producer is (the Throw convenience entry point's
	 * FRopeThrowContext::MakeDefault / URopeWielderComponent::BuildThrowContext / BP direct call)
	 * Convergence here. ③ The prepared path is passed once at the time of the preview build and the result is reused.
	 * In the past, there was a hook to "change the aiming protocol" in MakeDefaultThrowContext, but the Wielder path itself
	 * Even if you override the hook by creating a context, it was invalid → The hook was unified into one here.
	 *
	 * **override must be pure** — always the same output for the same input, no state changes. ③ is preview
	 * Throwing reuses the context interpreted at the time of build. Here, if you use random numbers (aiming distribution, etc.)
	 * The results vary between iteration preview queries, or the trajectory shown by the preview and the actual throwing diverge.
	 */
	virtual FRopeThrowContext ResolveThrowContext(const FRopeThrowContext& ThrowContext) const;

	/**
	 * Pull force application (Wrapped + Tension + Active Pull active for each frame). Default recipient chain:
	 * Physics simulation bone → CharacterMovement → Physics simulation root. Override for custom movements (Mover, etc.)/vehicles/special targets.
	 * Force = pulling direction × maximum tension (|Force| = tension cap). The physical body is applied by a tension cap velocity drive.
	 * (ApplyPullVelocityDrive). DeltaTime is used to calculate impulse cap (tension×dt).
	 *
	 * This is an active Pull *policy* (what to pull and how much) hook. To intercept on a per-recipient basis:
	 * Use ApplyTractionToReceiver — The default implementation of this function also passes through that gate just before actual application.
	 */
	virtual void ApplyPullForce(const FVector& Force, const FRopePullSample& Pull, float DeltaTime);

	/**
	 * **Single gateway** (GT, maximum number of times per-frame — cold pass) just before the rope applies traction to the receiver.
	 * If true is returned, it is considered “processed by the subclass” and the basic application is omitted. Default false = built-in.
	 *
	 * **All force/velocity interventions generated by the rope pass here** — automatic tether (both ends), active pull (target), climb-in
	 * (wielder), slack break. So, this is the only custom movement (Mover, etc.), vehicle, and special recipient.
	 * If you override it, you can take all of the rope traction to your own movement system.
	 *
	 * In the past, only active pull hooks (ApplyPullForce) were used, and tether/climb-in/slack breaks were used directly on the receiver.
	 * Even if impulse·velocity was plugged in and ApplyPullForce was overridden, the tether was still pushing.
	 *
	 * The unit of Request.Amount is different for each Request.Source (refer to the FRopeTractionRequest comment).
	 * returns true, rope's internal observations/state updates still occur — even if subclasses change whether to process it or not.
	 * This is to prevent the rope state from splitting.
	 */
	virtual bool ApplyTractionToReceiver(const FRopeTractionRequest& Request) { return false; }

	// Read-only access to simulation state (for subclasses). Changes can only be made through public APIs (Throw·Set series).
	const FRopeSimState& GetSimState() const { return Sim; }

private:
	//~ Tip attachment runtime status -----------------------------------------------
	// Because it is a UObject, unlike value type sim members, GC tracking is required (Transient UPROPERTY).
	// EnsureTipMesh secures the throwing entry, and FinalizeSimFrame follows it with a Free end every frame.
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> TipMeshComponent = nullptr;

	// Did we spawn — Ownership flag to destroy only the spawn in EndPlay (protecting external components).
	bool bTipMeshSpawnedByUs = false;

	// Tips for reusing tags StaticMeshComponent's existing world scale. Even if you cover it with SetWorldTransform, the visual size is preserved.
	FVector TipMeshAuthoredScale = FVector::OneVector;

	// Point-of-acquisition relative transform for tag reuse tips (original work). When teardown, restore to this value and reacquire
	// (Preset switching) prevents miscapture (scale accumulation contamination) of the transform overwritten by each frame batch with the authoring baseline.
	FTransform TipMeshAuthoredRelative = FTransform::Identity;

	// Acquisition point collision setting for tag reuse tips (original work). Turn it off with bTipMeshCollision=false and then in Teardown.
	// Returns this value (respecting ownership of external components). Meaningless for spawning (we made it).
	TEnumAsByte<ECollisionEnabled::Type> TipMeshAuthoredCollision = ECollisionEnabled::QueryAndPhysics;

	// Lightweight query separated so that branches that only require existence do not read the socket transform.
	bool HasTipSocket(FName Socket) const;

	// Secure/destroy/follow the tip attachment in BeginPlay~EndPlay units (operates only when bUseTipMesh is turned on).
	void EnsureTipMesh();
	void TeardownSpawnedTipMesh();
	void UpdateTipMeshTransform();

	// Reflects bTipMeshCollision to the current tip component (no-op if there is no tip). The acquisition time and editing time are called.
	void ApplyTipMeshCollision();

	// Single source for where the tip sits while Loaded: LoadedTipRelativeTransform composed onto the
	// (overridable) GetLoadedTipTransform() socket frame. Both Loaded consumers - the tip mesh placement
	// and the pinned free-end node - must read this, never the raw socket transform.
	FTransform MakeLoadedTipBaseWorld() const;

	//~ Pierce Embed (Socket-based) Helper --------------------------------------------
	// **single source of truth** of socket placement active condition = tip usage + socket opt-in + ③(Guaranteed). Place the head at the insertion point
	// The concept of matching exists only in ③, so ①② does not read even if the socket name is filled in.
	// (unified by segment tracking). Since HasTipSocket burns this predicate, the entire socket path is turned off as well.
	bool IsTipSocketPlacementActive() const
	{
		return bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap;
	}
	// Tip Read StaticMesh's socket as component-local transform. If socket placement is disabled or there is no socket,
	// false (caller falls back) — This is the only gateway for socket reads.
	bool ReadTipSocketLocal(FName Socket, FTransform& OutLocal) const;
	// Tip placement local containing the existing scale of the tag component and TipMeshRelativeTransform.
	FTransform MakeTipPlacementTransform() const;
	// "Placement criteria" socket local, including final tip placement. In reality, SetWorldTransform uses MakeTipWorldTransform(BaseWorld).
	FTransform MakeTipPlacementSocketLocal(const FTransform& SocketLocal) const;
	FTransform MakeTipWorldTransform(const FTransform& BaseWorld) const;
	// Invert the world transform based on the tip mesh so that the rope attachment point (tail socket, if not present, mesh origin) is in RopeAttachWorld.
	void ComputeTipFollowTransform(const FVector& RopeAttachWorld, const FVector& ForwardDir,
		FTransform& OutComponentWorld) const;
	// Obtains the actual world location (tail socket, if not present, mesh origin) where the rope should be attached from the given standard world transform.
	FVector ResolveTipRopeAttachWorld(const FTransform& ComponentWorld) const;
	// The prepared path stored as owner-local is confirmed as a world snapshot at the time of throwing and then reflected in the Pierce socket target.
	void ApplyPierceSocketTargetsToPrepared(FRopePreparedThrowPreview& InOutPrepared) const;
	// Restore the current world hit points from a single Pierce anchor in Prepared (based on bone-local anchors if possible).
	bool ResolvePreparedPierceHitPoint(const FRopePreparedThrowPreview& Prepared, FVector& OutHitPoint) const;
	// Mesh origin (component) world with tip socket at HitPoint and Tail->Head socket vector in pierce direction (PierceDir)
	// Invert the transform. If there is a tail socket, a rope connection point (world) is also provided (if not, the mesh origin).
	// TipSocketName false if there is no socket (Pierce embed disabled). Pure placement math is owned by FRopeTipPlacement.
	bool ComputePierceEmbed(const FVector& HitPoint, const FVector& PierceDir,
		FTransform& OutComponentWorld, FVector& OutTailWorld) const;

	//~ phase state machine ----------------------------------------------------
	ERopePhase Phase = ERopePhase::Free;
	// The frame that returned to Flight from Contacting, etc. during Prepare did not go through Advance/Solve of Flight.
	// Finalize contact detection is delayed by one frame to prevent immediate recapture with the stale guide candidate.
	bool bEnteredFlightDuringPrepareThisFrame = false;

#if WITH_GAMEPLAY_DEBUGGER
	// frame start point phase. Transitions occur throughout the frame, and only the phase at the end of the frame is “from what to what.”
	// has gone, a debug snapshot is preserved to include before and after the transition.
	ERopePhase DebugPhaseAtFrameStart = ERopePhase::Free;
	// The frame(GFrameCounter) in which the above value was recorded. For writing only once per frame — the subsystem is better than Prepare
	// **Earlier**, you can run ResolvePendingAimThrow and create a Flight transition with the path to StartFreshThrow.
	// If you catch it only in Prepare, the transition has already passed.
	uint64 DebugPhaseFrameStamp = 0;
	// Was the active pull actually approved through the tension gate in this Wrapped frame? Saved by SetActivePull
	// Looking at the request value alone, it is indistinguishable from a frame with “input but blocked at the gate”.
	bool DebugActivePullPassedGate = false;

public:
	/** At the first contact point of this frame, the frame start phase is established (once per frame, subsequent calls are no-op).
	 *  Called before the subsystem touches the rope, and also called in Prepare for ropes that do not ride that path.*/
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
	 * Single point of Phase assignment. Unify the transition log ("[Name] Old -> New (Reason)").
	 * Reason is additional explanation for log (omitted if nullptr). Event broadcasts and transitions
	 * Cleanup is transition-specific, so it is up to the caller — it is not done implicitly here.
	 */
	void SetPhase(ERopePhase NewPhase, const TCHAR* Reason = nullptr);

	/**
	 * Resets the set of "work in progress" transient states that must be discarded along with phase transitions:
	 * ContactTracker / PendingWrapSeed / CaptureTravelFrame / WrappingPhase.State / ContactingElapsed /
	 * FlightNoContactElapsed / TensionOverTime.
	 * Only successful Wrapping commits maintain the existing Chaos constraint identity with bPreservePhysicalTether=true.
	 * It is no-op for members in an idle state, so it is safe to call in any transition.
	 * (ReleaseCooldown has different values for each transition and is set directly by the caller.)
	 */
	void ResetTransientPhaseState(bool bPreservePhysicalTether = false);

	// Aim-ray aiming logic/state is separated into FRopeAimTargeting (AimTargeting member). Here is a subsystem
	// Only the frame contract entry point remains — StartFreshThrow transition (orchestration) and SimFrame access are caught.
	// is owned by the component.
	// Immediately after the Subsystem fills the AimFrameColliders, the HUD/preview pending query is confirmed as the result cache.
	void ResolvePendingAimQuery();
	// Guaranteed input request is confirmed to the prepared path with the same aiming list, and if it is in the execution standby state, it is immediately thrown.
	void ResolvePendingGuaranteedAimThrow();
	// Called immediately after the Subsystem fills FrameColliders to confirm the pending request as a hit/fallback context.
	void ResolvePendingAimThrow();
	// Only the mesh+bone specified by aim ray throw is maintained as a contact/wrap candidate.
	// For collision-Free aim flights, the solver intentionally ignores this list, but
	// The Wrapping path continues to use the filtered list. The general Flight solver also uses the same list.
	void FilterFrameCollidersForAimWrapTarget();
	/** FRopeAimTargeting Context snapshot (collider snapshot + fallback dimensions) to pass to the query.*/
	FRopeAimTargeting::FQueryContext MakeAimQueryContext() const;
	/**
	 * List of colliders to be used for aiming queries (aim ray hit, GuaranteedWrap preview arc search).
	 * If you are aiming, a snapshot dedicated to aiming (rope AABB ∪ ray area — distant targets are not included in the physics list),
	 * Physical snapshot when not aiming. The latter is a BP/AI path where Throw() is called directly without Wielder aiming flow —
	 * There is no ray region, so the rope neighborhood list is the only source.
	 */
	const TArray<IRopeCollider*>& GetAimQueryColliders() const;

	//~ Simulation state + logic possession per phase -------------------------------
	// Non-UObject — Owned by value and not subject to GC tracking (only holds POD/weak references).
	// The four logics below correspond 1:1 to the rope life order: Throw/Flight → Contacting → Wrapping → Wrapped.
	/** Single Truth: Particle chain shared by solvers/logic/render.*/
	FRopeSimState       Sim;
	/** Wrapped observation only current-frame view: GT scratch covering latest start pin/anchor override in Sim copy.*/
	FRopeSimState       PullObservationSim;

	/** XPBD physics (solver-owned Free span of Free/Flight and Wrapping/Wrapped).*/
	FRopeXPBDSolver     Solver;

	/** Throw/Flight: Whip swing (calculate + apply guide target).*/
	FRopeWhipGuide      WhipGuide;

	/** Flight/Contacting: Tracking the dominant bone of the contact candidate.*/
	FRopeContactTracker ContactTracker;

	// Candidate storage used alternately for CPU Flight fallback and Contacting re-detection. Since the detector appends
	// Reset each path immediately before use. GPU Flight directly consumes SimFrame.GpuFlightCandidates.
	TArray<FRopeContactCandidate> ContactCandidateScratch;

	// Next frame whip target of CPU Flight predictive contact. The GPU path is the subsystem's dispatch payload.
	// Do not use this scratch because it carries a separately owned array.
	TArray<FVector> NextGuideTargetScratch;

	/** Contacting: Wrap seed (Wrapping entry material) created during capture.*/
	FRopeWrapState      PendingWrapSeed;
	/** Immediately after capturing GPU Flight, the pending RT step and CPU Sim have not yet been matched authoritatively. During this
	 *  Hold all Contacting dwell/dismiss/seed checks and retry SyncGpuPositionsForHandoff.*/
	bool bPendingGpuCaptureHandoff = false;

	/** Contacting~Wrapping: Snapshot of the rope progress coordinate system at the moment of capture (velocity/lying direction/progress plane normal —
	 *  From Contacting, the node is stationary and can only be measured at this moment). Guide plane fallback for the CaptureTravelPlane axis.*/
	FRopeCaptureTravelFrame CaptureTravelFrame;

	/** Wrapping: path incremental creation+front motion+mask (working state is .State).*/
	FRopeWrappingPhase  WrappingPhase;

	/** Wrapped: Maintain/release bone-local latch.*/
	FRopeWrapController WrapController;

	/**
	 * Maintains the bounded no-anchor section of the Composite Analytic Helix as a straight line between both actual anchors.
	 * Component-only runtime state. Rather than attributing to one specific bone, both surface bindings are analyzed together every frame.
	 */
	struct FKinematicVirtualBridge
	{
		/** Internal projection nodes without anchors due to SDF failure between both actual surface points.*/
		TArray<int32> NodeIndices;
		/** Left/right actual surface anchors to be reinterpreted as bone-local binding every frame.*/
		FRopeSurfaceAnchor LeftAnchor;
		FRopeSurfaceAnchor RightAnchor;
		/** Number of original segments including both end nodes × SegmentLength. This is the standard for diagnosing excessive straight height.*/
		float RestSpanLength = 0.0f;
		/** Path distance to turn on the bridge when the Wrapping front reaches the actual anchor on the right.*/
		float ActivationFrontDistance = 0.0f;
		/** Before commit, the incrementally registered bridge waits as false, and becomes true the moment both actual anchors are pinned.*/
		bool bActive = true;
		/** A one-time log latch that prevents the overextension warning of the same bridge from repeating every frame.*/
		bool bLoggedStretchWarning = false;
	};

	/** Virtual run with real anchors on both sides. During Wrapping, it is pinned straight from reaching the front to Wrapped.*/
	TArray<FKinematicVirtualBridge> KinematicVirtualBridges;
	/** Prefix length synchronized to the component bridge during run calculated once by WrappingPhase.*/
	int32 KinematicVirtualBridgeRunCursor = 0;

	/** ③ GuidedThrow operation status: confirmed preview path (aiming) or ray end point arch (empty, bFreeThrow).*/
	FRopeGuidedThrowState GuidedThrowState;

	// The whip guide spline plane confirmed at the start of the Flight is normal. When entering Wrapping through Contacting
	// Reuse as the direction of the virtual Wrapping axis established at the bone location.
	bool bHasFlightGuidePlaneNormal = false;
	FVector FlightGuidePlaneNormal = FVector::RightVector;

	// Aim-ray aiming state (wrap target lock per throw + pending HUD/preview query/result + aim throw cue).
	// Contains query/lock check logic —
	// See comment FRopeAimTargeting(Logic/RopeAimTargeting.h).
	FRopeAimTargeting AimTargeting;

	/** ③ exclusive state in which the input ray is confirmed once in normal gather and then executed immediately or stored until montage notify.*/
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

	/** Executes the confirmed ③ request as prepared or Free-arc at the time of input without inquiry and sends a completion/rejection callback.*/
	bool ExecutePendingGuaranteedAimThrow();

	//~ phase timer --------------------------------------------------------
	/** Contacting dwell time (WrapDecisionTime check).*/
	float ContactingElapsed = 0.0f;

	/** Time spent in Flight without capture after exiting Whip.*/
	float FlightNoContactElapsed = 0.0f;

	/** Time remaining until Releasing → Free return.*/
	float ReleaseCooldown = 0.0f;

	/** The time during which the maximum tension continuously exceeds TensionReleaseForce during Wrapped.*/
	float TensionOverTime = 0.0f;

	// Wrapped traction/Smoothing state bundle (Pull sample/EMA 3 types/Active Pull/Tether overflow/Warning latch). Meaning of each member
	// See FRopePullDriveState(Core/RopePullDriveState.h) comment for reset protocol (what survives) on transition.
	FRopePullDriveState PullDrive;
	// Passive material-length authority: rejected movement, constraint λ/tension and live
	// material/anchor history. Never sourced from XPBD SegmentTension.
	FRopeLengthConstraintState LengthConstraintState;
	FRopeResolvedWrappedEndpoints WrappedEndpointCache;

	// Wrapping velocity(cm/s, +Wrapping/-unwrapping, 0=stationary). SetReelRate is set, UpdateReel is applied to each frame.
	float ReelRate = 0.0f;

	// Applying a rewrapping frame (at the beginning of Prepare): Adjust the length by ReelRate × dt in the allowable phase.
	void UpdateReel(float DeltaTime);

	//~ Sleep/LOD (Scaling) ---------------------------------------------------
	// Status and check are separated into FRopeSolverThrottle (Logic/RopeSolverThrottle.h) — The component includes camera access (GT) and
	// Only the sleep transition log remains.
	FRopeSolverThrottle Throttle;

	// Distance LOD scale calculation (Prepare, GT): The subsystem converts the camera position obtained per-frame into a distance and delegates it to Throttle.
	void ComputeSolverLOD(const TOptional<FVector>& CameraLocation);
	// LOD reflected valid iteration (CPU solve/GPU step shared — subsystem called).
	int32 GetLODScaledIterations() const { return Throttle.LODScaledIterations(SolverConfig.Iterations); }

	// Operation 1 — Automatic traction (Tether, Docs/PoC/05): Observation (entire chain C·opening velocity) → solve λ → apply impulse pairs at both ends.
	// ApplyWrappedTraction is called every Wrapped frame. The results are recorded as a single unit in LengthConstraintState.
	void UpdateConstraintTether(float DeltaTime);

	// (Constraint tether — ragdoll target half) Engine physics constraint: Kinematic proxy of corner ↔ anchor point of Wrapped bone
	// Tie with a spherical limit of leg rest length. The target is **all simulated bodies** (skeletal bone + component body):
	// GT per-frame velocity impulse is the joint body's “whole body size kick → runaway” vs. “bone size λ → traction force collapse” dilemma
	// (2026-07-20 Pierce actual measurement iteration), it also structurally loses under aerial load (suspended prop) — gravity and swing
	// During the physics substep, GT only performs post-offset one beat late, so floating, pendulum pumping, orthogonal damping
	// Dependency is created (2026-07-22 PIE). Chaos constraints are solved together with gravity, joints, and contact in substep (Docs/PoC/05
	// §3.4-1·§9). If Wielder is present, PrePhysics performs a single drive and PostPhysics only observes force.
	// Custom mover without Wielder uses UpdateConstraintTether's legacy drive. Dissolution is abort/release,
	// Change target/bone, performed in EndPlay.
	void UpdatePhysicalTether(class UPrimitiveComponent* TargetPrim, FName Bone,
		const FVector& AnchorWorld, const FVector& CornerWorld, float LegRestLen, float DeltaTime);
	/** Currently only Chaos constraint force is read. Proxy/limit transform does not change.*/
	void SamplePhysicalTetherForce(float DeltaTime);
	/**
	 * Wielder calls right after movement/right before Chaos. authoritative projection before attempted and actual rejection velocities
	 * is recorded as a reaction, and if it is a physical target, the same attempt is also delivered to the Chaos proxy.
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

	/** Physical constraint Tether's kinematic proxy (corner following) and constraint — runtime only, lives only on the simulation body target.*/
	UPROPERTY(Transient)
	TObjectPtr<class USphereComponent> PhysicalTetherProxy;
	UPROPERTY(Transient)
	TObjectPtr<class UPhysicsConstraintComponent> PhysicalTetherConstraint;
	// Target/bone (change detection → regeneration) bound by constraint and current limit (cm — for update skip, <0 = not set).
	TWeakObjectPtr<class UPrimitiveComponent> PhysicalTetherTarget;
	FName PhysicalTetherBone = NAME_None;
	float PhysicalTetherLimit = -1.0f;
	/** GFrameCounter: Wielder has already used authoritative proxy/limit in this frame PrePhysics.*/
	uint64 PhysicalTetherPrePhysicsFrame = MAX_uint64;
	// Body-local anchor pinned during creation (constraint Frame2) — When the wrap anchor is relocated within the same (target, bone)
	// The reference value of the guard that detects and regenerates drift.
	FVector PhysicalTetherAnchorLocal = FVector::ZeroVector;
	// EMA of observation anchorLocal — Prevents thrashing by drift guard regenerating every frame. AnchorWorld is a rope Sim
	// (GPU 1~2 frame delay mirror), BodyTM comes from the current bone, so the delay difference between the two in the fast ragdoll bone is
	// Shakes anchorLocal significantly every frame (not true relocation). You should check with this smoothing value, not the instantaneous value.
	// Delay noise is filtered out and only continuous relocations (seed joining/promotion) are captured.
	FVector PhysicalTetherSmoothedAnchorLocal = FVector::ZeroVector;

	// (Tether) Calculate the wielder traction direction (hand (node0) → rope first leg = anchor side) and use PullDrive.SmoothedWielderPullDir.
	// Returns EMA smoothing (PullDirSmoothTime) — Prevents the input axis from bouncing due to direction jitter (180° flip degenerate is raw reseed).
	// If bInstantaneous, omit EMA and use instantaneous geometry (air swing — EMA cannot keep up with orbital rotation)
	// Tangential error in lag direction interferes with swing operation; The state continues to seed and upon landing, EMA re-entry is continuous).
	FVector ComputeSmoothedWielderDir(const FVector& Aim, const FVector& DirToAim, float DeltaTime, bool bInstantaneous);

	// Updates the pullability check of this Wrapped frame regardless of overshoot — the climb-in direction of the active pull and
	// Distribution observation (LastTargetShare binary value) shares PullDrive.bTargetPullable. Comparison of effective mass at both ends + hysteresis.
	void UpdateTargetPullable();

	// In the Wrapped traction section, target/wielder is analyzed delayed and shared by check, tether, and basic pull of the same frame.
	const FRopeResolvedWrappedEndpoints* GetOrResolveWrappedEndpoints();

	// (not pullable) Apply active Pull force to wielder (rope owner) — target is heavy
	// climb-in, where the wielder is pulled towards the anchor. Mirror the owner side of ApplyPullForce (simulation root → CharacterMovement).
	void ApplyPullForceToWielder(const FVector& Force, float DeltaTime);

	// Active Pull tension cap velocity drive: Pull the target physical body to target velocity (ActivePullMaxLinearSpeed) along direction (Dir).
	// , clamp the impulse to J = min(mass×ΔV, MaxTension×dt). For light targets, the target velocity is immediate (no overshoot),
	// Heavy objects lag behind in tension limits (realistic mass dependence). Eliminates overshoot, dust, and bumps of constant force (a=F/m).
	void ApplyPullVelocityDrive(UPrimitiveComponent* Prim, FName BoneName, const FVector& Dir, float MaxTension, float DeltaTime) const;

	// Active Pull Clamps the angular velocity of the target physical body with the HoldConfig cap (residual ragdoll spin safety net — forces the force to the center of gravity)
	// , so there is no pull torque already). ApplyPullForce is called after force application. If BoneName None, component unit.
	void ClampPulledBodyVelocity(UPrimitiveComponent* Prim, FName BoneName) const;

	// Shared finalization of all release triggers (phase transition+node return+transient state disposal+cooldown+event).
	void FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog);

	// Shared finalization of departure before establishment (Captured~Wrapping): Per-instance release after Flight transition + temporary state disposal
	// Notify (post-clearance notification — DispatchReleased reentrant contract). Dismiss/stall/Wrapping-abort shared in 4 places. Bone is
	// Captured as a value at the time of call (before Reset). Before commit, bWasWrapped=false (per-instance only, no central signal).
	void FinishPreCommitReleaseToFlight(FName Bone, const TCHAR* PhaseLog);

	// ReleaseWrap/CutRope shared Body: Releases an ongoing grab/Wrapping for a given reason (including interpretation of bone attribution).
	void ReleaseWrapAs(ERopeReleaseReason Reason);

	// (ApplyPullForce — Action 2, Apply Pull Force — moves to the protected extension hook.)

	//~ Subsystem frame contract (written/read by RopeSimSubsystem) --------------
	// Simulation input/output bundle per frame. For meaning/lifetime conventions for each member, refer to the FRopeSimFrameIO (Core/RopeSimFrameIO.h) comment.
	// The field name is the same as when it was an individual member, so only the access path is SimFrame.X (CL 303).
	FRopeSimFrameIO SimFrame;

	// Last render push status. stationary rope skips dynamic-data/transform dirty, but GPU resident transition and
	// A component transform change must be compared to push the new WorldToLocal/local centerline.
	FTransform LastRenderDataComponentTransform = FTransform::Identity;
	bool bHasLastRenderDataComponentTransform = false;
	bool bLastRenderDataGpuResident = false;

	//~ Initialization/Utility ----------------------------------------------------------
	void InitRope();

	/** If the Sim is empty, it is initialized once (safety guard at the beginning of OnRegister/Throw/Prepare).*/
	void EnsureRopeInitialized();

#if WITH_GAMEPLAY_DEBUGGER
	// Populates the centerline/Wrapped/collider common fields into a snapshot when subject to debug capture (called from FinalizeSimFrame).
	/** Always fill the header summary, and fill the rest of the sections with only what's in the CaptureMask (you don't incur the cost of collecting disabled views).*/
	void FillDebugSnapshot(FRopeDebugSnapshot& Snapshot, ERopeDebugCapture CaptureMask) const;
#endif

	//~ Throw ----------------------------------------------------------------
	// (MakeDefaultThrowContext/ResolveThrowContext moved to protected extension hook.)
	/** Creates a guaranteed preview with a context that has already passed ResolveThrowContext.
	 *  The success/failure path of ThrowWithContext is an internal entry point to share the same analysis result.*/
	bool BuildPreparedWrappingPreviewFromResolvedContext(const FRopeThrowContext& ResolvedThrowContext,
		FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason) const;

	// Throwing startup is read in the pinned order of the helpers in step 4 below (StartFreshThrow is orchestration only).
	void StartFreshThrow(const FRopeThrowContext& ThrowContext);

	/** ① Common dictionary summary of all throws: active wrap is released with a normal release notification,
	 *  Allows discarding a bridge/phase transient and starting a new throw without cooldown.*/
	void ResetStateForNewThrow();

	/** Configure Prepared/Free shared GuidedThrow status and starting node pin.*/
	bool BeginGuidedThrowState(FRopePreparedThrowPreview&& Prepared, bool bFreeThrow);

	/** ② Chain reset: Pin hand (node ​​0) to the origin, all node velocity 0 (Prev=Pos), increase GPU resident buffer reseed generation.*/
	void ResetChainForThrow(const FVector& HandOrigin);

	/** ③ Start whip swing: Assemble swing base/inherited velocity in ResolvedThrow and activate WhipGuide + Snap T=0.*/
	void BeginWhipSwingFromThrow(const FRopeThrowContext& ResolvedThrow);

	/** ④ Verlet velocity injection: Inject throwing velocity by pushing PrevPositions in the direction opposite to aiming.
	 *  (velocity = (Pos-Prev)/dt in Verlet — just pushing Prev injects pure velocity without change in position).
	 *  Since ③ uses the confirmed aiming direction (WhipGuide.GetAimDir), it must be called after ③.*/
	void InjectThrowVelocityIntoVerlet(const FRopeThrowContext& ResolvedThrow);

	/** GuidedThrow phase One frame progresses. Move the node to the preview centerline and turn off the solver.*/
	void UpdateGuidedThrow(float DeltaTime);

	/** Upon completion of GuidedThrow, convert the prepared anchor to FRopeWrapState and immediately commit to Wrapped.*/
	void FinishGuidedThrow();

	/** Single point of Captured notification (native hook → BP delegate). ①② Flight Capture and ③ Reach share —
	 *  If you place an inline broadcast for each path, it will not be visible even if one side is missing (actually, ③ was like that).*/
	void DispatchCaptured(FName Bone);

	/** Throwing in the air (no target): Initiates an arch GuidedThrow toward the ray's endpoint (EndpointWorld) (Free if completed without being stuck).*/
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

	/** throwing strength contract gate: Interpret Context.ThrowSpeed (if 0, ThrowParams.ThrowSpeed fallback)
	 *  If it is a positive number (≥1cm/s), it is stored in OutThrowSpeed and true, otherwise it is false after logging the warning. **Called before state change**.*/
	bool TryResolveValidThrowSpeed(const FRopeThrowContext& Context, float& OutThrowSpeed) const;

	/** Create a configuration snapshot to be passed on to WhipGuide from Rope|Whip UPROPERTYs.*/
	FRopeWhipGuide::FConfig MakeWhipGuideConfig() const;

	/** Tail weight of throwing impulse (0→1 smooth from FirstTailNode to end).*/
	float TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const;

	//~ Flight ---------------------------------------------------------------
	// The contact detection pipeline itself is separated into FRopeFlightContactDetector (static, UObject independent).
	// All that remains here is assembly code that requires a UObject context.

	/** Parameter snapshot to be passed to the detector (WrapConfig + tube radius + component front + substep dt + frame dt).
	 *  SubstepDeltaTime is FixedDt derived from SolverConfig.Substeps and SurfaceVelocity(cm/s→displacement) for relative motion evaluation.
	 *  conversion, FrameDeltaTime is used to convert substep → frame displacement of predicted contact extrapolation.*/
	FRopeFlightContactDetector::FParams MakeFlightDetectParams(float DeltaTime) const;

	// FinalizeSimFrame's Flight block is read in the pinned order of the helpers below:
	// ① candidate calculation → ② capture check/transition → ③ observation (stats/debugger — read-only consumption separate from check).

	/** Applies the CanWrapTarget gate to the candidate list (removes prohibited targets). Flight calculation and contact re-collection
	 *  Puts the filter in one place so that it uses the same check set — fixing the condition causes the two phases to move together.*/
	void RemoveNonWrappableCandidates(TArray<FRopeContactCandidate>& Candidates) const;

	/** ① Select/calculate Flight candidate for this frame. GPU results consume the SimFrame array directly without copying it,
	 *  CPU fallback is created in ContactCandidateScratch in the order of actual→predicted→relative motion.*/
	TArray<FRopeContactCandidate>& GetOrBuildFlightContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams);

	/** Calculate candidate for CPU Flight fallback only. OutCandidates and NextGuideTargetScratch are reset by the caller in advance.*/
	void BuildCpuFlightContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeContactCandidate>& OutCandidates);

	/**
	 * Assisted aim lock-only synchronous supplement: The actual path of the CPU whip target is independent of the GPU asynchronous result.
	 * Inspect the bone collider that is accurately locked. Readback of middle frame loss and same-mesh deep neighboring bones
	 * Prevents primary occlusion. In Flight, the predicted path is also included, and in Contacting, it is kept as actual-only.
	 * No additional costs are added to regular full/non-aiming flights.
	 */
	void AddSynchronousAssistedAimContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeContactCandidate>& InOutCandidates);

	/** ②a Aggregate the candidates once to create a frame-local result to be shared between capture check and observation.*/
	FRopeFlightCaptureEvaluation EvaluateFlightCapture(const TArray<FRopeContactCandidate>& Candidates,
		const FRopeFlightContactDetector::FParams& DetectParams) const;

	/** ②b Apply the evaluation results to the game state. If captured, move the Tracker to ContactTracker and go to Contacting.
	 *  , otherwise, end the whip and roll the failure timer. Returns whether or not it is actually captured.*/
	bool ApplyFlightCaptureEvaluation(float DeltaTime, const TArray<FRopeContactCandidate>& Candidates,
		FRopeFlightCaptureEvaluation& Evaluation);

	/** ③ Observation: stat counter (only when collecting) + debugger snapshot (when OutSnapshot != null — debugger target)
	 *  only the rope comes over). All read-only consumption that is not involved in check(①②) is trapped here —
	 *  The purpose is to ensure that no debug/stat code is left in the body of FinalizeSimFrame.*/
	void RecordFlightObservation(const FRopeFlightContactDetector::FParams& DetectParams,
		const TArray<FRopeContactCandidate>& Candidates, const FRopeContactTracker& FrameTracker,
		bool bShouldCapture, FRopeDebugSnapshot* OutSnapshot);

#if WITH_GAMEPLAY_DEBUGGER
	/** ③ Observation assistance (debugger target rope only): Per-node detection input/check visualization data collection. bone pipeline and
	 *  Query the detector separately (previously node sweep) — Intentional duplication, costing only one target rope.*/
	void GatherFlightNodeDebug(const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeFlightNodeDebug>& OutNodeDebug) const;
#endif

	/** When capture is confirmed, the evaluation tracker moves to the owned state and enters the Contacting state.
	 *  (PendingWrapSeed/CaptureTravelFrame/Timer).*/
	void BuildContactingState(FRopeContactTracker&& EvaluatedTracker,
		const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);

	//~ Contacting -----------------------------------------------------------
	// Re-collect actual contacts every frame and update the tracker dwell: Continuous contact → Wrapping, contact loss →
	// dismiss(Flight), dwell is less than the threshold → safety net timeout (Flight). The check standard is the total progress
	// It is not a tracker dwell (reset when the dominant bone changes).
	void UpdateContacting(float DeltaTime);

	bool ShouldDismissContacting() const;

	bool ShouldStartWrapping() const;

	FRopeWrapState BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const;

	/**
	 * Configure the seed latch/anchor of one (Bone, Mesh) target (dominant/secondary shared by seed multiplexing).
	 * OutLatch is always filled, and is true if the surface frame is obtained from the contact candidate and an anchor is created.
	 * OutMesh is the result of falling back to the candidate mesh when there is no tracker mesh (null if both are missing).
	 */
	bool BuildSeedLatchForTarget(const TArray<FRopeContactCandidate>& Candidates,
		FName Bone, const USceneComponent* TrackedMesh, int32 NodeIndex, float RopeDistance,
		FRopeLatchNode& OutLatch, FRopeSurfaceAnchor& OutAnchor, const USceneComponent*& OutMesh) const;

	//~ Wrapping -------------------------------------------------------------
	// The actual logic of the Wrapping phase, such as path creation/front motion/mask, is FRopeWrappingPhase(WrappingPhase).
	// Separated. Here, only the orchestration that determines phase transitions and events remains.

	void StartWrappingFromContacting();

	void UpdateWrapping(float DeltaTime);

	/** Calling context to pass to WrappingPhase (WrapConfig/collider snapshot/tube radius/log name).*/
	FRopeWrappingPhase::FContext MakeWrappingContext() const;

	/**
	 * Storage of the list of colliders passed by MakeWrappingContext — contains only those that passed the CanWrapTarget gate
	 * (FContext holds an array *by reference*, so it needs storage that outlives the call).
	 * It is a gateway that prevents the prohibited object from being raised as a surface/attribution candidate in the Wrapping path build, and is a gate.
	 * In ropes that are not overridden, the contents are the same as FrameColliders (operation is unchanged).
	 */
	mutable TArray<IRopeCollider*> WrappableColliders;

	void CommitWrapping();

	/** Wrapped establishment event payload assembly (commit seed + decision value → NotifyWrapped/OnRopeWrapped shared).*/
	FRopeWrappedEventInfo MakeWrappedEventInfo(const FRopeWrapState& Seed, float AngleDeg, float CoverageDeg) const;

	/** wrap established Single broadcast: native hook + per-instance BP delegate + subsystem central signal (③/check shared).*/
	void DispatchWrapped(const FRopeWrappedEventInfo& Info);

	/** release Single broadcast. per-instance(NotifyReleased + OnRopeReleased) always fires — engagement
	 *  Match each pair at the end (including Contacting/Wrapping abort·destroy). Opening the engagement is Captured,
	 *  Wrapped, **or aimed ③ throwing**(ThrowWithPreparedPreview successful — no start event, but
	 *  ). **In the air ③ throwing(bFreeThrow) has no target and does not open anything, so release is also
	 *  None** — Landing is just Free. central OnAnyRopeReleased fires **only when wrap(bWasWrapped) is committed** —
	 *  If you shoot at abort before establishment, another rope wraps around and incorrectly restores the ragdolled target. WrappedMesh is the central signal
	 *  Payload (nullptr before establishment).*/
	void DispatchReleased(const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason, bool bWasWrapped);

	/**
	 * Queue containing release notifications received during Wrapped notifications — The purpose is to **prevent order reversal**.
	 * If the handler calls ReleaseWrap() within the notification, the release notification will overlap and finish first, so the subscriber
	 * Received in the order of Released → Wrapped (the ragdoll target ignores recovery first, then receives only Wrapped and is permanently stuck).
	 * Therefore, the release notification is postponed until the Wrapped notification is completed, ensuring the order **always Wrapped → Released**.
	 * State changes (ReleaseWrap itself) are not deferred — only notifications are deferred.
	 */
	struct FDeferredReleaseNotice
	{
		TWeakObjectPtr<USceneComponent> WrappedMesh;
		FName Bone = NAME_None;
		ERopeReleaseReason Reason = ERopeReleaseReason::Manual;
		bool bWasWrapped = false;
	};

	/** DispatchWrapped nesting depth (>0 queues release notifications).*/
	int32 WrappedDispatchDepth = 0;
	TArray<FDeferredReleaseNotice> DeferredReleaseNotices;

	/** Sends out queued release notifications in order (called only after Wrapped notifications are completely completed).*/
	void FlushDeferredReleaseNotices();

	void AbortWrapping(ERopeReleaseReason Reason);

	/** ③ Presentation(GuidedThrow) Interruption shared Finalization: Releasing transition + temporary state disposal + cooldown + release event.
	 *  Only aiming throwing fires events — empty throwing (bFreeThrow) does not have open engagement, so it does not match.*/
	void AbortGuidedThrow(ERopeReleaseReason Reason, const TCHAR* ReasonLog);

	//~ Wrapped --------------------------------------------------------------
	// The Wrapped case of PrepareSimFrame is read in the pinned order of the helper in step 4 below.

	/** ① bone following: Hold (reposition on skin bone — no velocity injection) + mass mask. If the target mesh is lost
	 *  After Broken release, false — the caller ends this frame here.*/
	bool HoldWrappedNodesToBone(float DeltaTime);

	/** ② Observation calculation: authoritative constraint tension + Pull sample(ComputePull) + 2-stage smoothing (aiming fractional)
	 *  EMA → direction EMA). traction(③)/release check(④)/debugger/BP creates input that is read as shared.*/
	void UpdateWrappedPullSample(float DeltaTime, const FRopeSimState& ObservationSim);

	/** ③ Application of traction: Tether (λ impulse constraint + ragdoll physical constraint) + active Pull (constant force when tense/climb-in).*/
	void ApplyWrappedTraction(float DeltaTime);

	/** ④ Automatic release check: Exceeding tension (TensionRelease*) / Exceeding distance (DistanceReleaseSlack —
	 *  Consumption of excess updated by tether in ③). true if release occurred — caller skips solve.*/
	bool CheckWrappedAutoRelease(float DeltaTime);

	/** latch/anchor node InvMass=0, remaining 1 — The solver moves only the Free span among Wrapped.*/
	void ApplyWrappedMassMask(bool bResetDynamicNodeVelocity = false);
	// The entire mass mask is recreated only when topology/binding changes. Each frame hold updates only the pinned node location/InvMass.
	bool bWrappedMassMaskDirty = true;

	/** WrappingPhase registers the newly calculated run as a bridge only once and activates it when it reaches the front.*/
	void UpdateWrappingKinematicVirtualBridges(const TArray<FRopeVirtualBridgeRun>& Runs,
		const TArray<FRopeSurfaceAnchor>& Anchors, float FrontDistance);

	/** Re-verify and update the existing bridge as the final commit anchor and activate it. does not regenerate*/
	bool FinalizeKinematicVirtualBridges(const TArray<FRopeVirtualBridgeRun>& Runs,
		const TArray<FRopeSurfaceAnchor>& CommitAnchors);

	/** Evenly arrange bridge nodes between the current world positions of both anchors and use hard kinematic override.*/
	void HoldKinematicVirtualBridges();

	/** Discards the previous bridge binding at release/rethrow/non-composite entry.*/
	void ResetKinematicVirtualBridges();

	/** Return the active bridge node to the solver mass, remove velocity, and discard the binding and scan states.*/
	void ReleaseKinematicVirtualBridgesToSolver();

};
