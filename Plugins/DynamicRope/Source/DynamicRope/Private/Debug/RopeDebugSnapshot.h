// Copyright Epic Games, Inc. All Rights Reserved.
//
// One frame's debug snapshot of a rope. The sim tick fills it in on the game thread and submits it to
// URopeDebugSubsystem, and FGameplayDebuggerCategory_Rope reads it back and draws it through AddShape
// and AddTextLine. It is the data carrier that replaces immediate-mode debug drawing, and like the
// solver and the logic classes it is plain data with no UObject coupling.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeContactTrackingTypes.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeSimTypes.h"
#include "Core/RopeWrappingTypes.h"

/**
 * Capture scope bits. They are produced by the category's view toggles and carried all the way to the
 * capture side. Blocking only the drawing would leave the collection cost of a disabled view in
 * place, namely re-querying nodes against colliders, the extra Flight sweep, and rebuilding collider
 * shapes and convex hulls, so the debugger would keep weighing down the simulation.
 * Aim reads the wielder live and Advanced only changes display detail, so neither costs anything to
 * capture and neither has a bit.
 * The header summary, that is the phase, the node positions and the solve path, is always filled in:
 * it is needed whichever view is on and it is cheap.
 */
enum class ERopeDebugCapture : uint8
{
	None      = 0,
	// Per-node proximity re-query. The most expensive item, at one CPU query per node per collider.
	Nodes     = 1 << 0,
	// The Flight node re-sweep plus copies of the candidates and the whip guide.
	Flight    = 1 << 1,
	// Wrapped detail, meaning copies of the latches and the pull observations, plus the wrap axis.
	Wrap      = 1 << 2,
	// Collider shape copies plus convex hull edge reconstruction, which is cubic in the plane count.
	Colliders = 1 << 3,
};
ENUM_CLASS_FLAGS(ERopeDebugCapture);

// Per-node Flight debug: the movement from the previous position to the current one, plus the contact
// where there is one. Filled in only for the rope being captured.
struct FRopeFlightNodeDebug
{
	int32 NodeIndex = INDEX_NONE;
	FVector PrevPosition = FVector::ZeroVector;
	FVector Position = FVector::ZeroVector;
	bool bNearBody = false;
	FRopeContact Contact;
	// The source mesh of the contact, frozen at capture time into a comparison-only key that says
	// whether a candidate refers to the same target. A snapshot outlives its frame by several frames,
	// so building the key from a raw pointer at draw time would dereference an already-destroyed
	// component.
	FObjectKey ContactMeshKey;
};

// The kind of collider visualization shape. FillDebugSnapshot classifies them through mutually
// exclusive accessors.
enum class ERopeDebugColliderShape : uint8
{
	// An A-to-B segment plus a radius, for skeletal bones and static spheres and sphyls.
	Capsule,
	// An oriented box, from its centre, rotation and half extents, for static boxes.
	Box,
	// A hull wireframe, where consecutive pairs in ConvexEdges form one edge, for static convexes and
	// shear boxes.
	Convex,
	// The world AABB fallback, for shapes of unknown form such as SDFs.
	Bounds,
};

// Per-node proximity diagnostics. It re-queries the post-solve node positions against the colliders to
// record, as data, which face a node is up against and with what normal. The GPU runtime does not read
// contacts back, so this is a debug-only CPU query.
// These are not the contacts the solver actually resolved: the query radius is deliberately wider than
// the collision radius, so nearby nodes that never touched are caught too. That is what makes it
// useful for confirming by eye that a node is sticking to a wall or a face.
struct FRopeNodeProximityDebug
{
	int32   NodeIndex = INDEX_NONE;
	// The node's world position, which is where the arrow starts.
	FVector Position = FVector::ZeroVector;
	// The outward contact normal as a unit vector; which face it is showing is this direction.
	FVector Normal = FVector::ZeroVector;
	// IsWorldStatic() of the collider that produced this contact, used to colour it. It is not inferred
	// from whether a bone exists, since a custom non-static collider that reports no bone and a static
	// prop carrying a virtual bone are both counterexamples.
	bool    bWorldStatic = false;
	// The bone name on a skeletal collider, or None for a static one.
	FName   Bone = NAME_None;
};

// One collider visualization. Only the fields matching Shape are valid.
struct FRopeDebugCollider
{
	ERopeDebugColliderShape Shape = ERopeDebugColliderShape::Bounds;
	// Static world geometry, meaning boxes, convexes and static capsules, versus skeletal, used to
	// colour it.
	bool bWorldStatic = false;
	// A static mesh wrap target served by URopeWrapTargetComponent: it has a virtual bone, so it joins
	// detection, but its source mesh is not skeletal. It counts only these static opt-in targets rather
	// than every wrappable target, which is why the on-screen label reads staticWrapTargets.
	bool bWrapTarget = false;
	// Whether this collider can actually be wrapped, meaning its attribution of bone and mesh is valid
	// and it passed the CanWrapTarget() gate.
	// That is separate from IsWorldStatic(): a target can take part in detection and still be refused
	// by the gate.
	bool bWrapAllowed = false;

	// Capsule
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;

	// Box (OBB)
	FVector Center = FVector::ZeroVector;
	FQuat   Rot = FQuat::Identity;
	FVector HalfExtents = FVector::ZeroVector;

	// Convex only: world-space edge endpoints, where consecutive pairs form one edge. It exists purely
	// for debug drawing and is not stored on the runtime collider.
	TArray<FVector> ConvexEdges;

	// The bounds fallback.
	FBox Bounds = FBox(ForceInit);
};

// One frame's debug snapshot of a single rope, holding the centreline, Flight, whip, Wrapped and
// collider sections together. An empty section is identified by its bHas flag; bHasFlight, for
// example, is only true during the Flight phase.
struct FRopeDebugSnapshot
{
	// The frame the snapshot was submitted on. The category ignores snapshots that are too old, which
	// stops a released target leaving an afterimage on screen.
	uint64 FrameStamp = 0;

	//~ Centreline, always present ------------------------------------------
	// The phase at the end of the frame, just before submission. Differing from PhaseAtFrameStart below
	// means the rope changed phase during this frame.
	ERopePhase Phase = ERopePhase::Free;
	// The phase at the start of the frame, on entering Prepare. Transitions happen inside Prepare and
	// Finalize, so one snapshot can span both sides of a transition, such as a frame that began in
	// Flight and ended in Contacting. The Flight overlay is still valid on such a frame, because it
	// holds the very observations that caused the transition; hiding it because the phase no longer
	// matches would discard the reason the transition happened.
	ERopePhase PhaseAtFrameStart = ERopePhase::Free;
	// The centreline positions. Only the Nodes, Flight and Wrap overlays read them, so they are filled
	// in only when at least one of those views is on and are left empty in the default aim-only state.
	// The node count in the header comes from the NodeCount scalar below instead.
	TArray<FVector> Positions;
	// The node count. It is always carried, because the header has to report it even in views that do
	// not copy the positions.
	int32 NodeCount = 0;
	// The index of the latch node to highlight on the centreline.
	TArray<int32> LatchedNodes;

	//~ Frame state for the header ------------------------------------------
	// The header reads these values rather than the live component so that one screen only ever uses a
	// single point in time. Mixing them with live data would draw the same node at two different
	// moments and read as simulation jitter.
	// Note that on the GPU path Sim.Positions is itself a readback mirror and lags one to two frames.
	// What is being aligned here is consistency within the debugger, not agreement with the tube the
	// GPU buffers actually draw.
	// Identity. With several ropes on screen this is the only way to tell which line belongs to which
	// component, since an index into the collection order can change from frame to frame. The owning
	// actor is reported as well because in a cross-actor wrap an index does not reveal which character
	// the rope belongs to.
	FString ComponentName;
	FString OwnerActorName;
	// The resolve mode. Each mode has an entirely different contract for establishing a wrap and a
	// different set of meaningful settings, so it is the premise for reading the rest of the screen.
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	// The resolved node collision radius (cm), from GetEffectiveCollisionRadius(). It is how far the
	// solver holds a node off a contact surface, and with the default automatic setting it equals the
	// render tube radius.
	// The latch marker size is derived from it, which gives "how big is that box" a real meaning.
	float NodeCollisionRadius = 0.0f;

	FName WrapBoneName = NAME_None;
	bool  bSleeping = false;
	float LodScale = 1.0f;
	// The input to the tube eligibility calculation, taken from the same settings the scene proxy uses.
	int32 NumParticles = 0;
	int32 TubeSmoothingSubdiv = 1;
	// The input to the solve path token; the combination of the three values, including bSleeping,
	// distinguishes six cases.
	bool  bSolveThisFrame = false;
	bool  bGpuStepped = false;
	bool  bLogicOverride = false;

	//~ Flight, during the Flight phase only ---------------------------------
	// The three capture decision values are deliberately absent. MinLatchNodes is a config constant
	// that does not change at runtime; bShouldCapture transitions to Contacting within the same frame
	// it becomes true, which duplicates the Flight to Contacting arrow in the header; and the positions
	// in TrackerNodes are already drawn by the candidate boxes. All three are momentary, so the
	// accumulating counters in stat RopeFlight, namely the candidate nodes and capture decisions, are
	// the useful form.
	bool bHasFlight = false;
	FName TrackerBone = NAME_None;
	// The dominant target's mesh, converted to a key at capture time. The identity contract for a
	// contact target is the pair of mesh and bone, described on FRopeContactTracker: comparing bone
	// names alone would highlight the wrong candidate as dominant whenever two actors sharing a
	// skeleton are touching.
	FObjectKey TrackerMeshKey;
	TArray<FRopeFlightNodeDebug> NodeDebug;
	// Note that Candidates[].Mesh is a raw pointer. A snapshot outlives its frame by several frames, so
	// it must never be dereferenced; decide target identity through CandidateMeshKeys below, which is
	// converted at capture time and indexed one-to-one with Candidates.
	TArray<FRopeContactCandidate> Candidates;
	TArray<FObjectKey> CandidateMeshKeys;

	//~ Whip guide, while the whip is active ---------------------------------
	bool bWhipActive = false;
	float WhipGuidedEnd = 0.0f;
	TArray<int32> WhipGuideNodeIndices;
	TArray<FVector> WhipGuideTargets;

	//~ Wrapped, during the Wrapped phase only -------------------------------
	bool bHasWrapped = false;
	FString MeshName;
	TArray<FRopeLatchNode> Latched;
	// The maximum segment tension, mirroring FRopeWrapState::Tension.
	float WrapTension = 0.0f;
	// The threshold tension, where 0 means disabled. For display.
	float TensionReleaseForce = 0.0f;
	// Whether the automatic tension and distance releases actually apply. GuaranteedWrap returns early
	// from CheckWrappedAutoRelease and never looks at the thresholds, since its guarantee is symmetric
	// and only an explicit release is valid.
	bool bAutoReleaseEnabled = true;
	// How long the tension has stayed above the threshold, and how long it must, where 0 disables the
	// tension release. Exceeding the threshold has to persist for TensionReleaseTime before the rope
	// lets go, so the progress towards that is displayed.
	float TensionOverTime = 0.0f;
	float TensionReleaseTime = 0.0f;
	// Whether ComputePull succeeded, meaning the anchor and direction are valid; it is true even at
	// zero tension.
	bool bPullValid = false;
	// The point the force is applied at, which is the anchor in world space.
	FVector PullPoint = FVector::ZeroVector;
	// The unit pull direction after smoothing, which is the direction actually applied.
	FVector PullDirection = FVector::ZeroVector;
	// The look-ahead direction before smoothing, kept to diagnose jitter against the smoothed value.
	FVector PullDirRaw = FVector::ZeroVector;
	// The end of the first straight leg, as the smoothed fractional aim position that feeds the
	// direction average.
	// It is not an integer node position: the aim index is smoothed and then interpolated between
	// nodes, so it does not land on one.
	FVector PullAimPoint = FVector::ZeroVector;
	// The integer aim node index before smoothing. Jumping between frames signals an unstable
	// direction, and it is the raw source of the fractional position above.
	int32 PullAimNode = INDEX_NONE;
	// The tension in the anchor segment.
	float PullTension = 0.0f;
	// The tether response, where 0 means disabled. For display.
	// How far the required path exceeds the available rope length (cm), where 0 means not taut.
	float TetherOvershoot = 0.0f;
	// This frame's tether tension, either lambda divided by dt or the measured force of the ragdoll
	// physics constraint (kg*cm/s^2), together with its limit. For display.
	float TetherTension = 0.0f;
	float MaxTetherTension = 0.0f;
	// Authoritative material-constraint backend and the PrePhysics motion it rejected.
	FString ConstraintBackend;
	float AttemptedOutwardSpeed = 0.0f;
	float AttemptedViolation = 0.0f;
	// The requested active pull force, where 0 means no input. It is the value SetActivePull stored and
	// is independent of whether it was applied.
	float ActivePullForce = 0.0f;
	// Whether that request passed this frame's taut gate and was actually applied. Reporting the
	// request alone would make "not taut but applied through the bypass setting" and "input present but
	// blocked" look like the same number.
	bool bActivePullApplied = false;
	// The taut gate state, which is the condition for applying active pull; the same latch as
	// IsPullTaut.
	bool bPullTaut = false;
	// The whole-chain geometric taut gate, a shared precondition for traction, both the tether and
	// active pull. It compares the sum of the corner-to-corner leg chords against the rest length.
	bool bChainTaut = false;
	// The sum of the corner-to-corner leg chords from the anchor to the hand (cm), which is the
	// observation behind bChainTaut.
	float TautChordLen = 0.0f;
	// The rest length of the free span from the hand to the anchor (cm), that is AnchorNode multiplied
	// by SegmentLength.
	float FreeRestLen = 0.0f;
	// The minimum segment tension across the free span. 0 means something is slack, so the tension is
	// not reaching the hand, whether from a kink or a partial stretch.
	float MinFreeTension = 0.0f;
	// The largest sag on any leg (cm), measured as how far an interior node departs from its leg chord.
	// It is the direct observation behind how straight the rope looks.
	float MaxLegSag = 0.0f;
	// The distance release limit (cm), where 0 means disabled. For display.
	float DistanceReleaseSlack = 0.0f;

	//~ Colliders, those this rope queried this frame ------------------------
	TArray<FRopeDebugCollider> Colliders;

	//~ Per-node proximity from the debug CPU re-query, for diagnosing sticking nodes ----
	TArray<FRopeNodeProximityDebug> NodeProximity;
	// The extra radius added for the re-query (cm). The screen uses it to state that these are the
	// results of a query at radius plus this margin rather than solver contacts.
	float ProximityQueryMargin = 0.0f;

	//~ The wrap axis, chosen by ResolveWrappingAxis during Wrapping, drawn as a line by the wrap view ---
	// Valid during the Wrapping phase only, indicated by bHasWrapAxis. It exists to confirm by eye
	// which axis the rope is wrapping about.
	bool    bHasWrapAxis = false;
	FVector WrapAxisOrigin = FVector::ZeroVector;
	FVector WrapAxisDirection = FVector::ForwardVector;
	// The segment length used to derive the length of the axis visualization. The debugger draws an axis
	// proportional to the rope's scale, as the larger of 80 and six times this value.
	float   WrapAxisSegmentLength = 0.0f;
};
