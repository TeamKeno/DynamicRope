// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"

class USceneComponent;

/** A latched wrap node. Frozen in bone-local space so it follows skinning without colliding again. */
struct FRopeLatchNode
{
	int32   NodeIndex = INDEX_NONE;
	FName   Bone = NAME_None;
	FVector BoneLocalPos = FVector::ZeroVector;
};

/** The surface anchor the Wrapping and Wrapped phases use to pin a node to a surface: a bone-local
 *  frame plus a progress measure along the rope. */
struct FRopeSurfaceAnchor
{
	int32 NodeIndex = INDEX_NONE;

	FName Bone = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	/** The bone-local anchor frame on the SDF surface: position, normal and tangent. */
	FVector LocalSurfacePosition = FVector::ZeroVector;
	FVector LocalNormal = FVector::UpVector;
	FVector LocalTangent = FVector::ForwardVector;

	/** The direction the tail extends in behind the composite wrapping front. It stores the ideal
	 *  helix guide, kept separate from the surface tangent, in bone-local space, and is not used by
	 *  the Wrapped physics or surface frame. */
	FVector LocalWrappingGuideTangent = FVector::ForwardVector;
	bool bHasWrappingGuideTangent = false;

	/** The world position at the moment wrapping started, used as the interpolation start point of the
	 *  front motion. */
	FVector StartWorldPosition = FVector::ZeroVector;

	/** Progress along the rope, used by multi-turn wrapping and length calculations. It is stored as
	 *  the measure at commit time, so it goes stale after reeling changes the length. */
	float RopeDistance = 0.f;

	/** How far to hold the rope centreline off the surface, normally the rope radius. */
	float SurfaceOffset = 0.0f;

	/** Pierce only: the bone-local transform, including scale, of the tip mesh origin frozen at the
	 *  moment it embedded. It is the single source for the tip's render pose while wrapped, freezing
	 *  the rotation while following the bone. Left as identity and unused outside Pierce. */
	FTransform LocalMeshTransform = FTransform::Identity;
};

/** One point on the wrapping path: a world frame, the bone and mesh it belongs to, and the distance
 *  travelled from the latch. */
struct FRopeWrapPathPoint
{
	/**
	 * When bVirtual is false this is the reference position before the rope centreline's normal
	 * offset: the actual projected surface for a normal point, and for a bridge point the virtual
	 * reference obtained by subtracting the offset from the centreline in mid-air. When bVirtual is
	 * true there is no surface, so the already-complete rope centreline is stored instead. Consumers
	 * of the centreline must use the shared conversion in FRopeWrappingPhase rather than
	 * reimplementing this convention.
	 */
	FVector SurfaceWorld = FVector::ZeroVector;
	FVector NormalWorld = FVector::UpVector;
	FVector TangentWorld = FVector::ForwardVector;

	/** The ideal helix direction, used by the composite wrapping tail only, which does not follow the
	 *  fine irregularities of the SDF normal. */
	FVector WrappingGuideTangentWorld = FVector::ForwardVector;
	bool bHasWrappingGuideTangent = false;

	/**
	 * The actual surface bone this path point was projected onto.
	 * Both the composite analytic helix and the sequential surface vector field store the real
	 * projection result here. It is NAME_None for a virtual point, where the composite radial ray
	 * missed. ProcessPathPointForAnchoring later stores a bone-local anchor based on this value.
	 */
	FName Bone = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	float DistanceFromLatch = 0.0f;

	/**
	 * The wrapping animation phase from the latch to this path point, in radians. The composite path
	 * stores the ideal helix angle; the sequential surface vector field stores the forward signed
	 * angle along the winding direction, interpolated by the resample alpha of the actual centreline
	 * arc. DistanceFromLatch preserves the physical rope spacing, while this value drives the
	 * angle-to-distance mapping the two modes share for the front.
	 */
	float WrapAngleFromLatchRad = 0.0f;

	/**
	 * A mid-air bridge (chord) path point, which only occurs while WrappingMaxGapBridgeDistance is
	 * above 0. It was produced by continuing straight along the tangent with no surface projection.
	 * It takes part in the front motion as a position target but produces no anchor, so after the
	 * commit the nodes in this stretch remain free rope.
	 */
	bool bBridge = false;

	/**
	 * A no-anchor point of the composite analytic helix, produced when the radial SDF ray does not
	 * intersect the surface, in which case the ideal helix position is kept only for the path and the
	 * debug phase. From Wrapping onwards the node matching this point receives neither a position
	 * override nor a surface anchor, and is handled directly by the solver.
	 */
	bool bVirtual = false;
};

/**
 * A contiguous run of virtual points on a composite path, closed at both ends by real surface points.
 * The path build produces it once, and URopeComponent then owns the same bridge from Wrapping through
 * Wrapped.
 */
struct FRopeVirtualBridgeRun
{
	int32 LeftNodeIndex = INDEX_NONE;
	int32 RightNodeIndex = INDEX_NONE;
	TArray<int32> VirtualNodeIndices;
};

/** The verdict on the pose-space gap actually evaluated at contact time. Read both by the island
 *  connectivity test and by failure diagnostics. */
enum class ERopeWrapIslandPortalState : uint8
{
	Open,
	ClosedGeometry,
	ClosedReachability
};

/** A contact-time volume snapshot of a candidate island SDF collider, so the runtime composite
 *  cross-section calculation needs no extra sampling. */
struct FRopeWrapIslandMember
{
	FName Bone = NAME_None;
	FBox WorldBounds = FBox(EForceInit::ForceInit);

	/** The SDF grid's actual oriented bounds. Falls back to WorldBounds for a collider with no SDF
	 *  accessor. */
	bool bHasOrientedSDFBounds = false;
	FVector SDFCenter = FVector::ZeroVector;
	FVector SDFHalfExtent = FVector::ZeroVector;
	FQuat SDFRotation = FQuat::Identity;
};

/** A snapshot of the two surface projections and the length verdict already computed while building
 *  an island. */
struct FRopeWrapIslandPortal
{
	FName BoneA = NAME_None;
	FName BoneB = NAME_None;
	FVector SurfacePointA = FVector::ZeroVector;
	FVector SurfacePointB = FVector::ZeroVector;
	ERopeWrapIslandPortalState State = ERopeWrapIslandPortalState::Open;
	float SurfaceGap = 0.0f;
};

/** Working state of the Wrapping phase (FRopeWrappingPhase::State): path build progress, accumulated
 *  anchors, and the inputs to the commit decision. */
struct FRopeWrappingState
{
	FName BoneName = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	TArray<FRopeSurfaceAnchor> Anchors;
	FRopeSurfaceAnchor LatchAnchor;
	TArray<FRopeWrapPathPoint> Path;

	/** Bounded virtual runs, discovered once each while generating the path and accumulated here. */
	TArray<FRopeVirtualBridgeRun> VirtualBridgeRuns;
	/** Incremental scan state, since the path is appended to each frame and only the new stretch needs
	 *  examining. */
	int32 VirtualBridgeScanPathIndex = 0;
	int32 VirtualRunStartPathIndex = INDEX_NONE;
	int32 VirtualRunLeftPathIndex = INDEX_NONE;

	/**
	 * Secondary seed anchors, used when MaxWrapSeeds is above 1: anchors adopted after contacting a
	 * different bone further along the tail than the dominant latch. They take no part in the path
	 * build or the front motion, are held against their own bone throughout Wrapping, and join the
	 * path anchors in BuildCommitSeed; BeginWrap and Hold already support a per-anchor bone and mesh.
	 * They must be filled in before Begin, because BeginProgressiveWrapPathBuild reads them to clamp
	 * the path length (NumTailNodes) to end before the first secondary seed node, which keeps path
	 * anchors and secondary anchors from claiming the same nodes.
	 */
	TArray<FRopeSurfaceAnchor> SecondarySeedAnchors;

	int32 NumTailNodes = 0;
	int32 LastAnchoredPathPointCount = 0;
	bool bPathBuildActive = false;
	bool bPathBuildComplete = false;
	bool bPathBuildFailed = false;

	/** Machine-readable cause of the last path build failure, consumed by the final abort log. */
	FString PathBuildFailureReason;

	/** The actual accumulated length of the rope centreline polyline established by projection (cm).
	 *  The next node on the path is created by resampling when this distance crosses a SegmentLength
	 *  boundary. */
	float PathCurrentDistance = 0.0f;

	/** The nominal distance the predictor or composite sweep attempted to advance (cm). It grows even
	 *  when the projection stays put, so the surface search phase cannot stall. */
	float PathSweepDistance = 0.0f;

	/** Whether the previous raw probe of the composite analytic helix was a virtual point that failed
	 *  SDF projection. It preserves the kind of each stretch through the arc-length resampling of the
	 *  raw polyline, which is independent of the output path. */
	bool bPathCompositeRawPointVirtual = false;

	/** The ideal helix tangent of the previous raw probe, before projection, used to interpolate the
	 *  guide for the next arc-length output point. */
	FVector PathCompositeRawGuideTangentWorld = FVector::ForwardVector;
	float FrontDistance = 0.0f;
	/** The shared animation phase the angle-to-distance front has advanced so far (radians). It is
	 *  independent of the physical rope distance, and the single and composite modes use the same
	 *  angular velocity policy. */
	float FrontWrapAngleRad = 0.0f;
	/** The physical front travel distance the current final path requires (cm). This is the final
	 *  distance target checked at commit time, not the temporary end of a path build in progress. */
	float FrontTargetDistance = 0.0f;
	/** The animation phase the current final path requires (radians). Storing it separately from the
	 *  distance means a full turn is judged complete at the same point regardless of how the path
	 *  density changed under projection. */
	float FrontTargetWrapAngleRad = 0.0f;
	/** Whether the front currently uses the per-point angle-to-distance mapping. A degenerate or
	 *  legacy path for which this is false falls back safely to the distance-based completion
	 *  policy. */
	bool bFrontUsesAngleMapping = false;
	/** The elapsed Wrapping phase time at which both the angle and the distance first reached their
	 *  final targets. The post-front settling time is measured from this point; negative means the
	 *  targets have not been reached yet. */
	float FrontReachedTargetElapsed = -1.0f;

	/** The elapsed Wrapping phase time at which the front first actually advanced. It drives the
	 *  per-mode seconds and turns logs of an angle-mapped front; negative means the front has not
	 *  started yet. */
	float FrontMotionStartElapsed = -1.0f;
	bool bFrontMotionStartLogged = false;
	bool bFrontMotionCompletionLogged = false;

	FVector PathSurfaceWorld = FVector::ZeroVector;
	FVector PathNormalWorld = FVector::UpVector;
	FVector PathTangentWorld = FVector::ForwardVector;
	FVector PathCircumferenceDir = FVector::ForwardVector;
	FVector PathAxisOrigin = FVector::ZeroVector;
	FVector PathAxisDirection = FVector::ForwardVector;
	FVector PathLatchRadial = FVector::ForwardVector;

	/**
	 * Tracks which bone the current surface point sits on while integrating the surface vector field.
	 * The candidate bones for the next step are chosen from the skeleton graph neighbourhood around
	 * this one.
	 */
	FName PathCurrentBone = NAME_None;

	/**
	 * The bone most recently left. A new candidate that is immediately this bone again is likely an
	 * A to B to A oscillation, so the scoring stage adds ImmediateBoneReturnPenalty to damp the
	 * flicker.
	 */
	FName PathPreviousBone = NAME_None;
	TWeakObjectPtr<const USceneComponent> PathCurrentMesh = nullptr;

	/**
	 * The bones making up the composite wrap target, assembled from the pose at the moment of contact.
	 * This is not a skeleton lineage or a transition depth: it is the collider island whose surfaces,
	 * expanded by the rope thickness within the throw slab, either actually touch each other or cannot
	 * be threaded between with the slack currently available.
	 * The surface vector field path lazily projects against this whole list at every step, treating it
	 * as a single column surface.
	 */
	TArray<FName> PathWrapIslandBones;

	/** Snapshots from the actual island test at contact time. Members feed the runtime composite
	 *  cross-section calculation and portals feed failure diagnostics; neither creates a separate path
	 *  or SDF. */
	TArray<FRopeWrapIslandMember> PathWrapIslandMembers;
	TArray<FRopeWrapIslandPortal> PathWrapIslandPortals;

	/** The slack length available on the unpinned rope, computed while assembling the composite island
	 *  (cm). Kept for debugging and to reproduce the portal verdict. */
	float PathAvailableSlack = 0.0f;

	/** When true the path uses the pose-space island composite analytic helix instead of the
	 *  sequential surface vector field. */
	bool bPathUsesPoseSpaceIsland = false;

	/** Whether the composite island path failed and the path is being rebuilt from scratch around the
	 *  single original latch bone. */
	bool bPathUsesSingleBoneFallback = false;

	/**
	 * The axis-perpendicular direction that sweeps the outline of the composite island, independent of
	 * the local tangent of any individual bone SDF. It rotates on its own at every step even where an
	 * arm's surface normal cancels the circumferential tangent.
	 */
	FVector PathCompositeSweepRadial = FVector::ForwardVector;

	/** The axial radius to start the SDF support projection from, outside the whole composite
	 *  cross-section (cm). */
	float PathCompositeProbeRadius = 0.0f;

	/** The maximum projection radius from the composite axis out to the SDF bounds of the whole island
	 *  (cm, excluding the probe margin). */
	float PathCompositeHelixRadius = 0.0f;

	/** The composite helix pitch, derived automatically from the axis-to-circumference ratio of the
	 *  tail direction at the moment of contact. */
	float PathCompositeHelixPitchScale = 0.0f;

	/** The axial extent of the whole island's SDF bounds, measured from the composite axis origin
	 *  (cm). */
	float PathCompositeAxisMinDistance = 0.0f;
	float PathCompositeAxisMaxDistance = 0.0f;
	bool bPathCompositeAxisRangeValid = false;

	/** How far the independent sweep radial has rotated from its starting point (radians), for
	 *  progress logging. */
	float PathCompositeSweepAngleRad = 0.0f;

	/** How many times an individual radial projection of the composite analytic helix failed and
	 *  produced a virtual point. */
	int32 PathCompositeProjectionFailureCount = 0;

	/**
	 * How far the path has advanced along the surface since the last bone transition (cm). The current
	 * bone is kept until MinBoneTransitionPathDistance is reached even when a new candidate looks
	 * better, which prevents the bone flickering every step or two.
	 */
	float PathDistanceSinceBoneTransition = 0.0f;

	float PathWindingSign = 1.0f;

	/**
	 * The accumulated length of the mid-air bridge (chord) currently being extended (cm). Reset to 0
	 * when the path re-enters the surface and the snap is accepted. Exceeding
	 * WrappingMaxGapBridgeDistance fails the path build; it stays at 0 while bridging is disabled.
	 */
	float PathBridgeDistance = 0.0f;

	/**
	 * The total wrapped angle integrated during the path build (radians). The sequential surface
	 * vector field adds the radial rotation about the rolling axis at each walking step, while the
	 * composite analytic helix stores the helix phase per node.
	 */
	float PathAccumulatedAngleRad = 0.0f;

	/** Angle diagnostics for the sequential surface vector field. PathAccumulatedAngleRad keeps the
	 *  absolute accumulation based on acos, while these record the signed, forward and reverse
	 *  components alongside it, with the winding direction taken as positive. */
	float PathSignedNetAngleRad = 0.0f;
	float PathForwardAngleRad = 0.0f;
	float PathReverseAngleRad = 0.0f;

	/** Statistics on the surface radius about the rolling axis actually used by the sequential steps. */
	float PathRadiusSum = 0.0f;
	float PathRadiusMin = 0.0f;
	float PathRadiusMax = 0.0f;
	int32 PathRadiusSampleCount = 0;
	int32 PathBoneTransitionCount = 0;

	float Elapsed = 0.0f;
	float Duration = 0.16f;
	float StableTime = 0.0f;

	int32 FirstNode = INDEX_NONE;
	int32 LastNode = INDEX_NONE;
	int32 LastStableFirstNode = INDEX_NONE;
	int32 LastStableLastNode = INDEX_NONE;
	int32 LastStableAnchorCount = 0;

	void Reset()
	{
		*this = FRopeWrappingState();
	}

	bool IsActive() const
	{
		return !BoneName.IsNone() && Anchors.Num() > 0;
	}
};

/**
 * The post-wrap data model, expressed as binding semantics. Created at the physics-to-logic handoff.
 * Two binding forms coexist: the Latched nodes and the surface Anchors that supersede them.
 */
struct FRopeWrapState
{
	FName                   BoneName = NAME_None;

	/** Legacy fallback that pins bone-local points. The current commit path fills in Anchors
	 *  instead. */
	TArray<FRopeLatchNode>  Latched;

	/** Surface anchors, the current mechanism. */
	TArray<FRopeSurfaceAnchor> Anchors;

	/** Maximum segment tension, refreshed every frame while wrapped. */
	float                   Tension = 0.0f;

	// The mesh owning BoneName, against which the wrap is held and tracked. It can belong to an actor
	// other than the rope's owner, and is resolved from the contact at decision time.
	// In a cross-actor wrap the target actor can be destroyed while still wrapped. Holding it as a raw
	// pointer would make Hold dereference a dangling pointer every frame, so it is held weakly and
	// becomes null safely on destruction. This keeps the struct plain data: a weak handle is not a hard
	// reference and does not keep the mesh alive.
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	bool IsWrapped() const { return Anchors.Num() > 0 || Latched.Num() > 0; }
	void Reset() { *this = FRopeWrapState(); }
};
