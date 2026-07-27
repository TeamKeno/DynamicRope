// Copyright Epic Games, Inc. All Rights Reserved.
//
// Logic for the Wrapping phase. Starting from the latch anchor established during Contacting, it
// progressively builds the wrap path, either as a composite analytic helix or a sequential surface
// vector field, moves the front of the rope along that path, masks the mass of wrapped nodes, and
// assembles the commit seed (FRopeWrapState). This is logic rather than physics, and is the stage
// before FRopeWrapController, which takes over once wrapped.
//
// It follows the same pattern as FRopeWrapController: a class with no UObject dependency that owns
// its working state (FRopeWrappingState) by value. Phase transitions and event broadcasts are
// decided by URopeComponent, while this class deals only with state and geometry. UObject context,
// namely the wrap config, the collider snapshot and the name used in logs, is injected per call
// through FContext.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

// Wrap targets are abstract: mesh parameters are generalized to USceneComponent. Functions that need
// a skeletal graph API, such as those choosing an axis or candidate bones, resolve it through the
// structural queries in RopeWrapTargets:: (parent and child keys). A static target has an empty
// graph and falls back naturally to a single bone; the assumption that a target is skeletal is
// isolated in Core/RopeWrapTarget.cpp.
class USceneComponent;
class IRopeCollider;

class DYNAMICROPE_API FRopeWrappingPhase
{
public:
	/** Plain-data working state for Wrapping. URopeComponent::ResetTransientPhaseState resets it on a
	 *  phase transition. */
	FRopeWrappingState State;

	/**
	 * Context for a single call: a bundle of non-owning references valid only for that call.
	 * The designer settings themselves stay as UPROPERTYs on URopeComponent and are only referenced
	 * from here.
	 */
	struct FContext
	{
		/** Wrap tuning: pitch, tail delay, step budget and contact radius. */
		const FRopeWrapConfig& Config;

		/** The frame's collider snapshot, used for surface projection. */
		const TArray<IRopeCollider*>& Colliders;

		/** Tube radius, that is how far the rope centre is held off the surface. */
		float SurfaceOffset;

		/** Log context, the component's name. */
		FString OwnerName;

		/** Set on calls such as the preview, which treat a partial path as a normal result. */
		bool bSuppressPathFailureLog = false;

		/** Passes the Flight whip guide spline's plane normal through to runtime wrapping. */
		bool bHasGuidePlaneNormal = false;
		FVector GuidePlaneNormal = FVector::RightVector;

		/**
		 * A snapshot of the travel frame at the moment of capture. Non-owning, pointing at the
		 * component's CaptureTravelFrame and only non-null while that is valid.
		 * Consumed by CaptureTravelPlane alone, which puts the axis origin at the centre of the contact
		 * region rather than at the latch bone, so that wrapping around two legs runs the axis through
		 * the centre of the pair, and takes the winding sign from the capture velocity rather than the
		 * latch tangent.
		 * BoneCenteredGuidePlane, the default, never reads it, so single-bone assisted behaviour is
		 * unchanged.
		 */
		const FRopeCaptureTravelFrame* TravelFrame = nullptr;

		/** The contact query radius as resolved at the component boundary, where a ContactQueryRadius
		 *  of 0 means automatic and is derived from the render radius. 0 here means an unresolved call,
		 *  such as from a unit test, and falls back to the raw config value through GetContactRadius. */
		float ResolvedContactRadius = 0.0f;

		/** Eligibility for composite multi-bone wrapping, which treats several bones as one pose-space
		 *  island. Only enabled in FullSimulation; the other modes use the sequential multi-bone path. */
		ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::FullSimulation;

		/** The single accessor for the contact radius used by the path build and by projection. */
		float GetContactRadius() const
		{
			return ResolvedContactRadius > 0.0f ? ResolvedContactRadius : Config.ContactQueryRadius;
		}

		/** The single accessor for the per-frame step budget used by the path build. The preview puts a
		 *  large value into its own copy of the config so it completes in one go. */
		int32 GetPathBuildStepsPerFrame() const
		{
			return Config.WrappingPathBuildStepsPerFrame;
		}
	};

	/**
	 * Starts wrapping: seeds the state with the bone, mesh and duration, and begins the progressive
	 * path build from the latch anchor, running until the first path point and anchor are secured. On
	 * success it also initializes stability tracking.
	 * LatchAnchor is the only source for the dominant target's mesh and bone.
	 * With MaxWrapSeeds above 1, secondary seed anchors must be loaded into
	 * State.SecondarySeedAnchors before this call, because it reads them to clamp the path length
	 * (NumTailNodes) to end before the first secondary node.
	 * @return false when the path build cannot start, in which case the caller must discard the state
	 *         and return to Flight.
	 */
	bool Begin(const FRopeSurfaceAnchor& LatchAnchor, float Duration,
		const FRopeSimState& Sim, const FContext& Ctx);

	/** Whether wrapping can continue: active, with a surviving mesh and a valid bone. */
	bool IsStillValid() const
	{
		return State.IsActive() && State.Mesh.IsValid() && !State.BoneName.IsNone();
	}

	/** Advances the progressive path and anchor generation by one frame's budget, derived from the
	 *  configured steps per frame. */
	void AdvancePathBuild(const FRopeSimState& Sim, const FContext& Ctx);

	/** The per-frame path build step budget: a size-proportional baseline of total work divided by
	 *  four, scaled by the configured steps per frame. It is a multiplier rather than a floor, so the
	 *  configured value still has effect on large ropes. Pure, and covered by unit tests. */
	static int32 ComputePathStepBudget(int32 NumTailNodes, int32 StepsPerFrame);

	/** Advances the front along the path and writes the on-path targets, including the surface offset,
	 *  for the nodes after the latch into OutFrame. */
	void ApplyWrappingMotionOverrides(const FRopeSimState& Sim, float DeltaTime, const FContext& Ctx, FRopeNodeOverrideFrame& OutFrame);

	/** Makes only the wrapping nodes that actually have a position override, the established anchors,
	 *  and the start pin kinematic. Tail and virtual path nodes the path has not reached yet are left
	 *  to the solver so they keep their rest length. */
	void ApplyWrappingKinematicMask(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const;

	/** Accumulates the time the anchor span has stayed unchanged, in StableTime. A supporting measure
	 *  for the commit decision. */
	void UpdateAnchorSpanStability(float DeltaTime);

	/** Commit conditions: anchors secured, path build finished, front arrived, and either the motion
	 *  completed or the settling timeout elapsed. */
	bool IsReadyToCommit(const FRopeSimState& Sim, const FRopeWrapConfig& Config) const;

	/**
	 * Reports whether to abort after a path build failure, which happens when the angle wrapped up to
	 * the last successful point falls short of the minimum and committing would leave the rope merely
	 * stuck to the target while looking wrapped. The measure is an angle rather than a number of turns
	 * because that is independent of target size; see the comment on
	 * FRopeWrapConfig::FailedWrapMinAngleDeg.
	 */
	bool ShouldAbortFailedShortWrap(const FRopeSimState& Sim, const FContext& Ctx,
		float MinRequiredAngleDeg, float& OutAngleDeg) const;

	/** Returns the accumulated wrapped angle of the built path, in degrees, approximating it from the
	 *  last successful path and anchor distances when no accumulated value exists. It is public so the
	 *  early abort above and the commit quality gate (CommitMinWrapAngleDeg, in
	 *  URopeComponent::CommitWrapping) use the same measure. The angle in the commit log is this value
	 *  as well. */
	bool ComputeBuiltPathWrapAngle(const FRopeSimState& Sim, const FContext& Ctx, float& OutAngleDeg) const;

	/**
	 * A shape-based measure of how well the target is bound: the angular coverage the path points
	 * actually enclose around the wrap axis, in degrees from 0 to 360. The path point angles are
	 * sorted, the largest gap is found, and it is subtracted from 360. Unlike the accumulated angle it
	 * is not inflated by oscillation or backtracking over the surface, so it answers directly whether
	 * a gap remains for the target to escape through. Bridge (chord) points are included, since the
	 * stretch a chord crosses is also a direction the rope blocks. Consumed by the commit gate
	 * (CommitMinWrapCoverageDeg) and by the transition log.
	 * @return false when there are fewer than two path points, or the axis is degenerate because every
	 *         point lies on it, in which case the caller skips the gate.
	 */
	bool ComputeWrapEnclosureCoverage(float& OutCoverageDeg) const;

	/**
	 * Assembles the seed for the Wrapped handoff from the current anchors, which is the input to
	 * FRopeWrapController::BeginWrap.
	 * With no valid nodes the returned seed has empty Anchors, which the caller checks and aborts on.
	 */
	FRopeWrapState BuildCommitSeed(const FRopeSimState& Sim) const;

	/** On abort, writes the return of the anchored nodes to the solver into OutFrame, setting InvMass
	 *  to 1 and Prev to Pos so they do not jump. */
	void ReleaseAnchoredNodesToSolver(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const;

	/** Builds a complete target centerline using the same helix/vector-field path code as runtime wrapping. */
	bool BuildPreviewCenterline(const FRopeSurfaceAnchor& LatchAnchor,
		const FRopeSimState& Sim, const FContext& Ctx, TArray<FVector>& OutCenterline) const;

private:
#if WITH_DEV_AUTOMATION_TESTS
	// Verifies directly that the surface gap in the current pose forms a composite island without any
	// notion of transition depth.
	friend class FRopeWrappingPoseSpaceIslandTest;
	// Verifies that discarding a composite path reinitializes around the original latch bone alone.
	friend class FRopeWrappingSingleBoneFallbackTest;
	// Verifies the stored coordinates of surface, bridge and virtual path points and the centreline
	// conversion contract.
	friend class FRopeWrappingPathPointCoordinateContractTest;
	// Verifies that a virtual run is produced once and that bridge ownership is handed to the
	// component.
	friend struct FRopeComponentRefactorTestSeam;
#endif

	struct FSurfaceVectorFieldBoneCandidate
	{
		FName Bone = NAME_None;
		int32 Depth = 0;
		float GraphCost = 0.0f;
		bool bCurrentBone = false;
	};

	//~ Progressive path build, split across frames. Begin starts it and AdvancePathBuild advances it
	//~ by one budget.
	bool BeginProgressiveWrapPathBuild(const FRopeSurfaceAnchor& LatchAnchor,
		const FRopeSimState& Sim, const FContext& Ctx);

	/** Advances the ideal helix raw probe by one step, independently of Path.Num(). It emits zero or
	 *  more output path points, and only when the actual projected centreline arc crosses a
	 *  SegmentLength boundary. */
	bool AdvanceCompositeAnalyticHelixProbeStep(const FRopeSimState& Sim, const FContext& Ctx);

	struct FCompositeHelixStepKinematics
	{
		float Radius = 0.0f;
		float BaseTangentialStep = 0.0f;
		float CircumferenceStep = 0.0f;
		float AxisStep = 0.0f;
		float AngleStepRad = 0.0f;
	};

	static int32 ComputeCompositeRadiusEntryStepCount(
		float StartRadius, float TargetRadius, float StepDistance);

	static FCompositeHelixStepKinematics EvaluateCompositeHelixStep(
		int32 StepIndex, float StepDistance, int32 RadiusEntryStepCount,
		float StartRadius, float TargetRadius, float PreviousRadius,
		float PitchScale, float WindingSign);

	bool InitializeProgressiveWrapPath(const FRopeSurfaceAnchor& LatchAnchor,
		const FRopeSimState& Sim, const FContext& Ctx);

	bool AdvanceSequentialSurfaceVectorFieldPath(int32 StepBudget, const FRopeSimState& Sim, const FContext& Ctx);

	/** On terminal failure of the composite analytic helix, discards the existing path and anchors and
	 *  retries around the original latch bone alone. */
	bool RestartPathBuildAsSingleBoneFallback(const FRopeSimState& Sim, const FContext& Ctx,
		const TCHAR* CompositeFailureReason);

	bool ProcessPathPointForAnchoring(int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx);

	/** Prefers the mesh named by the anchor, falling back to the wrapping state's mesh. */
	static const USceneComponent* ResolveWrappingMesh(
		const FRopeWrappingState& State, const FRopeSurfaceAnchor& Anchor);

	/** Finds each newly closed virtual run in Path exactly once and records it in
	 *  State.VirtualBridgeRuns. */
	void CollectCompletedVirtualBridgeRuns();

	/** Records the end of a path build, as complete on success or failed on failure, clearing the
	 *  active flag either way. Readers such as the commit decision only consume the three flags as
	 *  "has the build finished", that is their disjunction; nothing distinguishes the individual
	 *  combinations. */
	void FinishPathBuild(bool bFailed, const TCHAR* FailureReason = nullptr);

	//~ Wrap geometry
	FVector ComputeSurfaceVectorFieldTangent(const FVector& AxisOrigin, const FVector& AxisDirection,
		const FVector& LatchRadial, float WindingSign, const FVector& SurfaceWorld,
		const FVector& NormalWorld, const FContext& Ctx, FVector& InOutCircumferenceDir) const;

	/**
	 * Derives the wrap axis. Depending on Config.WrappingAxisSource, the same travel plane normal is
	 * placed at a different origin:
	 *  0) CaptureTravelPlane pins the axis at the centre of the captured contact region or collider
	 *     cluster, used by composite wrapping.
	 *  1) BoneCenteredGuidePlane stands the axis at the latch bone location and re-resolves it on every
	 *     bone transition, used by assisted single-bone wrapping.
	 *  2) The bone-to-parent axis on a skeletal target; on a non-skeletal one, whichever of the
	 *     component's basis axes is most perpendicular to the latch normal.
	 *  3) The bone's local X axis.
	 * On the surface vector field path this is not a one-off at latch time: on every bone transition
	 * ReseedWrappingAxisOnBoneTransition calls it again against the new bone, giving a rolling axis.
	 * Under CaptureTravelPlane the captured travel plane axis is preserved across reseeds.
	 */
	bool ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx,
		FVector& OutAxisOrigin, FVector& OutAxisDirection) const;

	/**
	 * Re-resolves the wrap axis against the new bone immediately after a transition, giving a rolling
	 * axis. A field that keeps circling the old bone's axis drifts away from the surface after the
	 * transition and tends to terminate early on a projection failure. So ResolveWrappingAxis is run
	 * again with a synthetic anchor that moves the current path point's surface frame into the new
	 * bone's local space; the new axis is signed to align with the previous one, keeping the pitch
	 * drift continuous, and the winding is re-elected from the current travel tangent so the wrap
	 * direction cannot flip at the transition. If deriving an axis fails, the existing axis is kept,
	 * which behaves as a single fixed axis.
	 */
	void ReseedWrappingAxisOnBoneTransition(FName Bone, const USceneComponent* Mesh, const FContext& Ctx);

	/** Returns the Flight guided spline's plane normal as the wrap axis direction.
	 *  Under CaptureTravelPlane, and with a capture snapshot available, the origin is replaced by the
	 *  centre of the contact region or collider cluster, so that the axis runs through the centre of a
	 *  pair even when contact starts on one side only, as when wrapping two legs. */
	static bool FindGuidePlaneAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx, const USceneComponent* Mesh,
		FVector& OutAxisOrigin, FVector& OutAxisDirection);

	/** A collider's representative centre: the midpoint of a capsule, the centre of a box, or the
	 *  centre of an SDF's bounds. Used to correct the cluster origin. */
	static bool GetColliderCenter(const IRopeCollider& Collider, FVector& OutCenter);

	void OrientWrappingAxisByTail(const FRopeSurfaceAnchor& LatchAnchor, const FRopeSimState& Sim,
		const USceneComponent* Mesh, FVector& InOutAxisDirection) const;

	/** Collects skeleton parent and child candidate bones for the non-composite path. The single-bone
	 *  fallback does not use this path. */
	void GatherSurfaceVectorFieldBoneCandidates(FName CurrentBone, const USceneComponent* Mesh,
		TArray<FSurfaceVectorFieldBoneCandidate>& OutCandidates, const FContext& Ctx) const;

	/**
	 * Assembles the pose-space collider island at the moment of contact. Within a thin slab about the
	 * current wrap axis, it connects only those colliders whose actual surface gap is smaller than the
	 * rope diameter, or where the slack available is less than the extra path needed to thread through
	 * an open gap.
	 * The result is independent of how many bone transitions occur, and is fixed for the duration of
	 * the wrap attempt.
	 */
	void GatherPoseSpaceWrapIsland(const FRopeSurfaceAnchor& LatchAnchor, const FRopeSimState& Sim,
		const USceneComponent* Mesh, TArray<FName>& OutBones,
		TArray<FRopeWrapIslandMember>& OutMembers,
		TArray<FRopeWrapIslandPortal>& OutPortals, float& OutAvailableSlack,
		const FContext& Ctx) const;

	/** For the composite failure fallback only. Projects against the original latch bone's collider
	 *  alone, with no candidate graph. */
	bool ProjectWrapPointToLatchBone(const USceneComponent* Mesh,
		const FRopeSimState& Sim, const FContext& Ctx,
		FVector& InOutSurfaceWorld, FVector& InOutNormalWorld, FVector& InOutTangentWorld,
		FVector& InOutCircumferenceDir, FName& InOutBone, const USceneComponent*& OutMesh) const;

	/** Scores the surface projections of the candidate bones, together with the graph cost and
	 *  hysteresis, to choose the path point's bone and mesh. */
	bool ProjectWrapPointToSurfaceMultiBone(FName CurrentBone, const USceneComponent* Mesh,
		const FRopeSimState& Sim, const FContext& Ctx,
		FName PreviousBone, float DistanceSinceLastTransition, const FVector& RopeNodeWorld,
		const FVector& PreviousNormalWorld, const FVector& PreviousTangentWorld,
		FVector& InOutSurfaceWorld, FVector& InOutNormalWorld, FVector& InOutTangentWorld,
		FVector& InOutCircumferenceDir, FName& InOutBone, const USceneComponent*& OutMesh) const;

	bool ProjectWrapPointToSurface(FName Bone, const USceneComponent* Mesh,
		const FRopeSimState& Sim, const FContext& Ctx,
		FVector& InOutSurfaceWorld, FVector& InOutNormalWorld) const;

	//~ Front motion
	void AdvanceWrappingFront(float DeltaTime, const FRopeSimState& Sim, const FContext& Ctx);

	/** Resolves a moving anchor frame into a world-space path point for the current frame. */
	bool ResolveWrappingAnchorPoint(const FRopeSurfaceAnchor& Anchor,
		FRopeWrapPathPoint& OutPoint) const;

	/** Builds this frame's world-space path in linear time, using the sorted order of the path points
	 *  and anchors. */
	bool BuildResolvedWrappingPath(TArray<FRopeWrapPathPoint>& OutResolvedPath) const;

	/** Only virtual points already store a centreline; normal surface and bridge points have the normal
	 *  offset applied. */
	static FVector GetPathPointCenterlineWorld(const FRopeWrapPathPoint& Point, float SurfaceOffset);

	/** Converts a centreline back into the storage convention of FRopeWrapPathPoint: unchanged for a
	 *  virtual point, and with the offset subtracted otherwise. */
	static FVector EncodePathPointPositionFromCenterline(const FVector& CenterlineWorld,
		const FVector& NormalWorld, bool bVirtual, float SurfaceOffset);

	/** Interpolates the resolved path, which is sorted by distance, using a binary search. */
	static bool SampleResolvedWrappingPath(const TArray<FRopeWrapPathPoint>& ResolvedPath,
		float DistanceFromLatch, float SurfaceOffset, FRopeWrapPathPoint& OutPoint);

	static void InterpolateWrappingPathPoints(const FRopeWrapPathPoint& LowerPoint,
		const FRopeWrapPathPoint& UpperPoint, float SampleDistance, float SurfaceOffset,
		FRopeWrapPathPoint& OutPoint);

	/** Reusable buffer for ApplyWrappingMotionOverrides, so path points and anchors are not resolved
	 *  again every frame. */
	TArray<FRopeWrapPathPoint> ResolvedPathScratch;
};
