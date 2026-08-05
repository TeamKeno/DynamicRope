// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The contact candidate detection pipeline used during Flight. It finds actual contacts along the
// travel path produced by the solve (Detect), extrapolates the next positions of fast-moving and
// whip-guided nodes to add predicted contacts (AddPredicted), evaluates motion relative to the
// surface (EvaluateRelativeMotion), and then produces the tracker and the capture decision together
// (EvaluateCapture). It is called every frame from FinalizeSimFrame on the game thread, and produces
// the input to the Flight to Contacting transition.
//
// It follows the same pattern as the solver and the wrap controller and has no UObject dependency.
// It holds no state, so everything is static.
// Its inputs are only plain data (FRopeSimState), a collider snapshot, a parameter snapshot
// (FParams) and a view of the whip guide data (FWhipGuideView), so it can be unit tested without a
// world and behaves identically whether the positions come from the CPU solver or the GPU mirror.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeContactTrackingTypes.h"
#include "Core/RopeSimTypes.h"

class IRopeCollider;
class USceneComponent;

/** The policy for choosing the dominant target during a Flight capture. Assisted aiming names a
 *  preferred target and requires it. */
struct FRopeFlightCapturePolicy
{
	const USceneComponent* PreferredMesh = nullptr;
	FName PreferredBone = NAME_None;
	bool bRequirePreferred = false;
};

/** The Flight capture verdict produced by a single pass over the candidates. Gameplay transitions and
 *  observation consume the same tracker. */
struct FRopeFlightCaptureEvaluation
{
	FRopeContactTracker Tracker;
	bool bShouldCapture = false;
};

class DYNAMICROPE_API FRopeFlightContactDetector
{
public:
	/** A snapshot of the detection parameters. The designer settings themselves stay on
	 *  URopeComponent and are copied here per call. */
	struct FParams
	{
		/** WrapConfig.ContactQueryRadius: the contact query radius. */
		float ContactRadius = 3.0f;

		/** Tube radius, added to the broad-phase bounds margin. */
		float RopeRadius = 2.0f;

		/** WrapConfig.PredictiveContactFrames: how many frames to extrapolate for prediction. */
		float PredictiveContactFrames = 0.0f;

		/** WrapConfig.MinLatchNodes: the minimum number of contacting nodes needed to capture. */
		int32 MinLatchNodes = 1;

		/** Component forward, used for the degenerate case in ExpectedWrapTangent. */
		FVector FallbackForward = FVector::ForwardVector;

		/** WrapConfig.ContactSweepStep: the sample spacing of the detection sweep (cm). This is the key
		 *  value for preventing tunnelling; see the comment there. */
		float ContactSweepStep = 2.0f;

		/** WrapConfig.ContactMaxSweepSamples: the cap on sweep samples, which bounds the cost. */
		int32 ContactMaxSweepSamples = 16;

		/**
		 * The time span of one rope Verlet displacement (s), which is the substep delta time, equal to
		 * FixedDt. It is the bridge that converts SurfaceVelocity, defined in cm/s by the frozen contact
		 * contract, into the same units as the rope displacement (Positions minus PrevPositions).
		 * The key point is that the rope displacement is the last substep's delta, roughly v * FixedDt,
		 * not a frame's, so it must be converted with the substep delta time, (1/60) / Substeps, rather
		 * than the frame delta. Using the frame delta would overstate the surface velocity by a factor
		 * of Substeps whenever Substeps exceeds 1, which makes the relative tangential velocity and the
		 * direction score wrong. The simulation caller, FinalizeSimFrame, passes FixedDt. The default is
		 * a placeholder for callers where the delta is meaningless, such as the preview, which is a
		 * static snapshot with zero surface velocity.
		 */
		float SubstepDeltaTime = 1.0f / 60.0f;

	/**
	 * This frame's delta time (s). It is the bridge that converts the rope Verlet displacement, which
	 * is the last substep's delta of roughly v * SubstepDeltaTime, into a frame displacement, by
	 * multiplying by FrameDeltaTime / SubstepDeltaTime. It is used by the free-node branch of
	 * AddPredictedContactCandidates. Without it, PredictiveContactFrames would be interpreted in
	 * substeps and the lookahead would be applied a factor of Substeps too small. The guided-node
	 * branch differences frame-rate targets and needs no conversion.
	 * Callers working from a static snapshot, such as the preview, do not use prediction, so the
	 * default suffices.
	 */
	float FrameDeltaTime = 1.0f / 60.0f;
	};

	/**
	 * Converts the Verlet displacement stored for one solver substep into the displacement for the
	 * current game frame. Collider broad-phase gathering and predictive narrow-phase detection must
	 * use the same ratio or the latter can sweep through colliders that the former already discarded.
	 */
	static float ComputeFrameToSubstepRatio(float FrameDeltaTime, float SubstepDeltaTime);

	// A view of the whip guide's frame data, which is the input to the guided-node branch of
	// predictive contact. The pointers are not owned and need only stay valid for the duration of the
	// call. With the guide inactive, pass the defaults, that is all nullptr.
	// NextTargets holds the guide targets as of the next frame, the result of
	// FRopeWhipGuide::PreviewNextTargets, computed by the caller beforehand: the detector sees data
	// only, never the guide class.
	struct FWhipGuideView
	{
		const TArray<uint8>* GuidedNodeMask = nullptr;
		const TArray<FVector>* CurrentTargets = nullptr;
		const TArray<FVector>* PrevTargets = nullptr;
		const TArray<FVector>* NextTargets = nullptr;

		bool HasGuidedNodes() const { return GuidedNodeMask && GuidedNodeMask->Num() > 0; }
		bool IsGuidedNode(int32 NodeIndex) const
		{
			return GuidedNodeMask && GuidedNodeMask->IsValidIndex(NodeIndex) && (*GuidedNodeMask)[NodeIndex] != 0;
		}
	};

	/**
	 * The safety cap for the assisted exact-target sweep on the game thread. The general detector and
	 * the GPU keep their fixed budgets; only this few-collider path, which exists to avoid losing an
	 * asynchronous readback, preserves a 2 cm sample spacing over up to 512 cm of travel.
	 */
	static constexpr int32 ReliableGuidedSweepMaxSamples = 256;

	/** Collects actual contact candidates along the travel path between the positions before and after
	 *  the solve, that is from Sim.PrevPositions to Sim.Positions. */
	static void DetectContactCandidates(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
		const FParams& Params, TArray<FRopeContactCandidate>& OutCandidates);

	/**
	 * Adds actual candidates by testing the whip guide's real frame movement, already computed on the
	 * CPU as PrevTargets to CurrentTargets, together with the current guide centreline edges including
	 * the boundary between guided and solver-owned nodes. The GPU-resident positions and the
	 * asynchronous contact readback lag one to two frames and can drop intermediate frames, so this is
	 * the synchronous path that lets assisted aiming confirm its locked target within the same frame.
	 * A candidate matching an existing (node, bone, mesh) is merged into it.
	 */
	static void AddGuidedContactCandidates(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
		const FParams& Params, const FWhipGuideView& Whip, TArray<FRopeContactCandidate>& InOutCandidates);

	/**
	 * Adds actual candidates by testing only the nodes and edges of the current Sim centreline. During
	 * Contacting it does not replay the movement pulse from the last Flight frame's PrevTargets, and
	 * asks only whether the synchronized current pose genuinely touches the exact primary target.
	 */
	static void AddCurrentCenterlineContactCandidates(const FRopeSimState& Sim,
		const TArray<IRopeCollider*>& Colliders, const FParams& Params,
		TArray<FRopeContactCandidate>& InOutCandidates);

	/**
	 * Adds predicted contact candidates by sweeping the extrapolated next positions of fast-moving,
	 * tail and guided nodes, which promotes contacts that have not landed yet but are about to into
	 * the same candidate pipeline. A candidate matching an existing (node, bone, mesh) has its
	 * SourceMask merged, and its Source updated by the priority order Guided, Actual, Free.
	 */
	static void AddPredictedContactCandidates(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
		const FParams& Params, const FWhipGuideView& Whip, TArray<FRopeContactCandidate>& InOutCandidates);

	/** Fills in each candidate's tangential velocity relative to the surface and its wrap direction
	 *  score. */
	static void EvaluateRelativeMotion(const FRopeSimState& Sim, const FParams& Params,
		TArray<FRopeContactCandidate>& Candidates);

	/** Aggregates the candidates once to produce the dominant tracker and the capture decision
	 *  together. */
	static FRopeFlightCaptureEvaluation EvaluateCapture(const TArray<FRopeContactCandidate>& Candidates,
		const FParams& Params, const FRopeFlightCapturePolicy& Policy = {});

	/** Older C++ convenience API. New code should use EvaluateCapture so the tracker can be reused. */
	UE_DEPRECATED(5.7, "Use EvaluateCapture so gameplay and observation can share the tracker.")
	static bool ShouldCapture(const TArray<FRopeContactCandidate>& Candidates, const FParams& Params);

	/**
	 * No additional quality filter is applied at present. Kept for compatibility with external callers
	 * and always returns true.
	 */
	static bool PassesCaptureQualityGate(const FRopeContactTracker& Tracker,
		const TArray<FRopeContactCandidate>& Candidates, const FParams& Params);

	//~ Individual helpers, also used by debug collection in FinalizeSimFrame and by the Contacting
	//~ seed build. The per-frame node travel distance lives on FRopeSimState::NodeSpeed, since it
	//~ measures the chain itself; the "fast node" threshold test against SegmentLength remains this
	//~ detector's policy.
	/** Whether the node is near the tail of the rope, meaning one of the last four. Tail nodes are
	 *  always tested regardless of speed. */
	static bool IsTailNode(const FRopeSimState& Sim, int32 NodeIndex);

	/** Whether the bounds of the segment from Prev to Pos overlap any collider's bounds, as the broad
	 *  phase. */
	static bool IsNearAnyColliderSegment(const FVector& PrevPosition, const FVector& Position,
		const TArray<IRopeCollider*>& Colliders, const FParams& Params);

	/** Narrows the list to the colliders whose bounds overlap the segment bounds, as the broad
	 *  phase. */
	static void GatherNearbyColliders(const FVector& PrevPosition, const FVector& Position,
		const TArray<IRopeCollider*>& Colliders, const FParams& Params, TArray<IRopeCollider*>& OutNearbyColliders);

	/** Splits the travel path into a number of samples, queries each, and picks the deepest contact.
	 *  Shared by the capsule and SDF paths. */
	static FRopeContact SweepOrSampleContact(const FRopeSimState& Sim, const FVector& PrevPosition,
		const FVector& Position, const TArray<IRopeCollider*>& Colliders, const FParams& Params);

	/** Converts an FRopeContact into a candidate, including its surface velocity. */
	static FRopeContactCandidate MakeCandidate(int32 NodeIndex, const FRopeContact& Contact);

	/** The expected wrap direction: from the contact point towards the hand, that is node 0, projected
	 *  onto the surface tangent plane. */
	static FVector ExpectedWrapTangent(const FRopeSimState& Sim, const FRopeContactCandidate& Candidate,
		const FVector& FallbackForward);

	static bool IsWrappableBone(FName Bone) { return !Bone.IsNone(); }

private:
	/** Merges the source masks of candidates sharing a node, bone and mesh. A new actual contact
	 *  refreshes the contact geometry of a predicted candidate. */
	static void AddUniqueCandidate(TArray<FRopeContactCandidate>& InOutCandidates,
		const FRopeContactCandidate& Candidate);

	/** On frames where the guide is active, runs the predictive test only for guided, tail and
	 *  fast-moving nodes, which saves cost. */
	static bool ShouldRunPredictiveContactForNode(const FRopeSimState& Sim, const FWhipGuideView& Whip,
		int32 NodeIndex, const FVector& FrameDisplacement);
};
