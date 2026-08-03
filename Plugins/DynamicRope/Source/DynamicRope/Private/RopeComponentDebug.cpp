// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponentInternal.h"

#include "Collision/RopeCollider.h"
#include "Debug/RopeDebugDraw.h"
#include "Debug/RopeDebugSnapshot.h"
#include "DynamicRopeLog.h"
#include "Engine/World.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

#pragma region Wrapping_Debug_And_Diagnostics

namespace RopeComponentPrivate
{
	void LogWrappingFailureState(const FString& OwnerName, const TCHAR* FailureSite,
		const FRopeWrappingState& State, const FRopeSimState& Sim)
	{
		FString IslandBones;
		for (const FName Bone : State.PathWrapIslandBones)
		{
			if (!IslandBones.IsEmpty())
			{
				IslandBones += TEXT(",");
			}
			IslandBones += Bone.ToString();
		}

		int32 ClosedGeometryCount = 0;
		int32 ClosedReachabilityCount = 0;
		int32 OpenPortalCount = 0;
		for (const FRopeWrapIslandPortal& Portal : State.PathWrapIslandPortals)
		{
			switch (Portal.State)
			{
			case ERopeWrapIslandPortalState::ClosedGeometry: ++ClosedGeometryCount; break;
			case ERopeWrapIslandPortalState::ClosedReachability: ++ClosedReachabilityCount; break;
			default: ++OpenPortalCount; break;
			}
		}

		const FRopeWrapPathPoint* LastPathPoint = State.Path.Num() > 0 ? &State.Path.Last() : nullptr;
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] WRAP FAILURE: site=%s reason=%s active=%d complete=%d failed=%d algorithm=%s latch=%s[%d] current=%s previous=%s path=%d/%d anchors=%d secondary=%d currentDistance=%.2fcm bridge=%.2fcm sweep=%.1fdeg accumulated=%.1fdeg projectionMisses=%d island=[%s] portals(geometry=%d reachability=%d open=%d)"),
			*OwnerName, FailureSite ? FailureSite : TEXT("Unknown"),
			State.PathBuildFailureReason.IsEmpty() ? TEXT("Unspecified") : *State.PathBuildFailureReason,
			State.bPathBuildActive ? 1 : 0, State.bPathBuildComplete ? 1 : 0,
			State.bPathBuildFailed ? 1 : 0,
			State.bPathUsesPoseSpaceIsland ? TEXT("CompositeAnalyticHelix") :
				(State.bPathUsesSingleBoneFallback ? TEXT("SingleBoneFallback") : TEXT("SequentialSurfaceVectorField")),
			*State.LatchAnchor.Bone.ToString(), State.LatchAnchor.NodeIndex,
			*State.PathCurrentBone.ToString(), *State.PathPreviousBone.ToString(),
			State.Path.Num(), State.NumTailNodes, State.Anchors.Num(), State.SecondarySeedAnchors.Num(),
			State.PathCurrentDistance, State.PathBridgeDistance,
			FMath::RadiansToDegrees(State.PathCompositeSweepAngleRad),
			FMath::RadiansToDegrees(State.PathAccumulatedAngleRad),
			State.PathCompositeProjectionFailureCount, *IslandBones,
			ClosedGeometryCount, ClosedReachabilityCount, OpenPortalCount);

		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] WRAP FAILURE GEOMETRY: simNodes=%d segment=%.2fcm surface=%s normal=%s tangent=%s axisOrigin=%s axisDirection=%s sweepRadial=%s probeRadius=%.2fcm slack=%.2fcm lastPathBone=%s lastPathSurface=%s lastPathDistance=%.2fcm"),
			*OwnerName, Sim.Num(), Sim.SegmentLength, *State.PathSurfaceWorld.ToString(),
			*State.PathNormalWorld.ToString(), *State.PathTangentWorld.ToString(),
			*State.PathAxisOrigin.ToString(), *State.PathAxisDirection.ToString(),
			*State.PathCompositeSweepRadial.ToString(), State.PathCompositeProbeRadius,
			State.PathAvailableSlack,
			LastPathPoint ? *LastPathPoint->Bone.ToString() : TEXT("None"),
			LastPathPoint ? *LastPathPoint->SurfaceWorld.ToString() : TEXT("None"),
			LastPathPoint ? LastPathPoint->DistanceFromLatch : 0.0f);
	}
}

#pragma endregion Wrapping_Debug_And_Diagnostics

#pragma region Gameplay_Debugger_Snapshot

#if WITH_GAMEPLAY_DEBUGGER
namespace
{
	// Computes the convex hull's wireframe edges from the body-local plane set, for debug drawing only.
	// It clips plane pairs: the intersection line of two faces is clipped against the remaining
	// half-spaces, and where a non-empty parameter range survives, that is a real hull edge, which leaves
	// only adjacent face pairs non-empty. The local edge endpoints are transformed into world space by the
	// rigid transform and appended to the output in pairs.
	// The plane convention is that a point is inside when its signed distance is negative on every face.
	// It is cubic in the plane count, which is harmless given a limit of 32 planes and a single actor
	// being debugged.
	void BuildConvexHullEdges(const TConstArrayView<FPlane>& Planes, const FQuat& Rot, const FVector& Trans,
		TArray<FVector>& OutWorldEdges)
	{
		const int32 N = Planes.Num();
		for (int32 i = 0; i < N; ++i)
		{
			const FVector Ni(Planes[i].X, Planes[i].Y, Planes[i].Z);
			const double  Wi = Planes[i].W;
			for (int32 j = i + 1; j < N; ++j)
			{
				const FVector Nj(Planes[j].X, Planes[j].Y, Planes[j].Z);
				const double  Wj = Planes[j].W;
				const FVector Dir = FVector::CrossProduct(Ni, Nj);
				const double  DirLenSq = Dir.SizeSquared();
				if (DirLenSq < 1e-8)
				{
					// Parallel faces, so there is no intersection line.
					continue;
				}
				// A point on the intersection line, from the standard formula for two planes.
				// The order of the cross product arguments is load-bearing: reversing them reflects the
				// point and drops large numbers of edges on an asymmetric convex.
				const FVector P0 = (FVector::CrossProduct(Nj, Dir) * Wi + FVector::CrossProduct(Dir, Ni) * Wj) / DirLenSq;

				// Clip the infinite line against the remaining planes.
				double TMin = -DBL_MAX, TMax = DBL_MAX;
				bool bValid = true;
				for (int32 k = 0; k < N; ++k)
				{
					if (k == i || k == j)
					{
						continue;
					}
					const FVector Nk(Planes[k].X, Planes[k].Y, Planes[k].Z);
					const double  Denom = FVector::DotProduct(Nk, Dir);
					// W_k - N_k·p0
					const double  Num = static_cast<double>(Planes[k].W) - FVector::DotProduct(Nk, P0);
					if (FMath::Abs(Denom) < 1e-8)
					{
						// The line lies outside this face, so there is no edge.
						if (Num < -1e-6) { bValid = false; break; }
						// The line is parallel to the face and inside it, which imposes no constraint.
						continue;
					}
					const double T = Num / Denom;
					if (Denom > 0.0) { TMax = FMath::Min(TMax, T); }
					else             { TMin = FMath::Max(TMin, T); }
				}
				if (!bValid || TMin >= TMax - 1e-4)
				{
					// Either the faces are not adjacent or the range vanished, so this is not a hull edge.
					continue;
				}
				const FVector L0 = P0 + Dir * TMin;
				const FVector L1 = P0 + Dir * TMax;
				// Local to world, through the rigid transform.
				OutWorldEdges.Add(Rot.RotateVector(L0) + Trans);
				OutWorldEdges.Add(Rot.RotateVector(L1) + Trans);
			}
		}
	}
}

void URopeComponent::FillDebugSnapshot(FRopeDebugSnapshot& Snapshot, ERopeDebugCapture CaptureMask) const
{
	Snapshot.Phase = Phase;
	Snapshot.PhaseAtFrameStart = DebugPhaseAtFrameStart;
	// The centreline positions are read only by the nodes, flight and wrap overlays. In the default
	// aim-only state the header needs nothing but the node count, so the count scalar alone is stored
	// rather than copying the whole array, which removes a per-frame allocation.
	Snapshot.NodeCount = Sim.Positions.Num();
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Nodes | ERopeDebugCapture::Flight | ERopeDebugCapture::Wrap))
	{
		Snapshot.Positions = Sim.Positions;
	}

	// The identity and the mode are reported by the header whichever view is on, and copying names is not
	// a query, so there is no gate on them.
	Snapshot.ComponentName = GetName();
	Snapshot.OwnerActorName = GetOwner() ? GetOwner()->GetName() : TEXT("None");
	Snapshot.ResolveMode = ResolveMode;

	// Frame state for the header, which the screen reads instead of the live component so it keeps a
	// single point in time.
	// Unlike the wrapped-only block, the wrapped bone is stored regardless of phase, because the header
	// always reports it.
	Snapshot.WrapBoneName = WrapController.State.BoneName;
	Snapshot.bSleeping = Throttle.IsAsleep();
	Snapshot.LodScale = Throttle.GetSolverLODScale();
	Snapshot.NumParticles = NumParticles;
	Snapshot.TubeSmoothingSubdiv = TubeSmoothingSubdiv;
	Snapshot.NodeCollisionRadius = GetEffectiveCollisionRadius();
	Snapshot.bSolveThisFrame = SimFrame.bSolveThisFrame;
	Snapshot.bGpuStepped = SimFrame.bGpuSteppedThisFrame;
	Snapshot.bLogicOverride = SimFrame.OverrideFrame.HasAny();

	// The index of the latch node to highlight on the centreline. Only the latch highlight of the nodes
	// view and the wrap view read it, so there is no reason to walk and copy it in the default aim-only
	// state.
	const FRopeWrapState& Wrap = WrapController.State;
	Snapshot.LatchedNodes.Reset();
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Nodes | ERopeDebugCapture::Wrap))
	{
		Snapshot.LatchedNodes.Reserve(Wrap.Latched.Num());
		for (const FRopeLatchNode& Latch : Wrap.Latched)
		{
			Snapshot.LatchedNodes.Add(Latch.NodeIndex);
		}
	}

	// The wrapped detail, used by the table, is captured only during the Wrapped phase and with the wrap
	// view enabled.
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Wrap) && Phase == ERopePhase::Wrapped && Wrap.IsWrapped())
	{
		Snapshot.bHasWrapped = true;
		const USceneComponent* Mesh = Wrap.Mesh.Get();
		Snapshot.MeshName = Mesh ? Mesh->GetName() : TEXT("None");
		Snapshot.Latched = Wrap.Latched;
		Snapshot.WrapTension = Wrap.Tension;
		Snapshot.TensionReleaseForce = HoldConfig.TensionReleaseForce;
		// Whether the automatic release is actually in effect, which is separate from the threshold values.
		// The test lives in this one place and the screen reads only this boolean, which keeps the display
		// from diverging from the mode gate inside CheckWrappedAutoRelease.
		Snapshot.bAutoReleaseEnabled = (ResolveMode != ERopeWrapResolveMode::GuaranteedWrap);
		Snapshot.TensionOverTime = TensionOverTime;
		Snapshot.TensionReleaseTime = HoldConfig.TensionReleaseTime;
		Snapshot.bPullValid = PullDrive.LastPullSample.bValid;
		Snapshot.PullPoint = PullDrive.LastPullSample.WorldPoint;
		// The smoothed direction, which is the one actually applied.
		Snapshot.PullDirection = PullDrive.LastPullSample.Direction;
		// The look-ahead direction before smoothing, for diagnosing jitter.
		Snapshot.PullDirRaw = PullDrive.LastPullDirRaw;
		// The raw integer aim node, as text for diagnosing hops.
		Snapshot.PullAimNode = PullDrive.LastPullSample.AimNode;
		// The smoothed fractional aim position, which is the input to the direction average. The direction
		// actually applied is the smoothed direction above.
		Snapshot.PullAimPoint = PullDrive.LastPullSample.bValid ? PullDrive.LastPullSample.AimPos
			: PullDrive.LastPullSample.WorldPoint;
		Snapshot.PullTension = GetConstraintTension();
		Snapshot.TetherOvershoot = LengthConstraintState.LastViolation;
		Snapshot.TetherTension = GetConstraintTension();
		Snapshot.MaxTetherTension = HoldConfig.MaxTetherTension;
		switch (LengthConstraintState.Backend)
		{
		case ERopeLengthConstraintBackend::Chaos:
			Snapshot.ConstraintBackend = TEXT("Chaos");
			break;
		case ERopeLengthConstraintBackend::HardReaction:
			Snapshot.ConstraintBackend = TEXT("HardReaction");
			break;
		case ERopeLengthConstraintBackend::Analytic:
			Snapshot.ConstraintBackend = TEXT("Analytic");
			break;
		default:
			Snapshot.ConstraintBackend = TEXT("None");
			break;
		}
		Snapshot.AttemptedOutwardSpeed =
			LengthConstraintState.WielderAttemptFrame == GFrameCounter
				? LengthConstraintState.WielderAttemptSeparatingSpeed
				: 0.0f;
		Snapshot.AttemptedViolation =
			LengthConstraintState.WielderAttemptFrame == GFrameCounter
				? LengthConstraintState.WielderAttemptViolation
				: 0.0f;
		Snapshot.ActivePullForce = PullDrive.ActivePullForce;
		Snapshot.bActivePullApplied = DebugActivePullPassedGate;
		Snapshot.bPullTaut = PullDrive.bPullTaut;
		Snapshot.bChainTaut = PullDrive.bChainTaut;
		Snapshot.TautChordLen = PullDrive.LastPullSample.TautChordLen;
		Snapshot.FreeRestLen = PullDrive.LastPullSample.FreeRestLen;
		Snapshot.MinFreeTension = PullDrive.LastPullSample.MinFreeTension;
		Snapshot.MaxLegSag = PullDrive.LastPullSample.MaxLegSag;
		Snapshot.DistanceReleaseSlack = HoldConfig.DistanceReleaseSlack;
	}

	// Wrap axis visualization: stores the path axis, as an origin and a direction, that ResolveWrappingAxis
	// chose during the Wrapping phase. The wrap view draws it as a line so the axis this wrap is turning
	// about can be confirmed by eye.
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Wrap)
		&& Phase == ERopePhase::Wrapping && WrappingPhase.State.IsActive())
	{
		Snapshot.bHasWrapAxis = true;
		Snapshot.WrapAxisOrigin = WrappingPhase.State.PathAxisOrigin;
		Snapshot.WrapAxisDirection = WrappingPhase.State.PathAxisDirection;
		Snapshot.WrapAxisSegmentLength = Sim.SegmentLength;
	}

	// The collider count has no separate field; the size of the collider array below is its single source,
	// used by the collider section of the display.

	// Visualization of the colliders this rope queried this frame, which replaces the per-provider debug
	// draw flag. The real shape is classified through the mutually exclusive accessors in order: a capsule
	// as a segment, a box as an oriented box, a convex as a hull wireframe, and anything else, such as an
	// SDF, as a world AABB fallback.
	// The frame colliders belong to the providers and are valid for this frame only; the third phase runs
	// serially on the game thread, so threading is not a concern.
	// It is captured only with the collider view enabled: the shape copies and the convex hull edge
	// reconstruction, which is cubic in the plane count and allocates arrays every frame, all live here,
	// and skipping the lot while the view is off is the point of this gate.
	Snapshot.Colliders.Reset();
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Colliders))
	for (const IRopeCollider* Collider : SimFrame.FrameColliders)
	{
		if (!Collider)
		{
			continue;
		}
		FRopeDebugCollider DC;
		DC.bWorldStatic = Collider->IsWorldStatic();

		// One attribution lookup, of the bone and mesh, produces two values.
		//  - Whether it is a static mesh wrap target served by URopeWrapTargetComponent, which has a
		//    virtual bone but a source mesh that is not skeletal. It drives the static wrap target count.
		//  - Whether it can actually be wrapped, which is decided through the same CanWrapTarget gate that
		//    candidate production uses, so the display and the decision share one basis.
		{
			FName AttribBone = NAME_None;
			const USceneComponent* AttribMesh = nullptr;
			Collider->GetGPUAttribution(AttribBone, AttribMesh);
			const bool bAttributed = !AttribBone.IsNone() && AttribMesh != nullptr;
			DC.bWrapTarget = bAttributed && !RopeWrapTargets::IsSkeletalTarget(AttribMesh);
			DC.bWrapAllowed = bAttributed && CanWrapTarget(AttribMesh, AttribBone);
		}

		TConstArrayView<FPlane> LocalPlanes;
		FBox LocalBounds(ForceInit);
		FQuat CvRot, CvPrevRot;
		FVector CvTrans, CvPrevTrans;
		float CvInvDt = 0.0f;
		if (Collider->GetGPUCapsule(DC.A, DC.B, DC.Radius))
		{
			DC.Shape = ERopeDebugColliderShape::Capsule;
		}
		else if (Collider->GetGPUBox(DC.Center, DC.Rot, DC.HalfExtents))
		{
			DC.Shape = ERopeDebugColliderShape::Box;
		}
		else if (Collider->GetGPUConvex(LocalPlanes, LocalBounds, CvRot, CvTrans, CvPrevRot, CvPrevTrans, CvInvDt)
			&& LocalPlanes.Num() >= 4)
		{
			DC.Shape = ERopeDebugColliderShape::Convex;
			BuildConvexHullEdges(LocalPlanes, CvRot, CvTrans, DC.ConvexEdges);
		}
		else
		{
			DC.Shape = ERopeDebugColliderShape::Bounds;
			DC.Bounds = Collider->GetWorldBounds();
		}
		Snapshot.Colliders.Add(MoveTemp(DC));
	}

	// The per-node proximity re-query, for debugging only: the post-solve node positions are queried
	// against the frame colliders again to record which face each node is up against, through its normal.
	// The GPU runtime does not read contacts back, so this re-queries on the CPU. The query radius is the
	// collision radius plus a margin, so a settled node, resting about a radius off the surface, is caught
	// too; that means this is not the set of contacts the solver actually resolved, and the screen states
	// that margin alongside. Only the deepest contact per node is kept.
	// At one CPU query per node per collider it is the most expensive item captured, so it runs only with
	// the nodes view enabled.
	constexpr float ProximityQueryMargin = 4.0f;
	Snapshot.NodeProximity.Reset();
	Snapshot.ProximityQueryMargin = ProximityQueryMargin;
	const float DebugQueryRadius = GetEffectiveCollisionRadius() + ProximityQueryMargin;
	const int32 ProximityNodeCount = EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Nodes)
		? Sim.Positions.Num() : 0;
	for (int32 i = 0; i < ProximityNodeCount; ++i)
	{
		const FVector NodePos = Sim.Positions[i];
		FRopeContact Best;
		bool bAny = false;
		// Whether the collider that produced the winning contact is static is carried alongside, since the
		// contact struct does not hold it and it would otherwise be lost here.
		bool bBestWorldStatic = false;
		for (const IRopeCollider* Collider : SimFrame.FrameColliders)
		{
			if (!Collider)
			{
				continue;
			}
			const FRopeContact C = Collider->Query(NodePos, DebugQueryRadius);
			if (C.bHit && (!bAny || C.Penetration > Best.Penetration))
			{
				Best = C;
				bBestWorldStatic = Collider->IsWorldStatic();
				bAny = true;
			}
		}
		if (bAny)
		{
			FRopeNodeProximityDebug NP;
			NP.NodeIndex = i;
			NP.Position = NodePos;
			NP.Normal = Best.Normal;
			NP.Bone = Best.Bone;
			NP.bWorldStatic = bBestWorldStatic;
			Snapshot.NodeProximity.Add(MoveTemp(NP));
		}
	}
}
#endif

#pragma endregion Gameplay_Debugger_Snapshot

#pragma region Flight_Observation

void URopeComponent::RecordFlightObservation(const FRopeFlightContactDetector::FParams& DetectParams,
	const TArray<FRopeContactCandidate>& Candidates, const FRopeContactTracker& FrameTracker,
	bool bShouldCapture, FRopeDebugSnapshot* OutSnapshot)
{
	// Observation only: the read-only consumers that take no part in the decisions are gathered here.
	// The statistics cost anything only while the stat system is collecting, and a snapshot is produced
	// only for the rope the debugger targets, with null passed for every other.
	const bool bNeedStats = RopeDebug::IsFlightStatEnabled();
	const bool bNeedSnapshot = OutSnapshot != nullptr;
	if (!bNeedStats && !bNeedSnapshot)
	{
		return;
	}

	const float WhipGuidedEnd = FMath::Clamp(WhipConfig.GuidedLength, 0.05f, 1.0f);
	int32 WhipGuidedNodeCount = 0;
#if WITH_GAMEPLAY_DEBUGGER
	if (OutSnapshot)
	{
		WhipGuide.CopyGuidedTargetsForDebug(
			OutSnapshot->WhipGuideNodeIndices, OutSnapshot->WhipGuideTargets);
		WhipGuidedNodeCount = OutSnapshot->WhipGuideNodeIndices.Num();
	}
#endif
	if (WhipGuidedNodeCount == 0 && bNeedStats)
	{
		WhipGuidedNodeCount = WhipGuide.GetGuidedNodeCountThisFrame();
	}
	const bool bWhipActive = WhipGuidedNodeCount > 0;
	RopeDebug::RecordFlightStats(Sim, SimFrame.bSolveThisFrame, SimFrame.FrameColliders.Num(), Candidates,
		FrameTracker, bShouldCapture);
	RopeDebug::RecordWhipStats(Sim, WhipGuidedNodeCount, WhipGuidedEnd);

#if WITH_GAMEPLAY_DEBUGGER
	if (OutSnapshot)
	{
		GatherFlightNodeDebug(DetectParams, OutSnapshot->NodeDebug);
		OutSnapshot->bHasFlight = true;
		// Whether the rope solves this frame, and the collider count, are filled in by FillDebugSnapshot,
		// which always runs and is their single source, so they are not written here.
		// The capture decision values are deliberately absent; the reason is given on the snapshot struct.
		OutSnapshot->TrackerBone = FrameTracker.CandidateBone;
		OutSnapshot->Candidates = Candidates;
		// A target is identified by the pair of mesh and bone, following the contact tracker's contract.
		// The mesh pointer may be dead once the snapshot has outlived its frame, so it is frozen now into a
		// comparison-only key.
		OutSnapshot->TrackerMeshKey = FObjectKey(FrameTracker.CandidateMesh);
		OutSnapshot->CandidateMeshKeys.Reset(Candidates.Num());
		for (const FRopeContactCandidate& Candidate : Candidates)
		{
			OutSnapshot->CandidateMeshKeys.Add(FObjectKey(Candidate.Mesh));
		}
		OutSnapshot->bWhipActive = bWhipActive;
		OutSnapshot->WhipGuidedEnd = WhipGuidedEnd;
	}
#endif
}

#if WITH_GAMEPLAY_DEBUGGER
void URopeComponent::GatherFlightNodeDebug(const FRopeFlightContactDetector::FParams& DetectParams,
	TArray<FRopeFlightNodeDebug>& OutNodeDebug) const
{
	// Visualization capture for the rope the debugger targets. Re-querying the detector per node,
	// separately from the real detection pipeline, is intentional duplication: the point is to show why a
	// node that the decision did not catch, whether too slow or too far, was not caught, which reusing the
	// decision's output cannot do. The cost falls on the single rope being debugged.
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightDebugGather);
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!Sim.PrevPositions.IsValidIndex(i) || !Sim.Positions.IsValidIndex(i))
		{
			continue;
		}

		FRopeFlightNodeDebug NodeDebug;
		NodeDebug.NodeIndex = i;
		NodeDebug.PrevPosition = Sim.PrevPositions[i];
		NodeDebug.Position = Sim.Positions[i];
		const bool bFast = FRopeFlightContactDetector::IsTailNode(Sim, i) || Sim.NodeSpeed(i) > Sim.SegmentLength;
		NodeDebug.bNearBody = FRopeFlightContactDetector::IsNearAnyColliderSegment(
			NodeDebug.PrevPosition, NodeDebug.Position, SimFrame.FrameColliders, DetectParams);
		if (bFast || NodeDebug.bNearBody)
		{
			NodeDebug.Contact = FRopeFlightContactDetector::SweepOrSampleContact(
				Sim, NodeDebug.PrevPosition, NodeDebug.Position, SimFrame.FrameColliders, DetectParams);
			// The same node's contact may be against a different target from the candidate's. The key used
			// to decide target identity is frozen now, since a raw pointer cannot be dereferenced once the
			// snapshot has outlived its frame.
			NodeDebug.ContactMeshKey = FObjectKey(NodeDebug.Contact.SourceMesh);
		}

		if (bFast || NodeDebug.bNearBody || NodeDebug.Contact.bHit)
		{
			OutNodeDebug.Add(NodeDebug);
		}
	}
}
#endif // WITH_GAMEPLAY_DEBUGGER

#pragma endregion Flight_Observation

