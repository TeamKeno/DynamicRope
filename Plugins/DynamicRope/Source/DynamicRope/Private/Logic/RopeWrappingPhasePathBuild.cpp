// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeWrappingPhase.h"
#include "Components/SceneComponent.h"
// ResolveBindingWorld, the single point that resolves a wrap binding, whether a bone, a socket or a
// component, into a transform.
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"
// RopeMath::AnyTangentFromNormal, included here to avoid a duplicate definition in the unity build.
#include "RopeMathHelpers.h"

const USceneComponent* FRopeWrappingPhase::ResolveWrappingMesh(
	const FRopeWrappingState& State, const FRopeSurfaceAnchor& Anchor)
{
	if (const USceneComponent* AnchorMesh = Anchor.Mesh.Get())
	{
		return AnchorMesh;
	}
	return State.Mesh.Get();
}

#pragma region Wrapping Path Build

void FRopeWrappingPhase::FinishPathBuild(bool bFailed, const TCHAR* FailureReason)
{
	State.bPathBuildActive = false;
	State.bPathBuildComplete = !bFailed;
	State.bPathBuildFailed = bFailed;
	if (bFailed)
	{
		State.PathBuildFailureReason = FailureReason && FailureReason[0] != 0
			? FailureReason
			: TEXT("UnknownPathBuildFailure");
	}
}

void FRopeWrappingPhase::CollectCompletedVirtualBridgeRuns()
{
	State.VirtualBridgeScanPathIndex = FMath::Clamp(
		State.VirtualBridgeScanPathIndex, 0, State.Path.Num());

	while (State.VirtualBridgeScanPathIndex < State.Path.Num())
	{
		const int32 PathIndex = State.VirtualBridgeScanPathIndex;
		const FRopeWrapPathPoint& Point = State.Path[PathIndex];

		if (State.VirtualRunStartPathIndex == INDEX_NONE)
		{
			if (Point.bVirtual)
			{
				State.VirtualRunStartPathIndex = PathIndex;
				State.VirtualRunLeftPathIndex = PathIndex - 1;
			}
			++State.VirtualBridgeScanPathIndex;
			continue;
		}

		if (Point.bVirtual)
		{
			++State.VirtualBridgeScanPathIndex;
			continue;
		}

		const int32 RunStart = State.VirtualRunStartPathIndex;
		const int32 RunEnd = PathIndex - 1;
		const int32 LeftPathIndex = State.VirtualRunLeftPathIndex;
		const int32 RightPathIndex = PathIndex;
		if (State.Path.IsValidIndex(LeftPathIndex) &&
			State.Path.IsValidIndex(RightPathIndex) &&
			!State.Path[LeftPathIndex].bBridge && !State.Path[LeftPathIndex].bVirtual &&
			!State.Path[RightPathIndex].bBridge && !State.Path[RightPathIndex].bVirtual)
		{
			const int32 LatchNodeIndex = State.LatchAnchor.NodeIndex;
			FRopeVirtualBridgeRun Run;
			Run.LeftNodeIndex = LatchNodeIndex + LeftPathIndex;
			Run.RightNodeIndex = LatchNodeIndex + RightPathIndex;
			if (Run.RightNodeIndex - Run.LeftNodeIndex > 1)
			{
				Run.VirtualNodeIndices.Reserve(RunEnd - RunStart + 1);
				for (int32 VirtualPathIndex = RunStart; VirtualPathIndex <= RunEnd; ++VirtualPathIndex)
				{
					Run.VirtualNodeIndices.Add(LatchNodeIndex + VirtualPathIndex);
				}
				State.VirtualBridgeRuns.Add(MoveTemp(Run));
			}
		}

		State.VirtualRunStartPathIndex = INDEX_NONE;
		State.VirtualRunLeftPathIndex = INDEX_NONE;
		++State.VirtualBridgeScanPathIndex;
	}
}

int32 FRopeWrappingPhase::ComputePathStepBudget(int32 NumTailNodes, int32 StepsPerFrame)
{
	// The total work is twice NumTailNodes, since the surface vector field consumes two steps per path
	// point. A size-proportional baseline, which completes in roughly four frames at the reference value,
	// is scaled by the configured value: higher finishes sooner and lower takes longer. It is used as a
	// multiplier rather than a floor so the setting still has effect on large ropes. The preview passes a
	// very large value, which effectively completes in a single frame. The minimum is 1.
	constexpr int32 BaselineStepsPerFrame = 8;   // Kept in step with the default of FRopeWrapConfig::WrappingPathBuildStepsPerFrame.
	const int32 SizeBaseline = FMath::DivideAndRoundUp(FMath::Max(0, NumTailNodes) * 2, 4);
	return FMath::Max(1, SizeBaseline * FMath::Max(1, StepsPerFrame) / BaselineStepsPerFrame);
}

void FRopeWrappingPhase::AdvancePathBuild(const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_AdvanceProgressiveWrapPathBuild);

	if (!State.bPathBuildActive ||
		State.bPathBuildComplete ||
		State.bPathBuildFailed ||
		State.NumTailNodes <= 0)
	{
		return;
	}

	// The frame budget: the total work, two steps per path point for the surface vector field, is taken as
	// a size-proportional baseline and scaled by the configured steps per frame, so the setting still
	// makes a difference on large ropes rather than acting as a simple floor. The preview completes in one
	// go.
	const int32 StepBudget = FRopeWrappingPhase::ComputePathStepBudget(
		State.NumTailNodes, Ctx.GetPathBuildStepsPerFrame());
	if (State.bPathUsesPoseSpaceIsland)
	{
		// One composite raw probe can produce zero or several output nodes, depending on how far the
		// projection actually moved. The path point count is recorded before the probe call and anchors are
		// attached in order across the whole range added by this call, which keeps the anchored path point
		// count contiguous.
		for (int32 StepIndex = 0;
			StepIndex < StepBudget && State.Path.Num() < State.NumTailNodes;
			++StepIndex)
		{
			const int32 FirstNewPathIndex = State.Path.Num();
			if (!AdvanceCompositeAnalyticHelixProbeStep(Sim, Ctx))
			{
				break;
			}

			bool bAnchorsAppended = true;
			for (int32 PathIndex = FirstNewPathIndex;
				PathIndex < State.Path.Num(); ++PathIndex)
			{
				if (!ProcessPathPointForAnchoring(PathIndex, Sim, Ctx))
				{
					FinishPathBuild(/*bFailed=*/true, TEXT("CompositeAnalyticHelixAnchorFailed"));
					bAnchorsAppended = false;
					break;
				}
			}
			if (!bAnchorsAppended)
			{
				break;
			}
		}
		CollectCompletedVirtualBridgeRuns();

		if (State.Path.Num() >= State.NumTailNodes)
		{
			FinishPathBuild(/*bFailed=*/false);
		}
		if (State.bPathBuildFailed)
		{
			const FString CompositeFailureReason = State.PathBuildFailureReason;
			RestartPathBuildAsSingleBoneFallback(
				Sim, Ctx, *CompositeFailureReason);
		}
		return;
	}
	AdvanceSequentialSurfaceVectorFieldPath(StepBudget, Sim, Ctx);
	CollectCompletedVirtualBridgeRuns();
}

bool FRopeWrappingPhase::BeginProgressiveWrapPathBuild(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_BeginProgressiveWrapPathBuild);

	// Every initialization attempt starts from the inactive state. Whichever validation step below fails,
	// it goes through FinishPathBuild so the active, complete and failed flags always end in one
	// consistent terminal state.
	State.bPathBuildActive = false;
	State.bPathBuildComplete = false;
	State.bPathBuildFailed = false;
	State.PathBuildFailureReason.Reset();

	const USceneComponent* Mesh = ResolveWrappingMesh(State, LatchAnchor);
	if (!Mesh || !Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) || LatchAnchor.Bone.IsNone())
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("InvalidLatchInput"));
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Path initialization rejected: reason=InvalidLatchInput mesh=%s bone=%s node=%d validNode=%d simNodes=%d"),
			*Ctx.OwnerName, Mesh ? *Mesh->GetName() : TEXT("None"),
			*LatchAnchor.Bone.ToString(), LatchAnchor.NodeIndex,
			Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) ? 1 : 0, Sim.Num());
		return false;
	}

	FRopeSurfaceAnchor StoredLatchAnchor = LatchAnchor;
	StoredLatchAnchor.Mesh = Mesh;

	State.Anchors.Reset();
	State.Path.Reset();
	State.VirtualBridgeRuns.Reset();
	State.VirtualBridgeScanPathIndex = 0;
	State.VirtualRunStartPathIndex = INDEX_NONE;
	State.VirtualRunLeftPathIndex = INDEX_NONE;
	State.LatchAnchor = StoredLatchAnchor;
	State.NumTailNodes = Sim.Num() - StoredLatchAnchor.NodeIndex;
	// Seed multiplexing: from the first secondary seed node onwards the nodes are owned by the secondary
	// anchors rather than the path, so the path length is cut to end before it. That stops a path anchor
	// and a secondary anchor both claiming the same node, which would otherwise be a last-writer-wins race
	// between the commit seed and Hold. The rope beyond the secondary nodes is frozen by the mask during
	// Wrapping and becomes a free span after the commit.
	for (const FRopeSurfaceAnchor& Secondary : State.SecondarySeedAnchors)
	{
		if (Secondary.NodeIndex > StoredLatchAnchor.NodeIndex)
		{
			State.NumTailNodes = FMath::Min(State.NumTailNodes, Secondary.NodeIndex - StoredLatchAnchor.NodeIndex);
		}
	}
	State.LastAnchoredPathPointCount = 0;
	State.bPathBuildActive = true;
	State.bPathBuildComplete = false;
	State.bPathBuildFailed = false;
	State.PathCurrentDistance = 0.0f;
	State.PathSweepDistance = 0.0f;
	State.bPathCompositeRawPointVirtual = false;
	State.PathAccumulatedAngleRad = 0.0f;
	State.PathSignedNetAngleRad = 0.0f;
	State.PathForwardAngleRad = 0.0f;
	State.PathReverseAngleRad = 0.0f;
	State.PathRadiusSum = 0.0f;
	State.PathRadiusMin = 0.0f;
	State.PathRadiusMax = 0.0f;
	State.PathRadiusSampleCount = 0;
	State.PathBoneTransitionCount = 0;
	State.PathBridgeDistance = 0.0f;
	State.FrontDistance = 0.0f;
	State.FrontWrapAngleRad = 0.0f;
	// Each new progressive path also resets the four-stage completion gate. Leaving the previous throw's
	// final target, or the time it reached it, would let a new wrap commit immediately, so they must be
	// reset together.
	State.FrontTargetDistance = 0.0f;
	State.FrontTargetWrapAngleRad = 0.0f;
	State.bFrontUsesAngleMapping = false;
	State.FrontReachedTargetElapsed = -1.0f;
	State.FrontMotionStartElapsed = -1.0f;
	State.bFrontMotionStartLogged = false;
	State.bFrontMotionCompletionLogged = false;
	State.PathCurrentBone = StoredLatchAnchor.Bone;
	State.PathPreviousBone = NAME_None;
	State.PathCurrentMesh = Mesh;
	State.PathDistanceSinceBoneTransition = 0.0f;
	State.FirstNode = TNumericLimits<int32>::Max();
	State.LastNode = INDEX_NONE;

	if (State.NumTailNodes <= 0)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("NoTailNodesAfterLatch"));
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Path initialization rejected: reason=NoTailNodesAfterLatch latchNode=%d simNodes=%d secondarySeeds=%d"),
			*Ctx.OwnerName, StoredLatchAnchor.NodeIndex, Sim.Num(), State.SecondarySeedAnchors.Num());
		return false;
	}

	State.Path.Reserve(State.NumTailNodes);

	const bool bInitialized =
		InitializeProgressiveWrapPath(StoredLatchAnchor, Sim, Ctx);
	if (!bInitialized)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("InitialSurfacePathPointFailed"));
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Path initialization failed: reason=InitialSurfacePathPointFailed bone=%s node=%d surface=%s normal=%s"),
			*Ctx.OwnerName, *StoredLatchAnchor.Bone.ToString(), StoredLatchAnchor.NodeIndex,
			*State.PathSurfaceWorld.ToString(), *State.PathNormalWorld.ToString());
		return false;
	}
	if (!ProcessPathPointForAnchoring(0, Sim, Ctx))
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("InitialAnchorBuildFailed"));
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Path initialization failed: reason=InitialAnchorBuildFailed bone=%s node=%d pathPoints=%d surface=%s"),
			*Ctx.OwnerName, *StoredLatchAnchor.Bone.ToString(), StoredLatchAnchor.NodeIndex,
			State.Path.Num(), *State.PathSurfaceWorld.ToString());
		return false;
	}

	return true;
}

bool FRopeWrappingPhase::RestartPathBuildAsSingleBoneFallback(
	const FRopeSimState& Sim, const FContext& Ctx, const TCHAR* CompositeFailureReason)
{
	const int32 FailedPathPointCount = State.Path.Num();
	const int32 FailedAnchorCount = State.Anchors.Num();
	const float FailedSweepAngleDeg = FMath::RadiansToDegrees(State.PathCompositeSweepAngleRad);
	const FRopeSurfaceAnchor OriginalLatch = State.LatchAnchor;

	State.bPathUsesSingleBoneFallback = true;
	State.PathBuildFailureReason.Reset();
	// The seeds removed when the composite path was activated cannot be partially restored. The remaining
	// seeds are cleared as well so the single-bone path unambiguously owns the entire tail, and it is
	// rebuilt from the original latch.
	State.SecondarySeedAnchors.Reset();
	UE_LOG(LogRopeWrap, Log,
		TEXT("[%s] Wrap algorithm fallback: from=CompositeAnalyticHelix to=SingleBoneSurfaceVectorField reason=%s "
			"latchBone=%s failedPath=%d failedAnchors=%d failedSweep=%.1fdeg"),
		*Ctx.OwnerName, CompositeFailureReason ? CompositeFailureReason : TEXT("Unknown"),
		*OriginalLatch.Bone.ToString(), FailedPathPointCount, FailedAnchorCount, FailedSweepAngleDeg);

	if (!BeginProgressiveWrapPathBuild(OriginalLatch, Sim, Ctx))
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("SingleBoneFallbackInitializationFailure"));
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Single-bone fallback failed: reason=InitializationFailure bone=%s node=%d"),
			*Ctx.OwnerName, *OriginalLatch.Bone.ToString(), OriginalLatch.NodeIndex);
		return false;
	}

	UE_LOG(LogRopeWrap, Log,
		TEXT("[%s] Single-bone fallback initialized: bone=%s node=%d tailNodes=%d "
			"axisOrigin=%s axisDirection=%s"),
		*Ctx.OwnerName, *OriginalLatch.Bone.ToString(), OriginalLatch.NodeIndex,
		State.NumTailNodes, *State.PathAxisOrigin.ToString(), *State.PathAxisDirection.ToString());
	return true;
}

bool FRopeWrappingPhase::InitializeProgressiveWrapPath(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_InitializeProgressiveWrapPath);

	const USceneComponent* Mesh = ResolveWrappingMesh(State, LatchAnchor);
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	if (!ResolveWrappingAxis(LatchAnchor, Ctx, State.PathAxisOrigin, State.PathAxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, State.PathAxisDirection);

	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);
	State.PathSurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
	State.PathNormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector LatchTangentWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalTangent);
	LatchTangentWorld = (LatchTangentWorld - FVector::DotProduct(LatchTangentWorld, State.PathNormalWorld) * State.PathNormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(State.PathNormalWorld));

	const float LatchAxisDistance = FVector::DotProduct(
		State.PathSurfaceWorld - State.PathAxisOrigin,
		State.PathAxisDirection);
	const FVector LatchAxisPoint = State.PathAxisOrigin + State.PathAxisDirection * LatchAxisDistance;
	State.PathLatchRadial = (State.PathSurfaceWorld - LatchAxisPoint)
		.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathNormalWorld);

	State.PathCircumferenceDir = FVector::CrossProduct(
		State.PathAxisDirection,
		State.PathLatchRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(State.PathNormalWorld));
	// The reference direction for the winding is the latch tangent by default, which is the direction the
	// rope lies in. Under CaptureTravelPlane, where a capture velocity exists, the velocity is used
	// instead, so the direction wrapping starts in matches the way the rope was actually moving and is not
	// shaken by tangent noise on the collision frame.
	FVector WindingReference = LatchTangentWorld;
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::CaptureTravelPlane &&
		Ctx.TravelFrame && Ctx.TravelFrame->bValid &&
		!Ctx.TravelFrame->AverageVelocity.IsNearlyZero())
	{
		WindingReference = Ctx.TravelFrame->AverageVelocity.GetSafeNormal();
	}
	State.PathWindingSign =
		FVector::DotProduct(State.PathCircumferenceDir, WindingReference) < 0.0f ? -1.0f : 1.0f;
	State.PathCircumferenceDir *= State.PathWindingSign;

	State.PathTangentWorld = ComputeSurfaceVectorFieldTangent(
		State.PathAxisOrigin,
		State.PathAxisDirection,
		State.PathLatchRadial,
		State.PathWindingSign,
		State.PathSurfaceWorld,
		State.PathNormalWorld,
		Ctx,
		State.PathCircumferenceDir);
	State.PathWrapIslandBones.Reset();
	State.PathWrapIslandMembers.Reset();
	State.PathWrapIslandPortals.Reset();
	State.PathAvailableSlack = 0.0f;
	State.bPathUsesPoseSpaceIsland = false;
	State.PathCompositeSweepRadial = State.PathLatchRadial;
	State.PathCompositeProbeRadius = 0.0f;
	State.PathCompositeHelixRadius = 0.0f;
	State.PathCompositeHelixPitchScale = 0.0f;
	State.PathCompositeAxisMinDistance = 0.0f;
	State.PathCompositeAxisMaxDistance = 0.0f;
	State.bPathCompositeAxisRangeValid = false;
	State.PathCompositeSweepAngleRad = 0.0f;
	// Composite multi-bone wrapping is for FullSimulation alone, which uses the pure physical outcome. The
	// assisted and guaranteed modes build no island and take the existing sequential parent and child
	// transition path of the surface vector field below.
	if (Ctx.ResolveMode == ERopeWrapResolveMode::FullSimulation &&
		Ctx.Config.bEnableMultiBoneWrapping && !State.bPathUsesSingleBoneFallback)
	{
		GatherPoseSpaceWrapIsland(LatchAnchor, Sim, Mesh,
			State.PathWrapIslandBones, State.PathWrapIslandMembers,
			State.PathWrapIslandPortals, State.PathAvailableSlack, Ctx);
		// A single surface keeps the existing projection and axis maths, which preserves the angles and
		// tuning of single-bone wrapping. The composite selector is enabled only when two or more bones
		// really did group into one pose-space column.
		State.bPathUsesPoseSpaceIsland = State.PathWrapIslandBones.Num() > 1;
		if (State.bPathUsesPoseSpaceIsland)
		{
			// ResolveWrappingAxis runs before the island is built, so contacting an arm first leaves the axis
			// origin at that arm's collider centre. Measuring the outer support against that axis makes the
			// opposite arm appear excessively far out, and a path that once crossed to an arm can never
			// return to the torso. The direction and the winding sign preserve the throw frame's information
			// exactly; only the component of the origin perpendicular to the axis moves to the outline centre
			// of the composite cross-section at the moment of contact.
			const FVector AxisDirection = State.PathAxisDirection.GetSafeNormal(
				KINDA_SMALL_NUMBER, FVector::UpVector);
			const FVector PlaneU = RopeMath::AnyTangentFromNormal(AxisDirection)
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
			const FVector PlaneV = FVector::CrossProduct(AxisDirection, PlaneU)
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::RightVector);
			const FVector SliceOrigin = State.PathSurfaceWorld;
			const float SliceTolerance = FMath::Max(0.5f, Ctx.GetContactRadius() * 0.25f);

			float MinU = TNumericLimits<float>::Max();
			float MaxU = -TNumericLimits<float>::Max();
			float MinV = TNumericLimits<float>::Max();
			float MaxV = -TNumericLimits<float>::Max();
			int32 CrossSectionContributorCount = 0;
			TArray<FVector2D, TInlineAllocator<128>> IslandProjectedBoundsPoints;
			TArray<FVector, TInlineAllocator<128>> IslandSDFAxisSamples;
			float IslandMinAxisCoordinate = TNumericLimits<float>::Max();
			float IslandMaxAxisCoordinate = -TNumericLimits<float>::Max();
			int32 IslandBoundsContributorCount = 0;
			int32 IslandSDFAxisContributorCount = 0;

			// Projecting the intersections of the twelve edges of the oriented boxes with the latch plane
			// gives the real outline extent of the cross-section, unbiased by the number of bones or by the
			// axial length of the SDF bounds. The world AABB is used as an identity-oriented box only where
			// no SDF box exists.
			static constexpr int32 BoxEdges[12][2] =
			{
				{ 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 },
				{ 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 },
				{ 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
			};

			for (const FRopeWrapIslandMember& Member : State.PathWrapIslandMembers)
			{
				if (!State.PathWrapIslandBones.Contains(Member.Bone))
				{
					continue;
				}

				FVector BoxCenter = FVector::ZeroVector;
				FVector BoxHalfExtent = FVector::ZeroVector;
				FQuat BoxRotation = FQuat::Identity;
				if (Member.bHasOrientedSDFBounds)
				{
					BoxCenter = Member.SDFCenter;
					BoxHalfExtent = Member.SDFHalfExtent;
					BoxRotation = Member.SDFRotation;
				}
				else if (Member.WorldBounds.IsValid)
				{
					BoxCenter = Member.WorldBounds.GetCenter();
					BoxHalfExtent = Member.WorldBounds.GetExtent();
				}
				else
				{
					continue;
				}

				FVector Corners[8];
				float SignedDistances[8];
				for (int32 CornerIndex = 0; CornerIndex < 8; ++CornerIndex)
				{
					const FVector LocalCorner(
						(CornerIndex & 1) ? BoxHalfExtent.X : -BoxHalfExtent.X,
						(CornerIndex & 2) ? BoxHalfExtent.Y : -BoxHalfExtent.Y,
						(CornerIndex & 4) ? BoxHalfExtent.Z : -BoxHalfExtent.Z);
					Corners[CornerIndex] = BoxCenter + BoxRotation.RotateVector(LocalCorner);
					SignedDistances[CornerIndex] = FVector::DotProduct(
						Corners[CornerIndex] - SliceOrigin, AxisDirection);
					IslandProjectedBoundsPoints.Emplace(
						FVector::DotProduct(Corners[CornerIndex], PlaneU),
						FVector::DotProduct(Corners[CornerIndex], PlaneV));
					const float AxisCoordinate = FVector::DotProduct(
						Corners[CornerIndex], AxisDirection);
					IslandMinAxisCoordinate = FMath::Min(
						IslandMinAxisCoordinate, AxisCoordinate);
					IslandMaxAxisCoordinate = FMath::Max(
						IslandMaxAxisCoordinate, AxisCoordinate);
				}
				if (Member.bHasOrientedSDFBounds)
				{
					for (const FVector& Corner : Corners)
					{
						IslandSDFAxisSamples.Add(Corner);
					}
					++IslandSDFAxisContributorCount;
				}
				++IslandBoundsContributorCount;

				bool bMemberContributed = false;
				const auto AccumulateCrossSectionPoint =
					[&](const FVector& Point)
					{
						const float U = FVector::DotProduct(Point, PlaneU);
						const float V = FVector::DotProduct(Point, PlaneV);
						MinU = FMath::Min(MinU, U);
						MaxU = FMath::Max(MaxU, U);
						MinV = FMath::Min(MinV, V);
						MaxV = FMath::Max(MaxV, V);
						bMemberContributed = true;
					};

				for (const int32* Edge : BoxEdges)
				{
					const int32 AIndex = Edge[0];
					const int32 BIndex = Edge[1];
					const float DistanceA = SignedDistances[AIndex];
					const float DistanceB = SignedDistances[BIndex];

					if (FMath::Abs(DistanceA) <= SliceTolerance)
					{
						AccumulateCrossSectionPoint(Corners[AIndex]);
					}
					if (FMath::Abs(DistanceB) <= SliceTolerance)
					{
						AccumulateCrossSectionPoint(Corners[BIndex]);
					}
					if ((DistanceA < -SliceTolerance && DistanceB > SliceTolerance) ||
						(DistanceA > SliceTolerance && DistanceB < -SliceTolerance))
					{
						const float Alpha = DistanceA / (DistanceA - DistanceB);
						AccumulateCrossSectionPoint(FMath::Lerp(
							Corners[AIndex], Corners[BIndex], Alpha));
					}
				}

				CrossSectionContributorCount += bMemberContributed ? 1 : 0;
			}

			// The island axis is the principal axis of the oriented SDF bounds in their captured world pose.
			// Sampling every corner preserves both each volume's orientation and the separation between members.
			// A nearly isotropic covariance has no meaningful axis, so it keeps the configured magnitude rather
			// than selecting an arbitrary world basis.
			FVector IslandAxisDirection = FVector::ZeroVector;
			bool bHasIslandAxis = false;
			float IslandAxisPrincipalVariance = 0.0f;
			float IslandAxisOtherVarianceMean = 0.0f;
			if (IslandSDFAxisSamples.Num() >= 8)
			{
				FVector SampleCenter = FVector::ZeroVector;
				for (const FVector& Sample : IslandSDFAxisSamples)
				{
					SampleCenter += Sample;
				}
				SampleCenter /= static_cast<double>(IslandSDFAxisSamples.Num());

				double CovXX = 0.0;
				double CovXY = 0.0;
				double CovXZ = 0.0;
				double CovYY = 0.0;
				double CovYZ = 0.0;
				double CovZZ = 0.0;
				for (const FVector& Sample : IslandSDFAxisSamples)
				{
					const FVector Delta = Sample - SampleCenter;
					CovXX += Delta.X * Delta.X;
					CovXY += Delta.X * Delta.Y;
					CovXZ += Delta.X * Delta.Z;
					CovYY += Delta.Y * Delta.Y;
					CovYZ += Delta.Y * Delta.Z;
					CovZZ += Delta.Z * Delta.Z;
				}
				const double InvSampleCount = 1.0 /
					static_cast<double>(IslandSDFAxisSamples.Num());
				CovXX *= InvSampleCount;
				CovXY *= InvSampleCount;
				CovXZ *= InvSampleCount;
				CovYY *= InvSampleCount;
				CovYZ *= InvSampleCount;
				CovZZ *= InvSampleCount;

				IslandAxisDirection = FVector::ForwardVector;
				if (CovYY > CovXX && CovYY >= CovZZ)
				{
					IslandAxisDirection = FVector::RightVector;
				}
				else if (CovZZ > CovXX && CovZZ > CovYY)
				{
					IslandAxisDirection = FVector::UpVector;
				}
				for (int32 Iteration = 0; Iteration < 8; ++Iteration)
				{
					FVector NextAxis(
						CovXX * IslandAxisDirection.X + CovXY * IslandAxisDirection.Y +
							CovXZ * IslandAxisDirection.Z,
						CovXY * IslandAxisDirection.X + CovYY * IslandAxisDirection.Y +
							CovYZ * IslandAxisDirection.Z,
						CovXZ * IslandAxisDirection.X + CovYZ * IslandAxisDirection.Y +
							CovZZ * IslandAxisDirection.Z);
					if (!NextAxis.Normalize())
					{
						IslandAxisDirection = FVector::ZeroVector;
						break;
					}
					IslandAxisDirection = NextAxis;
				}

				if (!IslandAxisDirection.IsNearlyZero())
				{
					const FVector CovarianceTimesAxis(
						CovXX * IslandAxisDirection.X + CovXY * IslandAxisDirection.Y +
							CovXZ * IslandAxisDirection.Z,
						CovXY * IslandAxisDirection.X + CovYY * IslandAxisDirection.Y +
							CovYZ * IslandAxisDirection.Z,
						CovXZ * IslandAxisDirection.X + CovYZ * IslandAxisDirection.Y +
							CovZZ * IslandAxisDirection.Z);
					IslandAxisPrincipalVariance = static_cast<float>(FVector::DotProduct(
						IslandAxisDirection, CovarianceTimesAxis));
					const float TotalVariance = static_cast<float>(CovXX + CovYY + CovZZ);
					IslandAxisOtherVarianceMean = FMath::Max(
						0.0f, (TotalVariance - IslandAxisPrincipalVariance) * 0.5f);
					constexpr float MinIslandAxisAnisotropyRatio = 1.05f;
					bHasIslandAxis = IslandAxisPrincipalVariance >
						IslandAxisOtherVarianceMean * MinIslandAxisAnisotropyRatio;
				}
			}

			// Re-centring happens only where the composite cross-section genuinely holds two or more
			// surfaces. Where the island graph merely connects but only one surface exists at the latch
			// height, the original latch axis is safer.
			if (CrossSectionContributorCount >= 2)
			{
				const FVector PreviousAxisOrigin = State.PathAxisOrigin;
				const float PreservedAxialCoordinate = FVector::DotProduct(
					PreviousAxisOrigin, AxisDirection);
				State.PathAxisOrigin =
					PlaneU * ((MinU + MaxU) * 0.5f) +
					PlaneV * ((MinV + MaxV) * 0.5f) +
					AxisDirection * PreservedAxialCoordinate;

				const float RecenteredAxisDistance = FVector::DotProduct(
					State.PathSurfaceWorld - State.PathAxisOrigin, AxisDirection);
				const FVector RecenteredAxisPoint =
					State.PathAxisOrigin + AxisDirection * RecenteredAxisDistance;
				State.PathLatchRadial = (State.PathSurfaceWorld - RecenteredAxisPoint)
					.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathNormalWorld);
				State.PathCircumferenceDir = FVector::CrossProduct(
					AxisDirection, State.PathLatchRadial)
					.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir) *
					State.PathWindingSign;
				State.PathTangentWorld = ComputeSurfaceVectorFieldTangent(
					State.PathAxisOrigin, State.PathAxisDirection, State.PathLatchRadial,
					State.PathWindingSign, State.PathSurfaceWorld, State.PathNormalWorld,
					Ctx, State.PathCircumferenceDir);

				UE_LOG(LogRopeWrap, Log,
					TEXT("[%s] Composite wrap axis recentered: contributors=%d oldOrigin=%s "
						"newOrigin=%s transverseShift=%.2fcm direction=%s"),
					*Ctx.OwnerName, CrossSectionContributorCount,
					*PreviousAxisOrigin.ToString(), *State.PathAxisOrigin.ToString(),
					FVector::Dist(PreviousAxisOrigin, State.PathAxisOrigin),
					*State.PathAxisDirection.ToString());
			}
			else
			{
				UE_LOG(LogRopeWrap, VeryVerbose,
					TEXT("[%s] Composite wrap axis kept latch origin: crossSectionContributors=%d"),
					*Ctx.OwnerName, CrossSectionContributorCount);
			}

			// The support query starts outside the whole composite cross-section rather than near the arm
			// surface currently selected. Extra margin is added to the outline half-diagonal so a collider on
			// the far side of the cross-section can be evaluated by the same probe.
			const float CurrentAxisDistance = FVector::DotProduct(
				State.PathSurfaceWorld - State.PathAxisOrigin, AxisDirection);
			const FVector CurrentAxisPoint =
				State.PathAxisOrigin + AxisDirection * CurrentAxisDistance;
			const float CurrentSurfaceRadius = FVector::Dist(
				State.PathSurfaceWorld, CurrentAxisPoint);
			float IslandOuterRadius = 0.0f;
			if (IslandProjectedBoundsPoints.Num() > 0)
			{
				const float AxisOriginU = FVector::DotProduct(State.PathAxisOrigin, PlaneU);
				const float AxisOriginV = FVector::DotProduct(State.PathAxisOrigin, PlaneV);
				for (const FVector2D& Point : IslandProjectedBoundsPoints)
				{
					IslandOuterRadius = FMath::Max(IslandOuterRadius,
						static_cast<float>(FVector2D::Distance(
							Point, FVector2D(AxisOriginU, AxisOriginV))));
				}

				const float AxisOriginCoordinate = FVector::DotProduct(
					State.PathAxisOrigin, AxisDirection);
				State.PathCompositeAxisMinDistance =
					IslandMinAxisCoordinate - AxisOriginCoordinate;
				State.PathCompositeAxisMaxDistance =
					IslandMaxAxisCoordinate - AxisOriginCoordinate;
				State.bPathCompositeAxisRangeValid =
					State.PathCompositeAxisMinDistance <= State.PathCompositeAxisMaxDistance;
			}
			const float CrossSectionHalfU = CrossSectionContributorCount > 0
				? FMath::Max(0.0f, (MaxU - MinU) * 0.5f)
				: 0.0f;
			const float CrossSectionHalfV = CrossSectionContributorCount > 0
				? FMath::Max(0.0f, (MaxV - MinV) * 0.5f)
				: 0.0f;
			const float CrossSectionOuterRadius = FMath::Sqrt(
				FMath::Square(CrossSectionHalfU) + FMath::Square(CrossSectionHalfV));
			const float ProbeMargin = FMath::Max(
				Sim.SegmentLength, Ctx.GetContactRadius() * 2.0f);
			const float CrossSectionProbeRadius = FMath::Max(
				CurrentSurfaceRadius, CrossSectionOuterRadius);
			State.PathCompositeHelixRadius = FMath::Max(
				CurrentSurfaceRadius, IslandOuterRadius);
			State.PathCompositeProbeRadius =
				CrossSectionProbeRadius + ProbeMargin;
			State.PathCompositeSweepRadial = State.PathLatchRadial.GetSafeNormal(
				KINDA_SMALL_NUMBER, State.PathNormalWorld);
			State.PathCompositeSweepAngleRad = 0.0f;

			// The contact span remains the preferred pitch source, but a straight flight guide can put that span
			// exactly in the wrapping plane and therefore erase its axial component. In that case the configured
			// pitch supplies a stable helical baseline. Contact evidence is blended in smoothly as its axial
			// component grows, while the fallback sign uses the side of the SDF island with more axial room.
			const TCHAR* PitchSource = TEXT("NotUsed");
			float PitchAxisComponent = 0.0f;
			float PitchCircumferenceComponent = 0.0f;
			float ContactPitchScale = 0.0f;
			float ContactPitchConfidence = 0.0f;
			float GeometryPitchFactor = 1.0f;
			float RopePlaneIslandAxisAlignment = 0.0f;
			float FallbackPitchScale = 0.0f;
			float PositiveAxisRoom = 0.0f;
			float NegativeAxisRoom = 0.0f;
			float UnclampedPitchScale = 0.0f;
			float AxisLimitedMaxAbsPitch = 0.0f;
			float PitchAvailableAxisDistance = 0.0f;
			float PitchUsableAxisDistance = 0.0f;
			float PitchPlannedBaseTravel = 0.0f;
			float PitchAxisSafetyMargin = 0.0f;
			bool bPitchClampedByAxisRange = false;
			const bool bHasContactSpan = Ctx.TravelFrame && Ctx.TravelFrame->bValid &&
				!Ctx.TravelFrame->SpanDirection.IsNearlyZero();
			const FVector ContactTailDirection = bHasContactSpan
				? Ctx.TravelFrame->SpanDirection.GetSafeNormal()
				: LatchTangentWorld;
			const FVector RadialDirection = State.PathCompositeSweepRadial;
			const FVector TangentialContactDirection =
				(ContactTailDirection - FVector::DotProduct(
					ContactTailDirection, RadialDirection) * RadialDirection)
				.GetSafeNormal();

			if (!TangentialContactDirection.IsNearlyZero())
			{
				const FVector BaseCircumference = FVector::CrossProduct(
					AxisDirection, RadialDirection)
					.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir);
				State.PathWindingSign = FVector::DotProduct(
					BaseCircumference, TangentialContactDirection) < 0.0f ? -1.0f : 1.0f;
				State.PathCircumferenceDir =
					BaseCircumference * State.PathWindingSign;
				PitchAxisComponent = FVector::DotProduct(
					TangentialContactDirection, AxisDirection);
				PitchCircumferenceComponent = FVector::DotProduct(
					TangentialContactDirection, State.PathCircumferenceDir);

				constexpr float MinContactCircumferenceComponent = 0.1f;
				if (PitchCircumferenceComponent >= MinContactCircumferenceComponent)
				{
					ContactPitchScale =
						PitchAxisComponent / PitchCircumferenceComponent;

					const float ConfiguredFallbackSign =
						Ctx.Config.WrappingHelixPitchScale < 0.0f ? -1.0f : 1.0f;
					float FallbackPitchSign = ConfiguredFallbackSign;
					if (State.bPathCompositeAxisRangeValid)
					{
						PositiveAxisRoom = FMath::Max(
							0.0f, State.PathCompositeAxisMaxDistance - CurrentAxisDistance);
						NegativeAxisRoom = FMath::Max(
							0.0f, CurrentAxisDistance - State.PathCompositeAxisMinDistance);
						if (!FMath::IsNearlyEqual(PositiveAxisRoom, NegativeAxisRoom))
						{
							FallbackPitchSign = PositiveAxisRoom > NegativeAxisRoom ? 1.0f : -1.0f;
						}
					}
					const FVector RopePlaneNormal = Ctx.bHasGuidePlaneNormal
						? Ctx.GuidePlaneNormal.GetSafeNormal(
							KINDA_SMALL_NUMBER, AxisDirection)
						: AxisDirection;
					if (bHasIslandAxis)
					{
						RopePlaneIslandAxisAlignment = FMath::Clamp(
							FMath::Abs(static_cast<float>(FVector::DotProduct(
								RopePlaneNormal, IslandAxisDirection))), 0.0f, 1.0f);
						GeometryPitchFactor = FMath::Sqrt(FMath::Max(
							0.0f, 1.0f - FMath::Square(RopePlaneIslandAxisAlignment)));
					}
					FallbackPitchScale = FallbackPitchSign *
						FMath::Abs(Ctx.Config.WrappingHelixPitchScale) * GeometryPitchFactor;

					constexpr float ContactPitchConfidenceStart = 0.02f;
					constexpr float ContactPitchConfidenceFull = 0.15f;
					ContactPitchConfidence = FMath::SmoothStep(
						ContactPitchConfidenceStart, ContactPitchConfidenceFull,
						FMath::Abs(PitchAxisComponent));
					UnclampedPitchScale = FMath::Lerp(
						FallbackPitchScale, ContactPitchScale, ContactPitchConfidence);
					State.PathCompositeHelixPitchScale = UnclampedPitchScale;

					// The sign and slope of the pitch taken from the contact are preserved, and only its
					// magnitude is reduced so the whole planned helix fits within the island's axial extent in
					// that direction. Since each step's real expression makes the axial travel depend on the
					// pitch through a square-root term, the pitch-zero travel including the radial entry is
					// summed first as the baseline.
					if (State.bPathCompositeAxisRangeValid &&
						!FMath::IsNearlyZero(UnclampedPitchScale))
					{
						PitchAvailableAxisDistance = UnclampedPitchScale > 0.0f
							? State.PathCompositeAxisMaxDistance - CurrentAxisDistance
							: CurrentAxisDistance - State.PathCompositeAxisMinDistance;
						// A fraction of one node is left as boundary margin so a pitch that wrapped well is
						// not reduced unnecessarily. Margins smaller than the contact radius are vulnerable to
						// the quantization of the SDF cap.
						PitchAxisSafetyMargin = FMath::Max(
							Ctx.GetContactRadius(), Sim.SegmentLength * 0.25f);
						PitchUsableAxisDistance = FMath::Max(
							0.0f, PitchAvailableAxisDistance - PitchAxisSafetyMargin);

						const float SegmentLength = FMath::Max(
							Sim.SegmentLength, KINDA_SMALL_NUMBER);
						const int32 PlannedPathSegmentCount = FMath::Max(
							0, Sim.Num() - LatchAnchor.NodeIndex - 1);
						const int32 RadiusEntrySegmentCount =
							ComputeCompositeRadiusEntryStepCount(
								CurrentSurfaceRadius, State.PathCompositeHelixRadius,
								SegmentLength);
						float PreviousPlannedRadius = CurrentSurfaceRadius;
						for (int32 StepIndex = 1;
							StepIndex <= PlannedPathSegmentCount; ++StepIndex)
						{
							const FCompositeHelixStepKinematics Step = EvaluateCompositeHelixStep(
								StepIndex, SegmentLength, RadiusEntrySegmentCount,
								CurrentSurfaceRadius, State.PathCompositeHelixRadius,
								PreviousPlannedRadius, 0.0f, 1.0f);
							PitchPlannedBaseTravel += Step.BaseTangentialStep;
							PreviousPlannedRadius = Step.Radius;
						}

						if (PitchPlannedBaseTravel > KINDA_SMALL_NUMBER)
						{
							const float MaxAxisRatio = FMath::Clamp(
								PitchUsableAxisDistance / PitchPlannedBaseTravel,
								0.0f, 0.999f);
							AxisLimitedMaxAbsPitch = MaxAxisRatio /
								FMath::Sqrt(FMath::Max(
									KINDA_SMALL_NUMBER, 1.0f - FMath::Square(MaxAxisRatio)));
							const float RawAbsPitch = FMath::Abs(UnclampedPitchScale);
							const float FittedAbsPitch = FMath::Min(
								RawAbsPitch, AxisLimitedMaxAbsPitch);
							State.PathCompositeHelixPitchScale =
								FMath::Sign(UnclampedPitchScale) * FittedAbsPitch;
							bPitchClampedByAxisRange =
								FittedAbsPitch + KINDA_SMALL_NUMBER < RawAbsPitch;
						}
					}
					if (ContactPitchConfidence >= 1.0f - KINDA_SMALL_NUMBER)
					{
						PitchSource = bHasContactSpan
							? TEXT("ContactSpan")
							: TEXT("LatchTangent");
					}
					else if (ContactPitchConfidence <= KINDA_SMALL_NUMBER)
					{
						PitchSource = TEXT("ConfiguredFallback");
					}
					else
					{
						PitchSource = TEXT("ContactFallbackBlend");
					}
				}
				else
				{
					PitchSource = TEXT("DegenerateCircumferenceZero");
				}
			}
			else
			{
				PitchSource = TEXT("DegenerateContactDirectionZero");
			}

			// Two tangents are separated from the same initial helix direction: the path tangent is projected
			// onto the normal plane for the surface anchor, while the raw guide keeps the pre-projection
			// direction, which stops the tail wobbling.
			State.PathCompositeRawGuideTangentWorld =
				(State.PathCircumferenceDir + AxisDirection *
					State.PathCompositeHelixPitchScale)
				.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir);
			State.PathTangentWorld = State.PathCompositeRawGuideTangentWorld;
			State.PathTangentWorld = (State.PathTangentWorld - FVector::DotProduct(
				State.PathTangentWorld, State.PathNormalWorld) * State.PathNormalWorld)
				.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir);

			UE_LOG(LogRopeWrap, Log,
				TEXT("[%s] Composite analytic helix initialized: crossSectionContributors=%d "
					"islandBoundsContributors=%d origin=%s direction=%s radial=%s "
					"probeRadius=%.2fcm helixRadius=%.2fcm latchRadius=%.2fcm "
					"axisRange=[%.2f,%.2f]cm winding=%+.0f "
					"pitch=%.3f rawPitch=%.3f contactPitch=%.3f fallbackPitch=%.3f "
					"contactConfidence=%.3f geometryPitchFactor=%.3f pitchSource=%s "
					"axisClamp=%d maxPitch=%.3f "
					"axisRoom=%.2fcm usableAxis=%.2fcm safety=%.2fcm plannedBase=%.2fcm "
					"contactComponents(axis=%.3f circumference=%.3f) "
					"fallbackRoom(positive=%.2fcm negative=%.2fcm) "
					"islandAxis(valid=%d contributors=%d direction=%s alignment=%.3f "
					"principalVariance=%.3f otherVarianceMean=%.3f)"),
				*Ctx.OwnerName, CrossSectionContributorCount, IslandBoundsContributorCount,
				*State.PathAxisOrigin.ToString(), *State.PathAxisDirection.ToString(),
				*State.PathCompositeSweepRadial.ToString(), State.PathCompositeProbeRadius,
				State.PathCompositeHelixRadius, CurrentSurfaceRadius,
				State.PathCompositeAxisMinDistance, State.PathCompositeAxisMaxDistance,
				State.PathWindingSign, State.PathCompositeHelixPitchScale,
				UnclampedPitchScale, ContactPitchScale, FallbackPitchScale,
				ContactPitchConfidence, GeometryPitchFactor, PitchSource,
				bPitchClampedByAxisRange ? 1 : 0,
				AxisLimitedMaxAbsPitch, PitchAvailableAxisDistance,
				PitchUsableAxisDistance, PitchAxisSafetyMargin, PitchPlannedBaseTravel,
				PitchAxisComponent, PitchCircumferenceComponent,
				PositiveAxisRoom, NegativeAxisRoom,
				bHasIslandAxis ? 1 : 0, IslandSDFAxisContributorCount,
				*IslandAxisDirection.ToString(), RopePlaneIslandAxisAlignment,
				IslandAxisPrincipalVariance, IslandAxisOtherVarianceMean);

			// A composite island owns several contact surfaces by itself. If an older secondary seed cut the
			// path short before the first secondary node, there would be no length left to trace the
			// arm-torso-arm outline, so in that case alone the whole tail is returned to the progressive path
			// and the contention between the secondary hold and the commit is removed.
			State.NumTailNodes = Sim.Num() - LatchAnchor.NodeIndex;
			if (State.SecondarySeedAnchors.Num() > 0)
			{
				UE_LOG(LogRopeWrap, Log,
					TEXT("[%s] Composite wrap island replaced %d secondary seed(s); restoredTailNodes=%d"),
					*Ctx.OwnerName, State.SecondarySeedAnchors.Num(), State.NumTailNodes);
				State.SecondarySeedAnchors.Reset();
			}
		}
	}

	FRopeWrapPathPoint LatchPoint;
	LatchPoint.SurfaceWorld = State.PathSurfaceWorld;
	LatchPoint.NormalWorld = State.PathNormalWorld;
	LatchPoint.TangentWorld = State.PathTangentWorld;
	LatchPoint.WrappingGuideTangentWorld = State.PathCompositeRawGuideTangentWorld;
	LatchPoint.bHasWrappingGuideTangent = State.bPathUsesPoseSpaceIsland;
	LatchPoint.Bone = State.PathCurrentBone;
	LatchPoint.Mesh = State.PathCurrentMesh;
	LatchPoint.DistanceFromLatch = 0.0f;
	LatchPoint.WrapAngleFromLatchRad = 0.0f;
	State.Path.Add(LatchPoint);
	State.PathCurrentDistance = 0.0f;
	State.PathSweepDistance = 0.0f;

	if (State.Path.Num() >= State.NumTailNodes)
	{
		FinishPathBuild(/*bFailed=*/false);
	}

	return true;
}

bool FRopeWrappingPhase::ProcessPathPointForAnchoring(int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx)
{
	if (PathIndex < State.LastAnchoredPathPointCount)
	{
		return true;
	}
	if (PathIndex != State.LastAnchoredPathPointCount ||
		!State.Path.IsValidIndex(PathIndex))
	{
		return false;
	}

	const FRopeSurfaceAnchor& LatchAnchor = State.LatchAnchor;
	const USceneComponent* Mesh = ResolveWrappingMesh(State, LatchAnchor);
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	const int32 NodeIndex = LatchAnchor.NodeIndex + PathIndex;
	if (!Sim.Positions.IsValidIndex(NodeIndex))
	{
		return false;
	}

	const FRopeWrapPathPoint& Point = State.Path[PathIndex];

	// A mid-air bridge chord, and a composite virtual helix point, have no surface frame and therefore
	// produce no anchor; after the commit those nodes remain free rope and the solver gives them their
	// chord or catenary shape. The anchor counter still has to advance: this function only processes a new
	// point when the path index equals the anchored path point count, so stopping here would block anchor
	// creation for every surface path point after the bridge re-enters the surface.
	if (Point.bBridge || Point.bVirtual)
	{
		State.LastAnchoredPathPointCount = PathIndex + 1;
		return true;
	}

	// The heart of it: the bone the path point selected is used as the bone that owns the anchor.
	// Storing every anchor in the latch bone's local space meant that even when the path crossed onto a
	// neighbouring bone's surface, the wrapped and hold stages still appeared pinned to a single bone.
	// A path point with no bone is treated as an analytic helix or legacy fallback and uses the latch bone.
	FName AnchorBone = Point.Bone.IsNone() ? LatchAnchor.Bone : Point.Bone;
	const USceneComponent* AnchorMesh = Point.Mesh.Get();
	if (!AnchorMesh)
	{
		AnchorMesh = Mesh;
	}
	if (!AnchorMesh || AnchorBone.IsNone())
	{
		return false;
	}

	const FTransform BoneXform = ResolveBindingWorld(AnchorMesh, AnchorBone);

	FRopeSurfaceAnchor Anchor;
	Anchor.NodeIndex = NodeIndex;
	Anchor.Bone = AnchorBone;
	Anchor.Mesh = AnchorMesh;
	Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Point.SurfaceWorld);
	Anchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(Point.NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	Anchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(Point.TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	if (Point.bHasWrappingGuideTangent)
	{
		// The guide is stored in the anchor bone's local space too, so it follows the character's animation
		// during Wrapping without being reprojected onto the fine variation of the surface normal.
		Anchor.LocalWrappingGuideTangent = BoneXform.InverseTransformVectorNoScale(
			Point.WrappingGuideTangentWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, Anchor.LocalTangent);
		Anchor.bHasWrappingGuideTangent = true;
	}
	Anchor.StartWorldPosition = Sim.Positions[NodeIndex];
	Anchor.SurfaceOffset = FMath::Max(0.0f, Ctx.SurfaceOffset);
	Anchor.RopeDistance = Point.DistanceFromLatch;

	State.FirstNode = FMath::Min(State.FirstNode, NodeIndex);
	State.LastNode = FMath::Max(State.LastNode, NodeIndex);
	State.Anchors.Add(Anchor);
	State.LastAnchoredPathPointCount = PathIndex + 1;
	return true;
}

#pragma endregion
