// Copyright Epic Games, Inc. All Rights Reserved.

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

#pragma region Wrapping Front Motion and Path Sampling

void FRopeWrappingPhase::ApplyWrappingMotionOverrides(const FRopeSimState& Sim, float DeltaTime, const FContext& Ctx, FRopeNodeOverrideFrame& OutFrame)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_ApplyWrappingMotionOverrides);

	AdvanceWrappingFront(DeltaTime, Sim, Ctx);

	const int32 LatchNode = State.LatchAnchor.NodeIndex;
	if (State.Anchors.Num() == 0 ||
		!Sim.Positions.IsValidIndex(LatchNode) ||
		!Sim.PrevPositions.IsValidIndex(LatchNode))
	{
		return;
	}

	// Path points and anchors are produced together, in path and node index order. Resolving the moving
	// bone frames of the current frame once and reusing them avoids the quadratic cost of every node
	// below repeating the linear search over the path and anchors inside SampleWrappingPath.
	if (!BuildResolvedWrappingPath(ResolvedPathScratch))
	{
		return;
	}

	const float SurfaceOffset = FMath::Max(0.0f, Ctx.SurfaceOffset);
	FRopeWrapPathPoint FrontPoint;
	if (!SampleResolvedWrappingPath(
		ResolvedPathScratch, State.FrontDistance, SurfaceOffset, FrontPoint))
	{
		return;
	}

	const FVector FrontWorld = GetPathPointCenterlineWorld(FrontPoint, SurfaceOffset);
	// Positions already wrapped follow the projected surface path, while the direction the not-yet-wrapped
	// tail extends in prefers the ideal helix guide, which is never projected onto the SDF normal. The
	// sequential path, and older data with no guide, fall back to the surface tangent as before.
	const FVector TailGuideDirection =
		State.bPathUsesPoseSpaceIsland && FrontPoint.bHasWrappingGuideTangent
			? FrontPoint.WrappingGuideTangentWorld.GetSafeNormal(
				KINDA_SMALL_NUMBER, FrontPoint.TangentWorld)
			: FrontPoint.TangentWorld;
	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	// The range the real surface path and anchors own is kept separate from the range driven visually
	// during Wrapping. A composite analytic helix aligns the whole remaining tail along the current ideal
	// helix guide even when the path ended earlier, at the limit of the island's axis. NumTailNodes itself
	// still means the reduced path and commit range.
	const bool bDriveFullCompositeTail = State.bPathUsesPoseSpaceIsland;
	int32 TailEndNode = Sim.Num() - 1;
	if (!bDriveFullCompositeTail && State.NumTailNodes > 0)
	{
		// The sequential path keeps the existing policy: it moves only the nodes the path it actually
		// produced owns, and the free tail beyond that is left out of this stage's forced position
		// animation.
		TailEndNode = FMath::Min(TailEndNode, LatchNode + State.NumTailNodes - 1);
	}
	// Seed multiplexing: path-driven motion also ends before the first secondary seed node, which is the
	// same boundary the NumTailNodes clamp uses.
	// The secondary nodes are held against their own bones below, and the rope beyond them stays
	// solver-owned, which stops the front's straight extension dragging the rope past the far side of the
	// secondary target.
	for (const FRopeSurfaceAnchor& Secondary : State.SecondarySeedAnchors)
	{
		if (!bDriveFullCompositeTail && Secondary.NodeIndex > LatchNode)
		{
			TailEndNode = FMath::Min(TailEndNode, Secondary.NodeIndex - 1);
		}
	}

	OutFrame.EnsureSize(Sim.Num());
	for (int32 NodeIndex = LatchNode; NodeIndex <= TailEndNode; ++NodeIndex)
	{
		if (!Sim.Positions.IsValidIndex(NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(NodeIndex))
		{
			continue;
		}

	// A composite helix point whose radial SDF ray found no surface receives no position override in this
	// phase either. Letting the solver compute that node immediately removes the positional discontinuity
	// of every virtual guide disappearing at once at the moment of the wrapped commit.
		const int32 PathIndex = NodeIndex - LatchNode;
		if (State.Path.IsValidIndex(PathIndex) && State.Path[PathIndex].bVirtual)
		{
			continue;
		}

		const float NodeDistance = static_cast<float>(NodeIndex - LatchNode) * SegmentLength;
		FVector World = FVector::ZeroVector;
		if (NodeDistance <= State.FrontDistance + KINDA_SMALL_NUMBER)
		{
			// Nodes the wrapping front has already passed follow the same rope distance along the real SDF
			// projection path. Path points are produced at exactly one segment length apart, so the array
			// resolved once this frame can be indexed directly.
			if (State.Path.IsValidIndex(PathIndex) &&
				ResolvedPathScratch.IsValidIndex(PathIndex))
			{
				World = GetPathPointCenterlineWorld(
					ResolvedPathScratch[PathIndex], SurfaceOffset);
			}
			else
			{
				// Older anchors-only state, with no path, binary searches the anchor array resolved and
				// sorted this frame. Even in that edge state it does not rescan every anchor per node.
				FRopeWrapPathPoint NodePoint;
				if (!SampleResolvedWrappingPath(
					ResolvedPathScratch, NodeDistance, SurfaceOffset, NodePoint))
				{
					continue;
				}
				World = GetPathPointCenterlineWorld(NodePoint, SurfaceOffset);
			}
		}
		else
		{
			// The composite tail follows the initial ideal helix guide rather than the SDF projection
			// tangent. The positions keep using the projected surface path, so fine variation in the normal
			// does not shake the whole tail.
			// In composite mode this expression is applied out to the last rope node even when the real path
			// terminated earlier at the end of the island axis. No new surface anchor is created over that
			// stretch; it is purely visual alignment for the duration of Wrapping.
			//   tail position = current front position + helix guide direction * rope length remaining
			//   beyond the front
			World = FrontWorld + TailGuideDirection * (NodeDistance - State.FrontDistance);
		}

		OutFrame.SetPosition(NodeIndex, World, /*bZeroVelocity*/ true);
	}

	// Holding the secondary seed nodes: independently of the path and the front, they follow the surface
	// anchor frame of their own bone, which follows a moving target using the same expression as Hold does
	// while wrapped. A frame where the mesh is gone is skipped: the node stays put through the mask freeze,
	// and after the commit Hold detects the lost mesh properly and releases.
	for (const FRopeSurfaceAnchor& Secondary : State.SecondarySeedAnchors)
	{
		if (!Sim.Positions.IsValidIndex(Secondary.NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(Secondary.NodeIndex))
		{
			continue;
		}

		const USceneComponent* AnchorComp = Secondary.Mesh.Get();
		if (!AnchorComp)
		{
			continue;
		}

		FRopeBindingFrame Binding;
		Binding.Component = AnchorComp;
		Binding.SocketOrBone = Secondary.Bone;
		const FTransform BoneXform = ResolveBindingWorld(Binding);

		const FVector SurfaceWorld = BoneXform.TransformPosition(Secondary.LocalSurfacePosition);
		const FVector NormalWorld = BoneXform.TransformVectorNoScale(Secondary.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

		OutFrame.SetPosition(Secondary.NodeIndex,
			SurfaceWorld + NormalWorld * Secondary.SurfaceOffset, /*bZeroVelocity*/ true);
	}
}

void FRopeWrappingPhase::ApplyWrappingKinematicMask(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const
{
	OutFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
	// Only nodes whose positions were actually driven are kinematic. Freezing the untouched tail beyond a
	// sequential angle cap or a secondary seed as well would preserve the strain left over from Flight for
	// the whole of Wrapping, and it would contract like a rubber band the moment the inverse mass is
	// released at the wrapped commit. A tail or virtual node with no position override is left to the
	// solver to keep at its segment length even during Wrapping. An active virtual bridge is pinned again
	// by Hold after this call.
		const bool bWrappingDrivenNode =
			OutFrame.Flags.IsValidIndex(i) &&
			(OutFrame.Flags[i] & RopeNodeOverride::Position) != 0;
	// If resolving a binding fails for one frame, leaving the position override empty, an already
	// established surface anchor is held at its current position; otherwise the latch and the secondary
	// seeds would be released to the solver momentarily.
		const bool bExplicitAnchorNode =
			State.Anchors.ContainsByPredicate(
				[i](const FRopeSurfaceAnchor& Anchor) { return Anchor.NodeIndex == i; }) ||
			State.SecondarySeedAnchors.ContainsByPredicate(
				[i](const FRopeSurfaceAnchor& Anchor) { return Anchor.NodeIndex == i; });
		OutFrame.SetInvMass(i,
			(bStartPin || bWrappingDrivenNode || bExplicitAnchorNode) ? 0.0f : 1.0f);
	}
}

void FRopeWrappingPhase::AdvanceWrappingFront(float DeltaTime, const FRopeSimState& Sim, const FContext& Ctx)
{
	if (State.Anchors.Num() == 0 || State.NumTailNodes <= 0)
	{
		State.FrontDistance = 0.0f;
		State.FrontWrapAngleRad = 0.0f;
		return;
	}

	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	const float FullDistance = static_cast<float>(FMath::Max(0, State.NumTailNodes - 1)) * SegmentLength;
	float BuiltPathMaxDistance = 0.0f;
	float BuiltPathMaxAngleRad = 0.0f;
	for (const FRopeWrapPathPoint& Point : State.Path)
	{
		BuiltPathMaxDistance = FMath::Max(BuiltPathMaxDistance, Point.DistanceFromLatch);
		BuiltPathMaxAngleRad = FMath::Max(
			BuiltPathMaxAngleRad, Point.WrapAngleFromLatchRad);
	}
	// Only where Path is empty, as with legacy state or a failed initialization, does the anchor distance
	// serve as the fallback.
	if (State.Path.Num() == 0)
	{
		for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
		{
			BuiltPathMaxDistance = FMath::Max(
				BuiltPathMaxDistance, Anchor.RopeDistance);
		}
	}

	const float TargetFrontDistance = FMath::Min(FullDistance, BuiltPathMaxDistance);
	// The actual distance cap used this frame is preserved in the state so that advancing and committing
	// never look at different targets. Once the final path build finishes, this value becomes the final
	// completion distance.
	State.FrontTargetDistance = TargetFrontDistance;
	if (TargetFrontDistance <= KINDA_SMALL_NUMBER)
	{
		State.FrontDistance = 0.0f;
		return;
	}

	const float FullTailDelay = (FullDistance / SegmentLength) * Ctx.Config.WrappingTailDelayPerSegment;
	const float TotalDuration = FMath::Max(State.Duration + FullTailDelay, KINDA_SMALL_NUMBER);
	const float BaseFrontSpeed = FullDistance / TotalDuration;
	const auto EvaluateSpeedScale = [](float ProgressAlpha)
	{
		// The front's existing curve, slow at first and accelerating later, is preserved and only its input
		// phase changes, from distance progress to the angle progress shared by the single and composite
		// modes.
		const float StartSpeedScale = 0.5f;
		const float EndSpeedScale = 3.0f;
		return FMath::Lerp(StartSpeedScale, EndSpeedScale,
			RopeMath::SmoothStep(FMath::Clamp(ProgressAlpha, 0.0f, 1.0f)));
	};

	const float PreviousFrontDistance = State.FrontDistance;
	float PreviousFrontAngleRad = State.FrontWrapAngleRad;
	float ProgressAlpha = 0.0f;
	float SpeedScale = 1.0f;
	float TargetFrontAngleRad = 0.0f;
	float BaseAngularSpeedDegPerSec = 0.0f;
	const bool bUseAngleMappedFront = State.Path.Num() >= 2 &&
		BuiltPathMaxAngleRad > KINDA_SMALL_NUMBER;
	// Record how the front is currently driven so IsReadyToCommit can select the same policy.
	State.bFrontUsesAngleMapping = bUseAngleMappedFront;
	const TCHAR* FrontPolicy = bUseAngleMappedFront
		? TEXT("AngleMapped")
		: TEXT("DistanceFallback");

	if (bUseAngleMappedFront)
	{
		// On both modes the path increases monotonically in both distance and animation angle. Interpolation
		// in either direction is offered only over the path built so far, which stops the front jumping
		// ahead of the progressive build.
		// No unconsumed angle debt accumulates separately while the build cap is holding it back.
		const auto FindAngleAtDistance = [this](float SampleDistance)
		{
			if (State.Path.Num() == 0)
			{
				return 0.0f;
			}
			if (SampleDistance <= State.Path[0].DistanceFromLatch)
			{
				return State.Path[0].WrapAngleFromLatchRad;
			}
			if (SampleDistance >= State.Path.Last().DistanceFromLatch)
			{
				return State.Path.Last().WrapAngleFromLatchRad;
			}

			int32 LowerSearchIndex = 1;
			int32 UpperSearchIndex = State.Path.Num() - 1;
			while (LowerSearchIndex < UpperSearchIndex)
			{
				const int32 MidIndex = LowerSearchIndex + (UpperSearchIndex - LowerSearchIndex) / 2;
				if (State.Path[MidIndex].DistanceFromLatch < SampleDistance)
				{
					LowerSearchIndex = MidIndex + 1;
				}
				else
				{
					UpperSearchIndex = MidIndex;
				}
			}

			const FRopeWrapPathPoint& UpperPoint = State.Path[LowerSearchIndex];
			const FRopeWrapPathPoint& LowerPoint = State.Path[LowerSearchIndex - 1];
			const float DistanceSpan = UpperPoint.DistanceFromLatch - LowerPoint.DistanceFromLatch;
			if (DistanceSpan <= KINDA_SMALL_NUMBER)
			{
				return UpperPoint.WrapAngleFromLatchRad;
			}
			const float Alpha = FMath::Clamp(
				(SampleDistance - LowerPoint.DistanceFromLatch) / DistanceSpan,
				0.0f, 1.0f);
			return FMath::Lerp(
				LowerPoint.WrapAngleFromLatchRad, UpperPoint.WrapAngleFromLatchRad, Alpha);
		};

		const auto FindDistanceAtAngle = [this](float SampleAngleRad)
		{
			if (State.Path.Num() == 0)
			{
				return 0.0f;
			}
			if (SampleAngleRad <= State.Path[0].WrapAngleFromLatchRad)
			{
				return State.Path[0].DistanceFromLatch;
			}
			if (SampleAngleRad >= State.Path.Last().WrapAngleFromLatchRad)
			{
				return State.Path.Last().DistanceFromLatch;
			}

			int32 LowerSearchIndex = 1;
			int32 UpperSearchIndex = State.Path.Num() - 1;
			while (LowerSearchIndex < UpperSearchIndex)
			{
				const int32 MidIndex = LowerSearchIndex + (UpperSearchIndex - LowerSearchIndex) / 2;
				if (State.Path[MidIndex].WrapAngleFromLatchRad < SampleAngleRad)
				{
					LowerSearchIndex = MidIndex + 1;
				}
				else
				{
					UpperSearchIndex = MidIndex;
				}
			}

			const FRopeWrapPathPoint& UpperPoint = State.Path[LowerSearchIndex];
			const FRopeWrapPathPoint& LowerPoint = State.Path[LowerSearchIndex - 1];
			const float AngleSpan = UpperPoint.WrapAngleFromLatchRad - LowerPoint.WrapAngleFromLatchRad;
			if (AngleSpan <= KINDA_SMALL_NUMBER)
			{
				return UpperPoint.DistanceFromLatch;
			}
			const float Alpha = FMath::Clamp(
				(SampleAngleRad - LowerPoint.WrapAngleFromLatchRad) / AngleSpan,
				0.0f, 1.0f);
			return FMath::Lerp(
				LowerPoint.DistanceFromLatch, UpperPoint.DistanceFromLatch, Alpha);
		};

		// Even if no angle points existed for the first few frames and the distance fallback moved first,
		// the moment the angular path is ready it continues from the angle at the current distance, so the
		// position does not wind back to the latch.
		if (State.FrontWrapAngleRad <= KINDA_SMALL_NUMBER &&
			State.FrontDistance > KINDA_SMALL_NUMBER)
		{
			State.FrontWrapAngleRad = FindAngleAtDistance(State.FrontDistance);
			PreviousFrontAngleRad = State.FrontWrapAngleRad;
		}

		TargetFrontAngleRad = FindAngleAtDistance(TargetFrontDistance);
		// The final angle is unknown while the build is still running, so the current mean angular density
		// is extrapolated out to the full distance. That stops the easing tripling the instant the end of a
		// partially built path is reached.
		float EstimatedFinalAngleRad = TargetFrontAngleRad;
		if (!State.bPathBuildComplete && BuiltPathMaxDistance > KINDA_SMALL_NUMBER)
		{
			EstimatedFinalAngleRad = FMath::Max(
				EstimatedFinalAngleRad,
				BuiltPathMaxAngleRad * FullDistance / BuiltPathMaxDistance);
		}
		ProgressAlpha = EstimatedFinalAngleRad > KINDA_SMALL_NUMBER
			? State.FrontWrapAngleRad / EstimatedFinalAngleRad
			: 0.0f;
		SpeedScale = EvaluateSpeedScale(ProgressAlpha);
		BaseAngularSpeedDegPerSec = FMath::Clamp(
			Ctx.Config.WrappingAngularSpeedDegPerSec, 1.0f, 7200.0f);
		const float AngularStepRad = FMath::DegreesToRadians(
			BaseAngularSpeedDegPerSec * SpeedScale) * FMath::Max(0.0f, DeltaTime);
		State.FrontWrapAngleRad = FMath::Min(
			State.FrontWrapAngleRad + AngularStepRad,
			TargetFrontAngleRad);
		State.FrontDistance = FMath::Min(
			FindDistanceAtAngle(State.FrontWrapAngleRad),
			TargetFrontDistance);
	}
	else
	{
		// Only a legacy or degenerate path with no angle parameters falls back safely to the existing
		// distance-driven front.
		ProgressAlpha = State.FrontDistance / FMath::Max(FullDistance, KINDA_SMALL_NUMBER);
		SpeedScale = EvaluateSpeedScale(ProgressAlpha);
		const float FrontSpeed = BaseFrontSpeed * SpeedScale;
		State.FrontDistance = FMath::Min(
			State.FrontDistance + FrontSpeed * FMath::Max(0.0f, DeltaTime),
			TargetFrontDistance);
	}

	if (State.FrontMotionStartElapsed < 0.0f &&
		State.FrontDistance > PreviousFrontDistance + KINDA_SMALL_NUMBER)
	{
		State.FrontMotionStartElapsed = FMath::Max(
			0.0f, State.Elapsed - FMath::Max(0.0f, DeltaTime));
	}

	// Angle-mapped fronts use the real per-point animation angle in both modes. Only the legacy and
	// degenerate fallback estimates the timing log from the overall mean of the sequential accumulated
	// forward angle.
	const float ModeBuiltAngleRad = bUseAngleMappedFront
		? BuiltPathMaxAngleRad
		: State.PathForwardAngleRad;
	const float BuiltCmPerRad = ModeBuiltAngleRad > KINDA_SMALL_NUMBER
		? BuiltPathMaxDistance / ModeBuiltAngleRad
		: 0.0f;
	const float EstimatedFrontAngleRad = bUseAngleMappedFront
		? State.FrontWrapAngleRad
		: (BuiltPathMaxDistance > KINDA_SMALL_NUMBER
			? ModeBuiltAngleRad * FMath::Clamp(
				State.FrontDistance / BuiltPathMaxDistance, 0.0f, 1.0f)
			: 0.0f);
	if (!bUseAngleMappedFront)
	{
		TargetFrontAngleRad = BuiltPathMaxDistance > KINDA_SMALL_NUMBER
			? ModeBuiltAngleRad * FMath::Clamp(
				TargetFrontDistance / BuiltPathMaxDistance, 0.0f, 1.0f)
			: 0.0f;
		BaseAngularSpeedDegPerSec = BuiltCmPerRad > KINDA_SMALL_NUMBER
			? FMath::RadiansToDegrees(BaseFrontSpeed / BuiltCmPerRad)
			: 0.0f;
	}
	State.FrontTargetWrapAngleRad = TargetFrontAngleRad;

	// The four-stage completion gate is armed only against the final path, never the progressive build's
	// temporary cap. The first time both the angle and the distance are complete is recorded, which allows
	// a short post-front settle time to be measured instead of the previous per-segment tail deadline.
	const float DistanceCompletionTolerance = FMath::Max(0.01f, SegmentLength * 0.001f);
	const float AngleCompletionToleranceRad = FMath::DegreesToRadians(0.1f);
	const bool bAngleTargetDone = !bUseAngleMappedFront ||
		State.FrontWrapAngleRad + AngleCompletionToleranceRad >= TargetFrontAngleRad;
	const bool bDistanceTargetDone =
		State.FrontDistance + DistanceCompletionTolerance >= TargetFrontDistance;
	const bool bFinalFrontTargetDone = State.bPathBuildComplete &&
		!State.bPathBuildFailed && bAngleTargetDone && bDistanceTargetDone;
	if (bUseAngleMappedFront && bFinalFrontTargetDone &&
		State.FrontReachedTargetElapsed < 0.0f)
	{
		State.FrontReachedTargetElapsed = State.Elapsed;
		UE_LOG(LogRopeWrap, Display,
			TEXT("[%s] Wrapping front reached angle+distance target: mode=%s "
				"angle=%.2f/%.2fdeg distance=%.2f/%.2fcm postSettle=%.3fs"),
			*Ctx.OwnerName,
			State.bPathUsesPoseSpaceIsland ? TEXT("Composite") : TEXT("Single"),
			FMath::RadiansToDegrees(State.FrontWrapAngleRad),
			FMath::RadiansToDegrees(TargetFrontAngleRad),
			State.FrontDistance, TargetFrontDistance,
			Ctx.Config.WrappingPostFrontSettleTime);
	}

	// Emitted once, after the progressive build finishes. The measured linear speed of the two modes can
	// differ with the path's local centimetres per radian, but the measured angular speed has to follow the
	// same shared angular policy.
	if (State.bPathBuildComplete && !State.bFrontMotionStartLogged)
	{
		const float SafeDeltaTime = FMath::Max(0.0f, DeltaTime);
		const float MeasuredFrontSpeed = SafeDeltaTime > KINDA_SMALL_NUMBER
			? (State.FrontDistance - PreviousFrontDistance) / SafeDeltaTime
			: 0.0f;
		const float MeasuredAngularSpeedDegPerSec = SafeDeltaTime > KINDA_SMALL_NUMBER
			? (bUseAngleMappedFront
				? FMath::RadiansToDegrees(
					(State.FrontWrapAngleRad - PreviousFrontAngleRad) / SafeDeltaTime)
				: (BuiltCmPerRad > KINDA_SMALL_NUMBER
					? FMath::RadiansToDegrees(MeasuredFrontSpeed / BuiltCmPerRad)
					: 0.0f))
			: 0.0f;

		UE_LOG(LogRopeWrap, Display,
			TEXT("[%s] Wrapping front speed metrics: mode=%s policy=%s "
				"legacyBaseLinearSpeed=%.2fcm/s measuredLinearSpeed=%.2fcm/s "
				"baseAngularSpeed=%.2fdeg/s measuredAngularSpeed=%.2fdeg/s speedScale=%.3f "
				"builtCmPerRad=%.3f frontAngle=%.2fdeg targetAngle=%.2fdeg "
				"front=%.2fcm target=%.2fcm"),
			*Ctx.OwnerName,
			State.bPathUsesPoseSpaceIsland ? TEXT("Composite") : TEXT("Single"),
			FrontPolicy, BaseFrontSpeed, MeasuredFrontSpeed,
			BaseAngularSpeedDegPerSec, MeasuredAngularSpeedDegPerSec, SpeedScale,
			BuiltCmPerRad, FMath::RadiansToDegrees(EstimatedFrontAngleRad),
			FMath::RadiansToDegrees(TargetFrontAngleRad),
			State.FrontDistance, TargetFrontDistance);
		State.bFrontMotionStartLogged = true;
	}

	const bool bReachedFinalBuiltFront = State.bPathBuildComplete &&
		bAngleTargetDone && bDistanceTargetDone;
	if (bReachedFinalBuiltFront && !State.bFrontMotionCompletionLogged)
	{
		const float FrontMotionElapsed = State.FrontMotionStartElapsed >= 0.0f
			? FMath::Max(0.0f, State.Elapsed - State.FrontMotionStartElapsed)
			: 0.0f;
		const float FrontAngleDeg = FMath::RadiansToDegrees(EstimatedFrontAngleRad);
		const float FrontTurns = FrontAngleDeg / 360.0f;
		const float AverageDegPerSec = FrontMotionElapsed > KINDA_SMALL_NUMBER
			? FrontAngleDeg / FrontMotionElapsed
			: 0.0f;
		const float AverageSecondsPerTurn = FrontTurns > KINDA_SMALL_NUMBER
			? FrontMotionElapsed / FrontTurns
			: 0.0f;
		UE_LOG(LogRopeWrap, Display,
			TEXT("[%s] Wrapping front timing metrics: mode=%s policy=%s "
				"frontElapsed=%.3fs frontAngle=%.2fdeg turns=%.3f "
				"averageDegPerSec=%.2f averageSecondsPerTurn=%.3fs "
				"frontDistance=%.2fcm builtDistance=%.2fcm"),
			*Ctx.OwnerName,
			State.bPathUsesPoseSpaceIsland ? TEXT("Composite") : TEXT("Single"),
			FrontPolicy, FrontMotionElapsed, FrontAngleDeg, FrontTurns,
			AverageDegPerSec, AverageSecondsPerTurn,
			State.FrontDistance, BuiltPathMaxDistance);
		State.bFrontMotionCompletionLogged = true;
	}
}

bool FRopeWrappingPhase::ResolveWrappingAnchorPoint(
	const FRopeSurfaceAnchor& Anchor, FRopeWrapPathPoint& OutPoint) const
{
	const USceneComponent* Mesh = ResolveWrappingMesh(State, Anchor);
	if (!Mesh || Anchor.Bone.IsNone())
	{
		return false;
	}

	const FTransform BoneXform = ResolveBindingWorld(Mesh, Anchor.Bone);
	OutPoint = FRopeWrapPathPoint();
	OutPoint.SurfaceWorld = BoneXform.TransformPosition(Anchor.LocalSurfacePosition);
	OutPoint.NormalWorld = BoneXform.TransformVectorNoScale(Anchor.LocalNormal)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	OutPoint.TangentWorld = BoneXform.TransformVectorNoScale(Anchor.LocalTangent);
	OutPoint.TangentWorld = (OutPoint.TangentWorld -
		FVector::DotProduct(OutPoint.TangentWorld, OutPoint.NormalWorld) * OutPoint.NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(OutPoint.NormalWorld));
	if (Anchor.bHasWrappingGuideTangent)
	{
		// Unlike the surface tangent, the guide is not projected onto the normal plane. This vector is
		// consumed only as the visual extension direction of the tail behind the front, never as a collision
		// or pinning frame.
		OutPoint.WrappingGuideTangentWorld = BoneXform.TransformVectorNoScale(
			Anchor.LocalWrappingGuideTangent)
			.GetSafeNormal(KINDA_SMALL_NUMBER, OutPoint.TangentWorld);
		OutPoint.bHasWrappingGuideTangent = true;
	}
	OutPoint.Bone = Anchor.Bone;
	OutPoint.Mesh = Mesh;
	OutPoint.DistanceFromLatch = Anchor.RopeDistance;
	return true;
}

FVector FRopeWrappingPhase::GetPathPointCenterlineWorld(
	const FRopeWrapPathPoint& Point, float SurfaceOffset)
{
	const float ClampedSurfaceOffset = FMath::Max(0.0f, SurfaceOffset);
	// Only a virtual point stores the rope centreline directly in its surface field. A sequential bridge
	// merely has no anchor and stores the same pre-offset reference position an ordinary surface point does.
	return Point.SurfaceWorld +
		(Point.bVirtual
			? FVector::ZeroVector
			: Point.NormalWorld * ClampedSurfaceOffset);
}

FVector FRopeWrappingPhase::EncodePathPointPositionFromCenterline(
	const FVector& CenterlineWorld, const FVector& NormalWorld,
	bool bVirtual, float SurfaceOffset)
{
	const float ClampedSurfaceOffset = FMath::Max(0.0f, SurfaceOffset);
	return CenterlineWorld -
		(bVirtual
			? FVector::ZeroVector
			: NormalWorld * ClampedSurfaceOffset);
}

void FRopeWrappingPhase::InterpolateWrappingPathPoints(
	const FRopeWrapPathPoint& LowerPoint, const FRopeWrapPathPoint& UpperPoint,
	float SampleDistance, float SurfaceOffset, FRopeWrapPathPoint& OutPoint)
{
	if (FMath::Abs(UpperPoint.DistanceFromLatch - LowerPoint.DistanceFromLatch)
		<= KINDA_SMALL_NUMBER)
	{
		OutPoint = LowerPoint;
		OutPoint.DistanceFromLatch = SampleDistance;
		return;
	}

	OutPoint = FRopeWrapPathPoint();
	const float Alpha = FMath::Clamp(
		(SampleDistance - LowerPoint.DistanceFromLatch) /
		(UpperPoint.DistanceFromLatch - LowerPoint.DistanceFromLatch),
		0.0f, 1.0f);
	// Both flags take part in the anchor omission policy, but only the virtual flag decides the coordinate
	// storage convention.
	OutPoint.bBridge = LowerPoint.bBridge || UpperPoint.bBridge;
	OutPoint.bVirtual = LowerPoint.bVirtual || UpperPoint.bVirtual;
	OutPoint.NormalWorld = FMath::Lerp(
		LowerPoint.NormalWorld, UpperPoint.NormalWorld, Alpha)
		.GetSafeNormal(KINDA_SMALL_NUMBER, LowerPoint.NormalWorld);
	// The two storage representations, surface-relative and virtual centreline, are never mixed directly.
	// Both endpoints are resolved to centrelines and interpolated, and the result is converted back once
	// into whichever convention the output point follows.
	const FVector LowerCenterlineWorld = GetPathPointCenterlineWorld(
		LowerPoint, SurfaceOffset);
	const FVector UpperCenterlineWorld = GetPathPointCenterlineWorld(
		UpperPoint, SurfaceOffset);
	const FVector CenterlineWorld = FMath::Lerp(
		LowerCenterlineWorld, UpperCenterlineWorld, Alpha);
	OutPoint.SurfaceWorld = EncodePathPointPositionFromCenterline(
		CenterlineWorld, OutPoint.NormalWorld, OutPoint.bVirtual, SurfaceOffset);
	OutPoint.TangentWorld = FMath::Lerp(
		LowerPoint.TangentWorld, UpperPoint.TangentWorld, Alpha);
	OutPoint.TangentWorld = (OutPoint.TangentWorld -
		FVector::DotProduct(OutPoint.TangentWorld, OutPoint.NormalWorld) * OutPoint.NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, LowerPoint.TangentWorld);
	OutPoint.bHasWrappingGuideTangent =
		LowerPoint.bHasWrappingGuideTangent || UpperPoint.bHasWrappingGuideTangent;
	if (OutPoint.bHasWrappingGuideTangent)
	{
		// As the front moves between two path points the guide is interpolated by the same alpha, so the
		// direction does not change in steps at node boundaries. A side with no guide falls back to the
		// surface tangent.
		const FVector LowerGuide = LowerPoint.bHasWrappingGuideTangent
			? LowerPoint.WrappingGuideTangentWorld
			: LowerPoint.TangentWorld;
		const FVector UpperGuide = UpperPoint.bHasWrappingGuideTangent
			? UpperPoint.WrappingGuideTangentWorld
			: UpperPoint.TangentWorld;
		OutPoint.WrappingGuideTangentWorld = FMath::Lerp(
			LowerGuide, UpperGuide, Alpha)
			.GetSafeNormal(KINDA_SMALL_NUMBER, LowerGuide);
	}
	OutPoint.Bone = Alpha < 0.5f ? LowerPoint.Bone : UpperPoint.Bone;
	OutPoint.Mesh = Alpha < 0.5f ? LowerPoint.Mesh : UpperPoint.Mesh;
	OutPoint.DistanceFromLatch = SampleDistance;
	OutPoint.WrapAngleFromLatchRad = FMath::Lerp(
		LowerPoint.WrapAngleFromLatchRad, UpperPoint.WrapAngleFromLatchRad, Alpha);
}

bool FRopeWrappingPhase::BuildResolvedWrappingPath(
	TArray<FRopeWrapPathPoint>& OutResolvedPath) const
{
	OutResolvedPath.Reset(FMath::Max(State.Path.Num(), State.Anchors.Num()));
	if (State.Path.Num() == 0)
	{
		for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
		{
			FRopeWrapPathPoint ResolvedPoint;
			if (!ResolveWrappingAnchorPoint(Anchor, ResolvedPoint))
			{
				OutResolvedPath.Reset();
				return false;
			}
			OutResolvedPath.Add(MoveTemp(ResolvedPoint));
		}
		OutResolvedPath.Sort([](const FRopeWrapPathPoint& A, const FRopeWrapPathPoint& B)
		{
			return A.DistanceFromLatch < B.DistanceFromLatch;
		});
		return true;
	}

	OutResolvedPath.SetNum(State.Path.Num());
	int32 AnchorIndex = 0;
	for (int32 PathIndex = 0; PathIndex < State.Path.Num(); ++PathIndex)
	{
		const FRopeWrapPathPoint& StoredPoint = State.Path[PathIndex];
		if (StoredPoint.bBridge || StoredPoint.bVirtual)
		{
			OutResolvedPath[PathIndex] = StoredPoint;
			continue;
		}

		const int32 ExpectedNodeIndex = State.LatchAnchor.NodeIndex + PathIndex;
		while (State.Anchors.IsValidIndex(AnchorIndex) &&
			State.Anchors[AnchorIndex].NodeIndex < ExpectedNodeIndex)
		{
			++AnchorIndex;
		}

		if (State.Anchors.IsValidIndex(AnchorIndex) &&
			State.Anchors[AnchorIndex].NodeIndex == ExpectedNodeIndex)
		{
			FRopeWrapPathPoint ResolvedPoint;
			if (!ResolveWrappingAnchorPoint(State.Anchors[AnchorIndex], ResolvedPoint))
			{
				OutResolvedPath.Reset();
				return false;
			}
			// On a moving bone the position and normal are refreshed from the anchor frame, while the
			// animation angle parameter settled when the path was built is restored from the stored point.
			ResolvedPoint.WrapAngleFromLatchRad = StoredPoint.WrapAngleFromLatchRad;
			OutResolvedPath[PathIndex] = MoveTemp(ResolvedPoint);
			++AnchorIndex;
		}
		else
		{
			// The snapshot is used for the short window within the same frame of a progressive build, before
			// the anchor has been attached.
			OutResolvedPath[PathIndex] = StoredPoint;
		}
	}
	return true;
}

bool FRopeWrappingPhase::SampleResolvedWrappingPath(
	const TArray<FRopeWrapPathPoint>& ResolvedPath, float DistanceFromLatch,
	float SurfaceOffset, FRopeWrapPathPoint& OutPoint)
{
	if (ResolvedPath.Num() == 0)
	{
		return false;
	}

	const float SampleDistance = FMath::Max(0.0f, DistanceFromLatch);
	int32 SearchMin = 0;
	int32 SearchMax = ResolvedPath.Num();
	while (SearchMin < SearchMax)
	{
		const int32 MidIndex = SearchMin + (SearchMax - SearchMin) / 2;
		if (ResolvedPath[MidIndex].DistanceFromLatch < SampleDistance)
		{
			SearchMin = MidIndex + 1;
		}
		else
		{
			SearchMax = MidIndex;
		}
	}

	const int32 UpperPathIndex = SearchMin;
	if (ResolvedPath.IsValidIndex(UpperPathIndex) &&
		ResolvedPath[UpperPathIndex].DistanceFromLatch == SampleDistance)
	{
		OutPoint = ResolvedPath[UpperPathIndex];
		OutPoint.DistanceFromLatch = SampleDistance;
		return true;
	}
	if (UpperPathIndex <= 0)
	{
		OutPoint = ResolvedPath[0];
		OutPoint.DistanceFromLatch = SampleDistance;
		return true;
	}
	if (UpperPathIndex >= ResolvedPath.Num())
	{
		OutPoint = ResolvedPath.Last();
		OutPoint.DistanceFromLatch = SampleDistance;
		return true;
	}

	InterpolateWrappingPathPoints(
		ResolvedPath[UpperPathIndex - 1], ResolvedPath[UpperPathIndex],
		SampleDistance, SurfaceOffset, OutPoint);
	return true;
}

#pragma endregion
