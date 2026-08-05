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

#pragma region Sequential Surface Vector Field Path

bool FRopeWrappingPhase::AdvanceSequentialSurfaceVectorFieldPath(int32 StepBudget, const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_AdvanceSequentialSurfaceVectorFieldPath);

	const USceneComponent* Mesh = ResolveWrappingMesh(State, State.LatchAnchor);
	if (!Mesh || State.LatchAnchor.Bone.IsNone())
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("AdvanceMissingMeshOrLatchBone"));
		return false;
	}

	const float StepSize = FMath::Max(1.0f, Sim.SegmentLength * 0.5f);
	// The same overall attempt limit as the composite probe. The sweep distance grows by the step size on
	// every integration step even when the projection stays put, so an unbounded stall is prevented with
	// no separate persistent counter.
	const int32 MaxIntegrationStepCount = FMath::Max(32, State.NumTailNodes * 16);
	const float MaxIntegrationSweepDistance =
		StepSize * static_cast<float>(MaxIntegrationStepCount);
	int32 StepsRemaining = FMath::Max(1, StepBudget);

	// The radial about the current axis, meaning the unit vector from the axis to the point. The angle
	// between the radials before and after a step is that step's wrapped angle increment; it returns
	// false when the point lies on the axis and is degenerate.
	const auto ComputeAxisRadial = [this](const FVector& Point, FVector& OutRadial) -> bool
	{
		const float AxisDistance = FVector::DotProduct(Point - State.PathAxisOrigin, State.PathAxisDirection);
		OutRadial = Point - (State.PathAxisOrigin + State.PathAxisDirection * AxisDistance);
		return OutRadial.Normalize(KINDA_SMALL_NUMBER);
	};

	while (StepsRemaining > 0 &&
		State.Path.Num() < State.NumTailNodes &&
		State.PathSweepDistance + KINDA_SMALL_NUMBER < MaxIntegrationSweepDistance)
	{
		const int32 PathIndex = State.Path.Num();
		bool bConsumedStep = false;

		// One predictor and projection pair is treated as exactly one centreline integration segment.
		// Every outer iteration consumes exactly one step, whether or not a node was produced.
		while (StepsRemaining > 0 && !bConsumedStep)
		{
			const float StepDistance = StepSize;

			const FVector PreviousSurfaceWorld = State.PathSurfaceWorld;
			const FVector PreviousNormalWorld = State.PathNormalWorld;
			const FVector PreviousTangentWorld = State.PathTangentWorld;
			const bool bPreviousPointWasBridge = State.PathBridgeDistance > 0.0f;
			FVector StepRadialBefore = FVector::ZeroVector;
			const bool bHasRadialBefore = ComputeAxisRadial(PreviousSurfaceWorld, StepRadialBefore);
			State.PathTangentWorld = ComputeSurfaceVectorFieldTangent(
				State.PathAxisOrigin,
				State.PathAxisDirection,
				State.PathLatchRadial,
				State.PathWindingSign,
				PreviousSurfaceWorld,
				State.PathNormalWorld,
				Ctx,
				State.PathCircumferenceDir);
			const FVector PathPredictorWorld =
				PreviousSurfaceWorld + State.PathTangentWorld * StepDistance;

			// The snap distance and bridging code below reads State.PathSurfaceWorld as this step's path
			// predictor.
			State.PathSurfaceWorld = PathPredictorWorld;
			const FName CurrentBone = State.PathCurrentBone.IsNone()
				? State.LatchAnchor.Bone
				: State.PathCurrentBone;
			const USceneComponent* ProjectedMesh = State.PathCurrentMesh.Get();
			if (!ProjectedMesh)
			{
				ProjectedMesh = Mesh;
			}

			// Both the single-bone fallback and sequential multi-bone projection work from the tangent
			// predictor. The actual rope node position is used as a tie-break, so between near-identical
			// candidates the surface nearest the rope's own body wins.
			const int32 RopeNodeIndex = State.LatchAnchor.NodeIndex + PathIndex;
			const FVector RopeNodeWorld = Sim.Positions.IsValidIndex(RopeNodeIndex)
				? Sim.Positions[RopeNodeIndex]
				: State.PathSurfaceWorld;
			// The projection result is taken into a local: bridging below may reject the snap, so the state
			// is left untouched until acceptance is settled.
			FVector ProjectedSurface = State.PathSurfaceWorld;
			FVector ProjectedNormal = State.PathNormalWorld;
			FVector ProjectedTangent = State.PathTangentWorld;
			FVector ProjectedCircumference = State.PathCircumferenceDir;
			FName ProjectedBone = CurrentBone;
			bool bOnSurface = false;
			if (State.bPathUsesSingleBoneFallback)
			{
				bOnSurface = ProjectWrapPointToLatchBone(Mesh, Sim, Ctx,
					ProjectedSurface, ProjectedNormal, ProjectedTangent,
					ProjectedCircumference, ProjectedBone, ProjectedMesh);
			}
			else
			{
				bOnSurface = ProjectWrapPointToSurfaceMultiBone(CurrentBone, Mesh, Sim, Ctx,
					State.PathPreviousBone, State.PathDistanceSinceBoneTransition, RopeNodeWorld,
					State.PathNormalWorld, State.PathTangentWorld,
					ProjectedSurface, ProjectedNormal, ProjectedTangent,
					ProjectedCircumference, ProjectedBone, ProjectedMesh);
			}
			const TCHAR* StepFailureReason = bOnSurface
				? TEXT("None")
				: TEXT("ProjectionMiss");

			// With gap bridging enabled, accepting a snap has to pass two gates. With bridging disabled
			// there are no gates at all, which preserves the previous, permissive behaviour that reduced
			// aborts and is still the default.
			//  First, a snap distance limit: a surface more than one segment away from the predicted point
			//     is treated as "there is no surface to wrap here" and the path continues as a chord.
			//     Without it, the permissive query radius of three segments would drag the path towards a
			//     distant surface even in mid-air between targets, curling it into the valley between them.
			//  Second, winding reversal, which is the point a taut string leaves the surface: if accepting
			//     the snap would reverse the central angle about the axis, the path is circling behind the
			//     first target. Wrapping around a pair, such as two legs, advances the central angle
			//     monotonically in the winding direction, and a taut rope leaves the surface here and
			//     continues as a chord.
			//     Without this gate the predicted point always sits near the previous surface point, so the
			//     first gate never fires, since on a circular cross-section the snap displacement is
			//     uniformly small everywhere and no local signal can identify the departure point, and the
			//     path would orbit the first target forever and never cross to the pair. Wrapping a single
			//     target is unaffected, because its central angle is monotonic by nature; the departure
			//     comes slightly after the true tangent point, and the slack left over is taken up by the
			//     solver pulling the free nodes after the commit.
			const float MaxBridgeDistance = Ctx.Config.WrappingMaxGapBridgeDistance;
			if (bOnSurface && MaxBridgeDistance > 0.0f)
			{
				const float SnapDistance = FVector::Dist(ProjectedSurface, State.PathSurfaceWorld);
				const float MaxSnapDistance = FMath::Max3(
					Sim.SegmentLength, Ctx.GetContactRadius() * 2.0f, Ctx.SurfaceOffset * 2.0f);
				if (SnapDistance > MaxSnapDistance)
				{
					bOnSurface = false;
					StepFailureReason = TEXT("SnapDistanceExceeded");
				}

				FVector RadialAfterSnap = FVector::ZeroVector;
				if (bOnSurface && bHasRadialBefore && ComputeAxisRadial(ProjectedSurface, RadialAfterSnap))
				{
					const float SignedWindingStep = static_cast<float>(FVector::DotProduct(
						State.PathAxisDirection,
						FVector::CrossProduct(StepRadialBefore, RadialAfterSnap))) * State.PathWindingSign;
					if (SignedWindingStep < 0.0f)
					{
						bOnSurface = false;
						StepFailureReason = TEXT("WindingRegression");
					}
				}
			}

			// Neither a surface nor any remaining bridge, whether exhausted or disabled, so this fails as
			// before. It breaks before integrating the angle, which keeps a step that was never walked out
			// of the angle reported at the moment of failure, in ShouldAbortFailedShortWrap.
			if (!bOnSurface &&
				(MaxBridgeDistance <= 0.0f ||
					State.PathBridgeDistance + StepDistance > MaxBridgeDistance))
			{
				FinishPathBuild(/*bFailed=*/true, StepFailureReason);
				if (State.bPathUsesSingleBoneFallback)
				{
					UE_LOG(LogRopeWrap, Warning,
						TEXT("[%s] Single-bone fallback failed: reason=%s bone=%s path=%d/%d "
							"anchors=%d predictor=%s bridge=%.1f/%.1fcm"),
						*Ctx.OwnerName, StepFailureReason, *State.LatchAnchor.Bone.ToString(),
						State.Path.Num(), State.NumTailNodes, State.Anchors.Num(),
						*PathPredictorWorld.ToString(), State.PathBridgeDistance, MaxBridgeDistance);
				}
				if (!Ctx.bSuppressPathFailureLog)
				{
					UE_LOG(LogDynamicRope, Log,
						TEXT("[%s] Progressive wrap path stopped by projection failure (bone=%s, path=%d/%d, anchors=%d, bridge=%.0f/%.0fcm)"),
						*Ctx.OwnerName,
						*State.LatchAnchor.Bone.ToString(),
						State.Path.Num(),
						State.NumTailNodes,
						State.Anchors.Num(),
						State.PathBridgeDistance,
						MaxBridgeDistance);
				}
				return false;
			}

			// Integrate the wrapped angle by accumulating the radial rotation this step produced. It has to
			// be measured before the axis is re-resolved, in the reseed inside the accept branch below, and
			// against the axis this step was actually walked on. Bridge steps are integrated too: the angular
			// span a chord crosses counts as wrapped, which matches the circumferential coverage measure.
			const float PreviousForwardAngleRad = State.PathForwardAngleRad;
			const FVector PostStepPosition = bOnSurface ? ProjectedSurface : State.PathSurfaceWorld;
			FVector StepRadialAfter = FVector::ZeroVector;
			if (bHasRadialBefore && ComputeAxisRadial(PostStepPosition, StepRadialAfter))
			{
				const float RadialDot = FMath::Clamp(
					static_cast<float>(FVector::DotProduct(StepRadialBefore, StepRadialAfter)),
					-1.0f, 1.0f);
				const float UnsignedStepAngleRad = FMath::Acos(RadialDot);
				State.PathAccumulatedAngleRad += UnsignedStepAngleRad;

				// The accumulation through acos adds both forward and backward motion as positive. The same
				// step is also measured with atan2 to separate advance along the winding direction from
				// backward wobble, which leaves the existing angle-based behaviour and quality tests
				// untouched.
				const FVector StepAxisDirection = State.PathAxisDirection.GetSafeNormal(
					KINDA_SMALL_NUMBER, FVector::UpVector);
				const float SignedStepAngleRad = FMath::Atan2(
					FVector::DotProduct(StepAxisDirection,
						FVector::CrossProduct(StepRadialBefore, StepRadialAfter)),
					RadialDot);
				const float WindingStepAngleRad = SignedStepAngleRad * State.PathWindingSign;
				State.PathSignedNetAngleRad += WindingStepAngleRad;
				if (WindingStepAngleRad >= 0.0f)
				{
					State.PathForwardAngleRad += WindingStepAngleRad;
				}
				else
				{
					State.PathReverseAngleRad += -WindingStepAngleRad;
				}

				const float BeforeAxisDistance = FVector::DotProduct(
					PreviousSurfaceWorld - State.PathAxisOrigin, StepAxisDirection);
				const float AfterAxisDistance = FVector::DotProduct(
					PostStepPosition - State.PathAxisOrigin, StepAxisDirection);
				const float BeforeRadius = FVector::Dist(
					PreviousSurfaceWorld,
					State.PathAxisOrigin + StepAxisDirection * BeforeAxisDistance);
				const float AfterRadius = FVector::Dist(
					PostStepPosition,
					State.PathAxisOrigin + StepAxisDirection * AfterAxisDistance);
				const float StepMeanRadius = (BeforeRadius + AfterRadius) * 0.5f;
				if (StepMeanRadius > KINDA_SMALL_NUMBER)
				{
					State.PathRadiusSum += StepMeanRadius;
					State.PathRadiusMin = State.PathRadiusSampleCount > 0
						? FMath::Min(State.PathRadiusMin, StepMeanRadius)
						: StepMeanRadius;
					State.PathRadiusMax = FMath::Max(State.PathRadiusMax, StepMeanRadius);
					++State.PathRadiusSampleCount;
				}
			}

			if (bOnSurface)
			{
				State.PathSurfaceWorld = ProjectedSurface;
				State.PathNormalWorld = ProjectedNormal;
				State.PathTangentWorld = ProjectedTangent;
				State.PathCircumferenceDir = ProjectedCircumference;
				State.PathBridgeDistance = 0.0f;

				// ProjectWrapPointToSurfaceMultiBone applies the hysteresis and returns the final bone, so
				// only the state is updated here. On a transition the previous bone is recorded so a
				// candidate that returns to it immediately can be penalized on the next step, and the
				// distance since the transition restarts from zero.
				if (ProjectedBone != CurrentBone)
				{
					++State.PathBoneTransitionCount;
					State.PathPreviousBone = CurrentBone;
					State.PathCurrentBone = ProjectedBone;
					State.PathDistanceSinceBoneTransition = 0.0f;
					// A sequential bone transition reseeds the rolling axis against the new bone's shape.
					ReseedWrappingAxisOnBoneTransition(ProjectedBone, ProjectedMesh, Ctx);
				}
				else
				{
					State.PathCurrentBone = ProjectedBone;
					State.PathDistanceSinceBoneTransition += StepDistance;
				}
				State.PathCurrentMesh = ProjectedMesh;
			}
			else
			{
				// A bridge step uses the predicted position, continuing straight along the tangent, and
				// passes through with no anchor frame.
				// The normal is kept as the axis radial so the next step's circumferential tangent comes out
				// cleanly. The bone and mesh are kept, so re-entry candidates are still searched from the
				// graph centred on the current bone.
				State.PathBridgeDistance += StepDistance;
				State.PathDistanceSinceBoneTransition += StepDistance;
				FVector BridgeRadial = FVector::ZeroVector;
				if (ComputeAxisRadial(State.PathSurfaceWorld, BridgeRadial))
				{
					State.PathNormalWorld = BridgeRadial;
				}
			}

			// The nominal distance the predictor or sweep attempted is kept separate from the centreline
			// length the projection actually produced. Recording the attempted step distance as the path
			// length, independently of the actual one, would bunch or stretch the rope node spacing
			// wherever a surface snap was shorter or longer.
			const float CenterlineOffset = FMath::Max(0.0f, Ctx.SurfaceOffset);
			const FVector PreviousCenterlineWorld =
				PreviousSurfaceWorld + PreviousNormalWorld * CenterlineOffset;
			const FVector CurrentCenterlineWorld =
				State.PathSurfaceWorld + State.PathNormalWorld * CenterlineOffset;
			const float ActualStepDistance = FVector::Dist(
				PreviousCenterlineWorld, CurrentCenterlineWorld);
			const float ArcStartDistance = State.PathCurrentDistance;
			const float ArcEndDistance = ArcStartDistance + ActualStepDistance;

			// When this actual integration segment crosses one or more segment length boundaries, the
			// centreline frame is resampled at each. A single projection jump can cross several node
			// boundaries, hence a while loop rather than an if. The distance from the latch is therefore a
			// real arc coordinate along the polyline.
			if (ActualStepDistance > KINDA_SMALL_NUMBER)
			{
				while (State.Path.Num() < State.NumTailNodes)
				{
					const int32 SamplePathIndex = State.Path.Num();
					const float TargetArcDistance =
						static_cast<float>(SamplePathIndex) * Sim.SegmentLength;
					if (TargetArcDistance > ArcEndDistance + KINDA_SMALL_NUMBER)
					{
						break;
					}

					const float Alpha = FMath::Clamp(
						(TargetArcDistance - ArcStartDistance) / ActualStepDistance,
						0.0f, 1.0f);
					const FVector SampleCenterlineWorld = FMath::Lerp(
						PreviousCenterlineWorld, CurrentCenterlineWorld, Alpha);
					const FVector SampleNormalWorld = FMath::Lerp(
						PreviousNormalWorld, State.PathNormalWorld, Alpha)
						.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathNormalWorld);
					FVector SampleTangentWorld = FMath::Lerp(
						PreviousTangentWorld, State.PathTangentWorld, Alpha)
						.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathTangentWorld);
					SampleTangentWorld = (SampleTangentWorld -
						FVector::DotProduct(SampleTangentWorld, SampleNormalWorld) * SampleNormalWorld)
						.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathTangentWorld);

					FRopeWrapPathPoint Point;
					Point.bVirtual = false;
					// If the step starts on a bridge, or is itself a bridge, the interpolated point is
					// treated as a mid-air chord too.
					Point.bBridge = bPreviousPointWasBridge || !bOnSurface;
					Point.SurfaceWorld = EncodePathPointPositionFromCenterline(
						SampleCenterlineWorld, SampleNormalWorld,
						Point.bVirtual, CenterlineOffset);
					Point.NormalWorld = SampleNormalWorld;
					Point.TangentWorld = SampleTangentWorld;
					Point.Bone = State.PathCurrentBone.IsNone()
						? State.LatchAnchor.Bone
						: State.PathCurrentBone;
					Point.Mesh = State.PathCurrentMesh.IsValid()
						? State.PathCurrentMesh.Get()
						: Mesh;
					Point.DistanceFromLatch = TargetArcDistance;
					// The physical position is resampled along the real centreline arc while the animation
					// phase interpolates the forward signed angle of the same integration segment. Removing
					// the reverse component keeps the point angle non-decreasing, which keeps the
					// angle-to-distance binary search stable.
					Point.WrapAngleFromLatchRad = FMath::Lerp(
						PreviousForwardAngleRad, State.PathForwardAngleRad, Alpha);
					State.Path.Add(Point);
					if (!ProcessPathPointForAnchoring(SamplePathIndex, Sim, Ctx))
					{
						// Appending a path point and its anchor is treated as one atomic operation. Leaving
						// the current point, or any later point of the same integration segment, unprocessed
						// would let a later resolve treat the world position stored at the time as though it
						// were a proper anchor, so they are all removed.
						State.Path.SetNum(FMath::Clamp(
							State.LastAnchoredPathPointCount, 0, State.Path.Num()));
						FinishPathBuild(/*bFailed=*/true, TEXT("SequentialAnchorBuildFailed"));
						return false;
					}
				}
			}

			State.PathCurrentDistance = ArcEndDistance;
			State.PathSweepDistance += StepDistance;

			--StepsRemaining;
			bConsumedStep = true;
		}

		State.PathTangentWorld = ComputeSurfaceVectorFieldTangent(
			State.PathAxisOrigin,
			State.PathAxisDirection,
			State.PathLatchRadial,
			State.PathWindingSign,
			State.PathSurfaceWorld,
			State.PathNormalWorld,
			Ctx,
			State.PathCircumferenceDir);

		// The wrap amount limit, active when the maximum wrap angle is above 0: once the accumulated
		// wrapped angle reaches the target the path finishes here as a success. That stops the helix
		// exceeding the intended number of turns and drawing in the entire remaining rope, leaving the rope
		// beyond the path hanging as a free span after the commit. NumTailNodes is reduced to the length of
		// the path that was built, so the front and commit target distance matches the real path: leaving
		// it at the whole rope would aim the front at a distance beyond the path, the arrival test would
		// never pass, and the wrap would only ever commit through the settle timeout.
		if (Ctx.Config.WrappingMaxWrapAngleDeg > 0.0f &&
			FMath::RadiansToDegrees(State.PathAccumulatedAngleRad) >= Ctx.Config.WrappingMaxWrapAngleDeg)
		{
			State.NumTailNodes = State.Path.Num();
			FinishPathBuild(/*bFailed=*/false);
		}

		if (!bConsumedStep)
		{
			--StepsRemaining;
		}
	}

	if (State.Path.Num() >= State.NumTailNodes)
	{
		FinishPathBuild(/*bFailed=*/false);
	}
	else if (State.bPathBuildActive &&
		State.PathSweepDistance + KINDA_SMALL_NUMBER >= MaxIntegrationSweepDistance)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("SequentialIntegrationStepLimitExceeded"));
	}
	if (State.bPathBuildComplete)
	{
		const float BuiltDistance = State.Path.Num() > 0
			? State.Path.Last().DistanceFromLatch
			: 0.0f;
		const float AbsoluteAngleRad = State.PathAccumulatedAngleRad;
		const float NetSignedAngleRad = State.PathSignedNetAngleRad;
		const float ForwardAngleRad = State.PathForwardAngleRad;
		const float ReverseAngleRad = State.PathReverseAngleRad;
		const float AverageRadius = State.PathRadiusSampleCount > 0
			? State.PathRadiusSum / static_cast<float>(State.PathRadiusSampleCount)
			: 0.0f;
		const float PitchScale = Ctx.Config.WrappingHelixPitchScale;
		const float ExpectedCmPerRad = AverageRadius *
			FMath::Sqrt(1.0f + FMath::Square(PitchScale));
		const float RawArcToSweepRatio = State.PathSweepDistance > KINDA_SMALL_NUMBER
			? State.PathCurrentDistance / State.PathSweepDistance
			: 0.0f;
		const int32 BuiltSegmentCount = FMath::Max(0, State.Path.Num() - 1);
		const auto CmPerRad = [BuiltDistance](float AngleRad)
		{
			return FMath::Abs(AngleRad) > KINDA_SMALL_NUMBER
				? BuiltDistance / FMath::Abs(AngleRad)
				: 0.0f;
		};
		const auto NodesPerTurn = [BuiltSegmentCount](float AngleRad)
		{
			return FMath::Abs(AngleRad) > KINDA_SMALL_NUMBER
				? static_cast<float>(BuiltSegmentCount) * (2.0f * PI) / FMath::Abs(AngleRad)
				: 0.0f;
		};
		const bool bEndedByAngleCap = Ctx.Config.WrappingMaxWrapAngleDeg > 0.0f &&
			FMath::RadiansToDegrees(AbsoluteAngleRad) >= Ctx.Config.WrappingMaxWrapAngleDeg;

		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Single angular density metrics: reason=%s "
				"builtDistance=%.2fcm anglesDeg(abs=%.2f net=%.2f forward=%.2f reverse=%.2f) "
				"cmPerRad(abs=%.3f net=%.3f forward=%.3f expected=%.3f) "
				"nodesPerTurn(abs=%.2f net=%.2f forward=%.2f) rawArcToSweep=%.3f "
				"radius(avg=%.2f min=%.2f max=%.2f samples=%d) pitch=%.3f "
				"boneTransitions=%d points=%d anchors=%d"),
			*Ctx.OwnerName, bEndedByAngleCap ? TEXT("AngleCap") : TEXT("FullPath"),
			BuiltDistance,
			FMath::RadiansToDegrees(AbsoluteAngleRad),
			FMath::RadiansToDegrees(NetSignedAngleRad),
			FMath::RadiansToDegrees(ForwardAngleRad),
			FMath::RadiansToDegrees(ReverseAngleRad),
			CmPerRad(AbsoluteAngleRad), CmPerRad(NetSignedAngleRad),
			CmPerRad(ForwardAngleRad), ExpectedCmPerRad,
			NodesPerTurn(AbsoluteAngleRad), NodesPerTurn(NetSignedAngleRad),
			NodesPerTurn(ForwardAngleRad), RawArcToSweepRatio,
			AverageRadius, State.PathRadiusMin, State.PathRadiusMax, State.PathRadiusSampleCount,
			PitchScale, State.PathBoneTransitionCount, State.Path.Num(), State.Anchors.Num());
	}
	if (State.bPathUsesSingleBoneFallback && State.bPathBuildComplete)
	{
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Single-bone fallback completed: bone=%s points=%d anchors=%d "
				"distance=%.2fcm angle=%.1fdeg"),
			*Ctx.OwnerName, *State.LatchAnchor.Bone.ToString(), State.Path.Num(),
			State.Anchors.Num(), State.PathCurrentDistance,
			FMath::RadiansToDegrees(State.PathAccumulatedAngleRad));
	}

	return !State.bPathBuildFailed;
}

FVector FRopeWrappingPhase::ComputeSurfaceVectorFieldTangent(const FVector& AxisOrigin, const FVector& AxisDirection,
	const FVector& LatchRadial, float WindingSign, const FVector& SurfaceWorld,
	const FVector& NormalWorld, const FContext& Ctx, FVector& InOutCircumferenceDir) const
{
	const float AxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
	const FVector AxisPoint = AxisOrigin + AxisDirection * AxisDistance;
	const FVector Radial = (SurfaceWorld - AxisPoint).GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);

	InOutCircumferenceDir = FVector::CrossProduct(AxisDirection, Radial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, InOutCircumferenceDir) * WindingSign;

	FVector TangentWorld = (InOutCircumferenceDir + AxisDirection * Ctx.Config.WrappingHelixPitchScale)
		.GetSafeNormal(KINDA_SMALL_NUMBER, InOutCircumferenceDir);
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, InOutCircumferenceDir);
	return TangentWorld;
}

void FRopeWrappingPhase::GatherSurfaceVectorFieldBoneCandidates(FName CurrentBone, const USceneComponent* Mesh,
	TArray<FSurfaceVectorFieldBoneCandidate>& OutCandidates, const FContext& Ctx) const
{
	OutCandidates.Reset();
	if (!Mesh || CurrentBone.IsNone())
	{
		return;
	}

	// Only skeleton parent and child candidates are collected, which preserves the existing sequential
	// multi-bone behaviour. The composite SDF failure fallback does not call this function at all.
	struct FBoneQueueEntry
	{
		FName Bone = NAME_None;
		int32 Depth = 0;
		float Cost = 0.0f;
	};

	const int32 MaxCandidateDepth = Ctx.Config.bEnableMultiBoneWrapping
		? FMath::Max(0, Ctx.Config.MaxBoneTransitionDepth)
		: 0;
	const float MaxCandidateCost = Ctx.Config.bEnableMultiBoneWrapping
		? FMath::Max(0.0f, Ctx.Config.MaxBoneTransitionCost)
		: 0.0f;
	const float EdgePenalty = FMath::Max(0.0f, Ctx.Config.AutoParentChildTransitionPenalty);

	TArray<FBoneQueueEntry, TInlineAllocator<16>> Queue;
	TMap<FName, float> BestCostByBone;
	TMap<FName, int32> BestDepthByBone;
	Queue.Add({ CurrentBone, 0, 0.0f });
	BestCostByBone.Add(CurrentBone, 0.0f);
	BestDepthByBone.Add(CurrentBone, 0);

	while (Queue.Num() > 0)
	{
		int32 BestQueueIndex = 0;
		for (int32 QueueIndex = 1; QueueIndex < Queue.Num(); ++QueueIndex)
		{
			if (Queue[QueueIndex].Cost < Queue[BestQueueIndex].Cost)
			{
				BestQueueIndex = QueueIndex;
			}
		}

		const FBoneQueueEntry Entry = Queue[BestQueueIndex];
		Queue.RemoveAtSwap(BestQueueIndex, 1, EAllowShrinking::No);
		const float* KnownBestCost = BestCostByBone.Find(Entry.Bone);
		if (KnownBestCost && Entry.Cost > *KnownBestCost + KINDA_SMALL_NUMBER)
		{
			continue;
		}

		FSurfaceVectorFieldBoneCandidate& Candidate = OutCandidates.AddDefaulted_GetRef();
		Candidate.Bone = Entry.Bone;
		Candidate.Depth = Entry.Depth;
		Candidate.GraphCost = Entry.Cost;
		Candidate.bCurrentBone = Entry.Bone == CurrentBone;

		if (Entry.Depth >= MaxCandidateDepth || Entry.Cost >= MaxCandidateCost)
		{
			continue;
		}

		const auto AddNeighbor = [&](FName Bone)
		{
			if (Bone.IsNone())
			{
				return;
			}
			const int32 NextDepth = Entry.Depth + 1;
			const float NextCost = Entry.Cost + EdgePenalty;
			if (NextDepth > MaxCandidateDepth || NextCost > MaxCandidateCost)
			{
				return;
			}

			const float* ExistingCost = BestCostByBone.Find(Bone);
			const int32* ExistingDepth = BestDepthByBone.Find(Bone);
			if (ExistingCost &&
				(*ExistingCost < NextCost - KINDA_SMALL_NUMBER ||
					(FMath::IsNearlyEqual(*ExistingCost, NextCost) &&
						ExistingDepth && *ExistingDepth <= NextDepth)))
			{
				return;
			}
			BestCostByBone.Add(Bone, NextCost);
			BestDepthByBone.Add(Bone, NextDepth);
			Queue.Add({ Bone, NextDepth, NextCost });
		};

		AddNeighbor(RopeWrapTargets::GetParentTargetKey(Mesh, Entry.Bone));
		TArray<FName> Children;
		RopeWrapTargets::AppendChildTargetKeys(Mesh, Entry.Bone, Children);
		for (const FName& Child : Children)
		{
			AddNeighbor(Child);
		}
	}

	UE_LOG(LogRopeWrap, VeryVerbose,
		TEXT("[%s] Multi-bone candidate graph: source=Skeleton current=%s candidates=%d"),
		*Ctx.OwnerName, *CurrentBone.ToString(), OutCandidates.Num());
}

bool FRopeWrappingPhase::ProjectWrapPointToSurfaceMultiBone(FName CurrentBone, const USceneComponent* Mesh,
	const FRopeSimState& Sim, const FContext& Ctx,
	FName PreviousBone, float DistanceSinceLastTransition, const FVector& RopeNodeWorld,
	const FVector& PreviousNormalWorld, const FVector& PreviousTangentWorld,
	FVector& InOutSurfaceWorld, FVector& InOutNormalWorld, FVector& InOutTangentWorld,
	FVector& InOutCircumferenceDir, FName& InOutBone, const USceneComponent*& OutMesh) const
{
	TArray<FSurfaceVectorFieldBoneCandidate> Candidates;
	GatherSurfaceVectorFieldBoneCandidates(CurrentBone, Mesh, Candidates, Ctx);
	if (Candidates.Num() == 0)
	{
		return false;
	}

	struct FScoredProjection
	{
		FVector SurfaceWorld = FVector::ZeroVector;
		FVector NormalWorld = FVector::UpVector;
		FVector TangentWorld = FVector::ForwardVector;
		FVector CircumferenceDir = FVector::ForwardVector;
		FName Bone = NAME_None;
		const USceneComponent* Mesh = nullptr;
		float Distance = 0.0f;
		float RopeNodeDistance = 0.0f;
		float GraphCost = 0.0f;
		float Score = TNumericLimits<float>::Max();
	};

	FScoredProjection BestProjection;
	FScoredProjection CurrentBoneProjection;
	bool bFound = false;
	bool bFoundCurrentBone = false;

	// This is the projection selector used by the surface vector field alone.
	// The older single-bone approach admitted only the latch anchor's bone, whereas this evaluates the
	// most plausible surface point from the predicted position for each candidate bone and preserves the
	// winner as the path point's bone and mesh.
	//
	// The scoring terms are:
	// - The projection distance, meaning how far it jumped from the predicted position to the surface;
	//   lower is better.
	// - The rope node distance, which prefers a candidate whose surface point is near the actual rope node.
	// - The graph cost, which penalizes a candidate reached across more parent and child edges.
	// - The tangent and normal penalties, which discourage a candidate that turns sharply away from the
	//   previous step's surface field.
	// - The current bone bonus and its hysteresis, which require a new bone to be clearly better before
	//   switching while the current one is still usable.
	// - The immediate return penalty, which discourages the A to B to A oscillation of going straight back
	//   to the bone just left.
	const FVector PreviousTangent = PreviousTangentWorld.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	const FVector PreviousNormal = PreviousNormalWorld.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	const float QueryRadius = FMath::Max3(
		FMath::Max(Ctx.GetContactRadius(), Ctx.SurfaceOffset),
		Sim.SegmentLength,
		Sim.SegmentLength * 3.0f);

	for (const IRopeCollider* Collider : Ctx.Colliders)
	{
		if (!Collider)
		{
			continue;
		}

		const FRopeSurfaceProjection Projection = Collider->ProjectToSurface(InOutSurfaceWorld, QueryRadius);
		if (!Projection.bHit)
		{
			continue;
		}

		const FSurfaceVectorFieldBoneCandidate* Candidate = Candidates.FindByPredicate(
			[&Projection](const FSurfaceVectorFieldBoneCandidate& Item)
			{
				return Item.Bone == Projection.Bone;
			});
		if (!Candidate)
		{
			continue;
		}

		if (Mesh && Projection.SourceMesh && Projection.SourceMesh != Mesh)
		{
			continue;
		}

		FVector CandidateCircumferenceDir = InOutCircumferenceDir;
		const FVector CandidateNormal = Projection.Normal.GetSafeNormal(KINDA_SMALL_NUMBER, PreviousNormal);
		const FVector CandidateTangent = ComputeSurfaceVectorFieldTangent(
			State.PathAxisOrigin,
			State.PathAxisDirection,
			State.PathLatchRadial,
			State.PathWindingSign,
			Projection.SurfacePoint,
			CandidateNormal,
			Ctx,
			CandidateCircumferenceDir);

		const float TangentPenalty = 1.0f - FMath::Clamp(FVector::DotProduct(CandidateTangent, PreviousTangent), -1.0f, 1.0f);
		const float NormalPenalty = 1.0f - FMath::Clamp(FVector::DotProduct(CandidateNormal, PreviousNormal), -1.0f, 1.0f);
		const bool bCurrentBone = Projection.Bone == CurrentBone;

		// The lowest score wins.
		// The distance terms measure how far it moved from the predicted path and the actual node, and the
		// continuity terms measure how sharply the surface field turned.
		// The current bone bonus is a genuine bonus, so it is subtracted last, which keeps the current bone
		// near a tie.
		float Score =
			Projection.Distance * Ctx.Config.ProjectionDistanceWeight +
			FVector::Dist(Projection.SurfacePoint, RopeNodeWorld) * Ctx.Config.RopeNodeDistanceWeight +
			Candidate->GraphCost * Ctx.Config.BoneTransitionPenaltyWeight +
			TangentPenalty * Ctx.Config.TangentContinuityWeight +
			NormalPenalty * Ctx.Config.NormalContinuityWeight -
			(bCurrentBone ? Ctx.Config.CurrentBoneBonus : 0.0f);

		if (!PreviousBone.IsNone() && Projection.Bone == PreviousBone && Projection.Bone != CurrentBone)
		{
			// A candidate that returns straight to the bone just left is likely to produce an A to B to A
			// oscillation even when its surface projection looks slightly better. It is not forbidden
			// outright, only penalized.
			Score += Ctx.Config.ImmediateBoneReturnPenalty;
		}

		if (!bFound || Score < BestProjection.Score)
		{
			BestProjection.SurfaceWorld = Projection.SurfacePoint;
			BestProjection.NormalWorld = CandidateNormal;
			BestProjection.TangentWorld = CandidateTangent;
			BestProjection.CircumferenceDir = CandidateCircumferenceDir;
			BestProjection.Bone = Projection.Bone;
			BestProjection.Mesh = Projection.SourceMesh ? Projection.SourceMesh : Mesh;
			BestProjection.Distance = Projection.Distance;
			BestProjection.RopeNodeDistance = FVector::Dist(Projection.SurfacePoint, RopeNodeWorld);
			BestProjection.GraphCost = Candidate->GraphCost;
			BestProjection.Score = Score;
			bFound = true;
		}

		if (bCurrentBone && (!bFoundCurrentBone || Score < CurrentBoneProjection.Score))
		{
			CurrentBoneProjection.SurfaceWorld = Projection.SurfacePoint;
			CurrentBoneProjection.NormalWorld = CandidateNormal;
			CurrentBoneProjection.TangentWorld = CandidateTangent;
			CurrentBoneProjection.CircumferenceDir = CandidateCircumferenceDir;
			CurrentBoneProjection.Bone = Projection.Bone;
			CurrentBoneProjection.Mesh = Projection.SourceMesh ? Projection.SourceMesh : Mesh;
			CurrentBoneProjection.Distance = Projection.Distance;
			CurrentBoneProjection.RopeNodeDistance = FVector::Dist(Projection.SurfacePoint, RopeNodeWorld);
			CurrentBoneProjection.GraphCost = Candidate->GraphCost;
			CurrentBoneProjection.Score = Score;
			bFoundCurrentBone = true;
		}
	}

	if (!bFound)
	{
		FString CandidateBones;
		for (const FSurfaceVectorFieldBoneCandidate& Candidate : Candidates)
		{
			if (!CandidateBones.IsEmpty())
			{
				CandidateBones += TEXT(",");
			}
			CandidateBones += Candidate.Bone.ToString();
		}
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Multi-bone surface projection empty: current=%s previous=%s candidates=%d [%s] predictor=%s ropeNode=%s normal=%s tangent=%s distanceSinceTransition=%.2fcm"),
			*Ctx.OwnerName, *CurrentBone.ToString(), *PreviousBone.ToString(), Candidates.Num(),
			*CandidateBones, *InOutSurfaceWorld.ToString(), *RopeNodeWorld.ToString(),
			*PreviousNormalWorld.ToString(), *PreviousTangentWorld.ToString(), DistanceSinceLastTransition);
		return false;
	}

	if (bFoundCurrentBone && BestProjection.Bone != CurrentBone)
	{
		// While the current bone's projection still succeeds, switching is conservative.
		// The best candidate is kept only when it beats the current one by the hysteresis margin and the
		// minimum distance since the last transition has also elapsed.
		// If either condition is unmet it reverts to the current bone's projection, which stops path points
		// and anchors being stored while flickering between several bones over a short stretch.
		const bool bEnoughScoreMargin =
			BestProjection.Score + Ctx.Config.BoneTransitionHysteresis < CurrentBoneProjection.Score;
		const bool bEnoughDistanceSinceTransition =
			DistanceSinceLastTransition >= Ctx.Config.MinBoneTransitionPathDistance;

		if (!bEnoughScoreMargin || !bEnoughDistanceSinceTransition)
		{
			UE_LOG(LogRopeWrap, VeryVerbose,
				TEXT("[%s] Bone transition held: from=%s candidate=%s source=Skeleton "
					"score=%.3f currentScore=%.3f scoreMargin=%d pathDistance=%d"),
				*Ctx.OwnerName, *CurrentBone.ToString(), *BestProjection.Bone.ToString(),
				BestProjection.Score, CurrentBoneProjection.Score,
				bEnoughScoreMargin ? 1 : 0, bEnoughDistanceSinceTransition ? 1 : 0);
			BestProjection = CurrentBoneProjection;
		}
	}

	if (BestProjection.Bone != CurrentBone)
	{
		UE_LOG(LogRopeWrap, VeryVerbose,
			TEXT("[%s] Bone transition selected: from=%s to=%s source=Skeleton "
				"score=%.3f graphCost=%.3f"),
			*Ctx.OwnerName, *CurrentBone.ToString(), *BestProjection.Bone.ToString(),
			BestProjection.Score, BestProjection.GraphCost);
	}

	InOutSurfaceWorld = BestProjection.SurfaceWorld;
	InOutNormalWorld = BestProjection.NormalWorld;
	InOutTangentWorld = BestProjection.TangentWorld;
	InOutCircumferenceDir = BestProjection.CircumferenceDir;
	InOutBone = BestProjection.Bone;
	OutMesh = BestProjection.Mesh;
	return true;
}

bool FRopeWrappingPhase::ProjectWrapPointToLatchBone(const USceneComponent* Mesh,
	const FRopeSimState& Sim, const FContext& Ctx,
	FVector& InOutSurfaceWorld, FVector& InOutNormalWorld, FVector& InOutTangentWorld,
	FVector& InOutCircumferenceDir, FName& InOutBone, const USceneComponent*& OutMesh) const
{
	// The composite path's candidate and graph state is not reused. Only colliders attributed to the same
	// mesh as the original latch bone are searched, through the existing single-bone projection, and the
	// tangent is recomputed from that surface frame.
	const FName LatchBone = State.LatchAnchor.Bone;
	if (LatchBone.IsNone() ||
		!ProjectWrapPointToSurface(LatchBone, Mesh, Sim, Ctx, InOutSurfaceWorld, InOutNormalWorld))
	{
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Single-bone fallback projection empty: latchBone=%s mesh=%s predictor=%s normal=%s colliders=%d contactRadius=%.2fcm segment=%.2fcm"),
			*Ctx.OwnerName, *LatchBone.ToString(), Mesh ? *Mesh->GetName() : TEXT("None"),
			*InOutSurfaceWorld.ToString(), *InOutNormalWorld.ToString(), Ctx.Colliders.Num(),
			Ctx.GetContactRadius(), Sim.SegmentLength);
		return false;
	}

	InOutCircumferenceDir = InOutCircumferenceDir.GetSafeNormal(
		KINDA_SMALL_NUMBER, State.PathCircumferenceDir);
	InOutTangentWorld = ComputeSurfaceVectorFieldTangent(
		State.PathAxisOrigin,
		State.PathAxisDirection,
		State.PathLatchRadial,
		State.PathWindingSign,
		InOutSurfaceWorld,
		InOutNormalWorld,
		Ctx,
		InOutCircumferenceDir);
	InOutBone = LatchBone;
	OutMesh = Mesh ? Mesh : ResolveWrappingMesh(State, State.LatchAnchor);
	return true;
}

bool FRopeWrappingPhase::ProjectWrapPointToSurface(FName Bone, const USceneComponent* Mesh,
	const FRopeSimState& Sim, const FContext& Ctx,
	FVector& InOutSurfaceWorld, FVector& InOutNormalWorld) const
{
	FRopeSurfaceProjection BestProjection;
	bool bFound = false;

	const float QueryRadius = FMath::Max(FMath::Max(Ctx.GetContactRadius(), Ctx.SurfaceOffset), Sim.SegmentLength);
	for (const IRopeCollider* Collider : Ctx.Colliders)
	{
		if (!Collider)
		{
			continue;
		}

		const FRopeSurfaceProjection Projection = Collider->ProjectToSurface(InOutSurfaceWorld, QueryRadius);
		if (!Projection.bHit || Projection.Bone != Bone)
		{
			continue;
		}

		if (Mesh && Projection.SourceMesh && Projection.SourceMesh != Mesh)
		{
			continue;
		}

		if (!bFound || Projection.Distance < BestProjection.Distance)
		{
			BestProjection = Projection;
			bFound = true;
		}
	}

	if (!bFound)
	{
		return false;
	}

	InOutSurfaceWorld = BestProjection.SurfacePoint;
	InOutNormalWorld = BestProjection.Normal.GetSafeNormal(KINDA_SMALL_NUMBER, InOutNormalWorld);
	return true;
}

#pragma endregion
