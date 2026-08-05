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

int32 FRopeWrappingPhase::ComputeCompositeRadiusEntryStepCount(
	float StartRadius, float TargetRadius, float StepDistance)
{
	const float RadiusDelta = FMath::Abs(TargetRadius - StartRadius);
	return RadiusDelta > KINDA_SMALL_NUMBER
		? FMath::Max(2, FMath::CeilToInt(
			RadiusDelta / (FMath::Max(StepDistance, KINDA_SMALL_NUMBER) * 0.6f)))
		: 0;
}

FRopeWrappingPhase::FCompositeHelixStepKinematics FRopeWrappingPhase::EvaluateCompositeHelixStep(
	int32 StepIndex, float StepDistance, int32 RadiusEntryStepCount,
	float StartRadius, float TargetRadius, float PreviousRadius,
	float PitchScale, float WindingSign)
{
	FCompositeHelixStepKinematics Result;
	const float RadiusAlpha = RadiusEntryStepCount > 0
		? FMath::Clamp(static_cast<float>(StepIndex) /
			static_cast<float>(RadiusEntryStepCount), 0.0f, 1.0f)
		: 1.0f;
	Result.Radius = FMath::Lerp(StartRadius, TargetRadius, RadiusAlpha);
	const float RadialStep = Result.Radius - PreviousRadius;
	Result.BaseTangentialStep = FMath::Sqrt(FMath::Max(
		0.0f, FMath::Square(StepDistance) - FMath::Square(RadialStep)));
	const float LengthScale = FMath::Sqrt(1.0f + FMath::Square(PitchScale));
	Result.CircumferenceStep = Result.BaseTangentialStep /
		FMath::Max(LengthScale, KINDA_SMALL_NUMBER);
	Result.AxisStep = Result.CircumferenceStep * PitchScale;
	const float MeanRadius = FMath::Max(
		(PreviousRadius + Result.Radius) * 0.5f, KINDA_SMALL_NUMBER);
	Result.AngleStepRad = WindingSign * Result.CircumferenceStep / MeanRadius;
	return Result;
}

#pragma region Composite Analytic Helix Path

bool FRopeWrappingPhase::AdvanceCompositeAnalyticHelixProbeStep(
	const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_AdvanceCompositeAnalyticHelixProbeStep);

	if (State.Path.Num() <= 0 || State.Path.Num() >= State.NumTailNodes ||
		!State.bPathUsesPoseSpaceIsland ||
		State.PathWrapIslandBones.Num() <= 1)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("CompositeAnalyticHelixInvalidState"));
		return false;
	}

	const USceneComponent* IslandMesh = ResolveWrappingMesh(State, State.LatchAnchor);
	if (!IslandMesh)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("CompositeAnalyticHelixMissingMesh"));
		return false;
	}

	// The ideal helix is advanced in small probe steps, independently of the number of output path points.
	// The sweep distance keeps advancing even when the projection barely moves and no output node is
	// produced.
	const FVector AxisDirection = State.PathAxisDirection.GetSafeNormal(
		KINDA_SMALL_NUMBER, FVector::UpVector);
	const FVector LatchSurfaceWorld = State.Path[0].SurfaceWorld;
	const float LatchAxisDistance = FVector::DotProduct(
		LatchSurfaceWorld - State.PathAxisOrigin, AxisDirection);
	const FVector LatchAxisPoint =
		State.PathAxisOrigin + AxisDirection * LatchAxisDistance;
	FVector LatchRadialOffset = LatchSurfaceWorld - LatchAxisPoint;
	const float LatchRadius = LatchRadialOffset.Size();
	if (LatchRadius <= KINDA_SMALL_NUMBER)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("CompositeAnalyticHelixDegenerateRadius"));
		return false;
	}
	const FVector LatchRadial = LatchRadialOffset / LatchRadius;
	const float HelixRadius = State.PathCompositeHelixRadius > KINDA_SMALL_NUMBER
		? State.PathCompositeHelixRadius
		: LatchRadius;

	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	// The raw projection polyline is built with half-segment probes, finer than the output node spacing.
	// The probe number is recovered from the sweep distance, which is independent of the path point count,
	// so a probe that produces no node because the surface point stayed put can still advance to the next
	// ideal helix phase.
	const float ProbeStepDistance = FMath::Max(0.5f, SegmentLength * 0.5f);
	const int32 ProbeStepIndex = FMath::RoundToInt(
		State.PathSweepDistance / ProbeStepDistance) + 1;
	const int32 MaxProbeStepCount = FMath::Max(32, State.NumTailNodes * 16);
	// This prevents an unbounded progressive build where the projection keeps returning the same point, as
	// on a closed or degenerate surface, and the path point count never grows. A healthy path normally
	// finishes within a few probes per node.
	if (ProbeStepIndex > MaxProbeStepCount)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("CompositeAnalyticHelixProbeExhausted"));
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Composite analytic helix probe exhausted: probeStep=%d max=%d "
				"path=%d/%d sweep=%.2fcm actualArc=%.2fcm"),
			*Ctx.OwnerName, ProbeStepIndex, MaxProbeStepCount,
			State.Path.Num(), State.NumTailNodes,
			State.PathSweepDistance, State.PathCurrentDistance);
		return false;
	}
	const float PitchScale = State.PathCompositeHelixPitchScale;
	const float PreviousRawAngleRad = State.PathCompositeSweepAngleRad;

	// It never jumps from the latch surface radius to the whole island radius in a single probe. Each raw
	// point is computed independently from the latch, the axis and the contact pitch alone, with no
	// dependence on the previous projection result.
	const int32 RadiusEntrySegmentCount = ComputeCompositeRadiusEntryStepCount(
		LatchRadius, HelixRadius, ProbeStepDistance);
	float IdealRadius = LatchRadius;
	float AxisAdvance = 0.0f;
	float AngleRadians = 0.0f;
	FVector IdealHelixWorld = LatchSurfaceWorld;
	FVector PreviousIdealHelixWorld = LatchSurfaceWorld;
	for (int32 IdealStepIndex = 1; IdealStepIndex <= ProbeStepIndex; ++IdealStepIndex)
	{
		PreviousIdealHelixWorld = IdealHelixWorld;
		const FCompositeHelixStepKinematics Step = EvaluateCompositeHelixStep(
			IdealStepIndex, ProbeStepDistance, RadiusEntrySegmentCount,
			LatchRadius, HelixRadius, IdealRadius, PitchScale, State.PathWindingSign);
		IdealRadius = Step.Radius;
		AxisAdvance += Step.AxisStep;
		AngleRadians += Step.AngleStepRad;

		const FVector StepRadial = FQuat(AxisDirection, AngleRadians)
			.RotateVector(LatchRadial)
			.GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);
		const FVector StepAxisPoint = State.PathAxisOrigin + AxisDirection *
			(LatchAxisDistance + AxisAdvance);
		IdealHelixWorld = StepAxisPoint + StepRadial * IdealRadius;
	}
	const float CurrentRawAngleRad = FMath::Abs(AngleRadians);
	const auto LogAngularDensityMetrics = [&](const TCHAR* CompletionReason)
	{
		const float BuiltDistance = State.Path.Num() > 0
			? State.Path.Last().DistanceFromLatch
			: 0.0f;
		const float BuiltAngleRad = State.Path.Num() > 0
			? State.Path.Last().WrapAngleFromLatchRad
			: 0.0f;
		const float BuiltAngleDeg = FMath::RadiansToDegrees(BuiltAngleRad);
		const float ActualCmPerRad = BuiltAngleRad > KINDA_SMALL_NUMBER
			? BuiltDistance / BuiltAngleRad
			: 0.0f;
		// The authored pitch of the single-bone surface vector field serves as the reference density. This
		// value diagnoses the arc variation of the composite projection itself and is not used for the speed
		// of the angle-mapped front.
		const float ReferencePitch = Ctx.Config.WrappingHelixPitchScale;
		const float ReferenceCmPerRad = HelixRadius *
			FMath::Sqrt(1.0f + FMath::Square(ReferencePitch));
		const float AngularDensityScale =
			ActualCmPerRad > KINDA_SMALL_NUMBER && ReferenceCmPerRad > KINDA_SMALL_NUMBER
				? ActualCmPerRad / ReferenceCmPerRad
				: 0.0f;
		const float RawArcToSweepRatio = State.PathSweepDistance > KINDA_SMALL_NUMBER
			? State.PathCurrentDistance / State.PathSweepDistance
			: 0.0f;
		const float NodesPerTurn = BuiltAngleRad > KINDA_SMALL_NUMBER
			? static_cast<float>(FMath::Max(0, State.Path.Num() - 1)) *
				(2.0f * PI) / BuiltAngleRad
			: 0.0f;

		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Composite angular density metrics: reason=%s "
				"builtDistance=%.2fcm builtAngle=%.2fdeg actualCmPerRad=%.3f "
				"referenceCmPerRad=%.3f angularDensityScale=%.3f "
				"rawArcToSweep=%.3f nodesPerTurn=%.2f "
				"compositePitch=%.3f referencePitch=%.3f points=%d"),
			*Ctx.OwnerName, CompletionReason,
			BuiltDistance, BuiltAngleDeg, ActualCmPerRad,
			ReferenceCmPerRad, AngularDensityScale,
			RawArcToSweepRatio, NodesPerTurn,
			PitchScale, ReferencePitch, State.Path.Num());
	};
	const FVector RotatedRadial = FQuat(AxisDirection, AngleRadians)
		.RotateVector(LatchRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);
	const FVector IdealTangentWorld = (IdealHelixWorld - PreviousIdealHelixWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER,
			FVector::CrossProduct(AxisDirection, RotatedRadial) * State.PathWindingSign);
	const float IdealAxisDistance = LatchAxisDistance + AxisAdvance;
	if (State.bPathCompositeAxisRangeValid &&
		(IdealAxisDistance < State.PathCompositeAxisMinDistance ||
			IdealAxisDistance > State.PathCompositeAxisMaxDistance))
	{
		const int32 PreviousTailNodeCount = State.NumTailNodes;
		LogAngularDensityMetrics(TEXT("AxisLimit"));
		State.NumTailNodes = State.Path.Num();
		FinishPathBuild(/*bFailed=*/false);
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Composite analytic helix reached island axial limit: "
				"nextProbe=%d idealAxis=%.2fcm range=[%.2f,%.2f]cm "
				"builtPoints=%d releasedSolverNodes=%d"),
			*Ctx.OwnerName, ProbeStepIndex, IdealAxisDistance,
			State.PathCompositeAxisMinDistance, State.PathCompositeAxisMaxDistance,
			State.Path.Num(), FMath::Max(0, PreviousTailNodeCount - State.NumTailNodes));
		return false;
	}

	// A radial ray is cast from the ideal helix point towards the axis point at the same height along the
	// axis. Of each SDF's first intersection, only the one nearest the ray's origin is used. The outer
	// band, the candidate scoring and the previous path direction are not consulted at all, so each raw
	// probe's result depends only on the independent analytic helix phase. Where no SDF intersects the ray,
	// the point is left as a no-anchor solver node.
	bool bFound = false;
	float BestRadialHitDistance = TNumericLimits<float>::Max();
	FRopeSurfaceProjection BestProjection;
	FName BestBone = NAME_None;
	const USceneComponent* BestMesh = IslandMesh;
	int32 MatchingSDFCount = 0;
	int32 RadialMissCount = 0;
	const FVector RadialRayStartWorld = IdealHelixWorld;
	const FVector RadialRayEndWorld =
		State.PathAxisOrigin + AxisDirection * IdealAxisDistance;
	for (const IRopeCollider* Collider : Ctx.Colliders)
	{
		if (!Collider || Collider->IsWorldStatic())
		{
			continue;
		}

		FName Bone = NAME_None;
		const USceneComponent* ColliderMesh = nullptr;
		Collider->GetGPUAttribution(Bone, ColliderMesh);
		if (!State.PathWrapIslandBones.Contains(Bone) || ColliderMesh != IslandMesh)
		{
			continue;
		}

		FRopeSDFColliderView SDFView;
		if (!Collider->GetGPUSDF(SDFView))
		{
			continue;
		}
		++MatchingSDFCount;

		// The sweep step is consumed in the SDF's local space. Sampling at half the smallest voxel keeps
		// even a thin surface from being stepped over, and the sample limit is derived so the whole ray is
		// always covered.
		const FVector VoxelSize(
			SDFView.LocalSize.X / FMath::Max(1, SDFView.ResX - 1),
			SDFView.LocalSize.Y / FMath::Max(1, SDFView.ResY - 1),
			SDFView.LocalSize.Z / FMath::Max(1, SDFView.ResZ - 1));
		const float MinVoxelSize = FMath::Max(0.1f,
			FMath::Min3(VoxelSize.X, VoxelSize.Y, VoxelSize.Z));
		const float SweepStep = FMath::Max(0.1f, MinVoxelSize * 0.5f);
		const FVector LocalRayStart =
			SDFView.BoneToWorld.InverseTransformPosition(RadialRayStartWorld);
		const FVector LocalRayEnd =
			SDFView.BoneToWorld.InverseTransformPosition(RadialRayEndWorld);
		const int32 MaxSamples = FMath::Clamp(
			FMath::CeilToInt(FVector::Dist(LocalRayStart, LocalRayEnd) / SweepStep) + 1,
			2, 512);

		FRopeSweptQuery RadialQuery;
		RadialQuery.WorldStart = RadialRayStartWorld;
		RadialQuery.WorldEnd = RadialRayEndWorld;
		// A threshold just above zero catches even a surface voxel whose sign was quantized as the first
		// intersection, while the position stored is the contact's surface point and is therefore not
		// inflated by the rope radius.
		RadialQuery.NodeRadius = 0.05f;
		RadialQuery.SweepStep = SweepStep;
		RadialQuery.MaxSamples = MaxSamples;

		FVector RadialHitWorld = RadialRayEndWorld;
		const FRopeContact RadialHit = Collider->QuerySwept(RadialQuery, RadialHitWorld);
		if (!RadialHit.bHit)
		{
			++RadialMissCount;
			continue;
		}

		const float RadialHitDistance = FVector::Dist(
			RadialRayStartWorld, RadialHitWorld);
		if (!bFound || RadialHitDistance < BestRadialHitDistance)
		{
			bFound = true;
			BestRadialHitDistance = RadialHitDistance;
			BestProjection.bHit = true;
			BestProjection.SurfacePoint = RadialHit.SurfacePoint;
			BestProjection.Normal = RadialHit.Normal;
			BestProjection.Distance = FVector::Dist(
				RadialRayStartWorld, RadialHit.SurfacePoint);
			BestProjection.Bone = RadialHit.Bone;
			BestProjection.SourceMesh = RadialHit.SourceMesh;
			BestBone = Bone;
			BestMesh = ColliderMesh;
		}
	}

	float ProjectedAxisDistance = IdealAxisDistance;
	if (bFound)
	{
		ProjectedAxisDistance = FVector::DotProduct(
			BestProjection.SurfacePoint - State.PathAxisOrigin, AxisDirection);
	}

	// The path state holds the previous raw projection rather than the last output node. That is what
	// allows the real raw polyline's length to be accumulated in full even when several probes fall between
	// output boundaries.
	const FVector PreviousRawSurfaceWorld = State.PathSurfaceWorld;
	const FVector PreviousRawNormalWorld = State.PathNormalWorld;
	const FVector PreviousRawTangentWorld = State.PathTangentWorld;
	const FVector PreviousRawGuideTangentWorld =
		State.PathCompositeRawGuideTangentWorld;
	const FName PreviousRawBone = State.PathCurrentBone;
	const USceneComponent* PreviousRawMesh = State.PathCurrentMesh.Get()
		? State.PathCurrentMesh.Get()
		: IslandMesh;
	const bool bPreviousRawVirtual = State.bPathCompositeRawPointVirtual;

	FRopeWrapPathPoint RawPoint;
	FVector CircumferenceDirection = FVector::CrossProduct(AxisDirection, RotatedRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir) * State.PathWindingSign;
	if (bFound)
	{
		const FVector NormalWorld = BestProjection.Normal.GetSafeNormal(
			KINDA_SMALL_NUMBER, RotatedRadial);
		const FVector ProjectedAxisPoint =
			State.PathAxisOrigin + AxisDirection * ProjectedAxisDistance;
		const FVector ProjectedRadial =
			(BestProjection.SurfacePoint - ProjectedAxisPoint)
			.GetSafeNormal(KINDA_SMALL_NUMBER, RotatedRadial);
		CircumferenceDirection = FVector::CrossProduct(AxisDirection, ProjectedRadial)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDirection) * State.PathWindingSign;
		FVector TangentWorld = (CircumferenceDirection + AxisDirection * PitchScale)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDirection);
		TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDirection);

		RawPoint.SurfaceWorld = BestProjection.SurfacePoint;
		RawPoint.NormalWorld = NormalWorld;
		RawPoint.TangentWorld = TangentWorld;
		RawPoint.Bone = BestProjection.Bone.IsNone() ? BestBone : BestProjection.Bone;
		RawPoint.Mesh = BestProjection.SourceMesh ? BestProjection.SourceMesh : BestMesh;
	}
	else
	{
		++State.PathCompositeProjectionFailureCount;
		RawPoint.SurfaceWorld = IdealHelixWorld;
		RawPoint.NormalWorld = RotatedRadial;
		RawPoint.TangentWorld = IdealTangentWorld;
		RawPoint.Bone = NAME_None;
		RawPoint.Mesh = IslandMesh;
		RawPoint.bVirtual = true;
	}

	// Accumulates the real rope centreline length of the raw projection polyline. A probe that snapped only
	// a short way produces no output node and moves on to the next, while a large projection movement is
	// split across several segment length boundaries by the loop.
	const float CenterlineOffset = FMath::Max(0.0f, Ctx.SurfaceOffset);
	const FVector PreviousRawCenterlineWorld = PreviousRawSurfaceWorld +
		(bPreviousRawVirtual ? FVector::ZeroVector : PreviousRawNormalWorld * CenterlineOffset);
	const FVector CurrentRawCenterlineWorld = RawPoint.SurfaceWorld +
		(RawPoint.bVirtual ? FVector::ZeroVector : RawPoint.NormalWorld * CenterlineOffset);
	const FVector RawCenterlineDelta =
		CurrentRawCenterlineWorld - PreviousRawCenterlineWorld;
	const float ActualStepDistance = RawCenterlineDelta.Size();
	const float ArcStartDistance = State.PathCurrentDistance;
	const float ArcEndDistance = ArcStartDistance + ActualStepDistance;
	const int32 FirstNewPathIndex = State.Path.Num();

	if (ActualStepDistance > KINDA_SMALL_NUMBER)
	{
		while (State.Path.Num() < State.NumTailNodes)
		{
			const int32 SamplePathIndex = State.Path.Num();
			const float TargetArcDistance =
				static_cast<float>(SamplePathIndex) * SegmentLength;
			if (TargetArcDistance > ArcEndDistance + KINDA_SMALL_NUMBER)
			{
				break;
			}

			const float Alpha = FMath::Clamp(
				(TargetArcDistance - ArcStartDistance) / ActualStepDistance,
				0.0f, 1.0f);
			const bool bSampleVirtual = bPreviousRawVirtual || RawPoint.bVirtual;
			const FVector SampleCenterlineWorld = FMath::Lerp(
				PreviousRawCenterlineWorld, CurrentRawCenterlineWorld, Alpha);
			const FVector SampleNormalWorld = FMath::Lerp(
				PreviousRawNormalWorld, RawPoint.NormalWorld, Alpha)
				.GetSafeNormal(KINDA_SMALL_NUMBER, RawPoint.NormalWorld);
			const FVector InterpolatedTangentWorld = FMath::Lerp(
				PreviousRawTangentWorld, RawPoint.TangentWorld, Alpha)
				.GetSafeNormal(KINDA_SMALL_NUMBER, RawPoint.TangentWorld);
			// Arc length corrects the spacing of the positions alone. Using the projection chord as the
			// tangent would make its direction swing sharply the moment the SDF support changes, and that
			// would jerk the straight extension direction of the whole tail behind the front. Interpolating
			// the tangents the independent analytic helix produced before and after preserves the original
			// wrapping flow.
			FVector SampleTangentWorld = (InterpolatedTangentWorld - FVector::DotProduct(
				InterpolatedTangentWorld, SampleNormalWorld) * SampleNormalWorld)
				.GetSafeNormal(KINDA_SMALL_NUMBER, RawPoint.TangentWorld);
			const FVector SampleWrappingGuideTangentWorld = FMath::Lerp(
				PreviousRawGuideTangentWorld, IdealTangentWorld, Alpha)
				.GetSafeNormal(KINDA_SMALL_NUMBER, IdealTangentWorld);

			// Where one raw span bridges two bones, the bone frame nearer the resample position is chosen as
			// the anchor's owner. If either side is virtual the output point is virtual too and no bone is
			// used.
			const bool bUseCurrentBinding =
				bPreviousRawVirtual || (!RawPoint.bVirtual && Alpha >= 0.5f);
			FRopeWrapPathPoint SamplePoint;
			SamplePoint.bBridge = false;
			SamplePoint.bVirtual = bSampleVirtual;
			SamplePoint.SurfaceWorld = EncodePathPointPositionFromCenterline(
				SampleCenterlineWorld, SampleNormalWorld, SamplePoint.bVirtual, CenterlineOffset);
			SamplePoint.NormalWorld = SampleNormalWorld;
			SamplePoint.TangentWorld = SampleTangentWorld;
			SamplePoint.WrappingGuideTangentWorld = SampleWrappingGuideTangentWorld;
			SamplePoint.bHasWrappingGuideTangent = true;
			SamplePoint.Bone = bSampleVirtual
				? NAME_None
				: (bUseCurrentBinding ? RawPoint.Bone : PreviousRawBone);
			SamplePoint.Mesh = bUseCurrentBinding ? RawPoint.Mesh.Get() : PreviousRawMesh;
			SamplePoint.DistanceFromLatch = TargetArcDistance;
			SamplePoint.WrapAngleFromLatchRad = FMath::Lerp(
				PreviousRawAngleRad, CurrentRawAngleRad, Alpha);
			State.Path.Add(SamplePoint);
		}
	}

	if (!RawPoint.bVirtual)
	{
		if (RawPoint.Bone != State.PathCurrentBone)
		{
			State.PathPreviousBone = State.PathCurrentBone;
		}
		State.PathCurrentBone = RawPoint.Bone;
		State.PathCurrentMesh = RawPoint.Mesh;
	}
	// The raw state and the sweep phase are always updated, whether or not an output node was produced. The
	// current distance is the real centreline arc and the sweep distance is the nominal advance of the
	// ideal helix probe.
	State.PathSurfaceWorld = RawPoint.SurfaceWorld;
	State.PathNormalWorld = RawPoint.NormalWorld;
	State.PathTangentWorld = RawPoint.TangentWorld;
	State.PathCompositeRawGuideTangentWorld = IdealTangentWorld;
	State.PathCircumferenceDir = CircumferenceDirection;
	State.PathCurrentDistance = ArcEndDistance;
	State.PathSweepDistance = static_cast<float>(ProbeStepIndex) * ProbeStepDistance;
	State.bPathCompositeRawPointVirtual = RawPoint.bVirtual;
	State.PathCompositeSweepRadial = RotatedRadial;
	State.PathCompositeSweepAngleRad = CurrentRawAngleRad;
	State.PathAccumulatedAngleRad = CurrentRawAngleRad;

	UE_LOG(LogRopeWrap, VeryVerbose,
		TEXT("[%s] Composite analytic helix raw probe: probe=%d type=%s bone=%s "
			"helixRadius=%.2fcm idealRadius=%.2fcm pitch=%.3f entrySegments=%d "
			"radialHitDistance=%.2fcm surfaceDistance=%.2fcm actualStep=%.2fcm "
			"arc=[%.2f,%.2f]cm appended=%d path=%d/%d "
			"angle=%.1fdeg idealAxis=%.2fcm surfaceAxis=%.2fcm "
			"ideal=%s selected=%s sdfCandidates=%d radialMiss=%d"),
		*Ctx.OwnerName, ProbeStepIndex,
		RawPoint.bVirtual ? TEXT("NoAnchorSolver") : TEXT("Surface"),
		*RawPoint.Bone.ToString(), HelixRadius, IdealRadius, PitchScale,
		RadiusEntrySegmentCount,
		bFound ? BestRadialHitDistance : -1.0f,
		bFound ? BestProjection.Distance : -1.0f,
		ActualStepDistance, ArcStartDistance, ArcEndDistance,
		State.Path.Num() - FirstNewPathIndex, State.Path.Num(), State.NumTailNodes,
		FMath::RadiansToDegrees(FMath::Abs(AngleRadians)), IdealAxisDistance,
		ProjectedAxisDistance, *IdealHelixWorld.ToString(),
		*(bFound ? BestProjection.SurfacePoint : RawPoint.SurfaceWorld).ToString(),
		MatchingSDFCount, RadialMissCount);

	if (State.Path.Num() >= State.NumTailNodes)
	{
		float MinCenterlineSpacing = TNumericLimits<float>::Max();
		float MaxCenterlineSpacing = 0.0f;
		float TotalCenterlineSpacing = 0.0f;
		int32 SpacingCount = 0;
		int32 VirtualPointCount = 0;
		for (int32 PathIndex = 0; PathIndex < State.Path.Num(); ++PathIndex)
		{
			const FRopeWrapPathPoint& CurrentPoint = State.Path[PathIndex];
			VirtualPointCount += CurrentPoint.bVirtual ? 1 : 0;
			if (PathIndex == 0)
			{
				continue;
			}

			const FRopeWrapPathPoint& PreviousPoint = State.Path[PathIndex - 1];
			const FVector PreviousCenterline = GetPathPointCenterlineWorld(
				PreviousPoint, CenterlineOffset);
			const FVector CurrentCenterline = GetPathPointCenterlineWorld(
				CurrentPoint, CenterlineOffset);
			const float Spacing = FVector::Dist(PreviousCenterline, CurrentCenterline);
			MinCenterlineSpacing = FMath::Min(MinCenterlineSpacing, Spacing);
			MaxCenterlineSpacing = FMath::Max(MaxCenterlineSpacing, Spacing);
			TotalCenterlineSpacing += Spacing;
			++SpacingCount;
		}
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Composite arc-length resample completed: points=%d probes=%d "
				"targetSpacing=%.2fcm chordSpacing(avg=%.2f min=%.2f max=%.2f)cm "
				"rawArc=%.2fcm sweep=%.2fcm virtualPoints=%d"),
			*Ctx.OwnerName, State.Path.Num(), ProbeStepIndex, SegmentLength,
			SpacingCount > 0 ? TotalCenterlineSpacing / static_cast<float>(SpacingCount) : 0.0f,
			SpacingCount > 0 ? MinCenterlineSpacing : 0.0f,
			MaxCenterlineSpacing, State.PathCurrentDistance, State.PathSweepDistance,
			VirtualPointCount);
		LogAngularDensityMetrics(TEXT("FullPath"));
		FinishPathBuild(/*bFailed=*/false);
	}
	return true;
}

void FRopeWrappingPhase::GatherPoseSpaceWrapIsland(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const USceneComponent* Mesh, TArray<FName>& OutBones,
	TArray<FRopeWrapIslandMember>& OutMembers,
	TArray<FRopeWrapIslandPortal>& OutPortals,
	float& OutAvailableSlack, const FContext& Ctx) const
{
	OutBones.Reset();
	OutMembers.Reset();
	OutPortals.Reset();
	OutAvailableSlack = 0.0f;
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return;
	}

	// The configuration-space radius of the rope centreline. Where the gap between two real surfaces is
	// smaller than that diameter, the rope centreline cannot pass between them and they are treated as part
	// of the same composite column.
	const float EffectiveRadius = FMath::Max3(
		Ctx.GetContactRadius(), Ctx.SurfaceOffset, Sim.SegmentLength * 0.25f);
	const float EffectiveDiameter = EffectiveRadius * 2.0f;
	// The island test looks at the whole real unpinned tail rather than the path length an older secondary
	// seed cut short. Once the island is established as spanning two or more bones, the caller removes that
	// seed and uses this length for the path.
	const int32 FullTailNodeCount = FMath::Max(0, Sim.Num() - LatchAnchor.NodeIndex);
	const float FreeRestLength = FMath::Max(
		0.0f, static_cast<float>(FMath::Max(0, FullTailNodeCount - 1)) * Sim.SegmentLength);
	const int32 TailNodeIndex = FMath::Clamp(
		LatchAnchor.NodeIndex + FMath::Max(0, FullTailNodeCount - 1), 0, FMath::Max(0, Sim.Num() - 1));
	const FVector TailWorld = Sim.Positions.IsValidIndex(TailNodeIndex)
		? Sim.Positions[TailNodeIndex]
		: State.PathSurfaceWorld;
	const float MinimumConnectionLength = FVector::Dist(State.PathSurfaceWorld, TailWorld);
	const float TensionReserve = FMath::Max(Sim.SegmentLength, EffectiveDiameter);
	OutAvailableSlack = FMath::Max(0.0f,
		FreeRestLength - MinimumConnectionLength - TensionReserve);

	struct FIslandCollider
	{
		const IRopeCollider* Collider = nullptr;
		FName Bone = NAME_None;
		FBox Bounds = FBox(EForceInit::ForceInit);
		FVector Center = FVector::ZeroVector;
		FVector Extent = FVector::ZeroVector;
		FRopeWrapIslandMember Member;
	};

	TArray<FIslandCollider, TInlineAllocator<16>> Candidates;
	const FVector AxisDirection = State.PathAxisDirection.GetSafeNormal(
		KINDA_SMALL_NUMBER, FVector::UpVector);
	const FVector AbsAxis(
		FMath::Abs(AxisDirection.X),
		FMath::Abs(AxisDirection.Y),
		FMath::Abs(AxisDirection.Z));
	const float CaptureSlabHalfWidth = FMath::Max(
		Sim.SegmentLength * 2.0f, EffectiveRadius * 4.0f);
	const float ReachDistance = FreeRestLength + EffectiveDiameter;

	for (const IRopeCollider* Collider : Ctx.Colliders)
	{
		if (!Collider || Collider->IsWorldStatic())
		{
			continue;
		}

		FName Bone = NAME_None;
		const USceneComponent* ColliderMesh = nullptr;
		Collider->GetGPUAttribution(Bone, ColliderMesh);
		if (Bone.IsNone() || ColliderMesh != Mesh)
		{
			continue;
		}

		const FBox Bounds = Collider->GetWorldBounds();
		if (!Bounds.IsValid)
		{
			continue;
		}

		const FVector Center = Bounds.GetCenter();
		const FVector Extent = Bounds.GetExtent();
		const bool bLatchCollider = Bone == LatchAnchor.Bone;
		if (!bLatchCollider)
		{
	// This stops the skeleton connectivity walking all the way down to the pelvis and legs, which have
	// nothing to do with the original throw's cross-section. It also accounts for the axial extent of the
	// bounds, so a shape that genuinely crosses the slab, such as a long torso, is retained.
			const float AxialOffset = FMath::Abs(FVector::DotProduct(
				Center - State.PathSurfaceWorld, AxisDirection));
			const float AxialExtent = FVector::DotProduct(Extent, AbsAxis);
			if (AxialOffset > AxialExtent + CaptureSlabHalfWidth)
			{
				continue;
			}

			if (ReachDistance > 0.0f &&
				FMath::Sqrt(Bounds.ComputeSquaredDistanceToPoint(State.PathSurfaceWorld)) > ReachDistance)
			{
				continue;
			}
		}

		FIslandCollider& Candidate = Candidates.AddDefaulted_GetRef();
		Candidate.Collider = Collider;
		Candidate.Bone = Bone;
		Candidate.Bounds = Bounds;
		Candidate.Center = Center;
		Candidate.Extent = Extent;
		Candidate.Member.Bone = Bone;
		Candidate.Member.WorldBounds = Bounds;

	// The SDF grid's local bounds and the bone transform at that moment are snapshotted as they are. The
	// composite cross-section calculation then uses those oriented boxes and never resamples the SDF or
	// converts it to a mesh.
		FRopeSDFColliderView SDFView;
		if (Collider->GetGPUSDF(SDFView))
		{
			const FVector LocalCenter = SDFView.LocalMin + SDFView.LocalSize * 0.5f;
			const FVector Scale = SDFView.BoneToWorld.GetScale3D();
			const FVector AbsScale(FMath::Abs(Scale.X), FMath::Abs(Scale.Y), FMath::Abs(Scale.Z));
			Candidate.Member.bHasOrientedSDFBounds = true;
			Candidate.Member.SDFCenter = SDFView.BoneToWorld.TransformPosition(LocalCenter);
			Candidate.Member.SDFHalfExtent = SDFView.LocalSize * 0.5f * AbsScale;
			Candidate.Member.SDFRotation = SDFView.BoneToWorld.GetRotation().GetNormalized();
		}
	}

	TArray<TArray<int32>, TInlineAllocator<16>> Links;
	Links.SetNum(Candidates.Num());
	const float ReachabilitySearchGap = FMath::Max3(
		EffectiveDiameter + Sim.SegmentLength * 2.0f,
		EffectiveRadius * 6.0f,
		Sim.SegmentLength * 3.0f);

	const auto ComputeBoxGap = [](const FBox& A, const FBox& B)
	{
		const double DX = FMath::Max3(A.Min.X - B.Max.X, B.Min.X - A.Max.X, 0.0);
		const double DY = FMath::Max3(A.Min.Y - B.Max.Y, B.Min.Y - A.Max.Y, 0.0);
		const double DZ = FMath::Max3(A.Min.Z - B.Max.Z, B.Min.Z - A.Max.Z, 0.0);
		return static_cast<float>(FVector(DX, DY, DZ).Size());
	};
	TArray<FRopeWrapIslandPortal, TInlineAllocator<32>> EvaluatedPortals;

	for (int32 AIndex = 0; AIndex < Candidates.Num(); ++AIndex)
	{
		for (int32 BIndex = AIndex + 1; BIndex < Candidates.Num(); ++BIndex)
		{
			const FIslandCollider& A = Candidates[AIndex];
			const FIslandCollider& B = Candidates[BIndex];
			if (A.Bone == B.Bone)
			{
				Links[AIndex].Add(BIndex);
				Links[BIndex].Add(AIndex);
				continue;
			}

			const float BoundsGap = ComputeBoxGap(A.Bounds, B.Bounds);
			if (BoundsGap > ReachabilitySearchGap)
			{
				continue;
			}

			const FVector BoundsPointA = A.Bounds.GetClosestPointTo(B.Center);
			const FVector BoundsPointB = B.Bounds.GetClosestPointTo(A.Center);
			const FVector PortalProbe = (BoundsPointA + BoundsPointB) * 0.5f;
			const float PairQueryRadius = static_cast<float>(FMath::Max3(
				static_cast<double>(ReachabilitySearchGap * 2.0f),
				A.Extent.Size(), B.Extent.Size()));
			const FRopeSurfaceProjection ProjectionA =
				A.Collider->ProjectToSurface(PortalProbe, PairQueryRadius);
			const FRopeSurfaceProjection ProjectionB =
				B.Collider->ProjectToSurface(PortalProbe, PairQueryRadius);
			if (!ProjectionA.bHit || !ProjectionB.bHit)
			{
				continue;
			}

			const float SurfaceGap = FVector::Dist(
				ProjectionA.SurfacePoint, ProjectionB.SurfacePoint);
			const FVector PortalMidpoint =
				(ProjectionA.SurfacePoint + ProjectionB.SurfacePoint) * 0.5f;
			const float DirectLength = FVector::Dist(State.PathSurfaceWorld, TailWorld);
			const float ViaPortalLength =
				FVector::Dist(State.PathSurfaceWorld, PortalMidpoint) +
				FVector::Dist(PortalMidpoint, TailWorld);
			const float BendAllowance = FMath::Max(0.0f, SurfaceGap - EffectiveDiameter) * 2.0f;
			const float RequiredExtraLength =
				FMath::Max(0.0f, ViaPortalLength - DirectLength) + BendAllowance;

			const bool bClosedByGeometry = SurfaceGap <= EffectiveDiameter;
			const bool bClosedByReachability =
				!bClosedByGeometry &&
				SurfaceGap <= ReachabilitySearchGap &&
				RequiredExtraLength > OutAvailableSlack;
			const bool bConnected = bClosedByGeometry || bClosedByReachability;

			FRopeWrapIslandPortal& Portal = EvaluatedPortals.AddDefaulted_GetRef();
			Portal.BoneA = A.Bone;
			Portal.BoneB = B.Bone;
			Portal.SurfacePointA = ProjectionA.SurfacePoint;
			Portal.SurfacePointB = ProjectionB.SurfacePoint;
			Portal.State = bClosedByGeometry
				? ERopeWrapIslandPortalState::ClosedGeometry
				: (bClosedByReachability
					? ERopeWrapIslandPortalState::ClosedReachability
					: ERopeWrapIslandPortalState::Open);
			Portal.SurfaceGap = SurfaceGap;

			// Logging every candidate pair would accumulate a quadratic number of lines for a single contact
			// and bury the real path failure. The detailed pair diagnostics stay at the most verbose level
			// and the ordinary log carries only the summary below.
			UE_LOG(LogRopeWrap, VeryVerbose,
				TEXT("[%s] Wrap island portal: a=%s b=%s state=%s gap=%.2f diameter=%.2f "
					"requiredExtra=%.2f slack=%.2f boundsGap=%.2f"),
				*Ctx.OwnerName, *A.Bone.ToString(), *B.Bone.ToString(),
				bClosedByGeometry ? TEXT("ClosedGeometry") :
					(bClosedByReachability ? TEXT("ClosedReachability") : TEXT("Open")),
				SurfaceGap, EffectiveDiameter, RequiredExtraLength, OutAvailableSlack, BoundsGap);

			if (bConnected)
			{
				Links[AIndex].Add(BIndex);
				Links[BIndex].Add(AIndex);
			}
		}
	}

	TArray<int32, TInlineAllocator<16>> Queue;
	TArray<bool, TInlineAllocator<16>> bInIsland;
	bInIsland.Init(false, Candidates.Num());
	for (int32 CandidateIndex = 0; CandidateIndex < Candidates.Num(); ++CandidateIndex)
	{
		if (Candidates[CandidateIndex].Bone == LatchAnchor.Bone)
		{
			bInIsland[CandidateIndex] = true;
			Queue.Add(CandidateIndex);
		}
	}

	for (int32 QueueIndex = 0; QueueIndex < Queue.Num(); ++QueueIndex)
	{
		const int32 CandidateIndex = Queue[QueueIndex];
		if (!OutBones.Contains(Candidates[CandidateIndex].Bone))
		{
			OutBones.Add(Candidates[CandidateIndex].Bone);
			OutMembers.Add(Candidates[CandidateIndex].Member);
		}
		for (const int32 NeighborIndex : Links[CandidateIndex])
		{
			if (!bInIsland[NeighborIndex])
			{
				bInIsland[NeighborIndex] = true;
				Queue.Add(NeighborIndex);
			}
		}
	}

	if (OutBones.Num() == 0)
	{
		OutBones.Add(LatchAnchor.Bone);
	}

	// Only the portals touching the adopted island are kept. Open portals are kept too, since why something
	// did not merge has to be visible, so closed edges are not filtered out. All of them are copies of
	// values the test loop above already computed.
	for (const FRopeWrapIslandPortal& Portal : EvaluatedPortals)
	{
		if (OutBones.Contains(Portal.BoneA) || OutBones.Contains(Portal.BoneB))
		{
			OutPortals.Add(Portal);
		}
	}

	FString BoneList;
	for (const FName Bone : OutBones)
	{
		if (!BoneList.IsEmpty())
		{
			BoneList += TEXT(",");
		}
		BoneList += Bone.ToString();
	}
	int32 ClosedGeometryPortalCount = 0;
	int32 ClosedReachabilityPortalCount = 0;
	int32 OpenPortalCount = 0;
	for (const FRopeWrapIslandPortal& Portal : OutPortals)
	{
		switch (Portal.State)
		{
		case ERopeWrapIslandPortalState::ClosedGeometry:
			++ClosedGeometryPortalCount;
			break;
		case ERopeWrapIslandPortalState::ClosedReachability:
			++ClosedReachabilityPortalCount;
			break;
		default:
			++OpenPortalCount;
			break;
		}
	}
	UE_LOG(LogRopeWrap, Log,
		TEXT("[%s] Pose-space wrap island built: latch=%s bones=%d candidates=%d slack=%.2fcm "
			"freeRest=%.2fcm slabHalf=%.2fcm portals=%d(closedGeometry=%d closedReachability=%d open=%d) "
			"members=[%s]"),
		*Ctx.OwnerName, *LatchAnchor.Bone.ToString(), OutBones.Num(), Candidates.Num(),
		OutAvailableSlack, FreeRestLength, CaptureSlabHalfWidth, OutPortals.Num(),
		ClosedGeometryPortalCount, ClosedReachabilityPortalCount, OpenPortalCount, *BoneList);
}

#pragma endregion
