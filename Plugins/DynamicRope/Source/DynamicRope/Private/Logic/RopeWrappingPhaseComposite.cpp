// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrappingPhase.h"
#include "Components/SceneComponent.h"
// ResolveBindingWorld — 랩 바인딩(본/소켓/컴포넌트) 트랜스폼 해석의 단일 지점(seam A).
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"
// RopeMath::AnyTangentFromNormal (unity 빌드 중복 정의 방지)
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

	// 출력 Path.Num()과 독립적인 작은 probe step으로 ideal helix를 계산한다. projection 결과가
	// 거의 움직이지 않아 출력 node가 생기지 않아도 PathSweepDistance는 계속 전진한다.
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
	// 출력 node 간격보다 촘촘한 반-segment probe로 원본 projection polyline을 만든다.
	// Path.Num()과 무관한 PathSweepDistance에서 probe 번호를 복원하므로, 표면점이 제자리여서
	// 이번 probe가 node를 만들지 못해도 다음 ideal helix 위상으로 전진할 수 있다.
	const float ProbeStepDistance = FMath::Max(0.5f, SegmentLength * 0.5f);
	const int32 ProbeStepIndex = FMath::RoundToInt(
		State.PathSweepDistance / ProbeStepDistance) + 1;
	const int32 MaxProbeStepCount = FMath::Max(32, State.NumTailNodes * 16);
	// projection이 계속 같은 점을 반환하는 폐곡면/축퇴 상황에서 Path.Num()이 영원히 늘지 않는
	// 무한 progressive build를 막는다. 정상 경로는 보통 node당 2~수 회 probe 안에 끝난다.
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

	// 래치 표면 반지름에서 island 전체 반지름으로 한 probe에 순간 이동하지 않는다. 각 raw point는
	// 직전 projection 결과와 무관하게 래치/축/contact pitch만으로 독립 계산한다.
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
		// Single Bone SurfaceVectorField의 저작 pitch를 기준 밀도로 사용한다. 이 값은 Composite
		// projection 자체의 arc 변화 진단용이며 angle-mapped front의 속도에는 사용하지 않는다.
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

	// Ideal helix point에서 같은 축 높이의 axis point를 향해 radial ray를 쏜다. 각 SDF의
	// 첫 교차점 중 ray 시작점에 가장 가까운 것만 사용한다. outer band/후보 점수/직전 path
	// 방향은 전혀 사용하지 않으므로 각 raw probe의 결과는 독립적인 analytic helix 위상에만
	// 의존한다. 어떤 SDF도 ray와 교차하지 않으면 이 점은 no-anchor solver node로 남긴다.
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

		// QuerySwept의 step은 SDF 로컬 공간에서 소비된다. 가장 작은 voxel의 절반으로
		// 샘플해 얇은 표면도 건너뛰지 않게 하고, ray 전체가 항상 샘플되도록 상한을 산출한다.
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
		// 0보다 아주 조금 큰 값으로 부호가 양자화된 표면 voxel도 첫 교차로 잡되,
		// 저장 위치는 Contact.SurfacePoint이므로 rope radius만큼 부풀리지 않는다.
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

	// State.Path*는 마지막 출력 node가 아니라 직전 raw projection을 보관한다. 그래야 출력 경계
	// 사이에 몇 번의 probe가 끼더라도 실제 raw polyline의 길이를 빠짐없이 누적할 수 있다.
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

	// Raw projection polyline의 실제 rope centerline 길이를 누적한다. 짧게 스냅된 probe는
	// 출력 node를 만들지 않고 다음 probe로 넘어가며, 큰 projection 이동은 while에서 여러
	// SegmentLength 경계로 나누어 출력한다.
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
			// Arc-length는 위치 간격만 보정한다. projection chord를 tangent로 쓰면 SDF support가
			// 바뀌는 순간 chord 방향이 급회전하고, front 뒤 tail 전체의 직선 연장 방향까지 튄다.
			// 독립 analytic helix가 만든 전/후 tangent를 보간해 원래 감김 흐름을 유지한다.
			FVector SampleTangentWorld = (InterpolatedTangentWorld - FVector::DotProduct(
				InterpolatedTangentWorld, SampleNormalWorld) * SampleNormalWorld)
				.GetSafeNormal(KINDA_SMALL_NUMBER, RawPoint.TangentWorld);
			const FVector SampleWrappingGuideTangentWorld = FMath::Lerp(
				PreviousRawGuideTangentWorld, IdealTangentWorld, Alpha)
				.GetSafeNormal(KINDA_SMALL_NUMBER, IdealTangentWorld);

			// 같은 raw 구간이 두 본 사이를 잇는 경우 재샘플 위치에 더 가까운 쪽의 bone frame을
			// anchor 소유자로 고른다. 어느 한쪽이 virtual이면 출력점도 virtual이라 bone은 쓰지 않는다.
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
	// 출력 node 생성 여부와 상관없이 raw 상태와 sweep 위상은 항상 갱신한다. PathCurrentDistance는
	// 실제 centerline arc, PathSweepDistance는 ideal helix probe의 명목 진행량이다.
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
	TArray<FRopeWrapIslandDebugMember>& OutDebugMembers,
	TArray<FRopeWrapIslandDebugPortal>& OutDebugPortals,
	float& OutAvailableSlack, const FContext& Ctx) const
{
	OutBones.Reset();
	OutDebugMembers.Reset();
	OutDebugPortals.Reset();
	OutAvailableSlack = 0.0f;
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return;
	}

	// 로프 중심선의 configuration-space 반지름. 두 실제 표면의 gap이 이 지름보다 작으면
	// 로프 중심선은 사이를 통과할 수 없으므로 같은 복합 기둥의 일부로 본다.
	const float EffectiveRadius = FMath::Max3(
		Ctx.GetContactRadius(), Ctx.SurfaceOffset, Sim.SegmentLength * 0.25f);
	const float EffectiveDiameter = EffectiveRadius * 2.0f;
	// island 판정은 구형 secondary seed가 잘라 놓은 경로 길이가 아니라 실제 미고정 tail 전체를 본다.
	// island가 둘 이상으로 확정되면 호출자가 해당 seed를 제거하고 이 길이를 경로에 사용한다.
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
		FRopeWrapIslandDebugMember DebugMember;
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
			// 최초 투척 단면과 무관한 골반/다리까지 skeleton 연결을 타고 내려가는 것을 막는다.
			// bounds의 축 방향 extent까지 고려하므로 긴 몸통처럼 slab을 가로지르는 형상은 유지된다.
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
		Candidate.DebugMember.Bone = Bone;
		Candidate.DebugMember.WorldBounds = Bounds;

		// SDF grid의 로컬 bounds와 당시 bone transform을 그대로 스냅샷한다. 이후 복합 단면 계산은
		// 이 oriented box를 사용해 SDF를 다시 샘플링하거나 메시로 변환하지 않는다.
		FRopeSDFColliderView SDFView;
		if (Collider->GetGPUSDF(SDFView))
		{
			const FVector LocalCenter = SDFView.LocalMin + SDFView.LocalSize * 0.5f;
			const FVector Scale = SDFView.BoneToWorld.GetScale3D();
			const FVector AbsScale(FMath::Abs(Scale.X), FMath::Abs(Scale.Y), FMath::Abs(Scale.Z));
			Candidate.DebugMember.bHasOrientedSDFBounds = true;
			Candidate.DebugMember.SDFCenter = SDFView.BoneToWorld.TransformPosition(LocalCenter);
			Candidate.DebugMember.SDFHalfExtent = SDFView.LocalSize * 0.5f * AbsScale;
			Candidate.DebugMember.SDFRotation = SDFView.BoneToWorld.GetRotation().GetNormalized();
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
	TArray<FRopeWrapIslandDebugPortal, TInlineAllocator<32>> EvaluatedPortals;

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

			FRopeWrapIslandDebugPortal& DebugPortal = EvaluatedPortals.AddDefaulted_GetRef();
			DebugPortal.BoneA = A.Bone;
			DebugPortal.BoneB = B.Bone;
			DebugPortal.SurfacePointA = ProjectionA.SurfacePoint;
			DebugPortal.SurfacePointB = ProjectionB.SurfacePoint;
			DebugPortal.State = bClosedByGeometry
				? ERopeWrapIslandPortalState::ClosedGeometry
				: (bClosedByReachability
					? ERopeWrapIslandPortalState::ClosedReachability
					: ERopeWrapIslandPortalState::Open);
			DebugPortal.SurfaceGap = SurfaceGap;
			DebugPortal.EffectiveDiameter = EffectiveDiameter;
			DebugPortal.RequiredExtraLength = RequiredExtraLength;
			DebugPortal.AvailableSlack = OutAvailableSlack;

			// 모든 후보 쌍을 Log로 출력하면 한 번의 접촉에 O(n^2) 줄이 쌓여 실제 경로 실패가
			// 묻힌다. 상세 pair 진단은 VeryVerbose에 남기고, 일반 로그에는 아래 집계만 출력한다.
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
			OutDebugMembers.Add(Candidates[CandidateIndex].DebugMember);
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

	// 채택된 island에 닿아 있던 portal만 남긴다. Open portal도 왜 합쳐지지 않았는지 볼 수 있어야 하므로
	// 폐쇄 edge만 필터링하지 않는다. 모두 위 판정 루프에서 이미 계산된 값의 복사본이다.
	for (const FRopeWrapIslandDebugPortal& Portal : EvaluatedPortals)
	{
		if (OutBones.Contains(Portal.BoneA) || OutBones.Contains(Portal.BoneB))
		{
			OutDebugPortals.Add(Portal);
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
	for (const FRopeWrapIslandDebugPortal& Portal : OutDebugPortals)
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
		OutAvailableSlack, FreeRestLength, CaptureSlabHalfWidth, OutDebugPortals.Num(),
		ClosedGeometryPortalCount, ClosedReachabilityPortalCount, OpenPortalCount, *BoneList);
}

#pragma endregion
