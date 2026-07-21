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

	// Path와 anchor는 PathIndex/NodeIndex 순서로 함께 생성된다. 현재 프레임의 움직이는 bone frame을
	// 한 번만 해석해 재사용하면, 아래 각 노드가 SampleWrappingPath의 path/anchor 선형 탐색을
	// 반복하던 O(N²) 비용을 피할 수 있다.
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
	// 이미 감긴 위치는 projected surface path를 따르지만, 아직 감기지 않은 tail의 연장 방향은
	// SDF normal에 투영하지 않은 ideal helix guide를 우선 사용한다. Sequential 경로와 guide가
	// 없는 구형 데이터는 종전 surface tangent로 폴백한다.
	const FVector TailGuideDirection =
		State.bPathUsesPoseSpaceIsland && FrontPoint.bHasWrappingGuideTangent
			? FrontPoint.WrappingGuideTangentWorld.GetSafeNormal(
				KINDA_SMALL_NUMBER, FrontPoint.TangentWorld)
			: FrontPoint.TangentWorld;
	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	// 실제 surface path/anchor의 소유 범위와 Wrapping 중 시각적으로 구동할 범위를 분리한다.
	// Composite analytic helix는 island 축 범위에서 path가 먼저 끝나더라도 남은 tail 전체를 현재
	// ideal helix guide 방향으로 정렬한다. NumTailNodes 자체는 줄어든 path/commit 범위를 계속 뜻한다.
	const bool bDriveFullCompositeTail = State.bPathUsesPoseSpaceIsland;
	int32 TailEndNode = Sim.Num() - 1;
	if (!bDriveFullCompositeTail && State.NumTailNodes > 0)
	{
		// Sequential은 기존 정책을 유지한다. 실제로 생성된 path가 소유하는 노드까지만 움직이고,
		// 그 뒤 자유 tail은 이 단계의 강제 위치 애니메이션에 포함하지 않는다.
		TailEndNode = FMath::Min(TailEndNode, LatchNode + State.NumTailNodes - 1);
	}
	// 시드 다중화: 경로 구동은 첫 보조 시드 노드 *앞*에서도 끝난다(NumTailNodes 클램프와 같은 경계).
	// 보조 노드는 아래에서 자기 본에 hold하고, 그 너머 남는 로프도 마찬가지로 동결 유지된다 —
	// front 직선 연장이 보조 대상 반대편으로 로프를 끌어가는 것을 막는다.
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

		// Radial SDF ray가 표면을 찾지 못한 composite helix point는 이 페이즈에서도
		// 위치 override를 주지 않는다. 이 노드만 즉시 solver가 계산하게 해 Wrapped
		// 커밋 순간 virtual guide가 한꺼번에 사라지는 위치 discontinuity를 없앤다.
		const int32 PathIndex = NodeIndex - LatchNode;
		if (State.Path.IsValidIndex(PathIndex) && State.Path[PathIndex].bVirtual)
		{
			continue;
		}

		const float NodeDistance = static_cast<float>(NodeIndex - LatchNode) * SegmentLength;
		FVector World = FVector::ZeroVector;
		if (NodeDistance <= State.FrontDistance + KINDA_SMALL_NUMBER)
		{
			// 감김 front가 이미 지나간 노드는 실제 SDF projection 경로의 같은 rope distance를
			// 따른다. Path point는 정확히 PathIndex * SegmentLength 간격으로 생성되므로 이번 프레임에
			// 한 번 해석한 배열을 직접 인덱싱할 수 있다.
			if (State.Path.IsValidIndex(PathIndex) &&
				ResolvedPathScratch.IsValidIndex(PathIndex))
			{
				World = GetPathPointCenterlineWorld(
					ResolvedPathScratch[PathIndex], SurfaceOffset);
			}
			else
			{
				// Path가 없는 구형 anchors-only 상태는 이번 프레임에 resolve/정렬한 anchor 배열을
				// binary search한다. 이 경계 상태에서도 노드마다 anchor 전체를 다시 훑지 않는다.
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
			// Composite tail은 SDF projection tangent가 아니라 초기 ideal helix guide를 따른다.
			// 위치는 계속 projected surface path를 사용하되, 미세 normal 변화가 tail 전체를 흔들지 않는다.
			// Composite에서는 실제 path가 island 축 끝에서 먼저 종료돼도 이 식을 마지막 rope node까지
			// 적용한다. 이 구간에는 새 surface anchor를 만들지 않고 Wrapping 동안의 시각적 정렬만 준다.
			//   tail 위치 = 현재 front 위치 + helix guide 방향 * front에서 남은 rope 길이
			World = FrontWorld + TailGuideDirection * (NodeDistance - State.FrontDistance);
		}

		OutFrame.SetPosition(NodeIndex, World, /*bZeroVelocity*/ true);
	}

	// 보조 시드 노드 hold: 경로/front와 무관하게 자기 본의 표면 앵커 프레임을 따라간다(움직이는
	// 대상 추종 — Wrapped의 Hold와 같은 수식). mesh가 사라진 프레임은 건너뛴다 — 노드는 마스크
	// 동결로 제자리에 남고, 커밋 후 Hold가 mesh 소실을 정식으로 감지해 release한다.
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
	const int32 LatchNode = State.LatchAnchor.NodeIndex;
	const bool bHasValidLatch = Sim.InvMass.IsValidIndex(LatchNode);
	// 실제 path 범위와 무관하게 Wrapping 애니메이션 동안에는 latch 이후 전체 tail을 kinematic으로
	// 유지한다. Composite axis limit 이후의 guide-only tail도 ApplyWrappingMotionOverrides가 강제로 애니메이팅하고,
	// 커밋 시 실제 anchor가 없는 노드만 마지막 위치에서 속도 0 상태로 solver에 반환된다.
	const int32 DrivenEndNode = Sim.Num() - 1;

	OutFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		// Radial projection에 실패한 virtual path node만 solver에 남긴다. Composite path 바깥의
		// guide-only tail은 위 DrivenEndNode 범위에 포함되어 ApplyWrappingMotionOverrides의 위치를 그대로 따른다.
		const int32 PathIndex = i - LatchNode;
		const bool bNoAnchorSolverNode =
			State.Path.IsValidIndex(PathIndex) && State.Path[PathIndex].bVirtual;
		const bool bWrappingDrivenNode =
			bHasValidLatch && i >= LatchNode && i <= DrivenEndNode && !bNoAnchorSolverNode;
		// ApplyWrappingMotionOverrides로 위치를 직접 쓰는 노드는 같은 프레임의 solver가 다시 움직이지 못하도록
		// 질량을 0으로 만든다. Wrapped 커밋 뒤에는 실제 anchor가 없는 guide-only tail이 다시 dynamic이 된다.
		OutFrame.SetInvMass(i, (bStartPin || bWrappingDrivenNode) ? 0.0f : 1.0f);
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
	// Legacy/실패 초기화처럼 Path가 비어 있는 경우만 anchor 거리를 폴백으로 쓴다.
	if (State.Path.Num() == 0)
	{
		for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
		{
			BuiltPathMaxDistance = FMath::Max(
				BuiltPathMaxDistance, Anchor.RopeDistance);
		}
	}

	const float TargetFrontDistance = FMath::Min(FullDistance, BuiltPathMaxDistance);
	// Advance와 commit이 서로 다른 목표를 보지 않도록, 이번 프레임에 사용한 실제 거리 cap을
	// 상태에 보존한다. 최종 path build가 끝나면 이 값이 최종 완료 거리로 확정된다.
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
		// 기존 front의 초반 완속/후반 가속 곡선은 보존하고 입력 위상만 distance progress에서
		// Single/Composite 공통 angle progress로 바꾼다.
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
	// IsReadyToCommit에서도 동일한 정책을 선택할 수 있도록 현재 front 구동 방식을 기록한다.
	State.bFrontUsesAngleMapping = bUseAngleMappedFront;
	const TCHAR* FrontPolicy = bUseAngleMappedFront
		? TEXT("AngleMapped")
		: TEXT("DistanceFallback");

	if (bUseAngleMappedFront)
	{
		// 두 모드의 path는 distance와 animation angle이 모두 단조 증가한다. 현재까지 빌드된 path만
		// 대상으로 양방향 보간을 제공해 front가 progressive build를 앞질러 순간 점프하지 않게 한다.
		// build cap에 막힌 동안 미소비 angle debt도 별도로 쌓지 않는다.
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

		// 초기 몇 frame에 angle point가 아직 없어 distance fallback이 먼저 움직였더라도, angular
		// path가 준비되는 순간 현재 distance의 angle로 이어 받아 위치가 latch로 되감기지 않게 한다.
		if (State.FrontWrapAngleRad <= KINDA_SMALL_NUMBER &&
			State.FrontDistance > KINDA_SMALL_NUMBER)
		{
			State.FrontWrapAngleRad = FindAngleAtDistance(State.FrontDistance);
			PreviousFrontAngleRad = State.FrontWrapAngleRad;
		}

		TargetFrontAngleRad = FindAngleAtDistance(TargetFrontDistance);
		// 빌드 중에는 최종 angle을 아직 모르므로 현재 평균 angular density를 FullDistance까지
		// 외삽한다. 이렇게 해야 짧게 빌드된 path의 끝에 닿았다는 이유로 easing이 즉시 3배가 되지 않는다.
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
		// 각도 parameter가 없는 legacy/degenerate path만 기존 distance front로 안전하게 폴백한다.
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

	// AngleMapped는 두 모드 모두 point별 실제 animation angle을 쓴다. legacy/degenerate fallback만
	// Sequential 누적 forward angle의 전체 평균으로 timing 로그를 추정한다.
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

	// 4단계 완료 게이트는 progressive build의 임시 cap이 아니라 최종 path에 대해서만 arm한다.
	// angle과 distance가 모두 끝난 최초 시간을 기록해 기존 per-segment tail deadline 대신 짧은
	// post-front settle 시간을 잴 수 있게 한다.
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

	// Progressive build가 끝난 뒤 한 번만 출력한다. 두 모드의 measuredLinearSpeed가 path의
	// 구간별 cm/rad에 따라 달라도 measuredAngularSpeed는 같은 공통 angular policy를 따라야 한다.
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
		// surface tangent와 달리 guide에는 normal 평면 투영을 적용하지 않는다. 이 벡터는
		// 충돌/고정 프레임이 아니라 front 뒤 tail의 시각적 연장 방향으로만 소비된다.
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
	// bVirtual만 SurfaceWorld에 이미 rope centerline을 저장한다. Sequential bridge는
	// anchor가 없을 뿐 일반 surface point와 같은 offset 이전 기준 위치를 저장한다.
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
	// 두 플래그 모두 anchor 생략 정책에는 참여하지만 좌표 저장 규약은 bVirtual만 결정한다.
	OutPoint.bBridge = LowerPoint.bBridge || UpperPoint.bBridge;
	OutPoint.bVirtual = LowerPoint.bVirtual || UpperPoint.bVirtual;
	OutPoint.NormalWorld = FMath::Lerp(
		LowerPoint.NormalWorld, UpperPoint.NormalWorld, Alpha)
		.GetSafeNormal(KINDA_SMALL_NUMBER, LowerPoint.NormalWorld);
	// 저장 표현(surface 기준/virtual centerline)을 직접 섞지 않는다. 양 끝점을 centerline으로
	// 해석해 보간한 뒤, 출력 point가 따르는 저장 규약으로 한 번만 되돌린다.
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
		// front가 두 path point 사이를 이동할 때 guide도 같은 alpha로 보간해 방향이 node
		// 경계에서 계단식으로 바뀌지 않게 한다. guide가 없는 쪽은 surface tangent로 폴백한다.
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
			// 움직이는 본에서 위치/법선은 anchor frame으로 갱신하되, path가 빌드될 때
			// 확정한 animation angle parameter는 저장된 point 값으로 복원한다.
			ResolvedPoint.WrapAngleFromLatchRad = StoredPoint.WrapAngleFromLatchRad;
			OutResolvedPath[PathIndex] = MoveTemp(ResolvedPoint);
			++AnchorIndex;
		}
		else
		{
			// Progressive build에서 anchor가 아직 붙기 전인 같은 프레임의 짧은 창은 스냅샷을 쓴다.
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
