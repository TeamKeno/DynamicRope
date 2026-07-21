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

#pragma region Sequential Surface Vector Field Path

bool FRopeWrappingPhase::AdvanceSurfaceVectorFieldProgressiveWrapPath(int32 StepBudget, const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_AdvanceSurfaceVectorFieldProgressivePath);

	const USceneComponent* Mesh = ResolveWrappingMesh(State, State.LatchAnchor);
	if (!Mesh || State.LatchAnchor.Bone.IsNone())
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("AdvanceMissingMeshOrLatchBone"));
		return false;
	}

	const float StepSize = FMath::Max(1.0f, Sim.SegmentLength * 0.5f);
	// Composite probe와 같은 전체 시도 상한을 둔다. PathSweepDistance는 projection이 제자리여도
	// 매 integration step마다 StepSize만큼 증가하므로 별도 persistent counter 없이 무한 정체를 막는다.
	const int32 MaxIntegrationStepCount = FMath::Max(32, State.NumTailNodes * 16);
	const float MaxIntegrationSweepDistance =
		StepSize * static_cast<float>(MaxIntegrationStepCount);
	int32 StepsRemaining = FMath::Max(1, StepBudget);

	// 현재 축 기준 radial(축에서 점으로 향하는 단위벡터). 스텝 전/후 radial 사이 각도가 그 스텝의
	// 감싼 각도 증분이다 — 점이 축 위(축퇴)면 false.
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

		// 한 번의 predictor/projection을 실제 centerline integration segment 하나로 취급한다.
		// 노드 생성 여부와 관계없이 매 outer iteration에서 정확히 한 step을 소비한다.
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

			// 아래의 스냅 거리/브리지 코드는 State.PathSurfaceWorld를 이번 step의 경로 predictor로 읽는다.
			State.PathSurfaceWorld = PathPredictorWorld;
			const FName CurrentBone = State.PathCurrentBone.IsNone()
				? State.LatchAnchor.Bone
				: State.PathCurrentBone;
			const USceneComponent* ProjectedMesh = State.PathCurrentMesh.Get();
			if (!ProjectedMesh)
			{
				ProjectedMesh = Mesh;
			}

			// SingleBone fallback과 sequential multi-bone 모두 tangent predictor에서 projection한다.
			// 실제 rope node 위치를 tie-break에 넣어 거의 같은 후보 사이에서 현재 로프 몸체와 가까운
			// 표면을 고른다.
			const int32 RopeNodeIndex = State.LatchAnchor.NodeIndex + PathIndex;
			const FVector RopeNodeWorld = Sim.Positions.IsValidIndex(RopeNodeIndex)
				? Sim.Positions[RopeNodeIndex]
				: State.PathSurfaceWorld;
			// 투영 결과는 로컬로 받는다: 브리징(아래)에서 스냅을 거부할 수 있으므로, 수용이 확정되기
			// 전에는 State를 건드리지 않는다.
			FVector ProjectedSurface = State.PathSurfaceWorld;
			FVector ProjectedNormal = State.PathNormalWorld;
			FVector ProjectedTangent = State.PathTangentWorld;
			FVector ProjectedCircumference = State.PathCircumferenceDir;
			FName ProjectedBone = CurrentBone;
			bool bOnSurface = false;
			if (State.bPathUsesSingleBoneFallback)
			{
				bOnSurface = ProjectWrapPointToSingleBone(Mesh, Sim, Ctx,
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

			// 갭 브리징(WrappingMaxGapBridgeDistance > 0)에서는 스냅 수용에 두 가지 관문을 둔다.
			// 브리징 비활성 시에는 아무 관문도 없다 — 관대한 스냅으로 abort를 줄이는 종전 동작(기본값) 그대로.
			//  ① 스냅 거리 상한: 표면이 예측점에서 한 세그먼트 이상 떨어져 있으면 "여기엔 감을 표면이
			//     없다"로 보고 chord로 간다. 없으면 관대한 QueryRadius(세그먼트×3) 때문에 대상 사이
			//     허공에서도 먼 표면으로 끌려가 경로가 골짜기로 말려든다.
			//  ② winding 역행(taut-string 이탈점): 스냅을 수용한 결과가 축 기준 중심각을 되돌리면
			//     경로가 첫 대상의 뒤편을 맴돌고 있는 것이다 — 쌍(양다리)을 도는 감김은 중심각이
			//     winding 방향으로 단조 전진한다. 팽팽한 줄은 여기서 표면을 떠나 chord로 간다.
			//     이 관문이 없으면 예측점이 항상 직전 표면점 근처라 ①에 안 걸리고(원 단면 위에서는
			//     스냅 변위가 어디서나 균일하게 작다 — 국소 신호로는 이탈점을 구분할 수 없다),
			//     경로가 첫 대상만 영원히 궤도 돌아 쌍으로 건너가지 못한다. 단일 대상 감김은 중심각이
			//     본래 단조라 오탐하지 않는다(이탈은 하울 접점보다 약간 늦게 오고, 남는 느슨함은
			//     커밋 후 자유 노드를 solver가 당겨 정리한다).
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

			// 표면도 없고 브리지도 소진/비활성 — 종전과 같은 실패 처리(각도 적분 전에 끊어,
			// 걷지 못한 스텝이 실패 시점 각도(ShouldAbortFailedShortWrap)에 섞이지 않게 한다).
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

			// 감싼 각도 적분: 이번 스텝이 만든 radial 회전량을 누적한다. 축 재해석(아래 accept 분기의
			// reseed) *전에*, 이 스텝을 실제로 걸었던 축 기준으로 전/후 radial을 재야 한다. 브리지
			// 스텝도 적분한다 — chord가 가로지른 각도 구간도 "감쌌다"에 포함되는 것이 둘레 커버리지
			// 척도(5단계 형상 기준 판정)와 일치한다.
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

				// acos 누적은 전진/후퇴를 모두 양수로 더한다. 같은 step을 atan2로도 측정해 winding
				// 방향 전진과 역방향 흔들림을 분리한다. 기존 각도 기반 동작/품질 판정은 건드리지 않는다.
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

				// ProjectWrapPointToSurfaceMultiBone이 hysteresis까지 적용해 최종 본을 돌려준다.
				// 여기서는 상태만 갱신한다. 전환했다면 직전 본을 기록해 다음 step에서 바로 되돌아가는 후보에
				// penalty를 줄 수 있게 하고, 전환 거리 누적은 0으로 다시 시작한다.
				if (ProjectedBone != CurrentBone)
				{
					++State.PathBoneTransitionCount;
					State.PathPreviousBone = CurrentBone;
					State.PathCurrentBone = ProjectedBone;
					State.PathDistanceSinceBoneTransition = 0.0f;
					// 순차 본 전환은 새 본 형상 축으로 rolling axis를 재시드한다.
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
				// 브리지 스텝: 예측 위치(tangent 직진)를 그대로 쓰고 앵커 프레임 없이 지나간다.
				// normal은 축 radial로 유지해 다음 스텝의 원주 tangent가 깨끗하게 나오게 한다.
				// bone/mesh는 유지 — 재진입 후보를 현재 본 중심 그래프에서 계속 찾는다.
				State.PathBridgeDistance += StepDistance;
				State.PathDistanceSinceBoneTransition += StepDistance;
				FVector BridgeRadial = FVector::ZeroVector;
				if (ComputeAxisRadial(State.PathSurfaceWorld, BridgeRadial))
				{
					State.PathNormalWorld = BridgeRadial;
				}
			}

			// predictor/sweep가 시도한 명목 거리와 projection 결과가 실제로 만든 centerline 길이를
			// 분리한다. 이전 구현은 아래 ActualStepDistance와 무관하게 StepDistance를 경로 길이로
			// 기록해, 표면 스냅이 짧거나 긴 구간에서 rope node 간격이 뭉치거나 늘어났다.
			const float CenterlineOffset = FMath::Max(0.0f, Ctx.SurfaceOffset);
			const FVector PreviousCenterlineWorld =
				PreviousSurfaceWorld + PreviousNormalWorld * CenterlineOffset;
			const FVector CurrentCenterlineWorld =
				State.PathSurfaceWorld + State.PathNormalWorld * CenterlineOffset;
			const float ActualStepDistance = FVector::Dist(
				PreviousCenterlineWorld, CurrentCenterlineWorld);
			const float ArcStartDistance = State.PathCurrentDistance;
			const float ArcEndDistance = ArcStartDistance + ActualStepDistance;

			// 이번 실제 integration segment가 하나 이상의 SegmentLength 경계를 통과하면 각 경계에서
			// centerline frame을 재샘플링한다. 한 projection 점프가 여러 node 경계를 넘을 수 있으므로
			// if가 아니라 while이다. DistanceFromLatch/RopeDistance는 이제 실제 polyline arc 좌표다.
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
					// 브리지에서 출발하거나 이번 step이 브리지면 보간점도 허공 chord로 취급한다.
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
					// 물리 위치는 실제 centerline arc로 재샘플링하되 animation phase는 같은
					// integration segment의 forward signed angle을 보간한다. reverse 성분을 빼므로
					// point angle은 감소하지 않고 angle -> distance binary search가 안정적으로 동작한다.
					Point.WrapAngleFromLatchRad = FMath::Lerp(
						PreviousForwardAngleRad, State.PathForwardAngleRad, Alpha);
					State.Path.Add(Point);
					if (!AppendWrappingAnchorFromPathPoint(SamplePathIndex, Sim, Ctx))
					{
						// Path와 anchor 처리를 하나의 원자적 append로 취급한다. 현재 점 또는 같은
						// integration segment에서 아직 처리하지 못한 뒤쪽 점을 남기면 이후 resolve가
						// 저장 당시 월드 위치를 정상 anchor처럼 사용할 수 있으므로 모두 제거한다.
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

		// 감는 양 상한(WrappingMaxWrapAngleDeg > 0): 누적 감싼 각도가 목표에 닿으면 경로를 여기서
		// *성공*으로 마감한다 — 나선이 목표 바퀴수를 넘어 남은 로프 전량을 감아 들어가는 것을 막고,
		// 경로 밖 로프는 커밋 후 자유 구간으로 늘어뜨린다. NumTailNodes를 빌드된 경로 길이로 줄여
		// front/커밋 목표 거리(RequestedFrontDistance = (NumTailNodes-1)·세그먼트)가 실제 경로와
		// 일치하게 한다 — 전체 로프 기준 그대로면 front가 경로 밖 거리를 겨냥해 도달 판정이 영원히
		// 안 되고 settle 타임아웃 커밋으로만 떨어진다.
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

	// 기존 Sequential Multi-Bone/비복합 동작을 보존하기 위한 skeleton parent/child 후보만 수집한다.
	// 복합 SDF 실패 폴백은 이 함수 자체를 호출하지 않는다.
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

	// 이 함수는 SurfaceVectorField 전용 projection 선택기다.
	// 기존 단일 본 방식은 LatchAnchor.Bone만 통과시켰지만, 여기서는 후보 본마다
	// "예측 위치에서 가장 그럴듯한 표면점"을 평가한 뒤 path point의 Bone/Mesh로 보존한다.
	//
	// 점수 항목:
	// - Projection.Distance: 예측 위치에서 표면까지 얼마나 멀리 튀었는지. 낮을수록 좋다.
	// - RopeNodeDistance: 실제 rope node와 표면점이 가까운 후보를 선호한다.
	// - GraphCost: parent/child graph를 많이 건넌 후보일수록 불리하다.
	// - Tangent/NormalPenalty: 이전 frame/step의 surface field와 갑자기 꺾이는 후보를 줄인다.
	// - CurrentBoneBonus + hysteresis: 현재 본이 아직 쓸 만하면 새 본이 확실히 좋아야 전환한다.
	// - ImmediateBoneReturnPenalty: 직전 본으로 바로 돌아가는 A->B->A 왕복을 줄인다.
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

		// 낮은 Score가 이긴다.
		// distance 계열은 "예측 path/실제 node에서 얼마나 멀어졌는가"이고,
		// continuity 계열은 "표면 field가 얼마나 갑자기 꺾였는가"다.
		// CurrentBoneBonus는 실제 보너스이므로 마지막에 빼서, 동점 근처에서는 현재 본을 유지하게 한다.
		float Score =
			Projection.Distance * Ctx.Config.ProjectionDistanceWeight +
			FVector::Dist(Projection.SurfacePoint, RopeNodeWorld) * Ctx.Config.RopeNodeDistanceWeight +
			Candidate->GraphCost * Ctx.Config.BoneTransitionPenaltyWeight +
			TangentPenalty * Ctx.Config.TangentContinuityWeight +
			NormalPenalty * Ctx.Config.NormalContinuityWeight -
			(bCurrentBone ? Ctx.Config.CurrentBoneBonus : 0.0f);

		if (!PreviousBone.IsNone() && Projection.Bone == PreviousBone && Projection.Bone != CurrentBone)
		{
			// 방금 떠난 본으로 바로 돌아가는 후보는 표면 projection이 조금 좋아 보여도
			// A->B->A 왕복 떨림을 만들 가능성이 높다. 완전 금지는 아니고 점수만 불리하게 만든다.
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
		// 현재 본 projection도 아직 성공했다면 전환은 보수적으로 한다.
		// 새 본이 hysteresis만큼 확실히 좋고, 마지막 전환 이후 최소 거리도 지난 경우에만 Best를 유지한다.
		// 둘 중 하나라도 부족하면 CurrentBoneProjection으로 되돌려, path point/anchor가 짧은 구간에서
		// 여러 본 사이를 흔들며 저장되는 것을 막는다.
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

bool FRopeWrappingPhase::ProjectWrapPointToSingleBone(const USceneComponent* Mesh,
	const FRopeSimState& Sim, const FContext& Ctx,
	FVector& InOutSurfaceWorld, FVector& InOutNormalWorld, FVector& InOutTangentWorld,
	FVector& InOutCircumferenceDir, FName& InOutBone, const USceneComponent*& OutMesh) const
{
	// 복합 경로의 후보/graph 상태를 재사용하지 않는다. 최초 latch 본과 동일 mesh에 귀속된 collider만
	// 기존 단일 본 projection으로 찾고, 그 표면 프레임에서 tangent를 다시 계산한다.
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
