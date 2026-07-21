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

void FRopeWrappingPhase::UpdateVirtualBridgeRuns()
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

	// 프레임 예산 자동화(표면 감사 B-2): 설정값은 하한(내부 기본 8)이고, 실제 예산은 로프 길이에
	// 비례해 자동 상향된다 — 노드가 많아도 빌드가 ~4프레임 안에 끝나도록(SVF는 경로점당 스텝 2개 소모).
	// preview 경로는 설정값을 4096으로 덮어 한 번에 완주한다(BuildPreviewCenterline).
	const int32 AutoBudget = FMath::DivideAndRoundUp(State.NumTailNodes * 2, 4);
	const int32 StepBudget = FMath::Max3(1, Ctx.Config.WrappingPathBuildStepsPerFrame, AutoBudget);
	if (State.bPathUsesPoseSpaceIsland)
	{
		// Composite의 한 raw probe는 실제 projection 이동량에 따라 출력 node를 0개 또는 여러 개
		// 만들 수 있다. probe 호출 전 Path.Num()을 기억하고, 이번 호출에서 새로 생긴 범위 전체에
		// 순서대로 anchor를 붙여 LastAnchoredPathPointCount의 연속성을 보장한다.
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
				if (!AppendWrappingAnchorFromPathPoint(PathIndex, Sim, Ctx))
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
		UpdateVirtualBridgeRuns();

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
	AdvanceSurfaceVectorFieldProgressiveWrapPath(StepBudget, Sim, Ctx);
	UpdateVirtualBridgeRuns();
}

bool FRopeWrappingPhase::BeginProgressiveWrapPathBuild(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_BeginProgressiveWrapPathBuild);

	// 모든 초기화 시도는 비활성 상태에서 시작한다. 아래 어느 검증 단계에서 실패하더라도
	// FinishPathBuild를 거쳐 Active/Complete/Failed 플래그가 하나의 일관된 종료 상태를 갖는다.
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
	// 시드 다중화: 첫 보조 시드 노드부터는 경로가 아니라 보조 앵커가 노드를 소유한다 — 경로 길이를
	// 그 앞까지로 줄여 같은 노드를 경로 앵커와 보조 앵커가 이중으로 잡지 않게 한다(커밋 시드/Hold의
	// 마지막 쓰기 승자 경합 방지). 남는 로프(보조 노드 이후)는 Wrapping 동안 마스크로 동결됐다가
	// 커밋 후 자유 구간이 된다.
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
	// 새 progressive path마다 4단계 완료 게이트도 초기화한다. 이전 throw의 최종 목표나
	// 목표 도달 시간이 남으면 새 wrap이 즉시 commit될 수 있으므로 반드시 함께 리셋해야 한다.
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
		InitializeSurfaceVectorFieldProgressiveWrapPath(StoredLatchAnchor, Sim, Ctx);
	if (!bInitialized)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("InitialSurfacePathPointFailed"));
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Path initialization failed: reason=InitialSurfacePathPointFailed bone=%s node=%d surface=%s normal=%s"),
			*Ctx.OwnerName, *StoredLatchAnchor.Bone.ToString(), StoredLatchAnchor.NodeIndex,
			*State.PathSurfaceWorld.ToString(), *State.PathNormalWorld.ToString());
		return false;
	}
	if (!AppendWrappingAnchorFromPathPoint(0, Sim, Ctx))
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
	// 복합 경로가 활성화되면서 제거한 seed를 부분적으로 복구할 수는 없다. 단일 본 경로가 전체 tail의
	// 소유권을 명확하게 갖도록 남은 seed도 비운 뒤 최초 latch부터 다시 만든다.
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

bool FRopeWrappingPhase::InitializeSurfaceVectorFieldProgressiveWrapPath(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_InitSurfaceVectorFieldProgressivePath);

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
	// winding 기준 방향: 기본은 latch tangent(로프가 누운 방향). CaptureTravelPlane에서 캡처 속도가
	// 있으면 속도를 쓴다 — 감기 시작 방향이 "로프가 실제로 움직이던 쪽"과 일치해, 충돌 프레임의
	// tangent 노이즈에 흔들리지 않는다(진행 방향 기반 wrap 3단계).
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
	State.PathWrapIslandDebugMembers.Reset();
	State.PathWrapIslandDebugPortals.Reset();
	State.PathAvailableSlack = 0.0f;
	State.bPathUsesPoseSpaceIsland = false;
	State.PathCompositeSweepRadial = State.PathLatchRadial;
	State.PathCompositeProbeRadius = 0.0f;
	State.PathCompositeHelixRadius = 0.0f;
	State.PathCompositeHelixPitchScale = 0.0f;
	State.bPathCompositeHelixPitchFromContact = false;
	State.PathCompositeAxisMinDistance = 0.0f;
	State.PathCompositeAxisMaxDistance = 0.0f;
	State.bPathCompositeAxisRangeValid = false;
	State.PathCompositeSweepAngleRad = 0.0f;
	// Composite Multi-Bone은 순수 물리 결과를 쓰는 FullSimulation 전용이다. Assisted/Guaranteed는
	// island를 만들지 않고 아래 기존 SurfaceVectorField parent/child 순차 전환 경로를 그대로 탄다.
	if (Ctx.ResolveMode == ERopeWrapResolveMode::FullSimulation &&
		Ctx.Config.bEnableMultiBoneWrapping && !State.bPathUsesSingleBoneFallback)
	{
		GatherPoseSpaceWrapIsland(LatchAnchor, Sim, Mesh,
			State.PathWrapIslandBones, State.PathWrapIslandDebugMembers,
			State.PathWrapIslandDebugPortals, State.PathAvailableSlack, Ctx);
		// 단일 표면은 기존 projection/축 수학을 그대로 사용해 단일 본 감김의 각도와 튜닝을 보존한다.
		// 실제로 둘 이상의 본이 같은 pose-space 기둥으로 묶였을 때만 composite selector를 켠다.
		State.bPathUsesPoseSpaceIsland = State.PathWrapIslandBones.Num() > 1;
		if (State.bPathUsesPoseSpaceIsland)
		{
			// ResolveWrappingAxis는 island를 만들기 전에 호출되므로, 팔에서 먼저 닿으면 축 원점도 팔
			// collider 중심에 남는다. 그 축으로 outer support를 재면 반대편 팔이 과도하게 바깥으로
			// 보이고, 한 번 팔로 넘어간 경로가 몸통으로 돌아오지 못한다. 방향/감김 부호는 투척 프레임의
			// 정보를 그대로 보존하고, 원점의 축 수직 성분만 접촉 순간 복합 단면의 외곽 중심으로 옮긴다.
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
			float IslandMinAxisCoordinate = TNumericLimits<float>::Max();
			float IslandMaxAxisCoordinate = -TNumericLimits<float>::Max();
			int32 IslandBoundsContributorCount = 0;

			// OBB의 12개 edge와 latch 평면의 교점을 투영하면, 본 개수나 SDF bounds의 축 방향
			// 길이에 편향되지 않는 실제 단면 외곽 범위를 얻을 수 있다. SDF OBB가 없을 때만 world AABB를
			// identity OBB로 사용한다.
			static constexpr int32 BoxEdges[12][2] =
			{
				{ 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 },
				{ 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 },
				{ 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
			};

			for (const FRopeWrapIslandDebugMember& Member : State.PathWrapIslandDebugMembers)
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

			// 복합 단면에 실제로 둘 이상의 표면이 있을 때만 재중앙화한다. island graph만 이어졌지만
			// latch 높이에는 한 표면밖에 없는 경우는 기존 latch 축이 더 안전하다.
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

			// support query는 현재 선택된 팔 표면 근처가 아니라 복합 단면 전체의 바깥에서 시작한다.
			// 단면 반대편 collider도 같은 probe로 평가할 수 있도록 외곽 반대각 길이에 여유를 더한다.
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

			// 새 독립 Analytic Helix의 pitch는 디자이너 상수가 아니라 Contacting 순간 로프가 누운
			// tail 방향에서 읽는다. radial 접근 성분을 제거한 뒤 T ~= C + A*pitch로 분해하면
			// pitch = dot(T,A) / dot(T,C)다. 원주 성분이 거의 없으면 비율이 폭주하므로 0으로 둔다.
			const TCHAR* PitchSource = TEXT("NotUsed");
			float PitchAxisComponent = 0.0f;
			float PitchCircumferenceComponent = 0.0f;
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
					UnclampedPitchScale =
						PitchAxisComponent / PitchCircumferenceComponent;
					State.PathCompositeHelixPitchScale = UnclampedPitchScale;

					// Contact에서 얻은 pitch의 부호/기울기는 유지하되, 그 방향의 island 축 여유 안에
					// 계획된 전체 helix가 들어가도록 크기만 줄인다. 각 step의 실제 수식은
					//   axial = sqrt(segment^2 - radialStep^2) * pitch / sqrt(1 + pitch^2)
					// 이므로 radial entry까지 포함한 pitch=0 기준 진행량(BaseTravel)을 먼저 합산한다.
					if (State.bPathCompositeAxisRangeValid &&
						!FMath::IsNearlyZero(UnclampedPitchScale))
					{
						PitchAvailableAxisDistance = UnclampedPitchScale > 0.0f
							? State.PathCompositeAxisMaxDistance - CurrentAxisDistance
							: CurrentAxisDistance - State.PathCompositeAxisMinDistance;
						// 마지막으로 잘 감긴 pitch(-0.062)가 불필요하게 줄지 않도록 한 node의 일부만
						// 경계 여유로 둔다. ContactRadius보다 작은 여유는 SDF cap 양자화에 취약하다.
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
					State.bPathCompositeHelixPitchFromContact = bHasContactSpan;
					PitchSource = bHasContactSpan
						? TEXT("ContactSpan")
						: TEXT("LatchTangentFallback");
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

			// 동일한 초기 helix 방향에서 두 tangent를 분리한다. PathTangentWorld는 표면 anchor용으로
			// normal 평면에 투영하고, RawGuide는 projection 전 방향을 유지해 tail 흔들림을 차단한다.
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
					"pitch=%.3f rawPitch=%.3f pitchSource=%s axisClamp=%d maxPitch=%.3f "
					"axisRoom=%.2fcm usableAxis=%.2fcm safety=%.2fcm plannedBase=%.2fcm "
					"contactComponents(axis=%.3f circumference=%.3f)"),
				*Ctx.OwnerName, CrossSectionContributorCount, IslandBoundsContributorCount,
				*State.PathAxisOrigin.ToString(), *State.PathAxisDirection.ToString(),
				*State.PathCompositeSweepRadial.ToString(), State.PathCompositeProbeRadius,
				State.PathCompositeHelixRadius, CurrentSurfaceRadius,
				State.PathCompositeAxisMinDistance, State.PathCompositeAxisMaxDistance,
				State.PathWindingSign, State.PathCompositeHelixPitchScale,
				UnclampedPitchScale, PitchSource, bPitchClampedByAxisRange ? 1 : 0,
				AxisLimitedMaxAbsPitch, PitchAvailableAxisDistance,
				PitchUsableAxisDistance, PitchAxisSafetyMargin, PitchPlannedBaseTravel,
				PitchAxisComponent, PitchCircumferenceComponent);

			// 복합 island 자체가 여러 접촉 표면을 소유한다. 구형 secondary seed가 경로를 첫 보조 노드
			// 앞에서 잘라버리면 팔-몸통-팔 외곽을 만들 길이가 사라지므로, 이 경우에만 전체 tail을
			// progressive path에 돌려주고 보조 hold/commit 경합을 제거한다.
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

bool FRopeWrappingPhase::AppendWrappingAnchorFromPathPoint(int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx)
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

	// 허공 브리지(chord)와 composite virtual helix point는 표면 프레임이 없으므로 앵커를 만들지
	// 않는다 — 커밋 후 이 노드는 자유 로프로 남아 solver가 chord/현수 형태를 잡는다. 앵커 카운터는
	// 전진시켜야 한다: 이 함수는
	// PathIndex == LastAnchoredPathPointCount일 때만 신규 처리하므로, 여기서 멈추면 브리지 뒤
	// 재진입한 표면 경로점들의 앵커 생성이 전부 막힌다.
	if (Point.bBridge || Point.bVirtual)
	{
		State.LastAnchoredPathPointCount = PathIndex + 1;
		return true;
	}

	// MVP의 핵심: 경로점이 선택한 본을 그대로 anchor 소유 본으로 사용한다.
	// 이전 구현은 모든 anchor를 LatchAnchor.Bone 로컬로 저장했기 때문에,
	// path가 이웃 본 표면으로 넘어가더라도 Wrapped/Hold 단계에서는 한 본에 고정되어 보였다.
	// Point.Bone이 비어 있는 경우는 AnalyticHelix/legacy fallback으로 보고 latch bone을 사용한다.
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
		// guide도 anchor bone-local로 저장해 Wrapping 도중 캐릭터 애니메이션은 따라가되,
		// 표면 normal의 미세 굴곡에는 다시 투영되지 않게 한다.
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
