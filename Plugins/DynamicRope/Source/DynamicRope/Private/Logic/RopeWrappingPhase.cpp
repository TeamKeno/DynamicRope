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

#pragma region Wrapping Lifecycle, Commit, and Preview

bool FRopeWrappingPhase::Begin(const FRopeSurfaceAnchor& LatchAnchor, float Duration,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	// 새 throw는 조건이 맞으면 Composite Analytic Helix부터 시작한다. 이 플래그는 같은 throw 안에서
	// 해당 경로가 terminal failure로 끝났을 때만 켜져 두 번째 composite 시도와 무한 fallback을 막는다.
	State.bPathUsesSingleBoneFallback = false;
	State.PathBuildFailureReason.Reset();
	State.PathCompositeProjectionFailureCount = 0;
	State.BoneName = LatchAnchor.Bone;
	State.Mesh = LatchAnchor.Mesh;
	State.Elapsed = 0.0f;
	State.Duration = Duration;
	State.FirstNode = TNumericLimits<int32>::Max();
	State.LastNode = INDEX_NONE;

	if (!BeginProgressiveWrapPathBuild(LatchAnchor, Sim, Ctx))
	{
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Wrap begin failed: reason=%s bone=%s node=%d mesh=%s simNodes=%d segment=%.2fcm colliders=%d"),
			*Ctx.OwnerName,
			State.PathBuildFailureReason.IsEmpty() ? TEXT("UnknownInitializationFailure") : *State.PathBuildFailureReason,
			*LatchAnchor.Bone.ToString(), LatchAnchor.NodeIndex,
			*GetNameSafe(LatchAnchor.Mesh.Get()),
			Sim.Num(), Sim.SegmentLength,
			Ctx.Colliders.Num());
		return false;
	}

	State.StableTime = 0.0f;
	State.LastStableFirstNode = State.FirstNode;
	State.LastStableLastNode = State.LastNode;
	State.LastStableAnchorCount = State.Anchors.Num();
	return true;
}

void FRopeWrappingPhase::UpdateAnchorSpanStability(float DeltaTime)
{
	const bool bSameStableSpan =
		State.FirstNode == State.LastStableFirstNode &&
		State.LastNode == State.LastStableLastNode &&
		State.Anchors.Num() == State.LastStableAnchorCount;

	if (bSameStableSpan)
	{
		State.StableTime += DeltaTime;
	}
	else
	{
		State.StableTime = 0.0f;
		State.LastStableFirstNode = State.FirstNode;
		State.LastStableLastNode = State.LastNode;
		State.LastStableAnchorCount = State.Anchors.Num();
	}
}

bool FRopeWrappingPhase::IsReadyToCommit(const FRopeSimState& Sim, const FRopeWrapConfig& Config) const
{
	const bool bHasEnoughAnchors = State.Anchors.Num() > 0;
	float MaxTailDelay = 0.0f;
	float BuiltPathMaxDistance = 0.0f;
	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		BuiltPathMaxDistance = FMath::Max(BuiltPathMaxDistance, Anchor.RopeDistance);
		MaxTailDelay = FMath::Max(MaxTailDelay,
			(Anchor.RopeDistance / SegmentLength) * Config.WrappingTailDelayPerSegment);
	}
	const float RequestedFrontDistance = State.NumTailNodes > 0
		? static_cast<float>(State.NumTailNodes - 1) * Sim.SegmentLength
		: 0.0f;
	const float CommitFrontDistance = State.bPathBuildFailed
		? BuiltPathMaxDistance
		: RequestedFrontDistance;

	// 정상 AngleMapped path는 animation phase와 물리 배치가 모두 끝나야 commit한다. path build
	// 실패/degenerate fallback은 angle target이 없을 수 있으므로 기존 distance 정책을 유지한다.
	const bool bUseAngularCompletion = State.bFrontUsesAngleMapping &&
		State.bPathBuildComplete && !State.bPathBuildFailed &&
		State.FrontTargetWrapAngleRad > KINDA_SMALL_NUMBER;
	const float EffectiveCommitDistance = bUseAngularCompletion
		? State.FrontTargetDistance
		: CommitFrontDistance;
	const float DistanceCompletionTolerance = FMath::Max(0.01f, SegmentLength * 0.001f);
	const float AngleCompletionToleranceRad = FMath::DegreesToRadians(0.1f);
	const bool bDistanceDone = State.FrontDistance + DistanceCompletionTolerance >=
		EffectiveCommitDistance;
	const bool bAngleDone = !bUseAngularCompletion ||
		State.FrontWrapAngleRad + AngleCompletionToleranceRad >=
			State.FrontTargetWrapAngleRad;
	const bool bFrontDone = bAngleDone && bDistanceDone;

	// DistanceFallback에만 종전 per-segment deadline을 적용한다. AngleMapped는 front가 목표에
	// 도달한 뒤의 짧은 settle과 최소 phase duration만 기다려 중복 tail delay를 제거한다.
	const bool bLegacyMotionDone = State.Elapsed >= State.Duration + MaxTailDelay;
	const bool bMinimumPhaseDurationDone = State.Elapsed >= State.Duration;
	const bool bPostFrontSettleDone = State.FrontReachedTargetElapsed >= 0.0f &&
		State.Elapsed + KINDA_SMALL_NUMBER >=
			State.FrontReachedTargetElapsed + FMath::Max(0.0f, Config.WrappingPostFrontSettleTime);
	const bool bCompletionTimingDone = bUseAngularCompletion
		? (bMinimumPhaseDurationDone && bPostFrontSettleDone)
		: bLegacyMotionDone;
	const bool bStable = !bUseAngularCompletion ||
		State.StableTime + KINDA_SMALL_NUMBER >= FMath::Max(0.0f, Config.WrappingStableTime);
	const bool bTimedOutWithAnchors =
		Config.WrappingMaxSettleTime > 0.0f &&
		State.Elapsed >= Config.WrappingMaxSettleTime;
	const bool bPathReadyToCommit =
		!State.bPathBuildActive ||
		State.bPathBuildComplete ||
		State.bPathBuildFailed;

	// timeout은 front 목표 자체를 건너뛰지는 않지만, 비정상적으로 안정화/settle이 끝나지 않는
	// 경우의 마지막 탈출구로 종전과 같이 timing gate를 우회한다.
	const bool bNormalCompletion = bFrontDone && bStable && bCompletionTimingDone;
	const bool bTimeoutCompletion = bFrontDone && bTimedOutWithAnchors;
	return bHasEnoughAnchors && bPathReadyToCommit &&
		(bNormalCompletion || bTimeoutCompletion);
}

bool FRopeWrappingPhase::ShouldAbortFailedShortWrap(const FRopeSimState& Sim, const FContext& Ctx,
	float MinRequiredAngleDeg, float& OutAngleDeg) const
{
	OutAngleDeg = 0.0f;
	if (MinRequiredAngleDeg <= 0.0f || !State.bPathBuildFailed)
	{
		return false;
	}

	if (!ComputeBuiltPathWrapAngle(Sim, Ctx, OutAngleDeg))
	{
		return false;
	}

	return OutAngleDeg < MinRequiredAngleDeg;
}

FRopeWrapState FRopeWrappingPhase::BuildCommitSeed(const FRopeSimState& Sim) const
{
	FRopeWrapState Seed;
	Seed.BoneName = State.LatchAnchor.Bone;
	Seed.Mesh = ResolveWrappingMesh(State, State.LatchAnchor);

	//각 LatchedNode에 정보 입력
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (!Sim.Positions.IsValidIndex(Anchor.NodeIndex))
		{
			continue;
		}

		FRopeLatchNode Latch;
		Latch.NodeIndex = Anchor.NodeIndex;
		Latch.Bone = Anchor.Bone;
		Seed.Latched.Add(Latch);
		Seed.Anchors.Add(Anchor);
	}

	// 시드 다중화: 보조 시드 앵커도 커밋에 합류한다 — BeginWrap/Hold는 앵커별 (Bone, Mesh)를
	// 이미 지원하므로 이 뒤로는 경로 앵커와 동일하게 취급된다. 노드 중복은 경로 길이 클램프가
	// 막지만(빌드 시점 보장), 시드가 어긋난 경우를 대비해 한 번 더 거른다.
	for (const FRopeSurfaceAnchor& Secondary : State.SecondarySeedAnchors)
	{
		if (!Sim.Positions.IsValidIndex(Secondary.NodeIndex))
		{
			continue;
		}

		const bool bNodeTaken = Seed.Anchors.ContainsByPredicate(
			[&Secondary](const FRopeSurfaceAnchor& Existing)
			{
				return Existing.NodeIndex == Secondary.NodeIndex;
			});
		if (bNodeTaken)
		{
			continue;
		}

		FRopeLatchNode Latch;
		Latch.NodeIndex = Secondary.NodeIndex;
		Latch.Bone = Secondary.Bone;
		Seed.Latched.Add(Latch);
		Seed.Anchors.Add(Secondary);
	}

	return Seed;
}

void FRopeWrappingPhase::ReleaseAnchoredNodesToSolver(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const
{
	OutFrame.EnsureSize(Sim.Num());
	auto ReturnNode = [&Sim, &OutFrame](int32 NodeIndex)
	{
		if (!Sim.InvMass.IsValidIndex(NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(NodeIndex) ||
			!Sim.Positions.IsValidIndex(NodeIndex))
		{
			return;
		}

		OutFrame.SetInvMass(NodeIndex, 1.0f);
		OutFrame.SetPrevFromPosition(NodeIndex);
	};

	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		ReturnNode(Anchor.NodeIndex);
	}
	// 보조 시드 노드도 abort 시 함께 복귀한다(front hold로 위치가 덮여 왔으므로 Prev=Pos 튐 방지 동일).
	for (const FRopeSurfaceAnchor& Secondary : State.SecondarySeedAnchors)
	{
		ReturnNode(Secondary.NodeIndex);
	}
}

bool FRopeWrappingPhase::BuildPreviewCenterline(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx, TArray<FVector>& OutCenterline) const
{
	OutCenterline.Reset();
	if (!LatchAnchor.Mesh.IsValid() || LatchAnchor.Bone.IsNone() ||
		!Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) || Sim.Num() < 2)
	{
		return false;
	}

	FRopeWrapConfig PreviewConfig = Ctx.Config;
	PreviewConfig.WrappingPathBuildStepsPerFrame = 4096;
	FContext PreviewCtx{ PreviewConfig, Ctx.Colliders, Ctx.SurfaceOffset, Ctx.OwnerName, true };

	// preview에도 runtime과 같은 축/접촉/resolve 컨텍스트를 승계한다.
	PreviewCtx.bHasGuidePlaneNormal = Ctx.bHasGuidePlaneNormal;
	PreviewCtx.GuidePlaneNormal = Ctx.GuidePlaneNormal;
	PreviewCtx.TravelFrame = Ctx.TravelFrame;
	PreviewCtx.ResolvedContactRadius = Ctx.ResolvedContactRadius;
	PreviewCtx.ResolveMode = Ctx.ResolveMode;

	FRopeWrappingPhase PreviewPhase;
	if (!PreviewPhase.Begin(LatchAnchor,
		FMath::Max(0.01f, PreviewConfig.WrappingMotionDuration), Sim, PreviewCtx))
	{
		return false;
	}

	const int32 MaxIterations = FMath::Max(1, Sim.Num() * 4);
	for (int32 Iteration = 0;
		Iteration < MaxIterations &&
		PreviewPhase.State.bPathBuildActive &&
		!PreviewPhase.State.bPathBuildComplete &&
		!PreviewPhase.State.bPathBuildFailed;
		++Iteration)
	{
		PreviewPhase.AdvancePathBuild(Sim, PreviewCtx);
	}

	if (PreviewPhase.State.Path.Num() == 0)
	{
		return false;
	}

	OutCenterline = Sim.Positions;
	const float SurfaceOffset = FMath::Max(0.0f, PreviewCtx.SurfaceOffset);
	int32 LastDrivenNode = INDEX_NONE;
	for (int32 PathIndex = 0; PathIndex < PreviewPhase.State.Path.Num(); ++PathIndex)
	{
		const int32 NodeIndex = LatchAnchor.NodeIndex + PathIndex;
		if (!OutCenterline.IsValidIndex(NodeIndex))
		{
			break;
		}

		const FRopeWrapPathPoint& Point = PreviewPhase.State.Path[PathIndex];
		OutCenterline[NodeIndex] = GetPathPointCenterlineWorld(Point, SurfaceOffset);
		LastDrivenNode = NodeIndex;
	}

	if (OutCenterline.IsValidIndex(LastDrivenNode))
	{
		const float SegmentLength = FMath::Max(Sim.SegmentLength, 1.0f);
		for (int32 NodeIndex = LastDrivenNode + 1; NodeIndex < OutCenterline.Num(); ++NodeIndex)
		{
			OutCenterline[NodeIndex] = OutCenterline[NodeIndex - 1] - FVector::UpVector * SegmentLength;
		}
	}

	return OutCenterline.Num() >= 2;
}

#pragma endregion
