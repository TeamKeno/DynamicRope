// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrappingPhase.h"
// ResolveBindingWorld — 랩 바인딩(본/소켓/컴포넌트) 트랜스폼 해석의 단일 지점(seam A).
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"
// RopeMath::AnyTangentFromNormal (unity 빌드 중복 정의 방지)
#include "RopeMathHelpers.h"

namespace
{
	// 실전 동작은 composite 복구 계층을 모두 소진한 뒤 SingleBone fallback을 허용한다.
	// 실패 자체를 관찰해야 하는 한정 테스트에서만 일시적으로 true로 바꾼다.
	constexpr bool bCancelWrapOnCompositeFailureForTesting = false;
}

bool FRopeWrappingPhase::Begin(const FRopeSurfaceAnchor& LatchAnchor, const USceneComponent* Mesh, FName Bone,
	float Duration, const FRopeSimState& Sim, const FContext& Ctx)
{
	// 새 throw는 항상 복합 SDF부터 시작한다. 이 플래그는 같은 throw 안에서 복합 경로가 실패했을 때만
	// 켜지며, 다음 Begin까지 유지되어 두 번째 복합 시도와 무한 fallback을 막는다.
	State.bPathUsesSingleBoneFallback = false;
	State.PathBuildFailureReason.Reset();
	State.PathCompositeRelaxedRecoveryCount = 0;
	State.PathCompositeContinuityRecoveryCount = 0;
	State.PathCompositeProjectionFailureCount = 0;
	State.BoneName = Bone;
	State.Mesh = Mesh;
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
			Mesh ? *Mesh->GetName() : TEXT("None"), Sim.Num(), Sim.SegmentLength,
			Ctx.Colliders.Num());
		return false;
	}

	State.StableTime = 0.0f;
	State.LastStableFirstNode = State.FirstNode;
	State.LastStableLastNode = State.LastNode;
	State.LastStableAnchorCount = State.Anchors.Num();
	return true;
}

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
		for (int32 StepIndex = 0;
			StepIndex < StepBudget && State.Path.Num() < State.NumTailNodes;
			++StepIndex)
		{
			const int32 PathIndex = State.Path.Num();
			if (!AppendCompositeAnalyticHelixPathPoint(PathIndex, Sim, Ctx))
			{
				break;
			}

			if (!AppendWrappingAnchorFromPathPoint(PathIndex, Sim, Ctx))
			{
				FinishPathBuild(/*bFailed=*/true, TEXT("CompositeAnalyticHelixAnchorFailed"));
				break;
			}
		}

		if (State.Path.Num() >= State.NumTailNodes)
		{
			FinishPathBuild(/*bFailed=*/false);
		}
		return;
	}
	AdvanceSurfaceVectorFieldProgressiveWrapPath(StepBudget, Sim, Ctx);
}

void FRopeWrappingPhase::ApplyFrontMotion(const FRopeSimState& Sim, float DeltaTime, const FContext& Ctx, FRopeNodeOverrideFrame& OutFrame)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_ApplyWrappingFrontMotion);

	AdvanceWrappingFront(DeltaTime, Sim, Ctx);

	const int32 LatchNode = State.LatchAnchor.NodeIndex;
	if (State.Anchors.Num() == 0 ||
		!Sim.Positions.IsValidIndex(LatchNode) ||
		!Sim.PrevPositions.IsValidIndex(LatchNode))
	{
		return;
	}

	FRopeWrapPathPoint FrontPoint;
	if (!SampleWrappingPath(State.FrontDistance, FrontPoint))
	{
		return;
	}

	const float SurfaceOffset = FMath::Max(0.0f, Ctx.SurfaceOffset);
	const auto PathPointToCenterline = [SurfaceOffset](const FRopeWrapPathPoint& Point)
	{
		// Surface point만 표면 법선 offset을 적용한다. Virtual/bridge는 이미 rope centerline의
		// 월드 위치이므로 offset을 다시 더하면 이상적인 나선 반지름이 이중으로 커진다.
		return Point.SurfaceWorld +
			(Point.bVirtual || Point.bBridge
				? FVector::ZeroVector
				: Point.NormalWorld * SurfaceOffset);
	};
	const FVector FrontWorld = PathPointToCenterline(FrontPoint);
	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	// front 구동 범위는 경로가 소유한 노드까지다. 상한 없는 기본 상태에서는 NumTailNodes가 로프
	// 끝까지라 종전과 동일하고, 감는 양 상한(WrappingMaxWrapAngleDeg)으로 경로가 로프보다 짧게
	// 마감되면 경로 밖 노드는 front 직선 연장으로 끌지 않는다 — 남는 로프는 마스크 동결로 제자리에
	// 있다가 커밋 후 자유 구간이 된다.
	int32 TailEndNode = Sim.Num() - 1;
	if (State.NumTailNodes > 0)
	{
		TailEndNode = FMath::Min(TailEndNode, LatchNode + State.NumTailNodes - 1);
	}
	// 시드 다중화: 경로 구동은 첫 보조 시드 노드 *앞*에서도 끝난다(NumTailNodes 클램프와 같은 경계).
	// 보조 노드는 아래에서 자기 본에 hold하고, 그 너머 남는 로프도 마찬가지로 동결 유지된다 —
	// front 직선 연장이 보조 대상 반대편으로 로프를 끌어가는 것을 막는다.
	for (const FRopeSurfaceAnchor& Secondary : State.SecondarySeedAnchors)
	{
		if (Secondary.NodeIndex > LatchNode)
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
			FRopeWrapPathPoint NodePoint;
			if (!SampleWrappingPath(NodeDistance, NodePoint))
			{
				continue;
			}
			World = PathPointToCenterline(NodePoint);
		}
		else
		{
			World = FrontWorld + FrontPoint.TangentWorld * (NodeDistance - State.FrontDistance);
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

void FRopeWrappingPhase::ApplyMassMask(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const
{
	const int32 LatchNode = State.LatchAnchor.NodeIndex;
	const bool bHasValidLatch = Sim.InvMass.IsValidIndex(LatchNode);
	const int32 DrivenEndNode = State.bPathEndedAtCompositeAxisLimit
		? FMath::Min(Sim.Num() - 1, LatchNode + FMath::Max(0, State.NumTailNodes - 1))
		: Sim.Num() - 1;

	OutFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		// Composite analytic helix가 island의 축 범위를 벗어나 부분 완료된 경우에는 실제로
		// 만들어진 경로까지만 Wrapping이 소유한다. 그 뒤 노드는 즉시 dynamic으로 돌려 solver가
		// 마지막 surface anchor에서 이어지는 자유 tail을 계산하게 한다.
		const int32 PathIndex = i - LatchNode;
		const bool bNoAnchorSolverNode =
			State.Path.IsValidIndex(PathIndex) && State.Path[PathIndex].bVirtual;
		const bool bWrappingDrivenNode =
			bHasValidLatch && i >= LatchNode && i <= DrivenEndNode && !bNoAnchorSolverNode;
		OutFrame.SetInvMass(i, (bStartPin || bWrappingDrivenNode) ? 0.0f : 1.0f);
	}
}

void FRopeWrappingPhase::UpdateStability(float DeltaTime)
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
	const bool bFrontDone = State.FrontDistance + KINDA_SMALL_NUMBER >= CommitFrontDistance;
	const bool bMotionDone = State.Elapsed >= State.Duration + MaxTailDelay;
	const bool bTimedOutWithAnchors =
		Config.WrappingMaxSettleTime > 0.0f &&
		State.Elapsed >= Config.WrappingMaxSettleTime;
	const bool bPathReadyToCommit =
		!State.bPathBuildActive ||
		State.bPathBuildComplete ||
		State.bPathBuildFailed;

	return bHasEnoughAnchors && bPathReadyToCommit && bFrontDone && (bMotionDone || bTimedOutWithAnchors);
}

bool FRopeWrappingPhase::ShouldAbortFailedShortWrap(const FRopeSimState& Sim, const FContext& Ctx,
	float MinRequiredAngleDeg, float& OutAngleDeg) const
{
	OutAngleDeg = 0.0f;
	if (MinRequiredAngleDeg <= 0.0f || !State.bPathBuildFailed)
	{
		return false;
	}

	if (!ComputeWrappedAngleAtLastBuiltPoint(Sim, Ctx, OutAngleDeg))
	{
		return false;
	}

	return OutAngleDeg < MinRequiredAngleDeg;
}

FRopeWrapState FRopeWrappingPhase::BuildCommitSeed(const FRopeSimState& Sim, const USceneComponent* Mesh) const
{
	FRopeWrapState Seed;
	Seed.BoneName = State.BoneName;
	Seed.Mesh = Mesh;

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

void FRopeWrappingPhase::ReturnNodesToSolver(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const
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

bool FRopeWrappingPhase::BuildPreviewCenterline(const FRopeSurfaceAnchor& LatchAnchor, const USceneComponent* Mesh, FName Bone,
	const FRopeSimState& Sim, const FContext& Ctx, TArray<FVector>& OutCenterline) const
{
	OutCenterline.Reset();
	if (!Mesh || Bone.IsNone() || !Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) || Sim.Num() < 2)
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
	if (!PreviewPhase.Begin(LatchAnchor, Mesh, Bone,
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
		OutCenterline[NodeIndex] = Point.SurfaceWorld + Point.NormalWorld * SurfaceOffset;
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

bool FRopeWrappingPhase::BeginProgressiveWrapPathBuild(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_BeginProgressiveWrapPathBuild);

	const USceneComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		Mesh = LatchAnchor.Mesh.Get();
	}
	if (!Mesh || !Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) || LatchAnchor.Bone.IsNone())
	{
		State.PathBuildFailureReason = TEXT("InvalidLatchInput");
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
	State.PathAccumulatedAngleRad = 0.0f;
	State.PathBridgeDistance = 0.0f;
	State.FrontDistance = 0.0f;
	State.PathCurrentBone = StoredLatchAnchor.Bone;
	State.PathPreviousBone = NAME_None;
	State.PathCurrentMesh = Mesh;
	State.PathDistanceSinceBoneTransition = 0.0f;
	State.FirstNode = TNumericLimits<int32>::Max();
	State.LastNode = INDEX_NONE;

	if (State.NumTailNodes <= 0)
	{
		State.PathBuildFailureReason = TEXT("NoTailNodesAfterLatch");
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
		State.PathBuildFailureReason = TEXT("InitialSurfacePathPointFailed");
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Path initialization failed: reason=InitialSurfacePathPointFailed bone=%s node=%d surface=%s normal=%s"),
			*Ctx.OwnerName, *StoredLatchAnchor.Bone.ToString(), StoredLatchAnchor.NodeIndex,
			*State.PathSurfaceWorld.ToString(), *State.PathNormalWorld.ToString());
		return false;
	}
	if (!AppendWrappingAnchorFromPathPoint(0, Sim, Ctx))
	{
		State.PathBuildFailureReason = TEXT("InitialAnchorBuildFailed");
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Path initialization failed: reason=InitialAnchorBuildFailed bone=%s node=%d pathPoints=%d surface=%s"),
			*Ctx.OwnerName, *StoredLatchAnchor.Bone.ToString(), StoredLatchAnchor.NodeIndex,
			State.Path.Num(), *State.PathSurfaceWorld.ToString());
		return false;
	}

	return true;
}

bool FRopeWrappingPhase::AppendCompositeAnalyticHelixPathPoint(
	int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_AppendCompositeAnalyticHelixPathPoint);

	if (PathIndex <= 0 || PathIndex >= State.NumTailNodes ||
		PathIndex != State.Path.Num() || !State.bPathUsesPoseSpaceIsland ||
		State.PathWrapIslandBones.Num() <= 1 || State.Path.Num() == 0)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("CompositeAnalyticHelixInvalidState"));
		return false;
	}

	const USceneComponent* IslandMesh = State.LatchAnchor.Mesh.Get();
	if (!IslandMesh)
	{
		IslandMesh = State.Mesh.Get();
	}
	if (!IslandMesh)
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("CompositeAnalyticHelixMissingMesh"));
		return false;
	}

	// 기존 AnalyticHelix의 5~8단계를 그대로 PathIndex 기반으로 계산한다. 축/래치 radial은
	// composite 초기화에서 한 번 확정된 값을 사용하며, 직전 projection 결과는 어떤 입력에도 쓰지 않는다.
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
	const float DistanceFromLatch = static_cast<float>(PathIndex) * SegmentLength;
	const float PitchScale = State.PathCompositeHelixPitchScale;
	const float LengthScale = FMath::Sqrt(1.0f + FMath::Square(PitchScale));

	// 래치 표면 반지름에서 island 전체 반지름으로 한 step에 순간 이동하지 않는다. 각 PathIndex의
	// ideal point는 직전 projection 결과와 무관하게 래치/축/config만으로 다시 계산하되, 반지름 변화가
	// segment의 60% 이하가 되도록 entry step 수를 자동 산출한다. 남은 길이만 원주/축 진행에 써서
	// ideal point 간격 자체도 SegmentLength에 가깝게 유지한다.
	const float RadiusDelta = FMath::Abs(HelixRadius - LatchRadius);
	const int32 RadiusEntrySegmentCount = RadiusDelta > KINDA_SMALL_NUMBER
		? FMath::Max(2, FMath::CeilToInt(RadiusDelta / (SegmentLength * 0.6f)))
		: 0;
	float IdealRadius = LatchRadius;
	float AxisAdvance = 0.0f;
	float AngleRadians = 0.0f;
	FVector IdealHelixWorld = LatchSurfaceWorld;
	FVector PreviousIdealHelixWorld = LatchSurfaceWorld;
	for (int32 IdealStepIndex = 1; IdealStepIndex <= PathIndex; ++IdealStepIndex)
	{
		PreviousIdealHelixWorld = IdealHelixWorld;
		const float PreviousRadius = IdealRadius;
		const float RadiusAlpha = RadiusEntrySegmentCount > 0
			? FMath::Clamp(static_cast<float>(IdealStepIndex) /
				static_cast<float>(RadiusEntrySegmentCount), 0.0f, 1.0f)
			: 1.0f;
		IdealRadius = FMath::Lerp(LatchRadius, HelixRadius, RadiusAlpha);
		const float RadialStep = IdealRadius - PreviousRadius;
		const float CircumferenceStep = FMath::Sqrt(FMath::Max(
			0.0f, FMath::Square(SegmentLength) - FMath::Square(RadialStep))) /
			FMath::Max(LengthScale, KINDA_SMALL_NUMBER);
		AxisAdvance += CircumferenceStep * PitchScale;
		const float MeanRadius = FMath::Max(
			(PreviousRadius + IdealRadius) * 0.5f, KINDA_SMALL_NUMBER);
		AngleRadians += State.PathWindingSign * CircumferenceStep / MeanRadius;

		const FVector StepRadial = FQuat(AxisDirection, AngleRadians)
			.RotateVector(LatchRadial)
			.GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);
		const FVector StepAxisPoint = State.PathAxisOrigin + AxisDirection *
			(LatchAxisDistance + AxisAdvance);
		IdealHelixWorld = StepAxisPoint + StepRadial * IdealRadius;
	}
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
		State.bPathEndedAtCompositeAxisLimit = true;
		State.NumTailNodes = State.Path.Num();
		FinishPathBuild(/*bFailed=*/false);
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Composite analytic helix reached island axial limit: "
				"nextIndex=%d idealAxis=%.2fcm range=[%.2f,%.2f]cm "
				"builtPoints=%d releasedSolverNodes=%d"),
			*Ctx.OwnerName, PathIndex, IdealAxisDistance,
			State.PathCompositeAxisMinDistance, State.PathCompositeAxisMaxDistance,
			State.Path.Num(), FMath::Max(0, PreviousTailNodeCount - State.NumTailNodes));
		return false;
	}

	// Ideal helix point에서 같은 축 높이의 axis point를 향해 radial ray를 쏜다. 각 SDF의
	// 첫 교차점 중 ray 시작점에 가장 가까운 것만 사용한다. outer band/후보 점수/직전 path
	// 방향은 전혀 사용하지 않으므로 각 PathIndex의 결과는 독립적인 analytic helix 위상에만
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

	FRopeWrapPathPoint Point;
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

		Point.SurfaceWorld = BestProjection.SurfacePoint;
		Point.NormalWorld = NormalWorld;
		Point.TangentWorld = TangentWorld;
		Point.Bone = BestProjection.Bone.IsNone() ? BestBone : BestProjection.Bone;
		Point.Mesh = BestProjection.SourceMesh ? BestProjection.SourceMesh : BestMesh;
	}
	else
	{
		++State.PathCompositeProjectionFailureCount;
		Point.SurfaceWorld = IdealHelixWorld;
		Point.NormalWorld = RotatedRadial;
		Point.TangentWorld = IdealTangentWorld;
		Point.Bone = NAME_None;
		Point.Mesh = IslandMesh;
		Point.bVirtual = true;
	}
	Point.DistanceFromLatch = DistanceFromLatch;
	Point.bBridge = false;
	State.Path.Add(Point);

	if (!Point.bVirtual)
	{
		State.PathPreviousBone = State.PathCurrentBone;
		State.PathCurrentBone = Point.Bone;
		State.PathCurrentMesh = Point.Mesh;
	}
	State.PathSurfaceWorld = Point.SurfaceWorld;
	State.PathNormalWorld = Point.NormalWorld;
	State.PathTangentWorld = Point.TangentWorld;
	State.PathCircumferenceDir = CircumferenceDirection;
	State.PathCurrentDistance = DistanceFromLatch;
	State.PathSweepDistance = DistanceFromLatch;
	State.PathCompositeSweepRadial = RotatedRadial;
	State.PathCompositeSweepAngleRad = FMath::Abs(AngleRadians);
	State.PathAccumulatedAngleRad = FMath::Abs(AngleRadians);

	UE_LOG(LogRopeWrap, Log,
		TEXT("[%s] Composite analytic helix point: index=%d type=%s bone=%s "
			"helixRadius=%.2fcm idealRadius=%.2fcm pitch=%.3f entrySegments=%d "
			"radialHitDistance=%.2fcm surfaceDistance=%.2fcm "
			"angle=%.1fdeg idealAxis=%.2fcm surfaceAxis=%.2fcm "
			"ideal=%s selected=%s sdfCandidates=%d radialMiss=%d"),
		*Ctx.OwnerName, PathIndex, Point.bVirtual ? TEXT("NoAnchorSolver") : TEXT("SurfaceAnchor"),
		*Point.Bone.ToString(), HelixRadius, IdealRadius, PitchScale,
		RadiusEntrySegmentCount,
		bFound ? BestRadialHitDistance : -1.0f,
		bFound ? BestProjection.Distance : -1.0f,
		FMath::RadiansToDegrees(FMath::Abs(AngleRadians)), IdealAxisDistance,
		ProjectedAxisDistance, *IdealHelixWorld.ToString(),
		*(bFound ? BestProjection.SurfacePoint : Point.SurfaceWorld).ToString(),
		MatchingSDFCount, RadialMissCount);

	if (State.Path.Num() >= State.NumTailNodes)
	{
		FinishPathBuild(/*bFailed=*/false);
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
		TEXT("[%s] Wrap algorithm fallback: from=CompositeSDF to=SingleBone reason=%s "
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

	const USceneComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = State.Mesh.Get();
	}
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
	// winding 기준 방향: 기본은 latch tangent(로프가 누운 방향). TravelPlaneFirst에서 캡처 속도가
	// 있으면 속도를 쓴다 — 감기 시작 방향이 "로프가 실제로 움직이던 쪽"과 일치해, 충돌 프레임의
	// tangent 노이즈에 흔들리지 않는다(진행 방향 기반 wrap 3단계).
	FVector WindingReference = LatchTangentWorld;
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::TravelPlaneFirst &&
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
	State.PathCurrentBone = LatchAnchor.Bone;
	State.PathPreviousBone = NAME_None;
	State.PathCurrentMesh = Mesh;
	State.PathDistanceSinceBoneTransition = 0.0f;
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
	State.bPathEndedAtCompositeAxisLimit = false;
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
						const float RadiusDelta = FMath::Abs(
							State.PathCompositeHelixRadius - CurrentSurfaceRadius);
						const int32 RadiusEntrySegmentCount = RadiusDelta > KINDA_SMALL_NUMBER
							? FMath::Max(2, FMath::CeilToInt(
								RadiusDelta / (SegmentLength * 0.6f)))
							: 0;
						float PreviousPlannedRadius = CurrentSurfaceRadius;
						for (int32 StepIndex = 1;
							StepIndex <= PlannedPathSegmentCount; ++StepIndex)
						{
							const float RadiusAlpha = RadiusEntrySegmentCount > 0
								? FMath::Clamp(static_cast<float>(StepIndex) /
									static_cast<float>(RadiusEntrySegmentCount), 0.0f, 1.0f)
								: 1.0f;
							const float PlannedRadius = FMath::Lerp(
								CurrentSurfaceRadius, State.PathCompositeHelixRadius, RadiusAlpha);
							const float RadialStep = PlannedRadius - PreviousPlannedRadius;
							PitchPlannedBaseTravel += FMath::Sqrt(FMath::Max(
								0.0f, FMath::Square(SegmentLength) - FMath::Square(RadialStep)));
							PreviousPlannedRadius = PlannedRadius;
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

			State.PathTangentWorld =
				(State.PathCircumferenceDir + AxisDirection *
					State.PathCompositeHelixPitchScale)
				.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir);
			State.PathTangentWorld = (State.PathTangentWorld - FVector::DotProduct(
				State.PathTangentWorld, State.PathNormalWorld) * State.PathNormalWorld)
				.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir);

			UE_LOG(LogRopeWrap, Log,
				TEXT("[%s] Composite sweep initialized: crossSectionContributors=%d "
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
	LatchPoint.Bone = State.PathCurrentBone;
	LatchPoint.Mesh = State.PathCurrentMesh;
	LatchPoint.DistanceFromLatch = 0.0f;
	State.Path.Add(LatchPoint);
	State.PathCurrentDistance = 0.0f;
	State.PathSweepDistance = 0.0f;

	if (State.Path.Num() >= State.NumTailNodes)
	{
		FinishPathBuild(/*bFailed=*/false);
	}

	return true;
}

bool FRopeWrappingPhase::AdvanceSurfaceVectorFieldProgressiveWrapPath(int32 StepBudget, const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_AdvanceSurfaceVectorFieldProgressivePath);

	const USceneComponent* Mesh = State.LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = State.Mesh.Get();
	}
	if (!Mesh || State.LatchAnchor.Bone.IsNone())
	{
		FinishPathBuild(/*bFailed=*/true, TEXT("AdvanceMissingMeshOrLatchBone"));
		return false;
	}

	const float StepSize = FMath::Max(1.0f, Sim.SegmentLength * 0.5f);
	int32 StepsRemaining = FMath::Max(1, StepBudget);

	// 현재 축 기준 radial(축에서 점으로 향하는 단위벡터). 스텝 전/후 radial 사이 각도가 그 스텝의
	// 감싼 각도 증분이다 — 점이 축 위(축퇴)면 false.
	const auto ComputeAxisRadial = [this](const FVector& Point, FVector& OutRadial) -> bool
	{
		const float AxisDistance = FVector::DotProduct(Point - State.PathAxisOrigin, State.PathAxisDirection);
		OutRadial = Point - (State.PathAxisOrigin + State.PathAxisDirection * AxisDistance);
		return OutRadial.Normalize(KINDA_SMALL_NUMBER);
	};

	while (StepsRemaining > 0 && State.Path.Num() < State.NumTailNodes)
	{
		const int32 PathIndex = State.Path.Num();
		bool bConsumedStep = false;

		// 한 번의 predictor/projection을 실제 centerline integration segment 하나로 취급한다.
		// 노드 생성 여부와 관계없이 매 outer iteration에서 정확히 한 step을 소비한다.
		while (StepsRemaining > 0 && !bConsumedStep)
		{
			const float StepDistance = StepSize;

			const bool bCompositeSweep = State.bPathUsesPoseSpaceIsland;
			const FVector PreviousSurfaceWorld = State.PathSurfaceWorld;
			const FVector PreviousNormalWorld = State.PathNormalWorld;
			const FVector PreviousTangentWorld = State.PathTangentWorld;
			const bool bPreviousPointWasBridge = State.PathBridgeDistance > 0.0f;
			FVector StepRadialBefore = FVector::ZeroVector;
			const bool bHasRadialBefore = ComputeAxisRadial(PreviousSurfaceWorld, StepRadialBefore);
			FVector DesiredSweepRadial = State.PathCompositeSweepRadial;
			float SweepStepAngleRad = 0.0f;
			FVector SupportProbeWorld = PreviousSurfaceWorld;
			FVector PathPredictorWorld = PreviousSurfaceWorld;

			if (bCompositeSweep)
			{
				// 복합 외곽의 진행 좌표는 개별 SDF tangent와 분리한다. 현재 축 반지름으로 이번 거리의
				// 각도 증분을 계산하고 2~12도로 제한해, 큰 대상에서는 충분히 전진하면서 작은 팔에서
				// 한 step에 반대편으로 건너뛰지 않게 한다.
				const FVector AxisDirection = State.PathAxisDirection.GetSafeNormal(
					KINDA_SMALL_NUMBER, FVector::UpVector);
				const float CurrentAxisDistance = FVector::DotProduct(
					PreviousSurfaceWorld - State.PathAxisOrigin, AxisDirection);
				const FVector CurrentAxisPoint =
					State.PathAxisOrigin + AxisDirection * CurrentAxisDistance;
				const float CurrentSurfaceRadius = FMath::Max(
					FVector::Dist(PreviousSurfaceWorld, CurrentAxisPoint), Sim.SegmentLength);
				SweepStepAngleRad = FMath::Clamp(
					StepDistance / CurrentSurfaceRadius,
					FMath::DegreesToRadians(2.0f),
					FMath::DegreesToRadians(12.0f));
				DesiredSweepRadial = FQuat(
					AxisDirection, SweepStepAngleRad * State.PathWindingSign)
					.RotateVector(State.PathCompositeSweepRadial)
					.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCompositeSweepRadial);

				const float PitchScale = Ctx.Config.WrappingHelixPitchScale;
				const float PitchRatio = PitchScale / FMath::Sqrt(1.0f + FMath::Square(PitchScale));
				const float NextAxisDistance = CurrentAxisDistance + StepDistance * PitchRatio;
				const FVector NextAxisPoint =
					State.PathAxisOrigin + AxisDirection * NextAxisDistance;
				const float ProbeRadius = FMath::Max(
					State.PathCompositeProbeRadius,
					CurrentSurfaceRadius + Sim.SegmentLength);
				PathPredictorWorld = NextAxisPoint + DesiredSweepRadial * CurrentSurfaceRadius;
				SupportProbeWorld = NextAxisPoint + DesiredSweepRadial * ProbeRadius;
				State.PathCircumferenceDir = FVector::CrossProduct(
					AxisDirection, DesiredSweepRadial)
					.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir) *
					State.PathWindingSign;
				State.PathTangentWorld =
					(PathPredictorWorld - PreviousSurfaceWorld).GetSafeNormal(
						KINDA_SMALL_NUMBER, State.PathCircumferenceDir);
			}
			else
			{
				State.PathTangentWorld = ComputeSurfaceVectorFieldTangent(
					State.PathAxisOrigin,
					State.PathAxisDirection,
					State.PathLatchRadial,
					State.PathWindingSign,
					PreviousSurfaceWorld,
					State.PathNormalWorld,
					Ctx,
					State.PathCircumferenceDir);
				PathPredictorWorld = PreviousSurfaceWorld + State.PathTangentWorld * StepDistance;
				SupportProbeWorld = PathPredictorWorld;
			}

			// 아래의 스냅 거리/브리지 코드는 State.PathSurfaceWorld를 이번 step의 경로 predictor로 읽는다.
			// composite의 외부 support probe는 별도 변수라 허공 브리지 지점으로 절대 사용하지 않는다.
			State.PathSurfaceWorld = PathPredictorWorld;
			const FName CurrentBone = State.PathCurrentBone.IsNone()
				? State.LatchAnchor.Bone
				: State.PathCurrentBone;
			const USceneComponent* ProjectedMesh = State.PathCurrentMesh.Get();
			if (!ProjectedMesh)
			{
				ProjectedMesh = Mesh;
			}

			// 단일/구형 경로는 tangent predictor에서 projection하고, 복합 island는 별도의 외부 support
			// probe에서 projection한다. 두 경우 모두 실제 rope node 위치를 tie-break에 넣어, 외곽 support가
			// 거의 같은 후보 사이에서 현재 로프 몸체와 가까운 표면을 고른다.
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
			ERopeCompositeSupportTier CompositeSupportTier = ERopeCompositeSupportTier::Failed;
			if (bCompositeSweep)
			{
				bOnSurface = ProjectWrapPointToCompositeIsland(Sim, Ctx, RopeNodeWorld,
					SupportProbeWorld, DesiredSweepRadial, PreviousSurfaceWorld,
					State.PathNormalWorld, State.PathTangentWorld,
					ProjectedSurface, ProjectedNormal, ProjectedTangent,
					ProjectedCircumference, ProjectedBone, ProjectedMesh, &CompositeSupportTier);
				if (CompositeSupportTier == ERopeCompositeSupportTier::Relaxed)
				{
					++State.PathCompositeRelaxedRecoveryCount;
				}
				else if (CompositeSupportTier == ERopeCompositeSupportTier::Continuity)
				{
					++State.PathCompositeContinuityRecoveryCount;
				}
				else if (CompositeSupportTier == ERopeCompositeSupportTier::Failed)
				{
					++State.PathCompositeProjectionFailureCount;
				}
			}
			else if (State.bPathUsesSingleBoneFallback)
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

			float CompositeSelectedSupport = 0.0f;
			float CompositeSelectedAlignment = 0.0f;
			float CompositeSelectedRadius = 0.0f;
			float CompositeProjectionDistance = 0.0f;
			if (bCompositeSweep && bOnSurface)
			{
				const FVector AxisDirection = State.PathAxisDirection.GetSafeNormal(
					KINDA_SMALL_NUMBER, FVector::UpVector);
				const FVector SelectedOffset = ProjectedSurface - State.PathAxisOrigin;
				const FVector SelectedRadialOffset = SelectedOffset -
					AxisDirection * FVector::DotProduct(SelectedOffset, AxisDirection);
				CompositeSelectedRadius = SelectedRadialOffset.Size();
				const FVector SelectedRadial = SelectedRadialOffset.GetSafeNormal(
					KINDA_SMALL_NUMBER, DesiredSweepRadial);
				CompositeSelectedAlignment = FVector::DotProduct(
					SelectedRadial, DesiredSweepRadial);
				CompositeSelectedSupport = FVector::DotProduct(
					SelectedRadialOffset, DesiredSweepRadial);
				// SupportProbeWorld는 이제 방향만 나타내는 원거리 기준점이다. 실제 query와 경로 품질은
				// predictor에서 표면까지 이동한 거리로 기록해야 수치가 40~100cm로 과장되지 않는다.
				CompositeProjectionDistance = FVector::Dist(
					PathPredictorWorld, ProjectedSurface);
			}

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
				if (bCompositeSweep)
				{
					UE_LOG(LogRopeWrap, Log,
						TEXT("[%s] Composite sweep projection stopped: sweep=%.1fdeg desiredRadial=%s "
							"probe=%s predictor=%s currentBone=%s probeRadius=%.2fcm recoveries(relaxed=%d continuity=%d failed=%d)"),
						*Ctx.OwnerName,
						FMath::RadiansToDegrees(State.PathCompositeSweepAngleRad + SweepStepAngleRad),
						*DesiredSweepRadial.ToString(), *SupportProbeWorld.ToString(),
						*PathPredictorWorld.ToString(), *CurrentBone.ToString(),
						State.PathCompositeProbeRadius, State.PathCompositeRelaxedRecoveryCount,
						State.PathCompositeContinuityRecoveryCount,
						State.PathCompositeProjectionFailureCount);
					if (bCancelWrapOnCompositeFailureForTesting)
					{
						// 테스트 중에는 부분 composite 경로를 커밋하거나 단일 본으로 재시도하지 않는다.
						// UpdateWrapping이 이 실패 상태를 보고 즉시 wrapping을 취소한다.
						FinishPathBuild(/*bFailed=*/true, TEXT("CompositeSupportRejectedByTestGate"));
						UE_LOG(LogRopeWrap, Warning,
							TEXT("[%s] CompositeSDF wrap cancelled by temporary test gate: "
								"reason=NoStrictCompositeSupport bone=%s path=%d/%d anchors=%d sweep=%.1fdeg"),
							*Ctx.OwnerName, *State.LatchAnchor.Bone.ToString(), State.Path.Num(),
							State.NumTailNodes, State.Anchors.Num(),
							FMath::RadiansToDegrees(State.PathCompositeSweepAngleRad));
						return false;
					}
					if (RestartPathBuildAsSingleBoneFallback(
						Sim, Ctx, TEXT("NoStrictCompositeSupport")))
					{
						// 기존 composite path/anchor는 helper가 모두 폐기했다. 새 단일 본 경로는
						// 다음 프레임 예산부터 latch에서 전진한다.
						return true;
					}
					return false;
				}
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
			const FVector PostStepPosition = bOnSurface ? ProjectedSurface : State.PathSurfaceWorld;
			FVector StepRadialAfter = FVector::ZeroVector;
			if (bHasRadialBefore && ComputeAxisRadial(PostStepPosition, StepRadialAfter))
			{
				State.PathAccumulatedAngleRad += FMath::Acos(FMath::Clamp(
					static_cast<float>(FVector::DotProduct(StepRadialBefore, StepRadialAfter)), -1.0f, 1.0f));
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
					State.PathPreviousBone = CurrentBone;
					State.PathCurrentBone = ProjectedBone;
					State.PathDistanceSinceBoneTransition = 0.0f;
					if (State.bPathUsesPoseSpaceIsland)
					{
						// 복합 island는 모든 표면을 하나의 기둥으로 보므로 본이 바뀌어도 축을 재해석하지 않는다.
						// 여기의 bone은 경로 상태가 아니라 최종 anchor 귀속 정보다.
						UE_LOG(LogRopeWrap, Log,
							TEXT("[%s] Composite wrap surface switched: from=%s to=%s islandBones=%d axis=fixed "
								"sweep=%.1fdeg support=%.2f radius=%.2f alignment=%.3f projection=%.2fcm"),
							*Ctx.OwnerName, *CurrentBone.ToString(), *ProjectedBone.ToString(),
							State.PathWrapIslandBones.Num(),
							FMath::RadiansToDegrees(State.PathCompositeSweepAngleRad + SweepStepAngleRad),
							CompositeSelectedSupport, CompositeSelectedRadius,
							CompositeSelectedAlignment, CompositeProjectionDistance);
					}
					else
					{
						// 구형 순차 전환 폴백만 새 본 형상 축으로 rolling axis를 재시드한다.
						ReseedWrappingAxisOnBoneTransition(ProjectedBone, ProjectedMesh, Ctx);
					}
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
					Point.SurfaceWorld =
						SampleCenterlineWorld - SampleNormalWorld * CenterlineOffset;
					Point.NormalWorld = SampleNormalWorld;
					Point.TangentWorld = SampleTangentWorld;
					Point.Bone = State.PathCurrentBone.IsNone()
						? State.LatchAnchor.Bone
						: State.PathCurrentBone;
					Point.Mesh = State.PathCurrentMesh.IsValid()
						? State.PathCurrentMesh.Get()
						: Mesh;
					Point.DistanceFromLatch = TargetArcDistance;
					// 브리지에서 출발하거나 이번 step이 브리지면 보간점도 허공 chord로 취급한다.
					Point.bBridge = bPreviousPointWasBridge || !bOnSurface;
					State.Path.Add(Point);
					AppendWrappingAnchorFromPathPoint(SamplePathIndex, Sim, Ctx);
				}
			}

			State.PathCurrentDistance = ArcEndDistance;
			State.PathSweepDistance += StepDistance;

			if (bCompositeSweep)
			{
				const float PreviousSweepAngleDeg = FMath::RadiansToDegrees(
					State.PathCompositeSweepAngleRad);
				State.PathCompositeSweepRadial = DesiredSweepRadial;
				State.PathCompositeSweepAngleRad += SweepStepAngleRad;
				const float NewSweepAngleDeg = FMath::RadiansToDegrees(
					State.PathCompositeSweepAngleRad);
				// 매 step 로그는 과도하므로 45도 경계를 지날 때만 실제 경로 각도와 선택 표면 상태를 남긴다.
				constexpr float SweepProgressLogIntervalDeg = 45.0f;
				const int32 PreviousLogBucket = FMath::FloorToInt(
					PreviousSweepAngleDeg / SweepProgressLogIntervalDeg);
				const int32 NewLogBucket = FMath::FloorToInt(
					NewSweepAngleDeg / SweepProgressLogIntervalDeg);
				if (NewLogBucket > PreviousLogBucket)
				{
					UE_LOG(LogRopeWrap, Log,
						TEXT("[%s] Composite sweep progress: sweep=%.1fdeg actualAngle=%.1fdeg "
							"bone=%s mode=%s support=%.2f radius=%.2f alignment=%.3f "
							"projection=%.2fcm pathDistance=%.2fcm radial=%s"),
						*Ctx.OwnerName, NewSweepAngleDeg,
						FMath::RadiansToDegrees(State.PathAccumulatedAngleRad),
						*State.PathCurrentBone.ToString(), bOnSurface ? TEXT("Surface") : TEXT("Bridge"),
						CompositeSelectedSupport, CompositeSelectedRadius,
						CompositeSelectedAlignment, CompositeProjectionDistance,
						State.PathCurrentDistance,
						*State.PathCompositeSweepRadial.ToString());
				}
			}

			--StepsRemaining;
			bConsumedStep = true;
		}

		if (!State.bPathUsesPoseSpaceIsland)
		{
			State.PathTangentWorld = ComputeSurfaceVectorFieldTangent(
				State.PathAxisOrigin,
				State.PathAxisDirection,
				State.PathLatchRadial,
				State.PathWindingSign,
				State.PathSurfaceWorld,
				State.PathNormalWorld,
				Ctx,
				State.PathCircumferenceDir);
		}

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
	const USceneComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		Mesh = LatchAnchor.Mesh.Get();
	}
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
	Anchor.StartWorldPosition = Sim.Positions[NodeIndex];
	Anchor.SurfaceOffset = FMath::Max(0.0f, Ctx.SurfaceOffset);
	Anchor.RopeDistance = Point.DistanceFromLatch;

	State.FirstNode = FMath::Min(State.FirstNode, NodeIndex);
	State.LastNode = FMath::Max(State.LastNode, NodeIndex);
	State.Anchors.Add(Anchor);
	State.LastAnchoredPathPointCount = PathIndex + 1;
	return true;
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

bool FRopeWrappingPhase::ComputeWrappedAngleAtLastBuiltPoint(const FRopeSimState& Sim, const FContext& Ctx, float& OutAngleDeg) const
{
	OutAngleDeg = 0.0f;
	if (State.Anchors.Num() == 0 && State.Path.Num() == 0)
	{
		return false;
	}

	// Sequential SurfaceVectorField와 Composite AnalyticHelix 모두 경로 빌드 중 누적한 실제 위상을
	// 우선 사용한다. 아직 한 스텝도 진행하지 못한 경로만 아래의 단일 축 근사로 폴백한다.
	if (State.PathAccumulatedAngleRad > KINDA_SMALL_NUMBER)
	{
		OutAngleDeg = FMath::RadiansToDegrees(State.PathAccumulatedAngleRad);
		return true;
	}

	const FRopeSurfaceAnchor& LatchAnchor = State.LatchAnchor;
	const USceneComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = State.Mesh.Get();
	}
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	FVector AxisOrigin = FVector::ZeroVector;
	FVector AxisDirection = FVector::ForwardVector;
	if (!ResolveWrappingAxis(LatchAnchor, Ctx, AxisOrigin, AxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, AxisDirection);

	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);
	const FVector LatchSurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
	const float LatchAxisDistance = FVector::DotProduct(LatchSurfaceWorld - AxisOrigin, AxisDirection);
	const FVector LatchAxisPoint = AxisOrigin + AxisDirection * LatchAxisDistance;
	const float HelixRadius = (LatchSurfaceWorld - LatchAxisPoint).Size();
	if (HelixRadius <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	float LastBuiltDistance = 0.0f;
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		LastBuiltDistance = FMath::Max(LastBuiltDistance, Anchor.RopeDistance);
	}
	if (State.Path.Num() > 0)
	{
		LastBuiltDistance = FMath::Max(LastBuiltDistance, State.Path.Last().DistanceFromLatch);
	}

	// 실제 SurfaceVectorField 경로가 얼마나 울퉁불퉁했는지와 별개로, 실패 판정은 helix 기준 누적
	// 감싼 각도만 본다. 각도(도) 반환 — 회전 수(=각도/360)는 2πr 로프를 요구해 대상 크기에 비례하는
	// 기준이 되므로 쓰지 않는다(FRopeWrapConfig::FailedWrapMinAngleDeg 주석 참고).
	if (State.bPathUsesPoseSpaceIsland && State.Path.Num() > 1)
	{
		// 자동 pitch와 진입 반지름을 사용한 독립 helix는 이미 실제 누적 위상을 저장한다.
		// 고정 config pitch로 다시 역산하면 커밋 로그/품질 관문의 각도가 경로와 달라진다.
		OutAngleDeg = FMath::RadiansToDegrees(
			FMath::Abs(State.PathCompositeSweepAngleRad));
		return true;
	}

	const float PitchScale = Ctx.Config.WrappingHelixPitchScale;
	const float LengthScale = FMath::Sqrt(1.0f + PitchScale * PitchScale);
	const float CircumferenceDistance = LastBuiltDistance / FMath::Max(LengthScale, KINDA_SMALL_NUMBER);
	const float AngleRadians = CircumferenceDistance / FMath::Max(HelixRadius, KINDA_SMALL_NUMBER);
	OutAngleDeg = FMath::RadiansToDegrees(FMath::Abs(AngleRadians));
	return true;
}

bool FRopeWrappingPhase::ComputeWrapEnclosureCoverage(float& OutCoverageDeg) const
{
	OutCoverageDeg = 0.0f;
	if (State.Path.Num() < 2)
	{
		return false;
	}

	const FVector Axis = State.PathAxisDirection.GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		return false;
	}

	const auto ComputeRadial = [this, &Axis](const FVector& Point, FVector& OutRadial) -> bool
	{
		const FVector Offset = Point - State.PathAxisOrigin;
		OutRadial = Offset - Axis * FVector::DotProduct(Offset, Axis);
		return OutRadial.Normalize(KINDA_SMALL_NUMBER);
	};

	// 각 경로점의 축 둘레 각도(첫 비축퇴 점 기준, (-180,180]). 브리지 점도 포함한다 —
	// chord가 가로지른 방향도 로프가 막고 있는 방향이다.
	FVector RefRadial = FVector::ZeroVector;
	TArray<float, TInlineAllocator<128>> AngleDegrees;
	for (const FRopeWrapPathPoint& Point : State.Path)
	{
		FVector Radial = FVector::ZeroVector;
		if (!ComputeRadial(Point.SurfaceWorld, Radial))
		{
			continue;
		}

		if (RefRadial.IsNearlyZero())
		{
			RefRadial = Radial;
		}

		const float AngleRad = FMath::Atan2(
			static_cast<float>(FVector::DotProduct(Axis, FVector::CrossProduct(RefRadial, Radial))),
			static_cast<float>(FVector::DotProduct(RefRadial, Radial)));
		AngleDegrees.Add(FMath::RadiansToDegrees(AngleRad));
	}

	if (AngleDegrees.Num() < 2)
	{
		return false;
	}

	// 정렬 후 최대 각도 공백(이웃 간 + 양끝 wrap-around)을 찾는다. 커버리지 = 360 − 최대 공백:
	// 점들이 축 둘레를 빈틈없이 두르면 공백이 스텝 각 수준으로 작아 360에 수렴하고,
	// 반쪽 훅이면 반대편이 통째로 비어 커버리지가 그만큼 낮다.
	AngleDegrees.Sort();
	float MaxGapDeg = 360.0f - (AngleDegrees.Last() - AngleDegrees[0]);
	for (int32 Index = 1; Index < AngleDegrees.Num(); ++Index)
	{
		MaxGapDeg = FMath::Max(MaxGapDeg, AngleDegrees[Index] - AngleDegrees[Index - 1]);
	}

	OutCoverageDeg = FMath::Clamp(360.0f - MaxGapDeg, 0.0f, 360.0f);
	return true;
}

bool FRopeWrappingPhase::FindColliderShapeAxis(const FContext& Ctx, FName Bone, const USceneComponent* Mesh,
	FVector& OutAxisOrigin, FVector& OutAxisDirection)
{
	if (Bone.IsNone())
	{
		return false;
	}

	for (const IRopeCollider* Collider : Ctx.Colliders)
	{
		if (!Collider)
		{
			continue;
		}
		FName ColliderBone = NAME_None;
		const USceneComponent* ColliderMesh = nullptr;
		Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
		// mesh까지 일치해야 한다(cross-actor: 다른 액터의 동명 본 오배정 방지). 귀속 미구현
		// collider(None/null)는 자연히 걸러진다. 같은 본에 셰이프가 여럿(피직스 에셋 멀티 셰이프)이면
		// 첫 매치를 쓴다 — 본당 주 셰이프가 먼저 빌드되는 provider 관례에 기댄 단순화.
		if (ColliderBone != Bone || (Mesh != nullptr && ColliderMesh != Mesh))
		{
			continue;
		}

		// 캡슐: 세그먼트가 곧 형상 축. 구(A≈B) 축퇴는 방향 정보가 없어 다음 폴백으로.
		FVector CapA, CapB;
		float CapRadius = 0.0f;
		if (Collider->GetGPUCapsule(CapA, CapB, CapRadius))
		{
			const FVector Axis = CapB - CapA;
			// 1cm 미만 세그먼트는 방향 신뢰 불가(사실상 구).
			if (Axis.SizeSquared() > 1.0f)
			{
				OutAxisOrigin = CapA;
				OutAxisDirection = Axis.GetSafeNormal();
				return true;
			}
			continue;
		}

		// 박스(정적 랩 가상 본 등): 최장 반변의 로컬 축을 회전시켜 축으로. origin = 박스 중심(축 위).
		FVector BoxCenter, BoxHalf;
		FQuat BoxRot;
		if (Collider->GetGPUBox(BoxCenter, BoxRot, BoxHalf))
		{
			FVector LocalAxis = FVector::XAxisVector;
			if (BoxHalf.Y > BoxHalf.X && BoxHalf.Y >= BoxHalf.Z)
			{
				LocalAxis = FVector::YAxisVector;
			}
			else if (BoxHalf.Z > BoxHalf.X && BoxHalf.Z > BoxHalf.Y)
			{
				LocalAxis = FVector::ZAxisVector;
			}
			OutAxisOrigin = BoxCenter;
			OutAxisDirection = BoxRot.RotateVector(LocalAxis);
			return true;
		}

		// SDF: 본 로컬 bounds의 최장축을 본 트랜스폼으로 월드에. origin = bounds 중심(월드).
		FRopeSDFColliderView SDFView;
		if (Collider->GetGPUSDF(SDFView))
		{
			const FVector Size = SDFView.LocalSize;
			FVector LocalAxis = FVector::XAxisVector;
			if (Size.Y > Size.X && Size.Y >= Size.Z)
			{
				LocalAxis = FVector::YAxisVector;
			}
			else if (Size.Z > Size.X && Size.Z > Size.Y)
			{
				LocalAxis = FVector::ZAxisVector;
			}
			OutAxisOrigin = SDFView.BoneToWorld.TransformPosition(SDFView.LocalMin + Size * 0.5);
			OutAxisDirection = SDFView.BoneToWorld.TransformVectorNoScale(LocalAxis)
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
			return true;
		}
	}
	return false;
}

bool FRopeWrappingPhase::FindGuidePlaneAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx,
	const USceneComponent* Mesh, FVector& OutAxisOrigin, FVector& OutAxisDirection)
{
	if (!Mesh || LatchAnchor.Bone.IsNone() || !Ctx.bHasGuidePlaneNormal)
	{
		return false;
	}

	const FVector GuidePlaneNormal = Ctx.GuidePlaneNormal.GetSafeNormal();
	if (GuidePlaneNormal.IsNearlyZero())
	{
		return false;
	}

	// TravelPlaneFirst: 축 origin을 latch 본 위치가 아니라 캡처 순간의 접촉 영역 중심에 둔다 —
	// 여러 본/대상에 걸친 접촉(양다리)에서 감김 반경이 한쪽 대상이 아닌 쌍의 중심을 기준으로 잡힌다.
	// origin은 캡처 시점 고정값이라 본 전환 재시드(rolling axis)에서도 움직이지 않는다.
	// ShapeAxisFirst의 폴백 경로(RopePlaneNormal)는 종전대로 본 위치를 쓴다 — 기본 동작 불변.
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::TravelPlaneFirst &&
		Ctx.TravelFrame && Ctx.TravelFrame->bValid)
	{
		FVector Origin = Ctx.TravelFrame->RegionCenter;

		// 접촉 군집 보정(브리징 = 분리 대상 랩 모드에서만): RegionCenter는 "첫 접촉" 순간의 접촉점
		// 평균이라, 캡처가 첫 다리에 닿는 즉시 일어나면(MinLatchNodes 기본 1) 한쪽 다리 위에 있다 —
		// 축이 대상 안을 지나면 그 대상만 도는 궤도가 winding 순방향이 되어 이탈 관문이 침묵하고,
		// 반대쪽 다리로 못 건너간다(PIE 실측 2026-07-13: 한 다리 1725° 나선, coverage는 축이 대상
		// 안이라 무의미하게 높음). 접촉 못 한 이웃 대상도 collider 스냅샷에는 있으므로, 같은 mesh의
		// 근방(브리지 거리) collider 중심들을 평균해 축이 군집(양다리 쌍)의 중심을 지나게 한다.
		// 축 방향 성분은 버린다 — 축은 선이라 수직 성분만 의미가 있다.
		const float ClusterRadius = Ctx.Config.WrappingMaxGapBridgeDistance;
		if (ClusterRadius > 0.0f)
		{
			FVector CenterSum = FVector::ZeroVector;
			int32 CenterCount = 0;
			for (const IRopeCollider* Collider : Ctx.Colliders)
			{
				if (!Collider)
				{
					continue;
				}

				FName ColliderBone = NAME_None;
				const USceneComponent* ColliderMesh = nullptr;
				Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
				if (Mesh != nullptr && ColliderMesh != nullptr && ColliderMesh != Mesh)
				{
					continue;
				}

				FVector Center = FVector::ZeroVector;
				if (!GetColliderCenter(*Collider, Center))
				{
					continue;
				}

				const FVector Delta = Center - Origin;
				const float AlongAxis = static_cast<float>(FVector::DotProduct(Delta, GuidePlaneNormal));
				if (FMath::Abs(AlongAxis) > ClusterRadius ||
					(Delta - GuidePlaneNormal * AlongAxis).Size() > ClusterRadius)
				{
					continue;
				}

				CenterSum += Center;
				++CenterCount;
			}

			if (CenterCount > 0)
			{
				const FVector ClusterDelta = CenterSum / static_cast<float>(CenterCount) - Origin;
				Origin += ClusterDelta - GuidePlaneNormal *
					static_cast<float>(FVector::DotProduct(ClusterDelta, GuidePlaneNormal));
			}
		}

		OutAxisOrigin = Origin;
	}
	else
	{
		OutAxisOrigin = ResolveBindingWorld(Mesh, LatchAnchor.Bone).GetLocation();
	}
	OutAxisDirection = GuidePlaneNormal;
	return true;
}

bool FRopeWrappingPhase::GetColliderCenter(const IRopeCollider& Collider, FVector& OutCenter)
{
	FVector CapA = FVector::ZeroVector;
	FVector CapB = FVector::ZeroVector;
	float CapRadius = 0.0f;
	if (Collider.GetGPUCapsule(CapA, CapB, CapRadius))
	{
		OutCenter = (CapA + CapB) * 0.5f;
		return true;
	}

	FVector BoxCenter = FVector::ZeroVector;
	FVector BoxHalf = FVector::ZeroVector;
	FQuat BoxRot = FQuat::Identity;
	if (Collider.GetGPUBox(BoxCenter, BoxRot, BoxHalf))
	{
		OutCenter = BoxCenter;
		return true;
	}

	FRopeSDFColliderView SDFView;
	if (Collider.GetGPUSDF(SDFView))
	{
		OutCenter = SDFView.BoneToWorld.TransformPosition(SDFView.LocalMin + SDFView.LocalSize * 0.5);
		return true;
	}

	return false;
}

/* 감김 축 정의 — 우선순위/근거는 헤더 주석 참고(형상 축 → rope 평면 가상축 → 컴포넌트 기저 → 로컬 X). */
bool FRopeWrappingPhase::ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx,
	FVector& OutAxisOrigin, FVector& OutAxisDirection) const
{
	const auto LogAxisSource = [&](const TCHAR* Source, const USceneComponent* MeshForLog)
	{
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Wrapping axis: source=%s, bone=%s, mesh=%s, origin=%s, dir=%s"),
			*Ctx.OwnerName,
			Source,
			*LatchAnchor.Bone.ToString(),
			*GetNameSafe(MeshForLog),
			*OutAxisOrigin.ToString(),
			*OutAxisDirection.ToString());
	};

	// 0) TravelPlaneFirst(설정): 로프 진행(스윙) 평면 normal 축을 형상 축보다 앞세운다. 여러 본에
	//    걸친 랩(양다리)이 특정 본 하나의 형상 축에 끌려가지 않게 하는 진행 방향 기반 wrap의 1단계.
	//    (CL 341이 형상 축을 주석 토글로 껐다 켰다 하던 실험의 정식화 — 기본값 ShapeAxisFirst는
	//    기존 우선순위 그대로다.) 가이드 평면이 없으면 아래 체인으로 자연 폴백.
	bool bTriedTravelPlane = false;
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::TravelPlaneFirst)
	{
		bTriedTravelPlane = true;
		const USceneComponent* TravelMesh = LatchAnchor.Mesh.Get();
		if (!TravelMesh)
		{
			TravelMesh = State.Mesh.Get();
		}
		if (TravelMesh && FindGuidePlaneAxis(LatchAnchor, Ctx, TravelMesh, OutAxisOrigin, OutAxisDirection))
		{
			LogAxisSource(TEXT("TravelPlaneAxis"), TravelMesh);
			return true;
		}
	}

	// 1) collider 형상 축: 실제 충돌 지오메트리의 장축 — 본 그래프 특성(짧은 몸통 본, 체인 본,
	//    임포트 축)과 무관하게 맞고, origin이 지오메트리 중심축 위라 helix 반지름도 정확하다.
	//if (FindColliderShapeAxis(Ctx, LatchAnchor.Bone, LatchAnchor.Mesh.Get(), OutAxisOrigin, OutAxisDirection))
	//{
	//	LogAxisSource(TEXT("ColliderShapeAxis"), LatchAnchor.Mesh.Get());
	//	return true;
	//}


	const USceneComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = State.Mesh.Get();
	}
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	const FName ParentBone = RopeWrapTargets::GetParentTargetKey(Mesh, LatchAnchor.Bone);
	const FVector BoneLocation = ResolveBindingWorld(Mesh, LatchAnchor.Bone).GetLocation();

	// 로프가 날아와 만든 spline guide 평면의 normal을 bone 위치에 세운 가상 축으로 쓴다.
	// (TravelPlaneFirst였다면 이미 위에서 시도·실패한 것이므로 재시도하지 않는다.)
	if (!bTriedTravelPlane && FindGuidePlaneAxis(LatchAnchor, Ctx, Mesh, OutAxisOrigin, OutAxisDirection))
	{
		LogAxisSource(TEXT("RopePlaneNormal"), Mesh);
		return true;
	}

	// rope 평면 축을 만들 수 없으면 스켈레탈 대상은 bone-parent 축으로 폴백한다.
	if (!ParentBone.IsNone())
	{
		const FVector ParentLocation = ResolveBindingWorld(Mesh, ParentBone).GetLocation();
		const FVector Axis = BoneLocation - ParentLocation;
		if (!Axis.IsNearlyZero())
		{
			OutAxisOrigin = ParentLocation;
			OutAxisDirection = Axis.GetSafeNormal();
			LogAxisSource(TEXT("BoneParentAxis"), Mesh);
			return true;
		}
	}

	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);

	// 정적/비-스켈레탈 대상(피드백 5): 본 그래프가 없어 축을 컴포넌트 기저에서 유도한다. 컴포넌트
	// 기저축(X/Y/Z) 중 latch 표면 normal에 가장 수직인 축을 감김 축으로 고른다 — 원기둥/캡슐의 장축은
	// 반경 방향(표면 normal)에 수직이므로, 축정렬 랩 캡슐(기둥=Z, 가로보=X/Y)에서 올바른 감김 축이
	// 자동 선택된다(부모가 있는 스켈레탈은 위에서 이미 반환됨 — 스켈레탈 루트 본은 아래 로컬 X 폴백).
	if (!RopeWrapTargets::IsSkeletalTarget(Mesh))
	{
		const FVector NormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		FVector BestAxis = BoneXform.GetUnitAxis(EAxis::Z);
		float BestParallel = TNumericLimits<float>::Max();
		for (const EAxis::Type CandidateAxis : { EAxis::X, EAxis::Y, EAxis::Z })
		{
			const FVector AxisWorld = BoneXform.GetUnitAxis(CandidateAxis);
			const float ParallelToNormal = FMath::Abs(FVector::DotProduct(AxisWorld, NormalWorld));
			if (ParallelToNormal < BestParallel)
			{
				BestParallel = ParallelToNormal;
				BestAxis = AxisWorld;
			}
		}
		OutAxisOrigin = BoneLocation;
		OutAxisDirection = BestAxis.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		LogAxisSource(TEXT("StaticBasisFallback"), Mesh);
		return true;
	}

	OutAxisOrigin = BoneLocation;
	OutAxisDirection = BoneXform.GetUnitAxis(EAxis::X).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	LogAxisSource(TEXT("BoneLocalXFallback"), Mesh);
	return true;
}

void FRopeWrappingPhase::OrientWrappingAxisByTail(const FRopeSurfaceAnchor& LatchAnchor, const FRopeSimState& Sim,
	const USceneComponent* Mesh, FVector& InOutAxisDirection) const
{
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return;
	}

	const FName ParentBone = RopeWrapTargets::GetParentTargetKey(Mesh, LatchAnchor.Bone);
	if (ParentBone.IsNone())
	{
		return;
	}

	float ParentScore = 0.0f;
	float BoneScore = 0.0f;
	float TotalWeight = 0.0f;
	const FVector ParentWorld = ResolveBindingWorld(Mesh, ParentBone).GetLocation();
	const FVector BoneWorld = ResolveBindingWorld(Mesh, LatchAnchor.Bone).GetLocation();

	const auto AddProbe = [&](int32 NodeIndex, float Weight)
	{
		if (!Sim.Positions.IsValidIndex(NodeIndex) || Weight <= 0.0f)
		{
			return;
		}

		const FVector ProbeWorld = Sim.Positions[NodeIndex];
		ParentScore += FVector::DistSquared(ProbeWorld, ParentWorld) * Weight;
		BoneScore += FVector::DistSquared(ProbeWorld, BoneWorld) * Weight;
		TotalWeight += Weight;
	};

	AddProbe(LatchAnchor.NodeIndex + 1, 2.0f);
	AddProbe(Sim.Num() - 1, 1.0f);

	if (TotalWeight <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	ParentScore /= TotalWeight;
	BoneScore /= TotalWeight;

	if (ParentScore < BoneScore)
	{
		InOutAxisDirection *= -1.0f;
	}
}

void FRopeWrappingPhase::ReseedWrappingAxisOnBoneTransition(FName Bone, const USceneComponent* Mesh, const FContext& Ctx)
{
	if (Bone.IsNone())
	{
		return;
	}

	// ResolveWrappingAxis의 입력 계약(본/메시 + 본 로컬 표면 프레임)만 채운 합성 anchor. 현재 경로
	// 지점의 표면 프레임을 새 본 로컬로 옮겨 담아, collider 형상 축 실패 시의 폴백(guide 평면/기저축)도
	// 새 본 위치·현재 normal 기준으로 동작하게 한다.
	FRopeSurfaceAnchor AxisAnchor;
	AxisAnchor.Bone = Bone;
	AxisAnchor.Mesh = Mesh;
	const FTransform BoneXform = ResolveBindingWorld(Mesh, Bone);
	AxisAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(State.PathSurfaceWorld);
	AxisAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(State.PathNormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	AxisAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(State.PathTangentWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);

	FVector NewAxisOrigin = State.PathAxisOrigin;
	FVector NewAxisDirection = State.PathAxisDirection;
	if (!ResolveWrappingAxis(AxisAnchor, Ctx, NewAxisOrigin, NewAxisDirection))
	{
		// 새 본 축 유도 실패 — 직전 본 축으로 계속 진행한다(종전 단일 축 동작과 동일한 폴백).
		return;
	}

	// 부호 정렬: 체인 본의 이웃 축은 대체로 이어지므로 이전 축과 반대면 뒤집는다 — 피치 드리프트
	// 방향(감기며 축을 따라 미끄러지는 쪽)이 전환점에서 반전되지 않게 한다. OrientWrappingAxisByTail은
	// latch 시점 tail 위치 휴리스틱이라 경로 중간 재해석에는 부적합하다.
	if (FVector::DotProduct(NewAxisDirection, State.PathAxisDirection) < 0.0f)
	{
		NewAxisDirection *= -1.0f;
	}

	// 현재 표면점 기준 radial/원주 재계산 + winding 재선출: 새 필드가 지금 진행 방향(tangent) 그대로
	// 새 축 주위를 돌게 한다. 여기서 winding을 다시 뽑지 않으면 전환 지점의 기하에 따라 감김 방향이
	// 뒤집힐 수 있다.
	const float AxisDistance = FVector::DotProduct(State.PathSurfaceWorld - NewAxisOrigin, NewAxisDirection);
	const FVector AxisPoint = NewAxisOrigin + NewAxisDirection * AxisDistance;
	const FVector Radial = (State.PathSurfaceWorld - AxisPoint)
		.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathNormalWorld);
	FVector CircumferenceDir = FVector::CrossProduct(NewAxisDirection, Radial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir);
	const float NewWindingSign =
		FVector::DotProduct(CircumferenceDir, State.PathTangentWorld) < 0.0f ? -1.0f : 1.0f;

	State.PathAxisOrigin = NewAxisOrigin;
	State.PathAxisDirection = NewAxisDirection;
	State.PathLatchRadial = Radial;
	State.PathWindingSign = NewWindingSign;
	State.PathCircumferenceDir = CircumferenceDir * NewWindingSign;

	UE_LOG(LogRopeWrap, Log,
		TEXT("[%s] Wrapping axis re-seeded on bone transition: bone=%s, origin=%s, dir=%s, winding=%+.0f"),
		*Ctx.OwnerName, *Bone.ToString(),
		*State.PathAxisOrigin.ToString(), *State.PathAxisDirection.ToString(), NewWindingSign);
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

		// SDF grid의 로컬 bounds와 당시 bone transform을 그대로 스냅샷한다. debug draw에서 이 값을
		// 다시 샘플링하거나 메시로 변환하지 않고 oriented box로만 표시한다.
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

bool FRopeWrappingPhase::ProjectWrapPointToCompositeIsland(const FRopeSimState& Sim,
	const FContext& Ctx, const FVector& RopeNodeWorld, const FVector& SupportProbeWorld,
	const FVector& DesiredSweepRadial, const FVector& PreviousSurfaceWorld,
	const FVector& PreviousNormalWorld,
	const FVector& PreviousTangentWorld, FVector& InOutSurfaceWorld,
	FVector& InOutNormalWorld, FVector& InOutTangentWorld,
	FVector& InOutCircumferenceDir, FName& InOutBone,
	const USceneComponent*& OutMesh, ERopeCompositeSupportTier* OutSupportTier) const
{
	if (OutSupportTier)
	{
		*OutSupportTier = ERopeCompositeSupportTier::Failed;
	}
	if (State.PathWrapIslandBones.Num() == 0)
	{
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Composite support rejected: reason=EmptyIsland currentBone=%s probe=%s predictor=%s"),
			*Ctx.OwnerName, *InOutBone.ToString(), *SupportProbeWorld.ToString(), *InOutSurfaceWorld.ToString());
		return false;
	}

	const USceneComponent* IslandMesh = State.LatchAnchor.Mesh.Get();
	if (!IslandMesh)
	{
		IslandMesh = State.Mesh.Get();
	}

	const FVector PreviousTangent = PreviousTangentWorld.GetSafeNormal(
		KINDA_SMALL_NUMBER, FVector::ForwardVector);
	const FVector PreviousNormal = PreviousNormalWorld.GetSafeNormal(
		KINDA_SMALL_NUMBER, FVector::UpVector);
	const FVector AxisDirection = State.PathAxisDirection.GetSafeNormal(
		KINDA_SMALL_NUMBER, FVector::UpVector);
	const FVector SupportProbeOffset = SupportProbeWorld - State.PathAxisOrigin;
	const FVector SupportProbeRadialOffset = SupportProbeOffset -
		AxisDirection * FVector::DotProduct(SupportProbeOffset, AxisDirection);
	const float SupportProbeRadius = SupportProbeRadialOffset.Size();
	const float QueryRadius = FMath::Max3(
		FMath::Max(Ctx.GetContactRadius(), Ctx.SurfaceOffset),
		Sim.SegmentLength * 3.0f,
		SupportProbeRadius * 2.0f + Sim.SegmentLength);

	// 복합 island는 선택된 개별 SDF 근처에서 다음 방향을 다시 유도하지 않는다. 호출자가 독립적으로
	// 회전시킨 DesiredSweepRadial을 support 방향으로 고정한다. SupportProbeWorld는 어느 면을 볼지 정하는
	// 공통 방향 기준일 뿐이다. 이 먼 점에서 SDF를 바로 투영하면 narrow band 바깥의 grid 경계에서
	// gradient가 0이 되어 첫 step부터 후보가 전멸할 수 있으므로, 실제 query는 collider별 bake bounds
	// 안쪽에서 시작한다.
	const FVector ExpectedRadial = DesiredSweepRadial.GetSafeNormal(
		KINDA_SMALL_NUMBER, State.PathLatchRadial);
	const FVector DesiredCircumference = FVector::CrossProduct(
		AxisDirection, ExpectedRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, InOutCircumferenceDir) * State.PathWindingSign;
	const FVector DesiredTangent =
		(DesiredCircumference + AxisDirection * Ctx.Config.WrappingHelixPitchScale)
		.GetSafeNormal(KINDA_SMALL_NUMBER, DesiredCircumference);
	const float ProbeAxisDistance = FVector::DotProduct(
		SupportProbeWorld - State.PathAxisOrigin, AxisDirection);
	const float MaxAxialProjectionDrift = FMath::Max3(
		Sim.SegmentLength * 1.5f, Ctx.GetContactRadius() * 2.0f, Ctx.SurfaceOffset * 2.0f);
	constexpr float MinSupportAlignment = 0.5f; // 목표 radial에서 60도 이내의 외곽만 사용.
	// 로프 표면 두께보다 작은 support 차이는 같은 외곽 band로 보고 기존 거리/연속성 점수로
	// tie-break한다. 이 허용폭보다 명확하게 바깥인 표면은 거리 점수가 다소 나빠도 우선한다.
	const float OuterSupportTieTolerance = FMath::Max(0.5f, Ctx.GetContactRadius() * 0.25f);

	struct FCompositeProjectionCandidate
	{
		FVector Surface = FVector::ZeroVector;
		FVector Normal = FVector::UpVector;
		FVector Tangent = FVector::ForwardVector;
		FVector Circumference = FVector::ForwardVector;
		FName Bone = NAME_None;
		const USceneComponent* Mesh = nullptr;
		float OuterSupport = -TNumericLimits<float>::Max();
		float Alignment = -1.0f;
		float AxialDrift = TNumericLimits<float>::Max();
		float PredictorDistance = TNumericLimits<float>::Max();
		float TieBreakScore = TNumericLimits<float>::Max();
		bool bPassesStrictFilters = false;
	};
	TArray<FCompositeProjectionCandidate, TInlineAllocator<16>> Candidates;
	float MaxStrictOuterSupport = -TNumericLimits<float>::Max();
	int32 MatchingColliderCount = 0;
	int32 BoundsQueryMissCount = 0;
	int32 PredictorFallbackHitCount = 0;
	int32 AxialRejectCount = 0;
	int32 AlignmentRejectCount = 0;

	float BestTieBreakScore = TNumericLimits<float>::Max();
	bool bFound = false;
	FVector BestSurface = FVector::ZeroVector;
	FVector BestNormal = FVector::UpVector;
	FVector BestTangent = FVector::ForwardVector;
	FVector BestCircumference = DesiredCircumference;
	FName BestBone = NAME_None;
	const USceneComponent* BestMesh = IslandMesh;

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
		++MatchingColliderCount;

		// 동일한 먼 probe를 모든 SDF에 직접 넣지 않는다. SDF의 원하는 쪽 bake-boundary에서
		// 1.5 voxel 안쪽으로 들어온 점을 사용하면 quantized narrow band 안에서 유효한 gradient로
		// 외곽 표면에 수렴한다. capsule 등 SDF view가 없는 collider는 기존 query를 유지한다.
		FVector ProjectionQueryWorld = SupportProbeWorld;
		FRopeSDFColliderView SDFView;
		if (Collider->GetGPUSDF(SDFView))
		{
			const FBox LocalBounds(SDFView.LocalMin, SDFView.LocalMin + SDFView.LocalSize);
			const FVector ProbeLocal = SDFView.BoneToWorld.InverseTransformPosition(SupportProbeWorld);
			const FVector BoundaryLocal = LocalBounds.GetClosestPointTo(ProbeLocal);
			const FVector BoundsCenter = LocalBounds.GetCenter();
			const FVector InwardDirection = (BoundsCenter - BoundaryLocal).GetSafeNormal();
			const FVector VoxelSize(
				SDFView.LocalSize.X / FMath::Max(1, SDFView.ResX - 1),
				SDFView.LocalSize.Y / FMath::Max(1, SDFView.ResY - 1),
				SDFView.LocalSize.Z / FMath::Max(1, SDFView.ResZ - 1));
			const float BoundsInset = FMath::Max(0.25f, VoxelSize.GetMax() * 1.5f);
			const FVector QueryLocal = LocalBounds.GetClosestPointTo(
				BoundaryLocal + InwardDirection * BoundsInset);
			ProjectionQueryWorld = SDFView.BoneToWorld.TransformPosition(QueryLocal);
		}

		FRopeSurfaceProjection Projection =
			Collider->ProjectToSurface(ProjectionQueryWorld, QueryRadius);
		if (!Projection.bHit)
		{
			++BoundsQueryMissCount;
			// 경계 voxel이 포화된 저해상도 bake도 경로 전체를 즉시 죽이지 않게 한다. 현재 경로
			// predictor는 직전 표면 근처이므로, 해당 collider에 유효한 local gradient가 있으면
			// 연속성을 유지할 수 있다. 외곽 선택 자체는 아래 support 비교가 계속 담당한다.
			Projection = Collider->ProjectToSurface(InOutSurfaceWorld, QueryRadius);
			if (!Projection.bHit)
			{
				continue;
			}
			++PredictorFallbackHitCount;
		}

		const float ProjectionAxisDistance = FVector::DotProduct(
			Projection.SurfacePoint - State.PathAxisOrigin, AxisDirection);
		const float CandidateAxialDrift = FMath::Abs(ProjectionAxisDistance - ProbeAxisDistance);
		const bool bPassesAxialFilter = CandidateAxialDrift <= MaxAxialProjectionDrift;
		AxialRejectCount += bPassesAxialFilter ? 0 : 1;

		const FVector CandidateOffset = Projection.SurfacePoint - State.PathAxisOrigin;
		const FVector CandidateRadialOffset = CandidateOffset -
			AxisDirection * FVector::DotProduct(CandidateOffset, AxisDirection);
		const FVector CandidateRadial = CandidateRadialOffset.GetSafeNormal(
			KINDA_SMALL_NUMBER, ExpectedRadial);
		const float CandidateAlignment = FVector::DotProduct(
			CandidateRadial, ExpectedRadial);
		const bool bPassesAlignmentFilter = CandidateAlignment >= MinSupportAlignment;
		AlignmentRejectCount += bPassesAlignmentFilter ? 0 : 1;

		const FVector CandidateNormal = Projection.Normal.GetSafeNormal(
			KINDA_SMALL_NUMBER, PreviousNormal);
		const FVector CandidateTangent =
			(Projection.SurfacePoint - PreviousSurfaceWorld).GetSafeNormal(
				KINDA_SMALL_NUMBER, DesiredTangent);
		const float DesiredTangentPenalty = 1.0f - FMath::Clamp(
			FVector::DotProduct(CandidateTangent, DesiredTangent), -1.0f, 1.0f);
		const float PreviousTangentPenalty = 1.0f - FMath::Clamp(
			FVector::DotProduct(CandidateTangent, PreviousTangent), -1.0f, 1.0f);
		const float TangentPenalty = DesiredTangentPenalty * 0.75f +
			PreviousTangentPenalty * 0.25f;
		const float NormalPenalty = 1.0f - FMath::Clamp(
			FVector::DotProduct(CandidateNormal, PreviousNormal), -1.0f, 1.0f);
		// collider별 bounds query 거리는 bounds 크기에 따라 달라지므로 tie-break에는 모든 collider에
		// 공통인 방향 probe에서 표면까지의 거리를 사용한다.
		const float TieBreakScore =
			FVector::Dist(SupportProbeWorld, Projection.SurfacePoint) *
				Ctx.Config.ProjectionDistanceWeight +
			FVector::Dist(Projection.SurfacePoint, RopeNodeWorld) * Ctx.Config.RopeNodeDistanceWeight +
			TangentPenalty * Ctx.Config.TangentContinuityWeight +
			NormalPenalty * Ctx.Config.NormalContinuityWeight;
		const float CandidateOuterSupport = FVector::DotProduct(
			CandidateRadialOffset, ExpectedRadial);


		FCompositeProjectionCandidate& Candidate = Candidates.AddDefaulted_GetRef();
		Candidate.Surface = Projection.SurfacePoint;
		Candidate.Normal = CandidateNormal;
		Candidate.Tangent = CandidateTangent;
		Candidate.Circumference = DesiredCircumference;
		Candidate.Bone = Bone;
		Candidate.Mesh = ColliderMesh;
		Candidate.OuterSupport = CandidateOuterSupport;
		Candidate.Alignment = CandidateAlignment;
		Candidate.AxialDrift = CandidateAxialDrift;
		Candidate.PredictorDistance = FVector::Dist(Projection.SurfacePoint, InOutSurfaceWorld);
		Candidate.TieBreakScore = TieBreakScore;
		Candidate.bPassesStrictFilters = bPassesAxialFilter && bPassesAlignmentFilter;
		if (Candidate.bPassesStrictFilters)
		{
			MaxStrictOuterSupport = FMath::Max(MaxStrictOuterSupport, CandidateOuterSupport);
		}

		UE_LOG(LogRopeWrap, VeryVerbose,
			TEXT("[%s] Composite support candidate: bone=%s strict=%d support=%.2f alignment=%.3f axialDrift=%.2f/%.2fcm predictorDistance=%.2fcm score=%.3f surface=%s"),
			*Ctx.OwnerName, *Bone.ToString(), Candidate.bPassesStrictFilters ? 1 : 0,
			Candidate.OuterSupport, Candidate.Alignment, Candidate.AxialDrift,
			MaxAxialProjectionDrift, Candidate.PredictorDistance, Candidate.TieBreakScore,
			*Candidate.Surface.ToString());
	}

	// Legacy outer-support 선택: strict 후보 중 최외곽 band만 남기고 기존 거리/연속성 점수로
	// tie-break한다. Analytic Helix는 이 함수를 호출하지 않고 별도 radial SDF ray 경로를 사용한다.
	for (const FCompositeProjectionCandidate& Candidate : Candidates)
	{
		if (!Candidate.bPassesStrictFilters ||
			Candidate.OuterSupport + OuterSupportTieTolerance < MaxStrictOuterSupport)
		{
			continue;
		}

		if (!bFound || Candidate.TieBreakScore < BestTieBreakScore)
		{
			bFound = true;
			BestTieBreakScore = Candidate.TieBreakScore;
			BestSurface = Candidate.Surface;
			BestNormal = Candidate.Normal;
			BestTangent = Candidate.Tangent;
			BestCircumference = Candidate.Circumference;
			BestBone = Candidate.Bone;
			BestMesh = Candidate.Mesh;
		}
	}

	if (!bFound)
	{
		// Strict 후보가 한 프레임의 pose/SDF 양자화 오차로 모두 탈락해도 즉시 접촉을 놓지 않는다.
		// 완화 단계는 축 drift/방향/예측점 거리에 넓은 하드 상한을 유지하고, 그 안에서만 최외곽을 고른다.
		const float RelaxedMaxAxialDrift = FMath::Max(
			MaxAxialProjectionDrift * 2.5f, Sim.SegmentLength * 4.0f);
		constexpr float MinRelaxedAlignment = -0.25f;
		const float MaxRelaxedPredictorDistance = FMath::Max3(
			Sim.SegmentLength * 4.0f, Ctx.GetContactRadius() * 6.0f, Ctx.SurfaceOffset * 6.0f);
		const float RelaxedSupportTolerance = FMath::Max3(
			OuterSupportTieTolerance, Ctx.GetContactRadius(), Sim.SegmentLength * 0.5f);
		float MaxRelaxedOuterSupport = -TNumericLimits<float>::Max();
		for (const FCompositeProjectionCandidate& Candidate : Candidates)
		{
			if (Candidate.AxialDrift <= RelaxedMaxAxialDrift &&
				Candidate.Alignment >= MinRelaxedAlignment &&
				Candidate.PredictorDistance <= MaxRelaxedPredictorDistance)
			{
				MaxRelaxedOuterSupport = FMath::Max(MaxRelaxedOuterSupport, Candidate.OuterSupport);
			}
		}

		for (const FCompositeProjectionCandidate& Candidate : Candidates)
		{
			if (Candidate.AxialDrift > RelaxedMaxAxialDrift ||
				Candidate.Alignment < MinRelaxedAlignment ||
				Candidate.PredictorDistance > MaxRelaxedPredictorDistance ||
				Candidate.OuterSupport + RelaxedSupportTolerance < MaxRelaxedOuterSupport)
			{
				continue;
			}

			const float RelaxedScore = Candidate.TieBreakScore +
				Candidate.AxialDrift * 0.35f +
				(1.0f - Candidate.Alignment) * 5.0f +
				Candidate.PredictorDistance * 0.25f;
			if (!bFound || RelaxedScore < BestTieBreakScore)
			{
				bFound = true;
				BestTieBreakScore = RelaxedScore;
				BestSurface = Candidate.Surface;
				BestNormal = Candidate.Normal;
				BestTangent = Candidate.Tangent;
				BestCircumference = Candidate.Circumference;
				BestBone = Candidate.Bone;
				BestMesh = Candidate.Mesh;
			}
		}

		if (bFound)
		{
			if (OutSupportTier)
			{
				*OutSupportTier = ERopeCompositeSupportTier::Relaxed;
			}
			UE_LOG(LogRopeWrap, Log,
				TEXT("[%s] Composite support recovered: tier=Relaxed selected=%s candidates=%d axialReject=%d alignmentReject=%d projectionMiss=%d predictorFallback=%d support=%.2f score=%.3f"),
				*Ctx.OwnerName, *BestBone.ToString(), Candidates.Num(), AxialRejectCount,
				AlignmentRejectCount, BoundsQueryMissCount, PredictorFallbackHitCount,
				MaxRelaxedOuterSupport, BestTieBreakScore);
		}
	}

	if (!bFound)
	{
		// 마지막 composite 내부 복구: 현재 선택 본의 직전 표면에서 다시 projection한다. 한 step의
		// 외곽 support가 비었다는 이유만으로 latch 자체를 놓지 않고 다음 sweep step에서 재진입시킨다.
		const FName ContinuityBone = State.PathWrapIslandBones.Contains(InOutBone)
			? InOutBone
			: (State.PathWrapIslandBones.Contains(State.LatchAnchor.Bone)
				? State.LatchAnchor.Bone
				: NAME_None);
		FVector ContinuitySurface = InOutSurfaceWorld;
		FVector ContinuityNormal = InOutNormalWorld;
		bool bContinuityHit = !ContinuityBone.IsNone() && ProjectWrapPointToSurface(
			ContinuityBone, IslandMesh, Sim, Ctx, ContinuitySurface, ContinuityNormal);
		if (!bContinuityHit)
		{
			ContinuitySurface = PreviousSurfaceWorld;
			ContinuityNormal = PreviousNormal;
			bContinuityHit = ProjectWrapPointToSurface(
				ContinuityBone, IslandMesh, Sim, Ctx, ContinuitySurface, ContinuityNormal);
		}

		if (bContinuityHit)
		{
			BestSurface = ContinuitySurface;
			BestNormal = ContinuityNormal.GetSafeNormal(KINDA_SMALL_NUMBER, PreviousNormal);
			BestTangent = (BestSurface - PreviousSurfaceWorld).GetSafeNormal(
				KINDA_SMALL_NUMBER, DesiredTangent);
			BestCircumference = DesiredCircumference;
			BestBone = ContinuityBone;
			BestMesh = IslandMesh;
			bFound = true;
			if (OutSupportTier)
			{
				*OutSupportTier = ERopeCompositeSupportTier::Continuity;
			}
			UE_LOG(LogRopeWrap, Warning,
				TEXT("[%s] Composite support recovered: tier=Continuity bone=%s colliders=%d candidates=%d projectionMiss=%d axialReject=%d alignmentReject=%d previous=%s recovered=%s"),
				*Ctx.OwnerName, *BestBone.ToString(), MatchingColliderCount, Candidates.Num(),
				BoundsQueryMissCount, AxialRejectCount, AlignmentRejectCount,
				*PreviousSurfaceWorld.ToString(), *BestSurface.ToString());
		}
	}

	if (!bFound)
	{
		FString IslandBoneList;
		for (const FName Bone : State.PathWrapIslandBones)
		{
			if (!IslandBoneList.IsEmpty())
			{
				IslandBoneList += TEXT(",");
			}
			IslandBoneList += Bone.ToString();
		}
		// 이 시점에는 아래 SingleBone fallback이 아직 남아 있으므로 최종 오류가 아니라 복구 경고다.
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Composite support exhausted: currentBone=%s island=[%s] colliders=%d candidates=%d projectionMiss=%d predictorFallback=%d axialReject=%d alignmentReject=%d queryRadius=%.2f maxAxialDrift=%.2f probeRadius=%.2f desiredRadial=%s probe=%s predictor=%s previous=%s ropeNode=%s axisOrigin=%s axisDir=%s"),
			*Ctx.OwnerName, *InOutBone.ToString(), *IslandBoneList, MatchingColliderCount,
			Candidates.Num(), BoundsQueryMissCount, PredictorFallbackHitCount, AxialRejectCount,
			AlignmentRejectCount, QueryRadius, MaxAxialProjectionDrift, SupportProbeRadius,
			*ExpectedRadial.ToString(), *SupportProbeWorld.ToString(), *InOutSurfaceWorld.ToString(),
			*PreviousSurfaceWorld.ToString(), *RopeNodeWorld.ToString(),
			*State.PathAxisOrigin.ToString(), *AxisDirection.ToString());
		return false;
	}
	if (OutSupportTier && *OutSupportTier == ERopeCompositeSupportTier::Failed)
	{
		*OutSupportTier = ERopeCompositeSupportTier::Strict;
	}

	InOutSurfaceWorld = BestSurface;
	InOutNormalWorld = BestNormal;
	InOutTangentWorld = BestTangent;
	InOutCircumferenceDir = BestCircumference;
	InOutBone = BestBone;
	OutMesh = BestMesh;
	return true;
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
		UE_LOG(LogRopeWrap, Log,
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
	OutMesh = Mesh ? Mesh : State.LatchAnchor.Mesh.Get();
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

void FRopeWrappingPhase::AdvanceWrappingFront(float DeltaTime, const FRopeSimState& Sim, const FContext& Ctx)
{
	if (State.Anchors.Num() == 0 || State.NumTailNodes <= 0)
	{
		State.FrontDistance = 0.0f;
		return;
	}

	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	const float FullDistance = static_cast<float>(FMath::Max(0, State.NumTailNodes - 1)) * SegmentLength;
	float BuiltPathMaxDistance = 0.0f;
	for (const FRopeWrapPathPoint& Point : State.Path)
	{
		BuiltPathMaxDistance = FMath::Max(BuiltPathMaxDistance, Point.DistanceFromLatch);
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
	if (TargetFrontDistance <= KINDA_SMALL_NUMBER)
	{
		State.FrontDistance = 0.0f;
		return;
	}

	const float FullTailDelay = (FullDistance / SegmentLength) * Ctx.Config.WrappingTailDelayPerSegment;
	const float TotalDuration = FMath::Max(State.Duration + FullTailDelay, KINDA_SMALL_NUMBER);
	const float BaseFrontSpeed = FullDistance / TotalDuration;

	// Accelerate the visual wrap front as more of the tail is already wound.
	// Use the full requested distance, not the currently built path cap, so early path-build frames do not spike speed.
	const float ProgressAlpha = FMath::Clamp(
		State.FrontDistance / FMath::Max(FullDistance, KINDA_SMALL_NUMBER),
		0.0f,
		1.0f);

	const float StartSpeedScale = 0.5f;
	const float EndSpeedScale = 3.0f;
	const float SpeedScale = FMath::Lerp(StartSpeedScale, EndSpeedScale, RopeMath::SmoothStep(ProgressAlpha));

	const float FrontSpeed = BaseFrontSpeed * SpeedScale;
	State.FrontDistance = FMath::Min(
		State.FrontDistance + FrontSpeed * FMath::Max(0.0f, DeltaTime),
		TargetFrontDistance);
}

bool FRopeWrappingPhase::SampleWrappingPath(float DistanceFromLatch, FRopeWrapPathPoint& OutPoint) const
{
	if (State.Path.Num() == 0 && State.Anchors.Num() == 0)
	{
		return false;
	}

	const auto AnchorToPoint = [this](const FRopeSurfaceAnchor& Anchor, FRopeWrapPathPoint& Point) -> bool
	{
		const USceneComponent* Mesh = Anchor.Mesh.Get();
		if (!Mesh)
		{
			Mesh = State.Mesh.Get();
		}
		if (!Mesh || Anchor.Bone.IsNone())
		{
			return false;
		}

		const FTransform BoneXform = ResolveBindingWorld(Mesh, Anchor.Bone);
		Point.SurfaceWorld = BoneXform.TransformPosition(Anchor.LocalSurfacePosition);
		Point.NormalWorld = BoneXform.TransformVectorNoScale(Anchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		Point.TangentWorld = BoneXform.TransformVectorNoScale(Anchor.LocalTangent);
		Point.TangentWorld = (Point.TangentWorld - FVector::DotProduct(Point.TangentWorld, Point.NormalWorld) * Point.NormalWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(Point.NormalWorld));
		Point.Bone = Anchor.Bone;
		Point.Mesh = Mesh;
		Point.DistanceFromLatch = Anchor.RopeDistance;
		return true;
	};
	const auto InterpolatePoints = [](const FRopeWrapPathPoint& LowerPoint,
		const FRopeWrapPathPoint& UpperPoint, float SampleDistance,
		FRopeWrapPathPoint& Point)
	{
		if (FMath::Abs(UpperPoint.DistanceFromLatch - LowerPoint.DistanceFromLatch)
			<= KINDA_SMALL_NUMBER)
		{
			Point = LowerPoint;
			Point.DistanceFromLatch = SampleDistance;
			return;
		}

		const float Alpha = FMath::Clamp(
			(SampleDistance - LowerPoint.DistanceFromLatch) /
			(UpperPoint.DistanceFromLatch - LowerPoint.DistanceFromLatch),
			0.0f, 1.0f);
		Point.SurfaceWorld = FMath::Lerp(
			LowerPoint.SurfaceWorld, UpperPoint.SurfaceWorld, Alpha);
		Point.NormalWorld = FMath::Lerp(
			LowerPoint.NormalWorld, UpperPoint.NormalWorld, Alpha)
			.GetSafeNormal(KINDA_SMALL_NUMBER, LowerPoint.NormalWorld);
		Point.TangentWorld = FMath::Lerp(
			LowerPoint.TangentWorld, UpperPoint.TangentWorld, Alpha);
		Point.TangentWorld = (Point.TangentWorld -
			FVector::DotProduct(Point.TangentWorld, Point.NormalWorld) * Point.NormalWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, LowerPoint.TangentWorld);
		Point.Bone = Alpha < 0.5f ? LowerPoint.Bone : UpperPoint.Bone;
		Point.Mesh = Alpha < 0.5f ? LowerPoint.Mesh : UpperPoint.Mesh;
		Point.DistanceFromLatch = SampleDistance;
		Point.bBridge = LowerPoint.bBridge || UpperPoint.bBridge;
		Point.bVirtual = LowerPoint.bVirtual || UpperPoint.bVirtual;
	};

	const float SampleDistance = FMath::Max(0.0f, DistanceFromLatch);
	if (State.Path.Num() > 0)
	{
		int32 LowerPathIndex = INDEX_NONE;
		int32 UpperPathIndex = INDEX_NONE;
		for (int32 PathIndex = 0; PathIndex < State.Path.Num(); ++PathIndex)
		{
			const FRopeWrapPathPoint& StoredPoint = State.Path[PathIndex];
			if (StoredPoint.DistanceFromLatch <= SampleDistance &&
				(LowerPathIndex == INDEX_NONE ||
					StoredPoint.DistanceFromLatch > State.Path[LowerPathIndex].DistanceFromLatch))
			{
				LowerPathIndex = PathIndex;
			}
			if (StoredPoint.DistanceFromLatch >= SampleDistance &&
				(UpperPathIndex == INDEX_NONE ||
					StoredPoint.DistanceFromLatch < State.Path[UpperPathIndex].DistanceFromLatch))
			{
				UpperPathIndex = PathIndex;
			}
		}

		if (LowerPathIndex == INDEX_NONE)
		{
			LowerPathIndex = UpperPathIndex;
		}
		if (UpperPathIndex == INDEX_NONE)
		{
			UpperPathIndex = LowerPathIndex;
		}
		if (LowerPathIndex == INDEX_NONE || UpperPathIndex == INDEX_NONE)
		{
			return false;
		}

		const auto ResolveStoredPoint = [this, &AnchorToPoint](
			int32 PathIndex, FRopeWrapPathPoint& Point) -> bool
		{
			const FRopeWrapPathPoint& StoredPoint = State.Path[PathIndex];
			if (StoredPoint.bBridge || StoredPoint.bVirtual)
			{
				Point = StoredPoint;
				return true;
			}

			const int32 NodeIndex = State.LatchAnchor.NodeIndex + PathIndex;
			const FRopeSurfaceAnchor* Anchor = State.Anchors.FindByPredicate(
				[NodeIndex](const FRopeSurfaceAnchor& Candidate)
				{
					return Candidate.NodeIndex == NodeIndex;
				});
			if (Anchor)
			{
				return AnchorToPoint(*Anchor, Point);
			}

			// Progressive build에서 anchor가 아직 붙기 전인 같은 프레임의 짧은 창은 스냅샷을 쓴다.
			Point = StoredPoint;
			return true;
		};

		FRopeWrapPathPoint LowerPoint;
		FRopeWrapPathPoint UpperPoint;
		if (!ResolveStoredPoint(LowerPathIndex, LowerPoint) ||
			!ResolveStoredPoint(UpperPathIndex, UpperPoint))
		{
			return false;
		}
		InterpolatePoints(LowerPoint, UpperPoint, SampleDistance, OutPoint);
		return true;
	}

	const FRopeSurfaceAnchor* LowerAnchor = nullptr;
	const FRopeSurfaceAnchor* UpperAnchor = nullptr;
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (Anchor.RopeDistance <= SampleDistance &&
			(!LowerAnchor || Anchor.RopeDistance > LowerAnchor->RopeDistance))
		{
			LowerAnchor = &Anchor;
		}
		if (Anchor.RopeDistance >= SampleDistance &&
			(!UpperAnchor || Anchor.RopeDistance < UpperAnchor->RopeDistance))
		{
			UpperAnchor = &Anchor;
		}
	}

	if (!LowerAnchor)
	{
		LowerAnchor = UpperAnchor;
	}
	if (!UpperAnchor)
	{
		UpperAnchor = LowerAnchor;
	}
	if (!LowerAnchor || !UpperAnchor)
	{
		return false;
	}

	FRopeWrapPathPoint LowerPoint;
	if (!AnchorToPoint(*LowerAnchor, LowerPoint))
	{
		return false;
	}

	if (LowerAnchor == UpperAnchor ||
		FMath::Abs(UpperAnchor->RopeDistance - LowerAnchor->RopeDistance) <= KINDA_SMALL_NUMBER)
	{
		OutPoint = LowerPoint;
		OutPoint.DistanceFromLatch = SampleDistance;
		return true;
	}

	FRopeWrapPathPoint UpperPoint;
	if (!AnchorToPoint(*UpperAnchor, UpperPoint))
	{
		return false;
	}

	InterpolatePoints(LowerPoint, UpperPoint, SampleDistance, OutPoint);
	return true;
}
