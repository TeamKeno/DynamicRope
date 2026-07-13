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

bool FRopeWrappingPhase::Begin(const FRopeSurfaceAnchor& LatchAnchor, const USceneComponent* Mesh, FName Bone,
	float Duration, const FRopeSimState& Sim, const FContext& Ctx)
{
	State.BoneName = Bone;
	State.Mesh = Mesh;
	State.Elapsed = 0.0f;
	State.Duration = Duration;
	State.FirstNode = TNumericLimits<int32>::Max();
	State.LastNode = INDEX_NONE;

	if (!BeginProgressiveWrapPathBuild(LatchAnchor, Sim, Ctx))
	{
		return false;
	}

	State.StableTime = 0.0f;
	State.LastStableFirstNode = State.FirstNode;
	State.LastStableLastNode = State.LastNode;
	State.LastStableAnchorCount = State.Anchors.Num();
	return true;
}

void FRopeWrappingPhase::FinishPathBuild(bool bFailed)
{
	State.bPathBuildActive = false;
	State.bPathBuildComplete = !bFailed;
	State.bPathBuildFailed = bFailed;
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
	if (State.PathMode == ERopeWrappingPathMode::AnalyticHelix)
	{
		for (int32 StepIndex = 0;
			StepIndex < StepBudget && State.Path.Num() < State.NumTailNodes;
			++StepIndex)
		{
			const int32 PathIndex = State.Path.Num();
			if (!AppendAnalyticProgressiveWrapPathPoint(PathIndex, Sim, Ctx))
			{
				break;
			}

			AppendWrappingAnchorFromPathPoint(PathIndex, Sim, Ctx);
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
	const FVector FrontWorld = FrontPoint.SurfaceWorld + FrontPoint.NormalWorld * SurfaceOffset;
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

		const float NodeDistance = static_cast<float>(NodeIndex - LatchNode) * SegmentLength;
		FVector World = FVector::ZeroVector;
		if (NodeDistance <= State.FrontDistance + KINDA_SMALL_NUMBER)
		{
			FRopeWrapPathPoint NodePoint;
			if (!SampleWrappingPath(NodeDistance, NodePoint))
			{
				continue;
			}
			World = NodePoint.SurfaceWorld + NodePoint.NormalWorld * SurfaceOffset;
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

	OutFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		const bool bWrappingDrivenNode = bHasValidLatch && i >= LatchNode;
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
	const FContext PreviewCtx{ PreviewConfig, Ctx.Colliders, Ctx.PathMode, Ctx.SurfaceOffset, Ctx.OwnerName, true };

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
		return false;
	}

	FRopeSurfaceAnchor StoredLatchAnchor = LatchAnchor;
	StoredLatchAnchor.Mesh = Mesh;

	State.Anchors.Reset();
	State.Path.Reset();
	State.LatchAnchor = StoredLatchAnchor;
	State.PathMode = Ctx.PathMode;
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
		return false;
	}

	State.Path.Reserve(State.NumTailNodes);

	const bool bInitialized =
		State.PathMode == ERopeWrappingPathMode::AnalyticHelix
			? AppendAnalyticProgressiveWrapPathPoint(0, Sim, Ctx)
			: InitializeSurfaceVectorFieldProgressiveWrapPath(StoredLatchAnchor, Sim, Ctx);
	if (!bInitialized || !AppendWrappingAnchorFromPathPoint(0, Sim, Ctx))
	{
		return false;
	}

	return true;
}

bool FRopeWrappingPhase::AppendAnalyticProgressiveWrapPathPoint(int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx)
{
	if (PathIndex < 0 ||
		PathIndex >= State.NumTailNodes ||
		PathIndex != State.Path.Num())
	{
		return false;
	}

	const float DistanceFromLatch = static_cast<float>(PathIndex) * Sim.SegmentLength;

	FRopeWrapPathPoint Point;
	Point.DistanceFromLatch = DistanceFromLatch;
	Point.Bone = State.LatchAnchor.Bone;
	Point.Mesh = State.LatchAnchor.Mesh;
	if (!ComputeAnalyticHelixWrapTarget(State.LatchAnchor, DistanceFromLatch, Sim, Ctx,
		Point.SurfaceWorld, Point.NormalWorld, Point.TangentWorld) &&
		!ComputeSurfaceVectorFieldWrapTarget(State.LatchAnchor, DistanceFromLatch, Sim, Ctx,
			Point.SurfaceWorld, Point.NormalWorld, Point.TangentWorld))
	{
		FinishPathBuild(/*bFailed=*/true);
		return false;
	}

	State.Path.Add(Point);
	if (State.Path.Num() >= State.NumTailNodes)
	{
		FinishPathBuild(/*bFailed=*/false);
	}
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

	FRopeWrapPathPoint LatchPoint;
	LatchPoint.SurfaceWorld = State.PathSurfaceWorld;
	LatchPoint.NormalWorld = State.PathNormalWorld;
	LatchPoint.TangentWorld = State.PathTangentWorld;
	LatchPoint.Bone = State.PathCurrentBone;
	LatchPoint.Mesh = State.PathCurrentMesh;
	LatchPoint.DistanceFromLatch = 0.0f;
	State.Path.Add(LatchPoint);
	State.PathCurrentDistance = 0.0f;

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
		FinishPathBuild(/*bFailed=*/true);
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
		const float TargetDistance = static_cast<float>(PathIndex) * Sim.SegmentLength;
		bool bConsumedStep = false;

		while (StepsRemaining > 0 &&
			State.PathCurrentDistance + KINDA_SMALL_NUMBER < TargetDistance)
		{
			const float StepDistance = FMath::Min(
				StepSize,
				TargetDistance - State.PathCurrentDistance);

			State.PathTangentWorld = ComputeSurfaceVectorFieldTangent(
				State.PathAxisOrigin,
				State.PathAxisDirection,
				State.PathLatchRadial,
				State.PathWindingSign,
				State.PathSurfaceWorld,
				State.PathNormalWorld,
				Ctx,
				State.PathCircumferenceDir);

			FVector StepRadialBefore = FVector::ZeroVector;
			const bool bHasRadialBefore = ComputeAxisRadial(State.PathSurfaceWorld, StepRadialBefore);

			State.PathSurfaceWorld += State.PathTangentWorld * StepDistance;
			const FName CurrentBone = State.PathCurrentBone.IsNone()
				? State.LatchAnchor.Bone
				: State.PathCurrentBone;
			const USceneComponent* ProjectedMesh = State.PathCurrentMesh.Get();
			if (!ProjectedMesh)
			{
				ProjectedMesh = Mesh;
			}

			// projection query 자체는 tangent field가 예측한 다음 표면 위치(State.PathSurfaceWorld)에서 시작한다.
			// 다만 같은 query 지점에서 여러 후보 본이 모두 맞을 수 있으므로, 실제 rope node가 현재 프레임에
			// 어디에 있는지도 scoring에 넣는다. 이 항목이 없으면 예측 path만 믿고 로프 몸체와 먼 본으로
			// 넘어가는 일이 생길 수 있다.
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
			bool bOnSurface = ProjectWrapPointToSurfaceMultiBone(CurrentBone, Mesh, Sim, Ctx,
				State.PathPreviousBone, State.PathDistanceSinceBoneTransition, RopeNodeWorld,
				State.PathNormalWorld, State.PathTangentWorld,
				ProjectedSurface, ProjectedNormal, ProjectedTangent,
				ProjectedCircumference, ProjectedBone, ProjectedMesh);

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
					Sim.SegmentLength, Ctx.Config.ContactRadius * 2.0f, Ctx.SurfaceOffset * 2.0f);
				if (SnapDistance > MaxSnapDistance)
				{
					bOnSurface = false;
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
					}
				}
			}

			// 표면도 없고 브리지도 소진/비활성 — 종전과 같은 실패 처리(각도 적분 전에 끊어,
			// 걷지 못한 스텝이 실패 시점 각도(ShouldAbortFailedShortWrap)에 섞이지 않게 한다).
			if (!bOnSurface &&
				(MaxBridgeDistance <= 0.0f || State.PathBridgeDistance + StepDistance > MaxBridgeDistance))
			{
				FinishPathBuild(/*bFailed=*/true);
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
					// rolling axis: 이후 스텝의 tangent field가 새 본 형상 축 주위를 돌게 한다.
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

			State.PathCurrentDistance += StepDistance;
			--StepsRemaining;
			bConsumedStep = true;
		}

		if (State.PathCurrentDistance + KINDA_SMALL_NUMBER < TargetDistance)
		{
			break;
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

		FRopeWrapPathPoint Point;
		Point.SurfaceWorld = State.PathSurfaceWorld;
		Point.NormalWorld = State.PathNormalWorld;
		Point.TangentWorld = State.PathTangentWorld;
		Point.Bone = State.PathCurrentBone.IsNone() ? State.LatchAnchor.Bone : State.PathCurrentBone;
		Point.Mesh = State.PathCurrentMesh.IsValid() ? State.PathCurrentMesh.Get() : Mesh;
		Point.DistanceFromLatch = TargetDistance;
		// 마지막 서브스텝이 브리지였으면 이 점은 허공 chord 위다 — 앵커 생성이 스킵된다.
		Point.bBridge = State.PathBridgeDistance > 0.0f;
		State.Path.Add(Point);
		AppendWrappingAnchorFromPathPoint(PathIndex, Sim, Ctx);

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

	// 허공 브리지(chord) 경로점: 표면 프레임이 없으므로 앵커를 만들지 않는다 — 커밋 후 이 노드는
	// 자유 로프로 남아 solver가 chord/현수 형태를 잡는다. 앵커 카운터는 전진시켜야 한다: 이 함수는
	// PathIndex == LastAnchoredPathPointCount일 때만 신규 처리하므로, 여기서 멈추면 브리지 뒤
	// 재진입한 표면 경로점들의 앵커 생성이 전부 막힌다.
	if (Point.bBridge)
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

bool FRopeWrappingPhase::ComputeSurfaceVectorFieldWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
	const FRopeSimState& Sim, const FContext& Ctx,
	FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const
{
	//1. Mesh / Bone 확인
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

	//2. 감김 축 구하기
	if (!ResolveWrappingAxis(LatchAnchor, Ctx, AxisOrigin, AxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, AxisDirection);

	//3. latch anchor를 world 좌표로 복원
	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);
	FVector SurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
	FVector NormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

	//4. 최초 rope tangent를 표면 위 방향으로 정리
	FVector LatchTangentWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalTangent);
	LatchTangentWorld = (LatchTangentWorld - FVector::DotProduct(LatchTangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(NormalWorld));

	//5. latch 지점의 radial 구하기
	const float LatchAxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
	const FVector LatchAxisPoint = AxisOrigin + AxisDirection * LatchAxisDistance;
	const FVector LatchRadial = (SurfaceWorld - LatchAxisPoint)
		.GetSafeNormal(KINDA_SMALL_NUMBER, NormalWorld);

	//6. 최초 원주 방향과 감김 방향 결정
	FVector CircumferenceDir = FVector::CrossProduct(AxisDirection, LatchRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(NormalWorld));
	const float WindingSign = FVector::DotProduct(CircumferenceDir, LatchTangentWorld) < 0.0f ? -1.0f : 1.0f;
	CircumferenceDir *= WindingSign;

	//7. 최초 tangent field 만들기
	FVector TangentWorld = (CircumferenceDir + AxisDirection * Ctx.Config.WrappingHelixPitchScale)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);
	//SDF 표면 밖으로 튀어나가는 성분 제거
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);

	//8. 거리 0이면 바로 latch point 반환
	if (DistanceFromLatch <= KINDA_SMALL_NUMBER)
	{
		OutSurfaceWorld = SurfaceWorld;
		OutNormalWorld = NormalWorld;
		OutTangentWorld = TangentWorld;
		return true;
	}

	//9. 걷기 step 계산
	const float StepSize = FMath::Max(1.0f, Sim.SegmentLength * 0.5f);
	const int32 StepCount = FMath::Max(1, FMath::CeilToInt(DistanceFromLatch / StepSize));
	float RemainingDistance = DistanceFromLatch;

	//10. 표면 위를 실제로 걷는 루프
	for (int32 StepIndex = 0; StepIndex < StepCount; ++StepIndex)
	{
		const float StepDistance = FMath::Min(StepSize, RemainingDistance);
		RemainingDistance -= StepDistance;

		//현재 위치에서 radial을 다시 구함. 현재 표면 위치에서 매번 radial을 다시 계산
		const float AxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
		const FVector AxisPoint = AxisOrigin + AxisDirection * AxisDistance;
		const FVector Radial = (SurfaceWorld - AxisPoint).GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);

		//원주 방향 재계산:
		CircumferenceDir = FVector::CrossProduct(AxisDirection, Radial)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir) * WindingSign;
		//pitch를 섞어 걸어갈 방향을 다시 만듦:
		TangentWorld = (CircumferenceDir + AxisDirection * Ctx.Config.WrappingHelixPitchScale)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);
		//표면 tangent plane에 다시 눕힘:
		TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);

		SurfaceWorld += TangentWorld * StepDistance;
		if (!ProjectWrapPointToSurface(LatchAnchor.Bone, Mesh, Sim, Ctx, SurfaceWorld, NormalWorld))
		{
			return false;
		}
	}

	//11. 최종 위치에서 tangent 다시 계산
	const float FinalAxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
	const FVector FinalAxisPoint = AxisOrigin + AxisDirection * FinalAxisDistance;
	const FVector FinalRadial = (SurfaceWorld - FinalAxisPoint).GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);
	CircumferenceDir = FVector::CrossProduct(AxisDirection, FinalRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir) * WindingSign;
	TangentWorld = (CircumferenceDir + AxisDirection * Ctx.Config.WrappingHelixPitchScale)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);

	OutSurfaceWorld = SurfaceWorld;
	OutNormalWorld = NormalWorld;
	OutTangentWorld = TangentWorld;
	return true;
}

/* 이 방식은 "수학적인 나선"을 먼저 만든 다음 SDF 표면에 붙입니다. */
//DistnaceFromLatch : tail node가 latch에서 얼마나 떨어졌는지
// LatchAnchor       = 감김이 시작된 최초 접촉점 정보
bool FRopeWrappingPhase::ComputeAnalyticHelixWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
	const FRopeSimState& Sim, const FContext& Ctx,
	FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const
{
	///1. Mesh와 Bone 확인
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

	//2. 감김 축 정의
	if (!ResolveWrappingAxis(LatchAnchor, Ctx, AxisOrigin, AxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, AxisDirection);

	//3. latch anchor를 world 좌표로 복원
	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);
	//LatchSurfaceWorld가 나선의 시작점
	const FVector LatchSurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);

	//4. LatchNormalWorld normal / tangent 복원
	//LatchNormalWorld는 접촉 표면 normal.
	//LatchTangentWorld는 최초 latch 순간 로프가 tail 방향으로 뻗던 방향
	const FVector LatchNormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector LatchTangentWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalTangent);
	// tangent를 normal plane에 투영한다.
	LatchTangentWorld = (LatchTangentWorld - FVector::DotProduct(LatchTangentWorld, LatchNormalWorld) * LatchNormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(LatchNormalWorld));

	// 5. latch 점을 축 기준으로 분해: latch point의 축 위 높이 → 그 축 위의 점 → 축에서 latch point로
	//    향하는 radial. HelixRadius = latch 지점에서 bone-parent 축까지의 거리(나선 반지름).
	const float LatchAxisDistance = FVector::DotProduct(LatchSurfaceWorld - AxisOrigin, AxisDirection);
	const FVector LatchAxisPoint = AxisOrigin + AxisDirection * LatchAxisDistance;
	FVector LatchRadial = LatchSurfaceWorld - LatchAxisPoint;
	const float HelixRadius = LatchRadial.Size();
	if (HelixRadius <= KINDA_SMALL_NUMBER)
	{
		return false;
	}
	// 정규화(radial 단위 벡터).
	LatchRadial /= HelixRadius;

	// 6. 감기는 방향 결정: 축×radial 원주 방향이 최초 latch rope tangent와 일치하는지로 부호를 정한다.
	FVector CircumferenceDir = FVector::CrossProduct(AxisDirection, LatchRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(LatchNormalWorld));
	const float WindingSign = FVector::DotProduct(CircumferenceDir, LatchTangentWorld) < 0.0f ? -1.0f : 1.0f;
	CircumferenceDir *= WindingSign;

	// 7. rope 거리 d를 나선 파라미터로 변환: 원주 이동분 + 축 방향 이동량 + 원주 회전량.
	const float PitchScale = Ctx.Config.WrappingHelixPitchScale;
	const float LengthScale = FMath::Sqrt(1.0f + PitchScale * PitchScale);
	const float CircumferenceDistance = DistanceFromLatch / FMath::Max(LengthScale, KINDA_SMALL_NUMBER);
	const float AxisDistance = CircumferenceDistance * PitchScale;
	const float AngleRadians = WindingSign * CircumferenceDistance / FMath::Max(HelixRadius, KINDA_SMALL_NUMBER);

	// 8. 나선점 생성: 축을 중심으로 LatchRadial을 AngleRadians만큼 돌리고(원주 위 방향), 축 위에서
	//    latch 높이보다 AxisDistance만큼 이동한 중심점에서 반지름만큼 바깥으로 나가면 나선 위 월드 위치.
	const FQuat AxisRotation(AxisDirection, AngleRadians);
	const FVector RotatedRadial = AxisRotation.RotateVector(LatchRadial).GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);
	const FVector TargetAxisPoint = AxisOrigin + AxisDirection * (LatchAxisDistance + AxisDistance);
	FVector SurfaceWorld = TargetAxisPoint + RotatedRadial * HelixRadius;

	// 9. 마지막으로 SDF 표면에 붙인다.
	FVector NormalWorld = RotatedRadial;
	if (!ProjectWrapPointToSurface(LatchAnchor.Bone, Mesh, Sim, Ctx, SurfaceWorld, NormalWorld))
	{
		return false;
	}

	// 10. 보정된 표면에서 tangent 다시 계산
	const float SurfaceAxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
	const FVector SurfaceAxisPoint = AxisOrigin + AxisDirection * SurfaceAxisDistance;
	FVector SurfaceRadial = SurfaceWorld - SurfaceAxisPoint;
	SurfaceRadial = SurfaceRadial.GetSafeNormal(KINDA_SMALL_NUMBER, RotatedRadial);

	FVector SurfaceCircumferenceDir = FVector::CrossProduct(AxisDirection, SurfaceRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir) * WindingSign;
	FVector TangentWorld = (SurfaceCircumferenceDir + AxisDirection * PitchScale)
		.GetSafeNormal(KINDA_SMALL_NUMBER, SurfaceCircumferenceDir);
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, SurfaceCircumferenceDir);

	//11. 결과 반환
	OutSurfaceWorld = SurfaceWorld;
	OutNormalWorld = NormalWorld;
	OutTangentWorld = TangentWorld;
	return true;
}

bool FRopeWrappingPhase::ComputeWrappedAngleAtLastBuiltPoint(const FRopeSimState& Sim, const FContext& Ctx, float& OutAngleDeg) const
{
	OutAngleDeg = 0.0f;
	if (State.Anchors.Num() == 0 && State.Path.Num() == 0)
	{
		return false;
	}

	// SurfaceVectorField는 경로 빌드가 스텝마다 적분해 둔 누적 각도를 그대로 쓴다 — 축이 본 전환마다
	// 재해석되므로(rolling axis) latch 축 하나로 전체 거리를 나누는 아래 helix 공식이 성립하지 않는다.
	// 단일 본(전환 없음)에서는 두 척도가 일치한다(RopeWrappingPhaseTests가 고정하는 계약).
	// AnalyticHelix(축 고정), 그리고 아직 한 스텝도 걷지 못한 SVF는 기존 helix 공식으로 폴백.
	if (State.PathMode == ERopeWrappingPathMode::SurfaceVectorField &&
		State.PathAccumulatedAngleRad > KINDA_SMALL_NUMBER)
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
	if (FindColliderShapeAxis(Ctx, LatchAnchor.Bone, LatchAnchor.Mesh.Get(), OutAxisOrigin, OutAxisDirection))
	{
		LogAxisSource(TEXT("ColliderShapeAxis"), LatchAnchor.Mesh.Get());
		return true;
	}


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

void FRopeWrappingPhase::GatherSurfaceVectorFieldBoneCandidates(FName CurrentBone, const USceneComponent* Mesh,
	TArray<FSurfaceVectorFieldBoneCandidate>& OutCandidates, const FContext& Ctx) const
{
	OutCandidates.Reset();
	if (!Mesh || CurrentBone.IsNone())
	{
		return;
	}

	// 후보 본 그래프(parent/child) 탐색은 본 그래프가 있는 대상(스켈레탈)에서만 유효하다. 정적 대상은
	// 부모/자식 키가 비어 후보가 CurrentBone 하나로 남는다(단일 본 랩 폴백 — 정적은 본 그래프가 없다).
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

	// SurfaceVectorField의 본 후보를 "가까운 본 이름 목록"이 아니라 작은 graph 탐색 결과로 다룬다.
	// 지금은 자동 parent/child edge만 쓰지만, 후보가 Depth/GraphCost를 들고 다니므로 이후 디자이너
	// transition edge를 같은 경로에 섞어 넣을 수 있다. 탐색은 bounded Dijkstra에 가깝게 비용이 낮은
	// 항목부터 확장하고, depth/cost 상한으로 팔->몸통->반대팔 같은 먼 bridge가 우연히 열리는 것을 막는다.
	TArray<FBoneQueueEntry, TInlineAllocator<16>> Queue;
	TMap<FName, float> BestCostByBone;
	TMap<FName, int32> BestDepthByBone;
	Queue.Add({ CurrentBone, 0, 0.0f });
	BestCostByBone.Add(CurrentBone, 0.0f);
	BestDepthByBone.Add(CurrentBone, 0);

	// 후보 수가 매우 작다는 전제의 Dijkstra-lite 구현이다.
	// Unreal 쪽 priority queue 의존을 늘리지 않고 TInlineAllocator 배열에서 가장 싼 항목을 직접 고른다.
	// 현재 parent/child edge 비용은 모두 같지만, 이 형태로 두면 designer transition edge에 다른
	// penalty를 붙였을 때도 함수 구조를 바꾸지 않고 그대로 확장할 수 있다.
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

		FSurfaceVectorFieldBoneCandidate Candidate;
		Candidate.Bone = Entry.Bone;
		Candidate.Depth = Entry.Depth;
		Candidate.GraphCost = Entry.Cost;
		Candidate.bCurrentBone = Entry.Bone == CurrentBone;
		OutCandidates.Add(Candidate);

		if (Entry.Depth >= MaxCandidateDepth)
		{
			continue;
		}
		if (Entry.Cost >= MaxCandidateCost)
		{
			continue;
		}

		// 지금 구현의 graph edge는 skeleton parent/child뿐이다.
		// 나중에 AllowedBoneTransitions/TransitionChains를 추가하면 여기에서 AddNeighbor(ToBone, Penalty)처럼
		// designer edge도 같이 넣으면 된다. 후보 구조가 이미 GraphCost를 들고 있어서 projection/scoring
		// 쪽은 edge 출처를 몰라도 된다.
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
					(FMath::IsNearlyEqual(*ExistingCost, NextCost) && ExistingDepth && *ExistingDepth <= NextDepth)))
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
		FMath::Max(Ctx.Config.ContactRadius, Ctx.SurfaceOffset),
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
			BestProjection = CurrentBoneProjection;
		}
	}

	InOutSurfaceWorld = BestProjection.SurfaceWorld;
	InOutNormalWorld = BestProjection.NormalWorld;
	InOutTangentWorld = BestProjection.TangentWorld;
	InOutCircumferenceDir = BestProjection.CircumferenceDir;
	InOutBone = BestProjection.Bone;
	OutMesh = BestProjection.Mesh;
	return true;
}

bool FRopeWrappingPhase::ProjectWrapPointToSurface(FName Bone, const USceneComponent* Mesh,
	const FRopeSimState& Sim, const FContext& Ctx,
	FVector& InOutSurfaceWorld, FVector& InOutNormalWorld) const
{
	FRopeSurfaceProjection BestProjection;
	bool bFound = false;

	const float QueryRadius = FMath::Max(FMath::Max(Ctx.Config.ContactRadius, Ctx.SurfaceOffset), Sim.SegmentLength);
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
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		BuiltPathMaxDistance = FMath::Max(BuiltPathMaxDistance, Anchor.RopeDistance);
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
	if (State.Anchors.Num() == 0)
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

	const float SampleDistance = FMath::Max(0.0f, DistanceFromLatch);
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

	const float Alpha = FMath::Clamp(
		(SampleDistance - LowerAnchor->RopeDistance) / (UpperAnchor->RopeDistance - LowerAnchor->RopeDistance),
		0.0f,
		1.0f);
	OutPoint.SurfaceWorld = FMath::Lerp(LowerPoint.SurfaceWorld, UpperPoint.SurfaceWorld, Alpha);
	OutPoint.NormalWorld = FMath::Lerp(LowerPoint.NormalWorld, UpperPoint.NormalWorld, Alpha)
		.GetSafeNormal(KINDA_SMALL_NUMBER, LowerPoint.NormalWorld);
	OutPoint.TangentWorld = FMath::Lerp(LowerPoint.TangentWorld, UpperPoint.TangentWorld, Alpha);
	OutPoint.TangentWorld = (OutPoint.TangentWorld - FVector::DotProduct(OutPoint.TangentWorld, OutPoint.NormalWorld) * OutPoint.NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, LowerPoint.TangentWorld);
	OutPoint.Bone = Alpha < 0.5f ? LowerPoint.Bone : UpperPoint.Bone;
	OutPoint.Mesh = Alpha < 0.5f ? LowerPoint.Mesh : UpperPoint.Mesh;
	OutPoint.DistanceFromLatch = SampleDistance;
	return true;
}
