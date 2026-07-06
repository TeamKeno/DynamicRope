// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrappingPhase.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
#include "Components/SkeletalMeshComponent.h"
#include "ProfilingDebugging/CpuProfilerTrace.h" // TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "RopeMathHelpers.h" // RopeMath::AnyTangentFromNormal (unity 빌드 중복 정의 방지)

bool FRopeWrappingPhase::Begin(const FRopeSurfaceAnchor& LatchAnchor, const USkeletalMeshComponent* Mesh, FName Bone,
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
	State.LostContactTime = 0.0f;
	State.LastStableFirstNode = State.FirstNode;
	State.LastStableLastNode = State.LastNode;
	State.LastStableAnchorCount = State.Anchors.Num();
	return true;
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

	const int32 StepBudget = FMath::Max(1, Ctx.Config.WrappingPathBuildStepsPerFrame);
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
			State.bPathBuildComplete = true;
			State.bPathBuildActive = false;
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
	const int32 TailEndNode = Sim.Num() - 1;

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
	float MinRequiredTurns, float& OutTurns) const
{
	OutTurns = 0.0f;
	if (MinRequiredTurns <= 0.0f || !State.bPathBuildFailed)
	{
		return false;
	}

	if (!ComputeHelixTurnsAtLastBuiltPoint(Sim, Ctx, OutTurns))
	{
		return false;
	}

	return OutTurns < MinRequiredTurns;
}

FRopeWrapState FRopeWrappingPhase::BuildCommitSeed(const FRopeSimState& Sim, const USkeletalMeshComponent* Mesh) const
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

	return Seed;
}

void FRopeWrappingPhase::ReturnNodesToSolver(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const
{
	OutFrame.EnsureSize(Sim.Num());
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (!Sim.InvMass.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.Positions.IsValidIndex(Anchor.NodeIndex))
		{
			continue;
		}

		OutFrame.SetInvMass(Anchor.NodeIndex, 1.0f);
		OutFrame.SetPrevFromPosition(Anchor.NodeIndex);
	}
}

bool FRopeWrappingPhase::BuildPreviewCenterline(const FRopeSurfaceAnchor& LatchAnchor, const USkeletalMeshComponent* Mesh, FName Bone,
	const FRopeSimState& Sim, const FContext& Ctx, TArray<FVector>& OutCenterline) const
{
	OutCenterline.Reset();
	if (!Mesh || Bone.IsNone() || !Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) || Sim.Num() < 2)
	{
		return false;
	}

	FRopeWrapConfig PreviewConfig = Ctx.Config;
	PreviewConfig.WrappingPathBuildStepsPerFrame = 4096;
	const FContext PreviewCtx{ PreviewConfig, Ctx.Colliders, Ctx.PathMode, Ctx.SurfaceOffset, Ctx.OwnerName };

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
	for (int32 PathIndex = 0; PathIndex < PreviewPhase.State.Path.Num(); ++PathIndex)
	{
		const int32 NodeIndex = LatchAnchor.NodeIndex + PathIndex;
		if (!OutCenterline.IsValidIndex(NodeIndex))
		{
			break;
		}

		const FRopeWrapPathPoint& Point = PreviewPhase.State.Path[PathIndex];
		OutCenterline[NodeIndex] = Point.SurfaceWorld + Point.NormalWorld * SurfaceOffset;
	}

	return OutCenterline.Num() >= 2;
}

bool FRopeWrappingPhase::BeginProgressiveWrapPathBuild(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_BeginProgressiveWrapPathBuild);

	const USkeletalMeshComponent* Mesh = State.Mesh.Get();
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
	State.LastAnchoredPathPointCount = 0;
	State.bPathBuildActive = true;
	State.bPathBuildComplete = false;
	State.bPathBuildFailed = false;
	State.PathCurrentDistance = 0.0f;
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
		State.bPathBuildFailed = true;
		State.bPathBuildComplete = true;
		State.bPathBuildActive = false;
		return false;
	}

	State.Path.Add(Point);
	if (State.Path.Num() >= State.NumTailNodes)
	{
		State.bPathBuildComplete = true;
		State.bPathBuildActive = false;
	}
	return true;
}

bool FRopeWrappingPhase::InitializeSurfaceVectorFieldProgressiveWrapPath(const FRopeSurfaceAnchor& LatchAnchor,
	const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_InitSurfaceVectorFieldProgressivePath);

	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = State.Mesh.Get();
	}
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	if (!ResolveWrappingAxis(LatchAnchor, State.PathAxisOrigin, State.PathAxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, State.PathAxisDirection);

	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);
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
	State.PathWindingSign =
		FVector::DotProduct(State.PathCircumferenceDir, LatchTangentWorld) < 0.0f ? -1.0f : 1.0f;
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
		State.bPathBuildComplete = true;
		State.bPathBuildActive = false;
	}

	return true;
}

bool FRopeWrappingPhase::AdvanceSurfaceVectorFieldProgressiveWrapPath(int32 StepBudget, const FRopeSimState& Sim, const FContext& Ctx)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_AdvanceSurfaceVectorFieldProgressivePath);

	const USkeletalMeshComponent* Mesh = State.LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = State.Mesh.Get();
	}
	if (!Mesh || State.LatchAnchor.Bone.IsNone())
	{
		State.bPathBuildFailed = true;
		State.bPathBuildComplete = true;
		State.bPathBuildActive = false;
		return false;
	}

	const float StepSize = FMath::Max(1.0f, Sim.SegmentLength * 0.5f);
	int32 StepsRemaining = FMath::Max(1, StepBudget);

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

			State.PathSurfaceWorld += State.PathTangentWorld * StepDistance;
			const FName CurrentBone = State.PathCurrentBone.IsNone()
				? State.LatchAnchor.Bone
				: State.PathCurrentBone;
			const USkeletalMeshComponent* ProjectedMesh = State.PathCurrentMesh.Get();
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
			FName ProjectedBone = CurrentBone;
			if (!ProjectWrapPointToSurfaceMultiBone(CurrentBone, Mesh, Sim, Ctx,
				State.PathPreviousBone, State.PathDistanceSinceBoneTransition, RopeNodeWorld,
				State.PathNormalWorld, State.PathTangentWorld,
				State.PathSurfaceWorld, State.PathNormalWorld, State.PathTangentWorld,
				State.PathCircumferenceDir, ProjectedBone, ProjectedMesh))
			{
				State.bPathBuildFailed = true;
				State.bPathBuildComplete = true;
				State.bPathBuildActive = false;
				UE_LOG(LogDynamicRope, Log,
					TEXT("[%s] Progressive wrap path stopped by projection failure (bone=%s, path=%d/%d, anchors=%d)"),
					*Ctx.OwnerName,
					*State.LatchAnchor.Bone.ToString(),
					State.Path.Num(),
					State.NumTailNodes,
					State.Anchors.Num());
				return false;
			}

			// ProjectWrapPointToSurfaceMultiBone이 hysteresis까지 적용해 최종 본을 돌려준다.
			// 여기서는 상태만 갱신한다. 전환했다면 직전 본을 기록해 다음 step에서 바로 되돌아가는 후보에
			// penalty를 줄 수 있게 하고, 전환 거리 누적은 0으로 다시 시작한다.
			if (ProjectedBone != CurrentBone)
			{
				State.PathPreviousBone = CurrentBone;
				State.PathCurrentBone = ProjectedBone;
				State.PathDistanceSinceBoneTransition = 0.0f;
			}
			else
			{
				State.PathCurrentBone = ProjectedBone;
				State.PathDistanceSinceBoneTransition += StepDistance;
			}
			State.PathCurrentMesh = ProjectedMesh;

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
		State.Path.Add(Point);
		AppendWrappingAnchorFromPathPoint(PathIndex, Sim, Ctx);

		if (!bConsumedStep)
		{
			--StepsRemaining;
		}
	}

	if (State.Path.Num() >= State.NumTailNodes)
	{
		State.bPathBuildComplete = true;
		State.bPathBuildActive = false;
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
	const USkeletalMeshComponent* Mesh = State.Mesh.Get();
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

	// MVP의 핵심: 경로점이 선택한 본을 그대로 anchor 소유 본으로 사용한다.
	// 이전 구현은 모든 anchor를 LatchAnchor.Bone 로컬로 저장했기 때문에,
	// path가 이웃 본 표면으로 넘어가더라도 Wrapped/Hold 단계에서는 한 본에 고정되어 보였다.
	// Point.Bone이 비어 있는 경우는 AnalyticHelix/legacy fallback으로 보고 latch bone을 사용한다.
	FName AnchorBone = Point.Bone.IsNone() ? LatchAnchor.Bone : Point.Bone;
	const USkeletalMeshComponent* AnchorMesh = Point.Mesh.Get();
	if (!AnchorMesh)
	{
		AnchorMesh = Mesh;
	}
	if (!AnchorMesh || AnchorBone.IsNone())
	{
		return false;
	}

	const FTransform BoneXform = AnchorMesh->GetSocketTransform(AnchorBone);

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
	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
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
	if (!ResolveWrappingAxis(LatchAnchor, AxisOrigin, AxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, AxisDirection);

	//3. latch anchor를 world 좌표로 복원
	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);
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
	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
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
	if (!ResolveWrappingAxis(LatchAnchor, AxisOrigin, AxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, AxisDirection);

	//3. latch anchor를 world 좌표로 복원
	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);
	//LatchSurfaceWorld가 나선의 시작점
	const FVector LatchSurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);

	//4. LatchNormalWorld normal / tangent 복원
	//LatchNormalWorld는 접촉 표면 normal.
	//LatchTangentWorld는 최초 latch 순간 로프가 tail 방향으로 뻗던 방향
	const FVector LatchNormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector LatchTangentWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalTangent);
	//tangent를 normal plane에 투영해
	LatchTangentWorld = (LatchTangentWorld - FVector::DotProduct(LatchTangentWorld, LatchNormalWorld) * LatchNormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(LatchNormalWorld));

	//5. latch 점을 축 기준으로 분해
	const float LatchAxisDistance = FVector::DotProduct(LatchSurfaceWorld - AxisOrigin, AxisDirection);//먼저 latch point가 축 위에서 어느 높이에 있는지 구함:
	const FVector LatchAxisPoint = AxisOrigin + AxisDirection * LatchAxisDistance;//그 축 위의 점:
	FVector LatchRadial = LatchSurfaceWorld - LatchAxisPoint; //축에서 latch point로 향하는 radial:
	const float HelixRadius = LatchRadial.Size();	// latch 지점에서 bone - parent 축까지의 거리(반지름:)
	if (HelixRadius <= KINDA_SMALL_NUMBER)
	{
		return false;
	}
	LatchRadial /= HelixRadius;	//정규화:
	//latch 지점이 축에서 얼마나 떨어져 있는지를 HelixRadius로 잡는 거야

	//6. 감기는 방향 결정
	FVector CircumferenceDir = FVector::CrossProduct(AxisDirection, LatchRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(LatchNormalWorld));
	//최초 latch rope tangent방향과 외적이 일치하나 안하냐.
	const float WindingSign = FVector::DotProduct(CircumferenceDir, LatchTangentWorld) < 0.0f ? -1.0f : 1.0f;
	CircumferenceDir *= WindingSign;

	//7. rope 거리 d를 나선 파라미터로 변환
	const float PitchScale = Ctx.Config.WrappingHelixPitchScale;
	const float LengthScale = FMath::Sqrt(1.0f + PitchScale * PitchScale);
	const float CircumferenceDistance = DistanceFromLatch / FMath::Max(LengthScale, KINDA_SMALL_NUMBER);

	//축 방향 이동량
	const float AxisDistance = CircumferenceDistance * PitchScale;

	//원주 회전량
	const float AngleRadians = WindingSign * CircumferenceDistance / FMath::Max(HelixRadius, KINDA_SMALL_NUMBER);

	//8. 축을 중심으로 radial 벡터를 회전시켜 나선 위의 점을 만듦 = 나선점 생성
	//축을 중심으로 LatchRadial 방향을 AngleRadians만큼 돌린다
	const FQuat AxisRotation(AxisDirection, AngleRadians);
	const FVector RotatedRadial = AxisRotation.RotateVector(LatchRadial).GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);	//원주 중의 한 점
	//TargetAxisPoint = 축 위에서, latch 높이보다 AxisDistance만큼 이동한 점
	const FVector TargetAxisPoint = AxisOrigin + AxisDirection * (LatchAxisDistance + AxisDistance);	//축 이동
	//// 축 위 중심점에서 바깥 방향으로 반지름만큼 나가면 나선 위의 world 위치가 된다.
	FVector SurfaceWorld = TargetAxisPoint + RotatedRadial * HelixRadius;

	//9. SDF 표면에 붙이기
	FVector NormalWorld = RotatedRadial;
	if (!ProjectWrapPointToSurface(LatchAnchor.Bone, Mesh, Sim, Ctx, SurfaceWorld, NormalWorld))	//마지막으로 SDF 표면에 붙임
	{
		return false;
	}

	//10. 보정된 표면에서 tangent 다시 계산
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

bool FRopeWrappingPhase::ComputeHelixTurnsAtLastBuiltPoint(const FRopeSimState& Sim, const FContext& Ctx, float& OutTurns) const
{
	OutTurns = 0.0f;
	if (State.Anchors.Num() == 0 && State.Path.Num() == 0)
	{
		return false;
	}

	const FRopeSurfaceAnchor& LatchAnchor = State.LatchAnchor;
	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
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
	if (!ResolveWrappingAxis(LatchAnchor, AxisOrigin, AxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, AxisDirection);

	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);
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

	// 실제 SurfaceVectorField 경로가 얼마나 울퉁불퉁했는지와 별개로, 실패 판정은 helix 기준 누적 회전량만 본다.
	const float PitchScale = Ctx.Config.WrappingHelixPitchScale;
	const float LengthScale = FMath::Sqrt(1.0f + PitchScale * PitchScale);
	const float CircumferenceDistance = LastBuiltDistance / FMath::Max(LengthScale, KINDA_SMALL_NUMBER);
	const float AngleRadians = CircumferenceDistance / FMath::Max(HelixRadius, KINDA_SMALL_NUMBER);
	OutTurns = FMath::Abs(AngleRadians) / (2.0f * PI);
	return true;
}

/* 감김 축 정의 */
bool FRopeWrappingPhase::ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor,
	FVector& OutAxisOrigin, FVector& OutAxisDirection) const
{
	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = State.Mesh.Get();
	}
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	const FName ParentBone = Mesh->GetParentBone(LatchAnchor.Bone);
	const FVector BoneLocation = Mesh->GetSocketTransform(LatchAnchor.Bone).GetLocation();
	if (!ParentBone.IsNone())
	{
		const FVector ParentLocation = Mesh->GetSocketTransform(ParentBone).GetLocation();
		const FVector Axis = BoneLocation - ParentLocation;
		if (!Axis.IsNearlyZero())
		{
			OutAxisOrigin = ParentLocation;
			OutAxisDirection = Axis.GetSafeNormal();
			return true;
		}
	}

	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);
	OutAxisOrigin = BoneLocation;
	OutAxisDirection = BoneXform.GetUnitAxis(EAxis::X).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	return true;
}

void FRopeWrappingPhase::OrientWrappingAxisByTail(const FRopeSurfaceAnchor& LatchAnchor, const FRopeSimState& Sim,
	const USkeletalMeshComponent* Mesh, FVector& InOutAxisDirection) const
{
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return;
	}

	const FName ParentBone = Mesh->GetParentBone(LatchAnchor.Bone);
	if (ParentBone.IsNone())
	{
		return;
	}

	float ParentScore = 0.0f;
	float BoneScore = 0.0f;
	float TotalWeight = 0.0f;
	const FVector ParentWorld = Mesh->GetSocketTransform(ParentBone).GetLocation();
	const FVector BoneWorld = Mesh->GetSocketTransform(LatchAnchor.Bone).GetLocation();

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

void FRopeWrappingPhase::GatherSurfaceVectorFieldBoneCandidates(FName CurrentBone, const USkeletalMeshComponent* Mesh,
	TArray<FSurfaceVectorFieldBoneCandidate>& OutCandidates, const FContext& Ctx) const
{
	OutCandidates.Reset();
	if (!Mesh || CurrentBone.IsNone())
	{
		return;
	}

	struct FBoneQueueEntry
	{
		FName Bone = NAME_None;
		int32 Depth = 0;
		float Cost = 0.0f;
	};

	const int32 NumBones = Mesh->GetNumBones();
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

		AddNeighbor(Mesh->GetParentBone(Entry.Bone));

		for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex)
		{
			const FName BoneName = Mesh->GetBoneName(BoneIndex);
			if (!BoneName.IsNone() && Mesh->GetParentBone(BoneName) == Entry.Bone)
			{
				AddNeighbor(BoneName);
			}
		}
	}
}

bool FRopeWrappingPhase::ProjectWrapPointToSurfaceMultiBone(FName CurrentBone, const USkeletalMeshComponent* Mesh,
	const FRopeSimState& Sim, const FContext& Ctx,
	FName PreviousBone, float DistanceSinceLastTransition, const FVector& RopeNodeWorld,
	const FVector& PreviousNormalWorld, const FVector& PreviousTangentWorld,
	FVector& InOutSurfaceWorld, FVector& InOutNormalWorld, FVector& InOutTangentWorld,
	FVector& InOutCircumferenceDir, FName& InOutBone, const USkeletalMeshComponent*& OutMesh) const
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
		const USkeletalMeshComponent* Mesh = nullptr;
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

bool FRopeWrappingPhase::ProjectWrapPointToSurface(FName Bone, const USkeletalMeshComponent* Mesh,
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
		const USkeletalMeshComponent* Mesh = Anchor.Mesh.Get();
		if (!Mesh)
		{
			Mesh = State.Mesh.Get();
		}
		if (!Mesh || Anchor.Bone.IsNone())
		{
			return false;
		}

		const FTransform BoneXform = Mesh->GetSocketTransform(Anchor.Bone);
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
