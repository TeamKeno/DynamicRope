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
	TSet<int32> AnchorNodes;
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
		{
			AnchorNodes.Add(Anchor.NodeIndex);
		}
	}

	OutFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		OutFrame.SetInvMass(i, (bStartPin || AnchorNodes.Contains(i)) ? 0.0f : 1.0f);
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
			: InitializeSurfaceVectorFieldProgressiveWrapPath(StoredLatchAnchor, Ctx);
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

bool FRopeWrappingPhase::InitializeSurfaceVectorFieldProgressiveWrapPath(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx)
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

	FRopeWrapPathPoint LatchPoint;
	LatchPoint.SurfaceWorld = State.PathSurfaceWorld;
	LatchPoint.NormalWorld = State.PathNormalWorld;
	LatchPoint.TangentWorld = State.PathTangentWorld;
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
			if (!ProjectWrapPointToSurface(State.LatchAnchor.Bone, Mesh, Sim, Ctx,
				State.PathSurfaceWorld, State.PathNormalWorld))
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
	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);

	FRopeSurfaceAnchor Anchor;
	Anchor.NodeIndex = NodeIndex;
	Anchor.Bone = LatchAnchor.Bone;
	Anchor.Mesh = Mesh;
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
	const float FrontSpeed = FullDistance / TotalDuration;
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
	OutPoint.DistanceFromLatch = SampleDistance;
	return true;
}
