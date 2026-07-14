// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeFlightContactDetector.h"
#include "Collision/RopeCollider.h"
// ResolveBindingWorld + RopeWrapTargets:: 구조 질의(스켈레탈 가정 격리 지점).
#include "Core/RopeWrapTarget.h"
#include "Components/SkeletalMeshComponent.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
// sin(30deg): 축-평면 각도 30도 이내면 miss.
constexpr float MaxBoneAxisPlaneNormalDotForMiss = 0.5f;

bool BypassCaptureQualityGateForNow()
{
	// 지금은 접촉 판정 폴리싱 전이라 항상 통과시킨다.
	// volatile로 읽어 아래 스캐폴드 계산식이 unreachable code 경고로 죽지 않게 둔다.
	static volatile bool bBypassCaptureQualityGate = true;
	return bBypassCaptureQualityGate;
}

void KeepLargestPlaneNormal(const FVector& A, const FVector& B, FVector& InOutNormal, float& InOutSizeSq)
{
	const FVector Normal = FVector::CrossProduct(A, B);
	const float SizeSq = Normal.SizeSquared();
	if (SizeSq > InOutSizeSq)
	{
		InOutNormal = Normal;
		InOutSizeSq = SizeSq;
	}
}

FVector ComputeRopeSplinePlaneNormal(const FRopeSimState& Sim, int32 NodeIndex, const FVector& RelativeVelocity)
{
	if (!Sim.Positions.IsValidIndex(NodeIndex))
	{
		return FVector::ZeroVector;
	}

	const FVector Origin = Sim.Positions[NodeIndex];
	const FVector ToHead = Sim.Positions.IsValidIndex(0) ? Sim.Positions[0] - Origin : FVector::ZeroVector;
	const FVector ToTail = Sim.Positions.Num() > 0 ? Sim.Positions.Last() - Origin : FVector::ZeroVector;
	const FVector ToPrev = Sim.Positions.IsValidIndex(NodeIndex - 1) ? Sim.Positions[NodeIndex - 1] - Origin : FVector::ZeroVector;
	const FVector ToNext = Sim.Positions.IsValidIndex(NodeIndex + 1) ? Sim.Positions[NodeIndex + 1] - Origin : FVector::ZeroVector;

	FVector BestNormal = FVector::ZeroVector;
	float BestSizeSq = 0.0f;

	// 로프/스플라인이 그리는 평면을 현재 곡선 형태에서 우선 추정하고, 거의 일직선이면 이동 방향까지 보조로 쓴다.
	KeepLargestPlaneNormal(ToHead, ToTail, BestNormal, BestSizeSq);
	KeepLargestPlaneNormal(ToPrev, ToNext, BestNormal, BestSizeSq);
	KeepLargestPlaneNormal(ToHead, RelativeVelocity, BestNormal, BestSizeSq);
	KeepLargestPlaneNormal(ToTail, RelativeVelocity, BestNormal, BestSizeSq);

	return BestNormal.GetSafeNormal();
}

FVector ComputeBoneParentAxis(const FRopeContactCandidate& Candidate)
{
	if (!Candidate.Mesh || Candidate.Bone.IsNone())
	{
		return FVector::ZeroVector;
	}

	// bone→parent 축은 본 그래프가 있는 대상(스켈레탈)에서만 정의된다. 정적 대상(부모 키 None)이면
	// 축 없음(ZeroVector).
	const FName ParentBone = RopeWrapTargets::GetParentTargetKey(Candidate.Mesh, Candidate.Bone);
	if (ParentBone.IsNone())
	{
		return FVector::ZeroVector;
	}

	const FVector BoneLocation = ResolveBindingWorld(Candidate.Mesh, Candidate.Bone).GetLocation();
	const FVector ParentLocation = ResolveBindingWorld(Candidate.Mesh, ParentBone).GetLocation();
	return (BoneLocation - ParentLocation).GetSafeNormal();
}

bool IsBoneAxisNearlyParallelToRopePlane(const FRopeSimState& Sim, const FRopeContactCandidate& Candidate,
	const FVector& RelativeVelocity)
{
	const FVector RopePlaneNormal = ComputeRopeSplinePlaneNormal(Sim, Candidate.NodeIndex, RelativeVelocity);
	const FVector BoneAxis = ComputeBoneParentAxis(Candidate);
	if (RopePlaneNormal.IsNearlyZero() || BoneAxis.IsNearlyZero())
	{
		return false;
	}

	const float AxisPlaneNormalDot = FMath::Abs(FVector::DotProduct(BoneAxis, RopePlaneNormal));
	return AxisPlaneNormalDot <= MaxBoneAxisPlaneNormalDotForMiss;
}
}

void FRopeFlightContactDetector::DetectContactCandidates(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
	const FParams& Params, TArray<FRopeContactCandidate>& OutCandidates)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightDetectContactCandidates);
	TArray<IRopeCollider*> NearbyColliders;
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!Sim.PrevPositions.IsValidIndex(i) || !Sim.Positions.IsValidIndex(i))
		{
			continue;
		}

		bool bFast = IsTailNode(Sim, i) || Sim.NodeSpeed(i) > Sim.SegmentLength;
		GatherNearbyColliders(Sim.PrevPositions[i], Sim.Positions[i], Colliders, Params, NearbyColliders);
		bool bNearBody = NearbyColliders.Num() > 0;

		if (!bFast && !bNearBody)
			continue;
		if (!bNearBody)
			continue;

		// 현재 위치만 보지 않고 이동 경로를 본다.
		// 캡슐은 segment-vs-capsule로 가능.
		// SDF는 path를 몇 개 샘플링하거나 SweepQuery adapter가 필요.
		FRopeContact Contact = SweepOrSampleContact(Sim, Sim.PrevPositions[i], Sim.Positions[i], NearbyColliders, Params);

		if (Contact.bHit)
		{
			FRopeContactCandidate Candidate = MakeCandidate(i, Contact);
			Candidate.Source = ERopeContactCandidateSource::Actual;
			Candidate.SourceMask = static_cast<uint8>(Candidate.Source);
			OutCandidates.Add(Candidate);
		}
	}
}

void FRopeFlightContactDetector::AddPredictedContactCandidates(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
	const FParams& Params, const FWhipGuideView& Whip, TArray<FRopeContactCandidate>& InOutCandidates)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightAddPredictedContactCandidates);
	const float PredictionFrames = FMath::Max(0.0f, Params.PredictiveContactFrames);
	if (PredictionFrames <= KINDA_SMALL_NUMBER || Sim.Num() == 0)
	{
		return;
	}

	auto AddUniqueCandidate = [&InOutCandidates](const FRopeContactCandidate& Candidate)
	{
		for (FRopeContactCandidate& Existing : InOutCandidates)
		{
			if (Existing.NodeIndex == Candidate.NodeIndex && Existing.Bone == Candidate.Bone && Existing.Mesh == Candidate.Mesh)
			{
				Existing.SourceMask |= Candidate.SourceMask;
				if (Candidate.Source == ERopeContactCandidateSource::PredictiveGuided ||
					(Existing.Source == ERopeContactCandidateSource::Actual && Candidate.Source == ERopeContactCandidateSource::PredictiveFree))
				{
					Existing.Source = Candidate.Source;
				}
				return;
			}
		}

		InOutCandidates.Add(Candidate);
	};

	TArray<IRopeCollider*> NearbyColliders;
	const bool bHasGuidedNodes = Whip.HasGuidedNodes();

	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!Sim.Positions.IsValidIndex(i) || !Sim.PrevPositions.IsValidIndex(i))
		{
			continue;
		}

		FVector CurrentPosition = Sim.Positions[i];
		FVector PredictedPosition = CurrentPosition;
		const FVector FrameDisplacement = Sim.Displacement(i);
		if (bHasGuidedNodes && !ShouldRunPredictiveContactForNode(Sim, Whip, i, FrameDisplacement))
		{
			continue;
		}

		const bool bGuidedNode = bHasGuidedNodes && Whip.IsGuidedNode(i);
		const bool bTailNode = IsTailNode(Sim, i);
		const bool bFastNode = FrameDisplacement.Size() > Sim.SegmentLength;

		bool bFastEnoughForPrediction = false;
		ERopeContactCandidateSource Source = ERopeContactCandidateSource::PredictiveFree;

		if (bGuidedNode && Whip.CurrentTargets && Whip.CurrentTargets->IsValidIndex(i))
		{
			Source = ERopeContactCandidateSource::PredictiveGuided;
			CurrentPosition = (*Whip.CurrentTargets)[i];
			if (Whip.NextTargets && Whip.NextTargets->IsValidIndex(i))
			{
				PredictedPosition = CurrentPosition + ((*Whip.NextTargets)[i] - CurrentPosition) * PredictionFrames;
			}
			else
			{
				const FVector PrevGuidePosition = (Whip.PrevTargets && Whip.PrevTargets->IsValidIndex(i))
					? (*Whip.PrevTargets)[i]
					: Sim.PrevPositions[i];
				PredictedPosition = CurrentPosition + (CurrentPosition - PrevGuidePosition) * PredictionFrames;
			}

			bFastEnoughForPrediction = FVector::Dist(CurrentPosition, PredictedPosition) > KINDA_SMALL_NUMBER;
		}
		else
		{
			PredictedPosition = CurrentPosition + FrameDisplacement * PredictionFrames;
			bFastEnoughForPrediction = bTailNode || bFastNode;
		}

		GatherNearbyColliders(CurrentPosition, PredictedPosition, Colliders, Params, NearbyColliders);
		const bool bPredictedPathNearBody = NearbyColliders.Num() > 0;
		if (!bFastEnoughForPrediction && !bPredictedPathNearBody)
		{
			continue;
		}
		if (!bPredictedPathNearBody)
		{
			continue;
		}

		// If a bone surface exists between the current node position and predicted next position,
		// promote it to the same candidate path that later builds the latch seed.
		const FRopeContact Contact = SweepOrSampleContact(Sim, CurrentPosition, PredictedPosition, NearbyColliders, Params);
		if (!Contact.bHit || Contact.Bone.IsNone())
		{
			continue;
		}

		FRopeContactCandidate Candidate = MakeCandidate(i, Contact);
		Candidate.Source = Source;
		Candidate.SourceMask = static_cast<uint8>(Source);
		AddUniqueCandidate(Candidate);
	}
}

void FRopeFlightContactDetector::EvaluateRelativeMotion(const FRopeSimState& Sim, const FParams& Params,
	TArray<FRopeContactCandidate>& Candidates)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightEvaluateRelativeMotion);
	for (FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Sim.Positions.IsValidIndex(Candidate.NodeIndex) || !Sim.PrevPositions.IsValidIndex(Candidate.NodeIndex))
		{
			continue;
		}

		// RopeVelocity는 Verlet 차분 = *마지막 substep*의 변위(cm/substep, ≈ v·FixedDt), SurfaceVelocity는 FROZEN
		// 계약상 cm/초. 같은 단위로 빼려면 표면속도를 *substep dt*(SubstepDeltaTime=FixedDt)로 환산해야 한다 —
		// 프레임 dt로 환산하면 Substeps>1일 때 표면속도가 Substeps배 과대 반영돼 상대운동 방향/속도가 틀린다.
		// 솔버 마찰(ApplyContactFriction)이 SubDt로 환산하는 것과 동일한 원칙.
		const FVector RopeVelocity = Sim.Positions[Candidate.NodeIndex] - Sim.PrevPositions[Candidate.NodeIndex];
		const FVector RelativeVelocity = RopeVelocity - Candidate.SurfaceVelocity * Params.SubstepDeltaTime;
		const FVector TangentVelocity = RelativeVelocity - FVector::DotProduct(RelativeVelocity, Candidate.Normal) * Candidate.Normal;

		Candidate.RelativeTangentialSpeed = TangentVelocity.Size();
		Candidate.WrapDirectionScore = FVector::DotProduct(TangentVelocity.GetSafeNormal(),
			ExpectedWrapTangent(Sim, Candidate, Params.FallbackForward));

		// 로프/스플라인 평면과 bone-parent 축이 거의 평행하면, 축을 따라 스치거나 찍는 접촉이라 감김 후보에서 제외한다.
		// 축-평면 각도 30도 이내를 miss cone으로 본다. 축이 평면에 수직에 가까울수록 실제 감김 후보로 남긴다.
		//if (IsBoneAxisNearlyParallelToRopePlane(Sim, Candidate, RelativeVelocity))
		//{
		//	Candidate.bValid = false;
		//	continue;
		//}
	}
}

bool FRopeFlightContactDetector::ShouldCapture(const TArray<FRopeContactCandidate>& Candidates, const FParams& Params)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightShouldCaptureImpl);
	FRopeContactTracker TempTracker;
	TempTracker.Update(Candidates, 0.0f);
	return TempTracker.CandidateNodes.Num() >= FMath::Max(1, Params.MinLatchNodes)
		&& IsWrappableBone(TempTracker.CandidateBone)
		&& PassesCaptureQualityGate(TempTracker, Candidates, Params);
}

bool FRopeFlightContactDetector::PassesCaptureQualityGate(const FRopeContactTracker& Tracker,
	const TArray<FRopeContactCandidate>& Candidates, const FParams& Params)
{
	if (BypassCaptureQualityGateForNow())
	{
		return true;
	}

	// 아래는 나중에 접촉 판정을 빡세게 만들 때 켤 재료들이다.
	// 지금은 감김 애니메이션/경로 폴리싱이 우선이라, 위 게이트에서 항상 통과시킨다.

	// 1. 접촉 노드 수/분포:
	//    MinLatchNodes는 이미 ShouldCapture에서 보고 있다. 여기에 더해 연속된 노드 구간인지,
	//    너무 한 점에만 몰린 접촉인지, head/tail 중 어느 쪽 접촉인지 볼 수 있다.
	const int32 ContactNodeCount = Tracker.CandidateNodes.Num();
	const bool bHasEnoughNodes = ContactNodeCount >= FMath::Max(1, Params.MinLatchNodes);
	TArray<int32> SortedContactNodes = Tracker.CandidateNodes;
	SortedContactNodes.Sort();
	int32 UniqueContactNodeCount = 0;
	int32 FirstContactNode = INDEX_NONE;
	int32 LastContactNode = INDEX_NONE;
	int32 LongestContinuousRun = 0;
	int32 CurrentContinuousRun = 0;
	int32 PreviousNode = INDEX_NONE;
	for (const int32 NodeIndex : SortedContactNodes)
	{
		if (NodeIndex == PreviousNode)
		{
			continue;
		}

		++UniqueContactNodeCount;
		FirstContactNode = (FirstContactNode == INDEX_NONE) ? NodeIndex : FirstContactNode;
		LastContactNode = NodeIndex;
		CurrentContinuousRun = (PreviousNode != INDEX_NONE && NodeIndex == PreviousNode + 1)
			? CurrentContinuousRun + 1
			: 1;
		LongestContinuousRun = FMath::Max(LongestContinuousRun, CurrentContinuousRun);
		PreviousNode = NodeIndex;
	}
	const int32 ContactSpan = (FirstContactNode != INDEX_NONE && LastContactNode != INDEX_NONE)
		? LastContactNode - FirstContactNode + 1
		: 0;
	const float ContactDensity = ContactSpan > 0
		? static_cast<float>(UniqueContactNodeCount) / static_cast<float>(ContactSpan)
		: 0.0f;

	float MaxPenetration = 0.0f;
	float TotalPenetration = 0.0f;
	float MaxTangentialSpeed = 0.0f;
	float TotalTangentialSpeed = 0.0f;
	float TotalWrapDirectionHint = 0.0f;
	bool bHasActualContact = false;
	bool bHasPredictiveContact = false;
	bool bAllCandidatesOnDominantTarget = true;
	int32 DominantCandidateCount = 0;

	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid || Candidate.Bone != Tracker.CandidateBone)
		{
			continue;
		}

		++DominantCandidateCount;

		// 2. 침투 깊이:
		//    감김은 접선 움직임이 핵심이라 얕은 침투를 컷 기준으로 쓰면 좋은 스침까지 버릴 수 있다.
		//    Penetration은 후보가 여러 개일 때 표면점 신뢰도/타이브레이커로 쓰고, 너무 깊은 값만
		//    관통 또는 보정 실패 의심 신호로 낮은 신뢰도를 주는 정도가 적당하다.
		MaxPenetration = FMath::Max(MaxPenetration, Candidate.Penetration);
		TotalPenetration += Candidate.Penetration;

		// 3. 상대 접선 속도:
		//    스쳤는지의 핵심 값. 표면 속도 SurfaceVelocity를 뺀 상대 접선 속도라 움직이는 본 위에서도 쓸 수 있다.
		MaxTangentialSpeed = FMath::Max(MaxTangentialSpeed, Candidate.RelativeTangentialSpeed);
		TotalTangentialSpeed += Candidate.RelativeTangentialSpeed;

		// 4. 감김 방향 선택 힌트:
		//    캡처 타이밍은 Flight이고, 이때 로프는 아직 풀려 있는 상태라 방향 판정은 컷 조건이 아니다.
		//    parent 쪽/child 쪽 둘 다 감길 수 있어야 하므로, 이 값은 Wrapping 시작 시 winding sign을
		//    고르는 힌트로만 쓴다. 현재 WrapDirectionScore는 손 방향 기준이라 나중에 재정의가 필요하다.
		TotalWrapDirectionHint += Candidate.WrapDirectionScore;

		// 5. 후보 출처:
		//    Actual은 실제 접촉, PredictiveFree/PredictiveGuided는 예측 접촉이다.
		//    예측 후보만 있을 때는 기준을 높이고, Actual이 섞이면 빠르게 잡는 식으로 조절할 수 있다.
		bHasActualContact |= (Candidate.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::Actual)) != 0;
		bHasPredictiveContact |=
			(Candidate.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)) != 0 ||
			(Candidate.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided)) != 0;

		// 6. dominant bone/mesh 일관성:
		//    같은 bone이라도 mesh가 섞이면 cross-actor 상황에서 잘못된 후보일 수 있다.
		//    Tracker.CandidateBone/CandidateMesh와 후보들의 Bone/Mesh 일치를 확인한다.
		bAllCandidatesOnDominantTarget &= Candidate.Mesh == Tracker.CandidateMesh;
	}

	const float AveragePenetration = DominantCandidateCount > 0
		? TotalPenetration / static_cast<float>(DominantCandidateCount)
		: 0.0f;
	const float AverageTangentialSpeed = DominantCandidateCount > 0
		? TotalTangentialSpeed / static_cast<float>(DominantCandidateCount)
		: 0.0f;
	const float AverageWrapDirectionHint = DominantCandidateCount > 0
		? TotalWrapDirectionHint / static_cast<float>(DominantCandidateCount)
		: 0.0f;
	const float SourceConfidence =
		(bHasActualContact ? 1.0f : 0.0f) +
		(bHasPredictiveContact ? 0.5f : 0.0f);

	// 실제로 기준을 켤 때는 아래처럼 "접선 운동이 있는가"를 중심으로 보고,
	// 침투 깊이와 방향 힌트는 보조값으로만 쓰는 편이 안전하다.
	const bool bDistributionLooksLikeRopeSpan =
		LongestContinuousRun >= FMath::Max(1, Params.MinLatchNodes) ||
		(ContactDensity >= 0.5f && ContactNodeCount >= FMath::Max(1, Params.MinLatchNodes));
	const bool bHasTangentialMotion = MaxTangentialSpeed > KINDA_SMALL_NUMBER || AverageTangentialSpeed > KINDA_SMALL_NUMBER;
	const bool bPenetrationLooksUsable = MaxPenetration > 0.0f && AveragePenetration >= 0.0f;
	const bool bHasUsableSource = SourceConfidence > 0.0f;
	const bool bDirectionHintIsNotACut =
		AverageWrapDirectionHint > -TNumericLimits<float>::Max() &&
		AverageWrapDirectionHint < TNumericLimits<float>::Max();

	// 7. 아직 후보에 없는 추가 재료:
	//    정면 충돌을 더 정확히 거르려면 RopeVelocity와 Normal의 내적(법선 접근 속도)을 후보에 저장하면 좋다.
	//    세워진 책 가운데를 때리는 상황은 접선 속도/감김 방향이 낮고 법선 접근 성분이 큰 접촉으로 분리할 수 있다.
	//    로프 세그먼트 방향과 표면 접선/본 축의 내적도 "걸쳐짐"과 "찍힘"을 구분하는 데 쓸 수 있다.

	return bHasEnoughNodes &&
		bDistributionLooksLikeRopeSpan &&
		bAllCandidatesOnDominantTarget &&
		bHasUsableSource &&
		bHasTangentialMotion &&
		bPenetrationLooksUsable &&
		bDirectionHintIsNotACut;
}

bool FRopeFlightContactDetector::IsTailNode(const FRopeSimState& Sim, int32 NodeIndex)
{
	return NodeIndex >= FMath::Max(1, Sim.Num() - 4);
}

bool FRopeFlightContactDetector::IsNearAnyColliderSegment(const FVector& PrevPosition, const FVector& Position,
	const TArray<IRopeCollider*>& Colliders, const FParams& Params)
{
	TArray<IRopeCollider*> NearbyColliders;
	GatherNearbyColliders(PrevPosition, Position, Colliders, Params, NearbyColliders);
	return NearbyColliders.Num() > 0;
}

void FRopeFlightContactDetector::GatherNearbyColliders(const FVector& PrevPosition, const FVector& Position,
	const TArray<IRopeCollider*>& Colliders, const FParams& Params, TArray<IRopeCollider*>& OutNearbyColliders)
{
	OutNearbyColliders.Reset();

	FBox SegmentBounds(ForceInit);
	SegmentBounds += PrevPosition;
	SegmentBounds += Position;
	SegmentBounds = SegmentBounds.ExpandBy(Params.ContactRadius + Params.RopeRadius + 5.0f);

	for (IRopeCollider* Collider : Colliders)
	{
		// 정적 월드 collider는 감지에서 제외 — 랩 대상(본 귀속)이 아니고, 최심-1건 후보 선정에서
		// 벽 접촉이 본 접촉을 가려 캡처를 조용히 막는다(GPU 감지 커널의 정적 제외와 동일 규약).
		if (!Collider || Collider->IsWorldStatic())
		{
			continue;
		}
		if (SegmentBounds.Intersect(Collider->GetWorldBounds().ExpandBy(Params.ContactRadius + Params.RopeRadius)))
		{
			OutNearbyColliders.Add(Collider);
		}
	}
}

FRopeContact FRopeFlightContactDetector::SweepOrSampleContact(const FRopeSimState& Sim, const FVector& PrevPosition,
	const FVector& Position, const TArray<IRopeCollider*>& Colliders, const FParams& Params)
{
	FRopeContact Best;
	const float Travel = FVector::Dist(PrevPosition, Position);
	const int32 SampleCount = FMath::Clamp(FMath::CeilToInt(Travel / FMath::Max(Sim.SegmentLength, 1.0f)), 1, 4);

	for (int32 SampleIdx = 0; SampleIdx <= SampleCount; ++SampleIdx)
	{
		const float Alpha = static_cast<float>(SampleIdx) / static_cast<float>(SampleCount);
		const FVector SamplePos = FMath::Lerp(PrevPosition, Position, Alpha);
		for (const IRopeCollider* Collider : Colliders)
		{
			// 정적 제외: 내부 호출은 GatherNearbyColliders가 이미 걸렀지만, 디버그 경로가 이 함수를
			// FrameColliders로 직접 부르므로 여기서도 방어한다(규약은 위 gather 주석 참조).
			if (!Collider || Collider->IsWorldStatic())
			{
				continue;
			}

			const FRopeContact Contact = Collider->Query(SamplePos, Params.ContactRadius);
			if (Contact.bHit && (!Best.bHit || Contact.Penetration > Best.Penetration))
			{
				Best = Contact;
			}
		}
	}

	return Best;
}

FRopeContactCandidate FRopeFlightContactDetector::MakeCandidate(int32 NodeIndex, const FRopeContact& Contact)
{
	FRopeContactCandidate Candidate;
	Candidate.bValid = Contact.bHit && !Contact.Bone.IsNone();
	Candidate.NodeIndex = NodeIndex;
	Candidate.Bone = Contact.Bone;
	Candidate.Mesh = Contact.SourceMesh;
	Candidate.Source = ERopeContactCandidateSource::Actual;
	Candidate.SourceMask = static_cast<uint8>(Candidate.Source);
	Candidate.WorldPoint = Contact.SurfacePoint;
	Candidate.Normal = Contact.Normal.GetSafeNormal();
	Candidate.Penetration = Contact.Penetration;
	Candidate.WrapDirectionScore = 0.0f;

	//움직이는 bone 위에서 로프가 상대적으로 어떻게 미끄러지는지 판단할 때 필요함.
	Candidate.SurfaceVelocity = Contact.SurfaceVelocity;

	return Candidate;
}

FVector FRopeFlightContactDetector::ExpectedWrapTangent(const FRopeSimState& Sim, const FRopeContactCandidate& Candidate,
	const FVector& FallbackForward)
{
	const FVector ToHand = (Sim.Num() > 0) ? (Sim.Positions[0] - Candidate.WorldPoint).GetSafeNormal() : FallbackForward;
	const FVector Tangent = ToHand - FVector::DotProduct(ToHand, Candidate.Normal) * Candidate.Normal;
	return Tangent.GetSafeNormal(UE_SMALL_NUMBER, FallbackForward);
}

bool FRopeFlightContactDetector::ShouldRunPredictiveContactForNode(const FRopeSimState& Sim, const FWhipGuideView& Whip,
	int32 NodeIndex, const FVector& FrameDisplacement)
{
	return Whip.IsGuidedNode(NodeIndex) || IsTailNode(Sim, NodeIndex) || FrameDisplacement.Size() > Sim.SegmentLength;
}
