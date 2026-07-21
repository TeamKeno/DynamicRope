// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeFlightContactDetector.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"

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

	// 로프 Verlet 변위(Positions-PrevPositions)는 *마지막 substep* 델타(≈ v·SubstepDeltaTime)라, 프레임
	// 단위 lookahead(PredictiveContactFrames)로 쓰려면 프레임/substep 비로 환산한다 — 안 하면 Substeps배
	// (기본 12배) 과소 적용된다. 가이드 노드 분기는 프레임 단위 타깃 차분을 쓰므로 이 환산을 적용하지 않는다.
	const float FrameToSubstepRatio = (Params.SubstepDeltaTime > KINDA_SMALL_NUMBER)
		? (Params.FrameDeltaTime / Params.SubstepDeltaTime) : 1.0f;

	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!Sim.Positions.IsValidIndex(i) || !Sim.PrevPositions.IsValidIndex(i))
		{
			continue;
		}

		FVector CurrentPosition = Sim.Positions[i];
		FVector PredictedPosition = CurrentPosition;
		// substep 변위 → 프레임 변위 환산(위 주석). bFastNode/ShouldRun 임계와 자유 노드 예측이 모두 이 값을 쓴다.
		const FVector FrameDisplacement = Sim.Displacement(i) * FrameToSubstepRatio;
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

	}
}

FRopeFlightCaptureEvaluation FRopeFlightContactDetector::EvaluateCapture(
	const TArray<FRopeContactCandidate>& Candidates, const FParams& Params,
	const FRopeFlightCapturePolicy& Policy)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightEvaluateCapture);
	FRopeFlightCaptureEvaluation Result;
	Result.Tracker.Update(Candidates, 0.0f,
		Policy.PreferredMesh, Policy.PreferredBone, Policy.bRequirePreferred);
	Result.bShouldCapture =
		Result.Tracker.CandidateNodes.Num() >= FMath::Max(1, Params.MinLatchNodes) &&
		IsWrappableBone(Result.Tracker.CandidateBone);
	return Result;
}

bool FRopeFlightContactDetector::ShouldCapture(const TArray<FRopeContactCandidate>& Candidates, const FParams& Params)
{
	return EvaluateCapture(Candidates, Params).bShouldCapture;
}

bool FRopeFlightContactDetector::PassesCaptureQualityGate(const FRopeContactTracker&,
	const TArray<FRopeContactCandidate>&, const FParams&)
{
	return true;
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
	// 샘플 간격은 **cm**로 끊는다(세그먼트 길이가 아니라). 세그먼트 길이는 대상 두께와 무관해서, 종전
	// "Travel/SegmentLength, 최대 4"는 빠른 노드가 얇은 collider를 샘플 사이로 통과하게 놔뒀다.
	// GPU RopeDetectSweep과 같은 식을 쓴다(둘이 갈라지면 parity 테스트가 못 잡는 종류의 버그가 된다).
	const float Step = FMath::Max(Params.ContactSweepStep, 0.1f);
	const int32 MaxSamples = FMath::Max(Params.ContactMaxSweepSamples, 1);
	const int32 SampleCount = FMath::Clamp(FMath::CeilToInt(Travel / Step), 1, MaxSamples);

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
