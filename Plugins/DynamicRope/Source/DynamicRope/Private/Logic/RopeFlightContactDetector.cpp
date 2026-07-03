// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeFlightContactDetector.h"
#include "Collision/RopeCollider.h"
#include "ProfilingDebugging/CpuProfilerTrace.h" // TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)

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

		bool bFast = IsTailNode(Sim, i) || NodeSpeed(Sim, i) > Sim.SegmentLength;
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
		const FVector FrameDisplacement = Sim.Positions[i] - Sim.PrevPositions[i];
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

		const FVector RopeVelocity = Sim.Positions[Candidate.NodeIndex] - Sim.PrevPositions[Candidate.NodeIndex];
		const FVector RelativeVelocity = RopeVelocity - Candidate.SurfaceVelocity;
		const FVector TangentVelocity = RelativeVelocity - FVector::DotProduct(RelativeVelocity, Candidate.Normal) * Candidate.Normal;

		Candidate.RelativeTangentialSpeed = TangentVelocity.Size();
		Candidate.WrapDirectionScore = FVector::DotProduct(TangentVelocity.GetSafeNormal(),
			ExpectedWrapTangent(Sim, Candidate, Params.FallbackForward));
	}
}

bool FRopeFlightContactDetector::ShouldCapture(const TArray<FRopeContactCandidate>& Candidates, const FParams& Params)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightShouldCaptureImpl);
	FRopeContactTracker TempTracker;
	TempTracker.Update(Candidates, 0.0f);
	return TempTracker.CandidateNodes.Num() >= FMath::Max(1, Params.MinLatchNodes)
		&& IsWrappableBone(TempTracker.CandidateBone);
}

bool FRopeFlightContactDetector::IsTailNode(const FRopeSimState& Sim, int32 NodeIndex)
{
	return NodeIndex >= FMath::Max(1, Sim.Num() - 4);
}

float FRopeFlightContactDetector::NodeSpeed(const FRopeSimState& Sim, int32 NodeIndex)
{
	if (!Sim.Positions.IsValidIndex(NodeIndex) || !Sim.PrevPositions.IsValidIndex(NodeIndex))
	{
		return 0.0f;
	}
	return (Sim.Positions[NodeIndex] - Sim.PrevPositions[NodeIndex]).Size();
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
		if (Collider && SegmentBounds.Intersect(Collider->GetWorldBounds().ExpandBy(Params.ContactRadius + Params.RopeRadius)))
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
			if (!Collider)
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
