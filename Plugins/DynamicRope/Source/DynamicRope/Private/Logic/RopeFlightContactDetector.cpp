// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeFlightContactDetector.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"

float FRopeFlightContactDetector::ComputeFrameToSubstepRatio(float FrameDeltaTime, float SubstepDeltaTime)
{
	return (SubstepDeltaTime > KINDA_SMALL_NUMBER) ? (FrameDeltaTime / SubstepDeltaTime) : 1.0f;
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

		// It looks at the path of travel rather than the current position alone.
		// A capsule can do this as a segment against a capsule.
		// An SDF needs the path sampled at a few points, or a swept query adapter.
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

void FRopeFlightContactDetector::AddGuidedContactCandidates(const FRopeSimState& Sim,
	const TArray<IRopeCollider*>& Colliders, const FParams& Params, const FWhipGuideView& Whip,
	TArray<FRopeContactCandidate>& InOutCandidates)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightAddGuidedContactCandidates);
	if (!Whip.HasGuidedNodes() || !Whip.CurrentTargets || Colliders.Num() == 0)
	{
		return;
	}

	// The sample budget of the ordinary detector and of the GPU is kept small for the sake of the frame cost. This
	// path checks only the few exact primary colliders of an assisted throw, so it uses a larger safety limit to stop
	// a fast guide movement immediately widening the contact sweep step into sixteen divisions and skipping a thin bone.
	FParams ReliableParams = Params;
	ReliableParams.ContactMaxSweepSamples = FMath::Max(
		ReliableParams.ContactMaxSweepSamples, ReliableGuidedSweepMaxSamples);

	TArray<IRopeCollider*> NearbyColliders;
	auto AddPathContact = [&](int32 NodeIndex, const FVector& PathStart, const FVector& PathEnd)
	{
		GatherNearbyColliders(PathStart, PathEnd, Colliders, ReliableParams, NearbyColliders);
		if (NearbyColliders.Num() == 0)
		{
			return;
		}

		const FRopeContact Contact = SweepOrSampleContact(Sim, PathStart, PathEnd, NearbyColliders, ReliableParams);
		if (!Contact.bHit || Contact.Bone.IsNone())
		{
			return;
		}

		FRopeContactCandidate Candidate = MakeCandidate(NodeIndex, Contact);
		Candidate.Source = ERopeContactCandidateSource::Actual;
		Candidate.SourceMask = static_cast<uint8>(Candidate.Source);
		AddUniqueCandidate(InOutCandidates, Candidate);
	};

	const int32 NodeCount = FMath::Min(Sim.Num(), Whip.CurrentTargets->Num());
	for (int32 NodeIndex = 0; NodeIndex < NodeCount; ++NodeIndex)
	{
		if (!Whip.IsGuidedNode(NodeIndex))
		{
			continue;
		}

		const FVector Current = (*Whip.CurrentTargets)[NodeIndex];
		const FVector Previous = Whip.PrevTargets && Whip.PrevTargets->IsValidIndex(NodeIndex)
			? (*Whip.PrevTargets)[NodeIndex]
			: (Sim.PrevPositions.IsValidIndex(NodeIndex) ? Sim.PrevPositions[NodeIndex] : Current);
		// The GPU detector's post-solve previous-to-current difference sees only the last substep, and the readback
		// lags as well. Checking the path the guide actually drew this frame directly on the game thread preserves a
		// target hit that lasts a single frame.
		AddPathContact(NodeIndex, Previous, Current);

	}

	// The full solver has node-against-edge internal collision, but the collision-free assisted detector looked at
	// nodes alone. The current centreline edges are now checked too. At the boundary between the root and the tip,
	// where only one side is guided, the guide target has to be joined to the solver pose; looking at guided pairs
	// alone lets a thin bone slip through exactly that boundary gap.
	for (int32 NodeIndex = 0; NodeIndex + 1 < Sim.Num(); ++NodeIndex)
	{
		const int32 NextNode = NodeIndex + 1;
		if (!Whip.IsGuidedNode(NodeIndex) && !Whip.IsGuidedNode(NextNode))
		{
			continue;
		}

		const FVector Current = Whip.IsGuidedNode(NodeIndex) && Whip.CurrentTargets->IsValidIndex(NodeIndex)
			? (*Whip.CurrentTargets)[NodeIndex] : Sim.Positions[NodeIndex];
		const FVector NextCurrent = Whip.IsGuidedNode(NextNode) && Whip.CurrentTargets->IsValidIndex(NextNode)
			? (*Whip.CurrentTargets)[NextNode] : Sim.Positions[NextNode];
		GatherNearbyColliders(Current, NextCurrent, Colliders, ReliableParams, NearbyColliders);
		if (NearbyColliders.Num() == 0)
		{
			continue;
		}

		const FRopeContact Contact = SweepOrSampleContact(
			Sim, Current, NextCurrent, NearbyColliders, ReliableParams);
		if (!Contact.bHit || Contact.Bone.IsNone())
		{
			continue;
		}

		const int32 ContactNode = FVector::DistSquared(Contact.SurfacePoint, Current)
			<= FVector::DistSquared(Contact.SurfacePoint, NextCurrent) ? NodeIndex : NextNode;
		FRopeContactCandidate Candidate = MakeCandidate(ContactNode, Contact);
		Candidate.Source = ERopeContactCandidateSource::Actual;
		Candidate.SourceMask = static_cast<uint8>(Candidate.Source);
		AddUniqueCandidate(InOutCandidates, Candidate);
	}
}

void FRopeFlightContactDetector::AddCurrentCenterlineContactCandidates(const FRopeSimState& Sim,
	const TArray<IRopeCollider*>& Colliders, const FParams& Params,
	TArray<FRopeContactCandidate>& InOutCandidates)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightAddCurrentCenterlineContactCandidates);
	if (Sim.Num() == 0 || Colliders.Num() == 0)
	{
		return;
	}

	FParams ReliableParams = Params;
	ReliableParams.ContactMaxSweepSamples = FMath::Max(
		ReliableParams.ContactMaxSweepSamples, ReliableGuidedSweepMaxSamples);
	TArray<IRopeCollider*> NearbyColliders;

	auto AddContact = [&](int32 NodeIndex, const FVector& Start, const FVector& End)
	{
		GatherNearbyColliders(Start, End, Colliders, ReliableParams, NearbyColliders);
		if (NearbyColliders.Num() == 0)
		{
			return;
		}
		const FRopeContact Contact = SweepOrSampleContact(Sim, Start, End, NearbyColliders, ReliableParams);
		if (!Contact.bHit || Contact.Bone.IsNone())
		{
			return;
		}
		FRopeContactCandidate Candidate = MakeCandidate(NodeIndex, Contact);
		Candidate.Source = ERopeContactCandidateSource::Actual;
		Candidate.SourceMask = static_cast<uint8>(Candidate.Source);
		AddUniqueCandidate(InOutCandidates, Candidate);
	};

	for (int32 NodeIndex = 0; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		AddContact(NodeIndex, Sim.Positions[NodeIndex], Sim.Positions[NodeIndex]);
	}
	for (int32 NodeIndex = 0; NodeIndex + 1 < Sim.Num(); ++NodeIndex)
	{
		const FVector& Current = Sim.Positions[NodeIndex];
		const FVector& Next = Sim.Positions[NodeIndex + 1];
		GatherNearbyColliders(Current, Next, Colliders, ReliableParams, NearbyColliders);
		if (NearbyColliders.Num() == 0)
		{
			continue;
		}
		const FRopeContact Contact = SweepOrSampleContact(Sim, Current, Next, NearbyColliders, ReliableParams);
		if (!Contact.bHit || Contact.Bone.IsNone())
		{
			continue;
		}
		const int32 ContactNode = FVector::DistSquared(Contact.SurfacePoint, Current)
			<= FVector::DistSquared(Contact.SurfacePoint, Next) ? NodeIndex : NodeIndex + 1;
		FRopeContactCandidate Candidate = MakeCandidate(ContactNode, Contact);
		Candidate.Source = ERopeContactCandidateSource::Actual;
		Candidate.SourceMask = static_cast<uint8>(Candidate.Source);
		AddUniqueCandidate(InOutCandidates, Candidate);
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

	TArray<IRopeCollider*> NearbyColliders;
	const bool bHasGuidedNodes = Whip.HasGuidedNodes();

	// The rope's Verlet displacement, current minus previous, is the delta of the last substep alone, meaning roughly
	// the velocity times the substep delta, so using it as a per-frame lookahead requires converting it by the ratio
	// of the frame to the substep. Without that it is applied too weakly by the substep count, twelve by default. The
	// guided node branch uses a per-frame target difference and does not apply this conversion.
	const float FrameToSubstepRatio =
		ComputeFrameToSubstepRatio(Params.FrameDeltaTime, Params.SubstepDeltaTime);

	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!Sim.Positions.IsValidIndex(i) || !Sim.PrevPositions.IsValidIndex(i))
		{
			continue;
		}

		FVector CurrentPosition = Sim.Positions[i];
		FVector PredictedPosition = CurrentPosition;
		// Converts the substep displacement to a frame displacement, as described above. The fast node and run
		// thresholds and the free node prediction all use this value.
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
		AddUniqueCandidate(InOutCandidates, Candidate);
	}
}

void FRopeFlightContactDetector::AddUniqueCandidate(TArray<FRopeContactCandidate>& InOutCandidates,
	const FRopeContactCandidate& Candidate)
{
	for (FRopeContactCandidate& Existing : InOutCandidates)
	{
		if (Existing.NodeIndex == Candidate.NodeIndex && Existing.Bone == Candidate.Bone && Existing.Mesh == Candidate.Mesh)
		{
			Existing.SourceMask |= Candidate.SourceMask;
			// A GPU candidate can lag by one or two frames. When a synchronous guide's actual contact arrives with
			// the same key, merging the source bits alone leaves the capture succeeding but the anchor starting from
			// a stale contact point and normal.
			if (Candidate.Source == ERopeContactCandidateSource::Actual)
			{
				Existing.bValid = Candidate.bValid;
				Existing.WorldPoint = Candidate.WorldPoint;
				Existing.Normal = Candidate.Normal;
				Existing.Penetration = Candidate.Penetration;
				Existing.SurfaceVelocity = Candidate.SurfaceVelocity;
			}
			if (Candidate.Source == ERopeContactCandidateSource::PredictiveGuided ||
				(Existing.Source == ERopeContactCandidateSource::Actual &&
					Candidate.Source == ERopeContactCandidateSource::PredictiveFree))
			{
				Existing.Source = Candidate.Source;
			}
			return;
		}
	}

	InOutCandidates.Add(Candidate);
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

		// The rope velocity is a Verlet difference, meaning the displacement of the last substep in centimetres per
		// substep, roughly the velocity times the fixed delta, while the surface velocity is contractually in
		// centimetres per second. Subtracting them in the same units requires converting the surface velocity by the
		// substep delta: converting by the frame delta overstates it by the substep count whenever there is more than
		// one substep and gives a wrong direction and magnitude for the relative motion. This is the same principle
		// by which the solver's friction converts to the substep delta.
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
		// Static world colliders are excluded from detection: they are not wrap targets, having no bone attribution,
		// and the single-candidate selection could otherwise let a wall contact hide a bone contact and silently
		// prevent a capture. The GPU detection kernel excludes static colliders under the same convention.
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
	// The sample spacing is measured in centimetres rather than in segment lengths. The segment length has nothing to
	// do with the target's thickness, and the previous rule of the travel distance over the segment length, capped at
	// four, let a fast node pass through a thin collider between samples.
	// The same expression is used as in the GPU sweep, since a divergence between them is exactly the kind of bug the
	// parity test cannot catch.
	const float Step = FMath::Max(Params.ContactSweepStep, 0.1f);
	const int32 MaxSamples = FMath::Max(Params.ContactMaxSweepSamples, 1);
	const int32 SampleCount = FMath::Clamp(FMath::CeilToInt(Travel / Step), 1, MaxSamples);

	for (int32 SampleIdx = 0; SampleIdx <= SampleCount; ++SampleIdx)
	{
		const float Alpha = static_cast<float>(SampleIdx) / static_cast<float>(SampleCount);
		const FVector SamplePos = FMath::Lerp(PrevPosition, Position, Alpha);
		FRopeContact SampleBest;
		for (const IRopeCollider* Collider : Colliders)
		{
			// The static exclusion: internal callers have already been filtered by the gather, but the debug path
			// calls this function directly with the frame colliders, so it is defended here as well. The convention
			// is described in the gather comment above.
			if (!Collider || Collider->IsWorldStatic())
			{
				continue;
			}

			const FRopeContact Contact = Collider->Query(SamplePos, Params.ContactRadius);
			if (Contact.bHit && (!SampleBest.bHit || Contact.Penetration > SampleBest.Penetration))
			{
				SampleBest = Contact;
			}
		}

		// Preserve sweep time ordering. A thick or decomposed convex may have a much deeper sample near its
		// centre, but using that sample can flip the normal to the exit face or select an internal hull face.
		// Colliders still compete by penetration at this one sample, giving a deterministic single candidate.
		if (SampleBest.bHit)
		{
			return SampleBest;
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

	// Needed to judge how the rope slides relative to a bone that is itself moving.
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
