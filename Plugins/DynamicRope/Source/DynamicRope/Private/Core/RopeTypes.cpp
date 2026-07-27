// Copyright Epic Games, Inc. All Rights Reserved.
//
// The out-of-line member functions of the data types in RopeTypes.h. The header concentrates on the type
// definitions, most of which are POD, and logic that needs engine component includes or has any bulk to it is
// collected here.

#include "Core/RopeTypes.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "GameFramework/Actor.h"

FRopeThrowContext FRopeThrowContext::MakeDefault(const USceneComponent& RopeComponent, const FRopeThrowParams& Params)
{
	FRopeThrowContext Context;
	Context.Origin = RopeComponent.GetComponentLocation();
	if (const AActor* Owner = RopeComponent.GetOwner())
	{
		Context.OwnerVelocity = Owner->GetVelocity();
		// The default entry point cannot measure the hand's swing, so the animation-relative velocity is zero; the wielder path alone measures and fills it in.
		Context.HandAnimationVelocity = FVector::ZeroVector;
	}

	// Configuration passed straight through.
	Context.FrameMode = Params.FrameMode;
	Context.ThrowSpeed = Params.ThrowSpeed;
	Context.SwingPlane = Params.SwingPlane;
	Context.CustomSwingPlaneNormal = Params.CustomSwingPlaneNormal;

	// The frame basis: all three axes are decided together in one place per frame mode. An earlier implementation put
	// the component basis in first and had per-mode branches overwrite some of the axes, deciding the world mode's up
	// vector separately in a ternary, which made it order-dependent.
	// Only the raw values are assembled here; normalization and the orthogonal fallback are ResolveThrowContext's
	// responsibility, which keeps the two separate.
	auto SetComponentBasis = [&Context, &RopeComponent]()
	{
		Context.FrameForward = RopeComponent.GetForwardVector();
		Context.FrameUp = RopeComponent.GetUpVector();
		Context.FrameRight = RopeComponent.GetRightVector();
	};
	switch (Params.FrameMode)
	{
	case ERopeThrowFrameMode::World:
		Context.FrameForward = FVector::ForwardVector;
		Context.FrameUp = FVector::UpVector;
		Context.FrameRight = FVector::RightVector;
		break;

	case ERopeThrowFrameMode::OwnerCamera:
	{
		const AActor* Owner = RopeComponent.GetOwner();
		const UCameraComponent* Camera = Owner ? Owner->FindComponentByClass<UCameraComponent>() : nullptr;
		if (Camera)
		{
			Context.FrameForward = Camera->GetForwardVector();
			Context.FrameUp = Camera->GetUpVector();
			Context.FrameRight = Camera->GetRightVector();
		}
		else
		{
			// An owner with no camera falls back to the component basis.
			SetComponentBasis();
		}
		break;
	}

	case ERopeThrowFrameMode::Custom:
		Context.FrameForward = Params.CustomFrameForward;
		Context.FrameUp = Params.CustomFrameUp;
		Context.FrameRight = Params.CustomFrameRight;
		break;

	case ERopeThrowFrameMode::Owner:
	case ERopeThrowFrameMode::Socket:
	default:
		SetComponentBasis();
		break;
	}
	return Context;
}

FRopeCaptureTravelFrame FRopeCaptureTravelFrame::Compute(const FRopeSimState& Sim,
	const TArray<FRopeContactCandidate>& Candidates, float DeltaTime)
{
	FRopeCaptureTravelFrame Frame;

	// Collects the valid candidates: the mean of the surface points, giving the region centre, the range of contacting
	// nodes, giving the span, and the mean velocity per node.
	// The same node can be caught by several colliders and appear as duplicate candidates, so the velocity is averaged per node.
	FVector CenterSum = FVector::ZeroVector;
	int32 CenterCount = 0;
	FVector VelocitySum = FVector::ZeroVector;
	int32 VelocityCount = 0;
	int32 MinNode = TNumericLimits<int32>::Max();
	int32 MaxNode = INDEX_NONE;
	TArray<int32, TInlineAllocator<32>> CountedNodes;

	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid || !Sim.Positions.IsValidIndex(Candidate.NodeIndex))
		{
			continue;
		}

		CenterSum += Candidate.WorldPoint;
		++CenterCount;
		MinNode = FMath::Min(MinNode, Candidate.NodeIndex);
		MaxNode = FMath::Max(MaxNode, Candidate.NodeIndex);

		if (DeltaTime > KINDA_SMALL_NUMBER &&
			Sim.PrevPositions.IsValidIndex(Candidate.NodeIndex) &&
			!CountedNodes.Contains(Candidate.NodeIndex))
		{
			CountedNodes.Add(Candidate.NodeIndex);
			VelocitySum += (Sim.Positions[Candidate.NodeIndex] - Sim.PrevPositions[Candidate.NodeIndex]) / DeltaTime;
			++VelocityCount;
		}
	}

	if (CenterCount == 0)
	{
		return Frame;
	}

	Frame.bValid = true;
	Frame.RegionCenter = CenterSum / static_cast<float>(CenterCount);
	if (VelocityCount > 0)
	{
		Frame.AverageVelocity = VelocitySum / static_cast<float>(VelocityCount);
	}

	// The span: when only one node is in contact, which is common when the tip lands first, it is widened to the neighbouring nodes to establish the direction the rope lies in.
	const int32 SpanStart = FMath::Max(0, MinNode - 1);
	const int32 SpanEnd = FMath::Min(Sim.Num() - 1, MaxNode + 1);
	if (SpanEnd > SpanStart)
	{
		Frame.SpanDirection = (Sim.Positions[SpanEnd] - Sim.Positions[SpanStart]).GetSafeNormal();
	}

	// The travel plane normal is the cross product of the velocity direction and the direction the rope lies in, both
	// unit vectors, so the magnitude of the cross product is the sine of the angle between them.
	// It degenerates when the velocity is zero or the rope flies straight along its own direction, as in a spear
	// throw, in which case the plane normal flag is left false so that the consumer moves on to the next fallback,
	// such as the shape axis. An angle below about six degrees is treated as numerical noise and discarded.
	const FVector VelocityDir = Frame.AverageVelocity.GetSafeNormal();
	const FVector Cross = FVector::CrossProduct(VelocityDir, Frame.SpanDirection);
	constexpr float MinPlaneSinAngle = 0.1f;
	if (Cross.SizeSquared() > FMath::Square(MinPlaneSinAngle))
	{
		Frame.PlaneNormal = Cross.GetSafeNormal();
		Frame.bHasPlaneNormal = true;
	}
	return Frame;
}

void FRopeContactTracker::Update(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime,
	const USceneComponent* PreferredMesh, FName PreferredBone, bool bRequirePreferred)
{
	if (Candidates.Num() == 0)
	{
		Decay(DeltaTime);
		return;
	}

	// The aggregation key is the pair of mesh and bone. Keying on the bone name alone would, when two actors sharing a
	// skeleton and therefore the bone names are touched in the same frame, sum candidates from different targets into
	// one bucket and attribute the mesh to whichever candidate came last.
	using FTargetKey = TPair<const USceneComponent*, FName>;
	TMap<FTargetKey, TArray<int32>> NodesByTarget;
	TMap<FTargetKey, float> ScoreByTarget;
	TMap<FTargetKey, int32> HeadNodeByTarget;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid || Candidate.Bone.IsNone())
		{
			continue;
		}

		const FTargetKey Key(Candidate.Mesh, Candidate.Bone);
		NodesByTarget.FindOrAdd(Key).Add(Candidate.NodeIndex);
		ScoreByTarget.FindOrAdd(Key) += Candidate.Penetration + FMath::Max(0.0f, Candidate.WrapDirectionScore);
		if (int32* ExistingHeadNode = HeadNodeByTarget.Find(Key))
		{
			*ExistingHeadNode = FMath::Min(*ExistingHeadNode, Candidate.NodeIndex);
		}
		else
		{
			HeadNodeByTarget.Add(Key, Candidate.NodeIndex);
		}
	}

	// Updates the per-target dwell ledger, which is the material for seed multiplexing: a key present this frame
	// accumulates dwell and replaces its nodes, while a key that has dropped out decays by the same amount and is
	// removed once exhausted, with the same flicker tolerance rule as the dominant dwell.
	// The dominant selection below is independent of this ledger and keeps the existing logic, so single-seed
	// behaviour is unchanged.
	for (int32 Index = Targets.Num() - 1; Index >= 0; --Index)
	{
		FRopeTrackedContactTarget& Target = Targets[Index];
		const FTargetKey Key(Target.Mesh, Target.Bone);
		if (const TArray<int32>* Nodes = NodesByTarget.Find(Key))
		{
			Target.DwellTime += DeltaTime;
			Target.Nodes = *Nodes;
		}
		else
		{
			Target.DwellTime -= DeltaTime;
			Target.Nodes.Reset();
			if (Target.DwellTime <= 0.0f)
			{
				Targets.RemoveAt(Index);
			}
		}
	}
	for (const TPair<FTargetKey, TArray<int32>>& Pair : NodesByTarget)
	{
		const bool bAlreadyTracked = Targets.ContainsByPredicate(
			[&Pair](const FRopeTrackedContactTarget& Target)
			{
				return Target.Mesh == Pair.Key.Key && Target.Bone == Pair.Key.Value;
			});
		if (!bAlreadyTracked)
		{
			FRopeTrackedContactTarget& Target = Targets.AddDefaulted_GetRef();
			Target.Bone = Pair.Key.Value;
			Target.Mesh = Pair.Key.Key;
			Target.Nodes = Pair.Value;
			Target.DwellTime = 0.0f;
		}
	}

	FTargetKey BestTarget(nullptr, NAME_None);
	int32 BestCount = 0;
	int32 BestHeadNode = INDEX_NONE;
	float BestScore = 0.0f;
	// The preferred bone stops a bone with many nodes, such as the pelvis, outranking the arm that was aimed at.
	// It affects the dominant selection alone, and the target update above is performed for every bone regardless.
	const FTargetKey PreferredTarget(PreferredMesh, PreferredBone);
	const bool bHasPreferredTarget = PreferredMesh && !PreferredBone.IsNone()
		&& NodesByTarget.Contains(PreferredTarget);
	if (bHasPreferredTarget)
	{
		BestTarget = PreferredTarget;
	}
	else if (!bRequirePreferred)
	{
		for (const TPair<FTargetKey, TArray<int32>>& Pair : NodesByTarget)
		{
			const float Score = ScoreByTarget.FindRef(Pair.Key);
			const int32 HeadNode = HeadNodeByTarget.FindRef(Pair.Key);
			if (Pair.Value.Num() > BestCount ||
				(Pair.Value.Num() == BestCount &&
					(BestHeadNode == INDEX_NONE || HeadNode < BestHeadNode ||
						(HeadNode == BestHeadNode && Score > BestScore))))
			{
				BestTarget = Pair.Key;
				BestCount = Pair.Value.Num();
				BestHeadNode = HeadNode;
				BestScore = Score;
			}
		}
	}

	if (BestTarget.Value.IsNone())
	{
		if (bRequirePreferred)
		{
			CandidateBone = NAME_None;
			CandidateMesh = nullptr;
			CandidateNodes.Reset();
			DwellTime = 0.0f;
		}
		else
		{
			Decay(DeltaTime);
		}
		return;
	}

	// Dwell continuity is also keyed on the pair of mesh and bone: the same bone name on a different mesh is a different target and is reset.
	if (BestTarget.Value == CandidateBone && BestTarget.Key == CandidateMesh)
	{
		DwellTime += DeltaTime;
	}
	else
	{
		CandidateBone = BestTarget.Value;
		DwellTime = 0.0f;
	}

	CandidateMesh = BestTarget.Key;
	CandidateNodes = NodesByTarget.FindRef(BestTarget);
}
