// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeThrowPreviewBuilder.h"

#include "Collision/RopeCollider.h"
#include "Logic/RopeFlightContactDetector.h"
#include "Logic/RopeWhipGuide.h"
#include "Logic/RopeWrappingPhase.h"
#include "RopeMathHelpers.h"
#include "Components/SkeletalMeshComponent.h"

namespace
{
	void SetPreviewFailureReason(FString* OutFailureReason, const FString& Reason)
	{
		if (OutFailureReason)
		{
			*OutFailureReason = Reason;
		}
	}

	const TArray<IRopeCollider*>& GetColliders(const FRopeThrowPreviewBuilder::FInput& Input)
	{
		static const TArray<IRopeCollider*> EmptyColliders;
		return Input.Colliders ? *Input.Colliders : EmptyColliders;
	}

	FVector ArcPreviewDirectionAtAlpha(const FRopeArcPreviewData& Preview, float Alpha)
	{
		const FVector Aim = FRopeWhipGuide::SafeNormalOr(Preview.AimDir, FVector::ForwardVector);
		FVector Up = Preview.GuideUp - FVector::DotProduct(Preview.GuideUp, Aim) * Aim;
		Up = FRopeWhipGuide::SafeNormalOr(Up, FVector::UpVector);

		const float ClampedAlpha = FMath::Clamp(Alpha, 0.0f, 1.0f);
		const float SweepRadians = FMath::DegreesToRadians(FMath::Clamp(Preview.SweepAngleDegrees, 1.0f, 180.0f));
		const float Angle = SweepRadians * (1.0f - ClampedAlpha);
		return (Aim * FMath::Cos(Angle) + Up * FMath::Sin(Angle)).GetSafeNormal();
	}

	int32 FindHeadValidNodeIndex(const TArray<int32>& NodeIndices, const FRopeSimState& Sim)
	{
		int32 HeadNodeIndex = INDEX_NONE;
		for (const int32 NodeIndex : NodeIndices)
		{
			if (!Sim.Positions.IsValidIndex(NodeIndex))
			{
				continue;
			}

			if (HeadNodeIndex == INDEX_NONE || NodeIndex < HeadNodeIndex)
			{
				HeadNodeIndex = NodeIndex;
			}
		}
		return HeadNodeIndex;
	}

	bool BuildThrowArcPreview(const FRopeThrowPreviewBuilder::FInput& Input, FRopeArcPreviewData& OutPreview,
		FString* OutFailureReason)
	{
		OutPreview = FRopeArcPreviewData();

		const FRopeSimState* Sim = Input.Sim;
		const float ClampedReachScale = FMath::Max(Input.ReachScale, 0.0f);
		if (ClampedReachScale <= KINDA_SMALL_NUMBER)
		{
			SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("throw arc preview build failed: reach scale too small (reachScale=%.2f)"),
					Input.ReachScale));
			return false;
		}

		const FRopeThrowContext& ThrowContext = Input.ThrowContext;
		const FRopeWhipGuide::FSwingBasis SwingBasis = FRopeWhipGuide::ResolveSwingBasis(
			ThrowContext, ThrowContext.SwingPlane, ThrowContext.CustomSwingPlaneNormal);
		const float SourceRopeLength = FMath::Max(Sim ? Sim->RopeLength : 0.0f, Input.RopeLength);

		OutPreview.Origin = ThrowContext.Origin;
		OutPreview.AimDir = SwingBasis.AimDir;
		OutPreview.GuideUp = SwingBasis.GuideUp;
		OutPreview.Radius = SourceRopeLength * ClampedReachScale;
		OutPreview.SweepAngleDegrees = FMath::Clamp(Input.SweepAngleDegrees, 1.0f, 180.0f);
		OutPreview.SegmentCount = FMath::Clamp(Input.SegmentCount, 1, 128);
		if (OutPreview.Radius <= KINDA_SMALL_NUMBER)
		{
			SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("throw arc preview build failed (reachScale=%.2f, segmentCount=%d, ropeLength=%.2f)"),
					Input.ReachScale, Input.SegmentCount, SourceRopeLength));
			return false;
		}
		return true;
	}

	struct FThrowPreviewContactCandidate
	{
		FRopeContactCandidate Candidate;
		float AngleAlpha = 1.0f;
		float DistanceAlpha = 1.0f;
		float ArcStartAngleDegrees = 180.0f;
		FVector Direction = FVector::ForwardVector;
	};

	bool IsBetterThrowPreviewContactCandidate(const FThrowPreviewContactCandidate& Candidate,
		const FThrowPreviewContactCandidate& Best)
	{
		if (!FMath::IsNearlyEqual(Candidate.ArcStartAngleDegrees, Best.ArcStartAngleDegrees, 0.1f))
		{
			return Candidate.ArcStartAngleDegrees < Best.ArcStartAngleDegrees;
		}
		if (!FMath::IsNearlyEqual(Candidate.DistanceAlpha, Best.DistanceAlpha, 0.01f))
		{
			return Candidate.DistanceAlpha < Best.DistanceAlpha;
		}
		return Candidate.AngleAlpha < Best.AngleAlpha;
	}

	bool FindThrowPreviewContactCandidate(const FRopeArcPreviewData& Preview, const TArray<IRopeCollider*>& Colliders,
		float RopeRadius, const FRopeWrapConfig& WrapConfig, const FRopeSimState& Sim,
		float SampleStep, float QueryRadius, FThrowPreviewContactCandidate& OutCandidate,
		FString* OutFailureReason = nullptr)
	{
		OutCandidate = FThrowPreviewContactCandidate();
		if (Preview.Radius <= KINDA_SMALL_NUMBER)
		{
			SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("free search rejected: arc radius too small (radius=%.2f)"), Preview.Radius));
			return false;
		}
		if (Colliders.Num() == 0)
		{
			SetPreviewFailureReason(OutFailureReason, TEXT("free search rejected: no frame colliders"));
			return false;
		}
		if (Sim.Num() < 2)
		{
			SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("free search rejected: rope sim has too few nodes (nodes=%d)"), Sim.Num()));
			return false;
		}

		constexpr int32 MaxPreviewAngleSamples = 24;
		constexpr int32 MaxPreviewRadialSamples = 16;
		constexpr int32 MaxPreviewTotalSamples = 256;

		const int32 AngleSamples = FMath::Clamp(Preview.SegmentCount, 1, MaxPreviewAngleSamples);
		const float RadialStep = FMath::Max(SampleStep, 1.0f);
		const int32 RequestedRadialSamples = FMath::Max(1, FMath::CeilToInt(Preview.Radius / RadialStep));
		const int32 TotalLimitedRadialSamples = FMath::Max(1, MaxPreviewTotalSamples / (AngleSamples + 1));
		const int32 RadialSamples = FMath::Clamp(RequestedRadialSamples, 1,
			FMath::Min(MaxPreviewRadialSamples, TotalLimitedRadialSamples));
		const float EffectiveQueryRadius = QueryRadius > KINDA_SMALL_NUMBER
			? QueryRadius
			: FMath::Max(RopeRadius, WrapConfig.ContactRadius);

		FBox PreviewBounds(EForceInit::ForceInit);
		PreviewBounds += Preview.Origin;
		for (int32 AngleIndex = 0; AngleIndex <= AngleSamples; ++AngleIndex)
		{
			const float AngleAlpha = static_cast<float>(AngleIndex) / static_cast<float>(AngleSamples);
			const FVector Direction = ArcPreviewDirectionAtAlpha(Preview, AngleAlpha);
			if (!Direction.IsNearlyZero())
			{
				PreviewBounds += Preview.Origin + Direction * Preview.Radius;
			}
		}
		PreviewBounds = PreviewBounds.ExpandBy(EffectiveQueryRadius);

		TArray<IRopeCollider*, TInlineAllocator<8>> CandidateColliders;
		for (IRopeCollider* Collider : Colliders)
		{
			if (Collider && Collider->GetWorldBounds().ExpandBy(EffectiveQueryRadius).Intersect(PreviewBounds))
			{
				CandidateColliders.Add(Collider);
			}
		}
		if (CandidateColliders.Num() == 0)
		{
			SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("free search found no colliders inside arc bounds (frameColliders=%d, radius=%.1f, queryRadius=%.1f)"),
					Colliders.Num(), Preview.Radius, EffectiveQueryRadius));
			return false;
		}

		const float SegmentLength = FMath::Max(Sim.SegmentLength, 1.0f);
		const FVector ArcStartDirection = ArcPreviewDirectionAtAlpha(Preview, 0.0f);
		bool bFoundCandidate = false;
		FThrowPreviewContactCandidate BestCandidate;
		int32 HitCount = 0;
		int32 InvalidHitCount = 0;
		for (int32 AngleIndex = 0; AngleIndex <= AngleSamples; ++AngleIndex)
		{
			const float AngleAlpha = static_cast<float>(AngleIndex) / static_cast<float>(AngleSamples);
			const FVector Direction = ArcPreviewDirectionAtAlpha(Preview, AngleAlpha);
			if (Direction.IsNearlyZero())
			{
				continue;
			}

			for (int32 RadialIndex = 1; RadialIndex <= RadialSamples; ++RadialIndex)
			{
				const float DistanceAlpha = static_cast<float>(RadialIndex) / static_cast<float>(RadialSamples);
				const float Distance = Preview.Radius * DistanceAlpha;
				const FVector SamplePoint = Preview.Origin + Direction * Distance;

				for (const IRopeCollider* Collider : CandidateColliders)
				{
					const FRopeContact Contact = Collider->Query(SamplePoint, EffectiveQueryRadius);
					if (!Contact.bHit || Contact.Bone.IsNone() || !Contact.SourceMesh)
					{
						if (Contact.bHit)
						{
							++InvalidHitCount;
						}
						continue;
					}
					++HitCount;

					const FVector OriginToContact = Contact.SurfacePoint - Preview.Origin;
					const FVector ContactDirection = OriginToContact.GetSafeNormal(KINDA_SMALL_NUMBER, Direction);
					const float ArcStartDot = FMath::Clamp(FVector::DotProduct(ArcStartDirection, ContactDirection),
						-1.0f, 1.0f);

					FThrowPreviewContactCandidate Candidate;
					Candidate.Candidate = FRopeFlightContactDetector::MakeCandidate(
						FMath::Clamp(FMath::RoundToInt(Distance / SegmentLength), 1, Sim.Num() - 1),
						Contact);
					Candidate.Candidate.Source = ERopeContactCandidateSource::PredictiveFree;
					Candidate.Candidate.SourceMask = static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree);
					Candidate.AngleAlpha = AngleAlpha;
					Candidate.DistanceAlpha = DistanceAlpha;
					Candidate.ArcStartAngleDegrees = FMath::RadiansToDegrees(FMath::Acos(ArcStartDot));
					Candidate.Direction = Direction;
					if (Candidate.Candidate.bValid &&
						(!bFoundCandidate || IsBetterThrowPreviewContactCandidate(Candidate, BestCandidate)))
					{
						BestCandidate = Candidate;
						bFoundCandidate = true;
					}
				}
			}
		}

		if (!bFoundCandidate)
		{
			SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("free search found no valid contact (candidateColliders=%d, angleSamples=%d, radialSamples=%d, hits=%d, invalidHits=%d, queryRadius=%.1f)"),
					CandidateColliders.Num(), AngleSamples, RadialSamples, HitCount, InvalidHitCount, EffectiveQueryRadius));
			return false;
		}

		OutCandidate = BestCandidate;
		return true;
	}

	FRopeSimState BuildThrowPreviewSim(const FRopeSimState& SourceSim, const FRopeArcPreviewData& Preview,
		const FThrowPreviewContactCandidate& ContactCandidate)
	{
		FRopeSimState PreviewSim = SourceSim;
		const int32 NumNodes = SourceSim.Num();
		if (NumNodes < 2)
		{
			return PreviewSim;
		}

		PreviewSim.Positions.SetNum(NumNodes);
		PreviewSim.PrevPositions.SetNum(NumNodes);
		PreviewSim.InvMass.SetNum(NumNodes);

		const int32 LatchNode = FMath::Clamp(ContactCandidate.Candidate.NodeIndex, 1, NumNodes - 1);
		const float SegmentLength = FMath::Max(SourceSim.SegmentLength, 1.0f);
		const FVector SurfacePoint = ContactCandidate.Candidate.WorldPoint;
		const FVector ToSurface = SurfacePoint - Preview.Origin;
		const float SurfaceDistance = FMath::Max(ToSurface.Size(), SegmentLength);
		const FVector ApproachDir = ToSurface.GetSafeNormal(KINDA_SMALL_NUMBER, ContactCandidate.Direction);
		const FVector TailDir = ContactCandidate.Direction.GetSafeNormal(KINDA_SMALL_NUMBER, ApproachDir);

		for (int32 NodeIndex = 0; NodeIndex < NumNodes; ++NodeIndex)
		{
			FVector Position = FVector::ZeroVector;
			if (NodeIndex <= LatchNode)
			{
				const float Alpha = LatchNode > 0
					? static_cast<float>(NodeIndex) / static_cast<float>(LatchNode)
					: 0.0f;
				if (NodeIndex == LatchNode)
				{
					Position = SurfacePoint;
				}
				else
				{
					const float ArcAlpha = FMath::Lerp(0.0f, ContactCandidate.AngleAlpha, Alpha);
					const FVector ArcDirection = ArcPreviewDirectionAtAlpha(Preview, ArcAlpha);
					const FVector ArcPosition = Preview.Origin + ArcDirection * (SurfaceDistance * Alpha);
					const FVector ContactLinePosition = FMath::Lerp(Preview.Origin, SurfacePoint, Alpha);
					Position = FMath::Lerp(ArcPosition, ContactLinePosition, Alpha * Alpha);
				}
			}
			else
			{
				Position = SurfacePoint + TailDir * (static_cast<float>(NodeIndex - LatchNode) * SegmentLength);
			}

			PreviewSim.Positions[NodeIndex] = Position;
			PreviewSim.PrevPositions[NodeIndex] = Position;
			PreviewSim.InvMass[NodeIndex] = (NodeIndex == 0 && SourceSim.bStartPinned) ? 0.0f : 1.0f;
		}

		PreviewSim.SegmentLength = SegmentLength;
		PreviewSim.RopeLength = SegmentLength * static_cast<float>(NumNodes - 1);
		return PreviewSim;
	}

	FRopeWrappingPhase::FContext MakeWrappingContext(const FRopeThrowPreviewBuilder::FInput& Input)
	{
		return FRopeWrappingPhase::FContext{
			Input.WrapConfig,
			GetColliders(Input),
			Input.PathMode,
			Input.RopeRadius,
			Input.OwnerName
		};
	}

	FRopeFlightContactDetector::FParams MakeFlightDetectParams(const FRopeThrowPreviewBuilder::FInput& Input)
	{
		FRopeFlightContactDetector::FParams Params;
		Params.ContactRadius = Input.WrapConfig.ContactRadius;
		Params.RopeRadius = Input.RopeRadius;
		Params.PredictiveContactFrames = Input.WrapConfig.PredictiveContactFrames;
		Params.MinLatchNodes = Input.WrapConfig.MinLatchNodes;
		Params.FallbackForward = Input.FallbackForward;
		return Params;
	}
}

bool FRopeThrowPreviewBuilder::BuildFreeWrappingPreview(const FInput& Input, FRopeWrapPreviewData& OutPreview,
	FString* OutFailureReason)
{
	OutPreview = FRopeWrapPreviewData();
	const FRopeSimState* Sim = Input.Sim;
	if (!Sim)
	{
		SetPreviewFailureReason(OutFailureReason, TEXT("free search rejected: no rope sim"));
		return false;
	}

	FRopeArcPreviewData ArcPreview;
	if (!BuildThrowArcPreview(Input, ArcPreview, OutFailureReason))
	{
		return false;
	}

	FThrowPreviewContactCandidate ContactCandidate;
	if (!FindThrowPreviewContactCandidate(ArcPreview, GetColliders(Input), Input.RopeRadius, Input.WrapConfig, *Sim,
		Input.SampleStep, Input.QueryRadius, ContactCandidate, OutFailureReason))
	{
		return false;
	}

	FRopeSimState PreviewSim = BuildThrowPreviewSim(*Sim, ArcPreview, ContactCandidate);
	ContactCandidate.Candidate.NodeIndex = FMath::Clamp(ContactCandidate.Candidate.NodeIndex, 1, PreviewSim.Num() - 1);
	return BuildWrappingPreviewFromCandidate(Input, ContactCandidate.Candidate, PreviewSim, OutPreview, OutFailureReason);
}

bool FRopeThrowPreviewBuilder::BuildFlightWrappingPreview(const FInput& Input, FRopeWrapPreviewData& OutPreview,
	FString* OutFailureReason)
{
	OutPreview = FRopeWrapPreviewData();
	const FRopeSimState* Sim = Input.Sim;
	const TArray<IRopeCollider*>& Colliders = GetColliders(Input);
	if (Colliders.Num() == 0)
	{
		SetPreviewFailureReason(OutFailureReason, TEXT("flight preview rejected: no frame colliders"));
		return false;
	}
	if (!Sim || Sim->Num() < 2)
	{
		SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("flight preview rejected: rope sim has too few nodes (nodes=%d)"), Sim ? Sim->Num() : 0));
		return false;
	}

	TArray<FRopeContactCandidate> Candidates;
	const FRopeFlightContactDetector::FParams Params = MakeFlightDetectParams(Input);
	FRopeFlightContactDetector::DetectContactCandidates(*Sim, Colliders, Params, Candidates);
	FRopeFlightContactDetector::FWhipGuideView EmptyWhip;
	FRopeFlightContactDetector::AddPredictedContactCandidates(*Sim, Colliders, Params, EmptyWhip, Candidates);
	FRopeFlightContactDetector::EvaluateRelativeMotion(*Sim, Params, Candidates);

	FRopeContactTracker PreviewTracker;
	PreviewTracker.Update(Candidates, 0.0f);
	if (PreviewTracker.CandidateBone.IsNone() || PreviewTracker.CandidateNodes.Num() == 0)
	{
		SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("flight preview found no tracked candidate (candidates=%d)"), Candidates.Num()));
		return false;
	}

	const int32 NodeIndex = FindHeadValidNodeIndex(PreviewTracker.CandidateNodes, *Sim);
	const FRopeContactCandidate* BestCandidate = nullptr;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid ||
			Candidate.NodeIndex != NodeIndex ||
			Candidate.Bone != PreviewTracker.CandidateBone ||
			Candidate.Mesh != PreviewTracker.CandidateMesh)
		{
			continue;
		}

		if (!BestCandidate || Candidate.Penetration > BestCandidate->Penetration)
		{
			BestCandidate = &Candidate;
		}
	}

	if (!BestCandidate)
	{
		SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("flight preview had tracker bone but no matching best candidate (candidates=%d, bone=%s, nodes=%d)"),
				Candidates.Num(), *PreviewTracker.CandidateBone.ToString(), PreviewTracker.CandidateNodes.Num()));
		return false;
	}

	return BuildWrappingPreviewFromCandidate(Input, *BestCandidate, *Sim, OutPreview, OutFailureReason);
}

bool FRopeThrowPreviewBuilder::BuildWrappingPreviewFromCandidate(const FInput& Input,
	const FRopeContactCandidate& Candidate, const FRopeSimState& SourceSim,
	FRopeWrapPreviewData& OutPreview, FString* OutFailureReason)
{
	OutPreview = FRopeWrapPreviewData();
	const USkeletalMeshComponent* Mesh = Candidate.Mesh;
	if (!Candidate.bValid || !Mesh || Candidate.Bone.IsNone() ||
		!SourceSim.Positions.IsValidIndex(Candidate.NodeIndex))
	{
		SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("wrap preview candidate invalid (valid=%d, mesh=%s, bone=%s, node=%d, sourceNodes=%d)"),
				Candidate.bValid ? 1 : 0, *GetNameSafe(Mesh), *Candidate.Bone.ToString(), Candidate.NodeIndex,
				SourceSim.Num()));
		return false;
	}

	const FVector NormalWorld = Candidate.Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector TangentWorld = FVector::ForwardVector;
	if (SourceSim.Positions.IsValidIndex(Candidate.NodeIndex + 1))
	{
		TangentWorld = SourceSim.Positions[Candidate.NodeIndex + 1] - SourceSim.Positions[Candidate.NodeIndex];
	}
	else if (SourceSim.Positions.IsValidIndex(Candidate.NodeIndex - 1))
	{
		TangentWorld = SourceSim.Positions[Candidate.NodeIndex] - SourceSim.Positions[Candidate.NodeIndex - 1];
	}
	else
	{
		TangentWorld = FRopeFlightContactDetector::ExpectedWrapTangent(SourceSim, Candidate, Input.FallbackForward);
	}
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(NormalWorld));

	const FTransform BoneXform = Mesh->GetSocketTransform(Candidate.Bone);

	FRopeSurfaceAnchor LatchAnchor;
	LatchAnchor.NodeIndex = Candidate.NodeIndex;
	LatchAnchor.Bone = Candidate.Bone;
	LatchAnchor.Mesh = Mesh;
	LatchAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Candidate.WorldPoint);
	LatchAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	LatchAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	LatchAnchor.StartWorldPosition = SourceSim.Positions[Candidate.NodeIndex];
	LatchAnchor.SurfaceOffset = FMath::Max(0.0f, Input.RopeRadius);
	LatchAnchor.RopeDistance = 0.0f;

	TArray<FVector> PreviewPoints;
	FRopeWrappingPhase PreviewWrappingPhase;
	if (!PreviewWrappingPhase.BuildPreviewCenterline(LatchAnchor, Mesh, Candidate.Bone,
		SourceSim, MakeWrappingContext(Input), PreviewPoints))
	{
		SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("wrap preview centerline build failed (mesh=%s, bone=%s, node=%d, sourceNodes=%d)"),
				*GetNameSafe(Mesh), *Candidate.Bone.ToString(), Candidate.NodeIndex, SourceSim.Num()));
		return false;
	}

	OutPreview.Points = MoveTemp(PreviewPoints);
	OutPreview.Radius = FMath::Max(0.1f, Input.RopeRadius * 1.05f);
	OutPreview.NumSides = FMath::Clamp(Input.RopeNumSides, 3, 32);
	if (!OutPreview.IsValid())
	{
		SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("wrap preview output invalid (points=%d, radius=%.2f, sides=%d)"),
				OutPreview.Points.Num(), OutPreview.Radius, OutPreview.NumSides));
	}
	return OutPreview.IsValid();
}
