// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeThrowPreviewBuilder.h"

#include "Collision/RopeCollider.h"
// ResolveBindingWorld is the single point at which a wrap binding, meaning a bone, socket or component, is resolved to a transform.
#include "Core/RopeWrapTarget.h"
#include "Logic/RopeWrappingPhase.h"
#include "RopeMathHelpers.h"
#include "Components/SkeletalMeshComponent.h"

namespace
{
	const TArray<IRopeCollider*>& GetColliders(const FRopeThrowPreviewBuilder::FInput& Input)
	{
		static const TArray<IRopeCollider*> EmptyColliders;
		return Input.Colliders ? *Input.Colliders : EmptyColliders;
	}

	struct FThrowPreviewContactCandidate
	{
		FRopeContactCandidate Candidate;
		/** The direction, from origin to hit, along which the nodes left behind the contact point are laid out. Unused when the contact is the last node. */
		FVector Direction = FVector::ForwardVector;
	};

	bool BuildAimGuideHitCandidate(const FRopeThrowContext& ThrowContext,
		const FRopeSimState& Sim, FThrowPreviewContactCandidate& OutCandidate)
	{
		OutCandidate = FThrowPreviewContactCandidate();
		const USceneComponent* GuideMesh = ThrowContext.AimGuideMesh.Get();
		if (!ThrowContext.bHasAimGuideHit || !GuideMesh || ThrowContext.AimGuideBone.IsNone() || Sim.Num() < 2)
		{
			return false;
		}

		const FVector ToHit = ThrowContext.AimGuideHitWorldPos - ThrowContext.Origin;
		const float HitDistance = ToHit.Size();
		const FVector HitDir = ToHit.GetSafeNormal();
		if (HitDistance <= KINDA_SMALL_NUMBER || HitDir.IsNearlyZero())
		{
			return false;
		}

		const float SegmentLength = FMath::Max(Sim.SegmentLength, 1.0f);
		const int32 DistanceNodeIndex = FMath::Clamp(FMath::RoundToInt(HitDistance / SegmentLength), 1, Sim.Num() - 1);
		// The lock alpha is only the spline's spatial interpolation range and is not the latch position, so the node at the real hit distance is used.
		const int32 AimGuideNodeIndex = DistanceNodeIndex;

		// Aiming with the aim ray has already picked a bone through a swept SDF or collider query.
		// A prepared candidate uses that hit as it stands: sweeping the area around the throw direction again would
		// pick a different bone in a different direction.
		// The surface point is an SDF projection and can differ from the yellow hit on the ray, so the reference for
		// the spline direction has to be the world hit position.
		// This candidate's node is decided by the real hit distance alone. The aim guide lock alpha and direction
		// bias are curve interpolation settings for the physical flight and do not move the prepared latch position.
		FRopeContactCandidate Candidate;
		Candidate.bValid = true;
		Candidate.NodeIndex = AimGuideNodeIndex;
		Candidate.Bone = ThrowContext.AimGuideBone;
		Candidate.Mesh = GuideMesh;
		Candidate.Source = ERopeContactCandidateSource::PredictiveFree;
		Candidate.SourceMask = static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree);
		Candidate.WorldPoint = ThrowContext.AimGuideHitWorldPos;
		Candidate.Normal = ThrowContext.AimGuideNormal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		Candidate.Penetration = 0.0f;
		Candidate.WrapDirectionScore = 0.0f;

		OutCandidate.Candidate = Candidate;
		OutCandidate.Direction = HitDir;
		return true;
	}

	FRopeSimState BuildThrowPreviewSim(const FRopeSimState& SourceSim, const FVector& Origin,
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
		const FVector ToSurface = SurfacePoint - Origin;
		const FVector ApproachDir = ToSurface.GetSafeNormal(KINDA_SMALL_NUMBER, ContactCandidate.Direction);
		const FVector TailDir = ContactCandidate.Direction.GetSafeNormal(KINDA_SMALL_NUMBER, ApproachDir);

		for (int32 NodeIndex = 0; NodeIndex < NumNodes; ++NodeIndex)
		{
			FVector Position = FVector::ZeroVector;
			if (NodeIndex <= LatchNode)
			{
				// The aim hit direction is already fixed, so the straight line from origin to hit is the authoritative shape of the spline prefix.
				const float Alpha = LatchNode > 0
					? static_cast<float>(NodeIndex) / static_cast<float>(LatchNode)
					: 0.0f;
				Position = (NodeIndex == LatchNode)
					? SurfacePoint
					: FMath::Lerp(Origin, SurfacePoint, Alpha);
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

	// The collider storage is owned by the caller: the context holds the array by reference, so building temporary
	// storage here would leave it dangling. It goes through the same gate as the runtime path
	// (URopeComponent::MakeWrappingContext), so the preview's chosen target and the real wrapping path see the same set.
	FRopeWrappingPhase::FContext MakeWrappingContext(const FRopeThrowPreviewBuilder::FInput& Input,
		TArray<IRopeCollider*>& OutColliderStorage)
	{
		RopeWrapTargets::FilterWrappableColliders(GetColliders(Input),
			[&Input](const USceneComponent* Mesh, FName Bone)
			{
				// Unset, as in a unit test or a caller with no gate, permits everything, matching the default CanWrapTarget implementation.
				return !Input.CanWrapTarget || Input.CanWrapTarget(Mesh, Bone);
			},
			OutColliderStorage);

		FRopeWrappingPhase::FContext Ctx{
			Input.WrapConfig,
			OutColliderStorage,
			Input.RopeRadius,
			Input.OwnerName
		};
		Ctx.ResolveMode = Input.ResolveMode;
		return Ctx;
	}

	void BuildPreparedAnchorsFromCenterline(const TArray<FVector>& Centerline, const FRopeContactCandidate& Candidate,
		const USceneComponent* Mesh, TArray<FRopeSurfaceAnchor>& OutAnchors)
	{
		OutAnchors.Reset();
		if (!Mesh || Candidate.Bone.IsNone())
		{
			return;
		}

		const FTransform BoneXform = ResolveBindingWorld(Mesh, Candidate.Bone);
		const int32 FirstNode = FMath::Clamp(Candidate.NodeIndex, 1, Centerline.Num() - 1);
		for (int32 NodeIndex = FirstNode; NodeIndex < Centerline.Num(); ++NodeIndex)
		{
			FRopeSurfaceAnchor Anchor;
			Anchor.NodeIndex = NodeIndex;
			Anchor.Bone = Candidate.Bone;
			Anchor.Mesh = Mesh;
			Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Centerline[NodeIndex]);
			Anchor.LocalNormal = FVector::UpVector;
			Anchor.LocalTangent = FVector::ForwardVector;
			if (Centerline.IsValidIndex(NodeIndex + 1))
			{
				Anchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(
					Centerline[NodeIndex + 1] - Centerline[NodeIndex]).GetSafeNormal(
						KINDA_SMALL_NUMBER, FVector::ForwardVector);
			}
			Anchor.StartWorldPosition = Centerline[NodeIndex];
			Anchor.SurfaceOffset = 0.0f;
			Anchor.RopeDistance = 0.0f;
			OutAnchors.Add(Anchor);
		}
	}

	void ForceAimGuidePrefixToHit(const FRopeThrowPreviewBuilder::FInput& Input,
		const FRopeContactCandidate& Candidate, TArray<FVector>& InOutPreviewPoints)
	{
		if (!Input.ThrowContext.bHasAimGuideHit ||
			!InOutPreviewPoints.IsValidIndex(0) ||
			!InOutPreviewPoints.IsValidIndex(Candidate.NodeIndex))
		{
			return;
		}

		const FVector Origin = Input.ThrowContext.Origin;
		const FVector Hit = Input.ThrowContext.AimGuideHitWorldPos;
		if ((Hit - Origin).IsNearlyZero())
		{
			return;
		}

		// Note that this function is for the fixed render preview of GuaranteedWrap alone. It is never called for the
		// whip guide of the physical flight, so fixing the hit point here has nothing to do with flight nodes being
		// pinned in advance.
		// BuildPreviewCenterline builds the wrapping path after the latch and can overwrite the latch node with the
		// surface path. When aiming with the aim ray, the spline prefix on screen has to point at the ray hit point,
		// so after the final render points are produced the span from the start to the latch node is pinned back onto
		// the straight origin-to-hit line.
		const int32 LastPrefixNode = FMath::Clamp(Candidate.NodeIndex, 1, InOutPreviewPoints.Num() - 1);
		for (int32 NodeIndex = 0; NodeIndex <= LastPrefixNode; ++NodeIndex)
		{
			const float Alpha = static_cast<float>(NodeIndex) / static_cast<float>(LastPrefixNode);
			InOutPreviewPoints[NodeIndex] = FMath::Lerp(Origin, Hit, Alpha);
		}
	}

	// Pierce lays every node out along the straight line from the hand origin to the embed point, so that the rope's
	// end, meaning the spear tip, lands there. No rope is left behind the tip: the spare length is slack between the
	// hand and the tip and the solver drapes it.
	void BuildPierceStraightCenterline(const FRopeThrowPreviewBuilder::FInput& Input,
		const FRopeContactCandidate& Candidate, const FRopeSimState& SourceSim, TArray<FVector>& OutCenterline)
	{
		OutCenterline = SourceSim.Positions;
		const int32 N = OutCenterline.Num();
		if (N < 2)
		{
			return;
		}

		const FVector Origin = Input.ThrowContext.Origin;
		const FVector Hit = Candidate.WorldPoint;
		const int32 Last = N - 1;

		// A straight line covering the whole rope from the hand at node zero to the tip at the last node, where the tip embeds. Nothing is left behind it.
		for (int32 NodeIndex = 0; NodeIndex <= Last; ++NodeIndex)
		{
			const float Alpha = static_cast<float>(NodeIndex) / static_cast<float>(Last);
			OutCenterline[NodeIndex] = FMath::Lerp(Origin, Hit, Alpha);
		}
	}

	bool BuildPreparedFromCandidate(const FRopeThrowPreviewBuilder::FInput& Input,
		const FRopeContactCandidate& Candidate, const FRopeSimState& SourceSim,
		FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason)
	{
		OutPrepared.Reset();
		const USceneComponent* Mesh = Candidate.Mesh;
		if (!Candidate.bValid || !Mesh || Candidate.Bone.IsNone() ||
			!SourceSim.Positions.IsValidIndex(Candidate.NodeIndex))
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
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
		TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(NormalWorld));

		const FTransform BoneXform = ResolveBindingWorld(Mesh, Candidate.Bone);

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

		// GuaranteedWrap means pierce: it skips the wrapping path build and the path anchor expansion and establishes
		// a single anchor at the aim hit contact point. The render preview is the straight line from the hand to the
		// embed point, for presentation. FinishGuidedThrow then promotes those anchors, of which there is one,
		// straight into the wrapped seed, leaving the commit path unchanged.
		if (Input.ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
		{
			// It is the tip, the spear, that embeds, so the anchor has to be the rope's end, meaning the last node
			// where the tip mesh sits, rather than the distance-based contact node. Otherwise an interior node is
			// pinned and the tip plus the spare rope hangs below the contact point. The anchor's local position is
			// already the embed point.
			LatchAnchor.NodeIndex = SourceSim.Num() - 1;
			LatchAnchor.StartWorldPosition = Candidate.WorldPoint;

			TArray<FVector> StraightPoints;
			BuildPierceStraightCenterline(Input, Candidate, SourceSim, StraightPoints);

			OutPrepared.RenderPreview.Points = MoveTemp(StraightPoints);
			OutPrepared.RenderPreview.Radius = FMath::Max(0.1f, Input.RopeRadius * 1.05f);
			OutPrepared.RenderPreview.NumSides = FMath::Clamp(Input.RopeNumSides, 3, 32);
			if (!OutPrepared.RenderPreview.IsValid())
			{
				RopeMath::SetPreviewFailureReason(OutFailureReason,
					FString::Printf(TEXT("pierce preview output invalid (points=%d, node=%d, sourceNodes=%d)"),
						OutPrepared.RenderPreview.Points.Num(), Candidate.NodeIndex, SourceSim.Num()));
				return false;
			}

			OutPrepared.bValid = true;
			OutPrepared.ThrowContext = Input.ThrowContext;
			OutPrepared.LatchAnchor = LatchAnchor;
			OutPrepared.Mesh = Mesh;
			OutPrepared.Bone = Candidate.Bone;
			OutPrepared.Anchors.Reset();
			OutPrepared.Anchors.Add(LatchAnchor); // A single anchor is the normal shape of a pierce.

			return OutPrepared.IsValid();
		}

		TArray<FVector> PreviewPoints;
		FRopeWrappingPhase PreviewWrappingPhase;
		// Storage for the colliders that passed the gate, which has to outlive the context that holds it by reference.
		TArray<IRopeCollider*> WrappableColliders;
		if (!PreviewWrappingPhase.BuildPreviewCenterline(LatchAnchor,
			SourceSim, MakeWrappingContext(Input, WrappableColliders), PreviewPoints))
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("wrap preview centerline build failed (mesh=%s, bone=%s, node=%d, sourceNodes=%d)"),
				*GetNameSafe(Mesh), *Candidate.Bone.ToString(), Candidate.NodeIndex, SourceSim.Num()));
			return false;
		}
		ForceAimGuidePrefixToHit(Input, Candidate, PreviewPoints);

		OutPrepared.RenderPreview.Points = MoveTemp(PreviewPoints);
		OutPrepared.RenderPreview.Radius = FMath::Max(0.1f, Input.RopeRadius * 1.05f);
		OutPrepared.RenderPreview.NumSides = FMath::Clamp(Input.RopeNumSides, 3, 32);
		if (!OutPrepared.RenderPreview.IsValid())
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("wrap preview output invalid (points=%d, radius=%.2f, sides=%d)"),
					OutPrepared.RenderPreview.Points.Num(), OutPrepared.RenderPreview.Radius,
					OutPrepared.RenderPreview.NumSides));
			return false;
		}

		OutPrepared.bValid = true;
		OutPrepared.ThrowContext = Input.ThrowContext;
		OutPrepared.LatchAnchor = LatchAnchor;
		OutPrepared.Mesh = Mesh;
		OutPrepared.Bone = Candidate.Bone;
		BuildPreparedAnchorsFromCenterline(OutPrepared.RenderPreview.Points, Candidate, Mesh, OutPrepared.Anchors);
		if (OutPrepared.Anchors.Num() == 0)
		{
			OutPrepared.Anchors.Add(LatchAnchor);
		}

		return OutPrepared.IsValid();
	}
}

bool FRopeThrowPreviewBuilder::BuildFreePreparedPreview(const FInput& Input, FRopePreparedThrowPreview& OutPrepared,
	FString* OutFailureReason)
{
	OutPrepared.Reset();
	const FRopeSimState* Sim = Input.Sim;
	if (!Sim)
	{
		RopeMath::SetPreviewFailureReason(OutFailureReason, TEXT("prepared preview rejected: no rope sim"));
		return false;
	}

	FThrowPreviewContactCandidate ContactCandidate;
	if (!BuildAimGuideHitCandidate(Input.ThrowContext, *Sim, ContactCandidate))
	{
		// A prepared preview is established for the aimed target alone. Sweeping the area around the throw direction
		// for a candidate because there was no aim hit would pick a neighbouring target unrelated to the aim, and the
		// miss indicator on screen would disagree with the preview's connecting line.
		// It therefore ends here with no alternative search: the caller falls back to StartFreeGuidedThrow, an arc
		// into empty space at the ray's end point, and a missed aim not embedding is the correct outcome.
		// The reason distinguishes an aim that was evaluated and missed from there being no aiming flow at all, as on
		// a direct Blueprint or AI call, because the log alone has to separate an aim configuration problem from a
		// call path problem.
		RopeMath::SetPreviewFailureReason(OutFailureReason, Input.ThrowContext.bAimRayEvaluated
			? TEXT("prepared preview rejected: aim ray found no target")
			: TEXT("prepared preview rejected: requires an aim hit"));
		return false;
	}

	FRopeSimState PreviewSim = BuildThrowPreviewSim(*Sim, Input.ThrowContext.Origin, ContactCandidate);
	ContactCandidate.Candidate.NodeIndex = FMath::Clamp(ContactCandidate.Candidate.NodeIndex, 1, PreviewSim.Num() - 1);
	return BuildPreparedFromCandidate(
		Input, ContactCandidate.Candidate, PreviewSim, OutPrepared, OutFailureReason);
}

