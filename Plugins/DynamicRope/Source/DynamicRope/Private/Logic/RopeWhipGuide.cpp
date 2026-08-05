// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeWhipGuide.h"
// RopeMath::SmoothStep, shared to avoid a duplicate definition in a unity build.
#include "RopeMathHelpers.h"

namespace
{
FVector ProjectAxisOffAim(const FVector& Axis, const FVector& AimDir, const FVector& Fallback)
{
	const FVector Aim = FRopeWhipGuide::SafeNormalOr(AimDir, FVector::ForwardVector);
	FVector Projected = Axis - FVector::DotProduct(Axis, Aim) * Aim;
	if (Projected.IsNearlyZero())
	{
		Projected = Fallback - FVector::DotProduct(Fallback, Aim) * Aim;
	}
	if (Projected.IsNearlyZero())
	{
		Projected = RopeMath::AnyTangentFromNormal(Aim);
	}
	return Projected.GetSafeNormal();
}

float ResolveGuideDuration(const FRopeWhipGuide::FConfig& Config, float ThrowSpeed)
{
	const float BaseDuration = FMath::Max(Config.Duration, KINDA_SMALL_NUMBER);
	if (ThrowSpeed <= KINDA_SMALL_NUMBER)
	{
		return BaseDuration;
	}

	const float ReferenceSpeed = FMath::Max(Config.ReferenceThrowSpeed, KINDA_SMALL_NUMBER);
	return FMath::Clamp(BaseDuration * ReferenceSpeed / ThrowSpeed, KINDA_SMALL_NUMBER, 10.0f);
}

float ResolveGuideBlendEnd(float FullyGuidedEnd, const FRopeWhipGuide::FConfig& Config)
{
	return FMath::Clamp(
		FMath::Clamp(FullyGuidedEnd, 0.05f, 1.0f) +
		FMath::Clamp(Config.AimHitTipSolverFraction, 0.0f, 0.45f),
		0.05f, 1.0f);
}

// FullyGuidedEnd is the end of the fully controlled span, not a binary ownership cut. Influence stays
// at one through that point, then falls smoothly to zero over the available Tip Physics Blend range.
// Both Assisted and Full Simulation use this same tail handoff.
float GuideTailEnvelope(float RopeAlpha, float FullyGuidedEnd, const FRopeWhipGuide::FConfig& Config)
{
	const float ClampedGuidedEnd = FMath::Clamp(FullyGuidedEnd, 0.05f, 1.0f);
	const float BlendEnd = ResolveGuideBlendEnd(ClampedGuidedEnd, Config);
	float Weight = 1.0f;
	if (RopeAlpha > ClampedGuidedEnd)
	{
		const float BlendRange = BlendEnd - ClampedGuidedEnd;
		Weight = BlendRange > KINDA_SMALL_NUMBER
			? RopeMath::SmoothStep((BlendEnd - RopeAlpha) / BlendRange)
			: 0.0f;
	}
	return Weight;
}

float AimGuideEnvelope(float RopeAlpha, float GuidedEnd, const FRopeWhipGuide::FConfig& Config)
{
	const float RootRange = FMath::Clamp(Config.AimHitRootSolverFraction, 0.0f, 0.45f);
	const float RootWeight = RootRange > KINDA_SMALL_NUMBER
		? RopeMath::SmoothStep(RopeAlpha / RootRange)
		: 1.0f;
	return FMath::Min(RootWeight, GuideTailEnvelope(RopeAlpha, GuidedEnd, Config));
}

float ResolveOrdinaryFullyGuidedEnd(float NormalizedTime, float GuidedEnd)
{
	return FMath::Lerp(1.0f, GuidedEnd, RopeMath::SmoothStep(NormalizedTime));
}

float ResolveFullSimCurveWeight(float NormalizedTime, const FRopeWhipGuide::FConfig& Config)
{
	const float StraightenTime = FMath::Clamp(
		Config.FullSimStraightenTimeFraction, KINDA_SMALL_NUMBER, 1.0f);
	return 1.0f - RopeMath::SmoothStep(
		FMath::Clamp(NormalizedTime / StraightenTime, 0.0f, 1.0f));
}

int32 ResolveLastGuidedNode(int32 LastNode, float InfluenceEnd)
{
	if (LastNode < 1)
	{
		return LastNode;
	}
	return FMath::Clamp(
		FMath::FloorToInt(static_cast<float>(LastNode) * FMath::Clamp(InfluenceEnd, 0.05f, 1.0f)),
		1, LastNode);
}

FVector ResolveStraightGuideDirection(const TArray<FVector>& GuideTargets, const FVector& Fallback)
{
	if (GuideTargets.Num() >= 2)
	{
		const FVector Direction = (GuideTargets.Last() - GuideTargets[0]).GetSafeNormal();
		if (!Direction.IsNearlyZero())
		{
			return Direction;
		}
	}
	return FRopeWhipGuide::SafeNormalOr(Fallback, FVector::ForwardVector);
}

// A spatially varying Lerp between the three-dimensional solver pose and a straight guide bends the
// result wherever the weight changes. Instead, both inputs are reduced to their scalar coordinate on
// one shared line before blending. The mode-specific root and shared tail envelopes can still relax
// influence, while every node that remains guide-owned is collinear.
FVector BlendOnStraightGuide(const FVector& SolverPosition, const FVector& GuideTarget,
	const FVector& LineOrigin, const FVector& LineDirection, float GuideWeight)
{
	const FVector Direction = FRopeWhipGuide::SafeNormalOr(LineDirection, FVector::ForwardVector);
	const float SolverDistance = FVector::DotProduct(SolverPosition - LineOrigin, Direction);
	const float GuideDistance = FVector::DotProduct(GuideTarget - LineOrigin, Direction);
	const float BlendedDistance = FMath::Lerp(
		SolverDistance, GuideDistance, FMath::Clamp(GuideWeight, 0.0f, 1.0f));
	return LineOrigin + Direction * BlendedDistance;
}

// The raw spline is resampled to the segment length, but the per-node aim envelope and solver blend applied
// afterwards can stretch the spacing again. The guided run is projected sequentially from the root end, which limits
// the maximum length of each target edge alone.
// The free end and solver-owned nodes are not modified, so the endpoint envelope contract and the real solver motion are preserved.
void ClampGuidedTargetStretch(const FVector& RootPosition, float NodeSpacing,
	const TArray<uint8>& GuidedMask, TArray<FVector>& InOutTargets)
{
	const int32 Count = FMath::Min(InOutTargets.Num(), GuidedMask.Num());
	const float SegmentLength = FMath::Max(NodeSpacing, KINDA_SMALL_NUMBER);
	FVector Leader = RootPosition;
	for (int32 NodeIndex = 1; NodeIndex < Count; ++NodeIndex)
	{
		if (GuidedMask[NodeIndex] == 0)
		{
			Leader = InOutTargets[NodeIndex];
			continue;
		}

		const FVector Delta = InOutTargets[NodeIndex] - Leader;
		const float Distance = Delta.Size();
		if (Distance > SegmentLength && Distance > KINDA_SMALL_NUMBER)
		{
			InOutTargets[NodeIndex] = Leader + Delta * (SegmentLength / Distance);
		}
		Leader = InOutTargets[NodeIndex];
	}
}

}

FVector FRopeWhipGuide::SafeNormalOr(const FVector& Value, const FVector& Fallback)
{
	const FVector Normalized = Value.GetSafeNormal();
	return Normalized.IsNearlyZero() ? Fallback.GetSafeNormal() : Normalized;
}

FRopeWhipGuide::FSwingBasis FRopeWhipGuide::ResolveSwingBasis(const FRopeThrowContext& ThrowContext,
	ERopeSwingPlane SwingPlane, const FVector& CustomPlaneNormal)
{
	FSwingBasis Basis;
	Basis.AimDir = SafeNormalOr(ThrowContext.FrameForward, FVector::ForwardVector);

	const FVector FrameUp = ProjectAxisOffAim(ThrowContext.FrameUp, Basis.AimDir, FVector::UpVector);
	const FVector FrameRight = ProjectAxisOffAim(ThrowContext.FrameRight, Basis.AimDir, FVector::CrossProduct(FrameUp, Basis.AimDir));

	switch (SwingPlane)
	{
	case ERopeSwingPlane::AimAndFrameDown:
		Basis.GuideUp = -FrameUp;
		break;
	case ERopeSwingPlane::AimAndFrameRight:
		Basis.GuideUp = FrameRight;
		break;
	case ERopeSwingPlane::AimAndFrameLeft:
		Basis.GuideUp = -FrameRight;
		break;
	case ERopeSwingPlane::CustomNormal:
	{
		const FVector PlaneNormal = SafeNormalOr(CustomPlaneNormal, FVector::CrossProduct(Basis.AimDir, FrameUp));
		Basis.GuideUp = FVector::CrossProduct(PlaneNormal, Basis.AimDir).GetSafeNormal();
		if (Basis.GuideUp.IsNearlyZero())
		{
			Basis.GuideUp = FrameUp;
		}
		break;
	}
	case ERopeSwingPlane::AimAndFrameUp:
	default:
		Basis.GuideUp = FrameUp;
		break;
	}

	Basis.GuideUp = ProjectAxisOffAim(Basis.GuideUp, Basis.AimDir, FrameUp);
	Basis.GuideRight = FVector::CrossProduct(Basis.GuideUp, Basis.AimDir).GetSafeNormal();
	if (Basis.GuideRight.IsNearlyZero())
	{
		Basis.GuideRight = FrameRight;
	}

	return Basis;
}

void FRopeWhipGuide::Begin(const FVector& InAimDir, const FVector& InOrigin,
	const FVector& FallbackAim, const FVector& FallbackUp, const FVector& FallbackSide,
	float InThrowSpeed, const FVector& InInheritedVelocity, bool bInHasAimTarget,
	const FVector& InAimTarget, float InAimSteerStartAlpha, float InAimLockAlpha)
{
	AimDir = InAimDir.GetSafeNormal();
	if (AimDir.IsNearlyZero())
	{
		AimDir = FallbackAim;
	}
	Origin = InOrigin;
	GuideForward = AimDir;
	GuideUp = FallbackUp.GetSafeNormal();
	if (GuideUp.IsNearlyZero())
	{
		GuideUp = FVector::UpVector;
	}
	if (FMath::Abs(FVector::DotProduct(GuideForward, GuideUp)) > 0.96f)
	{
		GuideUp = FallbackSide.GetSafeNormal();
		if (GuideUp.IsNearlyZero() || FMath::Abs(FVector::DotProduct(GuideForward, GuideUp)) > 0.96f)
		{
			GuideUp = FVector::UpVector;
		}
	}
	FVector GuideSide = FVector::CrossProduct(GuideUp, GuideForward).GetSafeNormal();
	if (GuideSide.IsNearlyZero())
	{
		GuideSide = FallbackSide.GetSafeNormal();
	}
	GuideUp = FVector::CrossProduct(GuideForward, GuideSide).GetSafeNormal();
	GuideThrowSpeed = InThrowSpeed;
	GuideInheritedVelocity = InInheritedVelocity;
	// An aim hit is stored not as a fixed coordinate but as the final direction from the origin towards the hit, plus its interpolation ranges.
	bHasAimTarget = bInHasAimTarget && !(InAimTarget - InOrigin).IsNearlyZero();
	AimTarget = InAimTarget;
	AimSteerStartAlpha = FMath::Clamp(InAimSteerStartAlpha, 0.0f, 0.9f);
	AimLockAlpha = FMath::Clamp(FMath::Max(InAimLockAlpha, AimSteerStartAlpha + 0.01f), 0.05f, 1.0f);

	Elapsed = 0.0f;
	bActive = true;

	// Starting a new swing clears the previous swing's per-frame products so they cannot leak into this throw.
	PrevTargetsThisFrame.Reset();
	CurrentTargetsThisFrame.Reset();
	GuidedNodesThisFrame.Reset();
}

void FRopeWhipGuide::SnapToInitialPose(FRopeSimState& Sim, const FConfig& Config)
{
	const int32 LastNode = Sim.Num() - 1;
	if (LastNode < 1)
	{
		return;
	}

	// Full Simulation starts on its initial C-shaped guide, including the future solver-owned tail.
	// Advance removes that curvature by the configured normalized time while releasing the tail gradually.
	// An aim hit keeps its separate straight endpoint-envelope contract.
	const float GuidedEnd = FMath::Clamp(Config.GuidedLength, 0.05f, 1.0f);
	const float InfluenceEnd = bHasAimTarget ? ResolveGuideBlendEnd(GuidedEnd, Config) : 1.0f;
	const int32 LastGuidedNode = ResolveLastGuidedNode(LastNode, InfluenceEnd);
	TArray<FVector> GuideTargets;
	// Keep the target array full-sized for the CPU/GPU frame contract; LastGuidedNode controls ownership.
	BuildGuideTargets(0.0f, LastNode, Sim, Config, GuideTargets);
	PreviousTargets = GuideTargets;
	PrevTargetsThisFrame = GuideTargets;
	CurrentTargetsThisFrame = GuideTargets;
	GuidedNodesThisFrame.SetNumZeroed(Sim.Num());
	const FVector GuideOrigin = GuideTargets[0];
	const FVector GuideDirection = ResolveStraightGuideDirection(GuideTargets, GuideForward);
	for (int32 i = 1; i <= LastGuidedNode; ++i)
	{
		if (!GuideTargets.IsValidIndex(i))
		{
			break;
		}

		// The same envelope places the middle on the initial line and relaxes towards both ends. Solver
		// influence changes only the coordinate along that line, never its transverse shape.
		const float S = static_cast<float>(i) / static_cast<float>(LastNode);
		const float GuideWeight = bHasAimTarget ? AimGuideEnvelope(S, GuidedEnd, Config) : 1.0f;
		const FVector Target = GuideTargets[i];
		const FVector Position = bHasAimTarget
			? (GuideWeight > KINDA_SMALL_NUMBER
				? BlendOnStraightGuide(
					Sim.Positions[i], Target, GuideOrigin, GuideDirection, GuideWeight)
				: Sim.Positions[i])
			: Target;
		Sim.Positions[i] = Position;
		Sim.PrevPositions[i] = Position;
		CurrentTargetsThisFrame[i] = Position;
		PrevTargetsThisFrame[i] = Position;
		if (GuideWeight > KINDA_SMALL_NUMBER && GuidedNodesThisFrame.IsValidIndex(i))
		{
			GuidedNodesThisFrame[i] = 1;
		}
	}

	// After the final envelope blend, the guided targets alone are made inextensible again. The free end keeps its existing solver pose.
	const FVector Root = Sim.bStartPinned ? Sim.StartPinTarget : Sim.Positions[0];
	ClampGuidedTargetStretch(Root, Sim.SegmentLength, GuidedNodesThisFrame, CurrentTargetsThisFrame);
	for (int32 i = 1; i <= LastGuidedNode; ++i)
	{
		if (GuidedNodesThisFrame.IsValidIndex(i) && GuidedNodesThisFrame[i] != 0 &&
			CurrentTargetsThisFrame.IsValidIndex(i) && PrevTargetsThisFrame.IsValidIndex(i))
		{
			Sim.Positions[i] = CurrentTargetsThisFrame[i];
			Sim.PrevPositions[i] = CurrentTargetsThisFrame[i];
			PrevTargetsThisFrame[i] = CurrentTargetsThisFrame[i];
		}
	}
}

void FRopeWhipGuide::Advance(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config)
{
	ResetFrameOutputs();

	if (Sim.Num() < 3)
	{
		bActive = false;
		return;
	}

	Elapsed += DeltaTime;
	const float Duration = ResolveGuideDuration(Config, GuideThrowSpeed);
	const float T = FMath::Clamp(Elapsed / Duration, 0.0f, 1.0f);
	const float PreviousT = FMath::Clamp((Elapsed - DeltaTime) / Duration, 0.0f, 1.0f);
	const int32 LastNode = Sim.Num() - 1;
	// A value of one intentionally keeps the whole rope hard-guided until the flight guide ends.
	// Do not silently reduce it: doing so creates a moving solver/guide boundary in the tail.
	const float GuidedEnd = FMath::Clamp(Config.GuidedLength, 0.05f, 1.0f);
	// Full Simulation moves the fully guided boundary from the initially seeded tip towards GuidedLength.
	// The shared tail envelope adds a smooth handoff after that boundary instead of releasing nodes in a
	// binary step. Assisted keeps a fixed fully guided boundary at GuidedLength.
	const float FullyGuidedEnd = bHasAimTarget
		? GuidedEnd
		: ResolveOrdinaryFullyGuidedEnd(T, GuidedEnd);
	const float InfluenceEnd = ResolveGuideBlendEnd(FullyGuidedEnd, Config);
	const int32 LastGuidedNode = ResolveLastGuidedNode(LastNode, InfluenceEnd);
	TArray<FVector> GuideTargets;
	BuildGuideTargets(T, LastNode, Sim, Config, GuideTargets);
	if (GuideTargets.Num() == 0)
	{
		bActive = false;
		return;
	}

	PrevTargetsThisFrame = Sim.PrevPositions;
	CurrentTargetsThisFrame = Sim.Positions;
	GuidedNodesThisFrame.SetNumZeroed(Sim.Num());
	const FVector CurrentGuideDirection = ResolveStraightGuideDirection(GuideTargets, GuideForward);
	const FVector PreviousGuideDirection = PreviousTargets.Num() >= 2
		? ResolveStraightGuideDirection(PreviousTargets, CurrentGuideDirection)
		: CurrentGuideDirection;
	// BuildGuideTargets anchors the current line at the live pin, while PreviousTargets retains the
	// preceding line's root. Their difference carries hand motion in the Verlet state without shifting
	// the target-aligned line away from the locked hit.
	const FVector CurrentGuideOrigin = GuideTargets[0];
	const FVector PreviousGuideOrigin = PreviousTargets.Num() > 0
		? PreviousTargets[0]
		: CurrentGuideOrigin;
	const bool bCurrentFullSimCurve = !bHasAimTarget &&
		ResolveFullSimCurveWeight(T, Config) > KINDA_SMALL_NUMBER;
	const bool bPreviousFullSimCurve = !bHasAimTarget &&
		ResolveFullSimCurveWeight(PreviousT, Config) > KINDA_SMALL_NUMBER;

	// This computes the mask and the debug data alone; the actual write is performed by ApplyToSim on the CPU path or
	// by the GPU override pass, which consume the same products, meaning the current and previous targets and the mask.
	for (int32 i = 1; i <= LastGuidedNode; ++i)
	{
		if (Sim.InvMass.IsValidIndex(i) && Sim.InvMass[i] <= 0.0f)
		{
			continue;
		}

		const float S = static_cast<float>(i) / static_cast<float>(LastNode);
		if (!GuideTargets.IsValidIndex(i))
		{
			break;
		}

		// Both modes own the middle through a smooth tail envelope. Full Simulation moves the start of that
		// envelope over time; Assisted keeps it at GuidedLength and additionally relaxes the hand end.
		const float GuideWeight = bHasAimTarget
			? AimGuideEnvelope(S, GuidedEnd, Config)
			: GuideTailEnvelope(S, FullyGuidedEnd, Config);
		if (GuideWeight <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const FVector CurrentGuide = GuideTargets[i];
		const FVector PreviousGuide = PreviousTargets.IsValidIndex(i)
			? PreviousTargets[i]
			: CurrentGuide;
		// Full Simulation follows the actual C target while curvature is active. Once it reaches zero,
		// switch back to longitudinal-only blending so every guided node is exactly collinear. Assisted
		// always uses that straight path to preserve its target alignment.
		CurrentTargetsThisFrame[i] = bCurrentFullSimCurve
			? FMath::Lerp(Sim.Positions[i], CurrentGuide, GuideWeight)
			: BlendOnStraightGuide(
				Sim.Positions[i], CurrentGuide,
				CurrentGuideOrigin, CurrentGuideDirection, GuideWeight);
		PrevTargetsThisFrame[i] = bPreviousFullSimCurve
			? FMath::Lerp(Sim.PrevPositions[i], PreviousGuide, GuideWeight)
			: BlendOnStraightGuide(
				Sim.PrevPositions[i], PreviousGuide,
				PreviousGuideOrigin, PreviousGuideDirection, GuideWeight);

		if (GuidedNodesThisFrame.IsValidIndex(i))
		{
			GuidedNodesThisFrame[i] = 1;
		}

	}

	// So that the endpoint envelope and socket blend applied after ResampleGuideByNodeSpacing do not break the spacing
	// again, the current and previous target runs are each projected sequentially from the current and previous pin.
	const FVector CurrentRoot = Sim.bStartPinned ? Sim.StartPinTarget : Sim.Positions[0];
	const FVector PreviousRoot = Sim.bStartPinned ? Sim.StartPinPrev : Sim.PrevPositions[0];
	ClampGuidedTargetStretch(
		CurrentRoot, Sim.SegmentLength, GuidedNodesThisFrame, CurrentTargetsThisFrame);
	ClampGuidedTargetStretch(
		PreviousRoot, Sim.SegmentLength, GuidedNodesThisFrame, PrevTargetsThisFrame);

	PreviousTargets = GuideTargets;
	bActive = Elapsed < Duration;
}

void FRopeWhipGuide::ApplyToSim(FRopeSimState& Sim) const
{
	for (int32 i = 0; i < GuidedNodesThisFrame.Num() && i < Sim.Num(); ++i)
	{
		if (GuidedNodesThisFrame[i] == 0 || !CurrentTargetsThisFrame.IsValidIndex(i))
		{
			continue;
		}

		Sim.PrevPositions[i] = PrevTargetsThisFrame.IsValidIndex(i) ? PrevTargetsThisFrame[i] : Sim.Positions[i];
		Sim.Positions[i] = CurrentTargetsThisFrame[i];
	}
}

void FRopeWhipGuide::PreviewNextTargets(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config,
	TArray<FVector>& OutTargets) const
{
	OutTargets.Reset();

	const int32 LastNode = Sim.Num() - 1;
	const float GuidedEnd = FMath::Clamp(Config.GuidedLength, 0.05f, 1.0f);
	const float Duration = ResolveGuideDuration(Config, GuideThrowSpeed);
	const float NextT = FMath::Clamp((Elapsed + DeltaTime) / Duration, 0.0f, 1.0f);
	const float FullyGuidedEnd = bHasAimTarget
		? GuidedEnd
		: ResolveOrdinaryFullyGuidedEnd(NextT, GuidedEnd);
	const float InfluenceEnd = ResolveGuideBlendEnd(FullyGuidedEnd, Config);
	const int32 LastGuidedNode = ResolveLastGuidedNode(LastNode, InfluenceEnd);
	TArray<FVector> GuideTargets;
	BuildGuideTargets(NextT, LastNode, Sim, Config, GuideTargets);
	if (GuideTargets.Num() == 0)
	{
		return;
	}

	OutTargets = Sim.Positions;
	TArray<uint8> PreviewGuidedMask;
	PreviewGuidedMask.SetNumZeroed(Sim.Num());
	const FVector GuideOrigin = GuideTargets[0];
	const FVector GuideDirection = ResolveStraightGuideDirection(GuideTargets, GuideForward);
	const bool bFullSimCurve = !bHasAimTarget &&
		ResolveFullSimCurveWeight(NextT, Config) > KINDA_SMALL_NUMBER;
	for (int32 i = 1; i <= LastGuidedNode; ++i)
	{
		if (!GuideTargets.IsValidIndex(i) ||
			(Sim.InvMass.IsValidIndex(i) && Sim.InvMass[i] <= 0.0f))
		{
			continue;
		}

		const float S = static_cast<float>(i) / static_cast<float>(LastNode);
		const float GuideWeight = bHasAimTarget
			? AimGuideEnvelope(S, GuidedEnd, Config)
			: GuideTailEnvelope(S, FullyGuidedEnd, Config);
		if (GuideWeight <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const FVector Target = GuideTargets[i];
		OutTargets[i] = bFullSimCurve
			? FMath::Lerp(Sim.Positions[i], Target, GuideWeight)
			: BlendOnStraightGuide(
				Sim.Positions[i], Target, GuideOrigin, GuideDirection, GuideWeight);
		PreviewGuidedMask[i] = 1;
	}

	const FVector CurrentRoot = Sim.bStartPinned ? Sim.StartPinTarget : Sim.Positions[0];
	ClampGuidedTargetStretch(
		CurrentRoot, Sim.SegmentLength, PreviewGuidedMask, OutTargets);
}

void FRopeWhipGuide::ResetFrameOutputs()
{
	PrevTargetsThisFrame.Reset();
	CurrentTargetsThisFrame.Reset();
	GuidedNodesThisFrame.Reset();
}

int32 FRopeWhipGuide::GetGuidedNodeCountThisFrame() const
{
	int32 Count = 0;
	for (const uint8 bGuided : GuidedNodesThisFrame)
	{
		Count += bGuided != 0 ? 1 : 0;
	}
	return Count;
}

void FRopeWhipGuide::CopyGuidedTargetsForDebug(
	TArray<int32>& OutNodeIndices, TArray<FVector>& OutTargets) const
{
	OutNodeIndices.Reset();
	OutTargets.Reset();
	for (int32 NodeIndex = 0; NodeIndex < GuidedNodesThisFrame.Num(); ++NodeIndex)
	{
		if (GuidedNodesThisFrame[NodeIndex] == 0 || !CurrentTargetsThisFrame.IsValidIndex(NodeIndex))
		{
			continue;
		}

		OutNodeIndices.Add(NodeIndex);
		OutTargets.Add(CurrentTargetsThisFrame[NodeIndex]);
	}
}

void FRopeWhipGuide::BuildGuideTargets(float NormalizedTime, int32 LastGuidedNode,
	const FRopeSimState& Sim, const FConfig& Config, TArray<FVector>& OutTargets) const
{
	OutTargets.Reset();
	if (Sim.Num() < 2 || LastGuidedNode < 0)
	{
		return;
	}

	const float T = FMath::Clamp(NormalizedTime, 0.0f, 1.0f);
	const FVector Forward = GuideForward.GetSafeNormal();
	if (Forward.IsNearlyZero())
	{
		return;
	}

	FVector Up = GuideUp.GetSafeNormal();
	if (Up.IsNearlyZero())
	{
		Up = FVector::UpVector;
	}

	// Rebuild both guide modes from the live hand pin. Keeping Full Simulation at the throw-time Origin
	// while node zero follows StartPinTarget makes the final spacing clamp bridge two different origins,
	// bending an otherwise straight guide as the hand animation moves.
	const FVector HandPos = Sim.bStartPinned ? Sim.StartPinTarget : Origin;
	const float GuidedEnd = FMath::Clamp(Config.GuidedLength, 0.05f, 1.0f);
	const int32 DesiredPointCount = FMath::Clamp(LastGuidedNode + 1, 1, Sim.Num());
	const int32 RawSampleCount = FMath::Max(DesiredPointCount * 4, 16);
	const float GuideLength = FMath::Max(Sim.RopeLength, Config.ComponentRopeLength) * GuidedEnd;
	const float GuideDuration = ResolveGuideDuration(Config, GuideThrowSpeed);
	const FVector InheritedDrift = GuideInheritedVelocity * (GuideDuration * T);
	const FVector LockedAimDir = bHasAimTarget
		? (AimTarget - HandPos).GetSafeNormal(KINDA_SMALL_NUMBER, Forward)
		: Forward;
	// Both modes keep the rotating whip presentation. Assisted rotates one coherent straight line around
	// its locked aim direction, so T=1 passes exactly through the target instead of merely ending parallel
	// to a line that was built at the old throw origin.
	const FVector SweepForward = bHasAimTarget ? LockedAimDir : Forward;
	const FVector SweepUp = ProjectAxisOffAim(Up, SweepForward, FVector::UpVector);
	const float SweepRadians = FMath::DegreesToRadians(
		FMath::Clamp(Config.SweepAngleDegrees, 1.0f, 180.0f));
	const float AngleFromAim = SweepRadians * (1.0f - T);
	FVector GuideDirection = (SweepForward * FMath::Cos(AngleFromAim) +
		SweepUp * FMath::Sin(AngleFromAim)).GetSafeNormal();
	if (T <= KINDA_SMALL_NUMBER)
	{
		GuideDirection = (SweepForward * FMath::Cos(SweepRadians) +
			SweepUp * FMath::Sin(SweepRadians)).GetSafeNormal();
	}
	else if (T >= 1.0f - KINDA_SMALL_NUMBER)
	{
		GuideDirection = SweepForward;
	}

	TArray<FVector> RawPoints;
	RopeMath::BuildWhipGuideRawPoints(HandPos, GuideDirection, LockedAimDir, bHasAimTarget, T,
		GuideLength, InheritedDrift, AimSteerStartAlpha, AimLockAlpha,
		Config.AimHitDirectionBias, RawSampleCount, RawPoints);
	if (!bHasAimTarget && RawPoints.Num() >= 2)
	{
		// A shallow one-sided bow gives Full Simulation a small C silhouette at the throw boundary.
		// Both endpoints stay on the rotating baseline, so at T=0 the free end remains opposite AimDir.
		// The bow fades to exactly zero at StraightenTime, after which this is the ordinary straight guide.
		const float CurveWeight = ResolveFullSimCurveWeight(T, Config);
		const float CurveAmplitude = GuideLength *
			FMath::Clamp(Config.FullSimInitialCurveFraction, 0.0f, 0.35f) * CurveWeight;
		const FVector CurveAxis = ProjectAxisOffAim(SweepUp, GuideDirection, GuideUp);
		for (int32 SampleIndex = 1; SampleIndex + 1 < RawPoints.Num(); ++SampleIndex)
		{
			const float RopeAlpha = static_cast<float>(SampleIndex) /
				static_cast<float>(RawPoints.Num() - 1);
			const float Bow = 4.0f * RopeAlpha * (1.0f - RopeAlpha);
			RawPoints[SampleIndex] += CurveAxis * (CurveAmplitude * Bow);
		}
	}

	ResampleGuideByNodeSpacing(RawPoints, Sim.SegmentLength, DesiredPointCount, OutTargets);
}

void FRopeWhipGuide::ResampleGuideByNodeSpacing(const TArray<FVector>& SourcePoints, float NodeSpacing,
	int32 DesiredPointCount, TArray<FVector>& OutPoints) const
{
	OutPoints.Reset();
	if (SourcePoints.Num() == 0 || DesiredPointCount <= 0)
	{
		return;
	}

	const float SegmentLength = FMath::Max(NodeSpacing, KINDA_SMALL_NUMBER);
	OutPoints.SetNum(DesiredPointCount);
	OutPoints[0] = SourcePoints[0];
	if (DesiredPointCount == 1)
	{
		return;
	}

	TArray<float> Accumulated;
	Accumulated.SetNum(SourcePoints.Num());
	Accumulated[0] = 0.0f;
	for (int32 i = 1; i < SourcePoints.Num(); ++i)
	{
		Accumulated[i] = Accumulated[i - 1] + FVector::Dist(SourcePoints[i - 1], SourcePoints[i]);
	}

	int32 SegmentIdx = 1;
	for (int32 PointIdx = 1; PointIdx < DesiredPointCount; ++PointIdx)
	{
		const float TargetDistance = SegmentLength * static_cast<float>(PointIdx);
		while (SegmentIdx < Accumulated.Num() && Accumulated[SegmentIdx] < TargetDistance)
		{
			++SegmentIdx;
		}

		if (SegmentIdx < Accumulated.Num())
		{
			const float SegmentStartDistance = Accumulated[SegmentIdx - 1];
			const float SegmentDistance = FMath::Max(Accumulated[SegmentIdx] - SegmentStartDistance, KINDA_SMALL_NUMBER);
			const float Alpha = (TargetDistance - SegmentStartDistance) / SegmentDistance;
			OutPoints[PointIdx] = FMath::Lerp(SourcePoints[SegmentIdx - 1], SourcePoints[SegmentIdx], Alpha);
			continue;
		}

		const FVector TailDir = (SourcePoints.Num() >= 2)
			? (SourcePoints.Last() - SourcePoints[SourcePoints.Num() - 2]).GetSafeNormal()
			: AimDir.GetSafeNormal();
		OutPoints[PointIdx] = SourcePoints.Last() + TailDir * (TargetDistance - Accumulated.Last());
	}
}
