// Copyright Epic Games, Inc. All Rights Reserved.

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

// The middle of the rope has a spline weight of one, easing to zero at the hand and the free end, which hands both ends back to the solver.
float AimGuideEnvelope(float RopeAlpha, const FRopeWhipGuide::FConfig& Config)
{
	const float RootRange = FMath::Clamp(Config.AimHitRootSolverFraction, 0.0f, 0.45f);
	const float TipRange = FMath::Clamp(Config.AimHitTipSolverFraction, 0.0f, 0.45f);
	const float RootWeight = RootRange > KINDA_SMALL_NUMBER
		? RopeMath::SmoothStep(RopeAlpha / RootRange)
		: 1.0f;
	const float TipWeight = TipRange > KINDA_SMALL_NUMBER
		? RopeMath::SmoothStep((1.0f - RopeAlpha) / TipRange)
		: 1.0f;
	return FMath::Min(RootWeight, TipWeight);
}

// The difference between the throw origin and the current hand socket is mixed into the root span alone, so it follows the animation naturally.
float AimRootSocketInfluence(float RopeAlpha, const FRopeWhipGuide::FConfig& Config)
{
	const float RootRange = FMath::Clamp(Config.AimHitRootSolverFraction, 0.0f, 0.45f);
	return RootRange > KINDA_SMALL_NUMBER
		? 1.0f - RopeMath::SmoothStep(RopeAlpha / RootRange)
		: 0.0f;
}

// The initial full-rope seed hands ownership back to the solver from the tip towards GuidedEnd.
// Ownership is deliberately binary: blending each node by a different amount bends an otherwise straight
// rotating guide into a hill (or an L shape) even before the physics solver runs.
float OrdinaryGuideOwnership(float RopeAlpha, float NormalizedTime, float GuidedEnd)
{
	const float ReleaseAlpha = RopeMath::SmoothStep(NormalizedTime);
	const float CurrentGuidedEnd = FMath::Lerp(1.0f, GuidedEnd, ReleaseAlpha);
	return RopeAlpha <= CurrentGuidedEnd + KINDA_SMALL_NUMBER ? 1.0f : 0.0f;
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

	// An ordinary whip starts as one continuous straight guide, including the future solver-owned tail.
	// This discards the old hanging or wrapped pose once, before the tail is released gradually by Advance.
	// An aim hit keeps its separate endpoint-envelope contract and blends both ends with the solver.
	const int32 LastGuidedNode = LastNode;
	TArray<FVector> GuideTargets;
	BuildGuideTargets(0.0f, LastGuidedNode, Sim, Config, GuideTargets);
	PreviousTargets = GuideTargets;
	PrevTargetsThisFrame = GuideTargets;
	CurrentTargetsThisFrame = GuideTargets;
	GuidedNodesThisFrame.SetNumZeroed(Sim.Num());
	for (int32 i = 1; i <= LastGuidedNode; ++i)
	{
		if (!GuideTargets.IsValidIndex(i))
		{
			break;
		}

		// The same envelope places the middle on the initial spline, and the hand end has the socket's current movement added.
		const float S = static_cast<float>(i) / static_cast<float>(LastNode);
		const float GuideWeight = bHasAimTarget ? AimGuideEnvelope(S, Config) : 1.0f;
		const float SocketInfluence = bHasAimTarget ? AimRootSocketInfluence(S, Config) : 0.0f;
		const FVector SocketOffset = Sim.bStartPinned ? Sim.StartPinTarget - Origin : FVector::ZeroVector;
		const FVector Target = GuideTargets[i] + SocketOffset * SocketInfluence;
		const FVector Position = FMath::Lerp(Sim.Positions[i], Target, GuideWeight);
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
	const int32 LastNode = Sim.Num() - 1;
	// A value of one intentionally keeps the whole rope hard-guided until the flight guide ends.
	// Do not silently reduce it: doing so creates a moving solver/guide boundary in the tail.
	const float GuidedEnd = bHasAimTarget ? 1.0f : FMath::Clamp(Config.GuidedLength, 0.05f, 1.0f);
	// The ordinary guide builds a target for the whole rope while its initial straight seed is being
	// released. OrdinaryGuideOwnership moves one hard ownership boundary towards GuidedEnd.
	const int32 LastGuidedNode = LastNode;
	TArray<FVector> GuideTargets;
	BuildGuideTargets(T, LastGuidedNode, Sim, Config, GuideTargets);
	if (GuideTargets.Num() == 0)
	{
		bActive = false;
		return;
	}

	PrevTargetsThisFrame = Sim.PrevPositions;
	CurrentTargetsThisFrame = Sim.Positions;
	GuidedNodesThisFrame.SetNumZeroed(Sim.Num());
	// The current and previous pin offsets are applied separately, which preserves the hand socket's movement speed in the Verlet state as well.
	const FVector CurrentSocketOffset = Sim.bStartPinned ? Sim.StartPinTarget - Origin : FVector::ZeroVector;
	const FVector PreviousSocketOffset = Sim.bStartPinned ? Sim.StartPinPrev - Origin : FVector::ZeroVector;

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

		// Aim-hit owns the middle through its endpoint envelope. An ordinary whip begins fully straight
		// and moves a hard ownership boundary towards its configured leading span. Hard ownership keeps
		// every still-guided node on the same straight line.
		const float GuideWeight = bHasAimTarget
			? AimGuideEnvelope(S, Config)
			: OrdinaryGuideOwnership(S, T, GuidedEnd);
		if (GuideWeight <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		if (bHasAimTarget)
		{
			const float SocketInfluence = AimRootSocketInfluence(S, Config);
			const FVector CurrentGuide = GuideTargets[i] + CurrentSocketOffset * SocketInfluence;
			const FVector PreviousGuide = PreviousTargets.IsValidIndex(i)
				? PreviousTargets[i] + PreviousSocketOffset * SocketInfluence
				: CurrentGuide;
			// The middle is pinned to the spline, and towards both ends the current solver state is increasingly preserved, which hands over smoothly.
			CurrentTargetsThisFrame[i] = FMath::Lerp(Sim.Positions[i], CurrentGuide, GuideWeight);
			PrevTargetsThisFrame[i] = FMath::Lerp(Sim.PrevPositions[i], PreviousGuide, GuideWeight);
		}
		else
		{
			const FVector PreviousGuide = PreviousTargets.IsValidIndex(i)
				? PreviousTargets[i]
				: GuideTargets[i];
			CurrentTargetsThisFrame[i] = GuideTargets[i];
			PrevTargetsThisFrame[i] = PreviousGuide;
		}

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
	const float GuidedEnd = bHasAimTarget ? 1.0f : FMath::Clamp(Config.GuidedLength, 0.05f, 1.0f);
	const int32 LastGuidedNode = LastNode;
	const float Duration = ResolveGuideDuration(Config, GuideThrowSpeed);
	const float NextT = FMath::Clamp((Elapsed + DeltaTime) / Duration, 0.0f, 1.0f);
	TArray<FVector> GuideTargets;
	BuildGuideTargets(NextT, LastGuidedNode, Sim, Config, GuideTargets);
	if (GuideTargets.Num() == 0)
	{
		return;
	}

	OutTargets = Sim.Positions;
	TArray<uint8> PreviewGuidedMask;
	PreviewGuidedMask.SetNumZeroed(Sim.Num());
	const FVector SocketOffset = Sim.bStartPinned ? Sim.StartPinTarget - Origin : FVector::ZeroVector;
	for (int32 i = 1; i <= LastGuidedNode; ++i)
	{
		if (!GuideTargets.IsValidIndex(i) ||
			(Sim.InvMass.IsValidIndex(i) && Sim.InvMass[i] <= 0.0f))
		{
			continue;
		}

		const float S = static_cast<float>(i) / static_cast<float>(LastNode);
		const float GuideWeight = bHasAimTarget
			? AimGuideEnvelope(S, Config)
			: OrdinaryGuideOwnership(S, NextT, GuidedEnd);
		if (GuideWeight <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		if (bHasAimTarget)
		{
			const FVector Target = GuideTargets[i] +
				SocketOffset * AimRootSocketInfluence(S, Config);
			OutTargets[i] = FMath::Lerp(Sim.Positions[i], Target, GuideWeight);
		}
		else
		{
			OutTargets[i] = GuideTargets[i];
		}
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

	const FVector HandPos = Origin;
	const float GuidedEnd = bHasAimTarget ? 1.0f : FMath::Clamp(Config.GuidedLength, 0.05f, 1.0f);
	const int32 DesiredPointCount = FMath::Clamp(LastGuidedNode + 1, 1, Sim.Num());
	const int32 RawSampleCount = FMath::Max(DesiredPointCount * 4, 16);
	const float GuideLength = FMath::Max(Sim.RopeLength, Config.ComponentRopeLength) * GuidedEnd;
	const float GuideDuration = ResolveGuideDuration(Config, GuideThrowSpeed);
	const FVector InheritedDrift = GuideInheritedVelocity * (GuideDuration * T);
	const float SweepRadians = FMath::DegreesToRadians(FMath::Clamp(Config.SweepAngleDegrees, 1.0f, 180.0f));
	const float AngleFromAim = SweepRadians * (1.0f - T);
	FVector SweepDir = (Forward * FMath::Cos(AngleFromAim) + Up * FMath::Sin(AngleFromAim)).GetSafeNormal();
	if (T <= KINDA_SMALL_NUMBER)
	{
		SweepDir = (Forward * FMath::Cos(SweepRadians) + Up * FMath::Sin(SweepRadians)).GetSafeNormal();
	}
	else if (T >= 1.0f - KINDA_SMALL_NUMBER)
	{
		SweepDir = Forward;
	}
	// No node is pinned to the hit point itself; only the normalized direction from the throw origin to the hit point is passed to the spline.
	const FVector LockedAimDir = bHasAimTarget
		? (AimTarget - HandPos).GetSafeNormal(KINDA_SMALL_NUMBER, Forward)
		: Forward;

	TArray<FVector> RawPoints;
	RopeMath::BuildWhipGuideRawPoints(HandPos, SweepDir, LockedAimDir, bHasAimTarget, T,
		GuideLength, InheritedDrift, AimSteerStartAlpha, AimLockAlpha,
		Config.AimHitDirectionBias, RawSampleCount, RawPoints);

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
