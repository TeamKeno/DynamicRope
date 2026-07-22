// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWhipGuide.h"
// RopeMath::SmoothStep (unity 빌드 중복 정의 방지)
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

// 로프 중앙은 spline 가중치 1, 손/자유단은 0으로 완화해 양끝을 solver에 돌려준다.
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

// throw origin과 현재 손 소켓의 차이는 뿌리 구간에만 섞어 애니메이션을 자연스럽게 따라간다.
float AimRootSocketInfluence(float RopeAlpha, const FRopeWhipGuide::FConfig& Config)
{
	const float RootRange = FMath::Clamp(Config.AimHitRootSolverFraction, 0.0f, 0.45f);
	return RootRange > KINDA_SMALL_NUMBER
		? 1.0f - RopeMath::SmoothStep(RopeAlpha / RootRange)
		: 0.0f;
}

// Raw spline은 SegmentLength로 리샘플되지만, 그 뒤 노드별 aim envelope/solver blend가 간격을
// 다시 늘릴 수 있다. 가이드된 run을 root 쪽부터 순차 투영해 각 target edge의 최대 길이만 제한한다.
// 자유단/solver-owned 노드는 수정하지 않으므로 endpoint envelope 계약과 실제 solver 운동은 유지된다.
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
	// Aim hit은 좌표 고정점이 아니라 origin에서 hit으로 향하는 최종 방향과 보간 구간으로만 저장한다.
	bHasAimTarget = bInHasAimTarget && !(InAimTarget - InOrigin).IsNearlyZero();
	AimTarget = InAimTarget;
	AimSteerStartAlpha = FMath::Clamp(InAimSteerStartAlpha, 0.0f, 0.9f);
	AimLockAlpha = FMath::Clamp(FMath::Max(InAimLockAlpha, AimSteerStartAlpha + 0.01f), 0.05f, 1.0f);

	Elapsed = 0.0f;
	bActive = true;

	// 새 스윙 시작: 직전 스윙의 프레임 산출물이 이번 throw로 새어들지 않게 비운다.
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

	// Aim hit은 중앙만 spline에 강하게 스냅하고 양끝은 solver가 자연스럽게 이어받는다.
	const float GuidedEnd = bHasAimTarget ? 1.0f : FMath::Clamp(Config.GuidedLength, 0.05f, 0.95f);
	const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * GuidedEnd), 1, LastNode);
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

		// 같은 envelope로 중앙만 초기 spline에 배치하고, 손 쪽에는 현재 소켓 이동량을 더한다.
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

	// 최종 envelope blend 뒤 가이드 target만 다시 비신축으로 만든다. 자유단은 기존 solver pose를 유지한다.
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
	const float GuidedEnd = bHasAimTarget ? 1.0f : FMath::Clamp(Config.GuidedLength, 0.05f, 0.95f);
	const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * GuidedEnd), 1, LastNode);
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
	// 현재/이전 pin 오프셋을 각각 적용해 손 소켓의 이동 속도까지 Verlet 상태에 보존한다.
	const FVector CurrentSocketOffset = Sim.bStartPinned ? Sim.StartPinTarget - Origin : FVector::ZeroVector;
	const FVector PreviousSocketOffset = Sim.bStartPinned ? Sim.StartPinPrev - Origin : FVector::ZeroVector;

	// 마스크/디버그 계산만 — 실제 기록은 CPU 경로의 ApplyToSim 또는 GPU override 패스가
	// 같은 산출물(CurrentTargets/PrevTargets/mask)을 소비해서 수행한다.
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

		// 뿌리 쪽(손 근처)과 가이드 구간 끝은 서서히 가이드에서 놓아준다.
		const float StrongGuideEnd = GuidedEnd * 0.55f;
		const float GuideFade = (S <= StrongGuideEnd)
			? 1.0f
			: 1.0f - RopeMath::SmoothStep((S - StrongGuideEnd) / FMath::Max(GuidedEnd - StrongGuideEnd, KINDA_SMALL_NUMBER));
		const float RootFade = RopeMath::SmoothStep(S / FMath::Max(StrongGuideEnd, KINDA_SMALL_NUMBER));
		const float GuideWeight = bHasAimTarget
			? AimGuideEnvelope(S, Config)
			: GuideFade * FMath::Lerp(0.65f, 1.0f, RootFade);
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
			// 중앙은 spline에 고정하고, 양끝으로 갈수록 현재 solver 상태를 보존해 부드럽게 넘긴다.
			CurrentTargetsThisFrame[i] = FMath::Lerp(Sim.Positions[i], CurrentGuide, GuideWeight);
			PrevTargetsThisFrame[i] = FMath::Lerp(Sim.PrevPositions[i], PreviousGuide, GuideWeight);
		}
		else
		{
			CurrentTargetsThisFrame[i] = GuideTargets[i];
			PrevTargetsThisFrame[i] = PreviousTargets.IsValidIndex(i)
				? PreviousTargets[i]
				: Sim.PrevPositions[i];
		}

		if (GuidedNodesThisFrame.IsValidIndex(i))
		{
			GuidedNodesThisFrame[i] = 1;
		}

	}

	// ResampleGuideByNodeSpacing 뒤의 endpoint envelope/socket blend가 spacing을 다시 깨뜨리지
	// 않게 Current/Prev target run을 각각 현재/직전 pin에서 순차 투영한다.
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
	const float GuidedEnd = bHasAimTarget ? 1.0f : FMath::Clamp(Config.GuidedLength, 0.05f, 0.95f);
	const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * GuidedEnd), 1, LastNode);
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
		const float StrongGuideEnd = GuidedEnd * 0.55f;
		const float GuideFade = (S <= StrongGuideEnd)
			? 1.0f
			: 1.0f - RopeMath::SmoothStep((S - StrongGuideEnd) /
				FMath::Max(GuidedEnd - StrongGuideEnd, KINDA_SMALL_NUMBER));
		const float RootFade = RopeMath::SmoothStep(S /
			FMath::Max(StrongGuideEnd, KINDA_SMALL_NUMBER));
		const float GuideWeight = bHasAimTarget
			? AimGuideEnvelope(S, Config)
			: GuideFade * FMath::Lerp(0.65f, 1.0f, RootFade);
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
	const float GuidedEnd = bHasAimTarget ? 1.0f : FMath::Clamp(Config.GuidedLength, 0.05f, 0.95f);
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
	// HitPoint 자체에 노드를 고정하지 않고 (HitPoint - throw Origin)의 정규화 방향만 spline에 전달한다.
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
