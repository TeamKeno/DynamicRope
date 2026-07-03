// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWhipGuide.h"
#include "RopeMathHelpers.h" // RopeMath::SmoothStep (unity 빌드 중복 정의 방지)

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
	const FVector& FallbackAim, const FVector& FallbackUp, const FVector& FallbackSide)
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

	Elapsed = 0.0f;
	bActive = true;

	// 새 스윙 시작: 직전 스윙의 프레임 산출물이 이번 throw로 새어들지 않게 비운다.
	// (디버그 배열은 원 코드와 동일하게 다음 Advance가 리셋한다.)
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

	const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * Config.GuidedLength), 1, LastNode);
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

		Sim.Positions[i] = GuideTargets[i];
		Sim.PrevPositions[i] = GuideTargets[i];
		if (GuidedNodesThisFrame.IsValidIndex(i))
		{
			GuidedNodesThisFrame[i] = 1;
		}
	}
}

void FRopeWhipGuide::Advance(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config, bool bCaptureDebugTargets)
{
	ResetFrameOutputs();

	if (Sim.Num() < 3)
	{
		bActive = false;
		return;
	}

	Elapsed += DeltaTime;
	const float Duration = FMath::Max(Config.Duration, KINDA_SMALL_NUMBER);
	const float T = FMath::Clamp(Elapsed / Duration, 0.0f, 1.0f);
	const int32 LastNode = Sim.Num() - 1;
	const float GuidedEnd = FMath::Clamp(Config.GuidedLength, 0.05f, 0.95f);
	const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * GuidedEnd), 1, LastNode);
	TArray<FVector> GuideTargets;
	BuildGuideTargets(T, LastGuidedNode, Sim, Config, GuideTargets);
	if (GuideTargets.Num() == 0)
	{
		bActive = false;
		return;
	}

	PrevTargetsThisFrame = PreviousTargets;
	CurrentTargetsThisFrame = GuideTargets;
	GuidedNodesThisFrame.SetNumZeroed(Sim.Num());

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

		// 뿌리 쪽(손 근처)과 가이드 구간 끝은 서서히 가이드에서 놓아준다 — 참여 여부 게이트로만 쓴다.
		const float StrongGuideEnd = GuidedEnd * 0.55f;
		const float GuideFade = (S <= StrongGuideEnd)
			? 1.0f
			: 1.0f - RopeMath::SmoothStep((S - StrongGuideEnd) / FMath::Max(GuidedEnd - StrongGuideEnd, KINDA_SMALL_NUMBER));
		const float RootFade = RopeMath::SmoothStep(S / FMath::Max(StrongGuideEnd, KINDA_SMALL_NUMBER));
		const float GuideWeight = GuideFade * FMath::Lerp(0.65f, 1.0f, RootFade);
		if (GuideWeight <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		if (GuidedNodesThisFrame.IsValidIndex(i))
		{
			GuidedNodesThisFrame[i] = 1;
		}

		if (bCaptureDebugTargets)
		{
			DebugGuideNodeIndices.Add(i);
			DebugGuideTargets.Add(GuideTargets[i]);
		}
	}

	PreviousTargets = GuideTargets;
	bActive = Elapsed < Config.Duration;
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
	const float GuidedEnd = FMath::Clamp(Config.GuidedLength, 0.05f, 0.95f);
	const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * GuidedEnd), 1, LastNode);
	const float Duration = FMath::Max(Config.Duration, KINDA_SMALL_NUMBER);
	const float NextT = FMath::Clamp((Elapsed + DeltaTime) / Duration, 0.0f, 1.0f);
	BuildGuideTargets(NextT, LastGuidedNode, Sim, Config, OutTargets);
}

void FRopeWhipGuide::ResetFrameOutputs()
{
	DebugGuideNodeIndices.Reset();
	DebugGuideTargets.Reset();
	PrevTargetsThisFrame.Reset();
	CurrentTargetsThisFrame.Reset();
	GuidedNodesThisFrame.Reset();
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
	const float GuidedEnd = FMath::Clamp(Config.GuidedLength, 0.05f, 0.95f);
	const int32 DesiredPointCount = FMath::Clamp(LastGuidedNode + 1, 1, Sim.Num());
	const int32 RawSampleCount = FMath::Max(DesiredPointCount * 4, 16);
	const float GuideLength = FMath::Max(Sim.RopeLength, Config.ComponentRopeLength) * GuidedEnd;
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

	TArray<FVector> RawPoints;
	RawPoints.Reserve(RawSampleCount);
	for (int32 SampleIdx = 0; SampleIdx < RawSampleCount; ++SampleIdx)
	{
		const float RawAlpha = (RawSampleCount > 1)
			? static_cast<float>(SampleIdx) / static_cast<float>(RawSampleCount - 1)
			: 0.0f;
		RawPoints.Add(HandPos + SweepDir * (RawAlpha * GuideLength));
	}

	ResampleGuideByNodeSpacing(RawPoints, Sim.RopeLength, Sim.Num(), DesiredPointCount, Sim.SegmentLength, OutTargets);
}

void FRopeWhipGuide::ResampleGuideByNodeSpacing(const TArray<FVector>& SourcePoints, float TotalLength, int32 NodeCount,
	int32 DesiredPointCount, float FallbackSegmentLength, TArray<FVector>& OutPoints) const
{
	OutPoints.Reset();
	if (SourcePoints.Num() == 0 || NodeCount < 2 || DesiredPointCount <= 0)
	{
		return;
	}

	const float SegmentLength = TotalLength > KINDA_SMALL_NUMBER
		? TotalLength / static_cast<float>(NodeCount - 1)
		: FallbackSegmentLength;
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
