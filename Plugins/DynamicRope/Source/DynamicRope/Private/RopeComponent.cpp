// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
#include "Render/RopeSceneProxy.h"
#include "Debug/RopeDebugDraw.h"       // stat 카운터(RopeDebug::Record*)
#include "Debug/RopeDebugSnapshot.h"   // 게이트플레이 디버거용 한 프레임 디버그 스냅샷
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "ProfilingDebugging/CpuProfilerTrace.h" // TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "Subsystem/RopeSimSubsystem.h"
#include "Subsystem/RopeDebugSubsystem.h" // 디버그 캡처 게이트 + 스냅샷 보관소
#include "Settings/DynamicRopeSettings.h"
namespace {
	float SmoothStep(float T)
	{
		T = FMath::Clamp(T, 0.0f, 1.0f);
		return T * T * (3.0f - 2.0f * T);
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

	FVector AnyTangentFromNormal(const FVector& Normal)
	{
		const FVector N = Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		const FVector Reference = FMath::Abs(FVector::DotProduct(N, FVector::UpVector)) < 0.9f
			? FVector::UpVector
			: FVector::RightVector;
		return FVector::CrossProduct(Reference, N).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	}

	// phase 전이 로그용 짧은 이름(UEnum 리플렉션 없이 hot-path에서도 안전).
	const TCHAR* PhaseName(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Free:       return TEXT("Free");
		case ERopePhase::Flight:     return TEXT("Flight");
		case ERopePhase::Contacting: return TEXT("Contacting");
		case ERopePhase::Wrapping: return TEXT("Wrapping");
		case ERopePhase::Wrapped:    return TEXT("Wrapped");
		case ERopePhase::Releasing:  return TEXT("Releasing");
		default:                     return TEXT("?");
		}
	}

}
URopeComponent::URopeComponent()
{
	// URopeSimSubsystem drives SimulateFrame() so all ropes share one orchestration point.
	PrimaryComponentTick.bCanEverTick = false;

	// primitive가 motion vector를 출력하도록 Movable로 설정한다(TAA/TSR가 움직이는 rope를 유지하게 한다).
	Mobility = EComponentMobility::Movable;
}

void URopeComponent::InitRope()
{
	const int32 N = FMath::Max(2, NumParticles);
	Sim.Positions.SetNum(N);
	Sim.PrevPositions.SetNum(N);
	Sim.InvMass.SetNum(N);
	Sim.RopeLength = RopeLength;
	Sim.SegmentLength = RopeLength / static_cast<float>(N - 1);

	const FVector Start = GetComponentLocation();
	const FVector End = Start + GetForwardVector() * RopeLength;
	for (int32 i = 0; i < N; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(N - 1);
		Sim.Positions[i] = FMath::Lerp(Start, End, Alpha);
		Sim.PrevPositions[i] = Sim.Positions[i];
		Sim.InvMass[i] = 1.0f;
	}

	// 시작점을 컴포넌트(hand/socket)에 pin한다; solver가 substep에 걸쳐 이를 sweep한다.
	Sim.InvMass[0] = 0.0f;
	Sim.bStartPinned = true;
	Sim.StartPinTarget = Start;
	Sim.StartPinPrev = Start;

	++SimGeneration; // Sim 전면 재구성 → GPU 상주 버퍼 재시드(M5).

	UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] InitRope: %d particles, length=%.1f, segment=%.2f"),
		*GetName(), N, Sim.RopeLength, Sim.SegmentLength);
}

void URopeComponent::OnRegister()
{
	Super::OnRegister();
	// 에디터에서도 Sim에 기본 직선 포즈를 채워 둔다(서브시스템 틱은 PIE에서만 돌기 때문).
	// 이미 채워져 있으면(InitRope 후/PIE 진행 중) 그대로 둔다.
	EnsureRopeInitialized();
}

void URopeComponent::CreateRenderState_Concurrent(FRegisterComponentContext* Context)
{
	Super::CreateRenderState_Concurrent(Context);
	// 프록시가 막 생성됐다. 틱이 없는 에디터/스폰 직후에도 한 번은 센터라인을 밀어 BuildTube가 돌게 한다
	// (그래야 bHasData=true가 되어 정적 드로우가 유효 지오메트리를 그린다). SendRenderDynamicData_Concurrent는
	// SceneProxy/Sim 유효성을 자체 검사하고 렌더 커맨드만 enqueue하므로 이 시점 호출이 안전하다.
	SendRenderDynamicData_Concurrent();
}

#if WITH_EDITOR
void URopeComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	// NumParticles/RopeLength가 바뀌면 프록시는 새 토폴로지(NumRings)로 재생성되지만 Sim은 옛 개수라
	// BuildTube가 Points.Num()!=NumRings로 건너뛰어 미리보기가 사라진다. EnsureRopeInitialized는 비어있을
	// 때만 init하므로, 여기선 Sim을 새 값으로 강제 재구성해 토폴로지를 맞춘다. 이후 Super가 렌더 상태를
	// 재생성하며 CreateRenderState_Concurrent에서 센터라인을 다시 푸시한다.
	const FName PropertyName = PropertyChangedEvent.GetPropertyName();
	if (PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, NumParticles) ||
		PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, RopeLength))
	{
		InitRope();
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

USkeletalMeshComponent* URopeComponent::ResolveWrapTargetMesh()
{
	if (!WrapTargetMesh)
	{
		if (AActor* Owner = GetOwner())
		{
			WrapTargetMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}
	}
	return WrapTargetMesh;
}

void URopeComponent::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->RegisterRope(this);
	}
	else
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] BeginPlay: RopeSimSubsystem unavailable — rope will not be simulated."),
			*GetName());
	}
}

void URopeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->UnregisterRope(this);
	}
	Super::EndPlay(EndPlayReason);
}

void URopeComponent::StartFreshThrow(const FVector& AimDir)
{
	WhipAimDir = AimDir.GetSafeNormal();
	if (WhipAimDir.IsNearlyZero())
	{
		WhipAimDir = GetForwardVector();
	}
	WhipGuideOrigin = GetComponentLocation();
	WhipGuideForward = WhipAimDir;
	WhipGuideUp = FVector::UpVector;
	if (FMath::Abs(FVector::DotProduct(WhipGuideForward, WhipGuideUp)) > 0.96f)
	{
		WhipGuideUp = GetUpVector().GetSafeNormal();
	}
	FVector WhipGuideSide = FVector::CrossProduct(WhipGuideUp, WhipGuideForward).GetSafeNormal();
	if (WhipGuideSide.IsNearlyZero())
	{
		WhipGuideSide = GetRightVector().GetSafeNormal();
	}
	WhipGuideUp = FVector::CrossProduct(WhipGuideForward, WhipGuideSide).GetSafeNormal();

	WhipElapsed = 0.0f;
	bWhipSwingActive = true;

	++SimGeneration; // throw로 tail 위치를 재설정 → GPU 상주 버퍼 재시드(M5).

	if (WrapController.IsActive())
	{
		WrapController.Release(ERopeReleaseReason::Manual);
	}
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	WrappingState.Reset();
	ContactingElapsed = 0.0f;
	FlightNoContactElapsed = 0.0f;
	ReleaseCooldown = 0.0f;
	WrappedSwayImpulse = FVector::ZeroVector;
	WrappedSwayTime = 0.0f;
	WhipGuidePrevTargetsThisFrame.Reset();
	WhipGuideCurrentTargetsThisFrame.Reset();
	WhipGuidedNodesThisFrame.Reset();

	const int32 LastNode = Sim.Num() - 1;
	if (LastNode >= 1)
	{
		const FVector Start = GetComponentLocation();
		Sim.bStartPinned = true;
		Sim.StartPinPrev = Start;
		Sim.StartPinTarget = Start;
		Sim.Positions[0] = Start;
		Sim.PrevPositions[0] = Start;

		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.InvMass[i] = (i == 0) ? 0.0f : 1.0f;
			Sim.PrevPositions[i] = Sim.Positions[i];
		}

		const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * WhipGuidedLength), 1, LastNode);
		TArray<FVector> GuideTargets;
		BuildWhipGuideTargets(0.0f, LastGuidedNode, GuideTargets);
		PreviousWhipGuideTargets = GuideTargets;
		WhipGuidePrevTargetsThisFrame = GuideTargets;
		WhipGuideCurrentTargetsThisFrame = GuideTargets;
		WhipGuidedNodesThisFrame.SetNumZeroed(Sim.Num());
		for (int32 i = 1; i <= LastGuidedNode; ++i)
		{
			if (!GuideTargets.IsValidIndex(i))
			{
				break;
			}

			Sim.Positions[i] = GuideTargets[i];
			Sim.PrevPositions[i] = GuideTargets[i];
			if (WhipGuidedNodesThisFrame.IsValidIndex(i))
			{
				WhipGuidedNodesThisFrame[i] = 1;
			}
		}

		// Temporary throw: inject Verlet velocity by moving previous positions opposite the aim.
		const float ReferenceDt = 1.0f / 60.0f;
		const float BaseImpulse = ThrowParams.ThrowSpeed * ReferenceDt;
		const float TipBoost = FMath::Clamp(ThrowParams.TipMass / 5.0f, 0.25f, 3.0f);
		const int32 FirstTailNode = FMath::Clamp(FMath::FloorToInt(static_cast<float>(LastNode) * WhipGuidedLength), 1, LastNode);
		for (int32 i = 1; i <= LastNode; ++i)
		{
			const float AlongRope = static_cast<float>(i) / static_cast<float>(LastNode);
			const float TailWeight = TailWeightByIndex(i, FirstTailNode, LastNode);
			const float Weight = FMath::Lerp(SmoothStep(AlongRope), 1.0f, TailWeight * 0.5f);
			const float Impulse = BaseImpulse * Weight * FMath::Lerp(1.0f, TipBoost, TailWeight);
			Sim.PrevPositions[i] -= WhipAimDir * Impulse;
		}
	}

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] %s -> Flight (fresh throw impulse, aim=%s, speed=%.1f)"),
		*GetName(), PhaseName(Phase), *WhipAimDir.ToCompactString(), ThrowParams.ThrowSpeed);
	Phase = ERopePhase::Flight;
}

void URopeComponent::BuildWhipGuideTargets(float NormalizedTime, int32 LastGuidedNode, TArray<FVector>& OutTargets) const
{
	OutTargets.Reset();
	if (Sim.Num() < 2 || LastGuidedNode < 0)
	{
		return;
	}

	const float T = FMath::Clamp(NormalizedTime, 0.0f, 1.0f);
	const FVector Forward = WhipGuideForward.GetSafeNormal();
	if (Forward.IsNearlyZero())
	{
		return;
	}

	FVector Up = WhipGuideUp.GetSafeNormal();
	if (Up.IsNearlyZero())
	{
		Up = FVector::UpVector;
	}

	const FVector HandPos = WhipGuideOrigin;
	const float GuidedEnd = FMath::Clamp(WhipGuidedLength, 0.05f, 0.95f);
	const int32 DesiredPointCount = FMath::Clamp(LastGuidedNode + 1, 1, Sim.Num());
	const int32 RawSampleCount = FMath::Max(DesiredPointCount * 4, 16);
	const float GuideLength = FMath::Max(Sim.RopeLength, RopeLength) * GuidedEnd;
	const float SweepRadians = FMath::DegreesToRadians(FMath::Clamp(WhipSweepAngleDegrees, 1.0f, 180.0f));
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

	ResampleGuideByNodeSpacing(RawPoints, Sim.RopeLength, Sim.Num(), DesiredPointCount, OutTargets);
}

void URopeComponent::ResampleGuideByNodeSpacing(const TArray<FVector>& SourcePoints, float TotalLength, int32 NodeCount,
	int32 DesiredPointCount, TArray<FVector>& OutPoints) const
{
	OutPoints.Reset();
	if (SourcePoints.Num() == 0 || NodeCount < 2 || DesiredPointCount <= 0)
	{
		return;
	}

	const float SegmentLength = TotalLength > KINDA_SMALL_NUMBER
		? TotalLength / static_cast<float>(NodeCount - 1)
		: Sim.SegmentLength;
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
			: WhipAimDir.GetSafeNormal();
		OutPoints[PointIdx] = SourcePoints.Last() + TailDir * (TargetDistance - Accumulated.Last());
	}
}

void URopeComponent::ApplyWhipSwing(float DeltaTime)
{
	DebugWhipGuideNodeIndices.Reset();
	DebugWhipGuideTargets.Reset();
	WhipGuidePrevTargetsThisFrame.Reset();
	WhipGuideCurrentTargetsThisFrame.Reset();
	WhipGuidedNodesThisFrame.Reset();

	if (Sim.Num() < 3)
	{
		bWhipSwingActive = false;
		return;
	}

	WhipElapsed += DeltaTime;
	const float Duration = FMath::Max(WhipDuration, KINDA_SMALL_NUMBER);
	const float T = FMath::Clamp(WhipElapsed / Duration, 0.0f, 1.0f);
	const int32 LastNode = Sim.Num() - 1;
	const float GuidedEnd = FMath::Clamp(WhipGuidedLength, 0.05f, 0.95f);
	// whip 가이드 타깃 캡처: 이 로프가 디버거 대상이거나(시각화) stat 수집 중일 때만(비용 절약).
#if WITH_GAMEPLAY_DEBUGGER
	const URopeDebugSubsystem* DebugSub = URopeDebugSubsystem::Get(GetWorld());
	const bool bCaptureGuideTargets = (DebugSub && DebugSub->ShouldCapture(this)) || RopeDebug::IsFlightStatEnabled();
#else
	const bool bCaptureGuideTargets = RopeDebug::IsFlightStatEnabled();
#endif
	const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * GuidedEnd), 1, LastNode);
	TArray<FVector> GuideTargets;
	BuildWhipGuideTargets(T, LastGuidedNode, GuideTargets);
	if (GuideTargets.Num() == 0)
	{
		bWhipSwingActive = false;
		return;
	}

	WhipGuidePrevTargetsThisFrame = PreviousWhipGuideTargets;
	WhipGuideCurrentTargetsThisFrame = GuideTargets;
	WhipGuidedNodesThisFrame.SetNumZeroed(Sim.Num());

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

		const float StrongGuideEnd = GuidedEnd * 0.55f;
		const float GuideFade = (S <= StrongGuideEnd)
			? 1.0f
			: 1.0f - SmoothStep((S - StrongGuideEnd) / FMath::Max(GuidedEnd - StrongGuideEnd, KINDA_SMALL_NUMBER));
		const float RootFade = SmoothStep(S / FMath::Max(StrongGuideEnd, KINDA_SMALL_NUMBER));
		const float GuideWeight = GuideFade * FMath::Lerp(0.65f, 1.0f, RootFade);
		if (GuideWeight <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const FVector Target = GuideTargets[i];

		Sim.PrevPositions[i] = PreviousWhipGuideTargets.IsValidIndex(i) ? PreviousWhipGuideTargets[i] : Sim.Positions[i];
		Sim.Positions[i] = Target;
		if (WhipGuidedNodesThisFrame.IsValidIndex(i))
		{
			WhipGuidedNodesThisFrame[i] = 1;
		}

		if (bCaptureGuideTargets)
		{
			DebugWhipGuideNodeIndices.Add(i);
			DebugWhipGuideTargets.Add(Target);
		}
	}

	PreviousWhipGuideTargets = GuideTargets;
	bWhipSwingActive = WhipElapsed < WhipDuration;
}

void URopeComponent::DetectContactCandidates(const TArray<FVector>& PrevPositions, const TArray<FVector>& Positions,
	const TArray<IRopeCollider*>& Colliders, TArray<FRopeContactCandidate>& OutCandidates) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightDetectContactCandidates);
	TArray<IRopeCollider*> NearbyColliders;
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!PrevPositions.IsValidIndex(i) || !Positions.IsValidIndex(i))
		{
			continue;
		}

		bool bFast = IsTailNode(i) || NodeSpeed(i) > Sim.SegmentLength;
		GatherNearbyColliders(PrevPositions[i], Positions[i], Colliders, NearbyColliders);
		bool bNearBody = NearbyColliders.Num() > 0;

		if (!bFast && !bNearBody)
			continue;
		if (!bNearBody)
			continue;

		// 현재 위치만 보지 않고 이동 경로를 본다.
		// 캡슐은 segment-vs-capsule로 가능.
		// SDF는 path를 몇 개 샘플링하거나 SweepQuery adapter가 필요.
		FRopeContact Contact = SweepOrSampleContact(PrevPositions[i], Positions[i], NearbyColliders);

		if (Contact.bHit)
		{
			FRopeContactCandidate Candidate = MakeCandidate(i, Contact);
			Candidate.Source = ERopeContactCandidateSource::Actual;
			Candidate.SourceMask = static_cast<uint8>(Candidate.Source);
			OutCandidates.Add(Candidate);
		}
	}
}

void URopeComponent::AddPredictedContactCandidates(TArray<FRopeContactCandidate>& InOutCandidates, float DeltaTime) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightAddPredictedContactCandidates);
	const float PredictionFrames = FMath::Max(0.0f, WrapConfig.PredictiveContactFrames);
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

	TArray<FVector> NextGuideTargets;
	TArray<IRopeCollider*> NearbyColliders;
	const bool bHasGuidedNodes = Phase == ERopePhase::Flight && WhipGuidedNodesThisFrame.Num() > 0;
	if (bHasGuidedNodes)
	{
		const int32 LastNode = Sim.Num() - 1;
		const float GuidedEnd = FMath::Clamp(WhipGuidedLength, 0.05f, 0.95f);
		const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * GuidedEnd), 1, LastNode);
		const float Duration = FMath::Max(WhipDuration, KINDA_SMALL_NUMBER);
		const float NextT = FMath::Clamp((WhipElapsed + DeltaTime) / Duration, 0.0f, 1.0f);
		BuildWhipGuideTargets(NextT, LastGuidedNode, NextGuideTargets);
	}

	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!Sim.Positions.IsValidIndex(i) || !Sim.PrevPositions.IsValidIndex(i))
		{
			continue;
		}

		FVector CurrentPosition = Sim.Positions[i];
		FVector PredictedPosition = CurrentPosition;
		const FVector FrameDisplacement = Sim.Positions[i] - Sim.PrevPositions[i];
		if (!ShouldRunPredictiveContactForNode(i, bHasGuidedNodes, FrameDisplacement))
		{
			continue;
		}

		const bool bGuidedNode = bHasGuidedNodes && IsWhipGuidedNodeThisFrame(i);
		const bool bTailNode = IsTailNode(i);
		const bool bFastNode = FrameDisplacement.Size() > Sim.SegmentLength;

		bool bFastEnoughForPrediction = false;
		ERopeContactCandidateSource Source = ERopeContactCandidateSource::PredictiveFree;

		if (bGuidedNode && WhipGuideCurrentTargetsThisFrame.IsValidIndex(i))
		{
			Source = ERopeContactCandidateSource::PredictiveGuided;
			CurrentPosition = WhipGuideCurrentTargetsThisFrame[i];
			if (NextGuideTargets.IsValidIndex(i))
			{
				PredictedPosition = CurrentPosition + (NextGuideTargets[i] - CurrentPosition) * PredictionFrames;
			}
			else
			{
				const FVector PrevGuidePosition = WhipGuidePrevTargetsThisFrame.IsValidIndex(i)
					? WhipGuidePrevTargetsThisFrame[i]
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

		GatherNearbyColliders(CurrentPosition, PredictedPosition, FrameColliders, NearbyColliders);
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
		const FRopeContact Contact = SweepOrSampleContact(CurrentPosition, PredictedPosition, NearbyColliders);
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

void URopeComponent::EvaluateRelativeMotion(TArray<FRopeContactCandidate>& Candidates) const
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
		Candidate.WrapDirectionScore = FVector::DotProduct(TangentVelocity.GetSafeNormal(), ExpectedWrapTangent(Candidate));
	}
}

bool URopeComponent::ShouldCommitWrap(const FRopeContactTracker& Tracker) const
{
	return Tracker.DwellTime >= WrapConfig.WrapDecisionTime
		&& Tracker.CandidateNodes.Num() >= WrapConfig.MinLatchNodes
		&& Tracker.BestWrapScore >= 0.0f
		&& IsWrappableBone(Tracker.CandidateBone);
}

//Contacting을 후보 판정만 하도록
void URopeComponent::UpdateContacting(float DeltaTime)
{
	// 지금 단계에서는 기존 함수를 재사용한다.
	// 나중에 여기서 매 프레임 접촉 후보를 다시 수집하고,
	// tangential speed / winding angle까지 갱신하게 만들면 된다.
	AdvanceWrappingMotion(DeltaTime);

	if (ShouldDismissContacting())
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Contacting -> Flight (contact lost before wrapping)"), *GetName());
		ContactTracker.Reset();
		PendingWrapSeed.Reset();
		ContactingElapsed = 0.0f;
		FlightNoContactElapsed = 0.0f;
		Phase = ERopePhase::Flight;
		return;
	}

	if (ShouldStartWrapping())
	{
		StartWrappingFromContacting();
		return;
	}
}

void URopeComponent::StartWrappingFromContacting()
{
	//PendingWrapSeed를 바로 BeginWrap에 넣지 않고, WrappingState로 변환한다.


	WrappingState.Reset();

	const USkeletalMeshComponent* Mesh = PendingWrapSeed.Mesh.Get();
	if (!Mesh)
	{
		Mesh = ResolveWrapTargetMesh();
	}

	if (!Mesh || PendingWrapSeed.BoneName.IsNone() || PendingWrapSeed.Latched.Num() == 0)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Contacting -> Flight (invalid wrapping seed)"), *GetName());
		ContactTracker.Reset();
		PendingWrapSeed.Reset();
		ContactingElapsed = 0.0f;
		FlightNoContactElapsed = 0.0f;
		Phase = ERopePhase::Flight;
		return;
	}

	WrappingState.BoneName = PendingWrapSeed.BoneName;
	WrappingState.Mesh = Mesh;
	WrappingState.Elapsed = 0.0f;
	WrappingState.Duration = FMath::Max(0.01f, WrapConfig.WrappingMotionDuration);
	WrappingState.FirstNode = TNumericLimits<int32>::Max();
	WrappingState.LastNode = INDEX_NONE;

	const FRopeLatchNode& Latch = PendingWrapSeed.Latched[0];
	FRopeSurfaceAnchor LatchAnchor;
	if (PendingWrapSeed.Anchors.Num() > 0)
	{
		LatchAnchor = PendingWrapSeed.Anchors[0];
		LatchAnchor.Mesh = Mesh;
	}
	else if (Sim.Positions.IsValidIndex(Latch.NodeIndex))
	{
		const FVector NormalWorld = FVector::UpVector;
		FVector TangentWorld = FVector::ForwardVector;
		if (Sim.Positions.IsValidIndex(Latch.NodeIndex + 1))
		{
			TangentWorld = (Sim.Positions[Latch.NodeIndex + 1] - Sim.Positions[Latch.NodeIndex])
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		}

		const FTransform BoneXform = Mesh->GetSocketTransform(Latch.Bone);
		LatchAnchor.NodeIndex = Latch.NodeIndex;
		LatchAnchor.Bone = Latch.Bone;
		LatchAnchor.Mesh = Mesh;
		LatchAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Sim.Positions[Latch.NodeIndex]);
		LatchAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		LatchAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		LatchAnchor.StartWorldPosition = Sim.Positions[Latch.NodeIndex];
		LatchAnchor.SurfaceOffset = FMath::Max(0.0f, Radius);
		LatchAnchor.RopeDistance = 0.0f;
	}

	if (!BuildWrappingAnchorsFromLatch(LatchAnchor))
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Contacting -> Flight (no valid wrapping anchors)"), *GetName());
		WrappingState.Reset();
		ContactTracker.Reset();
		PendingWrapSeed.Reset();
		ContactingElapsed = 0.0f;
		FlightNoContactElapsed = 0.0f;
		Phase = ERopePhase::Flight;
		return;
	}

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] Contacting -> Wrapping (bone=%s, %d anchor(s))"),
		*GetName(), *WrappingState.BoneName.ToString(), WrappingState.Anchors.Num());

	WrappingState.StableTime = 0.0f;
	WrappingState.LostContactTime = 0.0f;
	WrappingState.LastStableFirstNode = WrappingState.FirstNode;
	WrappingState.LastStableLastNode = WrappingState.LastNode;
	WrappingState.LastStableAnchorCount = WrappingState.Anchors.Num();

	Phase = ERopePhase::Wrapping;

}

void URopeComponent::UpdateWrapping(float DeltaTime)
{
	WrappingState.Elapsed += DeltaTime;

	if (!IsWrappingStillValid())
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Wrapping -> Releasing (invalid wrapping state)"), *GetName());
		AbortWrapping(ERopeReleaseReason::Broken);
		Phase = ERopePhase::Releasing;
		return;
	}

	WrappingState.LostContactTime = 0.0f;
	ApplyWrappingTargetMotion(DeltaTime);

	ApplyWrappingMassMask();

	const bool bSameStableSpan =
		WrappingState.FirstNode == WrappingState.LastStableFirstNode &&
		WrappingState.LastNode == WrappingState.LastStableLastNode &&
		WrappingState.Anchors.Num() == WrappingState.LastStableAnchorCount;

	if (bSameStableSpan)
	{
		WrappingState.StableTime += DeltaTime;
	}
	else
	{
		WrappingState.StableTime = 0.0f;
		WrappingState.LastStableFirstNode = WrappingState.FirstNode;
		WrappingState.LastStableLastNode = WrappingState.LastNode;
		WrappingState.LastStableAnchorCount = WrappingState.Anchors.Num();
	}

	const bool bHasEnoughAnchors = WrappingState.Anchors.Num() > 0;
	float MaxTailDelay = 0.0f;
	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);
	for (const FRopeSurfaceAnchor& Anchor : WrappingState.Anchors)
	{
		MaxTailDelay = FMath::Max(MaxTailDelay,
			(Anchor.RopeDistance / SegmentLength) * WrapConfig.WrappingTailDelayPerSegment);
	}
	const bool bMotionDone = WrappingState.Elapsed >= WrappingState.Duration + MaxTailDelay;
	const bool bTimedOutWithAnchors =
		WrapConfig.WrappingMaxSettleTime > 0.0f &&
		WrappingState.Elapsed >= WrapConfig.WrappingMaxSettleTime;

	if (bHasEnoughAnchors && (bMotionDone || bTimedOutWithAnchors))
	{
		CommitWrapping();
		return;
	}
}

bool URopeComponent::IsWrappingStillValid() const
{
	return WrappingState.IsActive()
		&& WrappingState.Mesh.IsValid()
		&& !WrappingState.BoneName.IsNone();
}

bool URopeComponent::BuildWrappingAnchorsFromLatch(const FRopeSurfaceAnchor& LatchAnchor)
{
	const USkeletalMeshComponent* Mesh = WrappingState.Mesh.Get();
	if (!Mesh || !Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	WrappingState.Anchors.Reset();
	WrappingState.FirstNode = TNumericLimits<int32>::Max();
	WrappingState.LastNode = INDEX_NONE;

	const int32 LatchNode = LatchAnchor.NodeIndex;
	const FName Bone = LatchAnchor.Bone;

	for (int32 NodeIndex = LatchNode; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		if (!Sim.Positions.IsValidIndex(NodeIndex))
		{
			continue;
		}

		const float DistanceFromLatch = static_cast<float>(NodeIndex - LatchNode) * Sim.SegmentLength;

		FVector SurfaceWorld = FVector::ZeroVector;
		FVector NormalWorld = FVector::UpVector;
		FVector TangentWorld = FVector::ForwardVector;
		if (!ComputeWrapSurfaceTarget(LatchAnchor, DistanceFromLatch, SurfaceWorld, NormalWorld, TangentWorld))
		{
			continue;
		}

		const FTransform BoneXform = Mesh->GetSocketTransform(Bone);

		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = NodeIndex;
		Anchor.Bone = Bone;
		Anchor.Mesh = Mesh;
		Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(SurfaceWorld);
		Anchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		Anchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		Anchor.StartWorldPosition = Sim.Positions[NodeIndex];
		Anchor.SurfaceOffset = FMath::Max(0.0f, Radius);
		Anchor.RopeDistance = DistanceFromLatch;

		WrappingState.FirstNode = FMath::Min(WrappingState.FirstNode, NodeIndex);
		WrappingState.LastNode = FMath::Max(WrappingState.LastNode, NodeIndex);
		WrappingState.Anchors.Add(Anchor);
	}

	if (WrappingState.Anchors.Num() == 0)
	{
		WrappingState.FirstNode = INDEX_NONE;
		WrappingState.LastNode = INDEX_NONE;
		return false;
	}

	return true;
}

bool URopeComponent::ComputeWrapSurfaceTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
	FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const
{
	const ERopeWrappingPathMode PathMode = GetWrappingPathMode();
	if (PathMode == ERopeWrappingPathMode::AnalyticHelix &&
		ComputeAnalyticHelixWrapTarget(LatchAnchor, DistanceFromLatch,
			OutSurfaceWorld, OutNormalWorld, OutTangentWorld))
	{
		return true;
	}

	if (PathMode == ERopeWrappingPathMode::SurfaceVectorField &&
		ComputeSurfaceVectorFieldWrapTarget(LatchAnchor, DistanceFromLatch,
			OutSurfaceWorld, OutNormalWorld, OutTangentWorld))
	{
		return true;
	}

	return ComputeSurfaceWalkWrapTarget(LatchAnchor, DistanceFromLatch,
		OutSurfaceWorld, OutNormalWorld, OutTangentWorld);
}

bool URopeComponent::ComputeSurfaceWalkWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
	FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const
{
	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = WrappingState.Mesh.Get();
	}
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);

	FVector SurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
	FVector NormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector TangentWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalTangent);
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, AnyTangentFromNormal(NormalWorld));

	if (DistanceFromLatch <= KINDA_SMALL_NUMBER)
	{
		OutSurfaceWorld = SurfaceWorld;
		OutNormalWorld = NormalWorld;
		OutTangentWorld = TangentWorld;
		return true;
	}

	const float StepSize = FMath::Max(1.0f, Sim.SegmentLength * 0.5f);
	const int32 StepCount = FMath::Max(1, FMath::CeilToInt(DistanceFromLatch / StepSize));
	float RemainingDistance = DistanceFromLatch;

	for (int32 StepIndex = 0; StepIndex < StepCount; ++StepIndex)
	{
		const float StepDistance = FMath::Min(StepSize, RemainingDistance);
		RemainingDistance -= StepDistance;

		SurfaceWorld += TangentWorld * StepDistance;
		ProjectWrapPointToSurface(LatchAnchor.Bone, Mesh, SurfaceWorld, NormalWorld);

		TangentWorld = TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld;
		TangentWorld = TangentWorld.GetSafeNormal(KINDA_SMALL_NUMBER, AnyTangentFromNormal(NormalWorld));
	}

	OutSurfaceWorld = SurfaceWorld;
	OutNormalWorld = NormalWorld;
	OutTangentWorld = TangentWorld;
	return true;
}

bool URopeComponent::ComputeSurfaceVectorFieldWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
	FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const
{
	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = WrappingState.Mesh.Get();
	}
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	FVector AxisOrigin = FVector::ZeroVector;
	FVector AxisDirection = FVector::ForwardVector;
	if (!ResolveWrappingAxis(LatchAnchor, AxisOrigin, AxisDirection))
	{
		return false;
	}

	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);
	FVector SurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
	FVector NormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector LatchTangentWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalTangent);
	LatchTangentWorld = (LatchTangentWorld - FVector::DotProduct(LatchTangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, AnyTangentFromNormal(NormalWorld));

	const float LatchAxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
	const FVector LatchAxisPoint = AxisOrigin + AxisDirection * LatchAxisDistance;
	const FVector LatchRadial = (SurfaceWorld - LatchAxisPoint)
		.GetSafeNormal(KINDA_SMALL_NUMBER, NormalWorld);

	FVector CircumferenceDir = FVector::CrossProduct(AxisDirection, LatchRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, AnyTangentFromNormal(NormalWorld));
	const float WindingSign = FVector::DotProduct(CircumferenceDir, LatchTangentWorld) < 0.0f ? -1.0f : 1.0f;
	CircumferenceDir *= WindingSign;

	FVector TangentWorld = (CircumferenceDir + AxisDirection * WrapConfig.WrappingHelixPitchScale)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);

	if (DistanceFromLatch <= KINDA_SMALL_NUMBER)
	{
		OutSurfaceWorld = SurfaceWorld;
		OutNormalWorld = NormalWorld;
		OutTangentWorld = TangentWorld;
		return true;
	}

	const float StepSize = FMath::Max(1.0f, Sim.SegmentLength * 0.5f);
	const int32 StepCount = FMath::Max(1, FMath::CeilToInt(DistanceFromLatch / StepSize));
	float RemainingDistance = DistanceFromLatch;

	for (int32 StepIndex = 0; StepIndex < StepCount; ++StepIndex)
	{
		const float StepDistance = FMath::Min(StepSize, RemainingDistance);
		RemainingDistance -= StepDistance;

		const float AxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
		const FVector AxisPoint = AxisOrigin + AxisDirection * AxisDistance;
		const FVector Radial = (SurfaceWorld - AxisPoint).GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);

		CircumferenceDir = FVector::CrossProduct(AxisDirection, Radial)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir) * WindingSign;
		TangentWorld = (CircumferenceDir + AxisDirection * WrapConfig.WrappingHelixPitchScale)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);
		TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);

		SurfaceWorld += TangentWorld * StepDistance;
		ProjectWrapPointToSurface(LatchAnchor.Bone, Mesh, SurfaceWorld, NormalWorld);
	}

	const float FinalAxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
	const FVector FinalAxisPoint = AxisOrigin + AxisDirection * FinalAxisDistance;
	const FVector FinalRadial = (SurfaceWorld - FinalAxisPoint).GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);
	CircumferenceDir = FVector::CrossProduct(AxisDirection, FinalRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir) * WindingSign;
	TangentWorld = (CircumferenceDir + AxisDirection * WrapConfig.WrappingHelixPitchScale)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir);

	OutSurfaceWorld = SurfaceWorld;
	OutNormalWorld = NormalWorld;
	OutTangentWorld = TangentWorld;
	return true;
}

bool URopeComponent::ComputeAnalyticHelixWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
	FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const
{
	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = WrappingState.Mesh.Get();
	}
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	FVector AxisOrigin = FVector::ZeroVector;
	FVector AxisDirection = FVector::ForwardVector;
	if (!ResolveWrappingAxis(LatchAnchor, AxisOrigin, AxisDirection))
	{
		return false;
	}

	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);
	const FVector LatchSurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
	const FVector LatchNormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector LatchTangentWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalTangent);
	LatchTangentWorld = (LatchTangentWorld - FVector::DotProduct(LatchTangentWorld, LatchNormalWorld) * LatchNormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, AnyTangentFromNormal(LatchNormalWorld));

	const float LatchAxisDistance = FVector::DotProduct(LatchSurfaceWorld - AxisOrigin, AxisDirection);
	const FVector LatchAxisPoint = AxisOrigin + AxisDirection * LatchAxisDistance;
	FVector LatchRadial = LatchSurfaceWorld - LatchAxisPoint;
	const float HelixRadius = LatchRadial.Size();
	if (HelixRadius <= KINDA_SMALL_NUMBER)
	{
		return false;
	}
	LatchRadial /= HelixRadius;

	FVector CircumferenceDir = FVector::CrossProduct(AxisDirection, LatchRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, AnyTangentFromNormal(LatchNormalWorld));
	const float WindingSign = FVector::DotProduct(CircumferenceDir, LatchTangentWorld) < 0.0f ? -1.0f : 1.0f;
	CircumferenceDir *= WindingSign;

	const float PitchScale = WrapConfig.WrappingHelixPitchScale;
	const float LengthScale = FMath::Sqrt(1.0f + PitchScale * PitchScale);
	const float CircumferenceDistance = DistanceFromLatch / FMath::Max(LengthScale, KINDA_SMALL_NUMBER);
	const float AxisDistance = CircumferenceDistance * PitchScale;
	const float AngleRadians = WindingSign * CircumferenceDistance / FMath::Max(HelixRadius, KINDA_SMALL_NUMBER);

	const FQuat AxisRotation(AxisDirection, AngleRadians);
	const FVector RotatedRadial = AxisRotation.RotateVector(LatchRadial).GetSafeNormal(KINDA_SMALL_NUMBER, LatchRadial);
	const FVector TargetAxisPoint = AxisOrigin + AxisDirection * (LatchAxisDistance + AxisDistance);

	FVector SurfaceWorld = TargetAxisPoint + RotatedRadial * HelixRadius;
	FVector NormalWorld = RotatedRadial;
	ProjectWrapPointToSurface(LatchAnchor.Bone, Mesh, SurfaceWorld, NormalWorld);

	const float SurfaceAxisDistance = FVector::DotProduct(SurfaceWorld - AxisOrigin, AxisDirection);
	const FVector SurfaceAxisPoint = AxisOrigin + AxisDirection * SurfaceAxisDistance;
	FVector SurfaceRadial = SurfaceWorld - SurfaceAxisPoint;
	SurfaceRadial = SurfaceRadial.GetSafeNormal(KINDA_SMALL_NUMBER, RotatedRadial);

	FVector SurfaceCircumferenceDir = FVector::CrossProduct(AxisDirection, SurfaceRadial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, CircumferenceDir) * WindingSign;
	FVector TangentWorld = (SurfaceCircumferenceDir + AxisDirection * PitchScale)
		.GetSafeNormal(KINDA_SMALL_NUMBER, SurfaceCircumferenceDir);
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, SurfaceCircumferenceDir);

	OutSurfaceWorld = SurfaceWorld;
	OutNormalWorld = NormalWorld;
	OutTangentWorld = TangentWorld;
	return true;
}

bool URopeComponent::ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor,
	FVector& OutAxisOrigin, FVector& OutAxisDirection) const
{
	const USkeletalMeshComponent* Mesh = LatchAnchor.Mesh.Get();
	if (!Mesh)
	{
		Mesh = WrappingState.Mesh.Get();
	}
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	const FName ParentBone = Mesh->GetParentBone(LatchAnchor.Bone);
	const FVector BoneLocation = Mesh->GetSocketTransform(LatchAnchor.Bone).GetLocation();
	if (!ParentBone.IsNone())
	{
		const FVector ParentLocation = Mesh->GetSocketTransform(ParentBone).GetLocation();
		const FVector Axis = BoneLocation - ParentLocation;
		if (!Axis.IsNearlyZero())
		{
			OutAxisOrigin = ParentLocation;
			OutAxisDirection = Axis.GetSafeNormal();
			return true;
		}
	}

	const FTransform BoneXform = Mesh->GetSocketTransform(LatchAnchor.Bone);
	OutAxisOrigin = BoneLocation;
	OutAxisDirection = BoneXform.GetUnitAxis(EAxis::X).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	return true;
}

ERopeWrappingPathMode URopeComponent::GetWrappingPathMode() const
{
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	return Settings ? Settings->WrappingPathMode : ERopeWrappingPathMode::SurfaceVectorField;
}

bool URopeComponent::ProjectWrapPointToSurface(FName Bone, const USkeletalMeshComponent* Mesh,
	FVector& InOutSurfaceWorld, FVector& InOutNormalWorld) const
{
	FRopeContact BestContact;
	bool bFound = false;

	const float QueryRadius = FMath::Max(FMath::Max(WrapConfig.ContactRadius, Radius), Sim.SegmentLength);
	for (const IRopeCollider* Collider : FrameColliders)
	{
		if (!Collider)
		{
			continue;
		}

		const FRopeContact Contact = Collider->Query(InOutSurfaceWorld, QueryRadius);
		if (!Contact.bHit || Contact.Bone != Bone)
		{
			continue;
		}

		if (Mesh && Contact.SourceMesh && Contact.SourceMesh != Mesh)
		{
			continue;
		}

		if (!bFound || Contact.Penetration > BestContact.Penetration)
		{
			BestContact = Contact;
			bFound = true;
		}
	}

	if (!bFound)
	{
		return false;
	}

	InOutSurfaceWorld = BestContact.SurfacePoint;
	InOutNormalWorld = BestContact.Normal.GetSafeNormal(KINDA_SMALL_NUMBER, InOutNormalWorld);
	return true;
}

void URopeComponent::ApplyWrappingTargetMotion(float /*DeltaTime*/)
{
	const float Duration = FMath::Max(WrappingState.Duration, KINDA_SMALL_NUMBER);
	const float SegmentLength = FMath::Max(Sim.SegmentLength, KINDA_SMALL_NUMBER);

	for (const FRopeSurfaceAnchor& Anchor : WrappingState.Anchors)
	{
		if (!Sim.Positions.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
		{
			continue;
		}

		const USkeletalMeshComponent* Mesh = Anchor.Mesh.Get();
		if (!Mesh)
		{
			continue;
		}

		const FTransform BoneXform = Mesh->GetSocketTransform(Anchor.Bone);
		const FVector SurfaceWorld = BoneXform.TransformPosition(Anchor.LocalSurfacePosition);
		const FVector NormalWorld = BoneXform.TransformVectorNoScale(Anchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		const FVector TargetWorld = SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;

		const float TailSegment = Anchor.RopeDistance / SegmentLength;
		const float TailDelay = TailSegment * WrapConfig.WrappingTailDelayPerSegment;
		const float Alpha = SmoothStep(FMath::Clamp((WrappingState.Elapsed - TailDelay) / Duration, 0.0f, 1.0f));
		const FVector World = FMath::Lerp(Anchor.StartWorldPosition, TargetWorld, Alpha);

		Sim.Positions[Anchor.NodeIndex] = World;
		Sim.PrevPositions[Anchor.NodeIndex] = World;
		Sim.InvMass[Anchor.NodeIndex] = 0.0f;
	}
}

bool URopeComponent::UpdateWrappingAnchorsFromCandidates(const TArray<FRopeContactCandidate>& Candidates)
{
	bool bSawWrappingContact = false;
	const int32 HeadAnchorNode = WrappingState.FirstNode;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid ||
			Candidate.Bone != WrappingState.BoneName ||
			Candidate.NodeIndex != HeadAnchorNode ||
			!Sim.Positions.IsValidIndex(Candidate.NodeIndex))
		{
			continue;
		}

		const USkeletalMeshComponent* Mesh = Candidate.Mesh ? Candidate.Mesh : WrappingState.Mesh.Get();
		if (!Mesh)
		{
			continue;
		}

		bSawWrappingContact = true;

		FRopeSurfaceAnchor* ExistingAnchor = WrappingState.Anchors.FindByPredicate(
			[&Candidate, Mesh](const FRopeSurfaceAnchor& Anchor)
			{
				return Anchor.NodeIndex == Candidate.NodeIndex && Anchor.Bone == Candidate.Bone && Anchor.Mesh.Get() == Mesh;
			});

		if (ExistingAnchor)
		{
			continue;
		}

		const FTransform BoneXform = Mesh->GetSocketTransform(Candidate.Bone);

		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = Candidate.NodeIndex;
		Anchor.Bone = Candidate.Bone;
		Anchor.Mesh = Mesh;
		Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Candidate.WorldPoint);
		Anchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(Candidate.Normal).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		Anchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(ExpectedWrapTangent(Candidate)).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		Anchor.StartWorldPosition = Sim.Positions[Candidate.NodeIndex];
		Anchor.SurfaceOffset = FMath::Max(0.0f, Radius);
		Anchor.RopeDistance = static_cast<float>(Candidate.NodeIndex) * Sim.SegmentLength;

		WrappingState.Anchors.Add(Anchor);
	}

	WrappingState.FirstNode = TNumericLimits<int32>::Max();
	WrappingState.LastNode = INDEX_NONE;
	for (const FRopeSurfaceAnchor& Anchor : WrappingState.Anchors)
	{
		WrappingState.FirstNode = FMath::Min(WrappingState.FirstNode, Anchor.NodeIndex);
		WrappingState.LastNode = FMath::Max(WrappingState.LastNode, Anchor.NodeIndex);
	}
	if (WrappingState.Anchors.Num() == 0)
	{
		WrappingState.FirstNode = INDEX_NONE;
	}

	return bSawWrappingContact;
}

void URopeComponent::ApplyWrappingMassMask()
{
	TSet<int32> AnchorNodes;
	for (const FRopeSurfaceAnchor& Anchor : WrappingState.Anchors)
	{
		if (Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
		{
			AnchorNodes.Add(Anchor.NodeIndex);
		}
	}

	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		Sim.InvMass[i] = (bStartPin || AnchorNodes.Contains(i)) ? 0.0f : 1.0f;
	}
}

void URopeComponent::CommitWrapping()
{
	const USkeletalMeshComponent* Mesh = WrappingState.Mesh.Get();

	//Wrapping 정보가 적절하지 않으면 바로 releasing
	if (!Mesh || WrappingState.BoneName.IsNone() || WrappingState.Anchors.Num() == 0)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Wrapping -> Releasing (commit failed)"), *GetName());
		AbortWrapping(ERopeReleaseReason::Broken);
		Phase = ERopePhase::Releasing;
		return;
	}

	FRopeWrapState Seed;
	Seed.BoneName = WrappingState.BoneName;
	Seed.Mesh = Mesh;

	//각 LatchedNode에 정보 입력
	for (const FRopeSurfaceAnchor& Anchor : WrappingState.Anchors)
	{
		if (!Sim.Positions.IsValidIndex(Anchor.NodeIndex))
		{
			continue;
		}

		FRopeLatchNode Latch;
		Latch.NodeIndex = Anchor.NodeIndex;
		Latch.Bone = Anchor.Bone;
		Seed.Latched.Add(Latch);
		Seed.Anchors.Add(Anchor);
	}

	if (Seed.Anchors.Num() == 0)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Wrapping -> Releasing (no valid latches)"), *GetName());
		AbortWrapping(ERopeReleaseReason::Broken);
		Phase = ERopePhase::Releasing;
		return;
	}

	WrapController.BeginWrap(Sim, Seed, Mesh);

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] Wrapping -> Wrapped (bone=%s, %d latched node(s))"),
		*GetName(), *Seed.BoneName.ToString(), Seed.Latched.Num());

	WrappingState.Reset();
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	ContactingElapsed = 0.0f;

	Phase = ERopePhase::Wrapped;
	OnRopeWrapped.Broadcast(Seed.BoneName);
}

void URopeComponent::AbortWrapping(ERopeReleaseReason Reason)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] AbortWrapping reason=%d"),
		*GetName(), static_cast<int32>(Reason));

	for (const FRopeSurfaceAnchor& Anchor : WrappingState.Anchors)
	{
		if (!Sim.InvMass.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.Positions.IsValidIndex(Anchor.NodeIndex))
		{
			continue;
		}

		Sim.InvMass[Anchor.NodeIndex] = 1.0f;
		Sim.PrevPositions[Anchor.NodeIndex] = Sim.Positions[Anchor.NodeIndex];
	}

	WrappingState.Reset();
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	ContactingElapsed = 0.0f;

	ReleaseCooldown = 0.08f;

}

void URopeComponent::PrepareSimFrame(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Prepare);

	EnsureRopeInitialized();

	// pinned-start target을 전진시킨다; solver가 substep에 걸쳐 Prev->Target을 sweep하므로 빠른
	// 캐릭터 이동이 chain을 홱 잡아당겨(폭주시켜) 버리지 않는다.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	// collider 스냅샷은 RopeSimSubsystem이 Tick의 collider 단계에서 중앙 수집해 FrameColliders에 채워둔다
	// (Prepare 이전). 여기서 로프마다 provider를 탐색/gather하지 않는다.

	bSolveThisFrame = false;

	switch (Phase)
	{
	case ERopePhase::Free:        // 손에서 늘어뜨려진 채 캐릭터를 따라간다
		bSolveThisFrame = true;
		break;

	case ERopePhase::Flight:
		if (bWhipSwingActive)
		{
			ApplyWhipSwing(DeltaTime);
		}
		else
		{
			DebugWhipGuideNodeIndices.Reset();
			DebugWhipGuideTargets.Reset();
			WhipGuidePrevTargetsThisFrame.Reset();
			WhipGuideCurrentTargetsThisFrame.Reset();
			WhipGuidedNodesThisFrame.Reset();
		}
		bSolveThisFrame = true; // 솔브 후 접촉 감지는 FinalizeSimFrame에서.
		break;

	case ERopePhase::Contacting:
		UpdateContacting(DeltaTime);
		break;

	case ERopePhase::Wrapping:
		UpdateWrapping(DeltaTime);
		break;

	case ERopePhase::Wrapped:
	{
		// latch된 node는 skinned bone을 따라간다(GT). 솔브 없음.
		// Hold가 false면 wrap 대상 mesh가 사라진 것(예: cross-actor 대상 액터 파괴) →
		// 노드를 솔버에 되돌려 안전하게 release한다(dangling 포인터 역참조 방지는 Hold 내부에서).
		if (!WrapController.Hold(Sim, ResolveWrapTargetMesh(), DeltaTime))
		{
			const FName Bone = WrapController.State.BoneName;
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Wrapped -> Releasing (wrap target mesh lost, bone=%s)"),
				*GetName(), *Bone.ToString());
			WrapController.Release(ERopeReleaseReason::Broken);
			ContactTracker.Reset();
			PendingWrapSeed.Reset();
			WrappingState.Reset();
			ContactingElapsed = 0.0f;
			ReleaseCooldown = 0.08f;
			Phase = ERopePhase::Releasing;
			OnRopeReleased.Broadcast(Bone, ERopeReleaseReason::Broken);
			break;
		}
		ApplyWrappedMassMask();
		bSolveThisFrame = true;
		UpdateWrappedKinematicShape(DeltaTime); // 선택: 찰랑임 연출만
		break;
	}

	case ERopePhase::Releasing:
		// 모든 node를 solver에 다시 넘긴다(hand pin만 유지), 그런 다음 free simulation을 재개한다.
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.InvMass[i] = (i == 0 && Sim.bStartPinned) ? 0.0f : 1.0f;

			// 이전 고정점 때문에 튀지 않게 PrevPositions 보정.
			Sim.PrevPositions[i] = Sim.Positions[i];
		}
		ReleaseCooldown -= DeltaTime;
		if (ReleaseCooldown <= 0)
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Releasing -> Free"), *GetName());
			Phase = ERopePhase::Free;
		}
		break;

	default:
		break;
	}

	// GPU 상주(M5): logic phase(Contacting/Wrapped/Releasing)는 Sim을 out-of-band로 바꾸고, whip은 CPU에서
	// 위치를 가이드한다 → 다음 GPU step에서 재시드되도록 generation을 올린다. 정상 Free/Flight(비-whip)에선
	// 불변이라 GPU 버퍼가 상주된 채 매 프레임 in-place로 전진한다.
	const bool bLogicMutatedSim =
		!bSolveThisFrame ||
		Phase == ERopePhase::Wrapping ||
		Phase == ERopePhase::Wrapped;
	if (bLogicMutatedSim || bWhipSwingActive)
	{
		++SimGeneration;
	}
}

void URopeComponent::SolveSimFrame(float DeltaTime)
{
	// 병렬 단계: POD 상태(Sim) + collider 스냅샷(FrameColliders)만 만진다. Query는 const → 스레드 안전.
	// Free/Flight만 물리 솔브(Contacting/Wrapped/Releasing은 로직 구동 = Prepare에서 GT 처리).
	if (!bSolveThisFrame)
	{
		return;
	}
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Solve);
	Solver.Step(Sim, SolverConfig, /*optional*/ FrameColliders, DeltaTime);
}

void URopeComponent::FinalizeSimFrame(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Finalize);

	// 디버그 캡처 게이트: 이 로프가 게이트플레이 디버거의 대상 액터일 때만 비주얼 데이터를 모은다.
	// 대상이 아닌 로프는 아래 flight sweep 등 캡처 비용을 전혀 내지 않는다(타깃 1개 로프만 부담).
#if WITH_GAMEPLAY_DEBUGGER
	URopeDebugSubsystem* DebugSub = URopeDebugSubsystem::Get(GetWorld());
	const bool bDebugCapture = DebugSub && DebugSub->ShouldCapture(this);
	FRopeDebugSnapshot DebugSnapshot;
#else
	constexpr bool bDebugCapture = false;
#endif

	// Flight: 솔브 후 이동 경로 기반 접촉 후보 감지 → 캡처. UObject·이벤트라 GT에서.
	if (Phase == ERopePhase::Flight)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FinalizeFlight);
		TArray<FRopeContactCandidate> Candidates;
		TArray<FRopeFlightNodeDebug> FlightNodeDebug;
		if (bDebugCapture)
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightDebugGather);
			for (int32 i = 0; i < Sim.Num(); ++i)
			{
				if (!Sim.PrevPositions.IsValidIndex(i) || !Sim.Positions.IsValidIndex(i))
				{
					continue;
				}

				FRopeFlightNodeDebug NodeDebug;
				NodeDebug.NodeIndex = i;
				NodeDebug.PrevPosition = Sim.PrevPositions[i];
				NodeDebug.Position = Sim.Positions[i];
				NodeDebug.NodeSpeed = NodeSpeed(i);
				NodeDebug.bFast = IsTailNode(i) || NodeDebug.NodeSpeed > Sim.SegmentLength;
				NodeDebug.bNearBody = IsNearAnyColliderSegment(NodeDebug.PrevPosition, NodeDebug.Position, FrameColliders);
				if (NodeDebug.bFast || NodeDebug.bNearBody)
				{
					NodeDebug.Contact = SweepOrSampleContact(NodeDebug.PrevPosition, NodeDebug.Position, FrameColliders);
				}

				if (NodeDebug.bFast || NodeDebug.bNearBody || NodeDebug.Contact.bHit)
				{
					FlightNodeDebug.Add(NodeDebug);
				}
			}
		}

		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightActualContacts);
			DetectContactCandidates(Sim.PrevPositions, Sim.Positions, FrameColliders, Candidates);
		}
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightPredictiveContacts);
			AddPredictedContactCandidates(Candidates, DeltaTime);
		}
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightEvaluateCandidates);
			EvaluateRelativeMotion(Candidates);
		}

		FRopeContactTracker FlightDebugTracker;
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightTrackerUpdate);
			FlightDebugTracker.Update(Candidates, 0.0f);
		}
		bool bShouldCapture = false;
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightShouldCapture);
			bShouldCapture = ShouldCapture(Candidates);
		}
		constexpr bool bEnableFlightNoContactReturn = false;

		if (bShouldCapture)
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightBuildContactingState);
			BuildContactingState(Candidates);
			FlightNoContactElapsed = 0.0f;
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Flight -> Contacting (bone=%s, %d node(s))"),
				*GetName(), *ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num());
			Phase = ERopePhase::Contacting;
			OnRopeCaptured.Broadcast(ContactTracker.CandidateBone);
		}
		else if (bEnableFlightNoContactReturn && !bWhipSwingActive && Candidates.Num() == 0 && WrapConfig.FlightNoContactReturnTime > 0.0f)
		{
			FlightNoContactElapsed += DeltaTime;
			if (FlightNoContactElapsed >= WrapConfig.FlightNoContactReturnTime)
			{
				UE_LOG(LogDynamicRope, Log, TEXT("[%s] Flight -> Free (no contact candidates for %.2fs)"),
					*GetName(), FlightNoContactElapsed);
				ContactTracker.Reset();
				PendingWrapSeed.Reset();
				WrappingState.Reset();
				ContactingElapsed = 0.0f;
				FlightNoContactElapsed = 0.0f;
				Phase = ERopePhase::Free;
			}
		}
		else
		{
			FlightNoContactElapsed = 0.0f;
		}

		// stat 카운터(stat 시스템이 수집 중일 때만; 디버그 캡처와 독립).
		const FRopeContactTracker& DebugTracker = bShouldCapture ? ContactTracker : FlightDebugTracker;
		const float WhipGuidedEnd = FMath::Clamp(WhipGuidedLength, 0.05f, 0.95f);
		const bool bWhipActive = DebugWhipGuideTargets.Num() > 0;
		RopeDebug::RecordFlightStats(Sim, bSolveThisFrame, FrameColliders.Num(), Candidates,
			DebugTracker, WrapConfig, bShouldCapture);
		RopeDebug::RecordWhipStats(Sim, DebugWhipGuideNodeIndices, DebugWhipGuideTargets, WhipGuidedEnd, bWhipActive);

#if WITH_GAMEPLAY_DEBUGGER
		if (bDebugCapture)
		{
			DebugSnapshot.bHasFlight = true;
			DebugSnapshot.bSolveThisFrame = bSolveThisFrame;
			DebugSnapshot.bShouldCapture = bShouldCapture;
			DebugSnapshot.FrameColliderCount = FrameColliders.Num();
			DebugSnapshot.MinLatchNodes = WrapConfig.MinLatchNodes;
			DebugSnapshot.TrackerBone = DebugTracker.CandidateBone;
			DebugSnapshot.TrackerNodes = DebugTracker.CandidateNodes;
			DebugSnapshot.NodeDebug = MoveTemp(FlightNodeDebug);
			DebugSnapshot.Candidates = Candidates;
			DebugSnapshot.bWhipActive = bWhipActive;
			DebugSnapshot.WhipGuidedEnd = WhipGuidedEnd;
			DebugSnapshot.WhipGuideNodeIndices = DebugWhipGuideNodeIndices;
			DebugSnapshot.WhipGuideTargets = DebugWhipGuideTargets;
		}
#endif
	}

	// 새 centerline을 render proxy로 push하고 bounds를 갱신한다.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_MarkRenderDirty);
		MarkRenderDynamicDataDirty();
		MarkRenderTransformDirty();
	}

	// wrapped stat 카운터(독립).
	if (Phase == ERopePhase::Wrapped)
	{
		RopeDebug::RecordWrappedStats(Sim, WrapController.State);
	}

	// 디버그 스냅샷 제출: centerline/collider/wrapped 공통 필드를 채워 디버거 보관소로 넘긴다.
#if WITH_GAMEPLAY_DEBUGGER
	if (bDebugCapture)
	{
		FillDebugSnapshot(DebugSnapshot);
		DebugSub->SubmitSnapshot(this, MoveTemp(DebugSnapshot));
	}
#endif
}

#if WITH_GAMEPLAY_DEBUGGER
void URopeComponent::FillDebugSnapshot(FRopeDebugSnapshot& Snapshot) const
{
	Snapshot.Phase = Phase;
	Snapshot.Positions = Sim.Positions;

	// centerline 상에서 강조할 latch 노드 인덱스.
	const FRopeWrapState& Wrap = WrapController.State;
	Snapshot.LatchedNodes.Reset();
	for (const FRopeLatchNode& Latch : Wrap.Latched)
	{
		Snapshot.LatchedNodes.Add(Latch.NodeIndex);
	}

	// wrapped 상세(테이블용)는 Wrapped phase일 때만.
	if (Phase == ERopePhase::Wrapped && Wrap.IsWrapped())
	{
		Snapshot.bHasWrapped = true;
		Snapshot.WrapBone = Wrap.BoneName;
		const USkeletalMeshComponent* Mesh = Wrap.Mesh.Get();
		Snapshot.MeshName = Mesh ? Mesh->GetName() : TEXT("None");
		Snapshot.Latched = Wrap.Latched;
	}

	// 이 로프가 이번 프레임 질의한 collider 시각화(provider bDrawDebug 대체). capsule이면 세그먼트,
	// 그 외(SDF 등)는 월드 bounds 박스. FrameColliders는 provider 소유라 이 프레임 동안만 유효.
	Snapshot.Colliders.Reset();
	for (const IRopeCollider* Collider : FrameColliders)
	{
		if (!Collider)
		{
			continue;
		}
		FRopeDebugCollider DC;
		if (Collider->GetGPUCapsule(DC.A, DC.B, DC.Radius))
		{
			DC.bIsCapsule = true;
		}
		else
		{
			DC.bIsCapsule = false;
			DC.Bounds = Collider->GetWorldBounds();
		}
		Snapshot.Colliders.Add(DC);
	}
}
#endif

void URopeComponent::SendRenderDynamicData_Concurrent()
{
	Super::SendRenderDynamicData_Concurrent();

	if (!SceneProxy || Sim.Num() < 2)
	{
		return;
	}

	// centerline을 component-local 공간으로 보낸다; proxy는 GetLocalToWorld()를 통해 렌더링한다.
	const FTransform Xform = GetComponentTransform();
	FRopeDynamicData* DynamicData = new FRopeDynamicData;
	DynamicData->bGpuResident = bGpuSteppedThisFrame; // M5b: GPU step된 프레임만 resident PosBuf 직접 렌더 허용.
	DynamicData->Points.SetNumUninitialized(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		DynamicData->Points[i] = Xform.InverseTransformPosition(Sim.Positions[i]);
	}

	FRopeSceneProxy* Proxy = static_cast<FRopeSceneProxy*>(SceneProxy);
	ENQUEUE_RENDER_COMMAND(RopeUpdateCenterline)(
		[Proxy, DynamicData](FRHICommandListBase& RHICmdList)
		{
			Proxy->SetDynamicData_RenderThread(RHICmdList, DynamicData);
		});
}

void URopeComponent::Throw(const FVector& AimDir)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] Throw requested (phase=%s, aim=%s)"),
		*GetName(), PhaseName(Phase), *AimDir.GetSafeNormal().ToCompactString());

	EnsureRopeInitialized();
	StartFreshThrow(AimDir);
}

void URopeComponent::ThrowFreeSpanWhileWrapped(const FVector& AimDir)
{
	WrappedSwayImpulse = FVector::ZeroVector;
	WrappedSwayTime = 0.0f;
}

float URopeComponent::TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const
{
	if (LastNode <= FirstTailNode)
	{
		return NodeIndex >= LastNode ? 1.0f : 0.0f;
	}

	const float T = static_cast<float>(NodeIndex - FirstTailNode) / static_cast<float>(LastNode - FirstTailNode);
	return SmoothStep(T);
}

bool URopeComponent::IsTailNode(int32 NodeIndex) const
{
	return NodeIndex >= FMath::Max(1, Sim.Num() - 4);
}

bool URopeComponent::IsWhipGuidedNodeThisFrame(int32 NodeIndex) const
{
	return WhipGuidedNodesThisFrame.IsValidIndex(NodeIndex) && WhipGuidedNodesThisFrame[NodeIndex] != 0;
}

bool URopeComponent::ShouldRunPredictiveContactForNode(int32 NodeIndex, bool bHasGuidedNodes, const FVector& FrameDisplacement) const
{
	if (!bHasGuidedNodes)
	{
		return true;
	}

	return IsWhipGuidedNodeThisFrame(NodeIndex) || IsTailNode(NodeIndex) || FrameDisplacement.Size() > Sim.SegmentLength;
}

float URopeComponent::NodeSpeed(int32 NodeIndex) const
{
	if (!Sim.Positions.IsValidIndex(NodeIndex) || !Sim.PrevPositions.IsValidIndex(NodeIndex))
	{
		return 0.0f;
	}
	return (Sim.Positions[NodeIndex] - Sim.PrevPositions[NodeIndex]).Size();
}

bool URopeComponent::IsNearAnyColliderSegment(const FVector& PrevPosition, const FVector& Position, const TArray<IRopeCollider*>& Colliders) const
{
	TArray<IRopeCollider*> NearbyColliders;
	GatherNearbyColliders(PrevPosition, Position, Colliders, NearbyColliders);
	return NearbyColliders.Num() > 0;
}

void URopeComponent::GatherNearbyColliders(const FVector& PrevPosition, const FVector& Position,
	const TArray<IRopeCollider*>& Colliders, TArray<IRopeCollider*>& OutNearbyColliders) const
{
	OutNearbyColliders.Reset();

	FBox SegmentBounds(ForceInit);
	SegmentBounds += PrevPosition;
	SegmentBounds += Position;
	SegmentBounds = SegmentBounds.ExpandBy(WrapConfig.ContactRadius + Radius + 5.0f);

	for (IRopeCollider* Collider : Colliders)
	{
		if (Collider && SegmentBounds.Intersect(Collider->GetWorldBounds().ExpandBy(WrapConfig.ContactRadius + Radius)))
		{
			OutNearbyColliders.Add(Collider);
		}
	}
}

FRopeContact URopeComponent::SweepOrSampleContact(const FVector& PrevPosition, const FVector& Position, const TArray<IRopeCollider*>& Colliders) const
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

			const FRopeContact Contact = Collider->Query(SamplePos, WrapConfig.ContactRadius);
			if (Contact.bHit && (!Best.bHit || Contact.Penetration > Best.Penetration))
			{
				Best = Contact;
			}
		}
	}

	return Best;
}

FRopeContactCandidate URopeComponent::MakeCandidate(int32 NodeIndex, const FRopeContact& Contact) const
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

FVector URopeComponent::ExpectedWrapTangent(const FRopeContactCandidate& Candidate) const
{
	const FVector ToHand = (Sim.Num() > 0) ? (Sim.Positions[0] - Candidate.WorldPoint).GetSafeNormal() : GetForwardVector();
	const FVector Tangent = ToHand - FVector::DotProduct(ToHand, Candidate.Normal) * Candidate.Normal;
	return Tangent.GetSafeNormal(UE_SMALL_NUMBER, GetForwardVector());
}

bool URopeComponent::ShouldCapture(const TArray<FRopeContactCandidate>& Candidates) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightShouldCaptureImpl);
	FRopeContactTracker TempTracker;
	TempTracker.Update(Candidates, 0.0f);
	return TempTracker.CandidateNodes.Num() >= FMath::Max(1, WrapConfig.MinLatchNodes)
		&& IsWrappableBone(TempTracker.CandidateBone);
}

void URopeComponent::BuildContactingState(const TArray<FRopeContactCandidate>& Candidates)
{
	ContactTracker.Reset();
	ContactTracker.Update(Candidates, 0.0f);
	ContactingElapsed = 0.0f;
	PendingWrapSeed = BuildWrapSeedFromContactingState(Candidates);
}

void URopeComponent::AdvanceWrappingMotion(float DeltaTime)
{
	ContactingElapsed += DeltaTime;
}

bool URopeComponent::ShouldDismissContacting() const
{
	return ContactTracker.CandidateBone.IsNone() || ContactTracker.CandidateNodes.Num() == 0;
}

//bool URopeComponent::ShouldFinishWrapping() const
//{
//	return false;
//}

bool URopeComponent::ShouldStartWrapping() const
{
	return ContactingElapsed >= WrapConfig.WrapDecisionTime
		&& PendingWrapSeed.Latched.Num() > 0
		&& !PendingWrapSeed.BoneName.IsNone();
}

FRopeWrapState URopeComponent::BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const
{
	FRopeWrapState Seed;
	Seed.BoneName = ContactTracker.CandidateBone;
	Seed.Mesh = ContactTracker.CandidateMesh;
	const int32 NodeIndex = FindHeadValidNodeIndex(ContactTracker.CandidateNodes, Sim);
	if (NodeIndex != INDEX_NONE)
	{
		FRopeLatchNode Latch;
		Latch.NodeIndex = NodeIndex;
		Latch.Bone = ContactTracker.CandidateBone;
		Seed.Latched.Add(Latch);

		const FRopeContactCandidate* LatchCandidate = nullptr;
		for (const FRopeContactCandidate& Candidate : Candidates)
		{
			if (!Candidate.bValid ||
				Candidate.NodeIndex != NodeIndex ||
				Candidate.Bone != ContactTracker.CandidateBone)
			{
				continue;
			}

			if (!LatchCandidate || Candidate.Penetration > LatchCandidate->Penetration)
			{
				LatchCandidate = &Candidate;
			}
		}

		const USkeletalMeshComponent* Mesh = Seed.Mesh.Get();
		if (!Mesh && LatchCandidate)
		{
			Mesh = LatchCandidate->Mesh;
			Seed.Mesh = Mesh;
		}

		if (LatchCandidate && Mesh)
		{
			const FVector NormalWorld = LatchCandidate->Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
			FVector TangentWorld = ExpectedWrapTangent(*LatchCandidate);
			if (Sim.Positions.IsValidIndex(NodeIndex + 1))
			{
				TangentWorld = Sim.Positions[NodeIndex + 1] - Sim.Positions[NodeIndex];
			}
			TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
				.GetSafeNormal(KINDA_SMALL_NUMBER, AnyTangentFromNormal(NormalWorld));

			const FTransform BoneXform = Mesh->GetSocketTransform(ContactTracker.CandidateBone);

			FRopeSurfaceAnchor Anchor;
			Anchor.NodeIndex = NodeIndex;
			Anchor.Bone = ContactTracker.CandidateBone;
			Anchor.Mesh = Mesh;
			Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(LatchCandidate->WorldPoint);
			Anchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
			Anchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
			Anchor.StartWorldPosition = Sim.Positions[NodeIndex];
			Anchor.SurfaceOffset = FMath::Max(0.0f, Radius);
			Anchor.RopeDistance = 0.0f;
			Seed.Anchors.Add(Anchor);
		}
	}
	return Seed;
}

void URopeComponent::UpdateWrappedKinematicShape(float DeltaTime)
{
	WrappedSwayTime += DeltaTime;
	WrappedSwayImpulse = FVector::ZeroVector;
}

void URopeComponent::ApplyWrappedMassMask()
{
	TSet<int32> AnchorNodes;

	for (const FRopeSurfaceAnchor& Anchor : WrapController.State.Anchors)
	{
		if (Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
		{
			AnchorNodes.Add(Anchor.NodeIndex);
		}
	}

	for (const FRopeLatchNode& Latch : WrapController.State.Latched)
	{
		if (Sim.InvMass.IsValidIndex(Latch.NodeIndex))
		{
			AnchorNodes.Add(Latch.NodeIndex);
		}
	}

	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		const bool bAnchor = AnchorNodes.Contains(i);
		Sim.InvMass[i] = (bStartPin || bAnchor) ? 0.0f : 1.0f;
	}
}

bool URopeComponent::DebugForceWrap()
{
	/** TODO
일단 그대로 둬도 돼.
왜냐면 DebugForceWrap은 “Wrapped hold를 바로 확인하는 용도”로 유용하거든.
다만 새 구조를 확인하고 싶으면 나중에 이렇게 바꿔.
PendingWrapSeed = Seed;
StartWrappingFromContacting();
return true;
**/
	if (Sim.Num() == 0)
	{
		InitRope();
	}

	// collider는 RopeSimSubsystem이 Tick에서 중앙 수집해 FrameColliders에 채워둔 것을 쓴다(가장 최근 프레임).
	const TArray<IRopeCollider*>& Colliders = FrameColliders;

	// 단일 접촉 node가 이번 frame에 commit되도록 decision gate를 완화한다(DecideWrap은 candidate의
	// 누적 시간이 >= WrapDecisionTime일 때 commit한다; 0은 "지금 즉시"를 의미한다).
	FRopeWrapConfig Relaxed = WrapConfig;
	Relaxed.WrapDecisionTime = 0.0f;
	Relaxed.MinLatchNodes = 1;

	FRopeWrapState Seed;
	if (!WrapController.DecideWrap(Sim, Colliders, Relaxed, 0.0f, Seed))
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] DebugForceWrap: no node in contact (%d collider(s)) — move rope/body to overlap first."),
			*GetName(), Colliders.Num());
		return false; // 접촉 중인 것이 없다; 먼저 rope/capsule을 움직여 겹치게 한다
	}

	WrapController.BeginWrap(Sim, Seed, ResolveWrapTargetMesh());
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] DebugForceWrap -> Wrapped (bone=%s)"),
		*GetName(), *WrapController.State.BoneName.ToString());
	Phase = ERopePhase::Wrapped;
	OnRopeWrapped.Broadcast(WrapController.State.BoneName);
	return true;
}

void URopeComponent::ReleaseWrap()
{
	if (Phase != ERopePhase::Wrapped && Phase != ERopePhase::Contacting && Phase != ERopePhase::Wrapping)
		return;

	FName Bone = NAME_None;

	if (Phase == ERopePhase::Wrapped)
	{
		Bone = WrapController.State.BoneName;
	}
	else if (Phase == ERopePhase::Wrapping)
	{
		Bone = WrappingState.BoneName;
	}
	else
	{
		Bone = ContactTracker.CandidateBone;
	}

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] %s -> Releasing (manual, bone=%s)"),
		*GetName(), PhaseName(Phase), *Bone.ToString());
	WrapController.Release(ERopeReleaseReason::Manual);
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	WrappingState.Reset();
	ContactingElapsed = 0.0f;
	ReleaseCooldown = 0.08f;
	Phase = ERopePhase::Releasing;
	OnRopeReleased.Broadcast(Bone, ERopeReleaseReason::Manual);
}

FPrimitiveSceneProxy* URopeComponent::CreateSceneProxy()
{
	return new FRopeSceneProxy(this);
}

int32 URopeComponent::GetNumMaterials() const
{
	return 1;
}

UMaterialInterface* URopeComponent::GetMaterial(int32 /*ElementIndex*/) const
{
	return RopeMaterial;
}

void URopeComponent::SetMaterial(int32 /*ElementIndex*/, UMaterialInterface* Material)
{
	RopeMaterial = Material;
	MarkRenderStateDirty();
}

FBoxSphereBounds URopeComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	// bounds를 컴포넌트(pinned start)에 anchor하되, rope가 어떻게 변형되든 항상 rope를 포함하는 반지름을
	// 사용한다: chain은 inextensible하므로 어떤 particle도 pin으로부터 RopeLength(+ tube radius)보다 멀리
	// 떨어지지 않는다. 대신 per-frame sim point로부터 bounds를 도출하면 render thread보다 한 frame 뒤처지며;
	// 빠른 캐릭터 모션 중에는 rope가 그 tight box를 앞질러 shadow/main pass에서 cull된다 -> 움직이는 동안
	// shadow가 사라지고 VSM cache는 오래된 afterimage를 유지한다. component transform에 anchor하면 엔진의
	// 추적되는 transform을 통해 bounds가 캐릭터와 함께 움직이므로, lag도 없고 잘못된 culling도 없다.
	const float Reach = RopeLength + Radius + 1.0f;
	return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(Reach), Reach);
}

