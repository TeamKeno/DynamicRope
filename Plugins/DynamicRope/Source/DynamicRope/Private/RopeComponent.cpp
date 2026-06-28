// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
#include "Render/RopeSceneProxy.h"
#include "Debug/RopeDebugDraw.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "ProfilingDebugging/CpuProfilerTrace.h" // TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "Subsystem/RopeSimSubsystem.h"
namespace {
	float SmoothStep(float T)
	{
		T = FMath::Clamp(T, 0.0f, 1.0f);
		return T * T * (3.0f - 2.0f * T);
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
	ReleaseCooldown = 0.0f;
	WrappedSwayImpulse = FVector::ZeroVector;
	WrappedSwayTime = 0.0f;

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
		for (int32 i = 1; i <= LastGuidedNode; ++i)
		{
			if (!GuideTargets.IsValidIndex(i))
			{
				break;
			}

			Sim.Positions[i] = GuideTargets[i];
			Sim.PrevPositions[i] = GuideTargets[i];
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
	const bool bCaptureGuideTargets = RopeDebug::IsFlightStatEnabled();
	const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * GuidedEnd), 1, LastNode);
	TArray<FVector> GuideTargets;
	BuildWhipGuideTargets(T, LastGuidedNode, GuideTargets);
	if (GuideTargets.Num() == 0)
	{
		bWhipSwingActive = false;
		return;
	}

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
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!PrevPositions.IsValidIndex(i) || !Positions.IsValidIndex(i))
		{
			continue;
		}

		bool bFast = IsTailNode(i) || NodeSpeed(i) > Sim.SegmentLength;
		bool bNearBody = IsNearAnyColliderSegment(PrevPositions[i], Positions[i], Colliders);

		if (!bFast && !bNearBody)
			continue;

		// 현재 위치만 보지 않고 이동 경로를 본다.
		// 캡슐은 segment-vs-capsule로 가능.
		// SDF는 path를 몇 개 샘플링하거나 SweepQuery adapter가 필요.
		FRopeContact Contact = SweepOrSampleContact(PrevPositions[i], Positions[i], Colliders);

		if (Contact.bHit)
			OutCandidates.Add(MakeCandidate(i, Contact));
	}
}

void URopeComponent::AddPredictedContactCandidates(TArray<FRopeContactCandidate>& InOutCandidates) const
{
	const float PredictionFrames = FMath::Max(0.0f, WrapConfig.PredictiveContactFrames);
	if (PredictionFrames <= KINDA_SMALL_NUMBER || Sim.Num() == 0)
	{
		return;
	}

	TArray<FVector> PredictedPositions;
	PredictedPositions.SetNumUninitialized(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!Sim.Positions.IsValidIndex(i) || !Sim.PrevPositions.IsValidIndex(i))
		{
			PredictedPositions[i] = Sim.Positions.IsValidIndex(i) ? Sim.Positions[i] : FVector::ZeroVector;
			continue;
		}

		const FVector FrameDisplacement = Sim.Positions[i] - Sim.PrevPositions[i];
		PredictedPositions[i] = Sim.Positions[i] + FrameDisplacement * PredictionFrames;
	}

	TArray<FRopeContactCandidate> PredictedCandidates;
	DetectContactCandidates(Sim.Positions, PredictedPositions, FrameColliders, PredictedCandidates);

	for (FRopeContactCandidate& Candidate : PredictedCandidates)
	{
		if (!Candidate.bValid)
		{
			continue;
		}

		bool bDuplicate = false;
		for (const FRopeContactCandidate& Existing : InOutCandidates)
		{
			if (Existing.NodeIndex == Candidate.NodeIndex && Existing.Bone == Candidate.Bone && Existing.Mesh == Candidate.Mesh)
			{
				bDuplicate = true;
				break;
			}
		}

		if (!bDuplicate)
		{
			InOutCandidates.Add(Candidate);
		}
	}
}

void URopeComponent::EvaluateRelativeMotion(TArray<FRopeContactCandidate>& Candidates) const
{
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
		Phase = ERopePhase::Flight;
		return;
	}

	WrappingState.BoneName = PendingWrapSeed.BoneName;
	WrappingState.Mesh = Mesh;
	WrappingState.Elapsed = 0.0f;
	WrappingState.Duration = 0.16f; // 나중에 WrapConfig로 빼면 됨.
	WrappingState.FirstNode = TNumericLimits<int32>::Max();
	WrappingState.LastNode = INDEX_NONE;

	for (const FRopeLatchNode& Latch : PendingWrapSeed.Latched)
	{
		if (!Sim.Positions.IsValidIndex(Latch.NodeIndex))
		{
			continue;
		}

		const FTransform BoneXform = Mesh->GetSocketTransform(Latch.Bone);
		//const FVector StartWorld = Sim.Positions[Latch.NodeIndex];

		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = Latch.NodeIndex;
		Anchor.Bone = Latch.Bone;
		Anchor.Mesh = Mesh;
		Anchor.StartWorldPosition = Sim.Positions[Latch.NodeIndex];

		// 1차 구조용 임시 anchor.
		// 아직 SDF surface projection을 넣기 전이라 현재 노드 위치를 bone-local로 저장한다.
			// 다음 단계에서 SDF SurfacePoint/Normal 기반으로 바꿀 자리.
		Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Anchor.StartWorldPosition);
		Anchor.LocalNormal = FVector::UpVector;
		Anchor.LocalTangent = FVector::ForwardVector;
		Anchor.SurfaceOffset = 0.0;
		/** TODO
		나중에 SDF 표면점으로 바꿀 때만:
		Anchor.LocalSurfacePosition = SurfaceLocalPosition;
		Anchor.LocalNormal = SurfaceLocalNormal;
		Anchor.SurfaceOffset = Radius;
		**/
		Anchor.RopeDistance = static_cast<float>(Latch.NodeIndex) * Sim.SegmentLength;

		WrappingState.FirstNode = FMath::Min(WrappingState.FirstNode, Latch.NodeIndex);
		WrappingState.LastNode = FMath::Max(WrappingState.LastNode, Latch.NodeIndex);

		WrappingState.Anchors.Add(Anchor);
	}

	if (WrappingState.Anchors.Num() == 0)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Contacting -> Flight (no valid wrapping anchors)"), *GetName());
		WrappingState.Reset();
		ContactTracker.Reset();
		PendingWrapSeed.Reset();
		ContactingElapsed = 0.0f;
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

	TArray<FRopeContactCandidate> Candidates;
	DetectContactCandidates(Sim.PrevPositions, Sim.Positions, FrameColliders, Candidates);
	AddPredictedContactCandidates(Candidates);
	EvaluateRelativeMotion(Candidates);

	const bool bSawWrappingContact = UpdateWrappingAnchorsFromCandidates(Candidates);
	if (bSawWrappingContact)
	{
		WrappingState.LostContactTime = 0.0f;
	}
	else
	{
		WrappingState.LostContactTime += DeltaTime;
	}

	if (WrappingState.LostContactTime > WrapConfig.WrappingContactGraceTime)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Wrapping -> Releasing (contact lost while settling)"), *GetName());
		AbortWrapping(ERopeReleaseReason::Broken);
		Phase = ERopePhase::Releasing;
		return;
	}

	// 지금 단계에서는 일단 anchor 위치로만 부드럽게 이동시킨다.
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
		const FVector NormalWorld = BoneXform.TransformVectorNoScale(Anchor.LocalNormal).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

		const FVector Target = SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;

		Sim.Positions[Anchor.NodeIndex] = Target;
		Sim.PrevPositions[Anchor.NodeIndex] = Target;
		Sim.InvMass[Anchor.NodeIndex] = 0.0f;
	}

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

	const bool bHasEnoughAnchors = WrappingState.Anchors.Num() >= FMath::Max(1, WrapConfig.MinLatchNodes);
	const bool bStable = WrappingState.StableTime >= WrapConfig.WrappingStableTime;
	const bool bTimedOutWithAnchors =
		WrapConfig.WrappingMaxSettleTime > 0.0f &&
		WrappingState.Elapsed >= WrapConfig.WrappingMaxSettleTime;

	if (bHasEnoughAnchors && (bStable || bTimedOutWithAnchors))
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

bool URopeComponent::UpdateWrappingAnchorsFromCandidates(const TArray<FRopeContactCandidate>& Candidates)
{
	bool bSawWrappingContact = false;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid ||
			Candidate.Bone != WrappingState.BoneName ||
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

	// Flight: 솔브 후 이동 경로 기반 접촉 후보 감지 → 캡처. UObject·이벤트라 GT에서.
	if (Phase == ERopePhase::Flight)
	{
		TArray<FRopeContactCandidate> Candidates;
		TArray<RopeDebug::FRopeFlightNodeDebug> FlightNodeDebug;
		const bool bDrawFlightStat = RopeDebug::IsFlightStatEnabled();
		if (bDrawFlightStat)
		{
			for (int32 i = 0; i < Sim.Num(); ++i)
			{
				if (!Sim.PrevPositions.IsValidIndex(i) || !Sim.Positions.IsValidIndex(i))
				{
					continue;
				}

				RopeDebug::FRopeFlightNodeDebug NodeDebug;
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

		DetectContactCandidates(Sim.PrevPositions, Sim.Positions, FrameColliders, Candidates);
		AddPredictedContactCandidates(Candidates);
		EvaluateRelativeMotion(Candidates);

		FRopeContactTracker FlightDebugTracker;
		FlightDebugTracker.Update(Candidates, 0.0f);
		const bool bShouldCapture = ShouldCapture(Candidates);

		if (bShouldCapture)
		{
			BuildContactingState(Candidates);
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Flight -> Contacting (bone=%s, %d node(s))"),
				*GetName(), *ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num());
			Phase = ERopePhase::Contacting;
			OnRopeCaptured.Broadcast(ContactTracker.CandidateBone);
		}

		if (bDrawFlightStat)
		{
			const FString RopeName = GetOwner()
				? FString::Printf(TEXT("%s.%s"), *GetOwner()->GetName(), *GetName())
				: GetName();
			RopeDebug::DrawFlight(GetWorld(), static_cast<uint64>(GetUniqueID()) + 0x10000000ull, RopeName, Sim,
				ERopePhase::Flight, bSolveThisFrame, FrameColliders.Num(), FlightNodeDebug, Candidates,
				bShouldCapture ? ContactTracker : FlightDebugTracker, WrapConfig, bShouldCapture);
			RopeDebug::DrawFlightWhipGuide(GetWorld(), Sim, DebugWhipGuideNodeIndices, DebugWhipGuideTargets,
				FMath::Clamp(WhipGuidedLength, 0.05f, 0.95f), DebugWhipGuideTargets.Num() > 0);
		}
	}

	// 새 centerline을 render proxy로 push하고 bounds를 갱신한다.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_MarkRenderDirty);
		MarkRenderDynamicDataDirty();
		MarkRenderTransformDirty();
	}

	RopeDebug::DrawCenterline(GetWorld(), Sim, Phase, WrapController.State, bDrawDebugCenterline);
	if (Phase == ERopePhase::Wrapped)
	{
		const FString RopeName = GetOwner()
			? FString::Printf(TEXT("%s.%s"), *GetOwner()->GetName(), *GetName())
			: GetName();
		RopeDebug::DrawWrappedTable(GetWorld(), static_cast<uint64>(GetUniqueID()) + 0x20000000ull,
			RopeName, Sim, WrapController.State);
	}
}

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
	FBox SegmentBounds(ForceInit);
	SegmentBounds += PrevPosition;
	SegmentBounds += Position;
	SegmentBounds = SegmentBounds.ExpandBy(WrapConfig.ContactRadius + Radius + 5.0f);

	for (const IRopeCollider* Collider : Colliders)
	{
		if (Collider && SegmentBounds.Intersect(Collider->GetWorldBounds().ExpandBy(WrapConfig.ContactRadius + Radius)))
		{
			return true;
		}
	}
	return false;
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
	PendingWrapSeed = BuildWrapSeedFromContactingState();
}

void URopeComponent::AdvanceWrappingMotion(float DeltaTime)
{
	ContactingElapsed += DeltaTime;
	if (PendingWrapSeed.Latched.Num() == 0)
	{
		PendingWrapSeed = BuildWrapSeedFromContactingState();
	}
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

FRopeWrapState URopeComponent::BuildWrapSeedFromContactingState() const
{
	FRopeWrapState Seed;
	Seed.BoneName = ContactTracker.CandidateBone;
	Seed.Mesh = ContactTracker.CandidateMesh;
	for (int32 NodeIndex : ContactTracker.CandidateNodes)
	{
		if (!Sim.Positions.IsValidIndex(NodeIndex))
		{
			continue;
		}

		FRopeLatchNode Latch;
		Latch.NodeIndex = NodeIndex;
		Latch.Bone = ContactTracker.CandidateBone;
		Seed.Latched.Add(Latch);
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

