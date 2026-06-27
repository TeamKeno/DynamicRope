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

	WhipElapsed = 0.0f;
	bWhipSwingActive = true;

	++SimGeneration; // throw로 tail 위치를 재설정 → GPU 상주 버퍼 재시드(M5).

	if (WrapController.IsActive())
	{
		WrapController.Release(ERopeReleaseReason::Manual);
	}
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
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

		// Prime the guided span behind the hand so the visible throw reads back-to-front.
		FVector Up = FVector::UpVector;
		if (FMath::Abs(FVector::DotProduct(WhipAimDir, Up)) > 0.96f)
		{
			Up = GetRightVector().GetSafeNormal();
		}
		FVector Side = FVector::CrossProduct(Up, WhipAimDir).GetSafeNormal();
		if (Side.IsNearlyZero())
		{
			Side = GetRightVector().GetSafeNormal();
		}
		Up = FVector::CrossProduct(WhipAimDir, Side).GetSafeNormal();

		const int32 LastGuidedNode = FMath::Clamp(FMath::CeilToInt(static_cast<float>(LastNode) * WhipGuidedLength), 1, LastNode);
		for (int32 i = 1; i <= LastGuidedNode; ++i)
		{
			const float S = static_cast<float>(i) / static_cast<float>(LastNode);
			const float Arc = FMath::Sin(S * PI);
			const FVector Primed =
				Start
				- WhipAimDir * (S * Sim.RopeLength * 0.85f)
				+ Up * (Arc * WhipArcHeight * 0.25f)
				- Side * (Arc * WhipSideOffset);
			Sim.Positions[i] = Primed;
			Sim.PrevPositions[i] = Primed;
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

void URopeComponent::ApplyWhipSwing(float DeltaTime)
{
	if (Sim.Num() < 3)
	{
		bWhipSwingActive = false;
		return;
	}

	WhipElapsed += DeltaTime;
	const float Duration = FMath::Max(WhipDuration, KINDA_SMALL_NUMBER);
	const float T = FMath::Clamp(WhipElapsed / Duration, 0.0f, 1.0f);
	const float EaseT = SmoothStep(T);

	const FVector Forward = WhipAimDir.GetSafeNormal();
	if (Forward.IsNearlyZero())
	{
		bWhipSwingActive = false;
		return;
	}

	FVector Up = FVector::UpVector;
	if (FMath::Abs(FVector::DotProduct(Forward, Up)) > 0.96f)
	{
		Up = GetRightVector().GetSafeNormal();
	}

	FVector Side = FVector::CrossProduct(Up, Forward).GetSafeNormal();
	if (Side.IsNearlyZero())
	{
		Side = GetRightVector().GetSafeNormal();
	}
	Up = FVector::CrossProduct(Forward, Side).GetSafeNormal();

	const FVector HandPos = GetComponentLocation();
	const int32 LastNode = Sim.Num() - 1;
	const float GuidedEnd = FMath::Clamp(WhipGuidedLength, 0.05f, 0.95f);
	const float FollowAlpha = FMath::Clamp(1.0f - FMath::Exp(-FMath::Max(0.0f, WhipFollowRate) * DeltaTime), 0.0f, 0.8f);
	const float WaveTravel = FMath::Max(WhipWaveTravelTime / Duration, 0.05f);
	const float WaveHead = FMath::Clamp(EaseT / WaveTravel, 0.0f, 1.35f);
	const float ThrowReach = FMath::Max(Sim.RopeLength, RopeLength) * (0.2f + 0.8f * EaseT);

	for (int32 i = 1; i <= LastNode; ++i)
	{
		if (Sim.InvMass.IsValidIndex(i) && Sim.InvMass[i] <= 0.0f)
		{
			continue;
		}

		const float S = static_cast<float>(i) / static_cast<float>(LastNode);
		if (S > GuidedEnd)
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

		const float RestDistance = S * Sim.RopeLength;
		const float Behind = (1.0f - EaseT) * RestDistance * 0.7f;
		const float Ahead = ThrowReach * S * EaseT;
		const float ArcEnvelope = FMath::Sin(S * PI);
		const float TravelingWave = FMath::Sin(FMath::Clamp((S - WaveHead + 0.35f) / 0.7f, 0.0f, 1.0f) * PI);
		const float Lift = ArcEnvelope * TravelingWave * FMath::Sin(T * PI) * WhipArcHeight;
		const float SideSweep = ArcEnvelope * FMath::Sin((S - EaseT) * PI) * WhipSideOffset;

		const FVector Target =
			HandPos
			- Forward * Behind
			+ Forward * Ahead
			+ Up * Lift
			+ Side * SideSweep;

		const FVector Delta = Target - Sim.Positions[i];
		Sim.Positions[i] += Delta * FollowAlpha * GuideWeight;
	}

	bWhipSwingActive = WhipElapsed < WhipDuration;
}

void URopeComponent::DetectContactCandidates(const TArray<FVector>& PrevPositions, const TArray<FVector>& Positions,
	const TArray<IRopeCollider*>& Colliders, TArray<FRopeContactCandidate>& OutCandidates) const
{
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
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
	{
		// 감기 진행 상태 — 감김 노드/위치/방향을 점진적으로 보정(솔브 없음, 로직 구동).
		AdvanceWrappingMotion(DeltaTime);

		if (ShouldDismissContacting())
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Contacting -> Flight (contact lost before wrap)"), *GetName());
			Phase = ERopePhase::Flight;
			break;
		}

		if (ShouldFinishWrapping())
		{
			const FRopeWrapState Seed = BuildWrapSeedFromContactingState();
			WrapController.BeginWrap(Sim, Seed, ResolveWrapTargetMesh());
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Contacting -> Wrapped (bone=%s, %d latched node(s))"),
				*GetName(), *Seed.BoneName.ToString(), Seed.Latched.Num());
			Phase = ERopePhase::Wrapped;
			OnRopeWrapped.Broadcast(Seed.BoneName);
		}
		break;
	}
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
			ContactingElapsed = 0.0f;
			ReleaseCooldown = 0.08f;
			Phase = ERopePhase::Releasing;
			OnRopeReleased.Broadcast(Bone, ERopeReleaseReason::Broken);
			break;
		}
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
	if (!bSolveThisFrame || bWhipSwingActive)
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
		DetectContactCandidates(Sim.PrevPositions, Sim.Positions, FrameColliders, Candidates);
		EvaluateRelativeMotion(Candidates);

		if (ShouldCapture(Candidates))
		{
			BuildContactingState(Candidates);
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Flight -> Contacting (bone=%s, %d node(s))"),
				*GetName(), *ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num());
			Phase = ERopePhase::Contacting;
			OnRopeCaptured.Broadcast(ContactTracker.CandidateBone);
		}
	}

	// 새 centerline을 render proxy로 push하고 bounds를 갱신한다.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_MarkRenderDirty);
		MarkRenderDynamicDataDirty();
		MarkRenderTransformDirty();
	}

	RopeDebug::DrawCenterline(GetWorld(), Sim, Phase, WrapController.State, bDrawDebugCenterline);
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

bool URopeComponent::ShouldFinishWrapping() const
{
	return ContactingElapsed >= WrapConfig.WrapDecisionTime && PendingWrapSeed.Latched.Num() > 0;
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

bool URopeComponent::DebugForceWrap()
{
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
	if (Phase != ERopePhase::Wrapped && Phase != ERopePhase::Contacting)
		return;

	const FName Bone = (Phase == ERopePhase::Wrapped) ? WrapController.State.BoneName : ContactTracker.CandidateBone;
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] %s -> Releasing (manual, bone=%s)"),
		*GetName(), PhaseName(Phase), *Bone.ToString());
	WrapController.Release(ERopeReleaseReason::Manual);
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
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

