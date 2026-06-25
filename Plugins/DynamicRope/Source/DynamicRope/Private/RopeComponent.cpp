// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
#include "Collision/RopeCollider.h"
#include "Collision/RopeColliderProvider.h"
#include "Render/RopeSceneProxy.h"
#include "Debug/RopeDebugDraw.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "ProfilingDebugging/CpuProfilerTrace.h" // TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "EngineUtils.h" // TActorIterator (world-wide provider collection)
#include "Subsystem/RopeSimSubsystem.h"
namespace {
	float SmoothStep(float T)
	{
		T = FMath::Clamp(T, 0.0f, 1.0f);
		return T * T * (3.0f - 2.0f * T);
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
}

void URopeComponent::GatherFrameColliders(TArray<IRopeCollider*>& OutColliders) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_GatherColliders);
	OutColliders.Reset();
	FBox RopeBounds(ForceInit);
	for (const FVector& P : Sim.Positions)
	{
		RopeBounds += P;
	}
	// contact reach만큼 확장한다: 거의 직선인 rope를 감싼 tight box는 두께가 ~0이라, 실제로는 contact
	// 거리 안에 있는 capsule을 잘못 cull해 버린다. narrow phase와 맞춘다
	// (node ContactRadius; capsule 자신의 반지름은 이미 GetWorldBounds()에 포함되어 있다).
	if (RopeBounds.IsValid)
	{
		RopeBounds = RopeBounds.ExpandBy(Radius + WrapConfig.ContactRadius + 5.0f);
	}

	RopeDebug::DrawBounds(GetWorld(), RopeBounds, bDrawDebugCenterline);

	for (const TScriptInterface<IRopeColliderProvider>& Provider : ColliderProviders)
	{
		if (IRopeColliderProvider* Raw = Provider.GetInterface())
		{
			Raw->GatherColliders(RopeBounds, OutColliders);
		}
	}

	RopeDebug::DrawStats(GetWorld(), reinterpret_cast<uint64>(this), Phase,
		ColliderProviders.Num(), OutColliders.Num(), WrapController.State.BoneName, bDrawDebugCenterline);
}

void URopeComponent::EnsureColliderProviders()
{
	if (ColliderProviders.Num() > 0)
	{
		return;
	}

	auto AddFrom = [this](AActor* Actor)
	{
		if (!Actor)
		{
			return;
		}
		for (UActorComponent* Comp : Actor->GetComponentsByInterface(URopeColliderProvider::StaticClass()))
		{
			ColliderProviders.AddUnique(TScriptInterface<IRopeColliderProvider>(Comp));
		}
	};

	// Cross-actor: rope가 한 actor에 고정되어 있지만 *다른* body를 잡아야 할 때, provider는 그 다른
	// actor 위에 존재한다. 명시적 리스트가 설정되어 있으면 그것을 사용하고, 그렇지 않으면 우리
	// 자신의 owner로 기본 설정한다(same-actor 경우).
	if (bGatherProvidersFromWholeWorld)
	{
		if (UWorld* World = GetWorld())
		{
			for (TActorIterator<AActor> It(World); It; ++It)
			{
				AddFrom(*It);
			}
		}
		return;
	}

	if (ColliderSourceActors.Num() > 0)
	{
		for (AActor* Actor : ColliderSourceActors)
		{
			AddFrom(Actor);
		}
	}
	else
	{
		AddFrom(GetOwner());
	}

	// 명시적으로 지정된 wrap-target mesh의 owner만 따라간다; 여기서는 절대 auto-resolve 하지 않는다(그러면
	// 우리 자신의 owner를 다시 추가하게 되어 rope가 자기 자신에게 latch하도록 만든다).
	if (WrapTargetMesh)
	{
		AddFrom(WrapTargetMesh->GetOwner());
	}
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

	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	ContactingElapsed = 0.0f;

	Phase = ERopePhase::Flight;
}

void URopeComponent::ApplyWhipSwing(float DeltaTime)
{
	WhipElapsed += DeltaTime;

	const float SafeDuration = FMath::Max(WhipDuration, UE_SMALL_NUMBER);
	if (WhipElapsed >= SafeDuration)
	{
		bWhipSwingActive = false;
	}

	const FVector Forward = WhipAimDir.GetSafeNormal();
	FVector Up = FVector::UpVector;
	if (FMath::Abs(FVector::DotProduct(Forward, Up)) > 0.92f)
	{
		Up = GetUpVector();
	}
	const FVector Side = FVector::CrossProduct(Up, Forward).GetSafeNormal();
	Up = FVector::CrossProduct(Forward, Side).GetSafeNormal();

	// 손이 호를 그리며 휘두르는 방향.
	// 초반엔 아래/뒤쪽, 후반엔 앞쪽으로 넘어오는 느낌.
	const FVector HandPos = GetComponentLocation();
	const int32 LastNode = Sim.Num() - 1;
	if (LastNode <= 0)
	{
		return;
	}

	const float GuidedLength = FMath::Clamp(WhipGuidedLength, 0.1f, 0.95f);
	const float FollowAlpha = 1.0f - FMath::Exp(-FMath::Max(0.0f, WhipFollowRate) * DeltaTime);
	const float SafeTravelTime = FMath::Max(WhipWaveTravelTime, UE_SMALL_NUMBER);

	for (int32 i = 1; i <= LastNode; ++i)
	{
		if (!Sim.Positions.IsValidIndex(i) || !Sim.PrevPositions.IsValidIndex(i) || !Sim.InvMass.IsValidIndex(i) || Sim.InvMass[i] <= 0.0f)
		{
			continue;
		}

		const float RopeAlpha = static_cast<float>(i) / static_cast<float>(LastNode);
		if (RopeAlpha >= GuidedLength)
		{
			continue;
		}

		const float GuideAlpha = RopeAlpha / GuidedLength;
		const float GuideWeight = FMath::Clamp(1.0f - SmoothStep((GuideAlpha - 0.62f) / 0.38f), 0.0f, 1.0f);
		if (GuideWeight <= 0.0f)
		{
			continue;
		}

		const float Delay = GuideAlpha * SafeTravelTime;
		const float LocalT = FMath::Clamp((WhipElapsed - Delay) / FMath::Max(SafeDuration - Delay, UE_SMALL_NUMBER), 0.0f, 1.0f);
		const float CurveT = SmoothStep(LocalT);
		const float RestDistance = RopeAlpha * Sim.RopeLength;
		const float ArcProfile = FMath::Sin(FMath::Clamp(GuideAlpha, 0.0f, 1.0f) * PI);

		const FVector BackPose =
			HandPos
			- Forward * (RestDistance * 0.85f)
			- Up * (RestDistance * 0.35f);

		const FVector ForwardPose =
			HandPos
			+ Forward * (RestDistance * 0.95f)
			+ Up * (ArcProfile * WhipArcHeight * 0.20f);

		const FVector ArcOffset =
			Up * (ArcProfile * FMath::Sin(CurveT * PI) * WhipArcHeight)
			+ Side * (ArcProfile * FMath::Sin(CurveT * PI) * WhipSideOffset);

		const FVector TargetPosition = FMath::Lerp(BackPose, ForwardPose, CurveT) + ArcOffset;
		const FVector Correction = TargetPosition - Sim.Positions[i];
		const FVector Delta = Correction * GuideWeight * FollowAlpha;

		Sim.Positions[i] += Delta;
		Sim.PrevPositions[i] += Delta * 0.35f;
		// 손 근처 노드에 강하게, 조금 떨어진 노드엔 약하게.
	}
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

void URopeComponent::SimulateFrame(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Simulate);

	EnsureRopeInitialized();

	// pinned-start target을 전진시킨다; solver가 substep에 걸쳐 Prev->Target을 sweep하므로 빠른
	// 캐릭터 이동이 chain을 홱 잡아당겨(폭주시켜) 버리지 않는다.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	EnsureColliderProviders();

	TArray<IRopeCollider*> Colliders;
	GatherFrameColliders(Colliders);

	switch (Phase)
	{
	case ERopePhase::Free:        // 손에서 늘어뜨려진 채 캐릭터를 따라간다
	{
		//TArray<IRopeCollider*> WorldColliders;
		//GatherWorldColliders(WorldColliders);

		Solver.Step(Sim, SolverConfig, /*optional*/ Colliders, DeltaTime);

		break;
	}
	case ERopePhase::Flight:
	{
		if (bWhipSwingActive)
		{
			ApplyWhipSwing(DeltaTime);
		}

		Solver.Step(Sim, SolverConfig, /*optional*/ Colliders, DeltaTime);

		// 1. 빠른 노드/끝단 노드에 대해 이동 경로 기반 후보 감지
		TArray<FRopeContactCandidate> Candidates;
		DetectContactCandidates(Sim.PrevPositions, Sim.Positions, Colliders, Candidates);
		EvaluateRelativeMotion(Candidates);

		if (ShouldCapture(Candidates))
		{
			BuildContactingState(Candidates);
			Phase = ERopePhase::Contacting;
			OnRopeCaptured.Broadcast(ContactTracker.CandidateBone);
		}

		break;
	}
	case ERopePhase::Contacting:
	{

		// 감기 진행 상태
	// 여기서 감김 노드/위치/방향을 점진적으로 보정
		AdvanceWrappingMotion(DeltaTime);

		if (ShouldDismissContacting()) 
		{
			Phase = ERopePhase::Flight;
			break;
		}

		if (ShouldFinishWrapping())
		{
			const FRopeWrapState Seed = BuildWrapSeedFromContactingState();
			WrapController.BeginWrap(Sim, Seed, ResolveWrapTargetMesh());
			Phase = ERopePhase::Wrapped;
			OnRopeWrapped.Broadcast(Seed.BoneName);
		}
		break;
	}
	case ERopePhase::Wrapped:
	{
		// logic이 latch된 node들을 소유한다(skinned bone에 올라탄다); solver는 여전히 free span을
		// settle하여 rope가 늘어지고 node 0에서 손에 붙어 있도록 유지한다.
		// 1. 감긴 노드는 bone-local 위치를 따라감
		WrapController.Hold(Sim, ResolveWrapTargetMesh(), DeltaTime);

		UpdateWrappedKinematicShape(DeltaTime); // 선택: 찰랑임 연출만

		break;
	}
	case ERopePhase::Releasing:
		// 모든 node를 solver에 다시 넘긴다(hand pin만 유지), 그런 다음 free simulation을 재개한다.
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.InvMass[i] = (i == 0 && Sim.bStartPinned) ? 0.0f : 1.0f;

			// 2. 이전 고정점 때문에 튀지 않게 PrevPositions 보정
			Sim.PrevPositions[i] = Sim.Positions[i];
		}
		// 3. 짧은 쿨다운
		ReleaseCooldown -= DeltaTime;	//static으로 할까 고민 중

		if (ReleaseCooldown <= 0)
		{
			Phase = ERopePhase::Free;
		}


		break;
	default:
		break;
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

	EnsureRopeInitialized();

	switch (Phase)		
	{
	case ERopePhase::Free:
	{
		StartFreshThrow(AimDir);
		break;
	}
	case ERopePhase::Flight:
	{
		StartFreshThrow(AimDir);
		break;
	}
	case ERopePhase::Contacting:
	{
		//아직 감긴 건 아니므로 후보만 버리고 새 throw를 시작한다.
		ContactTracker.Reset();
		PendingWrapSeed.Reset();
		ContactingElapsed = 0.0f;

		StartFreshThrow(AimDir);

		break;
	}
	case ERopePhase::Wrapped:
	{
		//감긴 상태 유지 latched node는 건드리징 않음
		ThrowFreeSpanWhileWrapped(AimDir);

		break;
	}
	case ERopePhase::Releasing:
		// 무시하거나 pending throw 저장
		break;
	default:
		break;
	}

	// Scaffold launch: Verlet prev-position offset을 통해 free tip에 초기 속도를 부여한다.
	//const int32 Last = Sim.Num() - 1;
	//if (Last > 0)
	//{
	//	Sim.InvMass[Last] = 1.0f;
	//	const FVector Velocity = AimDir.GetSafeNormal() * ThrowParams.ThrowSpeed * (1.0f / 60.0f);
	//	Sim.PrevPositions[Last] = Sim.Positions[Last] - Velocity;
	//}
}

void URopeComponent::ThrowFreeSpanWhileWrapped(const FVector& AimDir)
{
	FVector Dir = AimDir.GetSafeNormal();
	if (Dir.IsNearlyZero())
	{
		Dir = GetForwardVector();
	}

	WrappedSwayImpulse = Dir * ThrowParams.ThrowSpeed * (1.0f / 60.0f);
	WrappedSwayTime = 0.0f;
}

void URopeComponent::GatherWorldColliders(TArray<IRopeCollider*>& OutColliders) const
{
	// Reserved for floor/world providers. Keeping this separate prevents idle rope from latching to character limbs.
	OutColliders.Reset();
}

float URopeComponent::TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const
{
	const float Denom = FMath::Max(1, LastNode - FirstTailNode);
	const float Alpha = static_cast<float>(NodeIndex - FirstTailNode) / Denom;
	return FMath::Lerp(0.35f, 1.0f, Alpha);
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
	WrappedSwayImpulse *= FMath::Exp(-DeltaTime * 6.0f);
	if (WrappedSwayImpulse.IsNearlyZero())
	{
		return;
	}

	const FVector Offset = WrappedSwayImpulse * FMath::Sin(WrappedSwayTime * 18.0f) * 0.05f;
	for (int32 i = 1; i < Sim.Num(); ++i)
	{
		if (Sim.InvMass.IsValidIndex(i) && Sim.InvMass[i] > 0.0f)
		{
			Sim.Positions[i] += Offset;
			Sim.PrevPositions[i] = Sim.Positions[i];
		}
	}
}

bool URopeComponent::DebugForceWrap()
{
	if (Sim.Num() == 0)
	{
		InitRope();
	}

	EnsureColliderProviders();
	TArray<IRopeCollider*> Colliders;
	GatherFrameColliders(Colliders);

	// 단일 접촉 node가 이번 frame에 commit되도록 decision gate를 완화한다(DecideWrap은 candidate의
	// 누적 시간이 >= WrapDecisionTime일 때 commit한다; 0은 "지금 즉시"를 의미한다).
	FRopeWrapConfig Relaxed = WrapConfig;
	Relaxed.WrapDecisionTime = 0.0f;
	Relaxed.MinLatchNodes = 1;

	FRopeWrapState Seed;
	if (!WrapController.DecideWrap(Sim, Colliders, Relaxed, 0.0f, Seed))
	{
		return false; // 접촉 중인 것이 없다; 먼저 rope/capsule을 움직여 겹치게 한다
	}

	WrapController.BeginWrap(Sim, Seed, ResolveWrapTargetMesh());
	Phase = ERopePhase::Wrapped;
	OnRopeWrapped.Broadcast(WrapController.State.BoneName);
	return true;
}

void URopeComponent::ReleaseWrap()
{
	if (Phase != ERopePhase::Wrapped && Phase != ERopePhase::Contacting)
		return;

	const FName Bone = (Phase == ERopePhase::Wrapped) ? WrapController.State.BoneName : ContactTracker.CandidateBone;
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

