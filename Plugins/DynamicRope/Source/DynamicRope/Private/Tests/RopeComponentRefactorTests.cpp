// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeComponent의 virtual bridge 수명과 GuidedThrow 공통 진입 회귀 테스트.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeComponent.h"
#include "Components/SceneComponent.h"
#include "RopeTestHelpers.h"

struct FRopeComponentRefactorTestSeam
{
	static void ExtractVirtualBridgeRuns(FRopeWrappingPhase& Phase)
	{
		Phase.CollectCompletedVirtualBridgeRuns();
	}

	static void ConfigureSim(URopeComponent& Rope, int32 NumNodes, float RopeLength)
	{
		Rope.Sim = RopeTest::MakeStraightRope(NumNodes, RopeLength);
	}

	static void SyncVirtualBridges(URopeComponent& Rope,
		const TArray<FRopeVirtualBridgeRun>& Runs,
		const TArray<FRopeSurfaceAnchor>& Anchors,
		float FrontDistance)
	{
		Rope.UpdateWrappingKinematicVirtualBridges(Runs, Anchors, FrontDistance);
	}

	static bool FinalizeVirtualBridges(URopeComponent& Rope,
		const TArray<FRopeVirtualBridgeRun>& Runs,
		const TArray<FRopeSurfaceAnchor>& Anchors)
	{
		return Rope.FinalizeKinematicVirtualBridges(Runs, Anchors);
	}

	static TArray<URopeComponent::FKinematicVirtualBridge>& GetVirtualBridges(URopeComponent& Rope)
	{
		return Rope.KinematicVirtualBridges;
	}

	static void StartFreeGuidedThrow(URopeComponent& Rope,
		const FRopeThrowContext& Context, const FVector& Endpoint)
	{
		Rope.ReleaseCooldown = 3.0f;
		Rope.StartFreeGuidedThrow(Context, Endpoint);
	}

	static const FRopeGuidedThrowState& GetGuidedThrowState(const URopeComponent& Rope)
	{
		return Rope.GuidedThrowState;
	}

	static const FRopeSimState& GetSim(const URopeComponent& Rope)
	{
		return Rope.Sim;
	}

	static float GetReleaseCooldown(const URopeComponent& Rope)
	{
		return Rope.ReleaseCooldown;
	}

	static void SetPhase(URopeComponent& Rope, ERopePhase Phase)
	{
		Rope.Phase = Phase;
	}

	static void ForceNonStretchThisFrame(URopeComponent& Rope, bool bForce)
	{
		Rope.SimFrame.bForceNonStretchThisFrame = bForce;
	}

	static void ConfigureAssistedAimLock(URopeComponent& Rope,
		const USceneComponent* TargetMesh, FName TargetBone)
	{
		ConfigureAimLock(Rope, TargetMesh, TargetBone, ERopeWrapResolveMode::AssistedJudged);
	}

	static void ConfigureAimLock(URopeComponent& Rope, const USceneComponent* TargetMesh,
		FName TargetBone, ERopeWrapResolveMode Mode)
	{
		Rope.ResolveMode = Mode;
		Rope.Phase = ERopePhase::Flight;
		FRopeThrowContext Context;
		Context.bHasAimGuideHit = true;
		Context.AimGuideMesh = TargetMesh;
		Context.AimGuideBone = TargetBone;
		Rope.AimTargeting.SetWrapTargetLock(Context);
	}

	static TArray<IRopeCollider*>& GetFrameColliders(URopeComponent& Rope)
	{
		return Rope.SimFrame.FrameColliders;
	}

	static TArray<IRopeCollider*>& GetAimFrameColliders(URopeComponent& Rope)
	{
		return Rope.SimFrame.AimFrameColliders;
	}

	static void FilterFrameCollidersForAimWrapTarget(URopeComponent& Rope)
	{
		Rope.FilterFrameCollidersForAimWrapTarget();
	}

	static const FBox& GetLockedTargetColliderQueryBounds(const URopeComponent& Rope)
	{
		return Rope.SimFrame.LockedTargetColliderQueryBounds;
	}

	static void ResetStateForNewThrow(URopeComponent& Rope)
	{
		Rope.ResetStateForNewThrow();
	}

	static bool ApplyGpuFlightCapture(URopeComponent& Rope,
		const TArray<FRopeContactCandidate>& Candidates, FRopeFlightCaptureEvaluation& Evaluation)
	{
		Rope.SimFrame.bGpuSteppedThisFrame = true;
		return Rope.ApplyFlightCaptureEvaluation(1.0f / 60.0f, Candidates, Evaluation);
	}

	static bool IsGpuCaptureHandoffPending(const URopeComponent& Rope)
	{
		return Rope.bPendingGpuCaptureHandoff;
	}

	static float BuildContactingDwell(URopeComponent& Rope,
		const TArray<FRopeContactCandidate>& Candidates, float DeltaTime)
	{
		FRopeContactTracker Tracker;
		Tracker.Update(Candidates, 0.0f, Rope.AimTargeting.GetLockedTargetMesh(),
			Rope.AimTargeting.GetLockedTargetBone(), true);
		Rope.BuildContactingState(MoveTemp(Tracker), Candidates, DeltaTime);
		return Rope.ContactTracker.DwellTime;
	}
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeVirtualBridgeSingleLifecycleTest,
	"DynamicRope.Component.Wrapping.VirtualBridgeSingleLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeVirtualBridgeSingleLifecycleTest::RunTest(const FString& Parameters)
{
	FRopeWrappingPhase Phase;
	Phase.State.LatchAnchor.NodeIndex = 2;
	Phase.State.Path.SetNum(4);
	Phase.State.Path[1].bVirtual = true;
	Phase.State.Path[2].bVirtual = true;
	FRopeComponentRefactorTestSeam::ExtractVirtualBridgeRuns(Phase);
	FRopeComponentRefactorTestSeam::ExtractVirtualBridgeRuns(Phase);

	TestEqual(TEXT("one bounded virtual run is emitted"), Phase.State.VirtualBridgeRuns.Num(), 1);
	if (Phase.State.VirtualBridgeRuns.Num() != 1)
	{
		return false;
	}

	const FRopeVirtualBridgeRun& Run = Phase.State.VirtualBridgeRuns[0];
	TestEqual(TEXT("left boundary node"), Run.LeftNodeIndex, 2);
	TestEqual(TEXT("right boundary node"), Run.RightNodeIndex, 5);
	TestEqual(TEXT("two virtual nodes"), Run.VirtualNodeIndices.Num(), 2);
	TestTrue(TEXT("virtual node indices preserve path order"),
		Run.VirtualNodeIndices.Num() == 2 &&
		Run.VirtualNodeIndices[0] == 3 && Run.VirtualNodeIndices[1] == 4);

	USceneComponent* Mesh = NewObject<USceneComponent>();
	FRopeSurfaceAnchor Left;
	Left.NodeIndex = 2;
	Left.Bone = FName("arm");
	Left.Mesh = Mesh;
	Left.RopeDistance = 0.0f;
	FRopeSurfaceAnchor Right = Left;
	Right.NodeIndex = 5;
	Right.RopeDistance = 30.0f;
	const TArray<FRopeSurfaceAnchor> WrappingAnchors = { Left, Right };

	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeComponentRefactorTestSeam::ConfigureSim(*Rope, 8, 70.0f);
	FRopeComponentRefactorTestSeam::SyncVirtualBridges(
		*Rope, Phase.State.VirtualBridgeRuns, WrappingAnchors, /*FrontDistance*/ 100.0f);

	auto& Bridges = FRopeComponentRefactorTestSeam::GetVirtualBridges(*Rope);
	TestEqual(TEXT("Wrapping creates the bridge once"), Bridges.Num(), 1);
	if (Bridges.Num() != 1)
	{
		return false;
	}
	TestTrue(TEXT("front activates the registered bridge"), Bridges[0].bActive);
	TestTrue(TEXT("rest span comes from boundary node distance"),
		FMath::IsNearlyEqual(Bridges[0].RestSpanLength, 30.0f));

	// 재생성 여부를 식별하는 sentinel. Commit finalization이 기존 bridge를 유지하면 이 값이 보존된다.
	Bridges[0].bLoggedStretchWarning = true;
	Left.LocalSurfacePosition = FVector(10.0f, 0.0f, 0.0f);
	Right.LocalSurfacePosition = FVector(20.0f, 0.0f, 0.0f);
	Right.RopeDistance = 35.0f;
	const TArray<FRopeSurfaceAnchor> CommitAnchors = { Left, Right };

	TestTrue(TEXT("Commit anchors finalize the existing bridge"),
		FRopeComponentRefactorTestSeam::FinalizeVirtualBridges(
			*Rope, Phase.State.VirtualBridgeRuns, CommitAnchors));
	TestEqual(TEXT("Commit does not create a second bridge"), Bridges.Num(), 1);
	TestTrue(TEXT("existing bridge identity state survives finalization"),
		Bridges[0].bLoggedStretchWarning);
	TestTrue(TEXT("final binding is refreshed from commit anchors"),
		Bridges[0].RightAnchor.LocalSurfacePosition.Equals(
			FVector(20.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));
	TestTrue(TEXT("final bridge remains active"), Bridges[0].bActive);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFreeGuidedThrowCommonEntryTest,
	"DynamicRope.Component.GuidedThrow.FreeUsesCommonEntry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFreeGuidedThrowCommonEntryTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeComponentRefactorTestSeam::ConfigureSim(*Rope, 4, 60.0f);

	FRopeThrowContext Context;
	Context.Origin = FVector(10.0f, 20.0f, 30.0f);
	const FVector Endpoint(110.0f, 20.0f, 30.0f);
	FRopeComponentRefactorTestSeam::StartFreeGuidedThrow(*Rope, Context, Endpoint);

	const FRopeGuidedThrowState& State =
		FRopeComponentRefactorTestSeam::GetGuidedThrowState(*Rope);
	const FRopeSimState& Sim = FRopeComponentRefactorTestSeam::GetSim(*Rope);
	TestEqual(TEXT("free path enters GuidedThrow"), Rope->GetPhase(), ERopePhase::GuidedThrow);
	TestTrue(TEXT("guided state is active"), State.bActive);
	TestTrue(TEXT("free path sets the free-throw discriminator"), State.bFreeThrow);
	TestEqual(TEXT("free preview has one point per rope node"),
		State.Prepared.RenderPreview.Points.Num(), Sim.Num());
	TestTrue(TEXT("common entry pins the start target to the throw origin"),
		Sim.StartPinTarget.Equals(Context.Origin, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("common entry places node zero at the throw origin"),
		Sim.Positions[0].Equals(Context.Origin, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("common preparation clears release cooldown"),
		FMath::IsNearlyZero(FRopeComponentRefactorTestSeam::GetReleaseCooldown(*Rope)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePhysicalResolveNonStretchPolicyTest,
	"DynamicRope.Component.PhysicalResolveUsesNonStretchThrowWrap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePhysicalResolveNonStretchPolicyTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->SolverConfig.MaxStretchRatio = 1.5f;

	for (const ERopeWrapResolveMode Mode :
		{ ERopeWrapResolveMode::FullSimulation, ERopeWrapResolveMode::AssistedJudged })
	{
		Rope->ResolveMode = Mode;
		FRopeComponentRefactorTestSeam::SetPhase(*Rope, ERopePhase::Free);
		TestTrue(TEXT("Free keeps the configured stretch policy"),
			FMath::IsNearlyEqual(Rope->GetEffectiveMaxStretchRatio(), 1.5f));

		for (const ERopePhase Phase : { ERopePhase::Flight, ERopePhase::Wrapping })
		{
			FRopeComponentRefactorTestSeam::SetPhase(*Rope, Phase);
			TestTrue(*FString::Printf(TEXT("physical mode %d phase %d is non-stretched"),
				static_cast<int32>(Mode), static_cast<int32>(Phase)),
				FMath::IsNearlyEqual(Rope->GetEffectiveMaxStretchRatio(), 1.0f));
		}

		FRopeComponentRefactorTestSeam::SetPhase(*Rope, ERopePhase::Wrapped);
		TestTrue(TEXT("stable Wrapped restores the configured stretch policy"),
			FMath::IsNearlyEqual(Rope->GetEffectiveMaxStretchRatio(), 1.5f));
		FRopeComponentRefactorTestSeam::ForceNonStretchThisFrame(*Rope, true);
		TestTrue(TEXT("the Wrapping-to-Wrapped commit frame remains non-stretched"),
			FMath::IsNearlyEqual(Rope->GetEffectiveMaxStretchRatio(), 1.0f));
		FRopeComponentRefactorTestSeam::ForceNonStretchThisFrame(*Rope, false);
	}

	Rope->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	FRopeComponentRefactorTestSeam::SetPhase(*Rope, ERopePhase::Wrapped);
	TestTrue(TEXT("GuaranteedWrap keeps its configured stretch policy"),
		FMath::IsNearlyEqual(Rope->GetEffectiveMaxStretchRatio(), 1.5f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAssistedAimColliderHandoffTest,
	"DynamicRope.Component.AssistedAim.PromotesLockedTargetIntoFlightColliders",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAssistedAimColliderHandoffTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	USkeletalMeshComponent* TargetMesh = NewObject<USkeletalMeshComponent>();
	USkeletalMeshComponent* OtherMesh = NewObject<USkeletalMeshComponent>();
	const FName PrimaryBone("upperarm_l");
	const FName NeighborBone("clavicle_l");

	RopeTest::FSphereMockCollider Primary(FVector(300.0f, 0.0f, 0.0f), 8.0f, PrimaryBone, TargetMesh);
	RopeTest::FSphereMockCollider Neighbor(FVector(305.0f, 0.0f, 0.0f), 8.0f, NeighborBone, TargetMesh);
	RopeTest::FSphereMockCollider Unrelated(FVector(300.0f, 20.0f, 0.0f), 8.0f, PrimaryBone, OtherMesh);

	FRopeComponentRefactorTestSeam::ConfigureAssistedAimLock(*Rope, TargetMesh, PrimaryBone);
	FRopeComponentRefactorTestSeam::GetAimFrameColliders(*Rope) = { &Primary, &Neighbor, &Unrelated };
	FRopeComponentRefactorTestSeam::GetFrameColliders(*Rope).Reset();

	FRopeComponentRefactorTestSeam::FilterFrameCollidersForAimWrapTarget(*Rope);
	const TArray<IRopeCollider*>& FlightColliders =
		FRopeComponentRefactorTestSeam::GetFrameColliders(*Rope);
	TestEqual(TEXT("locked target colliders are handed to the first Flight step"), FlightColliders.Num(), 2);
	TestTrue(TEXT("primary collider is promoted"), FlightColliders.Contains(&Primary));
	TestTrue(TEXT("same-mesh neighbor remains available for Assisted wrapping"), FlightColliders.Contains(&Neighbor));
	TestFalse(TEXT("an unrelated aimed mesh does not leak into physical Flight"), FlightColliders.Contains(&Unrelated));
	const FBox& LockedBounds = FRopeComponentRefactorTestSeam::GetLockedTargetColliderQueryBounds(*Rope);
	TestTrue(TEXT("locked target bounds survive after the aim ray is cleared"), LockedBounds.IsValid != 0);
	TestTrue(TEXT("cached bounds cover the primary target"), LockedBounds.Intersect(Primary.GetWorldBounds()));
	TestFalse(TEXT("cached bounds exclude unrelated aimed meshes"), LockedBounds.IsInside(Unrelated.Center));

	FRopeComponentRefactorTestSeam::ResetStateForNewThrow(*Rope);
	TestFalse(TEXT("a new throw clears the previous target bounds before taking a new lock"),
		FRopeComponentRefactorTestSeam::GetLockedTargetColliderQueryBounds(*Rope).IsValid != 0);

	FRopeComponentRefactorTestSeam::ConfigureAimLock(
		*Rope, TargetMesh, PrimaryBone, ERopeWrapResolveMode::FullSimulation);
	FRopeComponentRefactorTestSeam::GetAimFrameColliders(*Rope) = { &Primary, &Neighbor, &Unrelated };
	FRopeComponentRefactorTestSeam::GetFrameColliders(*Rope).Reset();
	FRopeComponentRefactorTestSeam::FilterFrameCollidersForAimWrapTarget(*Rope);
	TestEqual(TEXT("FullSimulation promotes only colliders owned by the locked target mesh"),
		FRopeComponentRefactorTestSeam::GetFrameColliders(*Rope).Num(), 2);
	TestTrue(TEXT("FullSimulation keeps the remote locked target available to its first physical step"),
		FRopeComponentRefactorTestSeam::GetFrameColliders(*Rope).Contains(&Primary));
	TestFalse(TEXT("FullSimulation does not promote an unrelated aim-ray mesh"),
		FRopeComponentRefactorTestSeam::GetFrameColliders(*Rope).Contains(&Unrelated));
	TestTrue(TEXT("FullSimulation persists only the locked target query bounds"),
		FRopeComponentRefactorTestSeam::GetLockedTargetColliderQueryBounds(*Rope).IsValid != 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAssistedActualSweepDecisionIsFrameRateIndependentTest,
	"DynamicRope.Component.AssistedAim.ActualSweepDecisionIsFrameRateIndependent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAssistedActualSweepDecisionIsFrameRateIndependentTest::RunTest(const FString& Parameters)
{
	constexpr float HighFpsDeltaTime = 1.0f / 240.0f;
	USceneComponent* Mesh = NewObject<USceneComponent>();
	const FName Bone("upperarm_l");

	auto MakeRope = [&]()
	{
		URopeComponent* Rope = NewObject<URopeComponent>();
		FRopeComponentRefactorTestSeam::ConfigureSim(*Rope, 4, 60.0f);
		FRopeComponentRefactorTestSeam::ConfigureAssistedAimLock(*Rope, Mesh, Bone);
		Rope->DetectConfig.WrapDecisionTime = 0.016f;
		return Rope;
	};

	FRopeContactCandidate Candidate;
	Candidate.bValid = true;
	Candidate.NodeIndex = 1;
	Candidate.Bone = Bone;
	Candidate.Mesh = Mesh;
	Candidate.WorldPoint = FVector(20.0f, 2.0f, 0.0f);
	Candidate.Normal = FVector::YAxisVector;
	Candidate.Penetration = 1.0f;
	Candidate.Source = ERopeContactCandidateSource::Actual;
	Candidate.SourceMask = static_cast<uint8>(ERopeContactCandidateSource::Actual);

	URopeComponent* ActualRope = MakeRope();
	const float ActualDwell = FRopeComponentRefactorTestSeam::BuildContactingDwell(
		*ActualRope, { Candidate }, HighFpsDeltaTime);
	TestEqual(TEXT("locked Assisted Actual sweep receives one nominal 60 Hz contact frame at 240 Hz"),
		ActualDwell, 1.0f / 60.0f);
	TestTrue(TEXT("nominal contact frame clears the default time-based decision"),
		ActualDwell >= ActualRope->DetectConfig.WrapDecisionTime);

	Candidate.Source = ERopeContactCandidateSource::PredictiveGuided;
	Candidate.SourceMask = static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided);
	URopeComponent* PredictiveRope = MakeRope();
	const float PredictiveDwell = FRopeComponentRefactorTestSeam::BuildContactingDwell(
		*PredictiveRope, { Candidate }, HighFpsDeltaTime);
	TestEqual(TEXT("predictive-only candidate keeps one real frame of dwell"),
		PredictiveDwell, HighFpsDeltaTime);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAssistedGpuCaptureDefersImmediateWrapTest,
	"DynamicRope.Component.AssistedAim.GpuCaptureDefersWrapUntilCoherentHandoff",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAssistedGpuCaptureDefersImmediateWrapTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeComponentRefactorTestSeam::ConfigureSim(*Rope, 4, 60.0f);
	FRopeComponentRefactorTestSeam::SetPhase(*Rope, ERopePhase::Flight);
	Rope->DetectConfig.WrapDecisionTime = 0.0f;
	USceneComponent* Mesh = NewObject<USceneComponent>();
	const FName Bone("upperarm_l");

	FRopeContactCandidate Candidate;
	Candidate.bValid = true;
	Candidate.NodeIndex = 1;
	Candidate.Bone = Bone;
	Candidate.Mesh = Mesh;
	Candidate.WorldPoint = FVector(20.0f, 3.0f, 0.0f);
	Candidate.Normal = FVector::YAxisVector;
	Candidate.Penetration = 1.0f;
	TArray<FRopeContactCandidate> Candidates = { Candidate };

	FRopeFlightCaptureEvaluation Evaluation;
	Evaluation.bShouldCapture = true;
	Evaluation.Tracker.CandidateBone = Bone;
	Evaluation.Tracker.CandidateMesh = Mesh;
	Evaluation.Tracker.CandidateNodes.Add(1);

	TestTrue(TEXT("fixture captures"),
		FRopeComponentRefactorTestSeam::ApplyGpuFlightCapture(*Rope, Candidates, Evaluation));
	TestEqual(TEXT("GPU capture waits in Contacting until the pending Flight step is coherent"),
		Rope->GetPhase(), ERopePhase::Contacting);
	TestTrue(TEXT("GPU capture keeps a persistent handoff flag after bGpuSteppedThisFrame can be reset"),
		FRopeComponentRefactorTestSeam::IsGpuCaptureHandoffPending(*Rope));
	return true;
}

#endif
