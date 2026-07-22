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

#endif
