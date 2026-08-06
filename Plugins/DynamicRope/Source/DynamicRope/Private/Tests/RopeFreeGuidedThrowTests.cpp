// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Regression tests for the open-space (free) guided throw. The guide is the rope pulled taut along
// the input-frame aim ray: its endpoint is the furthest ray point within rope reach of the live hand,
// re-resolved every frame, so a stationary wielder keeps the throw-time endpoint, running forward
// extends the flight along the ray, and running away shortens it. The bugs locked out: nodes heading
// for a line frozen at the throw-time origin read as the rope being yanked backwards, and a landing
// with slack (a line shorter than the rope) folds the free end back toward the pinned hand.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeComponent.h"
#include "RopeMathHelpers.h"
#include "RopeTestHelpers.h"

struct FRopeFreeGuidedThrowTestSeam
{
	static void StartFreeGuidedThrow(URopeComponent& Rope,
		const FRopeThrowContext& Context, const FVector& Endpoint)
	{
		Rope.Sim = RopeTest::MakeStraightRope(5, 100.0f);
		Rope.StartFreeGuidedThrow(Context, Endpoint);
	}

	/** One guided-throw frame against an explicit hand position: PrepareSimFrame's per-frame pin
	 *  refresh and override reset, without a transform hierarchy or a world. */
	static const FRopeNodeOverrideFrame& TickGuidedThrow(URopeComponent& Rope,
		const FVector& HandWorld, float DeltaTime)
	{
		Rope.SimFrame.OverrideFrame.Reset();
		Rope.Sim.StartPinPrev = Rope.Sim.StartPinTarget;
		Rope.Sim.StartPinTarget = HandWorld;
		Rope.UpdateGuidedThrow(DeltaTime);
		return Rope.SimFrame.OverrideFrame;
	}

	static const FRopeNodeOverrideFrame& GetOverrideFrame(const URopeComponent& Rope)
	{
		return Rope.SimFrame.OverrideFrame;
	}
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFreeGuidedThrowFollowsMovingHandTest,
	"DynamicRope.Component.GuidedThrow.FreeGuidedThrowFollowsMovingHand",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFreeGuidedThrowFollowsMovingHandTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	FRopeThrowContext Context;
	Context.Origin = FVector::ZeroVector;
	Context.FrameForward = FVector(1.0f, 0.0f, 0.0f);
	const FVector Endpoint(100.0f, 0.0f, 0.0f);
	FRopeFreeGuidedThrowTestSeam::StartFreeGuidedThrow(*Rope, Context, Endpoint);
	TestEqual(TEXT("free throw enters GuidedThrow"), Rope->GetPhase(), ERopePhase::GuidedThrow);

	// A wielder sprinting 40 cm per 20 ms tick — fast enough to run past the throw-time endpoint
	// mid-flight.
	const FVector Dir(1.0f, 0.0f, 0.0f);
	const FVector HandStep(40.0f, 0.0f, 0.0f);
	constexpr float Dt = 0.02f;
	FVector Hand = FVector::ZeroVector;
	FVector LandingHand = FVector::ZeroVector;
	int32 SafetyTicks = 0;
	while (Rope->GetPhase() == ERopePhase::GuidedThrow && SafetyTicks++ < 64)
	{
		Hand += HandStep;
		FRopeFreeGuidedThrowTestSeam::TickGuidedThrow(*Rope, Hand, Dt);
		if (Rope->GetPhase() != ERopePhase::GuidedThrow)
		{
			LandingHand = Hand;
		}
	}

	TestEqual(TEXT("the free throw lands in Free"), Rope->GetPhase(), ERopePhase::Free);
	TestTrue(TEXT("the scenario actually ran past the throw-time endpoint"),
		static_cast<float>(FVector::DotProduct(LandingHand - Endpoint, Dir)) > 0.0f);

	// The landing frame's override is still in SimFrame. Running forward carried the taut endpoint
	// along the ray: the rope lands taut at rope reach from the live hand — flying farther than the
	// throw-time spot instead of dying there with slack — and the line it lands on runs from the live
	// hand, not from the throw-time origin.
	const FRopeNodeOverrideFrame& Landing = FRopeFreeGuidedThrowTestSeam::GetOverrideFrame(*Rope);
	TestEqual(TEXT("landing frame wrote every node"), Landing.Flags.Num(), 5);
	const int32 LastNode = Landing.Positions.Num() - 1;
	const FVector TautEndpoint = LandingHand + Dir * 100.0f;
	TestTrue(TEXT("the tip lands taut at rope reach along the ray"),
		Landing.Positions[LastNode].Equals(TautEndpoint, 0.5f));
	for (int32 NodeIndex = 1; NodeIndex < LastNode; ++NodeIndex)
	{
		const float NodeFrac = static_cast<float>(NodeIndex) / static_cast<float>(LastNode);
		const FVector Expected = FMath::Lerp(LandingHand, TautEndpoint, NodeFrac);
		TestTrue(*FString::Printf(TEXT("landed node %d sits on the live taut line"), NodeIndex),
			Landing.Positions[NodeIndex].Equals(Expected, 0.01f));
	}

	// The landing hands the nodes back to physics at rest: node 0 keeps the hand pin, the rest get
	// their mass back with zero velocity.
	for (int32 NodeIndex = 0; NodeIndex < Landing.Flags.Num(); ++NodeIndex)
	{
		TestTrue(*FString::Printf(TEXT("node %d lands with zero velocity"), NodeIndex),
			(Landing.Flags[NodeIndex] & RopeNodeOverride::PrevFromPosition) != 0);
		TestTrue(*FString::Printf(TEXT("node %d mass restored"), NodeIndex),
			FMath::IsNearlyEqual(Landing.InvMass[NodeIndex], NodeIndex == 0 ? 0.0f : 1.0f));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFreeGuidedThrowSlackSagTest,
	"DynamicRope.Component.GuidedThrow.FreeGuidedThrowSlackSag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFreeGuidedThrowSlackSagTest::RunTest(const FString& Parameters)
{
	// The sag depth: with the taut endpoint the line only stays shorter than the rope when the ray is
	// world-blocked, and the surplus is then laid in as a parabolic droop sized so its arc length
	// approximates the rope length — a compressed straight landing folds its slack back toward the
	// pinned hand.
	TestTrue(TEXT("no sag when the line covers the full rope"),
		FMath::IsNearlyZero(RopeMath::ComputeFreeGuideSagDepth(100.0f, 100.0f)));
	TestTrue(TEXT("no sag for a degenerate line"),
		FMath::IsNearlyZero(RopeMath::ComputeFreeGuideSagDepth(0.0f, 100.0f)));
	const float Sag = RopeMath::ComputeFreeGuideSagDepth(40.0f, 100.0f);
	TestTrue(TEXT("a short line droops"), Sag > 1.0f);
	TestTrue(TEXT("the sag depth encodes the slack as parabolic arc length"),
		FMath::IsNearlyEqual(40.0f + 8.0f * Sag * Sag / (3.0f * 40.0f), 100.0f, 1.0f));

	// A stationary full-reach throw: the taut endpoint equals the throw-time endpoint, so the rope
	// lands as the straight taut line at the aimed spot — the standing behavior is untouched by the
	// taut tracking.
	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeThrowContext Context;
	Context.Origin = FVector::ZeroVector;
	Context.FrameForward = FVector(1.0f, 0.0f, 0.0f);
	const FVector Endpoint(100.0f, 0.0f, 0.0f);
	FRopeFreeGuidedThrowTestSeam::StartFreeGuidedThrow(*Rope, Context, Endpoint);
	TestEqual(TEXT("stationary free throw enters GuidedThrow"), Rope->GetPhase(), ERopePhase::GuidedThrow);

	int32 SafetyTicks = 0;
	while (Rope->GetPhase() == ERopePhase::GuidedThrow && SafetyTicks++ < 64)
	{
		FRopeFreeGuidedThrowTestSeam::TickGuidedThrow(*Rope, FVector::ZeroVector, 0.02f);
	}
	TestEqual(TEXT("stationary free throw lands in Free"), Rope->GetPhase(), ERopePhase::Free);

	const FRopeNodeOverrideFrame& Landing = FRopeFreeGuidedThrowTestSeam::GetOverrideFrame(*Rope);
	const int32 LastNode = Landing.Positions.Num() - 1;
	TestTrue(TEXT("a stationary throw still lands the tip on the throw-time endpoint"),
		Landing.Positions[LastNode].Equals(Endpoint, 0.5f));
	for (int32 NodeIndex = 1; NodeIndex < LastNode; ++NodeIndex)
	{
		const float NodeFrac = static_cast<float>(NodeIndex) / static_cast<float>(LastNode);
		const FVector Expected = FMath::Lerp(FVector::ZeroVector, Endpoint, NodeFrac);
		TestTrue(*FString::Printf(TEXT("stationary landed node %d sits straight on the taut line"), NodeIndex),
			Landing.Positions[NodeIndex].Equals(Expected, 0.01f));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
