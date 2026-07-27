// Copyright Epic Games, Inc. All Rights Reserved.
//
// Unit tests for the sleep behaviour of FRopeSolverThrottle. They pin the contract of the wrapped rest throttle,
// which extends the Free sleep, with no world: (1) settling, meaning staying slow, leads to sleep in Wrapped as well;
// (2) node drift produced by a logic write while asleep, as when Hold follows its bone, wakes it; and (3) holding it
// awake, as an armed active pull does, prevents it entering sleep. A drifting threshold contract produces the kind of
// regression where a wrapped rope on an elevator has its free span freeze while the rope is moving, so it is pinned here.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeSolverThrottle.h"

namespace
{
	// A stationary chain of N nodes. The throttle reads the positions and the start pin target alone.
	FRopeSimState MakeStaticSim(int32 NumNodes)
	{
		FRopeSimState Sim;
		Sim.Positions.SetNum(NumNodes);
		Sim.PrevPositions.SetNum(NumNodes);
		for (int32 i = 0; i < NumNodes; ++i)
		{
			Sim.Positions[i] = FVector(i * 10.0, 0.0, 0.0);
			Sim.PrevPositions[i] = Sim.Positions[i];
		}
		Sim.StartPinTarget = Sim.Positions[0];
		return Sim;
	}

	FRopeSolverConfig MakeSleepConfig()
	{
		FRopeSolverConfig Config;
		Config.bAllowSleep = true;
		Config.SleepVelocityThreshold = 3.0f;
		// Twelve frames at 60 fps means it enters sleep, which keeps the frame count of the tests short.
		Config.SleepDelay = 0.2f;
		return Config;
	}

	// Runs the stationary measurement frames up to the maximum and reports whether it went to sleep.
	bool RunSettleFrames(FRopeSolverThrottle& Throttle, const FRopeSimState& Sim,
		const FRopeSolverConfig& Config, ERopePhase Phase, int32 MaxFrames, bool bHoldAwake = false)
	{
		const float Dt = 1.0f / 60.0f;
		for (int32 Frame = 0; Frame < MaxFrames; ++Frame)
		{
			Throttle.UpdateSleepState(Phase, Sim, Config, Dt, bHoldAwake);
			if (Throttle.IsAsleep())
			{
				return true;
			}
		}
		return Throttle.IsAsleep();
	}
}

// Settling at rest in Wrapped enters sleep, and node drift afterwards, meaning the bone movement Hold wrote into the
// mirror while asleep, wakes it. The drift threshold is 0.5 cm, the same as the collider rest test: below it stays
// asleep, above it wakes.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeThrottleWrappedSleepDriftWakeTest,
	"DynamicRope.Throttle.WrappedSleepEntryAndDriftWake",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeThrottleWrappedSleepDriftWakeTest::RunTest(const FString& Parameters)
{
	FRopeSolverThrottle Throttle;
	FRopeSimState Sim = MakeStaticSim(8);
	const FRopeSolverConfig Config = MakeSleepConfig();
	const TArray<IRopeCollider*> NoColliders;

	TestTrue(TEXT("settling at rest in Wrapped leads to sleep"),
		RunSettleFrames(Throttle, Sim, Config, ERopePhase::Wrapped, 60));

	TestFalse(TEXT("staying at rest does not wake it"),
		Throttle.ShouldWakeFromSleep(Sim, Config, /*ReelRate*/ 0.0f, NoColliders));

	// A drift below the threshold, of 0.3 cm, stays asleep.
	FRopeSimState SubThreshold = Sim;
	SubThreshold.Positions[3] += FVector(0.3, 0.0, 0.0);
	TestFalse(TEXT("a node drift below 0.5 cm stays asleep"),
		Throttle.ShouldWakeFromSleep(SubThreshold, Config, 0.0f, NoColliders));

	// A drift above the threshold, of 1 cm, meaning the wrapped bone moved, as when an elevator sets off.
	FRopeSimState Drifted = Sim;
	Drifted.Positions[3] += FVector(1.0, 0.0, 0.0);
	TestTrue(TEXT("a node drift above 0.5 cm wakes it"),
		Throttle.ShouldWakeFromSleep(Drifted, Config, 0.0f, NoColliders));

	Throttle.Wake();
	TestFalse(TEXT("it is no longer asleep after waking"), Throttle.IsAsleep());
	return true;
}

// While held awake, as by an armed active pull, it does not fall asleep even at rest, and once the gate is released the delay starts afresh and it sleeps.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeThrottleHoldAwakeBlocksSleepTest,
	"DynamicRope.Throttle.HoldAwakeBlocksSleepEntry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeThrottleHoldAwakeBlocksSleepTest::RunTest(const FString& Parameters)
{
	FRopeSolverThrottle Throttle;
	const FRopeSimState Sim = MakeStaticSim(8);
	const FRopeSolverConfig Config = MakeSleepConfig();

	TestFalse(TEXT("it does not fall asleep at rest while held awake"),
		RunSettleFrames(Throttle, Sim, Config, ERopePhase::Wrapped, 60, /*bHoldAwake*/ true));

	TestTrue(TEXT("it enters sleep normally once the gate is released"),
		RunSettleFrames(Throttle, Sim, Config, ERopePhase::Wrapped, 60, /*bHoldAwake*/ false));
	return true;
}

// A non-sleeping phase such as Wrapping discards the measurement, which prevents a carried-over measurement putting it to sleep immediately on entering Wrapped.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeThrottleNonSleepPhaseResetTest,
	"DynamicRope.Throttle.NonSleepPhaseDiscardsMeasurement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeThrottleNonSleepPhaseResetTest::RunTest(const FString& Parameters)
{
	FRopeSolverThrottle Throttle;
	const FRopeSimState Sim = MakeStaticSim(8);
	const FRopeSolverConfig Config = MakeSleepConfig();
	const float Dt = 1.0f / 60.0f;

	// Accumulates up to just before settling, meaning below the delay, then spends one frame in Wrapping, which has to discard the accumulation and the cache.
	for (int32 Frame = 0; Frame < 6; ++Frame)
	{
		Throttle.UpdateSleepState(ERopePhase::Wrapped, Sim, Config, Dt);
	}
	Throttle.UpdateSleepState(ERopePhase::Wrapping, Sim, Config, Dt);

	// With the cache discarded, the first Wrapped frame afterwards has nothing to compare against and needs the delay plus one frame again before entering sleep.
	Throttle.UpdateSleepState(ERopePhase::Wrapped, Sim, Config, Dt);
	TestFalse(TEXT("nothing carries over into sleep after passing through Wrapping"), Throttle.IsAsleep());
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
