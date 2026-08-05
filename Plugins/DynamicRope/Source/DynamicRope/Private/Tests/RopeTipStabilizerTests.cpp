// Copyright 2026 TeamKeno. All Rights Reserved.
//
// FRopeTipStabilizer hold, converge, pass-through and tail-direction unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeTipStabilizer.h"

namespace
{
	// 60 fps frames keep every scenario in the speed regime it is written for.
	constexpr float TestDt = 1.0f / 60.0f;

	// Explicit test calibration, independent of the production defaults: every scenario below is
	// written against these thresholds, so retuning the shipped values must not silently reshape
	// what the tests exercise.
	FRopeTipStabilizer::FParams DefaultParams()
	{
		FRopeTipStabilizer::FParams Params;
		Params.PositionDeadband = 0.3f;
		Params.AngleDeadbandDeg = 1.5f;
		Params.SmoothingHalfLife = 0.05f;
		Params.FadeOutSpeed = 120.0f;
		Params.TeleportDistance = 100.0f;
		return Params;
	}

	FVector YawDir(float Degrees)
	{
		const float Rad = FMath::DegreesToRadians(Degrees);
		return FVector(FMath::Cos(Rad), FMath::Sin(Rad), 0.0f);
	}
}

// The reason the stabilizer exists: solver residual and idle-animation propagation keep the resting
// tip moving by fractions of the deadband, and all of it must be absorbed into a perfectly still output.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerHoldsUnderNoiseTest,
	"DynamicRope.TipStabilizer.HoldsUnderNoise",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerHoldsUnderNoiseTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	const FRopeTipStabilizer::FParams Params = DefaultParams();
	const FRopeTipStabilizer::FSample Seed{ FVector(100, 50, 0), FVector::ForwardVector };
	Stabilizer.Stabilize(Seed, TestDt, Params);

	for (int32 Frame = 0; Frame < 60; ++Frame)
	{
		// Alternating sub-deadband noise: +-0.2 cm position (24 cm/s peak, inside the fade start)
		// and +-1 degree direction, both below the 0.3 cm / 1.5 degree bands.
		const float Sign = (Frame % 2 == 0) ? 1.0f : -1.0f;
		const FRopeTipStabilizer::FSample Noisy{
			Seed.Position + FVector(0, 0, 0.2f * Sign), YawDir(1.0f * Sign) };
		const FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize(Noisy, TestDt, Params);
		if (!Out.Position.Equals(Seed.Position, 1e-6f) || !Out.Direction.Equals(Seed.Direction, 1e-6f))
		{
			AddError(FString::Printf(TEXT("output moved under sub-deadband noise at frame %d"), Frame));
			return false;
		}
	}
	return true;
}

// Past the deadband the output must ramp, not snap, and settle on the target without overshoot.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerConvergesAfterDeadbandTest,
	"DynamicRope.TipStabilizer.ConvergesAfterDeadband",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerConvergesAfterDeadbandTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	const FRopeTipStabilizer::FParams Params = DefaultParams();
	const FRopeTipStabilizer::FSample Seed{ FVector::ZeroVector, FVector::ForwardVector };
	Stabilizer.Stabilize(Seed, TestDt, Params);

	// A 5 cm step, held constant. The long first dt keeps the implied speed inside the fade band, so
	// the converge (not the pass-through) is what gets exercised.
	const FRopeTipStabilizer::FSample Target{ FVector(5, 0, 0), FVector::ForwardVector };
	FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize(Target, 0.1f, Params);
	TestTrue(TEXT("the first frame ramps instead of snapping"),
		FVector::Dist(Out.Position, Target.Position) > 0.1f);

	float PrevGap = FVector::Dist(Out.Position, Target.Position);
	for (int32 Frame = 0; Frame < 40; ++Frame)
	{
		Out = Stabilizer.Stabilize(Target, TestDt, Params);
		const float Gap = FVector::Dist(Out.Position, Target.Position);
		if (Gap > PrevGap + 1e-6f)
		{
			AddError(FString::Printf(TEXT("distance to target grew (overshoot) at frame %d"), Frame));
			return false;
		}
		PrevGap = Gap;
	}
	TestTrue(TEXT("the output settles on the target"), Out.Position.Equals(Target.Position, 0.01f));
	return true;
}

// Flight and hard drags must not lag: at speed the filter has to become an exact pass-through.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerFastMotionPassesThroughTest,
	"DynamicRope.TipStabilizer.FastMotionPassesThrough",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerFastMotionPassesThroughTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	const FRopeTipStabilizer::FParams Params = DefaultParams();
	FRopeTipStabilizer::FSample Raw{ FVector::ZeroVector, FVector::ForwardVector };
	Stabilizer.Stabilize(Raw, TestDt, Params);

	// 500 cm/s, well past FadeOutSpeed. The sustained-speed estimate needs a frame or two of travel
	// to register (that is the price of jitter immunity), so the onset may ramp by a small margin;
	// from then on every frame's output must equal the raw input exactly.
	for (int32 Frame = 0; Frame < 30; ++Frame)
	{
		Raw.Position += FVector(500.0f * TestDt, 0, 0);
		Raw.Direction = YawDir(2.0f * static_cast<float>(Frame));
		const FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize(Raw, TestDt, Params);
		if (Frame < 2)
		{
			if (FVector::Dist(Out.Position, Raw.Position) > 2.0f)
			{
				AddError(FString::Printf(TEXT("the motion onset lagged more than 2 cm at frame %d"), Frame));
				return false;
			}
			continue;
		}
		if (!Out.Position.Equals(Raw.Position, 1e-4f) || !Out.Direction.Equals(Raw.Direction, 1e-4f))
		{
			AddError(FString::Printf(TEXT("fast motion did not pass through raw at frame %d"), Frame));
			return false;
		}
	}
	return true;
}

// The reported failure mode this guards against: in-place jitter spikes the per-frame delta (0.5 cm
// at 60 fps reads as 60 cm/s), and a per-frame speed estimate would fade the filter out against
// itself — the shakier the tip, the less filtering. The sustained estimate must keep holding.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerHighFrequencyJitterStaysHeldTest,
	"DynamicRope.TipStabilizer.HighFrequencyJitterStaysHeld",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerHighFrequencyJitterStaysHeldTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	const FRopeTipStabilizer::FParams Params = DefaultParams();
	const FRopeTipStabilizer::FSample Seed{ FVector(100, 50, 0), FVector::ForwardVector };
	Stabilizer.Stabilize(Seed, TestDt, Params);

	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		// +-0.45 cm around the seed: a 0.9 cm swing per frame reads as an instantaneous ~108 cm/s,
		// which a per-frame estimate maps to a near-zero weight (the angle band collapses to well
		// under the 1 degree wobble) — while the net travel stays near zero. The direction must
		// stay latched exactly, and the position (past its band, so latching each frame) must come
		// out heavily attenuated by the smoothing rather than passed through.
		const float Sign = (Frame % 2 == 0) ? 1.0f : -1.0f;
		const FRopeTipStabilizer::FSample Noisy{
			Seed.Position + FVector(0, 0, 0.45f * Sign), YawDir(1.0f * Sign) };
		const FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize(Noisy, TestDt, Params);
		if (!Out.Direction.Equals(Seed.Direction, 1e-6f))
		{
			AddError(FString::Printf(TEXT("high-frequency jitter unlatched the direction at frame %d"), Frame));
			return false;
		}
		if (!Out.Position.Equals(Seed.Position, 0.2f))
		{
			AddError(FString::Printf(TEXT("high-frequency position jitter leaked through at frame %d"), Frame));
			return false;
		}
	}
	return true;
}

// The latches are independent: a position drifting past its own band must not unlatch a steady
// direction, or raising the angle deadband would have no visible effect while the rope creeps.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerAngleHoldsUnderPositionDriftTest,
	"DynamicRope.TipStabilizer.AngleHoldsUnderPositionDrift",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerAngleHoldsUnderPositionDriftTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	FRopeTipStabilizer::FParams Params = DefaultParams();
	Params.AngleDeadbandDeg = 10.0f;

	FRopeTipStabilizer::FSample Raw{ FVector::ZeroVector, YawDir(0.0f) };
	FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize(Raw, TestDt, Params);

	// The position creeps at 6 cm/s (crossing its 0.3 cm band every few frames) while the direction
	// wobbles +-5 degrees, inside the widened band. The direction must stay latched throughout.
	for (int32 Frame = 0; Frame < 240; ++Frame)
	{
		const float Sign = (Frame % 2 == 0) ? 1.0f : -1.0f;
		Raw.Position += FVector(6.0f * TestDt, 0, 0);
		Raw.Direction = YawDir(5.0f * Sign);
		Out = Stabilizer.Stabilize(Raw, TestDt, Params);
		if (!Out.Direction.Equals(YawDir(0.0f), 1e-6f))
		{
			AddError(FString::Printf(TEXT("position drift unlatched the direction at frame %d"), Frame));
			return false;
		}
	}
	// The position channel kept following on its own.
	TestTrue(TEXT("the position followed the drift"), Out.Position.X > 15.0f);
	return true;
}

// A slow reel drifts the tip well under the deadband per frame; the latch steps must come out as
// small ramps, never as visible pops.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerSlowDragNoStaircaseTest,
	"DynamicRope.TipStabilizer.SlowDragNoStaircase",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerSlowDragNoStaircaseTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	const FRopeTipStabilizer::FParams Params = DefaultParams();
	FRopeTipStabilizer::FSample Raw{ FVector::ZeroVector, FVector::ForwardVector };
	FRopeTipStabilizer::FSample Prev = Stabilizer.Stabilize(Raw, TestDt, Params);

	// 2 cm/s: about nine frames per deadband crossing. The pop threshold is half the deadband.
	for (int32 Frame = 0; Frame < 300; ++Frame)
	{
		Raw.Position += FVector(2.0f * TestDt, 0, 0);
		const FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize(Raw, TestDt, Params);
		const float Step = FVector::Dist(Out.Position, Prev.Position);
		if (Step > 0.15f)
		{
			AddError(FString::Printf(TEXT("output stepped %.3f cm (pop) at frame %d"), Step, Frame));
			return false;
		}
		Prev = Out;
	}
	// The drag must still make progress: the output tracks the drift instead of freezing.
	TestTrue(TEXT("the output followed the slow drag"), Prev.Position.X > 8.0f);
	return true;
}

// A teleport must not be smoothed across space; it reseeds and holds at the new spot.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerTeleportSnapsAndReseedsTest,
	"DynamicRope.TipStabilizer.TeleportSnapsAndReseeds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerTeleportSnapsAndReseedsTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	const FRopeTipStabilizer::FParams Params = DefaultParams();
	const FRopeTipStabilizer::FSample Seed{ FVector::ZeroVector, FVector::ForwardVector };
	Stabilizer.Stabilize(Seed, TestDt, Params);

	const FRopeTipStabilizer::FSample Jumped{ FVector(500, 0, 0), FVector::RightVector };
	const FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize(Jumped, TestDt, Params);
	TestTrue(TEXT("a teleport returns the input exactly"),
		Out.Position.Equals(Jumped.Position, 1e-6f) && Out.Direction.Equals(Jumped.Direction, 1e-6f));

	// The new position is the new hold reference: sub-deadband noise around it stays put.
	const FRopeTipStabilizer::FSample Noisy{ Jumped.Position + FVector(0, 0, 0.2f), Jumped.Direction };
	const FRopeTipStabilizer::FSample Held = Stabilizer.Stabilize(Noisy, TestDt, Params);
	TestTrue(TEXT("the reseeded state holds at the new position"),
		Held.Position.Equals(Jumped.Position, 1e-6f));
	return true;
}

// Reset drops every latch; the next call must return its input and start over. Zero dt is inert.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerResetReturnsRawTest,
	"DynamicRope.TipStabilizer.ResetReturnsRaw",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerResetReturnsRawTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	const FRopeTipStabilizer::FParams Params = DefaultParams();

	// Before any seed, a zero dt has no state to hold and must fall back to the input.
	const FRopeTipStabilizer::FSample First{ FVector(1, 2, 3), FVector::RightVector };
	FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize(First, 0.0f, Params);
	TestTrue(TEXT("zero dt before any seed returns the input"), Out.Position.Equals(First.Position, 1e-6f));
	TestFalse(TEXT("zero dt does not seed"), Stabilizer.HasState());

	Stabilizer.Stabilize(First, TestDt, Params);
	TestTrue(TEXT("a real frame seeds"), Stabilizer.HasState());

	// A zero-dt frame after seeding holds the last output.
	Out = Stabilizer.Stabilize({ FVector(999, 0, 0), FVector::UpVector }, 0.0f, Params);
	TestTrue(TEXT("zero dt after seeding holds the last output"), Out.Position.Equals(First.Position, 1e-6f));

	Stabilizer.Reset();
	TestFalse(TEXT("Reset drops the state"), Stabilizer.HasState());
	const FRopeTipStabilizer::FSample Fresh{ FVector(50, 0, 0), FVector::UpVector };
	Out = Stabilizer.Stabilize(Fresh, TestDt, Params);
	TestTrue(TEXT("the first call after Reset returns the input exactly"),
		Out.Position.Equals(Fresh.Position, 1e-6f) && Out.Direction.Equals(Fresh.Direction, 1e-6f));
	return true;
}

// The averaged tail direction must beat the raw last segment on a jittered chain, reproduce it at
// SampleCount 1, and degrade through the fallback chain on degenerate input.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerTailDirectionAveragingTest,
	"DynamicRope.TipStabilizer.TailDirectionAveraging",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerTailDirectionAveragingTest::RunTest(const FString& Parameters)
{
	// A straight +X chain with alternating Z jitter: the raw last segment tilts by the full jitter,
	// the weighted average mostly cancels it.
	TArray<FVector> Chain;
	for (int32 i = 0; i < 5; ++i)
	{
		const float Z = (i % 2 == 0) ? 0.5f : -0.5f;
		Chain.Add(FVector(10.0f * static_cast<float>(i), 0, Z));
	}
	const FVector Truth = FVector::ForwardVector;
	const FVector RawDir = FRopeTipStabilizer::ComputeTailDirection(Chain, 1, FVector::ForwardVector);
	const FVector AvgDir = FRopeTipStabilizer::ComputeTailDirection(Chain, 4, FVector::ForwardVector);
	const float RawError = FMath::Acos(FMath::Clamp(static_cast<float>(RawDir | Truth), -1.0f, 1.0f));
	const float AvgError = FMath::Acos(FMath::Clamp(static_cast<float>(AvgDir | Truth), -1.0f, 1.0f));
	TestTrue(TEXT("averaging reduces the direction error"), AvgError < RawError * 0.5f);

	// SampleCount 1 is exactly the raw last segment.
	const FVector LastSeg = (Chain[4] - Chain[3]).GetSafeNormal();
	TestTrue(TEXT("SampleCount 1 reproduces the raw last segment"), RawDir.Equals(LastSeg, 1e-6f));

	// Below 2 nodes there is no segment at all: the caller's fallback comes back.
	const TArray<FVector> Single{ FVector(1, 2, 3) };
	TestTrue(TEXT("a single node returns the fallback"),
		FRopeTipStabilizer::ComputeTailDirection(Single, 4, FVector::RightVector).Equals(FVector::RightVector, 1e-6f));

	// Coincident nodes degenerate every segment; the fallback is the last resort.
	const TArray<FVector> Collapsed{ FVector(7, 7, 7), FVector(7, 7, 7), FVector(7, 7, 7) };
	TestTrue(TEXT("a collapsed chain returns the fallback"),
		FRopeTipStabilizer::ComputeTailDirection(Collapsed, 4, FVector::UpVector).Equals(FVector::UpVector, 1e-6f));
	return true;
}

// The latch is the hysteresis: direction noise hugging the threshold must not re-shiver, before or
// after the latch is pushed to a new heading.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTipStabilizerDirectionDeadbandHysteresisTest,
	"DynamicRope.TipStabilizer.DirectionDeadbandHysteresis",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTipStabilizerDirectionDeadbandHysteresisTest::RunTest(const FString& Parameters)
{
	FRopeTipStabilizer Stabilizer;
	const FRopeTipStabilizer::FParams Params = DefaultParams();
	const FVector Pos(0, 0, 0);
	Stabilizer.Stabilize({ Pos, YawDir(0.0f) }, TestDt, Params);

	// +-1.4 degrees around the latch, just inside the 1.5 degree band: the output must not move.
	for (int32 Frame = 0; Frame < 30; ++Frame)
	{
		const float Sign = (Frame % 2 == 0) ? 1.0f : -1.0f;
		const FRopeTipStabilizer::FSample Out = Stabilizer.Stabilize({ Pos, YawDir(1.4f * Sign) }, TestDt, Params);
		if (!Out.Direction.Equals(YawDir(0.0f), 1e-6f))
		{
			AddError(FString::Printf(TEXT("direction moved inside the deadband at frame %d"), Frame));
			return false;
		}
	}

	// A 5 degree turn crosses the band: the latch moves and the output converges to the new heading.
	FRopeTipStabilizer::FSample Out;
	for (int32 Frame = 0; Frame < 90; ++Frame)
	{
		Out = Stabilizer.Stabilize({ Pos, YawDir(5.0f) }, TestDt, Params);
	}
	TestTrue(TEXT("the output converges to the new heading"), Out.Direction.Equals(YawDir(5.0f), 1e-3f));

	// Noise around the new latch is held again — no re-shiver at the threshold.
	const FRopeTipStabilizer::FSample Settled = Out;
	for (int32 Frame = 0; Frame < 30; ++Frame)
	{
		const float Sign = (Frame % 2 == 0) ? 1.0f : -1.0f;
		Out = Stabilizer.Stabilize({ Pos, YawDir(5.0f + 1.4f * Sign) }, TestDt, Params);
		if (!Out.Direction.Equals(Settled.Direction, 1e-6f))
		{
			AddError(FString::Printf(TEXT("direction re-shivered around the new latch at frame %d"), Frame));
			return false;
		}
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
