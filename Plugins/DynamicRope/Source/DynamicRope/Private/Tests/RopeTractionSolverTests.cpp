// Copyright Epic Games, Inc. All Rights Reserved.
//
// Unit tests for RopeTraction, verifying the traction axis drive maths, covering both the tether and
// active pull, with no world.
// This maths was verified only through play-in-editor for a long time, and two measured defects arose in
// that period: an explosion at high tension and a permanent lag. The regression tests below pin both.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeTractionSolver.h"

namespace
{
	// A bidirectional exact servo, including braking, used to verify the servo semantics.
	RopeTraction::FRopeAxisServo MakeReelServo(float TargetSpeed)
	{
		RopeTraction::FRopeAxisServo Servo;
		Servo.TargetSpeed = TargetSpeed;
		Servo.Alpha = 1.0f;
		Servo.bBidirectional = true;
		Servo.bCancelOutward = false;
		return Servo;
	}

	// The one-directional servo, accelerating only, used by active pull and the wielder top-up.
	RopeTraction::FRopeAxisServo MakeOneWayServo(float TargetSpeed, float Alpha = 1.0f)
	{
		RopeTraction::FRopeAxisServo Servo;
		Servo.TargetSpeed = TargetSpeed;
		Servo.Alpha = Alpha;
		Servo.bBidirectional = false;
		Servo.bCancelOutward = false;
		return Servo;
	}
}

// A bidirectional servo removes the momentum beyond the target, which is braking. Regression: with
// acceleration only, high tension let the target overshoot the boundary, coast, go slack, bounce back and
// explode. Braking is what settles it on the boundary.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionBidirectionalBrakesTest,
	"DynamicRope.Traction.BidirectionalServoBrakesOvershoot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionBidirectionalBrakesTest::RunTest(const FString& Parameters)
{
	// The target is 100 but it is already racing inwards at 250, so the bidirectional servo brakes by 150.
	const float BrakeDv = RopeTraction::ComputeAxisDeltaV(250.0f, MakeReelServo(100.0f));
	TestEqual(TEXT("bidirectional brakes excess inward speed"), BrakeDv, -150.0f);

	// In the same situation the one-directional servo does nothing, leaving the excess momentum to
	// overshoot and coast, which is the old explosion path.
	const float OneWayDv = RopeTraction::ComputeAxisDeltaV(250.0f, MakeOneWayServo(100.0f));
	TestEqual(TEXT("one-directional servo does not brake"), OneWayDv, 0.0f);

	// When the target tapers to zero at the boundary, the bidirectional servo removes all remaining
	// momentum and stops with no overshoot.
	const float SettleDv = RopeTraction::ComputeAxisDeltaV(80.0f, MakeReelServo(0.0f));
	TestEqual(TEXT("bidirectional servo stops at the boundary"), SettleDv, -80.0f);
	return true;
}

// The tension clamp is what produces mass-dependent following, which is the skeleton of the active pull
// velocity drive. The threshold tension is roughly mass times velocity times the frame rate: above it the
// target is reached, and below it the receiver lags.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionTensionLimitTest,
	"DynamicRope.Traction.TensionLimitMakesHeavyTargetsLag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionTensionLimitTest::RunTest(const FString& Parameters)
{
	const float Dt = 1.0f / 60.0f;
	const float VTarget = 400.0f;
	const float DeltaV = RopeTraction::ComputeAxisDeltaV(0.0f, MakeReelServo(VTarget)); // From rest, so the full amount is needed.
	TestEqual(TEXT("delta v to reach target from rest"), DeltaV, VTarget);

	// The threshold tension for 50 kg to reach 400 cm/s in one frame is 50 * 400 * 60.
	const float Mass = 50.0f;
	const float Threshold = Mass * VTarget / Dt;

	// At or above the threshold it produces exactly the impulse required, which reaches the target with no
	// lag.
	const float JAmple = RopeTraction::ClampAxisImpulse(DeltaV, Mass, Threshold * Dt);
	TestEqual(TEXT("ample tension reaches the target exactly"), JAmple, Mass * VTarget);

	// The default is far below that threshold, so it clamps and lags, which reads as weight; the delta
	// velocity is only the impulse over the mass.
	const float JLimited = RopeTraction::ClampAxisImpulse(DeltaV, Mass, 150000.0f * Dt);
	TestEqual(TEXT("default tension is capped for a 50kg target"), JLimited, 150000.0f * Dt);
	TestTrue(TEXT("capped impulse lags the target"), JLimited < Mass * VTarget);

	// The same tension on a light body exceeds the threshold and reaches the target, which is what makes
	// the following mass-dependent.
	const float JLight = RopeTraction::ClampAxisImpulse(DeltaV, 1.0f, 150000.0f * Dt);
	TestEqual(TEXT("a light target reaches the target under the same tension"), JLight, 1.0f * VTarget);

	// A limit of 0 means unlimited, giving an exact servo that reaches the target regardless of mass.
	TestEqual(TEXT("zero max impulse means unlimited exact servo"),
		RopeTraction::ClampAxisImpulse(DeltaV, Mass, 0.0f), Mass * VTarget);

	// Braking is clamped symmetrically, since clamping only one direction would make the runaway
	// protection asymmetric.
	TestEqual(TEXT("braking impulse is clamped symmetrically"),
		RopeTraction::ClampAxisImpulse(-VTarget, Mass, 150000.0f * Dt), -150000.0f * Dt);
	return true;
}

	// Cancelling outward motion is immediate and complete, ignoring the interpolation factor, while only
	// the inward recovery is damped by it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionCancelOutwardTest,
	"DynamicRope.Traction.CancelOutwardIsImmediateButReclaimIsDamped",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionCancelOutwardTest::RunTest(const FString& Parameters)
{
	RopeTraction::FRopeAxisServo Servo = MakeOneWayServo(100.0f, /*Alpha*/ 0.5f);
	Servo.bCancelOutward = true;

	// Walking outwards at 200 along the negative axis: 200 is cancelled immediately and completely, and
	// half of the remaining 100 to the target is applied, giving 250 in total.
	TestEqual(TEXT("outward walk is cancelled fully, reclaim is damped"),
		RopeTraction::ComputeAxisDeltaV(-200.0f, Servo), 250.0f);

	// The outward cancellation still happens even with a target of zero, which holds the boundary.
	RopeTraction::FRopeAxisServo StopOnly = MakeOneWayServo(0.0f, /*Alpha*/ 1.0f);
	StopOnly.bCancelOutward = true;
	TestEqual(TEXT("outward walk is cancelled even with a zero target"),
		RopeTraction::ComputeAxisDeltaV(-200.0f, StopOnly), 200.0f);

	// Without the cancellation, a target of zero simply removes the outward velocity, which is the path
	// taken by a simulating body that cannot be pulled.
	TestEqual(TEXT("zero-target one-way servo removes outward velocity"),
		RopeTraction::ComputeAxisDeltaV(-200.0f, MakeOneWayServo(0.0f)), 200.0f);
	// Motion that is already inward is left untouched.
	TestEqual(TEXT("zero-target one-way servo leaves inward velocity alone"),
		RopeTraction::ComputeAxisDeltaV(200.0f, MakeOneWayServo(0.0f)), 0.0f);
	return true;
}

	// The injected speed cap is the larger of the cap and the existing speed, so faster external motion is
	// preserved.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionClampInjectedTest,
	"DynamicRope.Traction.InjectedVelocityCapPreservesFasterMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionClampInjectedTest::RunTest(const FString& Parameters)
{
	// Starting slow, an injection that exceeds the cap is clamped to it.
	const FVector Clamped = RopeTraction::ClampInjectedVelocity(
		FVector(3000.0f, 0.0f, 0.0f), FVector(100.0f, 0.0f, 0.0f), 1500.0f);
	TestEqual(TEXT("injection is capped at the speed cap"), static_cast<float>(Clamped.Size()), 1500.0f, 1e-2f);

	// Already falling faster than the cap, that speed is preserved and the injection cannot increase it
	// further.
	const FVector Fast = RopeTraction::ClampInjectedVelocity(
		FVector(0.0f, 0.0f, -3000.0f), FVector(0.0f, 0.0f, -2000.0f), 1500.0f);
	TestEqual(TEXT("pre-existing faster motion is preserved"), static_cast<float>(Fast.Size()), 2000.0f, 1e-2f);

	// A cap of 0 disables the clamp.
	const FVector Uncapped = RopeTraction::ClampInjectedVelocity(
		FVector(9000.0f, 0.0f, 0.0f), FVector::ZeroVector, 0.0f);
	TestEqual(TEXT("zero cap disables clamping"), static_cast<float>(Uncapped.Size()), 9000.0f, 1e-2f);
	return true;
}

	// The exponential smoothing coefficient converges on the same time constant at any frame rate and does
	// not overshoot even at a large delta.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionExpSmoothAlphaTest,
	"DynamicRope.Traction.ExpSmoothAlphaIsFrameRateIndependent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionExpSmoothAlphaTest::RunTest(const FString& Parameters)
{
	// A time constant at or below zero disables smoothing and reaches the target in one frame.
	TestEqual(TEXT("zero tau means no smoothing"), RopeTraction::ExpSmoothAlpha(0.0f, 1.0f / 60.0f), 1.0f);
	TestEqual(TEXT("negative tau means no smoothing"), RopeTraction::ExpSmoothAlpha(-1.0f, 1.0f / 60.0f), 1.0f);

	// With a delta equal to the time constant the coefficient is one minus the reciprocal of e, which is
	// the definition of a time constant.
	TestEqual(TEXT("one time constant leaves 1/e remaining"),
		RopeTraction::ExpSmoothAlpha(0.12f, 0.12f), 1.0f - FMath::Exp(-1.0f), 1e-4f);

	// However large the delta, the coefficient stays at or below one, so overshooting past the target is
	// impossible by construction.
	TestTrue(TEXT("a huge dt never overshoots"), RopeTraction::ExpSmoothAlpha(0.12f, 10.0f) <= 1.0f);

	// Frame rate independence: the remainder after two frames at 60 Hz equals the remainder after one at
	// 30 Hz, since both multiply by the same exponential.
	const float Tau = 0.12f;
	const float Remain60 = (1.0f - RopeTraction::ExpSmoothAlpha(Tau, 1.0f / 60.0f));
	const float Remain30 = (1.0f - RopeTraction::ExpSmoothAlpha(Tau, 1.0f / 30.0f));
	TestEqual(TEXT("two 60fps steps equal one 30fps step"), Remain60 * Remain60, Remain30, 1e-4f);
	return true;
}

	// The direction average: seeding when unseeded, ordinary interpolation, and the degenerate case of a
	// 180 degree reversal, which has to reseed. Without the reseed the direction stays at zero and the axis
	// vanishes.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionSmoothDirectionTest,
	"DynamicRope.Traction.SmoothDirectionReseedsOnDegenerateFlip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionSmoothDirectionTest::RunTest(const FString& Parameters)
{
	const FVector X = FVector::ForwardVector;

	// Unseeded, it is seeded from the measurement with no lag.
	TestEqual(TEXT("unseeded direction seeds from the target"),
		RopeTraction::SmoothDirection(FVector::ZeroVector, X, 0.5f), X);

	// A coefficient of one reaches the target immediately.
	TestEqual(TEXT("alpha one snaps to the target"),
		RopeTraction::SmoothDirection(FVector::UpVector, X, 1.0f), X);

	// A coefficient of zero keeps the current value.
	TestEqual(TEXT("alpha zero holds the current direction"),
		RopeTraction::SmoothDirection(X, FVector::UpVector, 0.0f), X);

	// Intermediate interpolation is renormalized to a unit vector; a shortened vector would shrink every
	// dot product computed from it.
	const FVector Half = RopeTraction::SmoothDirection(X, FVector::UpVector, 0.5f);
	TestEqual(TEXT("smoothed direction stays unit length"), static_cast<float>(Half.Size()), 1.0f, 1e-4f);

	// A 180 degree reversal at a coefficient of one half cancels exactly to zero, which is where the reseed
	// is needed.
	const FVector Flipped = RopeTraction::SmoothDirection(X, -X, 0.5f);
	TestFalse(TEXT("a 180 degree flip does not collapse to zero"), Flipped.IsNearlyZero());
	TestEqual(TEXT("a degenerate flip reseeds from the target"), Flipped, -X);
	return true;
}

	// Fractional aiming interpolates linearly between nodes. This function is what makes the discrete hops
	// of integer aiming continuous and removes the stutter.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionSampleFractionalAimTest,
	"DynamicRope.Traction.SampleFractionalAimInterpolatesBetweenNodes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionSampleFractionalAimTest::RunTest(const FString& Parameters)
{
	// The nodes are evenly spaced along x, with the anchor at the last one.
	const TArray<FVector> Positions = {
		FVector(0.0f, 0.0f, 0.0f), FVector(100.0f, 0.0f, 0.0f),
		FVector(200.0f, 0.0f, 0.0f), FVector(300.0f, 0.0f, 0.0f) };

	TestEqual(TEXT("integral aim lands on the node"),
		RopeTraction::SampleFractionalAim(Positions, 1.0f, 3), FVector(100.0f, 0.0f, 0.0f));
	TestEqual(TEXT("fractional aim interpolates between nodes"),
		RopeTraction::SampleFractionalAim(Positions, 1.25f, 3), FVector(125.0f, 0.0f, 0.0f));

	// At the anchor node the upper index clamps to the anchor and returns its position, with no
	// out-of-range access.
	TestEqual(TEXT("aim at the anchor clamps to the anchor node"),
		RopeTraction::SampleFractionalAim(Positions, 3.0f, 3), FVector(300.0f, 0.0f, 0.0f));

	// An empty array or an out-of-range index returns a zero vector rather than crashing.
	TestEqual(TEXT("an empty array yields zero"),
		RopeTraction::SampleFractionalAim(TArray<FVector>(), 0.0f, 0), FVector::ZeroVector);
	return true;
}

	// The taut gate: a threshold of 0 matches the previous hardcoded gate, meaning any tension at all, and
	// leaves behaviour unchanged, while a threshold above 0 separates the engage and hold levels into a
	// hysteresis that stops it flapping at the boundary. It is shared by applying active pull and by
	// IsPullTaut().
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionTautGateTest,
	"DynamicRope.Traction.TautGateHysteresisPreventsFlapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionTautGateTest::RunTest(const FString& Parameters)
{
	// A threshold of 0, the default, matches the previous gate: any tension at all is taut and zero is not,
	// regardless of the latch state, so the hysteresis does nothing.
	TestTrue(TEXT("zero threshold treats any tension as taut"),
		RopeTraction::EvaluateTautGate(1.0f, 0.0f, 0.5f, /*bWasTaut*/ false));
	TestFalse(TEXT("zero threshold treats zero tension as slack"),
		RopeTraction::EvaluateTautGate(0.0f, 0.0f, 0.5f, /*bWasTaut*/ false));
	TestFalse(TEXT("zero tension is slack even while latched taut"),
		RopeTraction::EvaluateTautGate(0.0f, 0.0f, 0.5f, /*bWasTaut*/ true));

	// With a threshold of 100 and a release ratio of one half, engaging requires exceeding 100.
	TestFalse(TEXT("tension below the threshold does not enter taut"),
		RopeTraction::EvaluateTautGate(80.0f, 100.0f, 0.5f, /*bWasTaut*/ false));
	TestTrue(TEXT("tension above the threshold enters taut"),
		RopeTraction::EvaluateTautGate(120.0f, 100.0f, 0.5f, /*bWasTaut*/ false));

	// Hysteresis: once taut it stays taut down to half the threshold, and only below that does it release.
	TestTrue(TEXT("latched taut survives a dip below the enter threshold"),
		RopeTraction::EvaluateTautGate(80.0f, 100.0f, 0.5f, /*bWasTaut*/ true));
	TestFalse(TEXT("latched taut releases below the stay threshold"),
		RopeTraction::EvaluateTautGate(40.0f, 100.0f, 0.5f, /*bWasTaut*/ true));

	// A ratio of 1 removes the hysteresis, making the hold level equal the engage level, so 80 is slack
	// whatever the latch state.
	TestFalse(TEXT("ratio one collapses the hysteresis band"),
		RopeTraction::EvaluateTautGate(80.0f, 100.0f, 1.0f, /*bWasTaut*/ true));

	// The ratio is clamped to the range 0 to 1, so passing a value above 1 cannot raise the hold level
	// above the engage level.
	TestTrue(TEXT("an out-of-range ratio is clamped to the enter threshold"),
		RopeTraction::EvaluateTautGate(101.0f, 100.0f, 2.0f, /*bWasTaut*/ true));
	return true;
}

	// The whole-chain geometric gate: the chord sum against the rest length, plus engage and hold
	// hysteresis, plus the guard for when no verdict is possible.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionChainTautGateTest,
	"DynamicRope.Traction.ChainTautGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionChainTautGateTest::RunTest(const FString& Parameters)
{
	// With a rest length of 100 and 3 percent slack allowed, engaging requires 97.
	TestTrue(TEXT("chord at rest enters taut"),
		RopeTraction::EvaluateChainTautGate(100.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ false));
	TestTrue(TEXT("chord within the slack ratio enters taut"),
		RopeTraction::EvaluateChainTautGate(97.5f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ false));
	TestFalse(TEXT("sagging chord stays slack"),
		RopeTraction::EvaluateChainTautGate(80.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ false));

	// With hysteresis, doubling the allowance to a hold level of 94, a rope once taut stays taut even after
	// sagging slightly below the engage level.
	TestFalse(TEXT("chord just below the enter threshold does not enter"),
		RopeTraction::EvaluateChainTautGate(96.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ false));
	TestTrue(TEXT("latched taut survives a dip into the hysteresis band"),
		RopeTraction::EvaluateChainTautGate(96.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ true));
	TestFalse(TEXT("latched taut releases below the stay threshold"),
		RopeTraction::EvaluateChainTautGate(90.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ true));

	// A scale of 1 removes the hysteresis, making the engage and hold levels equal.
	TestFalse(TEXT("release scale one collapses the hysteresis band"),
		RopeTraction::EvaluateChainTautGate(96.0f, 100.0f, 0.03f, 1.0f, /*bWasTaut*/ true));

	// Guard: a rest length at or below zero, meaning no free span, is always slack. A ratio and scale whose
	// product reaches 1 converges on allowing unlimited slack, which keeps a latched gate taut regardless of
	// the chord; the cap merely prevents a negative threshold and means the same thing.
	TestFalse(TEXT("zero rest length never reports taut"),
		RopeTraction::EvaluateChainTautGate(100.0f, 0.0f, 0.03f, 2.0f, /*bWasTaut*/ true));
	TestTrue(TEXT("ratio times scale of one or more keeps the latched gate open"),
		RopeTraction::EvaluateChainTautGate(0.0f, 100.0f, 0.9f, 5.0f, /*bWasTaut*/ true));
	return true;
}

	// The second line of defence when injecting into a target: an injected vector exceeding the maximum
	// speed is clamped, while faster existing external motion, such as free fall, is preserved. The target
	// servo uses the same helper the wielder's movement correction does.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionTargetInjectSpeedCapTest,
	"DynamicRope.Traction.TargetInjectedSpeedCapPreservesExternalMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionTargetInjectSpeedCapTest::RunTest(const FString& Parameters)
{
	const float Cap = 1500.0f;
	// An injection exceeding the cap is clamped to it.
	{
		const FVector OldVel(0.0f, 0.0f, 0.0f);
		const FVector NewVel(2000.0f, 0.0f, 0.0f);
		const FVector Clamped = RopeTraction::ClampInjectedVelocity(NewVel, OldVel, Cap);
		TestEqual(TEXT("clamps the injected excess"), static_cast<float>(Clamped.Size()), Cap);
	}
	// Where the existing speed already exceeds the cap, as in free fall, that speed is permitted, so an
	// injection never reduces external motion.
	{
		const FVector OldVel(0.0f, 0.0f, -3000.0f);
		const FVector NewVel(300.0f, 0.0f, -3000.0f);
		const FVector Clamped = RopeTraction::ClampInjectedVelocity(NewVel, OldVel, Cap);
		TestTrue(TEXT("preserves faster existing external motion"), Clamped.Size() >= OldVel.Size() - 1.0f);
	}
	return true;
}

//======================================================================================
	// The tether lambda constraint solve, which is the pure maths layer of the traction rework.
//======================================================================================

namespace
{
	// The shared default input: a 10 kg target and a 100 kg wielder, full recovery gain, and no limit or
	// compliance. Each test overrides only what it needs.
	RopeTraction::FRopeTetherConstraint MakeLambdaInput(float C, float SepSpeed)
	{
		RopeTraction::FRopeTetherConstraint In;
		In.C = C;
		In.SepSpeed = SepSpeed;
		In.InvMassTarget = 1.0f / 10.0f;
		In.InvMassWielder = 1.0f / 100.0f;
		In.SettleAlpha = 1.0f;
		return In;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePointMassJacobianTest,
	"DynamicRope.Traction.PointMassJacobianMatchesAppliedImpulse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePointMassJacobianTest::RunTest(const FString& Parameters)
{
	RopeTraction::FRopePointMassProperties Body;
	Body.Mass = 10.0f;
	Body.InertiaTensor = FVector(100.0f, 200.0f, 300.0f);
	Body.MassSpaceToWorld = FTransform(
		FQuat::Identity,
		FVector(5.0f, 0.0f, 0.0f));

	TestEqual(TEXT("COM application has translational inverse mass"),
		RopeTraction::ComputePointInverseMass(
			Body,
			Body.MassSpaceToWorld.GetLocation(),
			FVector::XAxisVector),
		0.1f,
		0.0001f);

	const FVector Point = Body.MassSpaceToWorld.GetLocation() +
		FVector(0.0f, 10.0f, 0.0f);
	const float PointInvMass = RopeTraction::ComputePointInverseMass(
		Body, Point, FVector::XAxisVector);
	TestEqual(TEXT("off-COM Jacobian includes rotational inverse mass"),
		PointInvMass,
		0.1f + 100.0f / 300.0f,
		0.0001f);

	constexpr float Lambda = 50.0f;
	const FVector PointDelta = RopeTraction::ComputePointVelocityDelta(
		Body, Point, FVector::XAxisVector * Lambda);
	TestEqual(TEXT("point impulse produces lambda times point inverse mass"),
		static_cast<float>(FVector::DotProduct(
			PointDelta, FVector::XAxisVector)),
		Lambda * PointInvMass,
		0.001f);
	TestTrue(TEXT("off-COM rope impulse produces point motion"),
		!PointDelta.IsNearlyZero());
	return true;
}

	// One-directional behaviour: the rope neither pushes when slack nor brakes an approach, and with both
	// ends anchored nothing can move.
	// It also verifies the definition of lambda: the total applied, lambda times the summed inverse masses,
	// exactly closes the separation cancellation plus the commanded positional recovery.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTetherLambdaUnilateralTest,
	"DynamicRope.Traction.TetherLambdaUnilateralAndAnchors",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTetherLambdaUnilateralTest::RunTest(const FString& Parameters)
{
	const float Dt = 1.0f / 60.0f;

	// Material slack stays unloaded, but the exact boundary rejects outward motion.
	TestEqual(TEXT("returns zero when slack"), RopeTraction::SolveTetherLambda(MakeLambdaInput(-5.0f, 500.0f), Dt), 0.0f);
	{
		const RopeTraction::FRopeTetherConstraint Boundary = MakeLambdaInput(0.0f, 500.0f);
		const float Lambda = RopeTraction::SolveTetherLambda(Boundary, Dt);
		TestTrue(TEXT("produces an inextensible reaction when separating at the boundary"), Lambda > 0.0f);
		TestEqual(TEXT("the boundary reaction cancels the separation speed exactly"),
			Lambda * (Boundary.InvMassTarget + Boundary.InvMassWielder), 500.0f, 0.01f);
	}

	// Guard against a degenerate delta.
	TestEqual(TEXT("returns zero at a delta of zero"), RopeTraction::SolveTetherLambda(MakeLambdaInput(10.0f, 500.0f), 0.0f), 0.0f);

	// Already approaching faster than commanded: no braking, since coasting while slack is legitimate physics.
	{
		RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(10.0f, -120.0f);
		In.SettleAlpha = 0.2f;
		TestEqual(TEXT("returns zero when approaching at the commanded speed"), RopeTraction::SolveTetherLambda(In, Dt), 0.0f);
		In.SepSpeed = -200.0f;
		TestEqual(TEXT("does not brake an approach faster than commanded"), RopeTraction::SolveTetherLambda(In, Dt), 0.0f);
	}

	// While separating, lambda times the summed inverse masses exactly closes the cancellation plus the
	// recovery.
	{
		RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(10.0f, 300.0f);
		In.SettleAlpha = 0.2f;
		const float WSum = In.InvMassTarget + In.InvMassWielder;
		const float Lambda = RopeTraction::SolveTetherLambda(In, Dt);
		TestTrue(TEXT("produces a positive lambda while separating"), Lambda > 0.0f);
		TestEqual(TEXT("the total applied equals the cancellation plus the recovery"), Lambda * WSum, 420.0f, 0.01f);
	}

	// With both ends anchored, nothing can move; exceeding the limit is handled by the distance release.
	{
		RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(50.0f, 500.0f);
		In.InvMassTarget = 0.0f;
		In.InvMassWielder = 0.0f;
		TestEqual(TEXT("returns zero with both ends anchored"), RopeTraction::SolveTetherLambda(In, Dt), 0.0f);
	}
	return true;
}

	// Distribution is automatic: the same lambda acts at both ends, so each end's delta velocity is lambda
	// times its inverse mass, the heavier end moves less, and an anchor stays still while the other end
	// recovers everything. Both the mass-share distribution and the binary pullable yielding follow from
	// this one expression.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTetherLambdaDistributionTest,
	"DynamicRope.Traction.TetherLambdaDistributesByInverseMass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTetherLambdaDistributionTest::RunTest(const FString& Parameters)
{
	const float Dt = 1.0f / 60.0f;

	// A 10 kg target against a 100 kg wielder gives a delta velocity ratio of ten to one, with the lighter
	// end moving ten times as far, and their sum closes the commanded recovery.
	{
		const RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(30.0f, 0.0f);
		const float Lambda = RopeTraction::SolveTetherLambda(In, Dt);
		const float DvTarget = Lambda * In.InvMassTarget;
		const float DvWielder = Lambda * In.InvMassWielder;
		TestEqual(TEXT("the delta velocity ratio equals the inverse mass ratio"), DvTarget / DvWielder, 10.0f, 0.01f);
		TestEqual(TEXT("the per-end delta velocities sum to the commanded recovery"), DvTarget + DvWielder, 1800.0f, 0.1f);
	}

	// With the target anchored, as against a wall, the wielder recovers everything, and lambda never
	// exceeds the command: there is no winching, because the recovery command is bounded by the gain over
	// the delta and no constant reeling floor exists.
	{
		RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(30.0f, 0.0f);
		In.InvMassTarget = 0.0f;
		const float Lambda = RopeTraction::SolveTetherLambda(In, Dt);
		TestEqual(TEXT("the anchored end receives no delta velocity"), Lambda * In.InvMassTarget, 0.0f);
		TestEqual(TEXT("the wielder recovers everything"), Lambda * In.InvMassWielder, 1800.0f, 0.1f);
	}
	return true;
}

	// The contracts of the three limits: the maximum tension, which makes a heavy target lag; the
	// compliance, which is deliberate elasticity; and the maximum bias speed, which caps the commanded
	// positional recovery alone and never the separation cancellation.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTetherLambdaCapsTest,
	"DynamicRope.Traction.TetherLambdaHonorsTensionComplianceAndBiasCaps",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTetherLambdaCapsTest::RunTest(const FString& Parameters)
{
	const float Dt = 1.0f / 60.0f;

	// The tension limit caps lambda at the maximum tension times the delta. The shortfall carries over as
	// violation into the next frame, which is what makes a heavy target lag, and is physical.
	{
		RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(30.0f, 0.0f);
		In.MaxTension = 60000.0f;
		TestEqual(TEXT("clamps at the tension limit"), RopeTraction::SolveTetherLambda(In, Dt), 60000.0f * Dt, 0.01f);
	}

	// Compliance is an inverse stiffness and solves the Kelvin-Voigt state implicitly.
	// Zero is inextensible and closes exactly, while a positive value reduces lambda through a stable
	// elastic response.
	{
		RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(30.0f, 0.0f);
		In.InvMassTarget = 0.0f; // The wielder alone, which simplifies the numbers.
		const float Rigid = RopeTraction::SolveTetherLambda(In, Dt);
		TestEqual(TEXT("the inextensible lambda"), Rigid, 1800.0f / 0.01f, 0.5f);
		In.Compliance = 0.0005f;
		const float Soft = RopeTraction::SolveTetherLambda(In, Dt);
		const float W = In.InvMassWielder;
		const float R = 2.0f * FMath::Sqrt(In.Compliance / W);
		const float ExpectedTension =
			In.C / (In.Compliance + W * Dt * (R + Dt));
		TestEqual(TEXT("the elastic lambda"), Soft, ExpectedTension * Dt, 0.5f);
		TestTrue(TEXT("elasticity reduces lambda"), Soft < Rigid);
	}

	// The bias cap limits the commanded positional recovery alone and leaves the separation cancellation
	// untouched: cancellation is momentum made real, since it stops an actual separation, while only the
	// bias can survive as coasting, which is why it is limited separately.
	{
		RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(100.0f, 0.0f);
		In.InvMassTarget = 0.0f;
		In.MaxBiasSpeed = 400.0f;
		TestEqual(TEXT("the recovery command is limited by the cap"),
			RopeTraction::SolveTetherLambda(In, Dt) * In.InvMassWielder, 400.0f, 0.1f);
		In.SepSpeed = 500.0f;
		TestEqual(TEXT("the separation cancellation is added on top of the cap"),
			RopeTraction::SolveTetherLambda(In, Dt) * In.InvMassWielder, 900.0f, 0.1f);
	}
	return true;
}

	// Convergence, as a miniature integration: starting from a violation with both ends free, lambda fires
	// on the first frame only, with no re-slam; the violation decreases monotonically past the boundary into
	// slack; and the residual approach speed never exceeds the first frame's recovery command.
	// It is the regression test pinning the absence of the old system's coast-then-retighten oscillation,
	// which came from per-end servos plus constant reeling.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTetherLambdaConvergenceTest,
	"DynamicRope.Traction.TetherLambdaConvergesWithoutOscillation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTetherLambdaConvergenceTest::RunTest(const FString& Parameters)
{
	const float Dt = 1.0f / 60.0f;

	auto Simulate = [&](float MaxBiasSpeed, float& OutFinalSep, int32& OutFramesWithLambda, bool& bOutMonotonic)
	{
		float C = 50.0f;
		float Sep = 0.0f; // Starting from rest.
		OutFramesWithLambda = 0;
		bOutMonotonic = true;
		for (int32 Frame = 0; Frame < 120; ++Frame)
		{
			RopeTraction::FRopeTetherConstraint In = MakeLambdaInput(C, Sep);
			In.SettleAlpha = 0.2f;
			In.MaxBiasSpeed = MaxBiasSpeed;
			const float Lambda = RopeTraction::SolveTetherLambda(In, Dt);
			if (Lambda > 0.0f)
			{
				++OutFramesWithLambda;
			}
			// With free bodies the impulse pair translates directly into relative approach, using the summed
			// inverse mass, and the velocity persists into the next frame.
			Sep -= Lambda * (In.InvMassTarget + In.InvMassWielder);
			const float NewC = C + Sep * Dt;
			bOutMonotonic &= (NewC <= C + KINDA_SMALL_NUMBER);
			C = NewC;
		}
		OutFinalSep = Sep;
		return C;
	};

	// With no limit the first frame's command is the gain times the violation over the delta. After that the
	// coasting outruns the command and lambda never fires again.
	{
		float FinalSep = 0.0f;
		int32 FramesWithLambda = 0;
		bool bMonotonic = false;
		const float FinalC = Simulate(0.0f, FinalSep, FramesWithLambda, bMonotonic);
		TestEqual(TEXT("lambda fires on the first frame only, with no re-slam"), FramesWithLambda, 1);
		TestTrue(TEXT("the violation decreases monotonically, with no bounce back"), bMonotonic);
		TestTrue(TEXT("it reaches the boundary and goes slack"), FinalC <= 0.0f);
		TestEqual(TEXT("the residual approach equals the first frame's recovery command"), FinalSep, -600.0f, 0.5f);
	}

	// With a bias cap the residual coasting approach is bounded by exactly that cap, which is the contract
	// that the cap is the maximum approach speed.
	{
		float FinalSep = 0.0f;
		int32 FramesWithLambda = 0;
		bool bMonotonic = false;
		const float FinalC = Simulate(200.0f, FinalSep, FramesWithLambda, bMonotonic);
		TestTrue(TEXT("it still reaches the boundary under the cap"), FinalC <= 0.0f);
		TestTrue(TEXT("the violation decreases monotonically under the cap"), bMonotonic);
		TestEqual(TEXT("the residual approach is bounded by the bias cap"), FinalSep, -200.0f, 0.5f);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
