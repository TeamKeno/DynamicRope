// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Core/RopeLengthConstraintState.h"
#include "Logic/RopeLengthConstraintSolver.h"

namespace
{
	RopeLengthConstraint::FInput MakeRigidLengthInput(float C, float Cdot)
	{
		RopeLengthConstraint::FInput In;
		In.Violation = C;
		In.SeparatingSpeed = Cdot;
		In.EffectiveInverseMass = 1.0f / 10.0f + 1.0f / 100.0f;
		In.SettleAlpha = 1.0f;
		return In;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeLengthConstraintBoundaryReactionTest,
	"DynamicRope.Constraint.Length.BoundaryReactionWithoutStretch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeLengthConstraintBoundaryReactionTest::RunTest(const FString& Parameters)
{
	constexpr float Dt = 1.0f / 60.0f;

	const RopeLengthConstraint::FResult Outward =
		RopeLengthConstraint::Solve(MakeRigidLengthInput(0.0f, 600.0f), Dt);
	TestTrue(TEXT("exact material boundary rejects outward motion"), Outward.bActive);
	TestTrue(TEXT("rigid cable has non-zero tension without extension"), Outward.Tension > 0.0f);
	TestEqual(TEXT("reaction cancels separating speed"),
		Outward.Lambda * (1.0f / 10.0f + 1.0f / 100.0f), 600.0f, 0.01f);

	const RopeLengthConstraint::FResult Inward =
		RopeLengthConstraint::Solve(MakeRigidLengthInput(0.0f, -100.0f), Dt);
	TestFalse(TEXT("rope never opposes inward motion"), Inward.bActive);
	TestEqual(TEXT("inward boundary motion has no tension"), Inward.Tension, 0.0f);

	const RopeLengthConstraint::FResult Slack =
		RopeLengthConstraint::Solve(MakeRigidLengthInput(-5.0f, 600.0f), Dt);
	TestFalse(TEXT("material slack does not transmit load"), Slack.bActive);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeLengthConstraintComplianceAndSlopTest,
	"DynamicRope.Constraint.Length.ComplianceAndActivationSlop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeLengthConstraintComplianceAndSlopTest::RunTest(const FString& Parameters)
{
	constexpr float Dt = 1.0f / 60.0f;

	RopeLengthConstraint::FInput Rigid = MakeRigidLengthInput(2.0f, 100.0f);
	const RopeLengthConstraint::FResult RigidResult =
		RopeLengthConstraint::Solve(Rigid, Dt);
	Rigid.Compliance = 0.0005f;
	const RopeLengthConstraint::FResult ElasticResult =
		RopeLengthConstraint::Solve(Rigid, Dt);
	TestTrue(TEXT("compliance reduces reaction instead of defining whether tension exists"),
		ElasticResult.Lambda < RigidResult.Lambda);

	RopeLengthConstraint::FInput Near = MakeRigidLengthInput(-0.25f, 100.0f);
	Near.ActivationSlop = 0.5f;
	TestTrue(TEXT("activation slop stabilizes a near-boundary outward attempt"),
		RopeLengthConstraint::Solve(Near, Dt).bActive);
	Near.Violation = -0.75f;
	TestFalse(TEXT("activation slop does not become physical rope length"),
		RopeLengthConstraint::Solve(Near, Dt).bActive);

	RopeLengthConstraint::FInput Capped = MakeRigidLengthInput(0.0f, 1000.0f);
	Capped.MaxTension = 60000.0f;
	TestEqual(TEXT("reaction force cap remains lambda <= tension * dt"),
		RopeLengthConstraint::Solve(Capped, Dt).Lambda, 60000.0f * Dt, 0.01f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeLengthConstraintFrameRateTest,
	"DynamicRope.Constraint.Length.ReactionForceIsFrameRateIndependent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeLengthConstraintFrameRateTest::RunTest(const FString& Parameters)
{
	// The same acceleration produces a rejected speed a*dt. Since T=lambda/dt,
	// the reported constraint force must be invariant across common frame rates.
	constexpr float Acceleration = 36000.0f;
	float ReferenceTension = -1.0f;
	for (const float Hz : { 30.0f, 60.0f, 120.0f })
	{
		const float Dt = 1.0f / Hz;
		RopeLengthConstraint::FInput In = MakeRigidLengthInput(
			0.0f, Acceleration * Dt);
		const RopeLengthConstraint::FResult Result =
			RopeLengthConstraint::Solve(In, Dt);
		TestTrue(FString::Printf(TEXT("%.0f Hz boundary reaction is active"), Hz),
			Result.bActive);
		if (ReferenceTension < 0.0f)
		{
			ReferenceTension = Result.Tension;
		}
		else
		{
			TestEqual(
				FString::Printf(TEXT("%.0f Hz reports the same reaction force"), Hz),
				Result.Tension,
				ReferenceTension,
				0.1f);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeLengthConstraintCompliantStabilityTest,
	"DynamicRope.Constraint.Length.CompliantDampingIsStable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeLengthConstraintCompliantStabilityTest::RunTest(const FString& Parameters)
{
	// At the exact boundary, damping may reduce an outward velocity but must not
	// explicitly overshoot and reverse it at a long (30 Hz) frame.
	for (const float Hz : { 30.0f, 60.0f, 120.0f })
	{
		const float Dt = 1.0f / Hz;
		RopeLengthConstraint::FInput In = MakeRigidLengthInput(
			/*C*/ 0.0f,
			/*Cdot*/ 100.0f);
		In.Compliance = 0.0002f;
		const RopeLengthConstraint::FResult Result =
			RopeLengthConstraint::Solve(In, Dt);
		TestTrue(FString::Printf(TEXT("%.0f Hz compliant reaction is active"), Hz),
			Result.bActive);
		const float PostSpeed =
			In.SeparatingSpeed -
			In.EffectiveInverseMass * Result.Lambda;
		TestTrue(
			FString::Printf(TEXT("%.0f Hz damping never reverses outward velocity"), Hz),
			PostSpeed >= -0.001f);
		TestTrue(
			FString::Printf(TEXT("%.0f Hz damping reduces outward velocity"), Hz),
			PostSpeed < In.SeparatingSpeed);
		TestEqual(
			FString::Printf(TEXT("%.0f Hz reports the applied implicit force"), Hz),
			Result.Lambda,
			Result.Tension * Dt,
			0.01f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeLengthConstraintCompliantEquilibriumTest,
	"DynamicRope.Constraint.Length.CompliantSteadyLoadIsFrameRateIndependent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeLengthConstraintCompliantEquilibriumTest::RunTest(const FString& Parameters)
{
	// One second under a constant outward load must converge to Hooke equilibrium
	// C=alpha*F at common game rates. This exercises the applied dynamics, unlike
	// comparing an unintegrated force formula at one frozen state.
	constexpr float Compliance = 0.0002f;
	constexpr float ExternalForce = 5000.0f;
	constexpr float ExpectedExtension = Compliance * ExternalForce;
	for (const float Hz : { 30.0f, 60.0f, 120.0f })
	{
		const float Dt = 1.0f / Hz;
		float C = 0.0f;
		float Cdot = 0.0f;
		float LastTension = 0.0f;
		for (int32 Step = 0; Step < FMath::RoundToInt(Hz); ++Step)
		{
			RopeLengthConstraint::FInput In = MakeRigidLengthInput(C, Cdot);
			In.Compliance = Compliance;
			Cdot += In.EffectiveInverseMass * ExternalForce * Dt;
			In.SeparatingSpeed = Cdot;
			const RopeLengthConstraint::FResult Result =
				RopeLengthConstraint::Solve(In, Dt);
			Cdot -= In.EffectiveInverseMass * Result.Lambda;
			C += Cdot * Dt;
			LastTension = Result.Tension;
		}

		TestEqual(
			FString::Printf(TEXT("%.0f Hz reaches Hooke extension"), Hz),
			C,
			ExpectedExtension,
			0.001f);
		TestEqual(
			FString::Printf(TEXT("%.0f Hz settles relative speed"), Hz),
			Cdot,
			0.0f,
			0.001f);
		TestEqual(
			FString::Printf(TEXT("%.0f Hz balances the external load"), Hz),
			LastTension,
			ExternalForce,
			0.5f);
	}

	RopeLengthConstraint::FInput Capped = MakeRigidLengthInput(100.0f, 1000.0f);
	Capped.Compliance = Compliance;
	Capped.MaxTension = 5000.0f;
	constexpr float Dt = 1.0f / 60.0f;
	const RopeLengthConstraint::FResult CappedResult =
		RopeLengthConstraint::Solve(Capped, Dt);
	TestEqual(TEXT("compliant force cap is authoritative"),
		CappedResult.Tension, Capped.MaxTension, 0.01f);
	TestEqual(TEXT("compliant capped impulse integrates the capped force"),
		CappedResult.Lambda, Capped.MaxTension * Dt, 0.01f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeLengthConstraintFrameStateTest,
	"DynamicRope.Constraint.Length.ReactionStateExpiresPerFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeLengthConstraintFrameStateTest::RunTest(const FString& Parameters)
{
	FRopeLengthConstraintState State;
	State.RecordWielderAttempt(
		/*Frame*/ 42,
		/*AnchorNode*/ 3,
		/*Violation*/ 10.0f,
		/*SeparatingSpeed*/ 600.0f,
		/*OutwardNormal*/ FVector::XAxisVector,
		/*bAtLimit*/ true);
	TestTrue(TEXT("recorded hard reaction is visible only to its matching frame/anchor"),
		State.HasWielderAttempt(42, 3));
	TestFalse(TEXT("hard reaction cannot leak into the next frame"),
		State.HasWielderAttempt(43, 3));
	TestFalse(TEXT("hard reaction cannot leak across a topology change"),
		State.HasWielderAttempt(42, 4));

	State.LastLambda = 1000.0f;
	State.LastLambdaDt = 1.0f / 60.0f;
	State.Backend = ERopeLengthConstraintBackend::HardReaction;
	State.BeginFrame(1.0f / 120.0f);
	TestEqual(TEXT("new frame clears stale reaction impulse"), State.LastLambda, 0.0f);
	TestEqual(TEXT("new frame owns its own tension dt"), State.LastLambdaDt, 1.0f / 120.0f);
	TestEqual(TEXT("new frame starts without a selected backend"),
		State.Backend, ERopeLengthConstraintBackend::None);
	return true;
}

#endif
