// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Core/RopeMovementConstraint.h"
#include "RopeTestHelpers.h"
#include "Solver/RopeXPBDSolver.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeHardLeashProjectionTest,
	"DynamicRope.Movement.HardLeash.Projection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeHardLeashProjectionTest::RunTest(const FString& Parameters)
{
	const FVector Pivot(100.0f, 0.0f, 0.0f);
	constexpr float Radius = 100.0f;

	const RopeMovementConstraint::FProjectionResult Outward =
		RopeMovementConstraint::ProjectPoint(FVector(-25.0f, 0.0f, 0.0f), Pivot, Radius);
	TestTrue(TEXT("outward crossing is constrained"), Outward.bConstrained);
	TestTrue(TEXT("outward crossing stops exactly at material radius"),
		Outward.Position.Equals(FVector::ZeroVector, 0.001f));

	const RopeMovementConstraint::FProjectionResult Inward =
		RopeMovementConstraint::ProjectPoint(FVector(25.0f, 0.0f, 0.0f), Pivot, Radius);
	TestFalse(TEXT("slack/inward point is not constrained"), Inward.bConstrained);
	TestTrue(TEXT("slack/inward point is unchanged"),
		Inward.Position.Equals(FVector(25.0f, 0.0f, 0.0f), 0.001f));

	const RopeMovementConstraint::FProjectionResult Tangent =
		RopeMovementConstraint::ProjectPoint(FVector(0.0f, 25.0f, 0.0f), Pivot, Radius);
	TestTrue(TEXT("tangent crossing receives the required inward projection"), Tangent.bConstrained);
	TestTrue(TEXT("tangent projection remains on the sphere"),
		FMath::IsNearlyEqual(
			static_cast<float>(FVector::Distance(Tangent.Position, Pivot)), Radius, 0.001f));
	TestTrue(TEXT("most tangential displacement is retained"), Tangent.Position.Y > 24.0f);

	const FVector PreservedInward = RopeMovementConstraint::RemoveOutwardVelocity(
		FVector(-100.0f, 20.0f, 0.0f), FVector::ZeroVector, FVector::XAxisVector);
	TestTrue(TEXT("inward/tangent velocity is preserved"),
		PreservedInward.Equals(FVector(-100.0f, 20.0f, 0.0f), 0.001f));
	const FVector RemovedOutward = RopeMovementConstraint::RemoveOutwardVelocity(
		FVector(100.0f, 20.0f, 0.0f), FVector::ZeroVector, FVector::XAxisVector);
	TestTrue(TEXT("only outward velocity is removed"),
		RemovedOutward.Equals(FVector(0.0f, 20.0f, 0.0f), 0.001f));

	const FVector MovingPivotResult = RopeMovementConstraint::RemoveOutwardVelocity(
		FVector(150.0f, 20.0f, 0.0f), FVector(100.0f, 0.0f, 0.0f),
		FVector::XAxisVector);
	TestTrue(TEXT("velocity removal preserves the moving pivot's carried velocity"),
		MovingPivotResult.Equals(FVector(100.0f, 20.0f, 0.0f), 0.001f));

	TestEqual(TEXT("rejected boundary velocity becomes separating-speed load"),
		RopeMovementConstraint::ComputeRejectedSeparatingSpeed(
			FVector(600.0f, 20.0f, 0.0f),
			FVector(0.0f, 20.0f, 0.0f),
			FVector::XAxisVector,
			0.0f,
			1.0f / 60.0f),
		600.0f,
		0.001f);
	TestEqual(TEXT("position-only crossing also becomes separating-speed load"),
		RopeMovementConstraint::ComputeRejectedSeparatingSpeed(
			FVector::ZeroVector,
			FVector::ZeroVector,
			FVector::XAxisVector,
			10.0f,
			1.0f / 60.0f),
		600.0f,
		0.001f);

	// Reeling 10 cm in one 60 Hz frame produces a 10 cm projection and therefore a
	// 600 cm/s rejected-position rate. MaterialLengthRate is already represented by
	// that projection and must not be subtracted a second time.
	TestEqual(TEXT("hard projection does not double-count reel-in rate"),
		RopeMovementConstraint::ComputeConstraintSeparatingSpeed(
			/*bHardProjectedAttempt*/ true,
			/*RejectedSeparatingSpeed*/ 600.0f,
			/*EndpointSeparatingSpeed*/ 0.0f,
			/*MaterialLengthRate*/ -600.0f),
		600.0f,
		0.001f);
	TestEqual(TEXT("newer live target separation can exceed the recorded hard attempt"),
		RopeMovementConstraint::ComputeConstraintSeparatingSpeed(
			/*bHardProjectedAttempt*/ true,
			/*RejectedSeparatingSpeed*/ 600.0f,
			/*EndpointSeparatingSpeed*/ 800.0f,
			/*MaterialLengthRate*/ 0.0f),
		800.0f,
		0.001f);
	TestEqual(TEXT("analytic path still includes material shortening rate"),
		RopeMovementConstraint::ComputeConstraintSeparatingSpeed(
			/*bHardProjectedAttempt*/ false,
			/*RejectedSeparatingSpeed*/ 0.0f,
			/*EndpointSeparatingSpeed*/ 0.0f,
			/*MaterialLengthRate*/ -600.0f),
		600.0f,
		0.001f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeHardLeashFixedEndpointsTest,
	"DynamicRope.Movement.HardLeash.FixedEndpointsRemainFeasible",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeHardLeashFixedEndpointsTest::RunTest(const FString& Parameters)
{
	auto MakePinnedSpan = [](float StartX) -> FRopeSimState
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(4, 60.0f);
		Sim.Positions[0] = FVector(StartX, 0.0f, 0.0f);
		Sim.PrevPositions[0] = Sim.Positions[0];
		Sim.bStartPinned = true;
		Sim.StartPinPrev = Sim.Positions[0];
		Sim.StartPinTarget = Sim.Positions[0];
		Sim.InvMass[0] = 0.0f;
		Sim.InvMass[3] = 0.0f;
		return Sim;
	};

	FRopeSolverConfig Config;
	Config.Substeps = 4;
	Config.Iterations = 8;
	Config.StretchCompliance = 0.0f;
	Config.BendCompliance = 0.0f;
	Config.Gravity = FVector::ZeroVector;
	Config.Damping = 0.0f;
	Config.MaxStretchRatio = 1.0f;
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;

	const RopeMovementConstraint::FProjectionResult Projected =
		RopeMovementConstraint::ProjectPoint(
			FVector(-30.0f, 0.0f, 0.0f), FVector(60.0f, 0.0f, 0.0f), 60.0f);
	FRopeSimState Feasible = MakePinnedSpan(Projected.Position.X);
	Solver.Step(Feasible, Config, NoColliders, 1.0f / 60.0f);
	TestTrue(TEXT("projected fixed endpoints have a feasible rest-length solution"),
		RopeTest::MaxSegmentError(Feasible) <= 0.01f);

	FRopeSimState Impossible = MakePinnedSpan(-30.0f);
	for (int32 Frame = 0; Frame < 10; ++Frame)
	{
		Solver.Step(Impossible, Config, NoColliders, 1.0f / 60.0f);
	}
	TestTrue(TEXT("unprojected fixed endpoints retain irreducible stretch"),
		RopeTest::MaxSegmentError(Impossible) >= 9.9f);
	return true;
}

#endif
