// Copyright 2026 TeamKeno. All Rights Reserved.
//
// FRopeWhipGuide and whip-path math unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWhipGuide.h"
#include "RopeMathHelpers.h"
#include "Solver/RopeXPBDSolver.h"
#include "RopeTestHelpers.h"

// Whether an aim-hit guide holds the middle alone and leaves the solver state at both ends.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimHitEndpointSolverBlendTest,
	"DynamicRope.Solver.AimHitEndpointSolverBlend",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimHitEndpointSolverBlendTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(21, 200.0f);
	// A moved hand pin is set up so that following the socket at the root end and guiding the middle apply together.
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0] + FVector(0.0f, 25.0f, 0.0f);
	Sim.InvMass[0] = 0.0f;

	FRopeWhipGuide::FConfig Config;
	Config.Duration = 0.5f;
	Config.SweepAngleDegrees = 120.0f;
	Config.ComponentRopeLength = Sim.RopeLength;
	Config.AimHitRootSolverFraction = 0.20f;
	Config.AimHitTipSolverFraction = 0.25f;

	FRopeWhipGuide Guide;
	Guide.Begin(FVector::ForwardVector, Sim.Positions[0], FVector::ForwardVector,
		FVector::UpVector, FVector::RightVector, 1500.0f, FVector::ZeroVector,
		/*bHasAimTarget*/ true, FVector(100.0f, 0.0f, 0.0f), 0.25f, 0.50f);
	const FVector TipBeforeSnap = Sim.Positions.Last();
	Guide.SnapToInitialPose(Sim, Config);
	TestTrue(TEXT("initial aim snap preserves the solver-owned tip"),
		Sim.Positions.Last().Equals(TipBeforeSnap, 0.01f));

	const int32 LastNode = Sim.Num() - 1;
	const int32 MiddleNode = LastNode / 2;
	// The free end is moved off the spline to verify that Advance does not overwrite the end node again.
	const FVector FreeTipBefore(200.0f, 40.0f, -15.0f);
	Sim.Positions[LastNode] = FreeTipBefore;
	Sim.PrevPositions[LastNode] = FreeTipBefore;
	Guide.Advance(1.0f / 60.0f, Sim, Config);

	TestTrue(TEXT("middle node remains spline-guided"), Guide.IsGuidedNodeThisFrame(MiddleNode));
	TestFalse(TEXT("tip node is released to solver"), Guide.IsGuidedNodeThisFrame(LastNode));
	TestTrue(TEXT("released tip keeps solver position before solve"),
		Guide.GetCurrentTargets().IsValidIndex(LastNode) &&
		Guide.GetCurrentTargets()[LastNode].Equals(FreeTipBefore, 0.01f));

	TArray<int32> DebugNodeIndices;
	TArray<FVector> DebugTargets;
	Guide.CopyGuidedTargetsForDebug(DebugNodeIndices, DebugTargets);
	TestEqual(TEXT("debug extraction reads the gameplay guided-node count"),
		DebugNodeIndices.Num(), Guide.GetGuidedNodeCountThisFrame());
	TestEqual(TEXT("debug node and target arrays stay paired"), DebugTargets.Num(), DebugNodeIndices.Num());
	for (int32 DebugIndex = 0; DebugIndex < DebugNodeIndices.Num(); ++DebugIndex)
	{
		const int32 NodeIndex = DebugNodeIndices[DebugIndex];
		TestTrue(TEXT("debug extraction contains only gameplay-guided nodes"),
			Guide.IsGuidedNodeThisFrame(NodeIndex));
		TestTrue(TEXT("debug target is the gameplay target"),
			Guide.GetCurrentTargets().IsValidIndex(NodeIndex) &&
			Guide.GetCurrentTargets()[NodeIndex].Equals(DebugTargets[DebugIndex], 0.01f));
	}
	return true;
}

// Interpolating the aim envelope per node after resampling to the segment length can widen the spacing again across
// the range where the weight changes. Whether the same inextensibility contract holds all the way to the final
// current, previous and predicted targets carried on the GPU override.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimHitGuideSegmentSpacingTest,
	"DynamicRope.Solver.AimHitGuidePreservesSegmentSpacing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimHitGuideSegmentSpacingTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(21, 200.0f);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0] + FVector(0.0f, 25.0f, 0.0f);
	Sim.InvMass[0] = 0.0f;

	FRopeWhipGuide::FConfig Config;
	Config.Duration = 0.5f;
	Config.SweepAngleDegrees = 120.0f;
	Config.ComponentRopeLength = Sim.RopeLength;
	Config.AimHitRootSolverFraction = 0.20f;
	Config.AimHitTipSolverFraction = 0.25f;

	FRopeWhipGuide Guide;
	Guide.Begin(FVector::ForwardVector, Sim.Positions[0], FVector::ForwardVector,
		FVector::UpVector, FVector::RightVector, 1500.0f, FVector::ZeroVector,
		/*bHasAimTarget*/ true, FVector(100.0f, 0.0f, 0.0f), 0.25f, 0.50f);
	const FVector TipBeforeSnap = Sim.Positions.Last();
	Guide.SnapToInitialPose(Sim, Config);
	TestTrue(TEXT("spacing repair does not teleport the solver-owned tip"),
		Sim.Positions.Last().Equals(TipBeforeSnap, 0.01f));

	const auto CheckGuidedSpacing = [this, &Guide, &Sim](
		const TCHAR* Stage, const TArray<FVector>& Positions, const FVector& Root)
	{
		bool bCheckedAny = false;
		for (int32 NodeIndex = 1; NodeIndex < Positions.Num(); ++NodeIndex)
		{
			if (!Guide.IsGuidedNodeThisFrame(NodeIndex))
			{
				continue;
			}

			bCheckedAny = true;
			const FVector Leader = NodeIndex == 1 ? Root : Positions[NodeIndex - 1];
			const float Distance = FVector::Dist(Leader, Positions[NodeIndex]);
			TestTrue(*FString::Printf(TEXT("%s guided edge %d-%d stays within spacing (%.3f <= %.3f)"),
				Stage, NodeIndex - 1, NodeIndex, Distance, Sim.SegmentLength),
				Distance <= Sim.SegmentLength + 0.05f);
		}
		TestTrue(*FString::Printf(TEXT("%s checks at least one guided edge"), Stage), bCheckedAny);
	};

	CheckGuidedSpacing(TEXT("initial snap"), Sim.Positions, Sim.StartPinTarget);
	Guide.Advance(1.0f / 60.0f, Sim, Config);
	CheckGuidedSpacing(TEXT("current target"), Guide.GetCurrentTargets(), Sim.StartPinTarget);
	CheckGuidedSpacing(TEXT("previous target"), Guide.GetPrevTargets(), Sim.StartPinPrev);

	float MaxGuideMotion = 0.0f;
	for (int32 NodeIndex = 1; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		if (Guide.IsGuidedNodeThisFrame(NodeIndex) && Guide.GetCurrentTargets().IsValidIndex(NodeIndex))
		{
			MaxGuideMotion = FMath::Max(MaxGuideMotion,
				FVector::Dist(Sim.Positions[NodeIndex], Guide.GetCurrentTargets()[NodeIndex]));
		}
	}
	TestTrue(*FString::Printf(TEXT("spacing repair keeps meaningful guide motion (%.3f cm)"), MaxGuideMotion),
		MaxGuideMotion > 1.0f);

	TArray<FVector> PreviewTargets;
	Guide.PreviewNextTargets(1.0f / 60.0f, Sim, Config, PreviewTargets);
	CheckGuidedSpacing(TEXT("predictive target"), PreviewTargets, Sim.StartPinTarget);

	// The real strain limit contract shared by the CPU and the GPU, where the phase policy passes one, removes it
	// across the whole resident pose including the boundary between the targets and the free tail. The same solver
	// result is verified through the CPU mirror.
	FRopeSimState Solved = Sim;
	Guide.ApplyToSim(Solved);
	FRopeSolverConfig SolverConfig;
	SolverConfig.Substeps = 1;
	SolverConfig.Iterations = 1;
	SolverConfig.StretchCompliance = 0.0f;
	SolverConfig.BendCompliance = 0.0f;
	SolverConfig.Gravity = FVector::ZeroVector;
	SolverConfig.Damping = 0.0f;
	SolverConfig.MaxStretchRatio = 1.0f;
	TArray<IRopeCollider*> NoColliders;
	FRopeXPBDSolver Solver;
	Solver.Step(Solved, SolverConfig, NoColliders, 1.0f / 60.0f);
	for (int32 NodeIndex = 0; NodeIndex + 1 < Solved.Num(); ++NodeIndex)
	{
		const float Distance = FVector::Dist(Solved.Positions[NodeIndex], Solved.Positions[NodeIndex + 1]);
		TestTrue(*FString::Printf(TEXT("post-solve edge %d-%d is non-stretched (%.3f <= %.3f)"),
			NodeIndex, NodeIndex + 1, Distance, Solved.SegmentLength),
			Distance <= Solved.SegmentLength + 0.05f);
	}
	return true;
}

// An ordinary FullSimulation whip seeds the whole rope onto one straight guide at the throw boundary, then releases
// the future solver-owned tail over normalized swing time instead of preserving a folded pose behind the guide.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWhipInitialStraightSeedAndReleaseTest,
	"DynamicRope.Solver.WhipInitialStraightSeedAndGradualRelease",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWhipInitialStraightSeedAndReleaseTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(21, 200.0f);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;

	FRopeWhipGuide::FConfig Config;
	Config.Duration = 1.0f;
	Config.ReferenceThrowSpeed = 1500.0f;
	Config.GuidedLength = 0.65f;
	Config.SweepAngleDegrees = 180.0f;
	Config.ComponentRopeLength = Sim.RopeLength;

	FRopeWhipGuide Guide;
	Guide.Begin(FVector::ForwardVector, Sim.Positions[0], FVector::ForwardVector,
		FVector::UpVector, FVector::RightVector, 1500.0f, FVector::ZeroVector);
	// Reproduce the problematic source pose: the chain goes away from the hand and its second half
	// returns, as after hanging or wrapping. The initial seed must discard this shape completely.
	for (int32 NodeIndex = 1; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		const float S = static_cast<float>(NodeIndex) / static_cast<float>(Sim.Num() - 1);
		Sim.Positions[NodeIndex] = FVector(100.0f * FMath::Sin(S * PI), 0.0f, -80.0f * FMath::Sin(S * PI));
		Sim.PrevPositions[NodeIndex] = Sim.Positions[NodeIndex];
	}
	Guide.SnapToInitialPose(Sim, Config);

	const int32 LastNode = Sim.Num() - 1;
	for (int32 NodeIndex = 0; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		const FVector Expected = Sim.StartPinTarget - FVector::ForwardVector *
			(Sim.SegmentLength * static_cast<float>(NodeIndex));
		TestTrue(*FString::Printf(TEXT("initial node %d is seeded on the full straight guide"), NodeIndex),
			Sim.Positions[NodeIndex].Equals(Expected, 0.01f));
	}
	TestTrue(TEXT("initial straight seed includes the future solver tail"),
		Guide.IsGuidedNodeThisFrame(LastNode));

	// Halfway through the swing the ownership boundary has moved from 100% to 82.5%. Nodes on its
	// guide side must remain exactly collinear; the released tail must remain completely solver-owned.
	Guide.Advance(0.5f, Sim, Config);
	const int32 HalfTimeLastGuidedNode = 16; // 16 / 20 = 0.80, below the 0.825 boundary.
	for (int32 NodeIndex = 1; NodeIndex <= HalfTimeLastGuidedNode; ++NodeIndex)
	{
		const FVector Expected = FVector::UpVector *
			(Sim.SegmentLength * static_cast<float>(NodeIndex));
		TestTrue(*FString::Printf(TEXT("half-time guided node %d stays on one straight line"), NodeIndex),
			Guide.GetCurrentTargets()[NodeIndex].Equals(Expected, 0.01f));
	}
	TestTrue(TEXT("node below the moving boundary remains guide-owned"),
		Guide.IsGuidedNodeThisFrame(HalfTimeLastGuidedNode));
	TestFalse(TEXT("node above the moving boundary is fully solver-owned"),
		Guide.IsGuidedNodeThisFrame(HalfTimeLastGuidedNode + 1));
	TestTrue(TEXT("released node is not position-blended with the rotating guide"),
		Guide.GetCurrentTargets()[HalfTimeLastGuidedNode + 1].Equals(
			Sim.Positions[HalfTimeLastGuidedNode + 1], 0.01f));

	Guide.Advance(0.5f, Sim, Config);
	TestFalse(TEXT("tail is fully released when the normalized swing completes"),
		Guide.IsGuidedNodeThisFrame(LastNode));
	return true;
}

// GuidedLength=1 is an explicit request to avoid a moving ownership boundary during flight. It must not
// be silently reduced to 0.95, because even that small reduction releases tail nodes one at a time.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWhipFullGuidedLengthTest,
	"DynamicRope.Solver.WhipFullGuidedLengthKeepsWholeRopeGuided",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWhipFullGuidedLengthTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(21, 200.0f);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;

	FRopeWhipGuide::FConfig Config;
	Config.Duration = 1.0f;
	Config.ReferenceThrowSpeed = 1500.0f;
	Config.GuidedLength = 1.0f;
	Config.SweepAngleDegrees = 180.0f;
	Config.ComponentRopeLength = Sim.RopeLength;

	FRopeWhipGuide Guide;
	Guide.Begin(FVector::ForwardVector, Sim.Positions[0], FVector::ForwardVector,
		FVector::UpVector, FVector::RightVector, 1500.0f, FVector::ZeroVector);
	Guide.SnapToInitialPose(Sim, Config);

	for (int32 StepIndex = 0; StepIndex < 4; ++StepIndex)
	{
		Guide.Advance(0.2f, Sim, Config);
		TestTrue(*FString::Printf(TEXT("step %d keeps the tail hard-guided"), StepIndex),
			Guide.IsGuidedNodeThisFrame(Sim.Num() - 1));
		TestEqual(*FString::Printf(TEXT("step %d keeps every movable node hard-guided"), StepIndex),
			Guide.GetGuidedNodeCountThisFrame(), Sim.Num() - 1);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimHitSweepingLineGuideTest,
	"DynamicRope.Solver.AimHitSweepingLineGuide",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimHitSweepingLineGuideTest::RunTest(const FString& Parameters)
{
	const FVector Origin(10.0f, -20.0f, 30.0f);
	const FVector AimDirection = FVector::ForwardVector;
	const FVector GuideUp = FVector::UpVector;
	const float SweepAngleDegrees = 120.0f;
	const float GuideLength = 240.0f;
	constexpr int32 SampleCount = 25;

	for (const float T : { 0.0f, 0.5f, 1.0f })
	{
		const FVector SweepDirection = RopeMath::ArcDirectionAtAlpha(
			AimDirection, GuideUp, SweepAngleDegrees, T);
		TArray<FVector> Points;
		RopeMath::BuildWhipGuideRawPoints(Origin, SweepDirection, AimDirection,
			/*bHasAimTarget*/ true, T, GuideLength, FVector(200.0f, -100.0f, 50.0f),
			/*AimSteerStartAlpha*/ 0.25f, /*AimLockAlpha*/ 0.50f,
			/*AimDirectionBias*/ 4.0f, SampleCount, Points);

		TestEqual(TEXT("sweeping line sample count"), Points.Num(), SampleCount);
		for (int32 Index = 0; Index < Points.Num(); ++Index)
		{
			const float RopeAlpha = static_cast<float>(Index) / static_cast<float>(Points.Num() - 1);
			const FVector Expected = Origin + SweepDirection * (RopeAlpha * GuideLength);
			TestTrue(*FString::Printf(TEXT("T=%.2f sample %d stays on one sweep line"), T, Index),
				Points[Index].Equals(Expected, 0.01f));
		}
	}

	const FVector HitPoint = Origin + AimDirection * 100.0f;
	const FVector FinalDirection = RopeMath::ArcDirectionAtAlpha(
		AimDirection, GuideUp, SweepAngleDegrees, 1.0f);
	TestTrue(TEXT("final sweep line uses Origin-to-hit direction"),
		FinalDirection.Equals((HitPoint - Origin).GetSafeNormal(), 0.01f));
	TestTrue(TEXT("reachable hit lies on final finite guide"),
		FVector::Dist(Origin, HitPoint) <= GuideLength);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
