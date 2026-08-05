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

	const FVector AimTarget(100.0f, 40.0f, 15.0f);
	const FVector LockedAimDirection = (AimTarget - Sim.StartPinTarget).GetSafeNormal();
	FRopeWhipGuide Guide;
	Guide.Begin(FVector::ForwardVector, Sim.Positions[0], FVector::ForwardVector,
		FVector::UpVector, FVector::RightVector, 1500.0f, FVector::ZeroVector,
		/*bHasAimTarget*/ true, AimTarget, 0.25f, 0.50f);
	const int32 LastNode = Sim.Num() - 1;
	const int32 FirstBlendedTailNode = FMath::Clamp(
		FMath::FloorToInt(static_cast<float>(LastNode) * Config.GuidedLength) + 1, 1, LastNode);
	const float InfluenceEnd = FMath::Clamp(
		Config.GuidedLength + Config.AimHitTipSolverFraction, 0.05f, 1.0f);
	const int32 FirstFullySolverTailNode = FMath::Clamp(
		FMath::FloorToInt(static_cast<float>(LastNode) * InfluenceEnd) + 1, 1, LastNode);
	const FVector TipBeforeSnap = Sim.Positions.Last();
	const FVector SolverTailBeforeSnap = Sim.Positions[FirstFullySolverTailNode];
	Guide.SnapToInitialPose(Sim, Config);
	TestTrue(TEXT("initial aim snap preserves the solver-owned tip"),
		Sim.Positions.Last().Equals(TipBeforeSnap, 0.01f));
	TestTrue(TEXT("Assisted crossfades instead of cutting ownership at GuidedLength"),
		Guide.IsGuidedNodeThisFrame(FirstBlendedTailNode));
	TestFalse(TEXT("Assisted releases nodes after the GuidedLength crossfade"),
		Guide.IsGuidedNodeThisFrame(FirstFullySolverTailNode));
	TestTrue(TEXT("initial aim snap preserves the tail beyond the Assisted crossfade"),
		Sim.Positions[FirstFullySolverTailNode].Equals(SolverTailBeforeSnap, 0.01f));

	const auto CheckGuidedNodesOnLine = [this, &Guide](const TCHAR* Stage,
		const TArray<FVector>& Positions, const FVector& LineOrigin, const FVector& LineDirection)
	{
		bool bCheckedAny = false;
		const FVector Direction = LineDirection.GetSafeNormal();
		for (int32 NodeIndex = 1; NodeIndex < Positions.Num(); ++NodeIndex)
		{
			if (!Guide.IsGuidedNodeThisFrame(NodeIndex))
			{
				continue;
			}
			bCheckedAny = true;
			const FVector FromOrigin = Positions[NodeIndex] - LineOrigin;
			const FVector OffLine = FromOrigin - Direction * FVector::DotProduct(FromOrigin, Direction);
			TestTrue(*FString::Printf(TEXT("%s guided node %d stays on the shared straight line (error %.4f)"),
				Stage, NodeIndex, OffLine.Size()), OffLine.Size() <= 0.01f);
		}
		TestTrue(*FString::Printf(TEXT("%s checks at least one guided node"), Stage), bCheckedAny);
	};

	const FVector InitialGuideDirection = RopeMath::ArcDirectionAtAlpha(
		LockedAimDirection, FVector::UpVector, Config.SweepAngleDegrees, 0.0f);
	CheckGuidedNodesOnLine(TEXT("initial aim snap"), Sim.Positions,
		Sim.StartPinTarget, InitialGuideDirection);

	const int32 MiddleNode = LastNode / 2;
	// The free end is moved off the spline to verify that Advance does not overwrite the end node again.
	const FVector FreeTipBefore(200.0f, 40.0f, -15.0f);
	Sim.Positions[LastNode] = FreeTipBefore;
	Sim.PrevPositions[LastNode] = FreeTipBefore;
	Sim.StartPinPrev = Sim.StartPinTarget;
	Guide.Advance(1.0f / 60.0f, Sim, Config);
	const float AdvancedT = (1.0f / 60.0f) / Config.Duration;
	const FVector CurrentGuideDirection = RopeMath::ArcDirectionAtAlpha(
		LockedAimDirection, FVector::UpVector, Config.SweepAngleDegrees, AdvancedT);
	CheckGuidedNodesOnLine(TEXT("advanced current targets"), Guide.GetCurrentTargets(),
		Sim.StartPinTarget, CurrentGuideDirection);
	CheckGuidedNodesOnLine(TEXT("advanced previous targets"), Guide.GetPrevTargets(),
		Sim.StartPinPrev, InitialGuideDirection);

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

	// The presentation rotates during the throw, but the last Assisted line must land on the explicit
	// AimTarget even when Begin's fallback aim direction is different.
	Guide.Advance(Config.Duration, Sim, Config);
	CheckGuidedNodesOnLine(TEXT("final aim targets"), Guide.GetCurrentTargets(),
		Sim.StartPinTarget, LockedAimDirection);
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
	Sim.StartPinPrev = Sim.StartPinTarget;
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
	TestTrue(*FString::Printf(TEXT("spacing repair keeps meaningful rotating guide motion (%.3f cm)"), MaxGuideMotion),
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

// An ordinary FullSimulation whip begins on a shallow C guide with its free end opposite AimDir, becomes
// exactly straight at the configured normalized time, then crossfades into the solver-owned tail.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWhipInitialCShapeAndReleaseTest,
	"DynamicRope.Solver.WhipInitialCShapeStraightensAndGradualRelease",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWhipInitialCShapeAndReleaseTest::RunTest(const FString& Parameters)
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
	Config.AimHitTipSolverFraction = 0.25f;
	Config.SweepAngleDegrees = 180.0f;
	Config.ComponentRopeLength = Sim.RopeLength;
	Config.FullSimInitialCurveFraction = 0.12f;
	Config.FullSimStraightenTimeFraction = 0.50f;

	FRopeWhipGuide Guide;
	Guide.Begin(FVector::ForwardVector, Sim.Positions[0], FVector::ForwardVector,
		FVector::UpVector, FVector::RightVector, 1500.0f, FVector::ZeroVector);
	// Reproduce a folded source pose to prove that the throw boundary deliberately replaces it with the
	// configured C guide rather than leaking the old shape into Flight.
	for (int32 NodeIndex = 1; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		const float S = static_cast<float>(NodeIndex) / static_cast<float>(Sim.Num() - 1);
		Sim.Positions[NodeIndex] = FVector(100.0f * FMath::Sin(S * PI), 0.0f, -80.0f * FMath::Sin(S * PI));
		Sim.PrevPositions[NodeIndex] = Sim.Positions[NodeIndex];
	}
	Guide.SnapToInitialPose(Sim, Config);

	const int32 LastNode = Sim.Num() - 1;
	const FVector InitialRoot = Sim.StartPinTarget;
	const FVector InitialTipVector = Sim.Positions[LastNode] - InitialRoot;
	TestTrue(TEXT("initial Full Simulation free end lies opposite AimDir"),
		FVector::DotProduct(InitialTipVector, FVector::ForwardVector) < 0.0f);
	const FVector InitialChordDirection = InitialTipVector.GetSafeNormal();
	float MaxInitialOffChord = 0.0f;
	for (int32 NodeIndex = 1; NodeIndex < LastNode; ++NodeIndex)
	{
		const FVector FromRoot = Sim.Positions[NodeIndex] - InitialRoot;
		const FVector OffChord = FromRoot - InitialChordDirection *
			FVector::DotProduct(FromRoot, InitialChordDirection);
		MaxInitialOffChord = FMath::Max(MaxInitialOffChord, OffChord.Size());
	}
	TestTrue(TEXT("initial Full Simulation guide has a visible shallow C curvature"),
		MaxInitialOffChord > Sim.SegmentLength * 0.25f);
	TestTrue(TEXT("initial C guide includes the future solver tail"),
		Guide.IsGuidedNodeThisFrame(LastNode));

	// Full Simulation must rebuild from the live hand just like Assisted. The old implementation kept
	// using the throw-time origin here, and the spacing clamp curved the straight targets while bridging
	// from this moved pin back to that stale origin.
	const FVector InitialHand = Sim.StartPinTarget;
	const FVector MovedHand = InitialHand + FVector(25.0f, -15.0f, 12.0f);
	Sim.StartPinPrev = InitialHand;
	Sim.StartPinTarget = MovedHand;

	// At 50% of the whip timer the C amplitude is exactly zero. The fully guided span and its crossfade
	// must now reproduce one straight line from the moved live hand.
	constexpr float StraightenTime = 0.50f;
	Guide.Advance(StraightenTime, Sim, Config);
	const FVector StraightDirection = RopeMath::ArcDirectionAtAlpha(
		FVector::ForwardVector, FVector::UpVector, Config.SweepAngleDegrees, StraightenTime);
	const float StraightGuidedEnd = FMath::Lerp(
		1.0f, Config.GuidedLength, RopeMath::SmoothStep(StraightenTime));
	const int32 StraightLastFullyGuidedNode = FMath::FloorToInt(
		static_cast<float>(LastNode) * StraightGuidedEnd);
	for (int32 NodeIndex = 1; NodeIndex <= StraightLastFullyGuidedNode; ++NodeIndex)
	{
		const FVector Expected = MovedHand + StraightDirection *
			(Sim.SegmentLength * static_cast<float>(NodeIndex));
		TestTrue(*FString::Printf(TEXT("straightened guided node %d lies on the guide"), NodeIndex),
			Guide.GetCurrentTargets()[NodeIndex].Equals(Expected, 0.01f));
	}
	TestTrue(TEXT("node below the moving boundary remains fully guide-owned"),
		Guide.IsGuidedNodeThisFrame(StraightLastFullyGuidedNode));
	const float StraightInfluenceEnd = FMath::Clamp(
		StraightGuidedEnd + Config.AimHitTipSolverFraction, 0.05f, 1.0f);
	const int32 StraightBlendedNode = FMath::RoundToInt(
		static_cast<float>(LastNode) * 0.5f * (StraightGuidedEnd + StraightInfluenceEnd));
	const FVector FullGuideTarget = MovedHand + StraightDirection *
		(Sim.SegmentLength * static_cast<float>(StraightBlendedNode));
	const FVector BlendedTarget = Guide.GetCurrentTargets()[StraightBlendedNode];
	TestTrue(TEXT("Full Simulation keeps the node after GuidedLength in the smooth crossfade"),
		Guide.IsGuidedNodeThisFrame(StraightBlendedNode));
	TestTrue(TEXT("Full Simulation crossfade target stays on the rotating straight line"),
		FVector::CrossProduct(BlendedTarget - MovedHand, StraightDirection).Size() <= 0.01f);
	for (int32 NodeIndex = 1; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		if (!Guide.IsGuidedNodeThisFrame(NodeIndex))
		{
			continue;
		}
		const FVector FromLiveHand = Guide.GetCurrentTargets()[NodeIndex] - MovedHand;
		const FVector OffLine = FromLiveHand - StraightDirection *
			FVector::DotProduct(FromLiveHand, StraightDirection);
		TestTrue(*FString::Printf(TEXT("moved-hand Full Simulation node %d remains collinear"), NodeIndex),
			OffLine.Size() <= 0.01f);
	}
	TestFalse(TEXT("Full Simulation crossfade is not a hard one-weight guide target"),
		BlendedTarget.Equals(FullGuideTarget, 0.01f));
	TestFalse(TEXT("Full Simulation crossfade is not a hard zero-weight solver target"),
		BlendedTarget.Equals(Sim.Positions[StraightBlendedNode], 0.01f));
	TestFalse(TEXT("the exact tip reaches zero guide influence at the straightened blend end"),
		Guide.IsGuidedNodeThisFrame(LastNode));
	TestTrue(TEXT("the exact tip remains fully solver-owned"),
		Guide.GetCurrentTargets()[LastNode].Equals(Sim.Positions[LastNode], 0.01f));

	Guide.Advance(1.0f - StraightenTime, Sim, Config);
	const int32 FinalBlendedNode = 14; // 14 / 20 = 0.70, inside the 0.65-0.90 crossfade.
	const int32 FinalSolverNode = 19; // 19 / 20 = 0.95, after the crossfade.
	TestTrue(TEXT("Full Simulation keeps a smooth tail crossfade after final GuidedLength"),
		Guide.IsGuidedNodeThisFrame(FinalBlendedNode));
	TestFalse(TEXT("Full Simulation releases nodes after the final crossfade"),
		Guide.IsGuidedNodeThisFrame(FinalSolverNode));
	TestTrue(TEXT("node after the final crossfade preserves its solver position"),
		Guide.GetCurrentTargets()[FinalSolverNode].Equals(Sim.Positions[FinalSolverNode], 0.01f));
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
