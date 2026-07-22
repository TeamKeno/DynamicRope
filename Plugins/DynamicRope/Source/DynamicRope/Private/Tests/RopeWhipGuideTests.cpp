// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeWhipGuide and whip-path math unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWhipGuide.h"
#include "RopeMathHelpers.h"
#include "Solver/RopeXPBDSolver.h"
#include "RopeTestHelpers.h"

// Aim-hit guide는 중앙만 잡고 양끝은 solver 상태를 유지하는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimHitEndpointSolverBlendTest,
	"DynamicRope.Solver.AimHitEndpointSolverBlend",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimHitEndpointSolverBlendTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(21, 200.0f);
	// 움직인 손 pin을 구성해 root 쪽 소켓 추종과 중앙 가이드가 함께 적용되는 조건을 만든다.
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
	// 자유단을 spline 밖으로 옮겨 Advance가 끝 노드를 다시 덮어쓰지 않는지 검증한다.
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

// SegmentLength 리샘플 뒤 aim envelope를 노드별 Lerp하면 가중치 변화 구간에서 spacing이 다시
// 벌어질 수 있다. GPU override에 실리는 최종 Current/Prev/예측 target까지 같은 비신축 계약을 지키는가.
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

	// 실제 CPU/GPU 공통 strain-limit 계약(phase policy가 1.0을 전달)은 target/free-tail 경계까지
	// resident pose 전체에서 제거한다. CPU 미러로 같은 solver 결과를 검증한다.
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

// 일반 FullSimulation whip은 앞쪽 guide만 스냅한다. 간격 보정을 위해 solver-owned tail까지
// 순간이동시키지 않으면서, 실제 guided run은 충분히 움직이고 SegmentLength 상한을 지키는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWhipInitialPoseOwnershipTest,
	"DynamicRope.Solver.WhipInitialPosePreservesSolverOwnedTail",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWhipInitialPoseOwnershipTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(21, 200.0f);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;

	FRopeWhipGuide::FConfig Config;
	Config.GuidedLength = 0.65f;
	Config.SweepAngleDegrees = 180.0f;
	Config.ComponentRopeLength = Sim.RopeLength;

	FRopeWhipGuide Guide;
	Guide.Begin(FVector::ForwardVector, Sim.Positions[0], FVector::ForwardVector,
		FVector::UpVector, FVector::RightVector, 1500.0f, FVector::ZeroVector);
	const TArray<FVector> BeforeSnap = Sim.Positions;
	Guide.SnapToInitialPose(Sim, Config);

	TestFalse(TEXT("free tip remains solver-owned"), Guide.IsGuidedNodeThisFrame(Sim.Num() - 1));
	float MaxGuidedMotion = 0.0f;
	for (int32 NodeIndex = 1; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		if (!Guide.IsGuidedNodeThisFrame(NodeIndex))
		{
			TestTrue(*FString::Printf(TEXT("solver-owned node %d is not teleported"), NodeIndex),
				Sim.Positions[NodeIndex].Equals(BeforeSnap[NodeIndex], 0.01f));
			continue;
		}

		MaxGuidedMotion = FMath::Max(MaxGuidedMotion,
			FVector::Dist(BeforeSnap[NodeIndex], Sim.Positions[NodeIndex]));
		const FVector Leader = NodeIndex == 1 ? Sim.StartPinTarget : Sim.Positions[NodeIndex - 1];
		const float Distance = FVector::Dist(Leader, Sim.Positions[NodeIndex]);
		TestTrue(*FString::Printf(TEXT("initial guided edge %d-%d stays within spacing (%.3f <= %.3f)"),
			NodeIndex - 1, NodeIndex, Distance, Sim.SegmentLength),
			Distance <= Sim.SegmentLength + 0.05f);
	}
	TestTrue(*FString::Printf(TEXT("initial guide still moves the rope meaningfully (%.3f cm)"), MaxGuidedMotion),
		MaxGuidedMotion > Sim.SegmentLength);
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
