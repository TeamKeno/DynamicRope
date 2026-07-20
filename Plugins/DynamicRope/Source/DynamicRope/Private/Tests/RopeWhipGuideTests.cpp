// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeWhipGuide and whip-path math unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWhipGuide.h"
#include "RopeMathHelpers.h"
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
	Guide.SnapToInitialPose(Sim, Config);

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
