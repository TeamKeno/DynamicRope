// Copyright 2026 TeamKeno. All Rights Reserved.

// Straight ordinary Flight guide regression tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeMathHelpers.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWhipOrdinaryStraightGuideTest,
	"DynamicRope.Solver.WhipOrdinaryGuideIgnoresInheritedDrift",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWhipOrdinaryStraightGuideTest::RunTest(const FString& Parameters)
{
	const FVector Origin(10.0f, -20.0f, 30.0f);
	const FVector SweepDirection = FVector(1.0f, 0.0f, 1.0f).GetSafeNormal();
	const FVector AimDirection = FVector::ForwardVector;
	const FVector NonParallelInheritedDrift(300.0f, -250.0f, 200.0f);
	constexpr float GuideLength = 240.0f;
	constexpr int32 SampleCount = 25;

	for (const float T : { 0.0f, 0.5f, 1.0f })
	{
		TArray<FVector> Points;
		RopeMath::BuildWhipGuideRawPoints(Origin, SweepDirection, AimDirection,
			/*bHasAimTarget*/ false, T, GuideLength, NonParallelInheritedDrift,
			/*AimSteerStartAlpha*/ 0.25f, /*AimLockAlpha*/ 0.50f,
			/*AimDirectionBias*/ 2.0f, SampleCount, Points);

		TestEqual(TEXT("ordinary straight guide sample count"), Points.Num(), SampleCount);
		for (int32 Index = 0; Index < Points.Num(); ++Index)
		{
			const float RopeAlpha = static_cast<float>(Index) / static_cast<float>(Points.Num() - 1);
			const FVector Expected = Origin + SweepDirection * (RopeAlpha * GuideLength);
			TestTrue(*FString::Printf(
				TEXT("T=%.2f ordinary sample %d ignores non-parallel drift and stays collinear"), T, Index),
				Points[Index].Equals(Expected, 0.01f));
		}
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
