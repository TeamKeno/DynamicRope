// Copyright Epic Games, Inc. All Rights Reserved.
//
// Unit tests for the taut-hold presentation shaping (RopeTautPresentation.h): the render-only
// straightening of the hand-side free span and the snap-taut thrum.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeTautPresentation.h"

// A slightly sagging five-node span: hand at the origin, first wrapped node 100 cm along X, interior
// nodes drooping a few centimetres in Z the way a taut XPBD leg does.
static TArray<FVector> MakeSaggingSpan()
{
	return {
		FVector(0.0f, 0.0f, 0.0f),
		FVector(25.0f, 0.0f, -3.0f),
		FVector(50.0f, 0.0f, -4.0f),
		FVector(75.0f, 0.0f, -3.0f),
		FVector(100.0f, 0.0f, 0.0f),
		// A tail node beyond the wrap, which must never be touched.
		FVector(120.0f, 0.0f, -10.0f)
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTautPresentationStraightenTest,
	"DynamicRope.Render.TautPresentation.StraightensSpanToChord",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTautPresentationStraightenTest::RunTest(const FString& Parameters)
{
	TArray<FVector> Points = MakeSaggingSpan();
	RopeTautPresentation::FParams Params;
	Params.EndNode = 4;
	Params.Straighten = 1.0f;

	TestTrue(TEXT("a sagging span reports movement"), RopeTautPresentation::Apply(Points, Params));

	// Full straightening puts every interior node exactly on the chord, redistributed by index.
	TestTrue(TEXT("node 1 lands on the chord"), Points[1].Equals(FVector(25.0f, 0.0f, 0.0f), 0.01f));
	TestTrue(TEXT("node 2 lands on the chord"), Points[2].Equals(FVector(50.0f, 0.0f, 0.0f), 0.01f));
	TestTrue(TEXT("node 3 lands on the chord"), Points[3].Equals(FVector(75.0f, 0.0f, 0.0f), 0.01f));
	// The ends and the wrapped tail are left alone.
	TestTrue(TEXT("the hand node is untouched"), Points[0].Equals(FVector::ZeroVector, 0.001f));
	TestTrue(TEXT("the first wrapped node is untouched"), Points[4].Equals(FVector(100.0f, 0.0f, 0.0f), 0.001f));
	TestTrue(TEXT("nodes beyond the wrap are untouched"), Points[5].Equals(FVector(120.0f, 0.0f, -10.0f), 0.001f));

	// A partial blend moves half way.
	TArray<FVector> Half = MakeSaggingSpan();
	Params.Straighten = 0.5f;
	RopeTautPresentation::Apply(Half, Params);
	TestTrue(TEXT("half straightening halves the sag"), Half[2].Equals(FVector(50.0f, 0.0f, -2.0f), 0.01f));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTautPresentationThrumTest,
	"DynamicRope.Render.TautPresentation.ThrumIsPinnedStandingWave",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTautPresentationThrumTest::RunTest(const FString& Parameters)
{
	// A dead-straight horizontal span, so every displacement observed is the thrum alone.
	TArray<FVector> Points = {
		FVector(0.0f, 0.0f, 0.0f), FVector(25.0f, 0.0f, 0.0f), FVector(50.0f, 0.0f, 0.0f),
		FVector(75.0f, 0.0f, 0.0f), FVector(100.0f, 0.0f, 0.0f)
	};
	RopeTautPresentation::FParams Params;
	Params.EndNode = 4;
	Params.ThrumOffset = 2.0f;

	TestTrue(TEXT("the thrum reports movement"), RopeTautPresentation::Apply(Points, Params));

	// Perpendicular to an X chord with world up as the reference, the offset is along Y. The envelope
	// is sin(pi * fraction): zero at both pinned ends, the antinode in the middle.
	TestTrue(TEXT("the hand end stays pinned"), Points[0].Equals(FVector(0.0f, 0.0f, 0.0f), 0.001f));
	TestTrue(TEXT("the wrapped end stays pinned"), Points[4].Equals(FVector(100.0f, 0.0f, 0.0f), 0.001f));
	TestEqual(TEXT("the antinode carries the full offset"),
		static_cast<float>(FMath::Abs(Points[2].Y)), 2.0f, 0.01f);
	TestEqual(TEXT("the quarter points carry sin(45 deg) of it"),
		static_cast<float>(FMath::Abs(Points[1].Y)), 2.0f * FMath::Sin(UE_PI * 0.25f), 0.01f);
	TestTrue(TEXT("the offset is purely perpendicular"),
		FMath::IsNearlyZero(Points[2].X - 50.0f, 0.001f) && FMath::IsNearlyZero(Points[2].Z, 0.001f));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTautPresentationCornerGuardTest,
	"DynamicRope.Render.TautPresentation.CornerSpanFadesOut",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTautPresentationCornerGuardTest::RunTest(const FString& Parameters)
{
	// A span bent hard over a corner: the middle node sits 50 cm off the chord, past the fade-out end,
	// as it does when the rope runs over a ledge or a pulley pivot. Straightening here would drag the
	// rope through the corner geometry, so the guard must leave the span exactly as solved.
	TArray<FVector> Points = {
		FVector(0.0f, 0.0f, 0.0f),
		FVector(50.0f, 0.0f, 50.0f),
		FVector(100.0f, 0.0f, 0.0f)
	};
	const TArray<FVector> Original = Points;
	RopeTautPresentation::FParams Params;
	Params.EndNode = 2;
	Params.Straighten = 1.0f;
	Params.ThrumOffset = 2.0f;

	TestFalse(TEXT("a bent span reports no movement"), RopeTautPresentation::Apply(Points, Params));
	TestTrue(TEXT("the corner node is exactly as solved"), Points[1].Equals(Original[1], 0.001f));

	// Degenerate inputs are refused outright: too short a span, an out-of-range end node, and a
	// zero-length chord.
	TArray<FVector> Short = { FVector::ZeroVector, FVector(10.0f, 0.0f, 0.0f) };
	Params.EndNode = 1;
	TestFalse(TEXT("a span with no interior nodes is refused"), RopeTautPresentation::Apply(Short, Params));
	Params.EndNode = 5;
	TestFalse(TEXT("an out-of-range end node is refused"), RopeTautPresentation::Apply(Short, Params));
	TArray<FVector> Collapsed = { FVector::ZeroVector, FVector(1.0f, 0.0f, 0.0f), FVector::ZeroVector };
	Params.EndNode = 2;
	TestFalse(TEXT("a zero-length chord is refused"), RopeTautPresentation::Apply(Collapsed, Params));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
