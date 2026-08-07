// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Gameplay/RopeReelGaugeComponent.h"
#include "UI/RopeReelGaugeWidget.h"

// The fade latch behind the reel gauge: activity snaps the alpha to 1, inactivity blends it linearly
// to 0 over the fade time, activity during the blend snaps it straight back, and a gauge that has
// never seen activity starts fully faded.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeReelGaugeFadeLatchTest,
	"DynamicRope.UI.ReelGauge.FadeLatchBlendsOutOverFadeTime",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeReelGaugeFadeLatchTest::RunTest(const FString& Parameters)
{
	constexpr float FadeTime = 3.0f;
	FRopeReelGaugeFadeState Fade;

	TestTrue(TEXT("never-active gauge starts fully faded"),
		FMath::IsNearlyZero(Fade.Update(false, 1.0f, FadeTime)));

	TestEqual(TEXT("activity snaps the alpha to 1"), Fade.Update(true, 1.0f, FadeTime), 1.0f);

	TestTrue(TEXT("one second into the blend leaves two thirds"),
		FMath::IsNearlyEqual(Fade.Update(false, 1.0f, FadeTime), 2.0f / 3.0f, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("two seconds in leaves one third"),
		FMath::IsNearlyEqual(Fade.Update(false, 1.0f, FadeTime), 1.0f / 3.0f, KINDA_SMALL_NUMBER));

	TestEqual(TEXT("activity mid-blend snaps straight back to 1"), Fade.Update(true, 1.0f, FadeTime), 1.0f);

	TestTrue(TEXT("the full fade time later the gauge is gone"),
		FMath::IsNearlyZero(Fade.Update(false, FadeTime, FadeTime)));
	TestTrue(TEXT("and it stays gone"),
		FMath::IsNearlyZero(Fade.Update(false, 1.0f, FadeTime)));

	// A zero fade time means hide the moment activity stops, with no blend.
	FRopeReelGaugeFadeState NoBlend;
	TestEqual(TEXT("zero fade time still shows while active"), NoBlend.Update(true, 0.016f, 0.0f), 1.0f);
	TestTrue(TEXT("zero fade time hides immediately after"),
		FMath::IsNearlyZero(NoBlend.Update(false, 0.016f, 0.0f)));
	return true;
}

// The fill mapping: where the current length sits between the reel-in floor and the initial length,
// clamped, with a degenerate range reading as full.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeReelGaugeFractionTest,
	"DynamicRope.UI.ReelGauge.LengthFractionClampsAndGuards",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeReelGaugeFractionTest::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("midpoint maps to a half"),
		FMath::IsNearlyEqual(URopeReelGaugeWidget::ComputeLengthFraction(350.0f, 100.0f, 600.0f), 0.5f, KINDA_SMALL_NUMBER));
	TestEqual(TEXT("the initial length is a full bar"),
		URopeReelGaugeWidget::ComputeLengthFraction(600.0f, 100.0f, 600.0f), 1.0f);
	TestEqual(TEXT("the reel-in floor is an empty bar"),
		URopeReelGaugeWidget::ComputeLengthFraction(100.0f, 100.0f, 600.0f), 0.0f);
	TestEqual(TEXT("below the floor clamps to empty"),
		URopeReelGaugeWidget::ComputeLengthFraction(10.0f, 100.0f, 600.0f), 0.0f);
	TestEqual(TEXT("above the initial length clamps to full"),
		URopeReelGaugeWidget::ComputeLengthFraction(900.0f, 100.0f, 600.0f), 1.0f);
	TestEqual(TEXT("a rope that cannot reel reads as full"),
		URopeReelGaugeWidget::ComputeLengthFraction(100.0f, 100.0f, 100.0f), 1.0f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
