// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeComponent::DecideTargetPullable(순수 정적 판정)의 유닛 테스트. BinaryPullable 테더 모드에서
// "대상을 끌 수 있는가"(대상 유효질량 ≤ wielder 유효질량)를 TetherPullMassMargin 히스테리시스로
// sticky하게 판정한다 — 월드/UObject 인스턴스 없이 검증한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeComponent.h"
#include "Math/NumericLimits.h"

namespace
{
	// 앵커(MOVE_None/정적 = 무한질량)를 넘길 때 쓰는 값 — UpdateTargetPullable/UpdateTether와 동일.
	const float RopeInfMass = TNumericLimits<float>::Max();
	const float RopeMargin = 1.15f;
}

// 가벼운/동일 대상은 끌 수 있고, 무거운 대상은 끌 수 없다(이전 상태 무관하게 밴드 밖이면 확정).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePullableBasicTest,
	"DynamicRope.Pull.PullableByMassCompare",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePullableBasicTest::RunTest(const FString& Parameters)
{
	// 명확히 가벼운 대상 → 이전 상태와 무관하게 끌림 가능.
	TestTrue(TEXT("light target pullable (prev not)"), URopeComponent::DecideTargetPullable(10.0f, 100.0f, /*bPrev*/ false, RopeMargin));
	TestTrue(TEXT("light target pullable (prev yes)"), URopeComponent::DecideTargetPullable(10.0f, 100.0f, /*bPrev*/ true, RopeMargin));

	// 명확히 무거운 대상 → 이전 상태와 무관하게 끌림 불가.
	TestFalse(TEXT("heavy target not pullable (prev not)"), URopeComponent::DecideTargetPullable(100.0f, 10.0f, /*bPrev*/ false, RopeMargin));
	TestFalse(TEXT("heavy target not pullable (prev yes)"), URopeComponent::DecideTargetPullable(100.0f, 10.0f, /*bPrev*/ true, RopeMargin));

	return true;
}

// 앵커(무한질량) 처리: 무한 대상은 못 끌고(→ wielder 제한), 무한 wielder에는 유한 대상이 끌린다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePullableAnchorTest,
	"DynamicRope.Pull.AnchorMassHandling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePullableAnchorTest::RunTest(const FString& Parameters)
{
	// 무한질량 대상(정적/MOVE_None) → 끌림 불가(이전 상태 무관).
	TestFalse(TEXT("infinite target not pullable (prev not)"), URopeComponent::DecideTargetPullable(RopeInfMass, 50.0f, false, RopeMargin));
	TestFalse(TEXT("infinite target not pullable (prev yes)"), URopeComponent::DecideTargetPullable(RopeInfMass, 50.0f, true, RopeMargin));

	// 무한질량 wielder(정적 앵커) → 유한 대상은 항상 끌림 가능.
	TestTrue(TEXT("finite target pullable vs infinite wielder (prev not)"), URopeComponent::DecideTargetPullable(50.0f, RopeInfMass, false, RopeMargin));
	TestTrue(TEXT("finite target pullable vs infinite wielder (prev yes)"), URopeComponent::DecideTargetPullable(50.0f, RopeInfMass, true, RopeMargin));

	return true;
}

// 히스테리시스: 질량이 마진 밴드 안(거의 동일)이면 이전 판정을 유지(sticky)하고, 밴드를 넘으면 뒤집힌다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePullableHysteresisTest,
	"DynamicRope.Pull.MassMarginHysteresis",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePullableHysteresisTest::RunTest(const FString& Parameters)
{
	// 대상 52 vs wielder 50 — 마진(1.15 → 밴드 [43.5, 57.5]) 안이라 이전 판정을 유지한다.
	TestTrue(TEXT("within band stays pullable"), URopeComponent::DecideTargetPullable(52.0f, 50.0f, /*bPrev*/ true, RopeMargin));
	TestFalse(TEXT("within band stays not-pullable"), URopeComponent::DecideTargetPullable(52.0f, 50.0f, /*bPrev*/ false, RopeMargin));

	// 상단 밴드를 넘김(대상 60 > 50×1.15=57.5): 끌 수 있던 상태에서 끌림 불가로 뒤집힌다.
	TestFalse(TEXT("crossing upper band flips to not-pullable"), URopeComponent::DecideTargetPullable(60.0f, 50.0f, /*bPrev*/ true, RopeMargin));

	// 하단 밴드를 넘김(대상 40×1.15=46 ≤ 50): 끌 수 없던 상태에서 끌림 가능으로 뒤집힌다.
	TestTrue(TEXT("crossing lower band flips to pullable"), URopeComponent::DecideTargetPullable(40.0f, 50.0f, /*bPrev*/ false, RopeMargin));

	// 마진 1.0(히스테리시스 없음): 순수 ≤ 비교로 동작.
	TestTrue(TEXT("no-margin equal is pullable"), URopeComponent::DecideTargetPullable(50.0f, 50.0f, /*bPrev*/ false, 1.0f));
	TestFalse(TEXT("no-margin heavier is not pullable"), URopeComponent::DecideTargetPullable(50.01f, 50.0f, /*bPrev*/ false, 1.0f));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
