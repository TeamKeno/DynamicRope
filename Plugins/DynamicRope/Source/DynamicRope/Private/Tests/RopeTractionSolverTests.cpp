// Copyright Epic Games, Inc. All Rights Reserved.
//
// RopeTraction 단위 테스트 — 견인(테더/능동 Pull) 축 드라이브 수학을 월드 없이 검증한다.
// 이 수학은 오래 PIE로만 검증됐고, 그 사이 두 번의 실측 버그(고장력 폭발/영구 뒤처짐)가 났다.
// 아래 회귀 테스트가 그 둘을 고정한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeTractionSolver.h"

namespace
{
	// 테더 리엘의 대상 서보(양방향 정확 서보) — BinaryPullable pullable 대상 경로.
	RopeTraction::FRopeAxisServo MakeReelServo(float TargetSpeed)
	{
		RopeTraction::FRopeAxisServo Servo;
		Servo.TargetSpeed = TargetSpeed;
		Servo.Alpha = 1.0f;
		Servo.bBidirectional = true;
		Servo.bCancelOutward = false;
		return Servo;
	}

	// 능동 Pull / wielder 톱업의 단방향 서보(가속만).
	RopeTraction::FRopeAxisServo MakeOneWayServo(float TargetSpeed, float Alpha = 1.0f)
	{
		RopeTraction::FRopeAxisServo Servo;
		Servo.TargetSpeed = TargetSpeed;
		Servo.Alpha = Alpha;
		Servo.bBidirectional = false;
		Servo.bCancelOutward = false;
		return Servo;
	}
}

// 양방향 서보는 목표 초과 관성을 제거한다(제동). CL 401 회귀: 단방향(가속만)이면 고장력에서 대상이
// 경계를 지나쳐 코스팅→슬랙→되튕김→물리 폭발이 났다. 제동이 살아 있어야 경계에 안착한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionBidirectionalBrakesTest,
	"DynamicRope.Traction.BidirectionalServoBrakesOvershoot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionBidirectionalBrakesTest::RunTest(const FString& Parameters)
{
	// 목표 100인데 이미 250으로 안쪽으로 질주 중 → 양방향은 -150(제동)을 낸다.
	const float BrakeDv = RopeTraction::ComputeAxisDeltaV(250.0f, MakeReelServo(100.0f));
	TestEqual(TEXT("bidirectional brakes excess inward speed"), BrakeDv, -150.0f);

	// 같은 상황에서 단방향은 무동작 → 초과 관성이 남아 경계를 지나쳐 코스팅한다(옛 폭발 경로).
	const float OneWayDv = RopeTraction::ComputeAxisDeltaV(250.0f, MakeOneWayServo(100.0f));
	TestEqual(TEXT("one-directional servo does not brake"), OneWayDv, 0.0f);

	// 경계에서 목표가 0으로 taper되면 양방향은 남은 관성을 전부 뺀다 → 오버슛 없이 정지.
	const float SettleDv = RopeTraction::ComputeAxisDeltaV(80.0f, MakeReelServo(0.0f));
	TestEqual(TEXT("bidirectional servo stops at the boundary"), SettleDv, -80.0f);
	return true;
}

// 리엘 목표 속도: taper 감속 + Overshoot/dt 캡. 남은 overshoot를 넘게 회수하지 않는다(경계 안착).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionReelTargetSpeedTest,
	"DynamicRope.Traction.ReelTargetSpeedTapersAndCaps",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionReelTargetSpeedTest::RunTest(const FString& Parameters)
{
	const float Dt = 1.0f / 60.0f;

	// overshoot(50) >> taper(1.5) → 고정 리엘 속도(400). 단, Overshoot/dt = 3000이라 캡에 안 걸린다.
	TestEqual(TEXT("far from boundary reels at the flat speed"),
		RopeTraction::ComputeReelTargetSpeed(50.0f, 400.0f, 1.5f, Dt), 400.0f);

	// overshoot(0.75) = taper(1.5)의 절반 → 선형 감속으로 200. Overshoot/dt = 45가 더 작아 캡이 이긴다.
	TestEqual(TEXT("inside the taper the overshoot/dt cap wins"),
		RopeTraction::ComputeReelTargetSpeed(0.75f, 400.0f, 1.5f, Dt), 45.0f);

	// 경계 도달(overshoot 0) → 0. 코스팅 없이 멈춘다.
	TestEqual(TEXT("at the boundary the reel target is zero"),
		RopeTraction::ComputeReelTargetSpeed(0.0f, 400.0f, 1.5f, Dt), 0.0f);

	// ReelSpeed=0 = 리엘 없음(0). "상한 없음"으로 오해하면 안 된다 — 그 의미는 호출자가 따로 가진다.
	TestEqual(TEXT("zero reel speed means no reel"),
		RopeTraction::ComputeReelTargetSpeed(50.0f, 0.0f, 1.5f, Dt), 0.0f);
	return true;
}

// 장력 클램프가 질량 의존 추종을 만든다. CL 401 회귀: 리엘 목표 속도 상한이 TetherReelSpeed(400)에
// 묶여 있어, 자유끝 wielder가 그보다 빠르면 *장력을 아무리 올려도* 대상이 영구히 뒤처졌다.
// 문턱 장력 ≈ M·V·fps — 그 이상이면 목표 도달, 미만이면 뒤처진다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionTensionLimitTest,
	"DynamicRope.Traction.TensionLimitMakesHeavyTargetsLag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionTensionLimitTest::RunTest(const FString& Parameters)
{
	const float Dt = 1.0f / 60.0f;
	const float VTarget = 400.0f;
	const float DeltaV = RopeTraction::ComputeAxisDeltaV(0.0f, MakeReelServo(VTarget)); // 정지 → 400 필요.
	TestEqual(TEXT("delta v to reach target from rest"), DeltaV, VTarget);

	// 50kg이 400cm/s에 한 프레임 만에 도달하는 문턱 장력 = 50*400*60 = 1,200,000.
	const float Mass = 50.0f;
	const float Threshold = Mass * VTarget / Dt;

	// 문턱 이상 → 필요 임펄스(mass*dV)를 그대로 낸다 = 목표 정확 도달(뒤처짐 없음).
	const float JAmple = RopeTraction::ClampAxisImpulse(DeltaV, Mass, Threshold * Dt);
	TestEqual(TEXT("ample tension reaches the target exactly"), JAmple, Mass * VTarget);

	// 기본값 150000은 문턱보다 훨씬 작다 → 클램프에 걸려 뒤처진다(무게감). ΔV = J/m = 50cm/s뿐.
	const float JLimited = RopeTraction::ClampAxisImpulse(DeltaV, Mass, 150000.0f * Dt);
	TestEqual(TEXT("default tension is capped for a 50kg target"), JLimited, 150000.0f * Dt);
	TestTrue(TEXT("capped impulse lags the target"), JLimited < Mass * VTarget);

	// 같은 장력이라도 가벼우면(1kg) 문턱을 넘어 목표에 도달한다 = 질량 의존 추종.
	const float JLight = RopeTraction::ClampAxisImpulse(DeltaV, 1.0f, 150000.0f * Dt);
	TestEqual(TEXT("a light target reaches the target under the same tension"), JLight, 1.0f * VTarget);

	// 상한 0 = 무제한(정확 서보) — 질량 무관하게 목표 도달.
	TestEqual(TEXT("zero max impulse means unlimited exact servo"),
		RopeTraction::ClampAxisImpulse(DeltaV, Mass, 0.0f), Mass * VTarget);

	// 제동 방향도 대칭으로 클램프된다(양방향) — 한쪽만 클램프하면 폭주 방어가 비대칭이 된다.
	TestEqual(TEXT("braking impulse is clamped symmetrically"),
		RopeTraction::ClampAxisImpulse(-VTarget, Mass, 150000.0f * Dt), -150000.0f * Dt);
	return true;
}

// bCancelOutward: 바깥 walk는 즉시·완전 상쇄하고(Alpha 무관), 안쪽 회수만 Alpha로 감쇠한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionCancelOutwardTest,
	"DynamicRope.Traction.CancelOutwardIsImmediateButReclaimIsDamped",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionCancelOutwardTest::RunTest(const FString& Parameters)
{
	RopeTraction::FRopeAxisServo Servo = MakeOneWayServo(100.0f, /*Alpha*/ 0.5f);
	Servo.bCancelOutward = true;

	// 바깥으로 200(음수 축)으로 걷는 중: 상쇄 200(즉시·완전) + 목표 100까지 Alpha 0.5 → 50. 합 250.
	TestEqual(TEXT("outward walk is cancelled fully, reclaim is damped"),
		RopeTraction::ComputeAxisDeltaV(-200.0f, Servo), 250.0f);

	// 목표가 0이어도 바깥 walk 상쇄는 그대로 일어난다(경계 유지).
	RopeTraction::FRopeAxisServo StopOnly = MakeOneWayServo(0.0f, /*Alpha*/ 1.0f);
	StopOnly.bCancelOutward = true;
	TestEqual(TEXT("outward walk is cancelled even with a zero target"),
		RopeTraction::ComputeAxisDeltaV(-200.0f, StopOnly), 200.0f);

	// 상쇄 없이(bCancelOutward=false) 목표 0이면 "바깥 속도 제거"와 같아진다(not-pullable 시뮬 바디 경로).
	TestEqual(TEXT("zero-target one-way servo removes outward velocity"),
		RopeTraction::ComputeAxisDeltaV(-200.0f, MakeOneWayServo(0.0f)), 200.0f);
	// 안쪽으로 가는 중이면 손대지 않는다.
	TestEqual(TEXT("zero-target one-way servo leaves inward velocity alone"),
		RopeTraction::ComputeAxisDeltaV(200.0f, MakeOneWayServo(0.0f)), 0.0f);
	return true;
}

// MassShare 분배: 선형 역질량 / 바이어스 지수 / 앵커는 항상 몫 0.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionMassShareTest,
	"DynamicRope.Traction.MassShareSplitsByInverseMass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionMassShareTest::RunTest(const FString& Parameters)
{
	const float WLight = RopeTraction::InvMassFromMass(50.0f);   // 0.02
	const float WHeavy = RopeTraction::InvMassFromMass(150.0f);  // 0.00667
	const float WAnchor = RopeTraction::InvMassFromMass(0.0f);   // 0 (앵커)
	TestEqual(TEXT("anchor mass yields zero inverse mass"), WAnchor, 0.0f);

	// 같은 질량 → 50:50.
	TestEqual(TEXT("equal masses split evenly"),
		RopeTraction::ComputeRawTargetShare(WLight, WLight, 1.0f), 0.5f);

	// 대상이 가볍고(50) wielder가 무거우면(150) 대상이 더 많이 움직인다 = 0.02/(0.02+0.00667) = 0.75.
	TestEqual(TEXT("the lighter end yields more"),
		RopeTraction::ComputeRawTargetShare(WLight, WHeavy, 1.0f), 0.75f, 1e-4f);

	// 앵커 wielder → 대상이 전부 움직인다.
	TestEqual(TEXT("an anchored wielder gives the target the whole share"),
		RopeTraction::ComputeRawTargetShare(WLight, WAnchor, 1.0f), 1.0f);

	// 앵커 대상 → 대상 몫 0(wielder가 전부 양보).
	TestEqual(TEXT("an anchored target takes no share"),
		RopeTraction::ComputeRawTargetShare(WAnchor, WLight, 1.0f), 0.0f);

	// 양끝 다 앵커 → 0(아무도 안 움직임; 호출자가 wielder 몫도 0으로 둔다).
	TestEqual(TEXT("two anchors move nobody"),
		RopeTraction::ComputeRawTargetShare(WAnchor, WAnchor, 1.0f), 0.0f);

	// Bias=0 → 질량 무시 50:50. 단 앵커는 지수와 무관하게 0이어야 한다(Pow(0,0)=1 함정).
	TestEqual(TEXT("zero bias ignores mass"),
		RopeTraction::ComputeRawTargetShare(WLight, WHeavy, 0.0f), 0.5f);
	TestEqual(TEXT("zero bias still gives an anchor no share"),
		RopeTraction::ComputeRawTargetShare(WAnchor, WLight, 0.0f), 0.0f);
	TestEqual(TEXT("zero bias still gives an anchored wielder the whole share"),
		RopeTraction::ComputeRawTargetShare(WLight, WAnchor, 0.0f), 1.0f);

	// Bias>1 → 질량차를 과장한다(가벼운 대상 몫이 선형 0.75보다 커진다).
	TestTrue(TEXT("high bias exaggerates the mass difference"),
		RopeTraction::ComputeRawTargetShare(WLight, WHeavy, 2.0f) > 0.75f);
	return true;
}

// 속력 상한 주입: 상한은 max(SpeedCap, 기존 속력) — 기존의 더 빠른 외부 운동은 보존한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionClampInjectedTest,
	"DynamicRope.Traction.InjectedVelocityCapPreservesFasterMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionClampInjectedTest::RunTest(const FString& Parameters)
{
	// 느리게 있다가 주입으로 상한(1500)을 넘으면 상한으로 잘린다.
	const FVector Clamped = RopeTraction::ClampInjectedVelocity(
		FVector(3000.0f, 0.0f, 0.0f), FVector(100.0f, 0.0f, 0.0f), 1500.0f);
	TestEqual(TEXT("injection is capped at the speed cap"), static_cast<float>(Clamped.Size()), 1500.0f, 1e-2f);

	// 이미 상한보다 빠른 자유낙하(2000) 중이면 그 속력은 보존된다(주입이 더 키우지만 못함).
	const FVector Fast = RopeTraction::ClampInjectedVelocity(
		FVector(0.0f, 0.0f, -3000.0f), FVector(0.0f, 0.0f, -2000.0f), 1500.0f);
	TestEqual(TEXT("pre-existing faster motion is preserved"), static_cast<float>(Fast.Size()), 2000.0f, 1e-2f);

	// SpeedCap=0 = 클램프 없음.
	const FVector Uncapped = RopeTraction::ClampInjectedVelocity(
		FVector(9000.0f, 0.0f, 0.0f), FVector::ZeroVector, 0.0f);
	TestEqual(TEXT("zero cap disables clamping"), static_cast<float>(Uncapped.Size()), 9000.0f, 1e-2f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
