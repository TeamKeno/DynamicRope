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

// 지수 스무딩 계수: 프레임률이 달라도 같은 시상수로 수렴하고, 큰 dt에서도 오버슛하지 않는다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionExpSmoothAlphaTest,
	"DynamicRope.Traction.ExpSmoothAlphaIsFrameRateIndependent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionExpSmoothAlphaTest::RunTest(const FString& Parameters)
{
	// Tau=0(또는 음수) = 스무딩 없음 → 한 프레임에 목표 도달.
	TestEqual(TEXT("zero tau means no smoothing"), RopeTraction::ExpSmoothAlpha(0.0f, 1.0f / 60.0f), 1.0f);
	TestEqual(TEXT("negative tau means no smoothing"), RopeTraction::ExpSmoothAlpha(-1.0f, 1.0f / 60.0f), 1.0f);

	// dt = Tau면 α = 1 - 1/e ≈ 0.632(시상수의 정의).
	TestEqual(TEXT("one time constant leaves 1/e remaining"),
		RopeTraction::ExpSmoothAlpha(0.12f, 0.12f), 1.0f - FMath::Exp(-1.0f), 1e-4f);

	// dt가 아무리 커도 α ≤ 1 — 오버슛(목표를 지나쳐 반대로 튐)이 원천적으로 없다.
	TestTrue(TEXT("a huge dt never overshoots"), RopeTraction::ExpSmoothAlpha(0.12f, 10.0f) <= 1.0f);

	// 프레임률 독립: 60fps로 2프레임 간 잔량 == 30fps로 1프레임 간 잔량(둘 다 exp(-dt/Tau) 곱).
	const float Tau = 0.12f;
	const float Remain60 = (1.0f - RopeTraction::ExpSmoothAlpha(Tau, 1.0f / 60.0f));
	const float Remain30 = (1.0f - RopeTraction::ExpSmoothAlpha(Tau, 1.0f / 30.0f));
	TestEqual(TEXT("two 60fps steps equal one 30fps step"), Remain60 * Remain60, Remain30, 1e-4f);
	return true;
}

// 방향 EMA: 미시드 시드 / 정상 보간 / **180° 반전 축퇴 재시드**. 재시드가 없으면 방향이 0으로 남아 축이 사라진다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionSmoothDirectionTest,
	"DynamicRope.Traction.SmoothDirectionReseedsOnDegenerateFlip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionSmoothDirectionTest::RunTest(const FString& Parameters)
{
	const FVector X = FVector::ForwardVector;

	// 미시드(0) → 측정값으로 시드(래그 없음).
	TestEqual(TEXT("unseeded direction seeds from the target"),
		RopeTraction::SmoothDirection(FVector::ZeroVector, X, 0.5f), X);

	// α=1 → 목표에 즉시 도달.
	TestEqual(TEXT("alpha one snaps to the target"),
		RopeTraction::SmoothDirection(FVector::UpVector, X, 1.0f), X);

	// α=0 → 현재 유지.
	TestEqual(TEXT("alpha zero holds the current direction"),
		RopeTraction::SmoothDirection(X, FVector::UpVector, 0.0f), X);

	// 중간 보간은 단위 벡터로 정규화돼 나온다(길이가 줄어들면 이후 dot 산출이 축소된다).
	const FVector Half = RopeTraction::SmoothDirection(X, FVector::UpVector, 0.5f);
	TestEqual(TEXT("smoothed direction stays unit length"), static_cast<float>(Half.Size()), 1.0f, 1e-4f);

	// 180° 반전 + α=0.5 → Lerp가 정확히 0으로 상쇄된다. 재시드가 없으면 여기서 0이 나온다.
	const FVector Flipped = RopeTraction::SmoothDirection(X, -X, 0.5f);
	TestFalse(TEXT("a 180 degree flip does not collapse to zero"), Flipped.IsNearlyZero());
	TestEqual(TEXT("a degenerate flip reseeds from the target"), Flipped, -X);
	return true;
}

// fractional 조준: 노드 사이 선형 보간. 정수 조준의 이산 홉("뚝뚝 끊김")을 없앤 연속화가 이 함수다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionSampleFractionalAimTest,
	"DynamicRope.Traction.SampleFractionalAimInterpolatesBetweenNodes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionSampleFractionalAimTest::RunTest(const FString& Parameters)
{
	// 노드 x = 0, 100, 200, 300. AnchorNode = 3.
	const TArray<FVector> Positions = {
		FVector(0.0f, 0.0f, 0.0f), FVector(100.0f, 0.0f, 0.0f),
		FVector(200.0f, 0.0f, 0.0f), FVector(300.0f, 0.0f, 0.0f) };

	TestEqual(TEXT("integral aim lands on the node"),
		RopeTraction::SampleFractionalAim(Positions, 1.0f, 3), FVector(100.0f, 0.0f, 0.0f));
	TestEqual(TEXT("fractional aim interpolates between nodes"),
		RopeTraction::SampleFractionalAim(Positions, 1.25f, 3), FVector(125.0f, 0.0f, 0.0f));

	// 앵커 노드에서는 A1이 앵커로 클램프돼 앵커 위치를 준다(범위 밖 인덱스 접근 없음).
	TestEqual(TEXT("aim at the anchor clamps to the anchor node"),
		RopeTraction::SampleFractionalAim(Positions, 3.0f, 3), FVector(300.0f, 0.0f, 0.0f));

	// 빈 배열/범위 밖 → ZeroVector(크래시 없음).
	TestEqual(TEXT("an empty array yields zero"),
		RopeTraction::SampleFractionalAim(TArray<FVector>(), 0.0f, 0), FVector::ZeroVector);
	return true;
}

// 뒤처짐은 리엘 속도 상한이 아니라 **장력**이 지배한다 — CL 401에서 상한을 400→1500으로 올려 "50kg 뒤처짐"을
// 고치려다 실패한 회귀. 장력이 지배하는 구간에서는 상한을 올려도 프레임당 ΔV가 그대로다(반면 가벼운 대상은
// 그 상한까지 순식간에 붙어 위험만 3.75배가 됐다).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionCeilingDoesNotFixLagTest,
	"DynamicRope.Traction.ReelCeilingDoesNotCureTensionLimitedLag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionCeilingDoesNotFixLagTest::RunTest(const FString& Parameters)
{
	const float Dt = 1.0f / 60.0f;
	const float Mass = 50.0f;
	const float MaxImpulse = 150000.0f * Dt; // 기본 장력.
	const float Overshoot = 50.0f;           // taper(1.5cm)보다 훨씬 큼 → 목표 = 고정 리엘 속도.

	// 상한 400 vs 1500: 목표 속도는 3.75배 차이가 난다.
	const float Target400 = RopeTraction::ComputeReelTargetSpeed(Overshoot, 400.0f, 1.5f, Dt);
	const float Target1500 = RopeTraction::ComputeReelTargetSpeed(Overshoot, 1500.0f, 1.5f, Dt);
	TestEqual(TEXT("the 400 ceiling targets 400"), Target400, 400.0f);
	TestEqual(TEXT("the 1500 ceiling targets 1500"), Target1500, 1500.0f);

	// 그런데 50kg에서는 둘 다 장력 클램프에 걸려 **프레임당 ΔV가 동일**하다 → 상한은 뒤처짐을 못 고친다.
	const float Dv400 = RopeTraction::ClampAxisImpulse(
		RopeTraction::ComputeAxisDeltaV(0.0f, MakeReelServo(Target400)), Mass, MaxImpulse) / Mass;
	const float Dv1500 = RopeTraction::ClampAxisImpulse(
		RopeTraction::ComputeAxisDeltaV(0.0f, MakeReelServo(Target1500)), Mass, MaxImpulse) / Mass;
	TestEqual(TEXT("a heavy target accelerates identically under both ceilings"), Dv400, Dv1500);
	TestEqual(TEXT("and that acceleration is set by tension alone"), Dv400, MaxImpulse / Mass);

	// 뒤처짐을 실제로 고치는 건 장력뿐 — 문턱 M·V·fps를 넘기면 한 프레임에 목표 도달.
	const float Threshold = Mass * Target400 / Dt;
	const float DvAmple = RopeTraction::ClampAxisImpulse(
		RopeTraction::ComputeAxisDeltaV(0.0f, MakeReelServo(Target400)), Mass, Threshold * Dt) / Mass;
	TestEqual(TEXT("only raising tension past M*V*fps removes the lag"), DvAmple, Target400);

	// 반면 **가벼운** 대상(1kg)은 같은 장력으로 상한까지 그대로 붙는다 = 상한을 올린 대가는 여기서 치른다.
	const float DvLight = RopeTraction::ClampAxisImpulse(
		RopeTraction::ComputeAxisDeltaV(0.0f, MakeReelServo(Target1500)), 1.0f, MaxImpulse) / 1.0f;
	TestEqual(TEXT("a light target reaches the raised ceiling in one frame"), DvLight, 1500.0f);
	return true;
}

// 팽팽(taut) 게이트: 임계 0 = 종전 하드코딩 게이트(장력 > ~0)와 동일(동작 불변), 임계 > 0이면
// 진입/유지 분리 히스테리시스로 경계 지터 퍼덕임을 막는다. 능동 Pull 인가와 IsPullTaut()의 공용 판정.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionTautGateTest,
	"DynamicRope.Traction.TautGateHysteresisPreventsFlapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionTautGateTest::RunTest(const FString& Parameters)
{
	// 임계 0(기본) = 종전 게이트: 장력이 조금이라도 있으면 팽팽, 0이면 아님 — 래치 상태와 무관(히스테리시스 무력).
	TestTrue(TEXT("zero threshold treats any tension as taut"),
		RopeTraction::EvaluateTautGate(1.0f, 0.0f, 0.5f, /*bWasTaut*/ false));
	TestFalse(TEXT("zero threshold treats zero tension as slack"),
		RopeTraction::EvaluateTautGate(0.0f, 0.0f, 0.5f, /*bWasTaut*/ false));
	TestFalse(TEXT("zero tension is slack even while latched taut"),
		RopeTraction::EvaluateTautGate(0.0f, 0.0f, 0.5f, /*bWasTaut*/ true));

	// 임계 100, 비율 0.5: 진입은 100 초과여야 한다.
	TestFalse(TEXT("tension below the threshold does not enter taut"),
		RopeTraction::EvaluateTautGate(80.0f, 100.0f, 0.5f, /*bWasTaut*/ false));
	TestTrue(TEXT("tension above the threshold enters taut"),
		RopeTraction::EvaluateTautGate(120.0f, 100.0f, 0.5f, /*bWasTaut*/ false));

	// 히스테리시스: 일단 팽팽이면 100×0.5 = 50까지는 유지, 그 아래로 떨어져야 해제.
	TestTrue(TEXT("latched taut survives a dip below the enter threshold"),
		RopeTraction::EvaluateTautGate(80.0f, 100.0f, 0.5f, /*bWasTaut*/ true));
	TestFalse(TEXT("latched taut releases below the stay threshold"),
		RopeTraction::EvaluateTautGate(40.0f, 100.0f, 0.5f, /*bWasTaut*/ true));

	// 비율 1 = 히스테리시스 없음(진입 임계 = 유지 임계) — 80은 래치 여부와 무관하게 슬랙.
	TestFalse(TEXT("ratio one collapses the hysteresis band"),
		RopeTraction::EvaluateTautGate(80.0f, 100.0f, 1.0f, /*bWasTaut*/ true));

	// 비율은 [0..1] 클램프 — 1 초과를 넘겨도 유지 임계가 진입 임계 위로 올라가지 않는다.
	TestTrue(TEXT("an out-of-range ratio is clamped to the enter threshold"),
		RopeTraction::EvaluateTautGate(101.0f, 100.0f, 2.0f, /*bWasTaut*/ true));
	return true;
}

// 전 체인 팽팽(기하) 게이트: chord 합 vs rest 길이 비교 + 진입/유지 히스테리시스 + 판정 불능 방어.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionChainTautGateTest,
	"DynamicRope.Traction.ChainTautGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionChainTautGateTest::RunTest(const FString& Parameters)
{
	// rest 100, 슬랙 허용 3%: 진입 임계 = 97.
	TestTrue(TEXT("chord at rest enters taut"),
		RopeTraction::EvaluateChainTautGate(100.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ false));
	TestTrue(TEXT("chord within the slack ratio enters taut"),
		RopeTraction::EvaluateChainTautGate(97.5f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ false));
	TestFalse(TEXT("sagging chord stays slack"),
		RopeTraction::EvaluateChainTautGate(80.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ false));

	// 히스테리시스(배율 2 → 유지 임계 = 94): 일단 팽팽이면 진입 임계 아래로 살짝 처져도 유지된다.
	TestFalse(TEXT("chord just below the enter threshold does not enter"),
		RopeTraction::EvaluateChainTautGate(96.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ false));
	TestTrue(TEXT("latched taut survives a dip into the hysteresis band"),
		RopeTraction::EvaluateChainTautGate(96.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ true));
	TestFalse(TEXT("latched taut releases below the stay threshold"),
		RopeTraction::EvaluateChainTautGate(90.0f, 100.0f, 0.03f, 2.0f, /*bWasTaut*/ true));

	// 배율 1 = 히스테리시스 없음(진입 = 유지).
	TestFalse(TEXT("release scale one collapses the hysteresis band"),
		RopeTraction::EvaluateChainTautGate(96.0f, 100.0f, 0.03f, 1.0f, /*bWasTaut*/ true));

	// 방어: rest ≤ 0(자유 구간 없음)은 항상 슬랙. 비율×배율 ≥ 1은 "슬랙 전량 허용"으로 수렴 —
	// 래치된 게이트가 chord와 무관하게 유지된다(캡은 음수 임계 방지일 뿐, 의미는 동일).
	TestFalse(TEXT("zero rest length never reports taut"),
		RopeTraction::EvaluateChainTautGate(100.0f, 0.0f, 0.03f, 2.0f, /*bWasTaut*/ true));
	TestTrue(TEXT("ratio times scale of one or more keeps the latched gate open"),
		RopeTraction::EvaluateChainTautGate(0.0f, 100.0f, 0.9f, 5.0f, /*bWasTaut*/ true));
	return true;
}

// 슬랙 브레이크 장부 회수(DecayVelocityDebt): 주입분만 회수·직교 운동량 보존·외부 감속분 자동 탕감.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionDecayVelocityDebtTest,
	"DynamicRope.Traction.DecayVelocityDebt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionDecayVelocityDebtTest::RunTest(const FString& Parameters)
{
	// 전량 회수(Alpha 1): 속도에서 장부 성분만 빠지고 장부는 0.
	{
		FVector Debt(400, 0, 0);
		const FVector NewVel = RopeTraction::DecayVelocityDebt(FVector(600, 0, 0), Debt, 1.0f);
		TestTrue(TEXT("full alpha removes the whole debt from velocity"), NewVel.Equals(FVector(200, 0, 0), 0.1f));
		TestTrue(TEXT("full alpha clears the ledger"), Debt.IsNearlyZero());
	}
	// 부분 회수(Alpha 0.5): 절반만 빼고 잔여는 장부에 남는다.
	{
		FVector Debt(400, 0, 0);
		const FVector NewVel = RopeTraction::DecayVelocityDebt(FVector(600, 0, 0), Debt, 0.5f);
		TestTrue(TEXT("half alpha removes half the debt"), NewVel.Equals(FVector(400, 0, 0), 0.1f));
		TestTrue(TEXT("half alpha keeps the remainder on the ledger"), Debt.Equals(FVector(200, 0, 0), 0.1f));
	}
	// 직교 보존: 장부 방향 성분이 없는 속도(스윙 접선 운동량)는 건드리지 않고 장부는 탕감된다.
	{
		FVector Debt(400, 0, 0);
		const FVector NewVel = RopeTraction::DecayVelocityDebt(FVector(0, 500, 0), Debt, 1.0f);
		TestTrue(TEXT("perpendicular momentum is preserved"), NewVel.Equals(FVector(0, 500, 0), 0.1f));
		TestTrue(TEXT("unavailable debt is forgiven"), Debt.IsNearlyZero());
	}
	// 자동 탕감: 실제 축 성분(100)이 장부(400)보다 작으면 그만큼만 회수 — 역방향으로 밀지 않는다.
	{
		FVector Debt(400, 0, 0);
		const FVector NewVel = RopeTraction::DecayVelocityDebt(FVector(100, 300, 0), Debt, 1.0f);
		TestTrue(TEXT("removal is capped at the available axis component"), NewVel.Equals(FVector(0, 300, 0), 0.1f));
		TestTrue(TEXT("excess debt is forgiven, not carried"), Debt.IsNearlyZero());
	}
	// 탕감 + 부분 회수 조합: 유효 장부(100)의 절반만 회수, 잔여 50만 장부에.
	{
		FVector Debt(400, 0, 0);
		const FVector NewVel = RopeTraction::DecayVelocityDebt(FVector(100, 300, 0), Debt, 0.5f);
		TestTrue(TEXT("forgiven ledger decays from the available amount"), NewVel.Equals(FVector(50, 300, 0), 0.1f));
		TestTrue(TEXT("ledger keeps only the un-recovered available part"), Debt.Equals(FVector(50, 0, 0), 0.1f));
	}
	// 빈 장부: 무동작.
	{
		FVector Debt = FVector::ZeroVector;
		const FVector NewVel = RopeTraction::DecayVelocityDebt(FVector(123, 45, 6), Debt, 1.0f);
		TestTrue(TEXT("empty ledger is a no-op"), NewVel.Equals(FVector(123, 45, 6), 0.01f));
	}
	return true;
}

// 한 프레임 축 ΔV 절대 상한(ClampAxisDeltaV): 0=무제한 통과, 상한 이하 통과, 초과는 ±대칭 클램프.
// 목표 속도 상한과 별개의 방어다 — 목표가 유한해도 현재 속도가 크면 정확 서보의 ΔV는 무제한이다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionClampAxisDeltaVTest,
	"DynamicRope.Traction.ClampAxisDeltaV",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionClampAxisDeltaVTest::RunTest(const FString& Parameters)
{
	// 0(무제한 설정) = 그대로 통과.
	TestEqual(TEXT("무제한(0) 통과"), RopeTraction::ClampAxisDeltaV(3400.0f, 0.0f), 3400.0f);
	TestEqual(TEXT("무제한(음수 설정) 통과"), RopeTraction::ClampAxisDeltaV(-3400.0f, -1.0f), -3400.0f);
	// 상한 이하 = 무손실.
	TestEqual(TEXT("상한 이하 통과"), RopeTraction::ClampAxisDeltaV(200.0f, 333.0f), 200.0f);
	// 상한 초과 = ±대칭 클램프(가속/제동 대칭 — 역전 슬램을 여러 프레임에 분산).
	TestEqual(TEXT("상한 초과 클램프(+)"), RopeTraction::ClampAxisDeltaV(3400.0f, 333.0f), 333.0f);
	TestEqual(TEXT("상한 초과 클램프(-)"), RopeTraction::ClampAxisDeltaV(-3400.0f, 333.0f), -333.0f);
	return true;
}

// 슬램 시나리오 조합: 고속 이탈(축속도 -3000) 대상에 목표 +400 양방향 정확 서보 → raw ΔV=3400을
// 가속 상한(20000cm/s² × 1/60s ≈ 333)이 잘라, 역전이 한 프레임 슬램이 아니라 ~10프레임에 걸쳐 분산된다.
// (Pierce 지면 관통 물리 폭발의 회귀 방어 — MassShare 대상 servo가 이 조합을 쓴다.)
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionSlamScenarioClampTest,
	"DynamicRope.Traction.FastDepartingTargetSlamClamped",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionSlamScenarioClampTest::RunTest(const FString& Parameters)
{
	const RopeTraction::FRopeAxisServo ExactServo{ /*TargetSpeed*/ 400.0f, /*Alpha*/ 1.0f, /*bBidirectional*/ true, /*bCancelOutward*/ false };
	const float RawDeltaV = RopeTraction::ComputeAxisDeltaV(-3000.0f, ExactServo);
	TestEqual(TEXT("정확 서보 raw ΔV = 목표-현재"), RawDeltaV, 3400.0f);

	const float Dt = 1.0f / 60.0f;
	const float MaxDeltaV = 20000.0f * Dt; // 기본 TetherMaxAcceleration × dt
	const float Clamped = RopeTraction::ClampAxisDeltaV(RawDeltaV, MaxDeltaV);
	TestEqual(TEXT("슬램이 가속 상한으로 제한"), Clamped, MaxDeltaV);
	TestTrue(TEXT("한 프레임 역전 불가(현재 속도보다 작은 ΔV)"), Clamped < 3000.0f);

	// 순항 중 보정(예: 200 → 400, ΔV=200 ≤ 상한 333)은 무손실 통과.
	const float CruiseDeltaV = RopeTraction::ComputeAxisDeltaV(200.0f, ExactServo);
	TestEqual(TEXT("순항 보정 ΔV는 상한 이하 통과"), RopeTraction::ClampAxisDeltaV(CruiseDeltaV, MaxDeltaV), CruiseDeltaV);

	// 정지 출발(0 → 400, ΔV=400)은 첫 프레임만 상한(≈333)에 걸리고 **두 프레임 안에 목표 도달** —
	// "기본값 켬이 정상 견인 체감을 바꾸지 않는다"의 정량 근거(1~2프레임 지연이 전부).
	const float Frame1 = RopeTraction::ClampAxisDeltaV(RopeTraction::ComputeAxisDeltaV(0.0f, ExactServo), MaxDeltaV);
	TestEqual(TEXT("정지 출발 첫 프레임은 상한"), Frame1, MaxDeltaV);
	const float Frame2 = RopeTraction::ClampAxisDeltaV(RopeTraction::ComputeAxisDeltaV(Frame1, ExactServo), MaxDeltaV);
	TestTrue(TEXT("두 프레임 내 목표 도달"), Frame1 + Frame2 >= 400.0f - 0.5f);
	return true;
}

// 대상 주입 2차 방어 조합: 주입 결과 벡터가 TetherMaxSpeed를 넘으면 클램프하되, 기존에 더 빠른
// 외부 운동(자유낙하 등)은 보존한다(wielder 쪽 CorrectMovement와 같은 헬퍼를 대상 servo도 쓴다).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTractionTargetInjectSpeedCapTest,
	"DynamicRope.Traction.TargetInjectedSpeedCapPreservesExternalMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTractionTargetInjectSpeedCapTest::RunTest(const FString& Parameters)
{
	const float Cap = 1500.0f;
	// 주입으로 상한을 넘는 경우 → 상한으로 클램프.
	{
		const FVector OldVel(0.0f, 0.0f, 0.0f);
		const FVector NewVel(2000.0f, 0.0f, 0.0f);
		const FVector Clamped = RopeTraction::ClampInjectedVelocity(NewVel, OldVel, Cap);
		TestEqual(TEXT("주입 초과분 클램프"), static_cast<float>(Clamped.Size()), Cap);
	}
	// 기존 속력이 이미 상한 초과(자유낙하 등) → 그 속력까지는 허용(주입이 외부 운동을 깎지 않는다).
	{
		const FVector OldVel(0.0f, 0.0f, -3000.0f);
		const FVector NewVel(300.0f, 0.0f, -3000.0f);
		const FVector Clamped = RopeTraction::ClampInjectedVelocity(NewVel, OldVel, Cap);
		TestTrue(TEXT("기존 고속 외부 운동 보존"), Clamped.Size() >= OldVel.Size() - 1.0f);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
