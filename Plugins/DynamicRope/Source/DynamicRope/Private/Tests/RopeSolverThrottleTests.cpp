// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeSolverThrottle 슬립 단위 테스트 — Wrapped 정지 스로틀(Free 슬립의 확장) 계약을 월드 없이 고정한다.
// ① Wrapped에서도 침전(저속 유지)이 슬립으로 이어지고 ② 슬립 중 로직 쓰기(Hold 본 추종)가 만든 노드
// 드리프트가 깨우며 ③ bHoldAwake(능동 Pull 장전)가 진입을 막는다. 임계값 계약이 흔들리면 엘리베이터
// 랩 로프가 "움직이는데 자유 구간이 굳는" 종류의 회귀가 나므로 여기서 고정한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeSolverThrottle.h"

namespace
{
	// N노드 정지 체인. 스로틀은 Positions/StartPinTarget만 읽는다.
	FRopeSimState MakeStaticSim(int32 NumNodes)
	{
		FRopeSimState Sim;
		Sim.Positions.SetNum(NumNodes);
		Sim.PrevPositions.SetNum(NumNodes);
		for (int32 i = 0; i < NumNodes; ++i)
		{
			Sim.Positions[i] = FVector(i * 10.0, 0.0, 0.0);
			Sim.PrevPositions[i] = Sim.Positions[i];
		}
		Sim.StartPinTarget = Sim.Positions[0];
		return Sim;
	}

	FRopeSolverConfig MakeSleepConfig()
	{
		FRopeSolverConfig Config;
		Config.bAllowSleep = true;
		Config.SleepVelocityThreshold = 3.0f;
		// 60fps 기준 12프레임이면 진입 — 테스트 프레임 수를 짧게 유지.
		Config.SleepDelay = 0.2f;
		return Config;
	}

	// 정지 상태 측정 프레임을 MaxFrames까지 돌리고 슬립 여부를 반환.
	bool RunSettleFrames(FRopeSolverThrottle& Throttle, const FRopeSimState& Sim,
		const FRopeSolverConfig& Config, ERopePhase Phase, int32 MaxFrames, bool bHoldAwake = false)
	{
		const float Dt = 1.0f / 60.0f;
		for (int32 Frame = 0; Frame < MaxFrames; ++Frame)
		{
			Throttle.UpdateSleepState(Phase, Sim, Config, Dt, bHoldAwake);
			if (Throttle.IsAsleep())
			{
				return true;
			}
		}
		return Throttle.IsAsleep();
	}
}

// Wrapped에서 정지 침전 → 슬립 진입, 이후 노드 드리프트(슬립 중 Hold가 mirror에 쓴 본 이동)가 깨운다.
// 드리프트 임계 0.5cm(콜라이더 정지 판정과 동일) — 미만은 유지, 초과는 wake.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeThrottleWrappedSleepDriftWakeTest,
	"DynamicRope.Throttle.WrappedSleepEntryAndDriftWake",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeThrottleWrappedSleepDriftWakeTest::RunTest(const FString& Parameters)
{
	FRopeSolverThrottle Throttle;
	FRopeSimState Sim = MakeStaticSim(8);
	const FRopeSolverConfig Config = MakeSleepConfig();
	const TArray<IRopeCollider*> NoColliders;

	TestTrue(TEXT("Wrapped 정지 침전이 슬립으로 이어진다"),
		RunSettleFrames(Throttle, Sim, Config, ERopePhase::Wrapped, 60));

	TestFalse(TEXT("정지 유지 중엔 깨우지 않는다"),
		Throttle.ShouldWakeFromSleep(Sim, Config, /*ReelRate*/ 0.0f, NoColliders));

	// 임계 미만 드리프트(0.3cm)는 유지.
	FRopeSimState SubThreshold = Sim;
	SubThreshold.Positions[3] += FVector(0.3, 0.0, 0.0);
	TestFalse(TEXT("0.5cm 미만 노드 드리프트는 유지"),
		Throttle.ShouldWakeFromSleep(SubThreshold, Config, 0.0f, NoColliders));

	// 임계 초과 드리프트(1cm) — 랩 본이 움직였다(엘리베이터 출발).
	FRopeSimState Drifted = Sim;
	Drifted.Positions[3] += FVector(1.0, 0.0, 0.0);
	TestTrue(TEXT("0.5cm 초과 노드 드리프트는 wake"),
		Throttle.ShouldWakeFromSleep(Drifted, Config, 0.0f, NoColliders));

	Throttle.Wake();
	TestFalse(TEXT("Wake 후 슬립 해제"), Throttle.IsAsleep());
	return true;
}

// bHoldAwake=true(능동 Pull 장전) 동안은 정지해도 잠들지 않고, 게이트가 풀리면 delay가 새로 시작돼 잠든다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeThrottleHoldAwakeBlocksSleepTest,
	"DynamicRope.Throttle.HoldAwakeBlocksSleepEntry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeThrottleHoldAwakeBlocksSleepTest::RunTest(const FString& Parameters)
{
	FRopeSolverThrottle Throttle;
	const FRopeSimState Sim = MakeStaticSim(8);
	const FRopeSolverConfig Config = MakeSleepConfig();

	TestFalse(TEXT("bHoldAwake 동안은 정지해도 잠들지 않는다"),
		RunSettleFrames(Throttle, Sim, Config, ERopePhase::Wrapped, 60, /*bHoldAwake*/ true));

	TestTrue(TEXT("게이트 해제 후에는 정상 진입"),
		RunSettleFrames(Throttle, Sim, Config, ERopePhase::Wrapped, 60, /*bHoldAwake*/ false));
	return true;
}

// 비슬립 페이즈(Wrapping)는 측정을 폐기한다 — Wrapped 진입 직후 곧바로 잠드는 이월 오염 방지.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeThrottleNonSleepPhaseResetTest,
	"DynamicRope.Throttle.NonSleepPhaseDiscardsMeasurement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeThrottleNonSleepPhaseResetTest::RunTest(const FString& Parameters)
{
	FRopeSolverThrottle Throttle;
	const FRopeSimState Sim = MakeStaticSim(8);
	const FRopeSolverConfig Config = MakeSleepConfig();
	const float Dt = 1.0f / 60.0f;

	// 침전 직전(delay 미만)까지 누적한 뒤 Wrapping 1프레임 — 누적/캐시가 버려져야 한다.
	for (int32 Frame = 0; Frame < 6; ++Frame)
	{
		Throttle.UpdateSleepState(ERopePhase::Wrapped, Sim, Config, Dt);
	}
	Throttle.UpdateSleepState(ERopePhase::Wrapping, Sim, Config, Dt);

	// 캐시가 버려졌으면 다음 Wrapped 첫 프레임은 비교 기준이 없어 진입까지 다시 delay+1프레임이 필요하다.
	Throttle.UpdateSleepState(ERopePhase::Wrapped, Sim, Config, Dt);
	TestFalse(TEXT("Wrapping 경유 직후 이월 진입 없음"), Throttle.IsAsleep());
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
