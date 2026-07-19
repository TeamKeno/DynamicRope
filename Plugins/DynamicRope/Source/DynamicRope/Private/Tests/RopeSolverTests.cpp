// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Solver/RopeXPBDSolver.h"
#include "Collision/RopeCollider.h"
#include "RopeTestHelpers.h"

namespace
{
	FRopeSolverConfig MakeStiffConfig()
	{
		FRopeSolverConfig C;
		C.Substeps = 4;
		C.Iterations = 8;
		// 비신축(rigid)
		C.StretchCompliance = 0.0f;
		C.BendCompliance = 0.02f;
		C.Gravity = FVector::ZeroVector;
		C.Damping = 0.0f;
		return C;
	}
}

// 2배로 늘린 자유 체인이 여러 스텝 뒤 rest 세그먼트 길이로 수렴하는가(distance 제약).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverDistanceTest,
	"DynamicRope.Solver.DistanceConvergesToRest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverDistanceTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		// 2배 stretch
		Sim.Positions[i] *= 2.0f;
		// 속도 0 유지
		Sim.SetStill(i);
	}

	const FRopeSolverConfig Config = MakeStiffConfig();
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;
	for (int32 Frame = 0; Frame < 60; ++Frame)
	{
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
	}

	const float Err = RopeTest::MaxSegmentError(Sim);
	TestTrue(FString::Printf(TEXT("max segment error %.3f cm should be < 1.0"), Err), Err < 1.0f);
	TestFalse(TEXT("no NaN"), RopeTest::AnyNaN(Sim));
	return true;
}

// Strain limiting: 긴 체인이 핀에 매달려 앵커 인접 세그먼트가 과신장될 때, iteration이 부족해도(it=1)
// substep 끝 순차 클램프가 모든 세그먼트를 ≤ MaxStretchRatio×SegmentLength로 가두는가. 그리고 비활성(0)이면
// 같은 조건에서 상한을 넘는가(클램프가 원인임을 대조로 증명).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverStrainLimitTest,
	"DynamicRope.Solver.StrainLimitBoundsStretch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverStrainLimitTest::RunTest(const FString& Parameters)
{
	auto MakeStretchedPinnedRope = []() -> FRopeSimState
	{
		// 40노드, SegmentLength=10. 노드를 3배 간격(30cm)으로 벌려 모든 세그먼트를 과신장시키고 node0을 핀 고정.
		FRopeSimState S = RopeTest::MakeStraightRope(40, 390.0f); // seg = 390/39 = 10
		for (int32 i = 0; i < S.Num(); ++i)
		{
			const FVector P = FVector(static_cast<float>(i) * 3.0f * S.SegmentLength, 0.0f, 0.0f);
			S.Positions[i] = P;
			S.PrevPositions[i] = P; // 속도 0
		}
		S.bStartPinned = true;
		S.StartPinPrev = S.Positions[0];
		S.StartPinTarget = S.Positions[0];
		S.InvMass[0] = 0.0f;
		return S;
	};

	auto MaxSegmentLen = [](const FRopeSimState& S) -> float
	{
		float M = 0.0f;
		for (int32 i = 0; i + 1 < S.Num(); ++i)
		{
			M = FMath::Max(M, static_cast<float>(FVector::Dist(S.Positions[i], S.Positions[i + 1])));
		}
		return M;
	};

	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;

	// 약한 솔버(it=1)로 strain limit의 단독 기여를 본다.
	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Iterations = 1;
	Config.MaxStretchRatio = 1.5f;

	// (1) strain limit ON: 한 스텝 뒤 모든 세그먼트가 ≤ 1.5×seg.
	{
		FRopeSimState Sim = MakeStretchedPinnedRope();
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
		const float MaxLen = MaxSegmentLen(Sim);
		const float Limit = 1.5f * Sim.SegmentLength;
		TestTrue(FString::Printf(TEXT("strain-limited max segment %.2f should be <= %.2f"), MaxLen, Limit * 1.02f),
			MaxLen <= Limit * 1.02f);
		TestFalse(TEXT("no NaN (strain limit on)"), RopeTest::AnyNaN(Sim));
	}

	// (2) 대조 — strain limit OFF(0): 같은 약한 솔버로는 한 스텝에 상한을 크게 초과한다(클램프가 원인임을 증명).
	{
		FRopeSolverConfig Off = Config;
		Off.MaxStretchRatio = 0.0f;
		FRopeSimState Sim = MakeStretchedPinnedRope();
		Solver.Step(Sim, Off, NoColliders, 1.0f / 60.0f);
		const float MaxLen = MaxSegmentLen(Sim);
		const float Limit = 1.5f * Sim.SegmentLength;
		TestTrue(FString::Printf(TEXT("without strain limit max segment %.2f should exceed %.2f"), MaxLen, Limit),
			MaxLen > Limit);
	}

	return true;
}

// CPU 접촉이 정지 콜라이더 위에서 반발(바깥 법선 속도 주입) 없이 정착하는가 — GPU가 push-out 뒤 VnOut을
// 제거하는 것과 parity. restitution 메모리(CL 189)는 "CPU SolveContacts는 제약식 구조라 미러 불필요"로 판단했다;
// 이 테스트가 그 계약을 못박아 회귀(반발/트램폴린)를 잡는다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverStaticContactNoReboundTest,
	"DynamicRope.Solver.StaticContactNoRebound",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverStaticContactNoReboundTest::RunTest(const FString& Parameters)
{
	// 노드0을 구(sphere) 표면 바로 위 rest 거리에 핀 고정하고, 노드1을 그 아래에서 중력으로 구 위에 떨군다.
	// 표면 z=7(구 반경5 + 노드두께2)에서 rest 거리(200)가 딱 맞아 안착 시 거리 제약력 0 → 순수 중력 vs 접촉.
	FRopeSimState Sim = RopeTest::MakeStraightRope(2, 200.0f); // SegmentLength=200
	Sim.Positions[0]     = FVector(0, 0, 207);
	Sim.PrevPositions[0] = FVector(0, 0, 207);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;
	Sim.Positions[1]     = FVector(0, 0, 100); // 구 위에서 정지 시작 → 중력 낙하
	Sim.PrevPositions[1] = FVector(0, 0, 100);

	RopeTest::FSphereMockCollider Sphere(FVector::ZeroVector, 5.0f, FName("static"));
	const TArray<IRopeCollider*> Colliders = { &Sphere };

	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -980.0f); // 중력으로 표면에 눌러 접촉 유지
	Config.Damping = 0.0f;                          // 감쇠로 반발을 가리지 않는다
	Config.CollisionRadius = 2.0f;                  // 노드 두께 → 표면 z ≈ 5+2 = 7

	const FRopeXPBDSolver Solver;
	// 낙하 + 안착까지 충분히 돌린다.
	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		Solver.Step(Sim, Config, Colliders, 1.0f / 60.0f);
	}
	// 안착 후 반발/트램폴린 관측: 표면 위로 튀는 상방 속도가 생기면 안 된다.
	float MaxZ = -1.0e30f;
	float MaxUpVel = -1.0e30f;
	for (int32 Frame = 0; Frame < 60; ++Frame)
	{
		Solver.Step(Sim, Config, Colliders, 1.0f / 60.0f);
		MaxZ = FMath::Max(MaxZ, static_cast<float>(Sim.Positions[1].Z));
		MaxUpVel = FMath::Max(MaxUpVel, static_cast<float>(Sim.Positions[1].Z - Sim.PrevPositions[1].Z));
	}

	const float SettledZ = static_cast<float>(Sim.Positions[1].Z);
	TestTrue(FString::Printf(TEXT("settles near surface (z=%.2f ~ 7)"), SettledZ), SettledZ > 6.0f && SettledZ < 8.0f);
	TestTrue(FString::Printf(TEXT("never bounces above surface (maxZ=%.2f)"), MaxZ), MaxZ < 8.5f);
	TestTrue(FString::Printf(TEXT("no outward velocity injection (maxUpVel=%.3f cm/substep)"), MaxUpVel), MaxUpVel < 1.0f);
	TestFalse(TEXT("no NaN"), RopeTest::AnyNaN(Sim));
	return true;
}

// InvMass 0 + bStartPinned 노드는 중력 아래에서도 핀 위치를 유지하는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverPinTest,
	"DynamicRope.Solver.PinnedStartHeld",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverPinTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(10, 180.0f);
	const FVector Pin = Sim.Positions[0];
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Pin;
	Sim.StartPinTarget = Pin;
	Sim.InvMass[0] = 0.0f;

	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -980.0f);
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;
	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
	}

	TestTrue(TEXT("pinned node stays at pin"), Sim.Positions[0].Equals(Pin, 0.01f));
	TestFalse(TEXT("no NaN under gravity"), RopeTest::AnyNaN(Sim));
	return true;
}

// 중력 아래 장시간 시뮬레이션에서 발산/NaN 없이 비신축 길이를 유지하는가(explosion 가드).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverStabilityTest,
	"DynamicRope.Solver.StableUnderGravity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverStabilityTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(16, 300.0f);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;

	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -980.0f);
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;
	for (int32 Frame = 0; Frame < 300; ++Frame)
	{
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
	}

	TestFalse(TEXT("no NaN over 300 frames"), RopeTest::AnyNaN(Sim));
	const float Err = RopeTest::MaxSegmentError(Sim);
	TestTrue(FString::Printf(TEXT("segment error %.2f bounded"), Err), Err < Sim.SegmentLength * 2.0f);
	return true;
}

// 세그먼트 장력(XPBD distance λ → F=max(0,-λ)/h²): 매달린 로프에서 위 세그먼트일수록 커야 하고
// (아래 매달린 질량이 많음), 상단 장력은 이론값(아래 노드 수 × g)에 근접해야 한다. 무중력 슬랙은 ~0.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverTensionTest,
	"DynamicRope.Solver.SegmentTensionHangingRope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverTensionTest::RunTest(const FString& Parameters)
{
	constexpr int32 NumNodes = 10;
	constexpr float Gravity = 980.0f;
	FRopeSimState Sim = RopeTest::MakeStraightRope(NumNodes, 180.0f);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;

	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -Gravity);
	// 장력(λ) 수렴 판정이므로 넉넉히.
	Config.Iterations = 32;
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;
	for (int32 Frame = 0; Frame < 240; ++Frame)
	{
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
	}

	TestTrue(TEXT("tension array sized to segments"), Sim.SegmentTension.Num() == NumNodes - 1);

	// 정적 평형에서 세그먼트 k의 장력 = 아래에 매달린 질량 × g = (N-1-k) × 980 (노드 질량 1).
	const float TopExpected = static_cast<float>(NumNodes - 1) * Gravity;
	const float Top = Sim.SegmentTension[0];
	TestTrue(FString::Printf(TEXT("top tension %.0f should be within 50%% of %.0f"), Top, TopExpected),
		Top > TopExpected * 0.5f && Top < TopExpected * 1.5f);

	// 위에서 아래로 단조 감소(수렴 오차 여유 10%).
	for (int32 k = 1; k < Sim.SegmentTension.Num(); ++k)
	{
		TestTrue(FString::Printf(TEXT("tension[%d]=%.0f <= tension[%d]=%.0f (+10%%)"),
			k, Sim.SegmentTension[k], k - 1, Sim.SegmentTension[k - 1]),
			Sim.SegmentTension[k] <= Sim.SegmentTension[k - 1] * 1.1f + 1.0f);
	}

	// 무중력 rest 길이 로프(슬랙) → 장력 ~0.
	FRopeSimState Slack = RopeTest::MakeStraightRope(8, 140.0f);
	// Gravity = 0
	FRopeSolverConfig SlackConfig = MakeStiffConfig();
	for (int32 Frame = 0; Frame < 30; ++Frame)
	{
		Solver.Step(Slack, SlackConfig, NoColliders, 1.0f / 60.0f);
	}
	float SlackMax = 0.0f;
	for (const float T : Slack.SegmentTension) { SlackMax = FMath::Max(SlackMax, T); }
	TestTrue(FString::Printf(TEXT("slack rope max tension %.1f should be ~0"), SlackMax), SlackMax < 1.0f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

