// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Solver/RopeXPBDSolver.h"
#include "Collision/RopeCollider.h"
#include "Logic/RopeWhipGuide.h"
#include "RopeTestHelpers.h"

namespace
{
	FRopeSolverConfig MakeStiffConfig()
	{
		FRopeSolverConfig C;
		C.Substeps = 4;
		C.Iterations = 8;
		C.StretchCompliance = 0.0f; // 비신축(rigid)
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
		Sim.Positions[i] *= 2.0f; // 2배 stretch
		Sim.SetStill(i);          // 속도 0 유지
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
	Guide.Advance(1.0f / 60.0f, Sim, Config, /*bCaptureDebugTargets*/ false);

	TestTrue(TEXT("middle node remains spline-guided"), Guide.IsGuidedNodeThisFrame(MiddleNode));
	TestFalse(TEXT("tip node is released to solver"), Guide.IsGuidedNodeThisFrame(LastNode));
	TestTrue(TEXT("released tip keeps solver position before solve"),
		Guide.GetCurrentTargets().IsValidIndex(LastNode) &&
		Guide.GetCurrentTargets()[LastNode].Equals(FreeTipBefore, 0.01f));
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
	Config.Iterations = 32; // 장력(λ) 수렴 판정이므로 넉넉히.
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
	FRopeSolverConfig SlackConfig = MakeStiffConfig(); // Gravity = 0
	for (int32 Frame = 0; Frame < 30; ++Frame)
	{
		Solver.Step(Slack, SlackConfig, NoColliders, 1.0f / 60.0f);
	}
	float SlackMax = 0.0f;
	for (const float T : Slack.SegmentTension) { SlackMax = FMath::Max(SlackMax, T); }
	TestTrue(FString::Printf(TEXT("slack rope max tension %.1f should be ~0"), SlackMax), SlackMax < 1.0f);
	return true;
}

// 움직이는 캡슐의 Query가 접촉 재질점의 표면 속도를 보고하는가(FRopeContact 계약: cm/s, 정적이면 0).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCapsuleSurfaceVelocityTest,
	"DynamicRope.Collision.CapsuleSurfaceVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCapsuleSurfaceVelocityTest::RunTest(const FString& Parameters)
{
	// Z축 캡슐(반지름 10)이 한 프레임(1/60s)에 +X로 6cm 이동. 노드는 축 중간 높이에서 +X쪽 표면에 겹침.
	FCapsuleCollider Cap(FVector(6, 0, 0), FVector(6, 0, 100), 10.0f, FName(TEXT("bone")));
	Cap.PrevA = FVector(0, 0, 0);
	Cap.PrevB = FVector(0, 0, 100);
	Cap.InvDeltaTime = 60.0f;

	const FRopeContact Contact = Cap.Query(FVector(14, 0, 50), 2.0f);
	TestTrue(TEXT("node overlaps capsule"), Contact.bHit);
	// 재질점(축 위 z=50)의 프레임 변위 = +X 6cm → 표면 속도 = 6 * 60 = 360 cm/s.
	TestTrue(FString::Printf(TEXT("surface velocity %s should be ~(360,0,0)"), *Contact.SurfaceVelocity.ToString()),
		Contact.SurfaceVelocity.Equals(FVector(360, 0, 0), 1.0f));
	TestTrue(TEXT("normal points outward (+X)"), Contact.Normal.Equals(FVector(1, 0, 0), 0.01f));

	// 정적 캡슐(InvDeltaTime 0)은 표면 속도 0 — 기존 동작 유지.
	FCapsuleCollider StaticCap(FVector(6, 0, 0), FVector(6, 0, 100), 10.0f);
	const FRopeContact StaticContact = StaticCap.Query(FVector(14, 0, 50), 2.0f);
	TestTrue(TEXT("static capsule overlaps"), StaticContact.bHit);
	TestTrue(TEXT("static capsule surface velocity is zero"), StaticContact.SurfaceVelocity.IsNearlyZero());
	return true;
}

// 움직이는 캡슐의 QuerySwept가 정지 노드를 추월할 때 접근(앞)면에서 잡는가(상대 운동 CCD).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCapsuleSweptRelativeMotionTest,
	"DynamicRope.Collision.CapsuleSweptRelativeMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCapsuleSweptRelativeMotionTest::RunTest(const FString& Parameters)
{
	// Z축 캡슐(반지름 10)이 한 프레임에 X=-40 → X=+40으로 80cm 이동. 노드는 X=0에 정지 —
	// 끝 포즈만 보면 캡슐이 이미 노드를 지나쳐(거리 40 > 12) 접촉 자체가 없다(= 관통). 상대 운동 스윕은
	// 접근 중 첫 접촉을 잡고, 노드를 캡슐 앞면(+X, 진행 방향)으로 밀어내는 법선을 보고해야 한다.
	FCapsuleCollider Cap(FVector(40, 0, 0), FVector(40, 0, 100), 10.0f, FName(TEXT("bone")));
	Cap.PrevA = FVector(-40, 0, 0);
	Cap.PrevB = FVector(-40, 0, 100);
	Cap.InvDeltaTime = 60.0f;

	FRopeSweptQuery Q;
	Q.WorldStart = FVector(0, 0, 50); // 정지 노드(이동 없음)
	Q.WorldEnd = FVector(0, 0, 50);
	Q.NodeRadius = 2.0f;
	Q.SweepStep = 2.0f;
	Q.MaxSamples = 64;
	Q.SubAlpha0 = 0.0f; // 프레임 전체를 한 substep으로
	Q.SubAlpha1 = 1.0f;

	FVector HitPos;
	const FRopeContact Contact = Cap.QuerySwept(Q, HitPos);
	TestTrue(TEXT("overtaking capsule is caught by relative sweep"), Contact.bHit);
	TestTrue(FString::Printf(TEXT("normal %s should push node ahead (+X)"), *Contact.Normal.ToString()),
		Contact.Normal.X > 0.9f);
	// 표면 속도는 캡슐 이동 방향(+X), 80cm/frame * 60 = 4800 cm/s.
	TestTrue(FString::Printf(TEXT("surface velocity %s should be ~(4800,0,0)"), *Contact.SurfaceVelocity.ToString()),
		Contact.SurfaceVelocity.Equals(FVector(4800, 0, 0), 10.0f));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
