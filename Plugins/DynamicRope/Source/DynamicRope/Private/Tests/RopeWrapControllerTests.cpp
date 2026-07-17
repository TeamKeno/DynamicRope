// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrapController.h"
#include "RopeTestHelpers.h"

// Pull 산출(ComputePull): 손 쪽 첫 앵커에서 손 쪽 인접 노드 방향 + 해당 세그먼트 장력을 데이터로 내는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapComputePullTest,
	"DynamicRope.Wrap.ComputePullDirectionAndTension",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapComputePullTest::RunTest(const FString& Parameters)
{
	// 노드 x = 0,20,...,140(+X 직선). 앵커 2개(노드 5, 노드 3) → 손 쪽 첫 앵커 = 노드 3.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	Sim.SegmentTension.SetNumZeroed(Sim.Num() - 1);
	// 앵커(3)-손 쪽 인접 노드(2) 세그먼트
	Sim.SegmentTension[2] = 1234.0f;
	// 다른 세그먼트(선택되면 안 됨)
	Sim.SegmentTension[4] = 9999.0f;

	FRopeWrapController Wrap;
	Wrap.State.BoneName = FName("arm");
	for (const int32 NodeIndex : { 5, 3 })
	{
		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = NodeIndex;
		Anchor.Bone = FName("arm");
		Wrap.State.Anchors.Add(Anchor);
	}

	// 코너 임계 30도. 곧은 로프는 손(노드 0)까지 걸어가 방향이 정확히 chord(-X)가 된다.
	const float BendDeg = 30.0f;
	FRopePullSample Pull;
	TestTrue(TEXT("ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
	TestTrue(TEXT("pull sample valid"), Pull.bValid);
	TestEqual(TEXT("hand-side head anchor wins"), Pull.AnchorNode, 3);
	TestTrue(TEXT("bone attributed"), Pull.Bone == FName("arm"));
	// 곧은 로프: 앵커(노드 3, x=60)에서 손 쪽 노드(x<60)는 -X → look-ahead가 chord와 일치한다.
	TestTrue(FString::Printf(TEXT("direction %s points toward hand (-X)"), *Pull.Direction.ToCompactString()),
		Pull.Direction.Equals(FVector(-1, 0, 0), 0.01f));
	TestEqual(TEXT("tension from anchor-hand segment"), Pull.Tension, 1234.0f);

	// 앵커가 노드 0(손 핀)뿐이면 손 쪽 세그먼트가 없어 무효.
	FRopeWrapController WrapAtHand;
	WrapAtHand.State.BoneName = FName("arm");
	FRopeSurfaceAnchor HandAnchor;
	HandAnchor.NodeIndex = 0;
	WrapAtHand.State.Anchors.Add(HandAnchor);
	FRopePullSample InvalidPull;
	TestFalse(TEXT("anchor at hand node yields no pull"), WrapAtHand.ComputePull(Sim, BendDeg, InvalidPull));

	// 꺾인 자유 구간(벽 모서리): 앵커(노드 4)에서 첫 다리는 +Z(위)로 오르고, 모서리(노드 2)에서 손 쪽으로
	// 수평으로 꺾인다. look-ahead 방향은 로프 경로(첫 다리 = +Z)를 따라야 하며, 앵커→손 직선 chord
	// (대각선, 모서리를 가로지름)와 명확히 달라야 한다 — 이게 벽에 걸린 로프에서 chord가 벽을 관통하던 버그의 수정.
	FRopeSimState Bent;
	Bent.Positions = {
		// 0 손
		FVector(-40, 0, 40),
		// 1
		FVector(-20, 0, 40),
		// 2 모서리
		FVector(  0, 0, 40),
		// 3 첫 다리
		FVector(  0, 0, 20),
		// 4 앵커
		FVector(  0, 0,  0),
	};
	Bent.PrevPositions = Bent.Positions;
	Bent.SegmentLength = 20.0f;
	FRopeWrapController WrapBent;
	WrapBent.State.BoneName = FName("arm");
	{
		FRopeSurfaceAnchor A; A.NodeIndex = 4; A.Bone = FName("arm");
		WrapBent.State.Anchors.Add(A);
	}
	FRopePullSample BentPull;
	TestTrue(TEXT("bent ComputePull succeeds"), WrapBent.ComputePull(Bent, BendDeg, BentPull));
	// 첫 다리(노드 4→3→2, +Z)를 걷다 노드 2에서 90도 꺾임 감지 → 멈춤 → 방향 +Z(로프 경로), chord가 아님.
	TestTrue(FString::Printf(TEXT("bent direction %s follows first leg (+Z)"), *BentPull.Direction.ToCompactString()),
		BentPull.Direction.Equals(FVector(0, 0, 1), 0.01f));
	// 대각선(모서리 관통)
	const FVector Chord = (Bent.Positions[0] - Bent.Positions[4]).GetSafeNormal();
	TestFalse(TEXT("bent direction is NOT the straight chord"), BentPull.Direction.Equals(Chord, 0.05f));
	return true;
}

// 전 체인 팽팽 관측치(ComputePull): 코너-다리 chord 합(TautChordLen)과 자유 구간 rest 길이(FreeRestLen)가
// "줄이 다 펴졌는가"를 기하로 구분하는가 — 팽팽 직선/슬랙(압축)/코너에 걸린 팽팽/앵커 다리만 스트레치된
// 슬랙(움직이는 대상 회귀 케이스) 네 가지.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapComputePullChainTautTest,
	"DynamicRope.Wrap.ComputePullChainTautObservables",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapComputePullChainTautTest::RunTest(const FString& Parameters)
{
	const float BendDeg = 30.0f;
	auto MakeWrap = [](int32 AnchorNode)
	{
		FRopeWrapController Wrap;
		Wrap.State.BoneName = FName("arm");
		FRopeSurfaceAnchor A; A.NodeIndex = AnchorNode; A.Bone = FName("arm");
		Wrap.State.Anchors.Add(A);
		return Wrap;
	};

	// ① 곧게 편(팽팽) 로프: 앵커 5, SegmentLength 20 → rest = 100, chord 합 = |P5-P0| = 100.
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
		FRopeWrapController Wrap = MakeWrap(5);
		FRopePullSample Pull;
		TestTrue(TEXT("straight ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
		TestEqual(TEXT("straight rest length"), Pull.FreeRestLen, 100.0f, 0.1f);
		TestEqual(TEXT("straight chord sum equals rest (taut)"), Pull.TautChordLen, 100.0f, 0.1f);
	}

	// ② 압축(슬랙) 로프: 노드 간격이 rest(20)의 절반(10)인 직선 — 굴곡 없이도 chord 합이 rest의 절반.
	// XPBD는 압축에 저항하지 않으므로 "줄이 안 펴진" 대표 형상이다.
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(5, 80.0f);
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.Positions[i] = FVector(10.0f * i, 0, 0);
			Sim.PrevPositions[i] = Sim.Positions[i];
		}
		FRopeWrapController Wrap = MakeWrap(4);
		FRopePullSample Pull;
		TestTrue(TEXT("compressed ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
		TestEqual(TEXT("compressed rest length"), Pull.FreeRestLen, 80.0f, 0.1f);
		TestEqual(TEXT("compressed chord sum is half the rest (slack)"), Pull.TautChordLen, 40.0f, 0.1f);
	}

	// ③ 코너에 걸렸지만 두 다리 모두 팽팽: 앵커(4)→모서리(2) 40 + 모서리(2)→손(0) 40 = rest 80.
	// 코너는 손해가 아니다 — 벽에 걸린 팽팽한 로프는 팽팽으로 인정돼 테더/Pull이 종전대로 발화한다.
	{
		FRopeSimState Bent;
		Bent.Positions = {
			FVector(-40, 0, 40), FVector(-20, 0, 40), FVector(0, 0, 40),
			FVector(0, 0, 20), FVector(0, 0, 0),
		};
		Bent.PrevPositions = Bent.Positions;
		Bent.SegmentLength = 20.0f;
		Bent.InvMass.Init(1.0f, 5);
		FRopeWrapController Wrap = MakeWrap(4);
		FRopePullSample Pull;
		TestTrue(TEXT("bent ComputePull succeeds"), Wrap.ComputePull(Bent, BendDeg, Pull));
		TestEqual(TEXT("bent rest length"), Pull.FreeRestLen, 80.0f, 0.1f);
		TestEqual(TEXT("bent leg chords sum to rest (taut around a corner)"), Pull.TautChordLen, 80.0f, 0.1f);
	}

	// ④ 회귀 케이스(움직이는 대상): 앵커 인접 다리(5→3)만 직선으로 스트레치(세그먼트 27 > rest 20)되고
	// 꼬리(3→0)는 뭉쳐 처짐. sub-leg만 보면 팽팽해 보이지만 전 체인 chord 합은 rest에 한참 못 미친다 —
	// 늘어진 줄이 끌려가던 증상의 판별이 바로 이 차이다.
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(6, 100.0f);
		Sim.Positions = {
			FVector(32, 0, -18), FVector(36, 0, -14), FVector(40, 0, -8),
			FVector(46, 0, 0), FVector(73, 0, 0), FVector(100, 0, 0),
		};
		Sim.PrevPositions = Sim.Positions;
		FRopeWrapController Wrap = MakeWrap(5);
		FRopePullSample Pull;
		TestTrue(TEXT("stretched-leg ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
		TestEqual(TEXT("stretched-leg rest length"), Pull.FreeRestLen, 100.0f, 0.1f);
		// 첫 다리는 꼬리 처짐 직전(노드 3)에서 멈춘다 — 방향/조준은 종전 산출 그대로.
		TestEqual(TEXT("first leg still aims at the bend"), Pull.AimNode, 3);
		TestTrue(FString::Printf(TEXT("chord sum %.1f stays well below rest 100 (slack chain)"), Pull.TautChordLen),
			Pull.TautChordLen < 90.0f);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
