// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeFlightContactDetector 단위 테스트 — Flight 접촉 감지 파이프라인(실제/예측 접촉 수집,
// 상대운동 평가, 캡처 판정)을 월드 없이 POD fixture + mock collider로 검증한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeFlightContactDetector.h"
#include "Collision/RopeCollider.h"
#include "RopeTestHelpers.h"

namespace
{
	FRopeFlightContactDetector::FParams MakeDetectParams(int32 MinLatchNodes = 3,
		float PredictiveFrames = 0.0f, float ContactRadius = 3.0f, float RopeRadius = 2.0f)
	{
		FRopeFlightContactDetector::FParams Params;
		Params.ContactRadius = ContactRadius;
		Params.RopeRadius = RopeRadius;
		Params.PredictiveContactFrames = PredictiveFrames;
		Params.MinLatchNodes = MinLatchNodes;
		Params.FallbackForward = FVector::ForwardVector;
		return Params;
	}
}

// 정지 로프가 collider 반경 안에 있으면 해당 노드들이 Actual 후보로 수집되는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightDetectActualTest,
	"DynamicRope.FlightContact.DetectsActualContactAlongPath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightDetectActualTest::RunTest(const FString& Parameters)
{
	// 노드 x = 0,20,...,140. Arm(center 60, r25)+ContactRadius 3 = reach 28 → 노드 40/60/80 접촉(3개).
	const FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	RopeTest::FSphereMockCollider Arm(FVector(60.0f, 0.0f, 0.0f), 25.0f, FName("arm"));
	TArray<IRopeCollider*> Colliders = { &Arm };

	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, Colliders, MakeDetectParams(), Candidates);

	TestEqual(TEXT("three nodes in contact"), Candidates.Num(), 3);
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		TestTrue(TEXT("candidate valid"), Candidate.bValid);
		TestTrue(TEXT("candidate bone is arm"), Candidate.Bone == FName("arm"));
		TestTrue(TEXT("candidate source is Actual"), Candidate.Source == ERopeContactCandidateSource::Actual);
	}
	return true;
}

// dominant bone 접촉 노드 수가 MinLatchNodes 문턱을 넘을 때만 캡처 판정하는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightShouldCaptureTest,
	"DynamicRope.FlightContact.ShouldCaptureRespectsMinLatchNodes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightShouldCaptureTest::RunTest(const FString& Parameters)
{
	// 위 테스트와 같은 fixture → 노드 3개 접촉.
	const FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	RopeTest::FSphereMockCollider Arm(FVector(60.0f, 0.0f, 0.0f), 25.0f, FName("arm"));
	TArray<IRopeCollider*> Colliders = { &Arm };

	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, Colliders, MakeDetectParams(), Candidates);
	TestEqual(TEXT("fixture produces three candidates"), Candidates.Num(), 3);

	TestTrue(TEXT("captures at threshold (3 >= 3)"),
		FRopeFlightContactDetector::ShouldCapture(Candidates, MakeDetectParams(3)));
	TestFalse(TEXT("no capture above threshold (3 < 4)"),
		FRopeFlightContactDetector::ShouldCapture(Candidates, MakeDetectParams(4)));
	return true;
}

// 아직 닿지 않았지만 이동 방향 외삽 경로가 collider를 지나는 tail 노드가 PredictiveFree로 승격되는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightPredictFreeTest,
	"DynamicRope.FlightContact.PredictsFreeNodeContactAhead",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightPredictFreeTest::RunTest(const FString& Parameters)
{
	// tip(노드 7, x=140)이 +X로 프레임당 20 이동 중. Arm(center 180, r10)+3 = reach 13 →
	// 현재 위치(140)는 접촉 밖이지만 3프레임 외삽(140→200) 경로가 sphere를 관통한다.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	Sim.PrevPositions[7] = FVector(120.0f, 0.0f, 0.0f);
	RopeTest::FSphereMockCollider Arm(FVector(180.0f, 0.0f, 0.0f), 10.0f, FName("arm"));
	TArray<IRopeCollider*> Colliders = { &Arm };

	const FRopeFlightContactDetector::FParams Params = MakeDetectParams(3, /*PredictiveFrames*/ 3.0f);

	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, Colliders, Params, Candidates);
	TestEqual(TEXT("no actual contact yet"), Candidates.Num(), 0);

	FRopeFlightContactDetector::AddPredictedContactCandidates(Sim, Colliders, Params,
		FRopeFlightContactDetector::FWhipGuideView(), Candidates);
	TestEqual(TEXT("one predicted candidate"), Candidates.Num(), 1);
	if (Candidates.Num() == 1)
	{
		TestEqual(TEXT("predicted node is tip"), Candidates[0].NodeIndex, 7);
		TestTrue(TEXT("source is PredictiveFree"), Candidates[0].Source == ERopeContactCandidateSource::PredictiveFree);
		TestTrue(TEXT("bone is arm"), Candidates[0].Bone == FName("arm"));
	}
	return true;
}

// whip 가이드 노드의 현재→다음 타깃 외삽 경로에서 PredictiveGuided 후보가 나오는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightPredictGuidedTest,
	"DynamicRope.FlightContact.PredictsGuidedNodeContactFromTargets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightPredictGuidedTest::RunTest(const FString& Parameters)
{
	// 노드 2만 가이드. 현재 타깃 (50,0,50) → 다음 타깃 (50,0,25), 2프레임 외삽 → (50,0,0).
	// Arm(center (50,0,0), r5)+3 = reach 8: 외삽 경로 끝이 sphere 중심을 때린다.
	const FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	RopeTest::FSphereMockCollider Arm(FVector(50.0f, 0.0f, 0.0f), 5.0f, FName("arm"));
	TArray<IRopeCollider*> Colliders = { &Arm };

	TArray<uint8> Mask;
	Mask.SetNumZeroed(Sim.Num());
	Mask[2] = 1;
	TArray<FVector> CurrentTargets, PrevTargets, NextTargets;
	CurrentTargets.SetNum(3); PrevTargets.SetNum(3); NextTargets.SetNum(3);
	CurrentTargets[2] = FVector(50.0f, 0.0f, 50.0f);
	PrevTargets[2] = FVector(50.0f, 0.0f, 50.0f);
	NextTargets[2] = FVector(50.0f, 0.0f, 25.0f);

	FRopeFlightContactDetector::FWhipGuideView Whip;
	Whip.GuidedNodeMask = &Mask;
	Whip.CurrentTargets = &CurrentTargets;
	Whip.PrevTargets = &PrevTargets;
	Whip.NextTargets = &NextTargets;

	const FRopeFlightContactDetector::FParams Params = MakeDetectParams(3, /*PredictiveFrames*/ 2.0f);

	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::AddPredictedContactCandidates(Sim, Colliders, Params, Whip, Candidates);

	TestEqual(TEXT("one guided predicted candidate"), Candidates.Num(), 1);
	if (Candidates.Num() == 1)
	{
		TestEqual(TEXT("candidate is guided node"), Candidates[0].NodeIndex, 2);
		TestTrue(TEXT("source is PredictiveGuided"), Candidates[0].Source == ERopeContactCandidateSource::PredictiveGuided);
	}
	return true;
}

// 감김 방향 점수: 손 쪽으로 미끄러지면 +, 반대면 -, 표면과 같이 움직이면(상대속도 0) 0인가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightWrapDirectionScoreTest,
	"DynamicRope.FlightContact.WrapDirectionScoreSign",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightWrapDirectionScoreTest::RunTest(const FString& Parameters)
{
	// 손(노드 0)이 원점, 접촉 노드 1이 (100,0,0). 표면 normal +Z → 접선면은 XY.
	FRopeSimState Sim = RopeTest::MakeStraightRope(2, 100.0f);
	const FRopeFlightContactDetector::FParams Params = MakeDetectParams();

	FRopeContactCandidate Candidate;
	Candidate.bValid = true;
	Candidate.NodeIndex = 1;
	Candidate.Bone = FName("arm");
	Candidate.WorldPoint = FVector(100.0f, 0.0f, 0.0f);
	Candidate.Normal = FVector::UpVector;

	// 손 쪽(-X)으로 5 이동 → 기대 감김 접선과 정렬 → 점수 +1.
	Sim.PrevPositions[1] = FVector(105.0f, 0.0f, 0.0f);
	TArray<FRopeContactCandidate> Candidates = { Candidate };
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, Params, Candidates);
	TestTrue(TEXT("toward-hand slide scores positive"), Candidates[0].WrapDirectionScore > 0.9f);
	TestTrue(TEXT("tangential speed matches displacement"),
		FMath::IsNearlyEqual(Candidates[0].RelativeTangentialSpeed, 5.0f, 0.01f));

	// 반대(+X) 이동 → 점수 -1.
	Sim.PrevPositions[1] = FVector(95.0f, 0.0f, 0.0f);
	Candidates = { Candidate };
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, Params, Candidates);
	TestTrue(TEXT("away-from-hand slide scores negative"), Candidates[0].WrapDirectionScore < -0.9f);

	// 표면이 로프와 같은 속도로 움직이면(상대속도 0) 미끄러짐 없음.
	// SurfaceVelocity는 cm/s 계약이므로 substep dt를 명시하고, 변위 -5cm와 등속이 되는 cm/s 값을 넣는다.
	FRopeFlightContactDetector::FParams DtParams = Params;
	DtParams.SubstepDeltaTime = 0.02f;
	Sim.PrevPositions[1] = FVector(105.0f, 0.0f, 0.0f);
	// -250cm/s × 0.02s = -5cm(이 변위의 substep 폭)
	Candidate.SurfaceVelocity = FVector(-250.0f, 0.0f, 0.0f);
	Candidates = { Candidate };
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DtParams, Candidates);
	TestTrue(TEXT("co-moving surface yields no tangential slide"),
		Candidates[0].RelativeTangentialSpeed < 0.01f);

	return true;
}

// 캡처 순간 진행 좌표계 스냅샷(FRopeCaptureTravelFrame::Compute, 진행 방향 기반 wrap 2단계):
// 접촉 영역 중심/평균 속도/누운 방향에서 진행 평면 normal(속도×span)을 유도하고,
// 속도와 span이 평행(창던지기)이거나 dt=0이면 normal 없이(bHasPlaneNormal=false) 폴백 신호를 남긴다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCaptureTravelFrameTest,
	"DynamicRope.FlightContact.CaptureTravelFrameSnapshot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCaptureTravelFrameTest::RunTest(const FString& Parameters)
{
	// 로프는 Y로 누워 있고(노드 간격 20) 전체가 +X로 비행 중(프레임당 10cm, dt 0.1 → 100cm/s).
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f, FVector::ZeroVector, FVector(0, 1, 0));
	for (int32 Index = 0; Index < Sim.Num(); ++Index)
	{
		Sim.PrevPositions[Index] = Sim.Positions[Index] - FVector(10.0f, 0.0f, 0.0f);
	}

	auto MakeCandidate = [&Sim](int32 Node)
	{
		FRopeContactCandidate C;
		C.bValid = true;
		C.NodeIndex = Node;
		C.Bone = FName("thigh_l");
		C.WorldPoint = Sim.Positions[Node] + FVector(2.0f, 0.0f, 0.0f);
		return C;
	};

	TArray<FRopeContactCandidate> Candidates = { MakeCandidate(3), MakeCandidate(4), MakeCandidate(5) };

	const FRopeCaptureTravelFrame Frame = FRopeCaptureTravelFrame::Compute(Sim, Candidates, 0.1f);
	TestTrue(TEXT("frame captured"), Frame.bValid);
	TestTrue(TEXT("region center is the contact average"),
		Frame.RegionCenter.Equals(FVector(2.0f, 80.0f, 0.0f), 0.1f));
	TestTrue(TEXT("average velocity follows the flight direction"),
		Frame.AverageVelocity.Equals(FVector(100.0f, 0.0f, 0.0f), 0.1f));
	TestTrue(TEXT("span direction follows the rope lay (Y)"),
		FMath::Abs(FVector::DotProduct(Frame.SpanDirection, FVector(0, 1, 0))) > 0.99f);
	TestTrue(TEXT("plane normal derived (velocity x span = Z)"), Frame.bHasPlaneNormal);
	TestTrue(TEXT("plane normal is Z"),
		FMath::Abs(FVector::DotProduct(Frame.PlaneNormal, FVector(0, 0, 1))) > 0.99f);

	// 축퇴 ①: 창던지기 — 로프가 누운 방향(+Y)으로 그대로 비행하면 normal을 만들 수 없다.
	for (int32 Index = 0; Index < Sim.Num(); ++Index)
	{
		Sim.PrevPositions[Index] = Sim.Positions[Index] - FVector(0.0f, 10.0f, 0.0f);
	}
	const FRopeCaptureTravelFrame Javelin = FRopeCaptureTravelFrame::Compute(Sim, Candidates, 0.1f);
	TestTrue(TEXT("javelin frame still captured"), Javelin.bValid);
	TestTrue(TEXT("javelin flight yields no plane normal"), !Javelin.bHasPlaneNormal);

	// 축퇴 ②: dt=0 — 속도 환산 불가. 스냅샷은 유효하되 normal 없음.
	const FRopeCaptureTravelFrame NoDt = FRopeCaptureTravelFrame::Compute(Sim, Candidates, 0.0f);
	TestTrue(TEXT("zero-dt frame still captured"), NoDt.bValid);
	TestTrue(TEXT("zero-dt velocity is zero"), NoDt.AverageVelocity.IsNearlyZero());
	TestTrue(TEXT("zero-dt yields no plane normal"), !NoDt.bHasPlaneNormal);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

