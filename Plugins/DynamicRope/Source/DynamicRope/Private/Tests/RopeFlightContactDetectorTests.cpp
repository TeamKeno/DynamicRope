// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeFlightContactDetector 단위 테스트 — Flight 접촉 감지 파이프라인(실제/예측 접촉 수집,
// 상대운동 평가, 캡처 판정)을 월드 없이 POD fixture + mock collider로 검증한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeFlightContactDetector.h"
#include "Collision/RopeCollider.h"
#include "Components/SceneComponent.h"
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

// 빠른 노드가 얇은 대상을 샘플 사이로 통과하지 않는가(터널링).
// 종전 샘플링은 간격을 SegmentLength(=20cm) 기준으로 잡고 4개로 잘라, 100cm 이동에서 25cm 간격이 됐다 —
// 두께 몇 cm짜리 팔뚝/난간은 그 사이로 그냥 지나갔고 CPU/GPU가 똑같이 틀려 parity 테스트도 통과했다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightThinColliderTunnelTest,
	"DynamicRope.FlightContact.FastNodeDoesNotTunnelThinCollider",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightThinColliderTunnelTest::RunTest(const FString& Parameters)
{
	const FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);

	// 한 프레임에 100cm 이동(x=0 → 100). 두께 3cm 대상이 x=60에 있고 질의 반경 1 → 도달 반경 4cm.
	// 옛 간격 25cm에서는 샘플이 50과 75라 둘 다 4cm 밖(3.75cm 초과)이라 놓쳤다.
	const FVector Prev(0.0f, 0.0f, 0.0f);
	const FVector Curr(100.0f, 0.0f, 0.0f);
	RopeTest::FSphereMockCollider Thin(FVector(60.0f, 0.0f, 0.0f), 3.0f, FName("forearm"));
	TArray<IRopeCollider*> Colliders = { &Thin };

	FRopeFlightContactDetector::FParams Params = MakeDetectParams(/*MinLatchNodes*/ 1,
		/*PredictiveFrames*/ 0.0f, /*ContactRadius*/ 1.0f);
	const FRopeContact Hit = FRopeFlightContactDetector::SweepOrSampleContact(Sim, Prev, Curr, Colliders, Params);
	TestTrue(TEXT("thin collider is found along a fast path"), Hit.bHit);
	TestTrue(TEXT("contact is attributed to the thin bone"), Hit.Bone == FName("forearm"));

	// 상한을 4로 낮추면 간격이 25cm로 되돌아가 다시 놓친다 — 이 테스트가 간격 자체를 보고 있음을 고정한다.
	Params.ContactMaxSweepSamples = 4;
	const FRopeContact Missed = FRopeFlightContactDetector::SweepOrSampleContact(Sim, Prev, Curr, Colliders, Params);
	TestFalse(TEXT("a 4-sample cap tunnels through it again"), Missed.bHit);
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

	const FRopeFlightCaptureEvaluation AtThreshold =
		FRopeFlightContactDetector::EvaluateCapture(Candidates, MakeDetectParams(3));
	TestTrue(TEXT("captures at threshold (3 >= 3)"), AtThreshold.bShouldCapture);
	TestEqual(TEXT("evaluation keeps the deciding nodes"), AtThreshold.Tracker.CandidateNodes.Num(), 3);

	const FRopeFlightCaptureEvaluation AboveThreshold =
		FRopeFlightContactDetector::EvaluateCapture(Candidates, MakeDetectParams(4));
	TestFalse(TEXT("no capture above threshold (3 < 4)"), AboveThreshold.bShouldCapture);
	return true;
}

// Assisted 정책은 조준 본을 dominant로 고르되 같은 mesh의 다른 본도 secondary target으로 보존하는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightCaptureEvaluationPolicyTest,
	"DynamicRope.FlightContact.EvaluationSharesPreferredTracker",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightCaptureEvaluationPolicyTest::RunTest(const FString& Parameters)
{
	const USceneComponent* Mesh = NewObject<USceneComponent>();
	const FName ArmBone("upperarm_l");
	const FName PelvisBone("pelvis");
	auto MakeCandidate = [Mesh](int32 NodeIndex, FName Bone)
	{
		FRopeContactCandidate Candidate;
		Candidate.bValid = true;
		Candidate.NodeIndex = NodeIndex;
		Candidate.Mesh = Mesh;
		Candidate.Bone = Bone;
		Candidate.Penetration = 1.0f;
		return Candidate;
	};

	TArray<FRopeContactCandidate> Candidates;
	Candidates.Add(MakeCandidate(4, ArmBone));
	Candidates.Add(MakeCandidate(7, PelvisBone));
	Candidates.Add(MakeCandidate(8, PelvisBone));
	Candidates.Add(MakeCandidate(9, PelvisBone));

	FRopeFlightCapturePolicy Policy;
	Policy.PreferredMesh = Mesh;
	Policy.PreferredBone = ArmBone;
	Policy.bRequirePreferred = true;
	const FRopeFlightCaptureEvaluation Evaluation =
		FRopeFlightContactDetector::EvaluateCapture(Candidates, MakeDetectParams(1), Policy);

	TestTrue(TEXT("required aim target captures"), Evaluation.bShouldCapture);
	TestTrue(TEXT("evaluation exposes the aim target used by gameplay"),
		Evaluation.Tracker.CandidateMesh == Mesh && Evaluation.Tracker.CandidateBone == ArmBone);
	TestEqual(TEXT("secondary target remains available for Contacting"), Evaluation.Tracker.Targets.Num(), 2);

	TArray<FRopeContactCandidate> PelvisOnly;
	PelvisOnly.Add(MakeCandidate(7, PelvisBone));
	const FRopeFlightCaptureEvaluation MissingPreferred =
		FRopeFlightContactDetector::EvaluateCapture(PelvisOnly, MakeDetectParams(1), Policy);
	TestFalse(TEXT("missing required aim target does not capture"), MissingPreferred.bShouldCapture);
	TestTrue(TEXT("missing required aim target leaves no dominant"),
		MissingPreferred.Tracker.CandidateBone.IsNone());
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightGuidedActualPathTest,
	"DynamicRope.FlightContact.AssistedGuidedPathUsesCpuTargets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightGuidedActualPathTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(4, 60.0f, FVector(0.0f, 100.0f, 0.0f));
	RopeTest::FSphereMockCollider Target(FVector(60.0f, 0.0f, 0.0f), 5.0f, FName("upperarm_l"));
	TArray<IRopeCollider*> Colliders = { &Target };

	TArray<uint8> Mask;
	Mask.SetNumZeroed(Sim.Num());
	Mask[2] = 1;
	TArray<FVector> CurrentTargets, PrevTargets;
	CurrentTargets.SetNumZeroed(Sim.Num());
	PrevTargets.SetNumZeroed(Sim.Num());
	PrevTargets[2] = FVector::ZeroVector;
	CurrentTargets[2] = FVector(100.0f, 0.0f, 0.0f);

	FRopeFlightContactDetector::FWhipGuideView Whip;
	Whip.GuidedNodeMask = &Mask;
	Whip.CurrentTargets = &CurrentTargets;
	Whip.PrevTargets = &PrevTargets;
	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::AddGuidedContactCandidates(
		Sim, Colliders, MakeDetectParams(/*MinLatchNodes*/ 1), Whip, Candidates);

	TestEqual(TEXT("the CPU guide path preserves a same-frame contact pulse"), Candidates.Num(), 1);
	if (Candidates.Num() == 1)
	{
		TestEqual(TEXT("guided contact belongs to the guided node"), Candidates[0].NodeIndex, 2);
		TestEqual(TEXT("guided actual contact keeps aimed bone attribution"), Candidates[0].Bone, FName("upperarm_l"));
		TestEqual(TEXT("guided actual path is not mislabeled predictive"), Candidates[0].Source,
			ERopeContactCandidateSource::Actual);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightGuidedEdgeContactTest,
	"DynamicRope.FlightContact.AssistedGuidedCenterlineDetectsBetweenNodes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightGuidedEdgeContactTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(4, 60.0f, FVector(0.0f, 100.0f, 0.0f));
	RopeTest::FSphereMockCollider ThinTarget(FVector::ZeroVector, 3.0f, FName("forearm_l"));
	TArray<IRopeCollider*> Colliders = { &ThinTarget };

	TArray<uint8> Mask;
	Mask.SetNumZeroed(Sim.Num());
	Mask[1] = 1;
	Mask[2] = 1;
	TArray<FVector> CurrentTargets, PrevTargets;
	CurrentTargets.SetNumZeroed(Sim.Num());
	PrevTargets.SetNumZeroed(Sim.Num());
	CurrentTargets[1] = PrevTargets[1] = FVector(0.0f, -20.0f, 0.0f);
	CurrentTargets[2] = PrevTargets[2] = FVector(0.0f, 20.0f, 0.0f);

	FRopeFlightContactDetector::FWhipGuideView Whip;
	Whip.GuidedNodeMask = &Mask;
	Whip.CurrentTargets = &CurrentTargets;
	Whip.PrevTargets = &PrevTargets;
	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::AddGuidedContactCandidates(
		Sim, Colliders, MakeDetectParams(/*MinLatchNodes*/ 1), Whip, Candidates);

	TestEqual(TEXT("a target between guided nodes is detected on the rope centerline"), Candidates.Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightGuidedBoundaryEdgeContactTest,
	"DynamicRope.FlightContact.AssistedGuidedBoundaryEdgeDetectsBetweenNodes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightGuidedBoundaryEdgeContactTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(3, 40.0f, FVector(-20.0f, 0.0f, 0.0f));
	RopeTest::FSphereMockCollider ThinTarget(FVector::ZeroVector, 3.0f, FName("forearm_l"));
	TArray<IRopeCollider*> Colliders = { &ThinTarget };

	TArray<uint8> Mask;
	Mask.SetNumZeroed(Sim.Num());
	Mask[1] = 1;
	TArray<FVector> CurrentTargets = Sim.Positions;
	TArray<FVector> PrevTargets = Sim.PrevPositions;
	CurrentTargets[1] = PrevTargets[1] = FVector(20.0f, 0.0f, 0.0f);

	FRopeFlightContactDetector::FWhipGuideView Whip;
	Whip.GuidedNodeMask = &Mask;
	Whip.CurrentTargets = &CurrentTargets;
	Whip.PrevTargets = &PrevTargets;
	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::AddGuidedContactCandidates(
		Sim, Colliders, MakeDetectParams(/*MinLatchNodes*/ 1), Whip, Candidates);

	TestEqual(TEXT("a target on the solver-owned to guided boundary edge is detected"), Candidates.Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightGuidedFastSweepBudgetTest,
	"DynamicRope.FlightContact.AssistedGuidedFastPathKeepsSweepSpacing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightGuidedFastSweepBudgetTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(4, 60.0f, FVector(0.0f, 100.0f, 0.0f));
	// 400cm / default cap16 = 25cm spacing. x=12.5의 reach 4cm target은 두 샘플 사이에 완전히
	// 놓이지만 reliable guided budget은 기본 2cm step을 유지해 잡아야 한다.
	RopeTest::FSphereMockCollider ThinTarget(FVector(12.5f, 0.0f, 0.0f), 3.0f, FName("forearm_l"));
	TArray<IRopeCollider*> Colliders = { &ThinTarget };

	TArray<uint8> Mask;
	Mask.SetNumZeroed(Sim.Num());
	Mask[2] = 1;
	TArray<FVector> CurrentTargets = Sim.Positions;
	TArray<FVector> PrevTargets = Sim.PrevPositions;
	PrevTargets[2] = FVector::ZeroVector;
	CurrentTargets[2] = FVector(400.0f, 0.0f, 0.0f);

	FRopeFlightContactDetector::FWhipGuideView Whip;
	Whip.GuidedNodeMask = &Mask;
	Whip.CurrentTargets = &CurrentTargets;
	Whip.PrevTargets = &PrevTargets;
	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::AddGuidedContactCandidates(
		Sim, Colliders, MakeDetectParams(/*MinLatchNodes*/ 1, /*PredictiveFrames*/ 0.0f,
			/*ContactRadius*/ 1.0f), Whip, Candidates);

	TestEqual(TEXT("a thin target survives a 400cm guided frame path"), Candidates.Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightGuidedActualRefreshesDelayedGeometryTest,
	"DynamicRope.FlightContact.AssistedGuidedActualRefreshesDelayedGeometry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightGuidedActualRefreshesDelayedGeometryTest::RunTest(const FString& Parameters)
{
	USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
	const FName Bone("upperarm_l");
	FRopeSimState Sim = RopeTest::MakeStraightRope(4, 60.0f, FVector(0.0f, 100.0f, 0.0f));
	RopeTest::FSphereMockCollider Target(FVector::ZeroVector, 3.0f, Bone, Mesh);
	Target.SurfaceVelocity = FVector(1.0f, 2.0f, 3.0f);
	TArray<IRopeCollider*> Colliders = { &Target };

	TArray<uint8> Mask;
	Mask.SetNumZeroed(Sim.Num());
	Mask[2] = 1;
	TArray<FVector> CurrentTargets = Sim.Positions;
	TArray<FVector> PrevTargets = Sim.PrevPositions;
	PrevTargets[2] = FVector(-20.0f, 0.0f, 0.0f);
	CurrentTargets[2] = FVector(20.0f, 0.0f, 0.0f);
	FRopeFlightContactDetector::FWhipGuideView Whip;
	Whip.GuidedNodeMask = &Mask;
	Whip.CurrentTargets = &CurrentTargets;
	Whip.PrevTargets = &PrevTargets;

	FRopeContactCandidate Delayed;
	Delayed.bValid = true;
	Delayed.NodeIndex = 2;
	Delayed.Bone = Bone;
	Delayed.Mesh = Mesh;
	Delayed.Source = ERopeContactCandidateSource::Actual;
	Delayed.SourceMask = static_cast<uint8>(Delayed.Source);
	Delayed.WorldPoint = FVector(999.0f);
	Delayed.Normal = FVector::ForwardVector;
	Delayed.Penetration = 0.1f;
	Delayed.SurfaceVelocity = FVector(999.0f);
	TArray<FRopeContactCandidate> Candidates = { Delayed };

	FRopeFlightContactDetector::AddGuidedContactCandidates(
		Sim, Colliders, MakeDetectParams(/*MinLatchNodes*/ 1), Whip, Candidates);

	TestEqual(TEXT("same key is merged instead of duplicated"), Candidates.Num(), 1);
	if (Candidates.Num() == 1)
	{
		TestFalse(TEXT("same-frame actual replaces the delayed world point"),
			Candidates[0].WorldPoint.Equals(Delayed.WorldPoint, KINDA_SMALL_NUMBER));
		TestTrue(TEXT("same-frame actual replaces surface velocity"),
			Candidates[0].SurfaceVelocity.Equals(Target.SurfaceVelocity, KINDA_SMALL_NUMBER));
		TestTrue(TEXT("same-frame actual replaces penetration"), Candidates[0].Penetration > Delayed.Penetration);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightAssistedPrimaryShadowTest,
	"DynamicRope.FlightContact.AssistedPrimaryNotShadowedByDeeperNeighbor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightAssistedPrimaryShadowTest::RunTest(const FString& Parameters)
{
	USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
	const FName PrimaryBone("upperarm_l");
	const FName NeighborBone("clavicle_l");
	RopeTest::FSphereMockCollider Primary(FVector(40.0f, 0.0f, 0.0f), 8.0f, PrimaryBone, Mesh);
	RopeTest::FSphereMockCollider DeeperNeighbor(FVector(60.0f, 0.0f, 0.0f), 15.0f, NeighborBone, Mesh);

	FRopeSimState Sim = RopeTest::MakeStraightRope(4, 60.0f, FVector(0.0f, 100.0f, 0.0f));
	Sim.PrevPositions[2] = FVector::ZeroVector;
	Sim.Positions[2] = FVector(100.0f, 0.0f, 0.0f);
	const FRopeFlightContactDetector::FParams Params = MakeDetectParams(/*MinLatchNodes*/ 1);
	TArray<FRopeContactCandidate> Candidates;
	TArray<IRopeCollider*> AllColliders = { &Primary, &DeeperNeighbor };
	FRopeFlightContactDetector::DetectContactCandidates(Sim, AllColliders, Params, Candidates);
	TestTrue(TEXT("the regular deepest-only pass demonstrates the neighboring-bone shadow"),
		Candidates.Num() == 1 && Candidates[0].Bone == NeighborBone);

	TArray<uint8> Mask;
	Mask.SetNumZeroed(Sim.Num());
	Mask[2] = 1;
	TArray<FVector> CurrentTargets = Sim.Positions;
	TArray<FVector> PrevTargets = Sim.PrevPositions;
	FRopeFlightContactDetector::FWhipGuideView Whip;
	Whip.GuidedNodeMask = &Mask;
	Whip.CurrentTargets = &CurrentTargets;
	Whip.PrevTargets = &PrevTargets;
	TArray<IRopeCollider*> ExactPrimary = { &Primary };
	FRopeFlightContactDetector::AddGuidedContactCandidates(Sim, ExactPrimary, Params, Whip, Candidates);

	FRopeFlightCapturePolicy Policy;
	Policy.PreferredMesh = Mesh;
	Policy.PreferredBone = PrimaryBone;
	Policy.bRequirePreferred = true;
	const FRopeFlightCaptureEvaluation Evaluation =
		FRopeFlightContactDetector::EvaluateCapture(Candidates, Params, Policy);
	TestTrue(TEXT("the exact aimed bone survives a deeper same-mesh neighbor"), Evaluation.bShouldCapture);
	TestEqual(TEXT("the aimed bone remains dominant"), Evaluation.Tracker.CandidateBone, PrimaryBone);
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

