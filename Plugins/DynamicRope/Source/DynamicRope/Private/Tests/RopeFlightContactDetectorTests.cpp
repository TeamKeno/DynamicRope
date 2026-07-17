// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeFlightContactDetector 단위 테스트 — Flight 접촉 감지 파이프라인(실제/예측 접촉 수집,
// 상대운동 평가, 캡처 판정)을 월드 없이 POD fixture + mock collider로 검증한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeFlightContactDetector.h"
#include "Logic/RopeAimTargeting.h"
#include "Collision/RopeCollider.h"
// 트래커 cross-mesh 테스트의 식별용 mock 컴포넌트(NewObject<USceneComponent>).
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
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

// 트래커가 (Mesh, Bone) 쌍으로 집계하는가: 같은 본 이름을 쓰는 두 액터(mesh)가 한 프레임에 함께 닿아도
// 후보가 한 버킷으로 합산되거나 mesh가 마지막 후보로 오귀속되지 않고, 본 이름이 같아도 mesh가 바뀌면
// dwell이 리셋된다(cross-actor 캡처 오귀속 수정 계약 — 2026-07 주석 전수조사 발견 건).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightTrackerCrossMeshTest,
	"DynamicRope.FlightContact.TrackerSeparatesSameBoneAcrossMeshes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightTrackerCrossMeshTest::RunTest(const FString& Parameters)
{
	// 식별용 컴포넌트 2개(월드 불필요 — 트래커는 포인터를 역참조하지 않는다).
	const USceneComponent* MeshA = NewObject<USceneComponent>();
	const USceneComponent* MeshB = NewObject<USceneComponent>();

	auto MakeCandidate = [](int32 Node, const USceneComponent* Mesh)
	{
		FRopeContactCandidate C;
		C.bValid = true;
		C.NodeIndex = Node;
		C.Bone = FName("hand_r");
		C.Mesh = Mesh;
		C.Penetration = 1.0f;
		return C;
	};

	// A에 2노드, B에 1노드(전부 같은 본 이름) — dominant는 (MeshA, hand_r)이어야 하고
	// B의 후보와 합산되면 안 된다(합산됐다면 노드 3개 + mesh가 B로 덮였을 것).
	TArray<FRopeContactCandidate> Candidates;
	Candidates.Add(MakeCandidate(5, MeshA));
	Candidates.Add(MakeCandidate(6, MeshA));
	Candidates.Add(MakeCandidate(2, MeshB));

	FRopeContactTracker Tracker;
	Tracker.Update(Candidates, 0.10f);
	TestTrue(TEXT("dominant bone is hand_r"), Tracker.CandidateBone == FName("hand_r"));
	TestTrue(TEXT("dominant mesh is A (no cross-mesh merge)"), Tracker.CandidateMesh == MeshA);
	TestEqual(TEXT("only A's nodes tracked"), Tracker.CandidateNodes.Num(), 2);

	// 같은 대상 유지 → dwell 누적.
	Tracker.Update(Candidates, 0.10f);
	TestTrue(TEXT("dwell accumulates on same (mesh, bone) target"), Tracker.DwellTime > 0.05f);

	// dominant가 (MeshB, hand_r)로 넘어가면 — 본 이름은 그대로여도 — dwell이 리셋되어야 한다.
	TArray<FRopeContactCandidate> Flipped;
	Flipped.Add(MakeCandidate(1, MeshB));
	Flipped.Add(MakeCandidate(2, MeshB));
	Flipped.Add(MakeCandidate(3, MeshB));
	Flipped.Add(MakeCandidate(5, MeshA));
	Tracker.Update(Flipped, 0.10f);
	TestTrue(TEXT("dominant switched to mesh B"), Tracker.CandidateMesh == MeshB);
	TestTrue(TEXT("same bone name on a different mesh resets dwell"), Tracker.DwellTime < KINDA_SMALL_NUMBER);
	return true;
}

// 시드 다중화 재료: 트래커가 dominant 외의 접촉 대상도 (Mesh, Bone)별 dwell과 함께 유지하고
// (Targets), 접촉이 빠진 대상은 같은 비율로 감쇠하다 소진되면 목록에서 빠지는가. dominant
// 선정/리셋 계약은 Targets 도입과 무관하게 유지된다(위 테스트가 고정).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightTrackerMultiTargetTest,
	"DynamicRope.FlightContact.TrackerKeepsSecondaryTargetDwell",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightTrackerMultiTargetTest::RunTest(const FString& Parameters)
{
	const USceneComponent* Mesh = NewObject<USceneComponent>();

	auto MakeCandidate = [Mesh](int32 Node, const FName& Bone)
	{
		FRopeContactCandidate C;
		C.bValid = true;
		C.NodeIndex = Node;
		C.Bone = Bone;
		C.Mesh = Mesh;
		C.Penetration = 1.0f;
		return C;
	};

	// 양다리 시나리오 축약: 한 mesh의 두 본에 동시 접촉(thigh_l 노드 1,2 / calf_r 노드 6,7).
	TArray<FRopeContactCandidate> BothLegs;
	BothLegs.Add(MakeCandidate(1, FName("thigh_l")));
	BothLegs.Add(MakeCandidate(2, FName("thigh_l")));
	BothLegs.Add(MakeCandidate(6, FName("calf_r")));
	BothLegs.Add(MakeCandidate(7, FName("calf_r")));

	FRopeContactTracker Tracker;
	Tracker.Update(BothLegs, 0.10f);
	Tracker.Update(BothLegs, 0.10f);

	TestTrue(TEXT("dominant is head-side thigh_l"), Tracker.CandidateBone == FName("thigh_l"));
	TestEqual(TEXT("both targets tracked"), Tracker.Targets.Num(), 2);

	const FRopeTrackedContactTarget* Secondary = Tracker.Targets.FindByPredicate(
		[](const FRopeTrackedContactTarget& Target) { return Target.Bone == FName("calf_r"); });
	if (!TestNotNull(TEXT("secondary target present"), Secondary))
	{
		return false;
	}
	TestTrue(TEXT("secondary dwell accumulates independently"), Secondary->DwellTime > 0.05f);
	TestEqual(TEXT("secondary nodes current"), Secondary->Nodes.Num(), 2);

	// calf_r 접촉이 끊기면: dwell이 같은 비율로 감쇠(우선 잔존 — 짧은 플리커 관용), 소진되면 제거.
	TArray<FRopeContactCandidate> OneLeg;
	OneLeg.Add(MakeCandidate(1, FName("thigh_l")));
	OneLeg.Add(MakeCandidate(2, FName("thigh_l")));

	Tracker.Update(OneLeg, 0.04f);
	Secondary = Tracker.Targets.FindByPredicate(
		[](const FRopeTrackedContactTarget& Target) { return Target.Bone == FName("calf_r"); });
	if (!TestNotNull(TEXT("secondary survives a short flicker"), Secondary))
	{
		return false;
	}
	TestTrue(TEXT("flickering secondary has no current nodes"), Secondary->Nodes.Num() == 0);

	Tracker.Update(OneLeg, 0.10f);
	TestEqual(TEXT("exhausted secondary is dropped"), Tracker.Targets.Num(), 1);
	TestTrue(TEXT("dominant unaffected by secondary decay"), Tracker.CandidateBone == FName("thigh_l"));
	return true;
}

// 이 파일의 변경 이유: 이후 exact-bone 필터가 되살아나거나 pelvis rank가 primary를 빼앗는 회귀를
// 자동으로 검출한다. 정책 테스트와 tracker 테스트를 함께 두어 허용 범위/선택 규칙을 각각 고정한다.
// Assisted의 aim target은 일반 rank(NodeCount > HeadNode > Score)보다 우선하지만, 같은 mesh의
// 다른 본은 Targets에서 secondary dwell 재료로 계속 추적해야 한다. preferred가 사라지면 몸통으로
// dominant가 자동 승계되지 않아야 팔 조준이 pelvis 랩으로 바뀌지 않는다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightTrackerPreferredTargetTest,
	"DynamicRope.FlightContact.TrackerPrefersRequiredAimTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightTrackerPreferredTargetTest::RunTest(const FString& Parameters)
{
	const USceneComponent* Mesh = NewObject<USceneComponent>();
	const FName ArmBone("upperarm_l");
	const FName PelvisBone("pelvis");

	auto MakeCandidate = [Mesh](int32 Node, FName Bone)
	{
		FRopeContactCandidate Candidate;
		Candidate.bValid = true;
		Candidate.NodeIndex = Node;
		Candidate.Bone = Bone;
		Candidate.Mesh = Mesh;
		Candidate.Penetration = 1.0f;
		return Candidate;
	};

	TArray<FRopeContactCandidate> Candidates;
	Candidates.Add(MakeCandidate(14, ArmBone));
	Candidates.Add(MakeCandidate(18, PelvisBone));
	Candidates.Add(MakeCandidate(19, PelvisBone));
	Candidates.Add(MakeCandidate(20, PelvisBone));

	FRopeContactTracker Tracker;
	Tracker.Update(Candidates, 0.10f, Mesh, ArmBone, true);
	TestTrue(TEXT("required aim bone wins over higher node-count pelvis"), Tracker.CandidateBone == ArmBone);
	TestEqual(TEXT("all same-mesh targets remain tracked"), Tracker.Targets.Num(), 2);

	TArray<FRopeContactCandidate> PelvisOnly;
	PelvisOnly.Add(MakeCandidate(18, PelvisBone));
	PelvisOnly.Add(MakeCandidate(19, PelvisBone));
	Tracker.Update(PelvisOnly, 0.10f, Mesh, ArmBone, true);
	TestTrue(TEXT("missing required aim target clears dominant"), Tracker.CandidateBone.IsNone());
	TestTrue(TEXT("pelvis does not inherit dominant"), Tracker.CandidateMesh == nullptr);
	return true;
}

// resolve mode별 aim lock 범위: Assisted는 같은 캐릭터의 다른 본까지 multi-bone 후보로 허용하고,
// Guaranteed는 prepared target의 exact bone만 허용한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimTargetResolvePolicyTest,
	"DynamicRope.FlightContact.AimTargetResolvePolicy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimTargetResolvePolicyTest::RunTest(const FString& Parameters)
{
	const USceneComponent* TargetMesh = NewObject<USceneComponent>();
	const USceneComponent* OtherMesh = NewObject<USceneComponent>();
	const FName PrimaryBone("upperarm_l");
	const FName NeighborBone("clavicle_l");

	FRopeThrowContext ThrowContext;
	ThrowContext.bHasAimGuideHit = true;
	ThrowContext.AimGuideMesh = TargetMesh;
	ThrowContext.AimGuideBone = PrimaryBone;

	FRopeAimTargeting Targeting;
	Targeting.SetWrapTargetLock(ThrowContext);
	TestTrue(TEXT("exact aim bone is primary"), Targeting.IsPrimaryTarget(TargetMesh, PrimaryBone));
	TestTrue(TEXT("neighbor bone on target mesh is allowed in Assisted"),
		Targeting.IsWrapTarget(ERopePhase::Flight, ERopeWrapResolveMode::AssistedJudged,
			TargetMesh, NeighborBone));
	TestFalse(TEXT("same bone on another mesh is rejected in Assisted"),
		Targeting.IsWrapTarget(ERopePhase::Flight, ERopeWrapResolveMode::AssistedJudged,
			OtherMesh, PrimaryBone));
	TestFalse(TEXT("neighbor bone is rejected in Guaranteed"),
		Targeting.IsWrapTarget(ERopePhase::Flight, ERopeWrapResolveMode::GuaranteedWrap,
			TargetMesh, NeighborBone));
	TestTrue(TEXT("exact bone remains allowed in Guaranteed"),
		Targeting.IsWrapTarget(ERopePhase::Flight, ERopeWrapResolveMode::GuaranteedWrap,
			TargetMesh, PrimaryBone));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimRayReachLengthTest,
	"DynamicRope.FlightContact.AimRayReachLengthUsesThrowOrigin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimRayReachLengthTest::RunTest(const FString& Parameters)
{
	const FVector RayDir(1.0f, 0.0f, 0.0f);
	const float RopeReach = 200.0f;

	TestEqual(TEXT("ray at hand uses rope reach"),
		FRopeAimTargeting::ResolveRayLengthForReach(FVector::ZeroVector, RayDir, FVector::ZeroVector, RopeReach),
		200.0f);
	TestEqual(TEXT("ray behind hand extends to hand reach"),
		FRopeAimTargeting::ResolveRayLengthForReach(FVector(-100.0f, 0.0f, 0.0f), RayDir, FVector::ZeroVector, RopeReach),
		300.0f);
	TestTrue(TEXT("lateral offset intersects reach sphere at chord end"),
		FMath::IsNearlyEqual(
			FRopeAimTargeting::ResolveRayLengthForReach(FVector(0.0f, 100.0f, 0.0f), RayDir, FVector::ZeroVector, RopeReach),
			FMath::Sqrt(30000.0f), 0.01f));
	TestEqual(TEXT("ray pointing away from reach sphere has no usable length"),
		FRopeAimTargeting::ResolveRayLengthForReach(FVector(300.0f, 0.0f, 0.0f), RayDir, FVector::ZeroVector, RopeReach),
		0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimRayResolveOutputsTest,
	"DynamicRope.FlightContact.AimRayResolveReturnsSingleSweepOutputs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimRayResolveOutputsTest::RunTest(const FString& Parameters)
{
	const FName Bone("upperarm_l");
	const USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
	RopeTest::FSphereMockCollider Target(FVector(100.0f, 0.0f, 0.0f), 10.0f, Bone, Mesh);
	TArray<IRopeCollider*> Colliders = { &Target };

	FRopeAimTargeting::FQueryContext QueryContext;
	QueryContext.Colliders = &Colliders;
	QueryContext.FallbackRayLength = 200.0f;
	QueryContext.FallbackQueryRadius = 2.0f;

	FRopeAimRayThrowRequest Request;
	Request.BaseContext.Origin = FVector::ZeroVector;
	Request.RayOrigin = FVector::ZeroVector;
	Request.RayDirection = FVector::ForwardVector;
	Request.RayLength = 200.0f;
	Request.ReachOrigin = FVector::ZeroVector;
	Request.ReachLength = 200.0f;
	Request.QueryRadius = 2.0f;
	Request.SweepStep = 1.0f;

	FRopeThrowContext Resolved;
	FRopeAimRayHitResult Hit;
	FRopeAimRayHitResult Blocked;
	const bool bResolved = FRopeAimTargeting::ResolveAimRayThrowContext(
		QueryContext, Request,
		[](const USceneComponent*, FName) { return true; },
		Resolved, &Hit, &Blocked);
	TestTrue(TEXT("wrap 가능 target은 context까지 해석"), bResolved);
	TestTrue(TEXT("같은 sweep의 hit 반환"), Hit.bHit);
	TestFalse(TEXT("wrap 가능 target은 blocked 아님"), Blocked.bHit);
	TestTrue(TEXT("context에 aim guide 설정"), Resolved.bHasAimGuideHit);
	TestTrue(TEXT("context와 hit의 mesh 일치"), Resolved.AimGuideMesh.Get() == Hit.Mesh);
	TestEqual(TEXT("context와 hit의 bone 일치"), Resolved.AimGuideBone, Hit.Bone);

	Resolved = FRopeThrowContext();
	Hit = FRopeAimRayHitResult();
	Blocked = FRopeAimRayHitResult();
	const bool bRejected = FRopeAimTargeting::ResolveAimRayThrowContext(
		QueryContext, Request,
		[](const USceneComponent*, FName) { return false; },
		Resolved, &Hit, &Blocked);
	TestFalse(TEXT("게이트 거부 target은 context fallback"), bRejected);
	TestFalse(TEXT("게이트 거부 target은 valid hit 아님"), Hit.bHit);
	TestTrue(TEXT("게이트 거부 target은 blocked로 반환"), Blocked.bHit);
	TestFalse(TEXT("fallback context에는 aim guide 없음"), Resolved.bHasAimGuideHit);
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
