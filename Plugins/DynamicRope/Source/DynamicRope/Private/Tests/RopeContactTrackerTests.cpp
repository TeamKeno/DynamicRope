// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeContactTracker unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Core/RopeContactTrackingTypes.h"
#include "Components/SceneComponent.h"

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

#endif // WITH_DEV_AUTOMATION_TESTS

