// Copyright Epic Games, Inc. All Rights Reserved.
//
// RopeFlightDebug::SelectCandidateBoxes 단위 테스트 — Flight 디버거의 후보 박스 선택 규칙을 월드 없이
// 검증한다. 핵심: 포착 대상(tracker) 대표는 penetration top-N 밖이어도 항상 포함하되 중복은 없고,
// Mesh가 다르면 같은 Bone이어도 대표로 오인하지 않는다. 기본은 대표만, Advanced는 top5로 채운다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeFlightDebugSelection.h"
#include "Components/SceneComponent.h"
#include "UObject/ObjectKey.h"

namespace
{
	FRopeContactCandidate MakeCand(int32 Node, FName Bone, float Penetration,
		ERopeContactCandidateSource Source = ERopeContactCandidateSource::Actual, bool bValid = true)
	{
		FRopeContactCandidate C;
		C.bValid = bValid;
		C.NodeIndex = Node;
		C.Bone = Bone;
		C.Penetration = Penetration;
		C.Source = Source;
		return C;
	}

	bool Contains(const TArray<int32>& Arr, int32 Value)
	{
		return Arr.Contains(Value);
	}
}

// 포착 대상이 penetration 상 꼴찌(top-5 밖)여도 기본/Advanced 모두에 포함된다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectCaptureOutsideTopTest,
	"DynamicRope.FlightDebug.CaptureTargetIncludedEvenIfLowPenetration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectCaptureOutsideTopTest::RunTest(const FString& Parameters)
{
	USceneComponent* MeshA = NewObject<USceneComponent>();
	USceneComponent* MeshB = NewObject<USceneComponent>();
	const FObjectKey KeyA(MeshA), KeyB(MeshB);

	// arm/MeshB 5개(높은 penetration) + spine/MeshA 1개(가장 낮은 penetration = tracker).
	TArray<FRopeContactCandidate> Cands = {
		MakeCand(10, FName("arm"), 50.0f), MakeCand(11, FName("arm"), 40.0f),
		MakeCand(12, FName("arm"), 30.0f), MakeCand(13, FName("arm"), 20.0f),
		MakeCand(14, FName("arm"), 10.0f), MakeCand(20, FName("spine"), 5.0f),
	};
	const TArray<FObjectKey> Keys = { KeyB, KeyB, KeyB, KeyB, KeyB, KeyA };

	// 기본 [U]: 대표 1개만.
	const RopeFlightDebug::FCandidateSelection Base =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 1, false);
	TestEqual(TEXT("base capture idx"), Base.CaptureTargetIndex, 5);
	TestEqual(TEXT("base shown"), Base.Shown, 1);
	TestEqual(TEXT("base hidden"), Base.Hidden, 5);
	TestTrue(TEXT("base box is capture"), Base.BoxIndices.Num() == 1 && Base.BoxIndices[0] == 5);

	// Advanced [U]+[K]: 대표 + top4 = 5, 대표 포함.
	const RopeFlightDebug::FCandidateSelection Adv =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 5, true);
	TestEqual(TEXT("adv shown"), Adv.Shown, 5);
	TestEqual(TEXT("adv hidden"), Adv.Hidden, 1);
	TestTrue(TEXT("adv contains capture"), Contains(Adv.BoxIndices, 5));
	TestEqual(TEXT("adv first is capture"), Adv.BoxIndices[0], 5);
	return true;
}

// 포착 대상이 이미 top-5 안(최고 penetration)이면 대표 슬롯과 일반 채우기에서 중복되지 않는다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectNoDuplicateTest,
	"DynamicRope.FlightDebug.CaptureTargetNotDuplicatedWhenInTop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectNoDuplicateTest::RunTest(const FString& Parameters)
{
	USceneComponent* MeshA = NewObject<USceneComponent>();
	USceneComponent* MeshB = NewObject<USceneComponent>();
	const FObjectKey KeyA(MeshA), KeyB(MeshB);

	// tracker(spine/MeshA)가 최고 penetration.
	TArray<FRopeContactCandidate> Cands = {
		MakeCand(20, FName("spine"), 100.0f), MakeCand(10, FName("arm"), 40.0f),
		MakeCand(11, FName("arm"), 30.0f), MakeCand(12, FName("arm"), 20.0f),
		MakeCand(13, FName("arm"), 10.0f),
	};
	const TArray<FObjectKey> Keys = { KeyA, KeyB, KeyB, KeyB, KeyB };

	const RopeFlightDebug::FCandidateSelection Adv =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 5, true);
	TestEqual(TEXT("capture idx"), Adv.CaptureTargetIndex, 0);
	TestEqual(TEXT("shown 5"), Adv.Shown, 5);
	// 인덱스 0이 정확히 한 번만 등장.
	int32 CountZero = 0;
	for (int32 i : Adv.BoxIndices) { if (i == 0) { ++CountZero; } }
	TestEqual(TEXT("capture appears once"), CountZero, 1);
	// 전 인덱스 유일(중복 없음).
	TSet<int32> Unique(Adv.BoxIndices);
	TestEqual(TEXT("all indices unique"), Unique.Num(), Adv.BoxIndices.Num());
	return true;
}

// Bone 이름이 같아도 Mesh가 다르면 포착 대상으로 오인하지 않는다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectMeshDistinctTest,
	"DynamicRope.FlightDebug.SameBoneDifferentMeshIsNotCaptureTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectMeshDistinctTest::RunTest(const FString& Parameters)
{
	USceneComponent* MeshA = NewObject<USceneComponent>();
	USceneComponent* MeshB = NewObject<USceneComponent>();
	const FObjectKey KeyA(MeshA), KeyB(MeshB);

	// tracker = (spine, MeshA)지만 후보의 spine은 MeshB다.
	TArray<FRopeContactCandidate> Cands = { MakeCand(20, FName("spine"), 100.0f) };
	const TArray<FObjectKey> Keys = { KeyB };

	const RopeFlightDebug::FCandidateSelection Base =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 1, false);
	TestEqual(TEXT("no capture (mesh differs)"), Base.CaptureTargetIndex, (int32)INDEX_NONE);
	TestEqual(TEXT("base shows nothing"), Base.Shown, 0);

	// Advanced는 대표가 없으니 일반 후보로 채운다(1개뿐).
	const RopeFlightDebug::FCandidateSelection Adv =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 5, true);
	TestEqual(TEXT("adv no capture"), Adv.CaptureTargetIndex, (int32)INDEX_NONE);
	TestEqual(TEXT("adv fills general"), Adv.Shown, 1);
	return true;
}

// 포착 대상이 없으면 기본은 0개, Advanced는 top5.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectNoTrackerTest,
	"DynamicRope.FlightDebug.NoTrackerBaseZeroAdvancedTop5",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectNoTrackerTest::RunTest(const FString& Parameters)
{
	// tracker 없음(TrackerBone = None). 유효 6개 + 무효 1개.
	TArray<FRopeContactCandidate> Cands = {
		MakeCand(10, FName("arm"), 60.0f), MakeCand(11, FName("arm"), 50.0f),
		MakeCand(12, FName("arm"), 40.0f), MakeCand(13, FName("arm"), 30.0f),
		MakeCand(14, FName("arm"), 20.0f), MakeCand(15, FName("arm"), 10.0f),
		MakeCand(16, FName("arm"), 5.0f, ERopeContactCandidateSource::Actual, /*bValid=*/false),
	};
	const TArray<FObjectKey> Keys; // 비움 — 이 테스트는 mesh 무관.

	const RopeFlightDebug::FCandidateSelection Base =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, NAME_None, FObjectKey(), 1, false);
	TestEqual(TEXT("base total valid"), Base.TotalValid, 6);
	TestEqual(TEXT("base shown zero"), Base.Shown, 0);
	TestEqual(TEXT("base hidden all"), Base.Hidden, 6);

	const RopeFlightDebug::FCandidateSelection Adv =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, NAME_None, FObjectKey(), 5, true);
	TestEqual(TEXT("adv shown 5"), Adv.Shown, 5);
	TestEqual(TEXT("adv hidden 1"), Adv.Hidden, 1);
	// 무효 후보(index 6)는 그려지지 않는다.
	TestFalse(TEXT("invalid not drawn"), Contains(Adv.BoxIndices, 6));
	// penetration 상위 5개(60/50/40/30/20 = index 0..4)만.
	TestEqual(TEXT("top by penetration"), Adv.BoxIndices[0], 0);
	return true;
}

// 동률 penetration이면 NodeIndex 오름차순으로 결정적.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectTieBreakTest,
	"DynamicRope.FlightDebug.TieBreakByNodeIndexDeterministic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectTieBreakTest::RunTest(const FString& Parameters)
{
	// 같은 penetration, NodeIndex만 다름 — 낮은 NodeIndex가 먼저.
	TArray<FRopeContactCandidate> Cands = {
		MakeCand(30, FName("arm"), 50.0f), MakeCand(10, FName("arm"), 50.0f),
		MakeCand(20, FName("arm"), 50.0f),
	};
	const TArray<FObjectKey> Keys;

	const RopeFlightDebug::FCandidateSelection Adv =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, NAME_None, FObjectKey(), 5, true);
	// NodeIndex 10(idx1) → 20(idx2) → 30(idx0).
	TestEqual(TEXT("order 0"), Adv.BoxIndices[0], 1);
	TestEqual(TEXT("order 1"), Adv.BoxIndices[1], 2);
	TestEqual(TEXT("order 2"), Adv.BoxIndices[2], 0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
