// Copyright Epic Games, Inc. All Rights Reserved.
//
// Unit tests for RopeFlightDebug::SelectCandidateBoxes, verifying the flight debugger's candidate box selection rules
// with no world. The essentials: the representative of the tracked target is always included even when it falls
// outside the top N by penetration, never duplicated, and a matching bone name on a different mesh is not mistaken
// for the representative. The default draws the representative alone, while the advanced view fills up to five.

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

// A tracked target last by penetration, meaning outside the top five, is included in both the default and the advanced view.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectCaptureOutsideTopTest,
	"DynamicRope.FlightDebug.CaptureTargetIncludedEvenIfLowPenetration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectCaptureOutsideTopTest::RunTest(const FString& Parameters)
{
	USceneComponent* MeshA = NewObject<USceneComponent>();
	USceneComponent* MeshB = NewObject<USceneComponent>();
	const FObjectKey KeyA(MeshA), KeyB(MeshB);

	// Five arm candidates on mesh B with high penetration, plus one spine candidate on mesh A with the lowest penetration, which is the tracked one.
	TArray<FRopeContactCandidate> Cands = {
		MakeCand(10, FName("arm"), 50.0f), MakeCand(11, FName("arm"), 40.0f),
		MakeCand(12, FName("arm"), 30.0f), MakeCand(13, FName("arm"), 20.0f),
		MakeCand(14, FName("arm"), 10.0f), MakeCand(20, FName("spine"), 5.0f),
	};
	const TArray<FObjectKey> Keys = { KeyB, KeyB, KeyB, KeyB, KeyB, KeyA };

	// The default view: the representative alone.
	const RopeFlightDebug::FCandidateSelection Base =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 1, false);
	TestEqual(TEXT("base capture idx"), Base.CaptureTargetIndex, 5);
	TestEqual(TEXT("base shown"), Base.Shown, 1);
	TestEqual(TEXT("base hidden"), Base.Hidden, 5);
	TestTrue(TEXT("base box is capture"), Base.BoxIndices.Num() == 1 && Base.BoxIndices[0] == 5);

	// The advanced view: the representative plus the top four, giving five, with the representative included.
	const RopeFlightDebug::FCandidateSelection Adv =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 5, true);
	TestEqual(TEXT("adv shown"), Adv.Shown, 5);
	TestEqual(TEXT("adv hidden"), Adv.Hidden, 1);
	TestTrue(TEXT("adv contains capture"), Contains(Adv.BoxIndices, 5));
	TestEqual(TEXT("adv first is capture"), Adv.BoxIndices[0], 5);
	return true;
}

// A tracked target already inside the top five, with the highest penetration, is not duplicated between the representative slot and the ordinary fill.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectNoDuplicateTest,
	"DynamicRope.FlightDebug.CaptureTargetNotDuplicatedWhenInTop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectNoDuplicateTest::RunTest(const FString& Parameters)
{
	USceneComponent* MeshA = NewObject<USceneComponent>();
	USceneComponent* MeshB = NewObject<USceneComponent>();
	const FObjectKey KeyA(MeshA), KeyB(MeshB);

	// The tracked spine on mesh A has the highest penetration.
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
	// Index zero appears exactly once.
	int32 CountZero = 0;
	for (int32 i : Adv.BoxIndices) { if (i == 0) { ++CountZero; } }
	TestEqual(TEXT("capture appears once"), CountZero, 1);
	// Every index is unique, with no duplicates.
	TSet<int32> Unique(Adv.BoxIndices);
	TestEqual(TEXT("all indices unique"), Unique.Num(), Adv.BoxIndices.Num());
	return true;
}

// A matching bone name on a different mesh is not mistaken for the tracked target.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectMeshDistinctTest,
	"DynamicRope.FlightDebug.SameBoneDifferentMeshIsNotCaptureTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectMeshDistinctTest::RunTest(const FString& Parameters)
{
	USceneComponent* MeshA = NewObject<USceneComponent>();
	USceneComponent* MeshB = NewObject<USceneComponent>();
	const FObjectKey KeyA(MeshA), KeyB(MeshB);

	// The tracked target is the spine on mesh A, but the candidate's spine is on mesh B.
	TArray<FRopeContactCandidate> Cands = { MakeCand(20, FName("spine"), 100.0f) };
	const TArray<FObjectKey> Keys = { KeyB };

	const RopeFlightDebug::FCandidateSelection Base =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 1, false);
	TestEqual(TEXT("no capture (mesh differs)"), Base.CaptureTargetIndex, (int32)INDEX_NONE);
	TestEqual(TEXT("base shows nothing"), Base.Shown, 0);

	// With no representative, the advanced view fills from the ordinary candidates, of which there is one.
	const RopeFlightDebug::FCandidateSelection Adv =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, FName("spine"), KeyA, 5, true);
	TestEqual(TEXT("adv no capture"), Adv.CaptureTargetIndex, (int32)INDEX_NONE);
	TestEqual(TEXT("adv fills general"), Adv.Shown, 1);
	return true;
}

// With no tracked target the default view draws none and the advanced view draws the top five.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectNoTrackerTest,
	"DynamicRope.FlightDebug.NoTrackerBaseZeroAdvancedTop5",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectNoTrackerTest::RunTest(const FString& Parameters)
{
	// No tracked target, meaning the tracker bone is none. Six valid candidates plus one invalid.
	TArray<FRopeContactCandidate> Cands = {
		MakeCand(10, FName("arm"), 60.0f), MakeCand(11, FName("arm"), 50.0f),
		MakeCand(12, FName("arm"), 40.0f), MakeCand(13, FName("arm"), 30.0f),
		MakeCand(14, FName("arm"), 20.0f), MakeCand(15, FName("arm"), 10.0f),
		MakeCand(16, FName("arm"), 5.0f, ERopeContactCandidateSource::Actual, /*bValid=*/false),
	};
	const TArray<FObjectKey> Keys; // Empty, since this test does not depend on the mesh.

	const RopeFlightDebug::FCandidateSelection Base =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, NAME_None, FObjectKey(), 1, false);
	TestEqual(TEXT("base total valid"), Base.TotalValid, 6);
	TestEqual(TEXT("base shown zero"), Base.Shown, 0);
	TestEqual(TEXT("base hidden all"), Base.Hidden, 6);

	const RopeFlightDebug::FCandidateSelection Adv =
		RopeFlightDebug::SelectCandidateBoxes(Cands, Keys, NAME_None, FObjectKey(), 5, true);
	TestEqual(TEXT("adv shown 5"), Adv.Shown, 5);
	TestEqual(TEXT("adv hidden 1"), Adv.Hidden, 1);
	// The invalid candidate at index six is not drawn.
	TestFalse(TEXT("invalid not drawn"), Contains(Adv.BoxIndices, 6));
	// The top five by penetration alone, being 60, 50, 40, 30 and 20 at indices zero to four.
	TestEqual(TEXT("top by penetration"), Adv.BoxIndices[0], 0);
	return true;
}

// Equal penetrations are ordered deterministically by ascending node index.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightSelectTieBreakTest,
	"DynamicRope.FlightDebug.TieBreakByNodeIndexDeterministic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightSelectTieBreakTest::RunTest(const FString& Parameters)
{
	// The same penetration with different node indices, so the lower node index comes first.
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
