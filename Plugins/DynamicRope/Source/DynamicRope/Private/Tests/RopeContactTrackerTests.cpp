// Copyright 2026 TeamKeno. All Rights Reserved.
//
// FRopeContactTracker unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Core/RopeContactTrackingTypes.h"
#include "Components/SceneComponent.h"

// Whether the tracker aggregates by the pair of mesh and bone: when two actors sharing a bone name are touched in the
// same frame, their candidates must not be summed into one bucket, the mesh must not be attributed to whichever
// candidate came last, and the dwell has to reset when the mesh changes even though the bone name is the same. This
// is the contract that keeps a cross-actor capture from being attributed to the wrong target.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightTrackerCrossMeshTest,
	"DynamicRope.FlightContact.TrackerSeparatesSameBoneAcrossMeshes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightTrackerCrossMeshTest::RunTest(const FString& Parameters)
{
	// Two components for identity alone, needing no world, since the tracker never dereferences the pointers.
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

	// Two nodes on A and one on B, all under the same bone name. The dominant target has to be the hand on mesh A, and
	// B's candidate must not be summed into it: summing would have given three nodes and overwritten the mesh with B.
	TArray<FRopeContactCandidate> Candidates;
	Candidates.Add(MakeCandidate(5, MeshA));
	Candidates.Add(MakeCandidate(6, MeshA));
	Candidates.Add(MakeCandidate(2, MeshB));

	FRopeContactTracker Tracker;
	Tracker.Update(Candidates, 0.10f);
	TestTrue(TEXT("dominant bone is hand_r"), Tracker.CandidateBone == FName("hand_r"));
	TestTrue(TEXT("dominant mesh is A (no cross-mesh merge)"), Tracker.CandidateMesh == MeshA);
	TestEqual(TEXT("only A's nodes tracked"), Tracker.CandidateNodes.Num(), 2);

	// Keeping the same target accumulates dwell.
	Tracker.Update(Candidates, 0.10f);
	TestTrue(TEXT("dwell accumulates on same (mesh, bone) target"), Tracker.DwellTime > 0.05f);

	// When the dominant target moves to the hand on mesh B, the dwell has to reset even though the bone name is unchanged.
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

// The material for seed multiplexing: whether the tracker keeps contact targets other than the dominant one, with a
// dwell per mesh and bone, in its target list, and whether a target that has stopped contacting decays at the same
// rate and drops off the list once exhausted. The dominant selection and reset contract is unaffected by the target
// list, as the test above pins.
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

	// A reduced two-leg scenario: two bones on one mesh contacting at once, the left thigh at nodes 1 and 2 and the right calf at nodes 6 and 7.
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

	// When the right calf stops contacting, its dwell decays at the same rate, surviving briefly to tolerate a short flicker, and is removed once exhausted.
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

// This detects automatically if an exact-bone filter is reintroduced or if the pelvis outranks the primary target.
// Keeping the policy test and the tracker test together pins the permitted range and the selection rule separately.
// The assisted policy's aim target takes precedence over the ordinary rank, being the node count, then the head node,
// then the score, while other bones on the same mesh have to remain tracked in the target list as secondary dwell
// material. When the preferred target disappears the dominant one must not be automatically inherited by the torso,
// which is what stops an arm being aimed at turning into a pelvis wrap.
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

