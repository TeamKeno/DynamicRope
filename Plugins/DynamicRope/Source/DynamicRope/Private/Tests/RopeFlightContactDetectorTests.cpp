// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Unit tests for FRopeFlightContactDetector. They verify the flight contact pipeline, meaning the collection of
// actual and predicted contacts, the relative motion evaluation and the capture decision, with no world, using a POD
// fixture and a mock collider.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeFlightContactDetector.h"
#include "Collision/RopeCollider.h"
#include "Collision/RopeStaticCollider.h"
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

// Whether the nodes of a stationary rope inside a collider's radius are collected as actual candidates.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightDetectActualTest,
	"DynamicRope.FlightContact.DetectsActualContactAlongPath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightDetectActualTest::RunTest(const FString& Parameters)
{
	// The nodes are at x = 0, 20 up to 140. An arm centred at 60 with radius 25 plus a contact radius of 3 gives a reach of 28, so the nodes at 40, 60 and 80 contact, giving three.
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

// Whether a fast node passes through a thin target between samples, meaning tunnelling.
// The previous sampling took its spacing from the segment length of 20 cm and cut it into four, giving a spacing of
// 25 cm over a movement of 100 cm; a forearm or railing a few centimetres thick simply passed between them, and
// because the CPU and the GPU were wrong in the same way the parity test passed as well.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightThinColliderTunnelTest,
	"DynamicRope.FlightContact.FastNodeDoesNotTunnelThinCollider",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightThinColliderTunnelTest::RunTest(const FString& Parameters)
{
	const FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);

	// A movement of 100 cm in one frame, from x = 0 to 100. A target 3 cm thick sits at x = 60 and the query radius is
	// 1, giving a reach of 4 cm. At the old spacing of 25 cm the samples were at 50 and 75, both outside that reach, and it was missed.
	const FVector Prev(0.0f, 0.0f, 0.0f);
	const FVector Curr(100.0f, 0.0f, 0.0f);
	RopeTest::FSphereMockCollider Thin(FVector(60.0f, 0.0f, 0.0f), 3.0f, FName("forearm"));
	TArray<IRopeCollider*> Colliders = { &Thin };

	FRopeFlightContactDetector::FParams Params = MakeDetectParams(/*MinLatchNodes*/ 1,
		/*PredictiveFrames*/ 0.0f, /*ContactRadius*/ 1.0f);
	const FRopeContact Hit = FRopeFlightContactDetector::SweepOrSampleContact(Sim, Prev, Curr, Colliders, Params);
	TestTrue(TEXT("thin collider is found along a fast path"), Hit.bHit);
	TestTrue(TEXT("contact is attributed to the thin bone"), Hit.Bone == FName("forearm"));

	// Lowering the cap to four returns the spacing to 25 cm and it is missed again, which pins that this test is watching the spacing itself.
	Params.ContactMaxSweepSamples = 4;
	const FRopeContact Missed = FRopeFlightContactDetector::SweepOrSampleContact(Sim, Prev, Curr, Colliders, Params);
	TestFalse(TEXT("a 4-sample cap tunnels through it again"), Missed.bHit);
	return true;
}

// A thick target must latch on its entry surface instead of a later internal/far-side sample.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightThickConvexEntryContactTest,
	"DynamicRope.FlightContact.ThickConvexKeepsEarliestEntrySurface",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightThickConvexEntryContactTest::RunTest(const FString& Parameters)
{
	const FRopeSimState Sim = RopeTest::MakeStraightRope(2, 20.0f);
	TArray<FPlane> Planes;
	Planes.Add(FPlane(FVector(1, 0, 0), 30.0));
	Planes.Add(FPlane(FVector(-1, 0, 0), 30.0));
	Planes.Add(FPlane(FVector(0, 1, 0), 30.0));
	Planes.Add(FPlane(FVector(0, -1, 0), 30.0));
	Planes.Add(FPlane(FVector(0, 0, 1), 30.0));
	Planes.Add(FPlane(FVector(0, 0, -1), 30.0));
	FRopeConvexCollider ThickConvex(MoveTemp(Planes), FBox(FVector(-30.0), FVector(30.0)));
	ThickConvex.Bone = FName(TEXT("log"));
	TArray<IRopeCollider*> Colliders = { &ThickConvex };

	FRopeFlightContactDetector::FParams Params = MakeDetectParams(/*MinLatchNodes*/ 1,
		/*PredictiveFrames*/ 0.0f, /*ContactRadius*/ 3.0f);
	Params.ContactSweepStep = 2.0f;
	Params.ContactMaxSweepSamples = 16;
	const FRopeContact Hit = FRopeFlightContactDetector::SweepOrSampleContact(
		Sim, FVector(-100.0f, 0.0f, 0.0f), FVector(100.0f, 0.0f, 0.0f), Colliders, Params);

	TestTrue(TEXT("the thick convex is detected"), Hit.bHit);
	TestTrue(TEXT("the contact normal points back through the entry face"), Hit.Normal.X < -0.99f);
	TestTrue(TEXT("the contact point stays on the negative-X entry plane"),
		FMath::IsNearlyEqual(Hit.SurfacePoint.X, -30.0f, 0.1f));
	return true;
}

// Whether a capture is decided only once the dominant bone's contact node count passes the minimum latch node threshold.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightShouldCaptureTest,
	"DynamicRope.FlightContact.ShouldCaptureRespectsMinLatchNodes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightShouldCaptureTest::RunTest(const FString& Parameters)
{
	// The same fixture as the test above, giving three contacting nodes.
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

// Whether the assisted policy picks the aimed bone as dominant while preserving other bones on the same mesh as secondary targets.
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

// Whether a tail node that has not yet touched anything, but whose extrapolated path along its movement direction passes through a collider, is promoted to a free predictive candidate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightPredictFreeTest,
	"DynamicRope.FlightContact.PredictsFreeNodeContactAhead",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightPredictFreeTest::RunTest(const FString& Parameters)
{
	// The tip, node 7 at x = 140, is moving along positive X at 20 per frame. An arm centred at 180 with radius 10 plus
	// 3 gives a reach of 13, so the current position at 140 is outside contact but the three-frame extrapolation from 140 to 200 passes through the sphere.
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

// Whether a guided predictive candidate comes out of the extrapolated path from a whip guide node's current target to its next one.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightPredictGuidedTest,
	"DynamicRope.FlightContact.PredictsGuidedNodeContactFromTargets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightPredictGuidedTest::RunTest(const FString& Parameters)
{
	// Node 2 alone is guided. Its current target is (50,0,50) and its next is (50,0,25), so a two-frame extrapolation
	// reaches (50,0,0). An arm centred at (50,0,0) with radius 5 plus 3 gives a reach of 8, so the end of the extrapolated path strikes the sphere's centre.
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
	// 400 cm at the default cap of 16 gives a spacing of 25 cm. A target at x = 12.5 with a reach of 4 cm falls
	// entirely between two samples, but the reliable guided budget has to keep its default step of 2 cm and catch it.
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
	TestTrue(TEXT("the regular time-ordered pass keeps the first contacting bone"),
		Candidates.Num() == 1 && Candidates[0].Bone == PrimaryBone);

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

// The wrap direction score: positive when sliding towards the hand, negative when sliding away, and zero when moving with the surface, meaning a relative velocity of zero.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeFlightWrapDirectionScoreTest,
	"DynamicRope.FlightContact.WrapDirectionScoreSign",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeFlightWrapDirectionScoreTest::RunTest(const FString& Parameters)
{
	// The hand at node zero is at the origin and the contacting node 1 is at (100,0,0). The surface normal is positive Z, so the tangent plane is XY.
	FRopeSimState Sim = RopeTest::MakeStraightRope(2, 100.0f);
	const FRopeFlightContactDetector::FParams Params = MakeDetectParams();

	FRopeContactCandidate Candidate;
	Candidate.bValid = true;
	Candidate.NodeIndex = 1;
	Candidate.Bone = FName("arm");
	Candidate.WorldPoint = FVector(100.0f, 0.0f, 0.0f);
	Candidate.Normal = FVector::UpVector;

	// Moving 5 towards the hand along negative X aligns with the expected wrap tangent, giving a score of plus one.
	Sim.PrevPositions[1] = FVector(105.0f, 0.0f, 0.0f);
	TArray<FRopeContactCandidate> Candidates = { Candidate };
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, Params, Candidates);
	TestTrue(TEXT("toward-hand slide scores positive"), Candidates[0].WrapDirectionScore > 0.9f);
	TestTrue(TEXT("tangential speed matches displacement"),
		FMath::IsNearlyEqual(Candidates[0].RelativeTangentialSpeed, 5.0f, 0.01f));

	// Moving the other way, along positive X, gives minus one.
	Sim.PrevPositions[1] = FVector(95.0f, 0.0f, 0.0f);
	Candidates = { Candidate };
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, Params, Candidates);
	TestTrue(TEXT("away-from-hand slide scores negative"), Candidates[0].WrapDirectionScore < -0.9f);

	// A surface moving at the same velocity as the rope, giving a relative velocity of zero, means no sliding.
	// The surface velocity is contractually in centimetres per second, so the substep delta is stated and a value giving the same speed as a displacement of minus 5 cm is used.
	FRopeFlightContactDetector::FParams DtParams = Params;
	DtParams.SubstepDeltaTime = 0.02f;
	Sim.PrevPositions[1] = FVector(105.0f, 0.0f, 0.0f);
	// Minus 250 cm/s over 0.02 s is minus 5 cm, which is this displacement over the substep.
	Candidate.SurfaceVelocity = FVector(-250.0f, 0.0f, 0.0f);
	Candidates = { Candidate };
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DtParams, Candidates);
	TestTrue(TEXT("co-moving surface yields no tangential slide"),
		Candidates[0].RelativeTangentialSpeed < 0.01f);

	return true;
}

// The travel frame snapshot taken at the moment of capture, FRopeCaptureTravelFrame::Compute: it derives the travel
// plane normal, the cross product of velocity and span, from the contact region's centre, its mean velocity and the
// direction the rope lies along, and when the velocity and the span are parallel, as in a spear throw, or the delta
// is zero, it leaves a fallback signal with no normal, meaning the plane normal flag is false.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCaptureTravelFrameTest,
	"DynamicRope.FlightContact.CaptureTravelFrameSnapshot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCaptureTravelFrameTest::RunTest(const FString& Parameters)
{
	// The rope lies along Y with a node spacing of 20 and the whole of it is flying along positive X at 10 cm per frame, which at a delta of 0.1 is 100 cm/s.
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

	// Degenerate case one, the spear throw: a rope flying straight along the direction it lies in, positive Y, admits no normal.
	for (int32 Index = 0; Index < Sim.Num(); ++Index)
	{
		Sim.PrevPositions[Index] = Sim.Positions[Index] - FVector(0.0f, 10.0f, 0.0f);
	}
	const FRopeCaptureTravelFrame Javelin = FRopeCaptureTravelFrame::Compute(Sim, Candidates, 0.1f);
	TestTrue(TEXT("javelin frame still captured"), Javelin.bValid);
	TestTrue(TEXT("javelin flight yields no plane normal"), !Javelin.bHasPlaneNormal);

	// Degenerate case two, a delta of zero: the velocity cannot be converted. The snapshot stays valid but has no normal.
	const FRopeCaptureTravelFrame NoDt = FRopeCaptureTravelFrame::Compute(Sim, Candidates, 0.0f);
	TestTrue(TEXT("zero-dt frame still captured"), NoDt.bValid);
	TestTrue(TEXT("zero-dt velocity is zero"), NoDt.AverageVelocity.IsNearlyZero());
	TestTrue(TEXT("zero-dt yields no plane normal"), !NoDt.bHasPlaneNormal);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

