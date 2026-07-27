// Copyright Epic Games, Inc. All Rights Reserved.
//
// Regression tests for URopeComponent's orchestration from Contacting into Wrapping.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeComponent.h"
#include "Collision/RopeCollider.h"
#include "Components/SceneComponent.h"
#include "RopeTestHelpers.h"

struct FRopeWrappingFallbackTestSeam
{
	static void ConfigureContactingSeed(URopeComponent& Rope, const USceneComponent* Mesh,
		IRopeCollider& Collider, const TArray<FRopeContactCandidate>& Candidates)
	{
		Rope.Sim = RopeTest::MakeStraightRope(
			9, 160.0f, FVector(25.0f, 0.0f, 0.0f), FVector(0.0f, 1.0f, 0.0f));
		Rope.Radius = 1.0f;
		Rope.WrapConfig.ContactQueryRadius = 3.0f;
		Rope.ContactTracker.CandidateMesh = Mesh;
		Rope.ContactTracker.CandidateBone = FName("arm");
		Rope.ContactTracker.CandidateNodes = { 0 };
		Rope.ContactTracker.DwellTime = Rope.WrapConfig.WrapDecisionTime;
		Rope.SimFrame.FrameColliders = { &Collider };

		// Pins BoneCenteredGuidePlane to the capsule's Z axis so the test does not depend on the tie-breaking order of
		// the component axis fallback.
		Rope.bHasFlightGuidePlaneNormal = true;
		Rope.FlightGuidePlaneNormal = FVector::ZAxisVector;
		Rope.PendingWrapSeed = Rope.BuildWrapSeedFromContactingState(Candidates);
		Rope.Phase = ERopePhase::Contacting;
	}

	static const FRopeWrapState& GetPendingSeed(const URopeComponent& Rope)
	{
		return Rope.PendingWrapSeed;
	}

	static void RemoveContactAnchorAndStartWrapping(URopeComponent& Rope)
	{
		Rope.PendingWrapSeed.Anchors.Reset();
		Rope.StartWrappingFromContacting();
	}

	static const FRopeWrappingState& GetWrappingState(const URopeComponent& Rope)
	{
		return Rope.WrappingPhase.State;
	}

	static void ConfigureAssistedContacting(URopeComponent& Rope, USkeletalMeshComponent* Mesh,
		FName PrimaryBone, IRopeCollider& Primary, IRopeCollider& DeeperNeighbor)
	{
		Rope.Sim = RopeTest::MakeStraightRope(4, 60.0f);
		Rope.Radius = 1.0f;
		Rope.WrapConfig.ContactQueryRadius = 3.0f;
		Rope.WrapConfig.WrapDecisionTime = 0.05f;
		Rope.ResolveMode = ERopeWrapResolveMode::AssistedJudged;
		Rope.Phase = ERopePhase::Contacting;

		FRopeThrowContext Context;
		Context.bHasAimGuideHit = true;
		Context.AimGuideMesh = Mesh;
		Context.AimGuideBone = PrimaryBone;
		Rope.AimTargeting.SetWrapTargetLock(Context);
		Rope.SimFrame.FrameColliders = { &Primary, &DeeperNeighbor };

		// Already captured on an exact primary during Flight. This verifies that on the next contacting tick at 120 Hz
		// a deeper neighbour on the same mesh does not hide the primary and cause it to be dismissed.
		Rope.ContactTracker.CandidateMesh = Mesh;
		Rope.ContactTracker.CandidateBone = PrimaryBone;
		Rope.ContactTracker.CandidateNodes = { 1 };
		Rope.ContactTracker.DwellTime = 1.0f / 120.0f;
	}

	static void UpdateContacting(URopeComponent& Rope, float DeltaTime)
	{
		Rope.UpdateContacting(DeltaTime);
	}

	static const FRopeContactTracker& GetContactTracker(const URopeComponent& Rope)
	{
		return Rope.ContactTracker;
	}

	static FRopeSurfaceAnchor ConfigureExternalAnchor(
		URopeComponent& Rope, const USceneComponent* Mesh, ERopePhase Phase,
		int32 AnchorNode = 3)
	{
		Rope.Sim = RopeTest::MakeStraightRope(4, 60.0f);
		Rope.Sim.bStartPinned = true;
		Rope.Sim.StartPinPrev = Rope.Sim.Positions[0];
		Rope.Sim.StartPinTarget = Rope.Sim.Positions[0];
		Rope.Sim.InvMass[0] = 0.0f;
		Rope.HoldConfig.bEnforceWielderLengthConstraint = true;
		Rope.Phase = Phase;

		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = AnchorNode;
		Anchor.Mesh = Mesh;
		Anchor.Bone = FName("root");
		Anchor.LocalSurfacePosition = FVector(60.0f, 0.0f, 0.0f);
		Anchor.LocalNormal = FVector::UpVector;
		Anchor.SurfaceOffset = 0.0f;

		if (Phase == ERopePhase::Wrapping)
		{
			Rope.WrappingPhase.State.Mesh = Mesh;
			Rope.WrappingPhase.State.BoneName = Anchor.Bone;
			Rope.WrappingPhase.State.Anchors = { Anchor };
			Rope.WrappingPhase.State.LatchAnchor = Anchor;
		}
		else if (Phase == ERopePhase::Wrapped)
		{
			Rope.WrapController.State.Mesh = Mesh;
			Rope.WrapController.State.Anchors = { Anchor };
			Rope.bWrappedMassMaskDirty = true;
		}
		return Anchor;
	}

	static void CommitWrapping(URopeComponent& Rope)
	{
		Rope.CommitWrapping();
	}

	static void Prepare(URopeComponent& Rope, float DeltaTime)
	{
		Rope.PrepareSimFrame(DeltaTime, TOptional<FVector>());
	}

	static const FRopePullSample& GetPullSample(const URopeComponent& Rope)
	{
		return Rope.PullDrive.LastPullSample;
	}

	static void Reel(URopeComponent& Rope, float DeltaTime)
	{
		Rope.UpdateReel(DeltaTime);
	}
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSyntheticLatchAnchorFallbackTest,
	"DynamicRope.Component.Wrapping.SyntheticLatchAnchorFallback",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSyntheticLatchAnchorFallbackTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = NewObject<USceneComponent>();
	FCapsuleCollider Capsule(
		FVector(0.0f, 0.0f, -50.0f), FVector(0.0f, 0.0f, 50.0f),
		25.0f, FName("arm"), Mesh);

	FRopeContactCandidate Candidate;
	Candidate.bValid = true;
	Candidate.NodeIndex = 0;
	Candidate.Bone = FName("arm");
	Candidate.Mesh = Mesh;
	Candidate.WorldPoint = FVector(25.0f, 0.0f, 0.0f);
	Candidate.Normal = FVector::XAxisVector;
	Candidate.Penetration = 1.0f;
	const TArray<FRopeContactCandidate> Candidates = { Candidate };

	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeWrappingFallbackTestSeam::ConfigureContactingSeed(*Rope, Mesh, Capsule, Candidates);

	// A normal contacting seed assembly must produce an anchor based on the real contact point, normal and tangent.
	const FRopeWrapState& NormalSeed = FRopeWrappingFallbackTestSeam::GetPendingSeed(*Rope);
	TestEqual(TEXT("valid Contacting candidate creates one latch"), NormalSeed.Latched.Num(), 1);
	TestEqual(TEXT("valid Contacting candidate creates one surface anchor"), NormalSeed.Anchors.Num(), 1);
	if (NormalSeed.Anchors.Num() == 1)
	{
		TestTrue(TEXT("normal seed preserves the contact surface normal"),
			NormalSeed.Anchors[0].LocalNormal.Equals(FVector::XAxisVector, KINDA_SMALL_NUMBER));
	}

	// The problem state is reproduced by removing the anchor alone from a normal assembly. A warning together with a
	// synthetic anchor being created and wrapping starting confirms the fallback branch is genuinely reachable code.
	AddExpectedError(TEXT("StartWrapping fell back to the synthetic latch anchor"),
		EAutomationExpectedErrorFlags::Contains, 1);
	FRopeWrappingFallbackTestSeam::RemoveContactAnchorAndStartWrapping(*Rope);

	TestTrue(TEXT("anchor-less seed enters Wrapping through the fallback"),
		Rope->GetPhase() == ERopePhase::Wrapping);
	const FRopeWrappingState& WrappingState =
		FRopeWrappingFallbackTestSeam::GetWrappingState(*Rope);
	TestEqual(TEXT("fallback installs one wrapping anchor"), WrappingState.Anchors.Num(), 1);
	TestTrue(TEXT("fallback uses the documented synthetic UpVector normal"),
		WrappingState.LatchAnchor.LocalNormal.Equals(FVector::UpVector, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("fallback starts from the current latch node position"),
		WrappingState.LatchAnchor.StartWorldPosition.Equals(FVector(25.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAssistedContactingPrimaryShadowTest,
	"DynamicRope.Component.Contacting.AssistedPrimarySurvivesDeeperNeighborAtHighFPS",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAssistedContactingPrimaryShadowTest::RunTest(const FString& Parameters)
{
	USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
	const FName PrimaryBone("upperarm_l");
	RopeTest::FSphereMockCollider Primary(FVector(20.0f, 0.0f, 0.0f), 8.0f, PrimaryBone, Mesh);
	RopeTest::FSphereMockCollider DeeperNeighbor(
		FVector(20.0f, 0.0f, 0.0f), 15.0f, FName("clavicle_l"), Mesh);

	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeWrappingFallbackTestSeam::ConfigureAssistedContacting(
		*Rope, Mesh, PrimaryBone, Primary, DeeperNeighbor);
	const float InitialDwell = FRopeWrappingFallbackTestSeam::GetContactTracker(*Rope).DwellTime;

	FRopeWrappingFallbackTestSeam::UpdateContacting(*Rope, 1.0f / 120.0f);
	const FRopeContactTracker& Tracker = FRopeWrappingFallbackTestSeam::GetContactTracker(*Rope);

	TestEqual(TEXT("exact primary contact remains in Contacting"), Rope->GetPhase(), ERopePhase::Contacting);
	TestEqual(TEXT("deeper same-mesh neighbor does not replace the aimed bone"),
		Tracker.CandidateBone, PrimaryBone);
	TestTrue(TEXT("the aimed mesh identity is preserved"), Tracker.CandidateMesh == Mesh);
	TestTrue(TEXT("the exact primary node remains tracked"), Tracker.CandidateNodes.Contains(1));
	TestTrue(TEXT("primary dwell advances at 120Hz instead of dismissing"), Tracker.DwellTime > InitialDwell);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCurrentFrameWrappedGeometryTest,
	"DynamicRope.Component.Wrapping.CurrentFramePinAndAnchorGeometry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCurrentFrameWrappedGeometryTest::RunTest(const FString& Parameters)
{
	USceneComponent* Target = NewObject<USceneComponent>();
	Target->SetWorldLocation(FVector(20.0f, 0.0f, 0.0f));

	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeWrappingFallbackTestSeam::ConfigureExternalAnchor(
		*Rope, Target, ERopePhase::Wrapped);
	Rope->SetWorldLocation(FVector(-10.0f, 0.0f, 0.0f));

	FRopeWrappingFallbackTestSeam::Prepare(*Rope, 1.0f / 60.0f);
	const FRopePullSample& Pull = FRopeWrappingFallbackTestSeam::GetPullSample(*Rope);

	TestTrue(TEXT("pull sample is valid"), Pull.bValid);
	TestTrue(TEXT("pull sees the current target binding, not the prior Sim anchor"),
		Pull.WorldPoint.Equals(FVector(80.0f, 0.0f, 0.0f), 0.01f));
	TestTrue(TEXT("pull sees the current start pin in the same Prepare"),
		FMath::IsNearlyEqual(Pull.PathChordLen, 90.0f, 0.01f));
	TestTrue(TEXT("material free-rest remains unchanged"),
		FMath::IsNearlyEqual(Pull.FreeRestLen, 60.0f, 0.01f));
	TestTrue(TEXT("geometry taut does not require delayed XPBD SegmentTension"),
		Rope->IsChainTaut());
	TestTrue(TEXT("default pull taut is geometry-only and can start its own load"),
		Rope->IsPullTaut());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingHardLeashContinuityTest,
	"DynamicRope.Movement.HardLeash.WrappingCommitContinuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingHardLeashContinuityTest::RunTest(const FString& Parameters)
{
	USceneComponent* Target = NewObject<USceneComponent>();
	Target->SetWorldLocation(FVector(20.0f, 0.0f, 0.0f));
	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeWrappingFallbackTestSeam::ConfigureExternalAnchor(
		*Rope, Target, ERopePhase::Wrapping);

	FRopeWielderMovementConstraint WrappingConstraint;
	TestTrue(TEXT("hard leash arms during Wrapping before commit"),
		Rope->BuildWielderMovementConstraint(WrappingConstraint));
	TestTrue(TEXT("live binding is used during Wrapping"),
		WrappingConstraint.PivotWorld.Equals(FVector(80.0f, 0.0f, 0.0f), 0.01f));
	TestTrue(TEXT("hard leash uses the material length"),
		FMath::IsNearlyEqual(WrappingConstraint.MaxDistance, 60.0f, 0.01f));

	URopeComponent* NodeZeroRope = NewObject<URopeComponent>();
	FRopeWrappingFallbackTestSeam::ConfigureExternalAnchor(
		*NodeZeroRope, Target, ERopePhase::Wrapping, /*AnchorNode*/ 0);
	FRopeWielderMovementConstraint NodeZeroConstraint;
	TestTrue(TEXT("a real node-zero contact arms an immediate hard leash"),
		NodeZeroRope->BuildWielderMovementConstraint(NodeZeroConstraint));
	TestTrue(TEXT("node-zero contact has an exact zero-radius material span"),
		FMath::IsNearlyZero(NodeZeroConstraint.MaxDistance));
	FVector NodeZeroProjected = FVector::ZeroVector;
	FVector NodeZeroNormal = FVector::ZeroVector;
	bool bNodeZeroWasProjected = false;
	NodeZeroRope->ConstrainWielderLocation(
		FVector(79.0f, 0.0f, 0.0f),
		NodeZeroProjected, NodeZeroNormal, bNodeZeroWasProjected);
	TestTrue(TEXT("node-zero contact pins the hand to its live contact point"),
		bNodeZeroWasProjected &&
		NodeZeroProjected.Equals(NodeZeroConstraint.PivotWorld, 0.01f));

	FRopeWrappingFallbackTestSeam::CommitWrapping(*Rope);
	TestEqual(TEXT("fixture exercised the real Wrapping commit path"),
		Rope->GetPhase(), ERopePhase::Wrapped);
	FRopeWielderMovementConstraint WrappedConstraint;
	TestTrue(TEXT("commit has no inactive hard-leash frame"),
		Rope->BuildWielderMovementConstraint(WrappedConstraint));
	TestTrue(TEXT("commit preserves the same pivot"),
		WrappedConstraint.PivotWorld.Equals(WrappingConstraint.PivotWorld, 0.01f));
	TestTrue(TEXT("commit preserves the same material radius"),
		FMath::IsNearlyEqual(
			WrappedConstraint.MaxDistance, WrappingConstraint.MaxDistance, 0.01f));

	FVector Constrained = FVector::ZeroVector;
	FVector Normal = FVector::ZeroVector;
	bool bProjected = false;
	TestTrue(TEXT("public movement adapter sees the committed constraint"),
		Rope->ConstrainWielderLocation(
			FVector(-20.0f, 0.0f, 0.0f), Constrained, Normal, bProjected));
	TestTrue(TEXT("outward point is projected"), bProjected);
	TestTrue(TEXT("projection stops on the current material boundary"),
		Constrained.Equals(FVector(20.0f, 0.0f, 0.0f), 0.01f));

	Rope->ReleaseWrap();
	FRopeWielderMovementConstraint ReleasedConstraint;
	TestFalse(TEXT("release clears the hard leash"),
		Rope->BuildWielderMovementConstraint(ReleasedConstraint));
	return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeReelFeasibilityStallTest,
	"DynamicRope.Component.Reel.WrappedFeasibilityStall",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeReelFeasibilityStallTest::RunTest(const FString& Parameters)
{
	// The wrapped reel-in stall gate: an inextensible rope cannot become shorter than the straight-line distance from
	// the hand to the anchor, so reeling in while the target does not come closer accumulates an unbounded violation
	// and the hard Chaos limit fights the joints.
	USceneComponent* Target = NewObject<USceneComponent>();
	Target->SetWorldLocation(FVector(20.0f, 0.0f, 0.0f)); // The anchor's world position is 20 + 60 = (80,0,0).

	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeWrappingFallbackTestSeam::ConfigureExternalAnchor(*Rope, Target, ERopePhase::Wrapped);
	Rope->RopeLength = 60.0f;    // Matches the SetRopeLength clamp's upper bound to the simulation's 60 cm.
	Rope->MinRopeLength = 10.0f; // Keeps the lower clamp from hiding the stall decision.
	Rope->SetReelRate(120.0f);
	const float Dt = 1.0f / 60.0f;

	// (1) Stalled: the straight line from the hand at -10 to the anchor at 80 is 90 cm, longer than the material's 60 cm, so it reels in no further.
	Rope->SetWorldLocation(FVector(-10.0f, 0.0f, 0.0f));
	FRopeWrappingFallbackTestSeam::Reel(*Rope, Dt);
	TestTrue(TEXT("reel stalls when the straight span already exceeds material length"),
		FMath::IsNearlyEqual(Rope->GetCurrentRopeLength(), 60.0f, 0.01f));

	// (2) Normal traction: a distance of 30 cm is under 60 cm, so it reels in at the previous rate.
	Rope->SetWorldLocation(FVector(50.0f, 0.0f, 0.0f));
	FRopeWrappingFallbackTestSeam::Reel(*Rope, Dt);
	TestTrue(TEXT("reel proceeds at full rate while the span leaves room"),
		FMath::IsNearlyEqual(Rope->GetCurrentRopeLength(), 60.0f - 120.0f * Dt, 0.01f));

	// (3) Settling at the lower bound: reeling in continuously stops at the distance minus the slack, being two frames of reel step, giving a bounded pulling bias.
	for (int32 i = 0; i < 60; ++i)
	{
		FRopeWrappingFallbackTestSeam::Reel(*Rope, Dt);
	}
	const float StallSlack = FMath::Max(120.0f * Dt * 2.0f, 1.0f);
	TestTrue(TEXT("reel settles at span minus the bounded pull bias"),
		FMath::IsNearlyEqual(Rope->GetCurrentRopeLength(), 30.0f - StallSlack, 0.1f));

	// (4) Paying out, being negative, is unaffected by the gate and lengthens exactly as before.
	Rope->SetReelRate(-120.0f);
	FRopeWrappingFallbackTestSeam::Reel(*Rope, Dt);
	TestTrue(TEXT("reel-out is not gated"),
		Rope->GetCurrentRopeLength() > 30.0f - StallSlack + 1.0f);
	return true;
}

#endif
