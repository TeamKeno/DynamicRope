// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeComponent의 Contacting -> Wrapping 오케스트레이션 회귀 테스트.

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

		// BoneCenteredGuidePlane이 캡슐의 Z축을 사용하도록 고정해 테스트가 컴포넌트 축 폴백의
		// 동률 선택 순서에 의존하지 않게 한다.
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

		// Flight에서 exact primary로 이미 capture된 상태. 다음 120Hz Contacting tick에서도 deeper
		// same-mesh neighbor가 primary를 가려 dismiss시키지 않는지 검증한다.
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

	// 정상 Contacting 시드 조립은 실제 contact point/normal/tangent 기반 anchor를 반드시 만든다.
	const FRopeWrapState& NormalSeed = FRopeWrappingFallbackTestSeam::GetPendingSeed(*Rope);
	TestEqual(TEXT("valid Contacting candidate creates one latch"), NormalSeed.Latched.Num(), 1);
	TestEqual(TEXT("valid Contacting candidate creates one surface anchor"), NormalSeed.Anchors.Num(), 1);
	if (NormalSeed.Anchors.Num() == 1)
	{
		TestTrue(TEXT("normal seed preserves the contact surface normal"),
			NormalSeed.Anchors[0].LocalNormal.Equals(FVector::XAxisVector, KINDA_SMALL_NUMBER));
	}

	// 정상 조립 결과에서 anchor만 제거해 문제 상태를 재현한다. 경고 발생과 함께 synthetic anchor가
	// 만들어지고 Wrapping이 시작되면 fallback 분기가 실제 실행 가능한 코드임이 확인된다.
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
	// Wrapped 릴-인 실속 게이트: 비신축 로프는 손↔앵커 직선 거리보다 짧아질 수 없다 — 대상이 안
	// 끌려오는데 계속 감으면 위반이 무한 누적돼 하드 Chaos 리밋이 관절과 싸운다(스네어 바둥거림).
	USceneComponent* Target = NewObject<USceneComponent>();
	Target->SetWorldLocation(FVector(20.0f, 0.0f, 0.0f)); // 앵커 월드 = 20 + 60 = (80,0,0)

	URopeComponent* Rope = NewObject<URopeComponent>();
	FRopeWrappingFallbackTestSeam::ConfigureExternalAnchor(*Rope, Target, ERopePhase::Wrapped);
	Rope->RopeLength = 60.0f;    // SetRopeLength 클램프 상한을 Sim(60cm)과 일치.
	Rope->MinRopeLength = 10.0f; // 하한 클램프가 실속 판정을 가리지 않게.
	Rope->SetReelRate(120.0f);
	const float Dt = 1.0f / 60.0f;

	// (1) 실속: 손(-10)↔앵커(80) 직선 90cm > 재질 60cm — 더 감기지 않는다.
	Rope->SetWorldLocation(FVector(-10.0f, 0.0f, 0.0f));
	FRopeWrappingFallbackTestSeam::Reel(*Rope, Dt);
	TestTrue(TEXT("reel stalls when the straight span already exceeds material length"),
		FMath::IsNearlyEqual(Rope->GetCurrentRopeLength(), 60.0f, 0.01f));

	// (2) 정상 견인: 거리 30cm < 60cm — 종전 속도 그대로 감긴다.
	Rope->SetWorldLocation(FVector(50.0f, 0.0f, 0.0f));
	FRopeWrappingFallbackTestSeam::Reel(*Rope, Dt);
	TestTrue(TEXT("reel proceeds at full rate while the span leaves room"),
		FMath::IsNearlyEqual(Rope->GetCurrentRopeLength(), 60.0f - 120.0f * Dt, 0.01f));

	// (3) 하한 안착: 계속 감으면 거리 − 슬랙(릴 2프레임 스텝)에서 멈춘다 = 유계 당김 바이어스.
	for (int32 i = 0; i < 60; ++i)
	{
		FRopeWrappingFallbackTestSeam::Reel(*Rope, Dt);
	}
	const float StallSlack = FMath::Max(120.0f * Dt * 2.0f, 1.0f);
	TestTrue(TEXT("reel settles at span minus the bounded pull bias"),
		FMath::IsNearlyEqual(Rope->GetCurrentRopeLength(), 30.0f - StallSlack, 0.1f));

	// (4) 풀기(-)는 게이트 무관 — 종전 그대로 늘어난다.
	Rope->SetReelRate(-120.0f);
	FRopeWrappingFallbackTestSeam::Reel(*Rope, Dt);
	TestTrue(TEXT("reel-out is not gated"),
		Rope->GetCurrentRopeLength() > 30.0f - StallSlack + 1.0f);
	return true;
}

#endif
