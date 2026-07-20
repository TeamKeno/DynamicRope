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
		Rope.ContactTracker.DwellTime = Rope.DetectConfig.WrapDecisionTime;
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

#endif
