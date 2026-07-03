// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrapController.h"
#include "Collision/RopeCollider.h"
#include "Components/SkeletalMeshComponent.h"
#include "RopeTestHelpers.h"

namespace
{
	FRopeWrapConfig MakeWrapConfig(int32 MinNodes = 3, float DecisionTime = 0.15f, float ContactRadius = 3.0f)
	{
		FRopeWrapConfig C;
		C.ContactRadius = ContactRadius;
		C.MinLatchNodes = MinNodes;
		C.WrapDecisionTime = DecisionTime;
		return C;
	}

	/** 계약상 SourceMesh 자리에 넣을 빈 스켈레탈 메시 컴포넌트(월드 불필요 — 식별/전파 검증용). */
	USkeletalMeshComponent* MakeMockMesh()
	{
		return NewObject<USkeletalMeshComponent>();
	}
}

// MinLatchNodes 이상이 WrapDecisionTime 동안 지속 접촉하면 wrap을 커밋하는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapCommitTest,
	"DynamicRope.Wrap.CommitsAfterSustainedContact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapCommitTest::RunTest(const FString& Parameters)
{
	// 노드 x = 0,20,...,140. Arm(center 60, r25)+ContactRadius 3 = reach 28 → 노드 40/60/80 접촉(3개).
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	const USkeletalMeshComponent* Mesh = MakeMockMesh();
	RopeTest::FSphereMockCollider Arm(FVector(60.0f, 0.0f, 0.0f), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Arm };

	FRopeWrapController Wrap;
	const FRopeWrapConfig Config = MakeWrapConfig();
	FRopeWrapState Seed;
	bool bCommitted = false;
	for (int32 i = 0; i < 10 && !bCommitted; ++i)
	{
		bCommitted = Wrap.DecideWrap(Sim, Colliders, Config, 0.05f, Seed);
	}

	TestTrue(TEXT("wrap commits after sustained contact"), bCommitted);
	TestTrue(TEXT("committed bone is arm"), Seed.BoneName == FName("arm"));
	TestTrue(TEXT("contact SourceMesh propagates into seed"), Seed.Mesh.Get() == Mesh);
	TestEqual(TEXT("single head latch node"), Seed.Latched.Num(), 1);
	if (Seed.Latched.Num() > 0)
	{
		TestEqual(TEXT("head-most contact node wins"), Seed.Latched[0].NodeIndex, 2);
	}
	return true;
}

// MinLatchNodes 미만 접촉이면 아무리 오래 닿아도 커밋하지 않는가(스치는 접촉).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapNoCommitTest,
	"DynamicRope.Wrap.NoCommitBelowMinNodes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapNoCommitTest::RunTest(const FString& Parameters)
{
	// Arm(center 60, r8)+3 = reach 11 → 노드 60만 접촉(1개) < MinLatchNodes 3.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	RopeTest::FSphereMockCollider Arm(FVector(60.0f, 0.0f, 0.0f), 8.0f, FName("arm"), MakeMockMesh());
	TArray<IRopeCollider*> Colliders = { &Arm };

	FRopeWrapController Wrap;
	const FRopeWrapConfig Config = MakeWrapConfig(3, 0.15f, 3.0f);
	FRopeWrapState Seed;
	bool bCommitted = false;
	for (int32 i = 0; i < 20 && !bCommitted; ++i)
	{
		bCommitted = Wrap.DecideWrap(Sim, Colliders, Config, 0.05f, Seed);
	}

	TestFalse(TEXT("no commit below MinLatchNodes"), bCommitted);
	return true;
}

// C3: 두 bone이 동일 노드 수로 접촉(동점)할 때 dominant 선택이 안정적인가.
// 현재 코드는 TMap 순회 순서(FName 해시 의존, 비결정적)에 좌우된다. 이 테스트는 안정 tie-break
// (알파벳상 앞선 bone)을 단언하며 C3 수정의 "done 정의" 역할이다. 수정 전에는 red일 수 있다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapTieBreakTest,
	"DynamicRope.Wrap.TieBreakIsStable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapTieBreakTest::RunTest(const FString& Parameters)
{
	// 노드 x = 0,20,...,220. ArmA(40)→노드 20/40/60, ArmB(160)→노드 140/160/180. 3:3 동점.
	FRopeSimState Sim = RopeTest::MakeStraightRope(12, 220.0f);
	USkeletalMeshComponent* Mesh = MakeMockMesh();
	RopeTest::FSphereMockCollider ArmA(FVector(40.0f, 0.0f, 0.0f), 25.0f, FName("armA"), Mesh);
	RopeTest::FSphereMockCollider ArmB(FVector(160.0f, 0.0f, 0.0f), 25.0f, FName("armB"), Mesh);
	TArray<IRopeCollider*> Colliders = { &ArmA, &ArmB };

	FRopeWrapController Wrap;
	const FRopeWrapConfig Config = MakeWrapConfig(3, 0.0f); // 즉시 결정
	FRopeWrapState Seed;
	const bool bCommitted = Wrap.DecideWrap(Sim, Colliders, Config, 0.0f, Seed);

	TestTrue(TEXT("commits on tie"), bCommitted);
	TestTrue(TEXT("stable tie-break picks armA (C3 contract)"), Seed.BoneName == FName("armA"));
	TestEqual(TEXT("tie-break still latches only the head node"), Seed.Latched.Num(), 1);
	if (Seed.Latched.Num() > 0)
	{
		TestEqual(TEXT("armA head contact node"), Seed.Latched[0].NodeIndex, 1);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
