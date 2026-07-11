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
	// 즉시 결정
	const FRopeWrapConfig Config = MakeWrapConfig(3, 0.0f);
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

// Pull 산출(ComputePull): 손 쪽 첫 앵커에서 손 쪽 인접 노드 방향 + 해당 세그먼트 장력을 데이터로 내는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapComputePullTest,
	"DynamicRope.Wrap.ComputePullDirectionAndTension",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapComputePullTest::RunTest(const FString& Parameters)
{
	// 노드 x = 0,20,...,140(+X 직선). 앵커 2개(노드 5, 노드 3) → 손 쪽 첫 앵커 = 노드 3.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	Sim.SegmentTension.SetNumZeroed(Sim.Num() - 1);
	// 앵커(3)-손 쪽 인접 노드(2) 세그먼트
	Sim.SegmentTension[2] = 1234.0f;
	// 다른 세그먼트(선택되면 안 됨)
	Sim.SegmentTension[4] = 9999.0f;

	FRopeWrapController Wrap;
	Wrap.State.BoneName = FName("arm");
	for (const int32 NodeIndex : { 5, 3 })
	{
		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = NodeIndex;
		Anchor.Bone = FName("arm");
		Wrap.State.Anchors.Add(Anchor);
	}

	// 코너 임계 30도. 곧은 로프는 손(노드 0)까지 걸어가 방향이 정확히 chord(-X)가 된다.
	const float BendDeg = 30.0f;
	FRopePullSample Pull;
	TestTrue(TEXT("ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
	TestTrue(TEXT("pull sample valid"), Pull.bValid);
	TestEqual(TEXT("hand-side head anchor wins"), Pull.AnchorNode, 3);
	TestTrue(TEXT("bone attributed"), Pull.Bone == FName("arm"));
	// 곧은 로프: 앵커(노드 3, x=60)에서 손 쪽 노드(x<60)는 -X → look-ahead가 chord와 일치한다.
	TestTrue(FString::Printf(TEXT("direction %s points toward hand (-X)"), *Pull.Direction.ToCompactString()),
		Pull.Direction.Equals(FVector(-1, 0, 0), 0.01f));
	TestEqual(TEXT("tension from anchor-hand segment"), Pull.Tension, 1234.0f);

	// 앵커가 노드 0(손 핀)뿐이면 손 쪽 세그먼트가 없어 무효.
	FRopeWrapController WrapAtHand;
	WrapAtHand.State.BoneName = FName("arm");
	FRopeSurfaceAnchor HandAnchor;
	HandAnchor.NodeIndex = 0;
	WrapAtHand.State.Anchors.Add(HandAnchor);
	FRopePullSample InvalidPull;
	TestFalse(TEXT("anchor at hand node yields no pull"), WrapAtHand.ComputePull(Sim, BendDeg, InvalidPull));

	// 꺾인 자유 구간(벽 모서리): 앵커(노드 4)에서 첫 다리는 +Z(위)로 오르고, 모서리(노드 2)에서 손 쪽으로
	// 수평으로 꺾인다. look-ahead 방향은 로프 경로(첫 다리 = +Z)를 따라야 하며, 앵커→손 직선 chord
	// (대각선, 모서리를 가로지름)와 명확히 달라야 한다 — 이게 벽에 걸린 로프에서 chord가 벽을 관통하던 버그의 수정.
	FRopeSimState Bent;
	Bent.Positions = {
		// 0 손
		FVector(-40, 0, 40),
		// 1
		FVector(-20, 0, 40),
		// 2 모서리
		FVector(  0, 0, 40),
		// 3 첫 다리
		FVector(  0, 0, 20),
		// 4 앵커
		FVector(  0, 0,  0),
	};
	Bent.PrevPositions = Bent.Positions;
	Bent.SegmentLength = 20.0f;
	FRopeWrapController WrapBent;
	WrapBent.State.BoneName = FName("arm");
	{
		FRopeSurfaceAnchor A; A.NodeIndex = 4; A.Bone = FName("arm");
		WrapBent.State.Anchors.Add(A);
	}
	FRopePullSample BentPull;
	TestTrue(TEXT("bent ComputePull succeeds"), WrapBent.ComputePull(Bent, BendDeg, BentPull));
	// 첫 다리(노드 4→3→2, +Z)를 걷다 노드 2에서 90도 꺾임 감지 → 멈춤 → 방향 +Z(로프 경로), chord가 아님.
	TestTrue(FString::Printf(TEXT("bent direction %s follows first leg (+Z)"), *BentPull.Direction.ToCompactString()),
		BentPull.Direction.Equals(FVector(0, 0, 1), 0.01f));
	// 대각선(모서리 관통)
	const FVector Chord = (Bent.Positions[0] - Bent.Positions[4]).GetSafeNormal();
	TestFalse(TEXT("bent direction is NOT the straight chord"), BentPull.Direction.Equals(Chord, 0.05f));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
