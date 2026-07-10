// Copyright Epic Games, Inc. All Rights Reserved.
//
// 랙돌 전환 매트릭스 단위 테스트(빌드 점검2 피드백 #1).
// 랙돌 전환 자체는 플러그인 밖(게임 코드)에서 일어나지만, 그 순간 로프가 지켜야 할 계약은
// 로직 레벨에서 월드 없이 검증할 수 있다:
//   (a) Hold: 본 트랜스폼이 한 프레임에 크게 점프해도 속도 주입 없이(Prev=Pos) 그대로 추종한다.
//   (b) Hold: 감긴 대상이 파괴되면(weak null) 폴백 없이 false — 호출자가 release.
//   (c) DecideWrap: 포즈 팝으로 지배 본이 바뀌면 dwell 타이머가 재시작한다(전이 프레임 오탐 방어).
//   (d) 솔버 마찰: 표면속도 스파이크의 드래그가 Coulomb 한계 μ·λ·w로 클램프된다(스파이크 크기 비비례).
//   (e) 감지기: 상대운동 평가가 표면속도를 빼고 계산한다 + 스파이크가 캡처 게이트에서 안 걸리는
//       현재 동작의 특성 고정(방어선은 Contacting 체류 + 후보 소실 dismiss라는 문서화).
// 실제 물리 본(IsSimulatingPhysics 분기), 부분 랙돌, 캡슐 재빌드는 PIE 체크리스트로 커버한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrapController.h"
#include "Logic/RopeFlightContactDetector.h"
#include "Solver/RopeXPBDSolver.h"
#include "Collision/RopeCollider.h"
#include "Components/SkeletalMeshComponent.h"
#include "RopeTestHelpers.h"

namespace
{
	/**
	 * 트랜스폼을 움직일 수 있는 mock mesh. 스켈레탈 에셋이 없으므로 GetSocketTransform(NAME_None)이
	 * 경고 없이 컴포넌트 트랜스폼을 반환한다 — 테스트에서는 컴포넌트 트랜스폼이 "본" 역할을 한다
	 * (랙돌 포즈 팝 = 이 트랜스폼의 한 프레임 점프로 모델링). 미등록 컴포넌트라 MoveComponent 경로를
	 * 타지 않도록 _Direct + UpdateComponentToWorld로 직접 옮긴다.
	 */
	USkeletalMeshComponent* MakeMovableMockMesh()
	{
		USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
		Mesh->UpdateComponentToWorld();
		return Mesh;
	}

	void TeleportMockMesh(USkeletalMeshComponent* Mesh, const FVector& NewLocation)
	{
		Mesh->SetRelativeLocation_Direct(NewLocation);
		Mesh->UpdateComponentToWorld();
	}

	/** 본 이름 없이(=컴포넌트 트랜스폼 추종) 노드 하나를 latch한 wrap을 시작한다. */
	void BeginMockWrap(FRopeWrapController& Wrap, const FRopeSimState& Sim,
		USkeletalMeshComponent* Mesh, int32 NodeIndex, FRopeNodeOverrideFrame& OutFrame)
	{
		FRopeWrapState Seed;
		Seed.BoneName = NAME_None;
		Seed.Mesh = Mesh;
		FRopeLatchNode Latch;
		Latch.NodeIndex = NodeIndex;
		Latch.Bone = NAME_None;
		Seed.Latched.Add(Latch);
		Wrap.BeginWrap(Sim, Seed, OutFrame);
	}
}

// (a) 랙돌 전환 프레임: 본(=mock 트랜스폼)이 한 프레임에 크게 점프해도 Hold가 노드를
// 새 본 위치에 정확히 재배치하고, 속도는 0으로 쓴다(PrevFromPosition) — 점프가 솔버로
// 속도로 주입되지 않는 것이 랙돌 전환 시 로프가 튀지 않는 근거다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeRagdollHoldFollowsJumpTest,
	"DynamicRope.Ragdoll.HoldFollowsBoneJumpWithZeroVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeRagdollHoldFollowsJumpTest::RunTest(const FString& Parameters)
{
	// 노드 x = 0,20,...,140. 노드 3(x=60)을 latch. mesh는 원점(identity)에서 시작.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakeMovableMockMesh();
	const FVector NodeWorld = Sim.Positions[3];

	FRopeWrapController Wrap;
	FRopeNodeOverrideFrame BeginFrame;
	BeginMockWrap(Wrap, Sim, Mesh, 3, BeginFrame);
	TestTrue(TEXT("wrap holds after BeginWrap"), Wrap.State.IsWrapped());

	// 점프 전 Hold: 앵커는 begin 시점 위치 그대로.
	FRopeNodeOverrideFrame HoldFrame;
	TestTrue(TEXT("Hold succeeds before jump"), Wrap.Hold(Sim, 0.016f, HoldFrame));
	TestTrue(TEXT("node stays at latch position before jump"),
		HoldFrame.Positions[3].Equals(NodeWorld, 0.01f));

	// 랙돌 포즈 팝: 본이 한 프레임에 +Z 500cm 점프.
	const FVector Jump(0.0f, 0.0f, 500.0f);
	TeleportMockMesh(Mesh, Jump);
	TestTrue(TEXT("mock mesh transform actually moved (test rig sanity)"),
		Mesh->GetComponentLocation().Equals(Jump, 0.01f));

	FRopeNodeOverrideFrame JumpFrame;
	TestTrue(TEXT("Hold succeeds on jump frame"), Wrap.Hold(Sim, 0.016f, JumpFrame));
	// 노드는 본을 따라 정확히 점프량만큼 이동.
	TestTrue(FString::Printf(TEXT("node follows bone jump exactly (%s)"), *JumpFrame.Positions[3].ToCompactString()),
		JumpFrame.Positions[3].Equals(NodeWorld + Jump, 0.01f));
	// 속도 0 기록(PrevFromPosition): 점프가 Verlet 속도로 주입되지 않는다.
	TestTrue(TEXT("jump is written with zero velocity (PrevFromPosition)"),
		(JumpFrame.Flags[3] & RopeNodeOverride::PrevFromPosition) != 0);
	// 노드는 여전히 logic 소유(InvMass 0).
	TestTrue(TEXT("node stays logic-owned (InvMass override 0)"),
		(JumpFrame.Flags[3] & RopeNodeOverride::InvMass) != 0 && JumpFrame.InvMass[3] == 0.0f);
	return true;
}

// (b) 감긴 대상 소실: 랙돌 사망 연출 등으로 mesh가 파괴되면 Hold가 false를 반환해
// 호출자(component)가 release하게 한다 — dangling 역참조/엉뚱한 위치로 끌기 없음.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeRagdollHoldMeshLossTest,
	"DynamicRope.Ragdoll.HoldReleasesOnMeshLoss",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeRagdollHoldMeshLossTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakeMovableMockMesh();

	FRopeWrapController Wrap;
	FRopeNodeOverrideFrame BeginFrame;
	BeginMockWrap(Wrap, Sim, Mesh, 3, BeginFrame);

	FRopeNodeOverrideFrame HoldFrame;
	TestTrue(TEXT("Hold succeeds while mesh alive"), Wrap.Hold(Sim, 0.016f, HoldFrame));

	// 대상 파괴(가비지 마킹 → weak Get() null).
	Mesh->MarkAsGarbage();
	FRopeNodeOverrideFrame LostFrame;
	TestFalse(TEXT("Hold returns false after mesh loss (caller must release)"),
		Wrap.Hold(Sim, 0.016f, LostFrame));
	TestFalse(TEXT("no node override is produced on the loss frame"), LostFrame.HasAny());
	return true;
}

// (c) 전이 프레임 오탐 방어: 랙돌 포즈 팝으로 지배 본이 A→B로 바뀌면 DecideWrap의 dwell
// 타이머가 재시작해야 한다 — A에서 쌓은 체류 시간이 B로 이월되면 팝 순간 잘못된 본에 즉시
// 커밋될 수 있다. 커밋은 B가 WrapDecisionTime을 온전히 다시 채운 뒤에만 일어나야 한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeRagdollDwellResetTest,
	"DynamicRope.Ragdoll.DecideWrapDwellResetsOnBoneSwitch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeRagdollDwellResetTest::RunTest(const FString& Parameters)
{
	// 노드 x = 0,20,...,140. 반경 25 + ContactRadius 3 = reach 28 → (60,0,0) 구는 노드 40/60/80 접촉(3개).
	const FVector NearCenter(60.0f, 0.0f, 0.0f);
	const FVector FarAway(0.0f, 0.0f, 100000.0f); // 접촉 불가 위치(비활성화용)

	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
	RopeTest::FSphereMockCollider BoneA(NearCenter, 25.0f, FName("boneA"), Mesh);
	RopeTest::FSphereMockCollider BoneB(FarAway, 25.0f, FName("boneB"), Mesh);
	TArray<IRopeCollider*> Colliders = { &BoneA, &BoneB };

	FRopeWrapConfig Config;
	Config.ContactRadius = 3.0f;
	Config.MinLatchNodes = 3;
	Config.WrapDecisionTime = 0.15f;
	const float Dt = 0.05f;

	FRopeWrapController Wrap;
	FRopeWrapState Seed;

	// boneA 접촉 3프레임: dwell 0 → 0.05 → 0.10 (< 0.15) — 아직 커밋 없음.
	for (int32 i = 0; i < 3; ++i)
	{
		TestFalse(FString::Printf(TEXT("no commit while boneA dwell below threshold (frame %d)"), i),
			Wrap.DecideWrap(Sim, Colliders, Config, Dt, Seed));
	}

	// 포즈 팝: 접촉이 boneA → boneB로 스왑(같은 위치에 B가 들어옴).
	BoneA.Center = FarAway;
	BoneB.Center = NearCenter;

	// 스왑 직후 3프레임: A의 dwell(0.10)이 이월됐다면 두 번째 프레임(0.15)에 조기 커밋된다.
	// 재시작이 맞으면 B 기준 dwell 0 → 0.05 → 0.10 — 여전히 커밋 없음.
	for (int32 i = 0; i < 3; ++i)
	{
		TestFalse(FString::Printf(TEXT("dwell restarts on bone switch — no early commit (frame %d)"), i),
			Wrap.DecideWrap(Sim, Colliders, Config, Dt, Seed));
	}

	// B가 자체적으로 WrapDecisionTime을 채우는 프레임(0.15)에 커밋 — 본은 반드시 B.
	TestTrue(TEXT("commits after boneB accumulates full dwell"),
		Wrap.DecideWrap(Sim, Colliders, Config, Dt, Seed));
	TestTrue(TEXT("committed bone is the post-pop bone (boneB)"), Seed.BoneName == FName("boneB"));
	return true;
}

// (d) 전이 프레임 표면속도 스파이크: 랙돌 켜지는 프레임에 본이 튀면 캡슐 prev 끝점 대비
// 이동이 커져 SurfaceVelocity가 스파이크한다. 마찰 드래그는 Coulomb 한계 μ·λ·w로 클램프되므로
// 로프가 스파이크 속도에 비례해 쓸려가면 안 된다 — 스파이크를 10배로 키워도 프레임 변위가
// 거의 그대로여야 한다(클램프 활성 증명).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeRagdollFrictionClampTest,
	"DynamicRope.Ragdoll.FrictionClampBoundsSurfaceVelocitySpike",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeRagdollFrictionClampTest::RunTest(const FString& Parameters)
{
	// 구(중심 (60,0,-200), r200): 상단 표면이 z=0 근방 → z=0 직선 로프의 x=60 주변 노드가
	// CollisionRadius(2) 안으로 침투해 접촉·마찰 활성. 중력이 로프를 표면에 눌러 λ가 쌓인다.
	auto MaxXDisplacementAfterOneStep = [](float SpikeSpeed) -> float
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
		TArray<FVector> InitialPositions = Sim.Positions;

		USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
		RopeTest::FSphereMockCollider Body(FVector(60.0f, 0.0f, -200.0f), 200.0f, FName("body"), Mesh);
		Body.SurfaceVelocity = FVector(SpikeSpeed, 0.0f, 0.0f); // 포즈 팝: 접선(+X) 스파이크
		TArray<IRopeCollider*> Colliders = { &Body };

		FRopeSolverConfig Config; // 기본값: Friction 0.5, CollisionRadius 2, 중력 -Z
		FRopeXPBDSolver Solver;
		Solver.Step(Sim, Config, Colliders, 1.0f / 60.0f);

		float MaxDisp = 0.0f;
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			MaxDisp = FMath::Max(MaxDisp,
				FMath::Abs(static_cast<float>(Sim.Positions[i].X - InitialPositions[i].X)));
		}
		return RopeTest::AnyNaN(Sim) ? -1.0f : MaxDisp;
	};

	// 600 m/s 스파이크: 무클램프 전량 전달이면 프레임당 60000/60 = 1000cm 쓸림.
	const float Disp1x = MaxXDisplacementAfterOneStep(60000.0f);
	// 10배 스파이크(6 km/s): 무클램프면 변위도 ~10배.
	const float Disp10x = MaxXDisplacementAfterOneStep(600000.0f);

	TestTrue(TEXT("no NaN with 1x spike"), Disp1x >= 0.0f);
	TestTrue(TEXT("no NaN with 10x spike"), Disp10x >= 0.0f);
	// 스모크 상한: 표면 이동량(1000cm)의 절반도 전달되지 않아야 한다(클램프가 없으면 ~1000).
	TestTrue(FString::Printf(TEXT("drag is far below full surface transport (%.1fcm < 500cm)"), Disp1x),
		Disp1x < 500.0f);
	// 핵심 성질: 클램프 한계는 μ·λ·w(스파이크와 무관) → 스파이크 10배에도 변위는 거의 동일해야 한다.
	TestTrue(FString::Printf(TEXT("drag does not scale with spike (1x=%.2fcm, 10x=%.2fcm)"), Disp1x, Disp10x),
		Disp10x < Disp1x * 2.0f + 1.0f);
	return true;
}

// (e) 상대운동 평가와 캡처의 현재 계약 고정:
//  - EvaluateRelativeMotion은 로프 속도에서 표면속도를 빼므로, 정지 로프 + 움직이는 표면이면
//    상대 접선 속도 = 표면 속도 크기다(움직이는 본 위에서도 "스침" 판정이 가능한 근거).
//  - ShouldCapture는 현재 품질 게이트가 바이패스라 스파이크가 캡처를 막지 않는다(특성 고정).
//    전이 프레임 오탐의 실제 방어선은 Contacting 체류(WrapDecisionTime)와 후보 소실 dismiss,
//    그리고 위 (c)의 dwell 재시작이다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeRagdollRelativeMotionTest,
	"DynamicRope.Ragdoll.RelativeMotionSubtractsSurfaceVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeRagdollRelativeMotionTest::RunTest(const FString& Parameters)
{
	// 정지 로프(Prev == Pos), 표면은 +X 500으로 이동 중. 법선 +Z → 표면 운동은 전부 접선 성분.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();

	auto MakeSpikeCandidate = [&](int32 NodeIndex, float SurfaceSpeed)
	{
		FRopeContactCandidate C;
		C.bValid = true;
		C.NodeIndex = NodeIndex;
		C.Bone = FName("arm");
		C.Mesh = Mesh;
		C.WorldPoint = Sim.Positions[NodeIndex] - FVector(0.0f, 0.0f, 2.0f);
		C.Normal = FVector::UpVector;
		C.Penetration = 1.0f;
		C.SurfaceVelocity = FVector(SurfaceSpeed, 0.0f, 0.0f);
		return C;
	};

	FRopeFlightContactDetector::FParams Params;
	Params.MinLatchNodes = 2;

	TArray<FRopeContactCandidate> Candidates;
	Candidates.Add(MakeSpikeCandidate(3, 500.0f));
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, Params, Candidates);

	TestTrue(TEXT("candidate stays valid (mock mesh has no bone axis to judge miss cone)"),
		Candidates[0].bValid);
	TestTrue(FString::Printf(TEXT("relative tangential speed equals surface speed for a resting rope (%.1f)"),
		Candidates[0].RelativeTangentialSpeed),
		FMath::IsNearlyEqual(Candidates[0].RelativeTangentialSpeed, 500.0f, 0.5f));

	// 캡처 특성 고정: 표면속도 스파이크(60000)가 있어도 MinLatchNodes만 차면 캡처된다.
	// 이 단언이 깨지는 날은 품질 게이트가 켜진 날이다 — 그때 스파이크 컷 기준과 함께 갱신할 것.
	TArray<FRopeContactCandidate> SpikeCandidates;
	SpikeCandidates.Add(MakeSpikeCandidate(3, 60000.0f));
	SpikeCandidates.Add(MakeSpikeCandidate(4, 60000.0f));
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, Params, SpikeCandidates);
	TestTrue(TEXT("capture proceeds despite surface-velocity spike (quality gate bypassed — documented behavior)"),
		FRopeFlightContactDetector::ShouldCapture(SpikeCandidates, Params));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
