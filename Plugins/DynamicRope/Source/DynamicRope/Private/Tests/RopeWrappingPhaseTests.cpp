// Copyright Epic Games, Inc. All Rights Reserved.
//
// Wrapping 페이즈(FRopeWrappingPhase) 단위 테스트. 정적 랩 대상 경로(가상 본 이름 + 컴포넌트
// 트랜스폼)를 이용해 월드 없이 SurfaceVectorField 경로 빌드를 완주시킨다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrappingPhase.h"
#include "Collision/RopeCollider.h"
#include "Components/SceneComponent.h"
#include "RopeTestHelpers.h"

namespace
{
	// 정적 랩 대상 경로를 그대로 쓰는 mock 대상 — ResolveBindingWorld가 가상 본 이름에 대해
	// 경고 없이 컴포넌트(identity) 트랜스폼을 반환하므로 본 로컬 = 월드가 된다.
	USceneComponent* MakeMockTarget()
	{
		return NewObject<USceneComponent>();
	}
}

// SurfaceVectorField 경로 빌드가 캡슐(원기둥) 주위를 완주하고, 앵커가 표면 위에 놓이며, 빌드 중
// 적분한 누적 감싼 각도가 단일 축 helix 공식과 일치하는가 — rolling axis(본 전환 시 축 재해석)
// 도입 후에도 단일 본(전환 없음) 결과가 기존 공식과 동치임을 고정하는 회귀 계약.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingCapsuleAngleTest,
	"DynamicRope.Wrapping.SurfaceVectorFieldCapsuleWrapAngle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingCapsuleAngleTest::RunTest(const FString& Parameters)
{
	// 캡슐: Z축 정렬(-50~+50), 반지름 25 → 둘레 ≈157cm. 로프 160cm(9노드)면 약 한 바퀴 감는다.
	USceneComponent* Mesh = MakeMockTarget();
	FCapsuleCollider Capsule(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Capsule };

	// 로프는 latch 지점(+X 표면)에서 +Y(원주 방향)로 뻗어 있다.
	FRopeSimState Sim = RopeTest::MakeStraightRope(9, 160.0f, FVector(25, 0, 0), FVector(0, 1, 0));

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("arm");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(25, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(25, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	const FRopeWrapConfig Config;
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		ERopeWrappingPathMode::SurfaceVectorField, /*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true };

	FRopeWrappingPhase Wrapping;
	TestTrue(TEXT("wrapping begins on capsule"), Wrapping.Begin(Latch, Mesh, FName("arm"), 0.16f, Sim, Ctx));

	for (int32 Iteration = 0; Iteration < 256 && Wrapping.State.bPathBuildActive; ++Iteration)
	{
		Wrapping.AdvancePathBuild(Sim, Ctx);
	}

	TestTrue(TEXT("path build completes around the capsule"), Wrapping.State.bPathBuildComplete);
	TestEqual(TEXT("one anchor per tail node"), Wrapping.State.Anchors.Num(), 9);

	// 모든 anchor는 캡슐 표면(축 반경 25cm) 위 — identity 트랜스폼이라 로컬 XY 거리로 바로 잰다.
	for (const FRopeSurfaceAnchor& Anchor : Wrapping.State.Anchors)
	{
		const float RadialDistance = static_cast<float>(
			FVector2D(Anchor.LocalSurfacePosition.X, Anchor.LocalSurfacePosition.Y).Size());
		TestTrue(FString::Printf(TEXT("anchor %d on capsule surface (r=%.1f)"), Anchor.NodeIndex, RadialDistance),
			FMath::IsNearlyEqual(RadialDistance, 25.0f, 3.0f));
	}

	// 누적 각도 ≈ 단일 축 helix 공식: angle = (거리 / 피치 보정) / 반지름 ≈ 356°.
	// 걷기가 현(chord) 스텝(세그먼트 절반 = 10cm)이라 호 기반 공식보다 스텝당 atan만큼 낮게 적분된다
	// (r=25에서 약 -4.7% ≈ -16°) — 허용 오차는 그 이산화 편차를 포함해 30°로 잡는다.
	float AngleDeg = 0.0f;
	TestTrue(TEXT("wrapped angle computable"), Wrapping.ComputeWrappedAngleAtLastBuiltPoint(Sim, Ctx, AngleDeg));
	const float PitchScale = Config.WrappingHelixPitchScale;
	const float ExpectedDeg = FMath::RadiansToDegrees(
		(160.0f / FMath::Sqrt(1.0f + PitchScale * PitchScale)) / 25.0f);
	TestTrue(FString::Printf(TEXT("accumulated angle matches helix formula (%.0f vs %.0f deg)"), AngleDeg, ExpectedDeg),
		FMath::IsNearlyEqual(AngleDeg, ExpectedDeg, 30.0f));
	return true;
}

// 시드 다중화: 보조 시드 앵커(State.SecondarySeedAnchors, Begin 전에 적재)가 있으면
// ① 경로 길이가 첫 보조 노드 앞까지로 클램프되고(경로 앵커와 보조 앵커의 노드 이중 소유 방지),
// ② front 모션이 보조 노드를 경로로 끌어가는 대신 자기 본의 앵커 프레임에 hold하며, 보조 노드
//    너머 남는 로프는 건드리지 않고,
// ③ 커밋 시드에 경로 앵커와 보조 앵커가 함께 실린다(BeginWrap/Hold는 앵커별 Bone/Mesh 지원).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingSecondarySeedTest,
	"DynamicRope.Wrapping.SecondarySeedAnchorsHoldAndCommit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingSecondarySeedTest::RunTest(const FString& Parameters)
{
	// dominant 대상: 위 테스트와 같은 캡슐(본 "arm"). 보조 대상: 다른 mesh의 본 "leg" —
	// 노드 6이 (100,0,0) 표면점에 접촉해 있었다고 가정한 시드 앵커.
	USceneComponent* Mesh = MakeMockTarget();
	USceneComponent* SecondaryMesh = MakeMockTarget();
	FCapsuleCollider Capsule(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Capsule };

	FRopeSimState Sim = RopeTest::MakeStraightRope(9, 160.0f, FVector(25, 0, 0), FVector(0, 1, 0));

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("arm");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(25, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(25, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	FRopeSurfaceAnchor Secondary;
	Secondary.NodeIndex = 6;
	Secondary.Bone = FName("leg");
	Secondary.Mesh = SecondaryMesh;
	Secondary.LocalSurfacePosition = FVector(100, 0, 0);
	Secondary.LocalNormal = FVector(1, 0, 0);
	Secondary.LocalTangent = FVector(0, 1, 0);
	Secondary.StartWorldPosition = FVector(100, 0, 0);
	Secondary.SurfaceOffset = 1.0f;
	Secondary.RopeDistance = 6 * 20.0f;

	const FRopeWrapConfig Config;
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		ERopeWrappingPathMode::SurfaceVectorField, /*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true };

	FRopeWrappingPhase Wrapping;
	Wrapping.State.SecondarySeedAnchors.Add(Secondary);
	TestTrue(TEXT("wrapping begins with a secondary seed"), Wrapping.Begin(Latch, Mesh, FName("arm"), 0.16f, Sim, Ctx));

	// ① 경로는 보조 노드(6) 앞까지만: NumTailNodes 6 == 노드 0~5.
	TestEqual(TEXT("path length clamps before the secondary node"), Wrapping.State.NumTailNodes, 6);

	for (int32 Iteration = 0; Iteration < 256 && Wrapping.State.bPathBuildActive; ++Iteration)
	{
		Wrapping.AdvancePathBuild(Sim, Ctx);
	}
	TestTrue(TEXT("clamped path build completes"), Wrapping.State.bPathBuildComplete);
	TestEqual(TEXT("path anchors stop before the secondary node"), Wrapping.State.Anchors.Num(), 6);

	// ② front 모션: 충분한 dt로 front를 끝까지 민 뒤 — 보조 노드는 자기 앵커 프레임(표면+오프셋),
	//    보조 노드 너머(7~8)는 어떤 위치 쓰기도 받지 않는다.
	FRopeNodeOverrideFrame Frame;
	for (int32 Step = 0; Step < 64; ++Step)
	{
		Frame.Reset();
		Wrapping.ApplyFrontMotion(Sim, 0.05f, Ctx, Frame);
	}
	TestTrue(TEXT("secondary node receives a position override"),
		Frame.Flags.IsValidIndex(6) && (Frame.Flags[6] & RopeNodeOverride::Position) != 0);
	TestTrue(TEXT("secondary node held at its own anchor frame"),
		Frame.Positions.IsValidIndex(6) && Frame.Positions[6].Equals(FVector(101, 0, 0), 0.1f));
	TestTrue(TEXT("nodes beyond the secondary stay untouched"),
		Frame.Flags.IsValidIndex(8) &&
		(Frame.Flags[7] & RopeNodeOverride::Position) == 0 &&
		(Frame.Flags[8] & RopeNodeOverride::Position) == 0);

	// ③ 커밋 시드: 경로 앵커 6개 + 보조 앵커 1개, 보조 본/mesh가 그대로 실린다.
	const FRopeWrapState Seed = Wrapping.BuildCommitSeed(Sim, Mesh);
	TestEqual(TEXT("commit seed carries path + secondary anchors"), Seed.Anchors.Num(), 7);
	const FRopeSurfaceAnchor* Committed = Seed.Anchors.FindByPredicate(
		[](const FRopeSurfaceAnchor& Anchor) { return Anchor.NodeIndex == 6; });
	if (!TestNotNull(TEXT("secondary anchor committed"), Committed))
	{
		return false;
	}
	TestTrue(TEXT("secondary anchor keeps its bone"), Committed->Bone == FName("leg"));
	TestTrue(TEXT("secondary anchor keeps its mesh"), Committed->Mesh.Get() == SecondaryMesh);
	return true;
}

// 감김 축 소스 설정(ERopeWrappingAxisSource, 진행 방향 기반 wrap 1단계):
// - TravelPlaneFirst: 가이드 평면 normal이 있으면 형상 축(캡슐 Z)보다 앞선다 — 축이 운동 평면에 고정.
// - ShapeAxisFirst(기본): 가이드 평면이 있어도 형상 축이 이긴다 — 기존 동작 그대로(CL 341 주석 토글의
//   정식화이므로, 기본값에서 아무것도 달라지지 않음을 함께 고정한다).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingAxisSourceTest,
	"DynamicRope.Wrapping.AxisSourceConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingAxisSourceTest::RunTest(const FString& Parameters)
{
	// 캡슐(형상 축 = Z) + 가이드 평면 normal = Y — 두 소스가 뚜렷이 구분되는 배치.
	USceneComponent* Mesh = MakeMockTarget();
	FCapsuleCollider Capsule(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Capsule };

	FRopeSimState Sim = RopeTest::MakeStraightRope(9, 160.0f, FVector(25, 0, 0), FVector(0, 1, 0));

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("arm");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(25, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(25, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	const FVector GuidePlaneNormal(0, 1, 0);

	FRopeWrapConfig TravelConfig;
	TravelConfig.WrappingAxisSource = ERopeWrappingAxisSource::TravelPlaneFirst;
	const FRopeWrappingPhase::FContext TravelCtx{ TravelConfig, Colliders,
		ERopeWrappingPathMode::SurfaceVectorField, /*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, GuidePlaneNormal };

	FRopeWrappingPhase TravelWrapping;
	TestTrue(TEXT("wrapping begins with travel-plane axis"),
		TravelWrapping.Begin(Latch, Mesh, FName("arm"), 0.16f, Sim, TravelCtx));
	TestTrue(FString::Printf(TEXT("TravelPlaneFirst axis follows the guide plane normal (dir=%s)"),
			*TravelWrapping.State.PathAxisDirection.ToString()),
		FMath::Abs(FVector::DotProduct(TravelWrapping.State.PathAxisDirection, GuidePlaneNormal)) > 0.99f);

	FRopeWrapConfig DefaultConfig;
	const FRopeWrappingPhase::FContext DefaultCtx{ DefaultConfig, Colliders,
		ERopeWrappingPathMode::SurfaceVectorField, /*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, GuidePlaneNormal };

	FRopeWrappingPhase DefaultWrapping;
	TestTrue(TEXT("wrapping begins with default axis source"),
		DefaultWrapping.Begin(Latch, Mesh, FName("arm"), 0.16f, Sim, DefaultCtx));
	TestTrue(FString::Printf(TEXT("ShapeAxisFirst keeps the capsule shape axis (dir=%s)"),
			*DefaultWrapping.State.PathAxisDirection.ToString()),
		FMath::Abs(FVector::DotProduct(DefaultWrapping.State.PathAxisDirection, FVector(0, 0, 1))) > 0.99f);
	return true;
}

#endif
