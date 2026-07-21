// Copyright Epic Games, Inc. All Rights Reserved.
//
// Wrapping 페이즈(FRopeWrappingPhase) 단위 테스트. 정적 랩 대상 경로(가상 본 이름 + 컴포넌트
// 트랜스폼)를 이용해 월드 없이 SurfaceVectorField 경로 빌드를 완주시킨다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrappingPhase.h"
#include "Collision/RopeCollider.h"
#include "Core/RopeWrapTarget.h"
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

	// 구조체 기본값이 0=auto(컴포넌트 경계에서만 해석)가 되면서, 월드 없는 직접 호출은
	// 구 기본값(3cm)을 명시로 고정한다 — 테스트 기하(reach 계산)가 이 값 기준.
	FRopeWrapConfig MakeTestWrapConfig()
	{
		FRopeWrapConfig C;
		C.ContactQueryRadius = 3.0f;
		return C;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingInitializationFailureStateTest,
	"DynamicRope.Wrapping.InitializationFailureState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingInitializationFailureStateTest::RunTest(const FString& Parameters)
{
	FRopeWrappingPhase Wrapping;
	// Seed contradictory stale flags to prove that even the earliest validation failure
	// establishes the canonical failed terminal state.
	Wrapping.State.bPathBuildActive = true;
	Wrapping.State.bPathBuildComplete = true;
	Wrapping.State.bPathBuildFailed = false;
	Wrapping.State.PathBuildFailureReason = TEXT("StaleFailure");

	FRopeSurfaceAnchor InvalidLatch;
	FRopeSimState EmptySim;
	TArray<IRopeCollider*> NoColliders;
	const FRopeWrapConfig Config = MakeTestWrapConfig();
	const FRopeWrappingPhase::FContext Ctx{ Config, NoColliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingInitializationFailureTest"), true };

	AddExpectedError(TEXT("Path initialization rejected: reason=InvalidLatchInput"),
		EAutomationExpectedErrorFlags::Contains, 1);
	AddExpectedError(TEXT("Wrap begin failed: reason=InvalidLatchInput"),
		EAutomationExpectedErrorFlags::Contains, 1);
	TestFalse(TEXT("invalid latch rejects wrapping initialization"),
		Wrapping.Begin(InvalidLatch, 0.16f, EmptySim, Ctx));
	TestFalse(TEXT("failed initialization is not active"), Wrapping.State.bPathBuildActive);
	TestFalse(TEXT("failed initialization is not complete"), Wrapping.State.bPathBuildComplete);
	TestTrue(TEXT("failed initialization records failure"), Wrapping.State.bPathBuildFailed);
	TestEqual(TEXT("failed initialization preserves its reason"),
		Wrapping.State.PathBuildFailureReason, FString(TEXT("InvalidLatchInput")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingPathPointCoordinateContractTest,
	"DynamicRope.Wrapping.PathPointCoordinateContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingPathPointCoordinateContractTest::RunTest(const FString& Parameters)
{
	constexpr float SurfaceOffset = 2.0f;
	const FVector NormalWorld = FVector::XAxisVector;

	FRopeWrapPathPoint SurfacePoint;
	SurfacePoint.SurfaceWorld = FVector(10.0f, 0.0f, 0.0f);
	SurfacePoint.NormalWorld = NormalWorld;
	SurfacePoint.TangentWorld = FVector::YAxisVector;
	SurfacePoint.DistanceFromLatch = 0.0f;
	TestTrue(TEXT("surface point applies the normal offset"),
		FRopeWrappingPhase::GetPathPointCenterlineWorld(SurfacePoint, SurfaceOffset)
			.Equals(FVector(12.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));

	FRopeWrapPathPoint BridgePoint = SurfacePoint;
	BridgePoint.bBridge = true;
	TestTrue(TEXT("bridge point uses the same stored-position contract as a surface point"),
		FRopeWrappingPhase::GetPathPointCenterlineWorld(BridgePoint, SurfaceOffset)
			.Equals(FVector(12.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));

	FRopeWrapPathPoint VirtualPoint = SurfacePoint;
	VirtualPoint.SurfaceWorld = FVector(20.0f, 0.0f, 0.0f);
	VirtualPoint.DistanceFromLatch = 10.0f;
	VirtualPoint.bVirtual = true;
	TestTrue(TEXT("virtual point already stores its centerline"),
		FRopeWrappingPhase::GetPathPointCenterlineWorld(VirtualPoint, SurfaceOffset)
			.Equals(FVector(20.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));

	const FVector EncodedSurface = FRopeWrappingPhase::EncodePathPointPositionFromCenterline(
		FVector(12.0f, 0.0f, 0.0f), NormalWorld, /*bVirtual=*/false, SurfaceOffset);
	const FVector EncodedVirtual = FRopeWrappingPhase::EncodePathPointPositionFromCenterline(
		FVector(20.0f, 0.0f, 0.0f), NormalWorld, /*bVirtual=*/true, SurfaceOffset);
	TestTrue(TEXT("surface centerline encoding removes the offset"),
		EncodedSurface.Equals(FVector(10.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));
	TestTrue(TEXT("virtual centerline encoding preserves the position"),
		EncodedVirtual.Equals(FVector(20.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));

	FRopeWrapPathPoint SurfaceToVirtualMidpoint;
	FRopeWrappingPhase::InterpolateWrappingPathPoints(
		SurfacePoint, VirtualPoint, /*SampleDistance=*/5.0f,
		SurfaceOffset, SurfaceToVirtualMidpoint);
	TestTrue(TEXT("surface-to-virtual interpolation remains virtual"),
		SurfaceToVirtualMidpoint.bVirtual);
	TestTrue(TEXT("surface-to-virtual interpolation lerps decoded centerlines"),
		FRopeWrappingPhase::GetPathPointCenterlineWorld(
			SurfaceToVirtualMidpoint, SurfaceOffset)
			.Equals(FVector(16.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));

	BridgePoint.SurfaceWorld = FVector(18.0f, 0.0f, 0.0f);
	BridgePoint.DistanceFromLatch = 10.0f;
	FRopeWrapPathPoint SurfaceToBridgeMidpoint;
	FRopeWrappingPhase::InterpolateWrappingPathPoints(
		SurfacePoint, BridgePoint, /*SampleDistance=*/5.0f,
		SurfaceOffset, SurfaceToBridgeMidpoint);
	TestTrue(TEXT("surface-to-bridge interpolation preserves the bridge flag"),
		SurfaceToBridgeMidpoint.bBridge);
	TestFalse(TEXT("bridge interpolation does not imply virtual storage"),
		SurfaceToBridgeMidpoint.bVirtual);
	TestTrue(TEXT("surface-to-bridge interpolation round-trips the centerline"),
		FRopeWrappingPhase::GetPathPointCenterlineWorld(
			SurfaceToBridgeMidpoint, SurfaceOffset)
			.Equals(FVector(16.0f, 0.0f, 0.0f), KINDA_SMALL_NUMBER));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingSequentialPathBuildGuardsTest,
	"DynamicRope.Wrapping.SequentialPathBuildGuards",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingSequentialPathBuildGuardsTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = MakeMockTarget();
	FCapsuleCollider Capsule(
		FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Capsule };
	FRopeSimState Sim = RopeTest::MakeStraightRope(
		9, 160.0f, FVector(25, 0, 0), FVector(0, 1, 0));

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("arm");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(25, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(25, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	const FRopeWrapConfig Config = MakeTestWrapConfig();
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("SequentialPathBuildGuardsTest"), true,
		/*bHasGuidePlaneNormal*/ true, /*GuidePlaneNormal*/ FVector::ZAxisVector };

	FRopeWrappingPhase AnchorFailureWrapping;
	TestTrue(TEXT("anchor failure fixture begins"),
		AnchorFailureWrapping.Begin(Latch, 0.16f, Sim, Ctx));
	TestFalse(TEXT("fixture uses the sequential path"),
		AnchorFailureWrapping.State.bPathUsesPoseSpaceIsland);

	// Shrinking the simulation after Begin makes the next sampled path point lack a valid
	// rope node. The failed append must leave only the previously processed path/anchor pair.
	FRopeSimState TruncatedSim = Sim;
	TruncatedSim.Positions.SetNum(1);
	TruncatedSim.PrevPositions.SetNum(1);
	TruncatedSim.InvMass.SetNum(1);
	for (int32 Iteration = 0;
		Iteration < 64 && AnchorFailureWrapping.State.bPathBuildActive;
		++Iteration)
	{
		AnchorFailureWrapping.AdvancePathBuild(TruncatedSim, Ctx);
	}

	TestFalse(TEXT("anchor failure terminates the path build"),
		AnchorFailureWrapping.State.bPathBuildActive);
	TestTrue(TEXT("anchor failure records a failed build"),
		AnchorFailureWrapping.State.bPathBuildFailed);
	TestEqual(TEXT("anchor failure records its reason"),
		AnchorFailureWrapping.State.PathBuildFailureReason,
		FString(TEXT("SequentialAnchorBuildFailed")));
	TestEqual(TEXT("unprocessed path points are rolled back"),
		AnchorFailureWrapping.State.Path.Num(), 1);
	TestEqual(TEXT("the previous anchor remains valid"),
		AnchorFailureWrapping.State.Anchors.Num(), 1);
	TestEqual(TEXT("processed path count matches the retained path"),
		AnchorFailureWrapping.State.LastAnchoredPathPointCount,
		AnchorFailureWrapping.State.Path.Num());

	FRopeWrappingPhase StepLimitWrapping;
	TestTrue(TEXT("step limit fixture begins"),
		StepLimitWrapping.Begin(Latch, 0.16f, Sim, Ctx));
	const float StepSize = FMath::Max(1.0f, Sim.SegmentLength * 0.5f);
	const int32 MaxIntegrationStepCount =
		FMath::Max(32, StepLimitWrapping.State.NumTailNodes * 16);
	StepLimitWrapping.State.PathSweepDistance =
		StepSize * static_cast<float>(MaxIntegrationStepCount);
	StepLimitWrapping.AdvancePathBuild(Sim, Ctx);

	TestFalse(TEXT("integration limit terminates the path build"),
		StepLimitWrapping.State.bPathBuildActive);
	TestTrue(TEXT("integration limit records a failed build"),
		StepLimitWrapping.State.bPathBuildFailed);
	TestEqual(TEXT("integration limit records its reason"),
		StepLimitWrapping.State.PathBuildFailureReason,
		FString(TEXT("SequentialIntegrationStepLimitExceeded")));
	return true;
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

	const FRopeWrapConfig Config = MakeTestWrapConfig();
	// Collider 장축 자동 추론은 제거됐으므로 테스트가 의도한 캡슐 Z축을 가이드 평면으로 명시한다.
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, /*GuidePlaneNormal*/ FVector::ZAxisVector };

	FRopeWrappingPhase Wrapping;
	TestTrue(TEXT("wrapping begins on capsule"), Wrapping.Begin(Latch, 0.16f, Sim, Ctx));

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

	const FRopeWrapConfig Config = MakeTestWrapConfig();
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true };

	FRopeWrappingPhase Wrapping;
	Wrapping.State.SecondarySeedAnchors.Add(Secondary);
	TestTrue(TEXT("wrapping begins with a secondary seed"), Wrapping.Begin(Latch, 0.16f, Sim, Ctx));

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
	const FRopeWrapState Seed = Wrapping.BuildCommitSeed(Sim);
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

// 감김 축 소스 설정:
// - CaptureTravelPlane: 진행 평면 normal을 캡처 접촉 영역 중심에 배치한다(Composite용).
// - BoneCenteredGuidePlane(기본): 같은 normal을 latch 본 위치에 배치한다(Assisted single bone용).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingAxisSourceTest,
	"DynamicRope.Wrapping.AxisSourceConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingAxisSourceTest::RunTest(const FString& Parameters)
{
	// 가이드 평면 normal = Y. 이 테스트에는 캡처 스냅샷이 없으므로 두 모드 모두 mock 본 원점을 쓰며,
	// 기본 모드가 폐기된 collider 형상 축(Z)으로 돌아가지 않는 것도 함께 검증한다.
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

	FRopeWrapConfig CaptureConfig = MakeTestWrapConfig();
	CaptureConfig.WrappingAxisSource = ERopeWrappingAxisSource::CaptureTravelPlane;
	const FRopeWrappingPhase::FContext CaptureCtx{ CaptureConfig, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, GuidePlaneNormal };

	FRopeWrappingPhase CaptureWrapping;
	TestTrue(TEXT("wrapping begins with capture travel-plane axis"),
		CaptureWrapping.Begin(Latch, 0.16f, Sim, CaptureCtx));
	TestTrue(FString::Printf(TEXT("CaptureTravelPlane axis follows the guide plane normal (dir=%s)"),
			*CaptureWrapping.State.PathAxisDirection.ToString()),
		FMath::Abs(FVector::DotProduct(CaptureWrapping.State.PathAxisDirection, GuidePlaneNormal)) > 0.99f);

	FRopeWrapConfig DefaultConfig = MakeTestWrapConfig();
	const FRopeWrappingPhase::FContext DefaultCtx{ DefaultConfig, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, GuidePlaneNormal };

	FRopeWrappingPhase DefaultWrapping;
	TestTrue(TEXT("wrapping begins with default axis source"),
		DefaultWrapping.Begin(Latch, 0.16f, Sim, DefaultCtx));
	TestTrue(FString::Printf(TEXT("BoneCenteredGuidePlane follows the guide plane normal (dir=%s)"),
			*DefaultWrapping.State.PathAxisDirection.ToString()),
		FMath::Abs(FVector::DotProduct(DefaultWrapping.State.PathAxisDirection, GuidePlaneNormal)) > 0.99f);
	TestTrue(FString::Printf(TEXT("BoneCenteredGuidePlane uses the latch bone origin (origin=%s)"),
			*DefaultWrapping.State.PathAxisOrigin.ToString()),
		DefaultWrapping.State.PathAxisOrigin.Equals(FVector::ZeroVector, 0.1f));
	return true;
}

// 캡처 스냅샷 소비(진행 방향 기반 wrap 3단계, CaptureTravelPlane 한정):
// - 축 origin = latch 본 위치가 아니라 접촉 영역 중심(RegionCenter) — 양다리에서 쌍의 중심 기준 반경.
// - winding 기준 = latch tangent가 아니라 캡처 속도 — 감기 시작 방향이 실제 운동 방향과 일치.
// - 스냅샷이 없으면 종전(본 위치 origin, tangent winding)으로 폴백.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingCaptureTravelPlaneTest,
	"DynamicRope.Wrapping.CaptureTravelPlaneUsesCaptureFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingCaptureTravelPlaneTest::RunTest(const FString& Parameters)
{
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

	// 캡처 스냅샷: 접촉 중심은 본 위치(원점)와 다른 (0,0,30), 캡처 속도는 +X.
	FRopeCaptureTravelFrame Frame;
	Frame.bValid = true;
	Frame.RegionCenter = FVector(0, 0, 30);
	Frame.AverageVelocity = FVector(100, 0, 0);

	FRopeWrapConfig CaptureConfig = MakeTestWrapConfig();
	CaptureConfig.WrappingAxisSource = ERopeWrappingAxisSource::CaptureTravelPlane;
	const FVector GuidePlaneNormal(0, 1, 0);
	const FRopeWrappingPhase::FContext FrameCtx{ CaptureConfig, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, GuidePlaneNormal, &Frame };

	FRopeWrappingPhase FrameWrapping;
	TestTrue(TEXT("wrapping begins with a capture frame"),
		FrameWrapping.Begin(Latch, 0.16f, Sim, FrameCtx));
	TestTrue(FString::Printf(TEXT("axis origin sits at the contact region center (origin=%s)"),
			*FrameWrapping.State.PathAxisOrigin.ToString()),
		FrameWrapping.State.PathAxisOrigin.Equals(FVector(0, 0, 30), 0.1f));
	// latch tangent(+Y)는 원주와 수직이라 종전 규칙으로는 부호가 못 정해지는 배치 — 속도(+X)가
	// 기준이 됐을 때만 감기 시작 방향이 +X 쪽을 향한다.
	TestTrue(FString::Printf(TEXT("winding starts along the capture velocity (circ=%s)"),
			*FrameWrapping.State.PathCircumferenceDir.ToString()),
		FVector::DotProduct(FrameWrapping.State.PathCircumferenceDir, FVector(1, 0, 0)) > 0.1f);

	// 스냅샷이 없으면 origin은 종전대로 latch 본 위치(mock identity = 원점).
	const FRopeWrappingPhase::FContext NoFrameCtx{ CaptureConfig, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, GuidePlaneNormal };

	FRopeWrappingPhase NoFrameWrapping;
	TestTrue(TEXT("wrapping begins without a capture frame"),
		NoFrameWrapping.Begin(Latch, 0.16f, Sim, NoFrameCtx));
	TestTrue(FString::Printf(TEXT("axis origin falls back to the bone location (origin=%s)"),
			*NoFrameWrapping.State.PathAxisOrigin.ToString()),
		NoFrameWrapping.State.PathAxisOrigin.Equals(FVector::ZeroVector, 0.1f));
	return true;
}

// 갭 브리징(진행 방향 기반 wrap 4단계) — 양다리 축약 기하: 나란히 선 두 캡슐(x=±30, r=12) 쌍을
// 진행 평면 축(쌍의 중심을 지나는 Z)으로 감는다. 브리징이 없으면 이 경로는 두 지점에서 반드시
// 죽는다: 다리 사이 허공에서 투영이 (i) 먼 표면으로 스냅해 경로가 골짜기로 말려들거나
// (ii) 실패해 빌드가 끊긴다. 계약:
// ① 경로가 실패 없이 완주하고, ② chord(bBridge) 경로점이 존재하며 그 노드들은 앵커가 없고,
// ③ 앵커가 양쪽 캡슐 표면 모두에 생기며, ④ 누적 감싼 각도가 쌍 순회를 증명한다(>270°).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingGapBridgeTest,
	"DynamicRope.Wrapping.GapBridgeWrapsCapsulePair",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingGapBridgeTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = MakeMockTarget();
	// 두 "다리": 같은 mesh, 같은 본 이름(비스켈레탈 mock은 본 그래프가 없어 후보가 현재 본뿐이므로
	// 본 이름을 공유시킨다 — 이 테스트의 대상은 갭 횡단이지 본 전환이 아니다).
	FCapsuleCollider LegA(FVector(30, 0, -50), FVector(30, 0, 50), 12.0f, FName("legs"), Mesh);
	FCapsuleCollider LegB(FVector(-30, 0, -50), FVector(-30, 0, 50), 12.0f, FName("legs"), Mesh);
	TArray<IRopeCollider*> Colliders = { &LegA, &LegB };

	// 로프 280cm(15노드) — 쌍의 헐 둘레(~195cm)를 한 바퀴 이상 감을 길이.
	FRopeSimState Sim = RopeTest::MakeStraightRope(15, 280.0f, FVector(42, 0, 0), FVector(0, 1, 0));

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("legs");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(42, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(42, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	// 캡처 스냅샷: 축 origin = 쌍의 중심, winding = 캡처 속도(+Y).
	FRopeCaptureTravelFrame Frame;
	Frame.bValid = true;
	Frame.RegionCenter = FVector::ZeroVector;
	Frame.AverageVelocity = FVector(0, 100, 0);

	FRopeWrapConfig Config = MakeTestWrapConfig();
	Config.WrappingAxisSource = ERopeWrappingAxisSource::CaptureTravelPlane;
	// chord는 60cm지만 브리지 경로는 축 반경(~32) 원호를 따라 우회한다(~77cm) — 여유를 둔다.
	Config.WrappingMaxGapBridgeDistance = 120.0f;
	// 이 테스트는 진행 평면 안의 순회만 검증한다 — 축 방향 나선 상승이 캡슐 꼭대기 밖으로 새지 않게.
	Config.WrappingHelixPitchScale = 0.0f;
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, /*GuidePlaneNormal*/ FVector(0, 0, 1), &Frame };

	FRopeWrappingPhase Wrapping;
	TestTrue(TEXT("wrapping begins on the pair"), Wrapping.Begin(Latch, 0.16f, Sim, Ctx));

	for (int32 Iteration = 0; Iteration < 512 && Wrapping.State.bPathBuildActive; ++Iteration)
	{
		Wrapping.AdvancePathBuild(Sim, Ctx);
	}

	// ① 브리징 덕에 실패 없이 완주.
	TestTrue(TEXT("path build completes around the pair"), Wrapping.State.bPathBuildComplete);
	TestTrue(TEXT("path build did not fail"), !Wrapping.State.bPathBuildFailed);

	// ② chord 경로점 존재 + 그 노드에는 앵커가 없다.
	int32 BridgePointCount = 0;
	for (const FRopeWrapPathPoint& Point : Wrapping.State.Path)
	{
		if (Point.bBridge)
		{
			++BridgePointCount;
		}
	}
	TestTrue(TEXT("bridge (chord) path points exist"), BridgePointCount > 0);
	TestEqual(TEXT("bridge nodes carry no anchors"),
		Wrapping.State.Anchors.Num() + BridgePointCount, Wrapping.State.Path.Num());
	for (const FRopeSurfaceAnchor& Anchor : Wrapping.State.Anchors)
	{
		const int32 PathIndex = Anchor.NodeIndex - Latch.NodeIndex;
		TestTrue(FString::Printf(TEXT("anchored path point %d is on-surface"), PathIndex),
			Wrapping.State.Path.IsValidIndex(PathIndex) && !Wrapping.State.Path[PathIndex].bBridge);
	}

	// ③ 양쪽 캡슐 표면 모두에 앵커가 생겼다(identity 트랜스폼 — 로컬 X로 바로 판별).
	bool bAnchorOnLegA = false;
	bool bAnchorOnLegB = false;
	for (const FRopeSurfaceAnchor& Anchor : Wrapping.State.Anchors)
	{
		bAnchorOnLegA |= Anchor.LocalSurfacePosition.X > 15.0f;
		bAnchorOnLegB |= Anchor.LocalSurfacePosition.X < -15.0f;
	}
	TestTrue(TEXT("anchors reached the near leg"), bAnchorOnLegA);
	TestTrue(TEXT("anchors reached the far leg"), bAnchorOnLegB);

	// ④ 누적 감싼 각도가 쌍 순회를 증명한다.
	float AngleDeg = 0.0f;
	TestTrue(TEXT("wrapped angle computable"), Wrapping.ComputeWrappedAngleAtLastBuiltPoint(Sim, Ctx, AngleDeg));
	TestTrue(FString::Printf(TEXT("accumulated angle circles the pair (%.0f deg)"), AngleDeg),
		AngleDeg > 270.0f);

	// ⑤ 형상 기준 커버리지(5단계): 쌍을 완주했으니 축 둘레에 큰 공백이 없어야 한다.
	float CoverageDeg = 0.0f;
	TestTrue(TEXT("enclosure coverage computable"), Wrapping.ComputeWrapEnclosureCoverage(CoverageDeg));
	TestTrue(FString::Printf(TEXT("pair wrap leaves no escape gap (coverage=%.0f deg)"), CoverageDeg),
		CoverageDeg > 300.0f);

	// ⑥ 실제 centerline arc-length 재샘플링 계약: 경로 좌표는 SegmentLength 배수이고,
	// 인접 node의 chord는 그 사이 polyline arc보다 길 수 없으므로 SegmentLength를 초과하지 않는다.
	const float SegmentLength = Sim.SegmentLength;
	const float CenterlineOffset = Ctx.SurfaceOffset;
	for (int32 PathIndex = 0; PathIndex < Wrapping.State.Path.Num(); ++PathIndex)
	{
		const FRopeWrapPathPoint& Point = Wrapping.State.Path[PathIndex];
		const float ExpectedDistance = static_cast<float>(PathIndex) * SegmentLength;
		TestTrue(FString::Printf(TEXT("path point %d uses arc-length coordinate (%.2f vs %.2f)"),
				PathIndex, Point.DistanceFromLatch, ExpectedDistance),
			FMath::IsNearlyEqual(Point.DistanceFromLatch, ExpectedDistance, 0.01f));
		if (PathIndex > 0)
		{
			const FRopeWrapPathPoint& PreviousPoint = Wrapping.State.Path[PathIndex - 1];
			const FVector PreviousCenter = PreviousPoint.SurfaceWorld +
				PreviousPoint.NormalWorld * CenterlineOffset;
			const FVector CurrentCenter = Point.SurfaceWorld +
				Point.NormalWorld * CenterlineOffset;
			const float ChordDistance = FVector::Dist(PreviousCenter, CurrentCenter);
			TestTrue(FString::Printf(TEXT("path chord %d->%d does not exceed one segment (%.2f <= %.2f)"),
					PathIndex - 1, PathIndex, ChordDistance, SegmentLength),
				ChordDistance <= SegmentLength + 0.05f);
		}
	}
	const float LastSampleDistance =
		static_cast<float>(Wrapping.State.Path.Num() - 1) * SegmentLength;
	TestTrue(TEXT("actual centerline distance reaches the last emitted sample"),
		Wrapping.State.PathCurrentDistance + KINDA_SMALL_NUMBER >= LastSampleDistance);
	TestTrue(TEXT("nominal sweep distance advances independently"),
		Wrapping.State.PathSweepDistance > 0.0f);

	return true;
}

// 군집 중심 축 보정(양다리 PIE 실측 수정): 캡처는 첫 다리에 닿는 즉시 일어나 RegionCenter가
// 한쪽 다리 위에 있다 — 그대로 축을 세우면 필드가 그 다리만 나선으로 돌고(축이 대상 안이면
// winding 이탈 관문도 침묵) 반대쪽으로 못 건너간다(실측: 한 다리 1725°). 브리징 모드에서는
// 같은 mesh의 근방 collider 중심 군집이 origin을 쌍의 중심으로 옮겨, 접촉이 한쪽에서 시작해도
// 경로가 쌍을 순회해야 한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingClusterAxisOriginTest,
	"DynamicRope.Wrapping.ClusterAxisRecentersFirstContactCapture",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingClusterAxisOriginTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = MakeMockTarget();
	FCapsuleCollider LegA(FVector(30, 0, -50), FVector(30, 0, 50), 12.0f, FName("legs"), Mesh);
	FCapsuleCollider LegB(FVector(-30, 0, -50), FVector(-30, 0, 50), 12.0f, FName("legs"), Mesh);
	TArray<IRopeCollider*> Colliders = { &LegA, &LegB };

	FRopeSimState Sim = RopeTest::MakeStraightRope(15, 280.0f, FVector(42, 0, 0), FVector(0, 1, 0));

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("legs");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(42, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(42, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	// 실전 캡처 재현: 접촉은 첫 다리(LegA)뿐 — RegionCenter가 그 표면 위에 있다.
	FRopeCaptureTravelFrame Frame;
	Frame.bValid = true;
	Frame.RegionCenter = FVector(42, 0, 0);
	Frame.AverageVelocity = FVector(0, 100, 0);

	FRopeWrapConfig Config = MakeTestWrapConfig();
	Config.WrappingAxisSource = ERopeWrappingAxisSource::CaptureTravelPlane;
	Config.WrappingMaxGapBridgeDistance = 120.0f;
	Config.WrappingHelixPitchScale = 0.0f;
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, /*GuidePlaneNormal*/ FVector(0, 0, 1), &Frame };

	FRopeWrappingPhase Wrapping;
	TestTrue(TEXT("wrapping begins from a one-leg capture"),
		Wrapping.Begin(Latch, 0.16f, Sim, Ctx));

	// 축 origin이 첫 접촉점(42,0,0)이 아니라 collider 군집 중심(두 캡슐 중점 = x 0)으로 옮겨진다.
	TestTrue(FString::Printf(TEXT("axis origin recentered between the legs (origin=%s)"),
			*Wrapping.State.PathAxisOrigin.ToString()),
		FMath::Abs(Wrapping.State.PathAxisOrigin.X) < 5.0f &&
		FMath::Abs(Wrapping.State.PathAxisOrigin.Y) < 5.0f);

	for (int32 Iteration = 0; Iteration < 512 && Wrapping.State.bPathBuildActive; ++Iteration)
	{
		Wrapping.AdvancePathBuild(Sim, Ctx);
	}
	TestTrue(TEXT("path build completes around the pair"), Wrapping.State.bPathBuildComplete);

	bool bAnchorOnLegA = false;
	bool bAnchorOnLegB = false;
	for (const FRopeSurfaceAnchor& Anchor : Wrapping.State.Anchors)
	{
		bAnchorOnLegA |= Anchor.LocalSurfacePosition.X > 15.0f;
		bAnchorOnLegB |= Anchor.LocalSurfacePosition.X < -15.0f;
	}
	TestTrue(TEXT("anchors reached the contacted leg"), bAnchorOnLegA);
	TestTrue(TEXT("anchors reached the uncontacted far leg"), bAnchorOnLegB);
	return true;
}

// 감는 양 상한(WrappingMaxWrapAngleDeg): 긴 로프가 대상을 여러 바퀴 감아 들어가는 대신, 누적
// 감싼 각도가 목표에 닿으면 경로가 *성공*으로 조기 마감되고(NumTailNodes = 빌드된 경로 길이),
// 경로 밖 남는 로프는 front 모션이 끌지 않는다(동결 유지 → 커밋 후 자유). PIE 실측의
// 3000~4400° 문어발 나선 방지책.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingWrapAngleCapTest,
	"DynamicRope.Wrapping.WrapAngleCapStopsSpiral",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingWrapAngleCapTest::RunTest(const FString& Parameters)
{
	// 캡슐 r=25 + 로프 320cm(17노드): 상한이 없으면 두 바퀴 이상 나선을 만들 길이다.
	USceneComponent* Mesh = MakeMockTarget();
	FCapsuleCollider Capsule(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Capsule };

	FRopeSimState Sim = RopeTest::MakeStraightRope(17, 320.0f, FVector(25, 0, 0), FVector(0, 1, 0));

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("arm");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(25, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(25, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	FRopeWrapConfig Config = MakeTestWrapConfig();
	Config.WrappingMaxWrapAngleDeg = 360.0f;
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true };

	FRopeWrappingPhase Wrapping;
	TestTrue(TEXT("wrapping begins"), Wrapping.Begin(Latch, 0.16f, Sim, Ctx));

	for (int32 Iteration = 0; Iteration < 512 && Wrapping.State.bPathBuildActive; ++Iteration)
	{
		Wrapping.AdvancePathBuild(Sim, Ctx);
	}

	// 상한 도달 = 실패가 아니라 성공 마감. 경로는 로프보다 짧고, front/커밋 목표 거리의 기준인
	// NumTailNodes가 경로 길이로 줄어 있어야 한다.
	TestTrue(TEXT("capped path finishes as success"), Wrapping.State.bPathBuildComplete);
	TestTrue(TEXT("capped path did not fail"), !Wrapping.State.bPathBuildFailed);
	TestTrue(FString::Printf(TEXT("path stops short of the rope (%d/17 points)"), Wrapping.State.Path.Num()),
		Wrapping.State.Path.Num() < 17);
	TestEqual(TEXT("NumTailNodes shrinks to the built path"),
		Wrapping.State.NumTailNodes, Wrapping.State.Path.Num());
	TestEqual(TEXT("every capped path point is anchored"),
		Wrapping.State.Anchors.Num(), Wrapping.State.Path.Num());

	// 마감 시점 각도는 목표를 갓 넘긴 값(목표 ~ 목표+스텝각)이어야 한다 — 여러 바퀴 나선 금지.
	float AngleDeg = 0.0f;
	TestTrue(TEXT("wrapped angle computable"), Wrapping.ComputeWrappedAngleAtLastBuiltPoint(Sim, Ctx, AngleDeg));
	TestTrue(FString::Printf(TEXT("angle stops just past the cap (%.0f deg)"), AngleDeg),
		AngleDeg >= 360.0f && AngleDeg < 460.0f);

	// 경로 밖 남는 로프는 front 모션이 끌지 않는다(동결 유지 — 커밋 후 자유 구간).
	FRopeNodeOverrideFrame Frame;
	for (int32 Step = 0; Step < 64; ++Step)
	{
		Frame.Reset();
		Wrapping.ApplyFrontMotion(Sim, 0.05f, Ctx, Frame);
	}
	const int32 LastDrivenNode = Latch.NodeIndex + Wrapping.State.NumTailNodes - 1;
	TestTrue(TEXT("last path node is front-driven"),
		Frame.Flags.IsValidIndex(LastDrivenNode) &&
		(Frame.Flags[LastDrivenNode] & RopeNodeOverride::Position) != 0);
	TestTrue(TEXT("leftover rope beyond the cap stays untouched"),
		Frame.Flags.IsValidIndex(16) &&
		(Frame.Flags[LastDrivenNode + 1] & RopeNodeOverride::Position) == 0 &&
		(Frame.Flags[16] & RopeNodeOverride::Position) == 0);
	return true;
}

// 형상 기준 묶임 척도(진행 방향 기반 wrap 5단계, ComputeWrapEnclosureCoverage):
// 같은 캡슐에서 로프 길이만 달리해 — 만감김(~356°)은 커버리지가 360°에 수렴하고,
// 반쪽 훅(경로 ~132°)은 축 둘레 반대편이 통째로 비어 커버리지가 그만큼 낮게 나온다.
// CommitMinWrapCoverageDeg 관문이 이 값으로 "둘러싸임 vs 걸침"을 가른다는 계약.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingEnclosureCoverageTest,
	"DynamicRope.Wrapping.EnclosureCoverageSeparatesHookFromWrap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingEnclosureCoverageTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = MakeMockTarget();
	FCapsuleCollider Capsule(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Capsule };

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("arm");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(25, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(25, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	const FRopeWrapConfig Config = MakeTestWrapConfig();
	// 커버리지 비교의 기준 축을 캡슐 Z축으로 고정한다(형상 축 자동 추론에 의존하지 않음).
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
		/*bHasGuidePlaneNormal*/ true, /*GuidePlaneNormal*/ FVector::ZAxisVector };

	auto BuildAndMeasure = [&](int32 NumNodes, float RopeLength, float& OutCoverageDeg) -> bool
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(NumNodes, RopeLength, FVector(25, 0, 0), FVector(0, 1, 0));
		FRopeWrappingPhase Wrapping;
		if (!Wrapping.Begin(Latch, 0.16f, Sim, Ctx))
		{
			return false;
		}
		for (int32 Iteration = 0; Iteration < 256 && Wrapping.State.bPathBuildActive; ++Iteration)
		{
			Wrapping.AdvancePathBuild(Sim, Ctx);
		}
		return Wrapping.State.bPathBuildComplete &&
			Wrapping.ComputeWrapEnclosureCoverage(OutCoverageDeg);
	};

	// 만감김: 160cm(9노드) ≈ 356° — 공백이 스텝 각 수준이라 커버리지가 360°에 수렴.
	float FullCoverageDeg = 0.0f;
	TestTrue(TEXT("full wrap builds and measures"), BuildAndMeasure(9, 160.0f, FullCoverageDeg));
	TestTrue(FString::Printf(TEXT("full wrap coverage nears 360 (%.0f deg)"), FullCoverageDeg),
		FullCoverageDeg > 300.0f);

	// 반쪽 훅: 80cm(5노드) — 경로가 ~1/3바퀴에서 끝나 반대편이 통째로 빈다.
	float HookCoverageDeg = 0.0f;
	TestTrue(TEXT("hook wrap builds and measures"), BuildAndMeasure(5, 80.0f, HookCoverageDeg));
	TestTrue(FString::Printf(TEXT("hook coverage stays low (%.0f deg)"), HookCoverageDeg),
		HookCoverageDeg < 220.0f);
	TestTrue(TEXT("coverage separates hook from wrap"), FullCoverageDeg > HookCoverageDeg + 90.0f);
	return true;
}

// 도달 모드 × 결착 모델 조합 제약(RopeWrapModes — 에디터 보정·던지기 진입이 공용 소비하는 단일
// 소스): ①FullSimulation·②AssistedJudged = BareWrap 전용, ③GuaranteedWrap = Pierce/Cinch 전용.
// 보정 방향도 계약이다: ①②의 팁 결착은 BareWrap으로, ③의 BareWrap은 Pierce로 승격.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeResolveModeEngagementTest,
	"DynamicRope.Wrapping.ResolveModeEngagementConstraint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeResolveModeEngagementTest::RunTest(const FString& Parameters)
{
	const ERopeWrapResolveMode Sim = ERopeWrapResolveMode::FullSimulation;
	const ERopeWrapResolveMode Judged = ERopeWrapResolveMode::AssistedJudged;
	const ERopeWrapResolveMode Guaranteed = ERopeWrapResolveMode::GuaranteedWrap;
	const ERopeTipEngagement Bare = ERopeTipEngagement::BareWrap;
	const ERopeTipEngagement Pierce = ERopeTipEngagement::Pierce;
	const ERopeTipEngagement Cinch = ERopeTipEngagement::Cinch;

	// 허용 매트릭스.
	TestTrue(TEXT("FullSim x BareWrap allowed"), RopeWrapModes::IsEngagementAllowed(Sim, Bare));
	TestTrue(TEXT("Assisted x BareWrap allowed"), RopeWrapModes::IsEngagementAllowed(Judged, Bare));
	TestTrue(TEXT("Guaranteed x Pierce allowed"), RopeWrapModes::IsEngagementAllowed(Guaranteed, Pierce));
	TestTrue(TEXT("Guaranteed x Cinch allowed"), RopeWrapModes::IsEngagementAllowed(Guaranteed, Cinch));
	TestTrue(TEXT("FullSim x Pierce forbidden"), !RopeWrapModes::IsEngagementAllowed(Sim, Pierce));
	TestTrue(TEXT("FullSim x Cinch forbidden"), !RopeWrapModes::IsEngagementAllowed(Sim, Cinch));
	TestTrue(TEXT("Assisted x Pierce forbidden"), !RopeWrapModes::IsEngagementAllowed(Judged, Pierce));
	TestTrue(TEXT("Assisted x Cinch forbidden"), !RopeWrapModes::IsEngagementAllowed(Judged, Cinch));
	TestTrue(TEXT("Guaranteed x BareWrap forbidden"), !RopeWrapModes::IsEngagementAllowed(Guaranteed, Bare));

	// 보정 방향: 유효 조합은 그대로, 무효 조합은 모드에 맞는 기본값으로.
	TestTrue(TEXT("valid combo passes through"),
		RopeWrapModes::ClampEngagement(Guaranteed, Cinch) == Cinch);
	TestTrue(TEXT("tip engagement under Assisted clamps to BareWrap"),
		RopeWrapModes::ClampEngagement(Judged, Pierce) == Bare);
	TestTrue(TEXT("tip engagement under FullSim clamps to BareWrap"),
		RopeWrapModes::ClampEngagement(Sim, Cinch) == Bare);
	TestTrue(TEXT("BareWrap under Guaranteed promotes to Pierce"),
		RopeWrapModes::ClampEngagement(Guaranteed, Bare) == Pierce);
	return true;
}

// wrap 대상 게이트(URopeComponent::CanWrapTarget)가 감김 *경로 빌드*에도 적용되는지 잠근다.
// 과거엔 조준/preview/판정만 게이트를 태우고 Wrapping 경로는 콜라이더 스냅샷을 그대로 받아,
// "조준은 거부한 대상 위로 경로가 깔리는" 불일치가 있었다. 관문은 컴포넌트 경계의 필터 하나다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapTargetGateFilterTest,
	"DynamicRope.Wrapping.CanWrapTargetGateFiltersWrappingColliders",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapTargetGateFilterTest::RunTest(const FString& Parameters)
{
	USceneComponent* AllowedMesh = MakeMockTarget();
	USceneComponent* DeniedMesh = MakeMockTarget();

	FCapsuleCollider AllowedCol(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), AllowedMesh);
	FCapsuleCollider DeniedCol(FVector(100, 0, -50), FVector(100, 0, 50), 25.0f, FName("leg"), DeniedMesh);
	// 귀속 없는 collider(월드 정적 기하) — 감김 대상이 될 수 없으므로 게이트 대상이 아니다.
	FCapsuleCollider WorldGeo(FVector(0, 200, -50), FVector(0, 200, 50), 25.0f, NAME_None, nullptr);

	const TArray<IRopeCollider*> In = { &AllowedCol, &DeniedCol, &WorldGeo };
	TArray<IRopeCollider*> Out;

	// ① 기본 게이트(전부 허용) = 입력 불변 — 오버라이드하지 않은 로프의 동작 보존 계약.
	RopeWrapTargets::FilterWrappableColliders(In,
		[](const USceneComponent*, FName) { return true; }, Out);
	TestEqual(TEXT("게이트 기본값이면 collider 집합이 그대로다"), Out.Num(), In.Num());

	// ② 특정 대상 거부 → 그 collider만 빠진다.
	RopeWrapTargets::FilterWrappableColliders(In,
		[DeniedMesh](const USceneComponent* Mesh, FName) { return Mesh != DeniedMesh; }, Out);
	TestEqual(TEXT("거부된 대상 1개가 빠진다"), Out.Num(), 2);
	TestTrue(TEXT("허용 대상은 남는다"), Out.Contains(&AllowedCol));
	TestFalse(TEXT("거부 대상은 감김 경로에 보이지 않는다"), Out.Contains(&DeniedCol));
	TestTrue(TEXT("귀속 없는 월드 기하는 게이트와 무관하게 남는다"), Out.Contains(&WorldGeo));

	// ③ 전부 거부해도 귀속 없는 표면 기하는 유지된다(로프가 벽을 통과하면 안 된다).
	RopeWrapTargets::FilterWrappableColliders(In,
		[](const USceneComponent*, FName) { return false; }, Out);
	TestEqual(TEXT("전부 거부해도 귀속 없는 기하는 남는다"), Out.Num(), 1);
	TestTrue(TEXT("남은 것은 월드 기하"), Out.Contains(&WorldGeo));

	return true;
}

#endif
