// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Unit tests for the wrapping phase. They drive a surface vector field path build to completion with no
// world, using the static wrap target path, meaning a virtual bone name plus a component transform.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrappingPhase.h"
#include "Collision/RopeCollider.h"
// FRopeConvexCollider (Log convex in ShapeExtentAxis test)
#include "Collision/RopeStaticCollider.h"
#include "Core/RopeWrapTarget.h"
#include "Components/SceneComponent.h"
#include "RopeTestHelpers.h"

namespace
{
	// A mock target that uses the static wrap target path directly. ResolveBindingWorld returns the
	// component's identity transform for a virtual bone name with no warning, which makes bone-local space
	// equal world space.
	USceneComponent* MakeMockTarget()
	{
		return NewObject<USceneComponent>();
	}

	// The struct default became automatic, which is resolved only at the component boundary, so a direct
	// call with no world pins the previous default explicitly; the test geometry, including the reach
	// calculation, is based on that value.
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

	// Verifies that a surface vector field path build completes all the way round a capsule, that the
	// anchors land on the surface, and that the wrapped angle integrated during the build matches the
	// single-axis helix formula. It is the regression contract pinning that a single bone, with no
	// transition, still matches the original formula after the rolling axis was introduced.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingCapsuleAngleTest,
	"DynamicRope.Wrapping.SurfaceVectorFieldCapsuleWrapAngle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingCapsuleAngleTest::RunTest(const FString& Parameters)
{
	// A capsule aligned to Z with a radius of 25, giving a circumference of about 157 cm. A 160 cm rope of
	// nine nodes wraps roughly one full turn.
	USceneComponent* Mesh = MakeMockTarget();
	FCapsuleCollider Capsule(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Capsule };

	// The rope extends from the latch point on the positive X surface along positive Y, around the
	// circumference.
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
	// Automatic inference of a collider's long axis was removed, so the test states the capsule's Z axis as
	// the guide plane explicitly.
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

	// Every anchor sits on the capsule surface, at a radius of 25 from the axis. With an identity transform
	// it can be measured directly as a local XY distance.
	for (const FRopeSurfaceAnchor& Anchor : Wrapping.State.Anchors)
	{
		const float RadialDistance = static_cast<float>(
			FVector2D(Anchor.LocalSurfacePosition.X, Anchor.LocalSurfacePosition.Y).Size());
		TestTrue(FString::Printf(TEXT("anchor %d on capsule surface (r=%.1f)"), Anchor.NodeIndex, RadialDistance),
			FMath::IsNearlyEqual(RadialDistance, 25.0f, 3.0f));
	}

	// The accumulated angle matches the single-axis helix formula, which is the distance over the pitch
	// correction divided by the radius, giving about 356 degrees.
	// The walk takes chord steps of half a segment, so it integrates slightly less per step than the
	// arc-based formula, by the arctangent difference, which is roughly five percent at this radius. The
	// tolerance accommodates that discretization error.
	float AngleDeg = 0.0f;
	TestTrue(TEXT("wrapped angle computable"), Wrapping.ComputeBuiltPathWrapAngle(Sim, Ctx, AngleDeg));
	const float PitchScale = Config.WrappingHelixPitchScale;
	const float ExpectedDeg = FMath::RadiansToDegrees(
		(160.0f / FMath::Sqrt(1.0f + PitchScale * PitchScale)) / 25.0f);
	TestTrue(FString::Printf(TEXT("accumulated angle matches helix formula (%.0f vs %.0f deg)"), AngleDeg, ExpectedDeg),
		FMath::IsNearlyEqual(AngleDeg, ExpectedDeg, 30.0f));
	return true;
}

	// Seed multiplexing: with secondary seed anchors loaded into the state before Begin,
	//  1) the path length is clamped to end before the first secondary node, which stops a path anchor and
	//     a secondary anchor both owning the same node;
	//  2) the front motion holds a secondary node against its own bone's anchor frame instead of dragging
	//     it along the path, and leaves the rope beyond it untouched;
	//  3) the commit seed carries both the path anchors and the secondary ones, since BeginWrap and Hold
	//     support a per-anchor bone and mesh.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingSecondarySeedTest,
	"DynamicRope.Wrapping.SecondarySeedAnchorsHoldAndCommit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingSecondarySeedTest::RunTest(const FString& Parameters)
{
	// The dominant target is the same capsule as the test above, on the bone named arm. The secondary target
	// is the bone named leg on a different mesh, seeded as though node 6 had contacted its surface point.
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

	// One: the path stops before the secondary node, so the tail node count covers nodes zero to five.
	TestEqual(TEXT("path length clamps before the secondary node"), Wrapping.State.NumTailNodes, 6);

	for (int32 Iteration = 0; Iteration < 256 && Wrapping.State.bPathBuildActive; ++Iteration)
	{
		Wrapping.AdvancePathBuild(Sim, Ctx);
	}
	TestTrue(TEXT("clamped path build completes"), Wrapping.State.bPathBuildComplete);
	TestEqual(TEXT("path anchors stop before the secondary node"), Wrapping.State.Anchors.Num(), 6);

	// Two: the front motion. After pushing the front to the end with a large enough delta, the secondary
	// node sits on its own anchor frame, at the surface plus the offset, and the nodes beyond it receive no
	// position write at all.
	FRopeNodeOverrideFrame Frame;
	for (int32 Step = 0; Step < 64; ++Step)
	{
		Frame.Reset();
		Wrapping.ApplyWrappingMotionOverrides(Sim, 0.05f, Ctx, Frame);
	}
	TestTrue(TEXT("secondary node receives a position override"),
		Frame.Flags.IsValidIndex(6) && (Frame.Flags[6] & RopeNodeOverride::Position) != 0);
	TestTrue(TEXT("secondary node held at its own anchor frame"),
		Frame.Positions.IsValidIndex(6) && Frame.Positions[6].Equals(FVector(101, 0, 0), 0.1f));
	TestTrue(TEXT("nodes beyond the secondary stay untouched"),
		Frame.Flags.IsValidIndex(8) &&
		(Frame.Flags[7] & RopeNodeOverride::Position) == 0 &&
		(Frame.Flags[8] & RopeNodeOverride::Position) == 0);
	Wrapping.ApplyWrappingKinematicMask(Sim, Frame);
	TestTrue(TEXT("path and secondary position overrides stay kinematic"),
		Frame.InvMass.IsValidIndex(6) &&
		Frame.InvMass[5] == 0.0f && Frame.InvMass[6] == 0.0f);
	TestTrue(TEXT("untouched rope beyond the secondary remains solver-owned"),
		Frame.InvMass.IsValidIndex(8) &&
		Frame.InvMass[7] > 0.0f && Frame.InvMass[8] > 0.0f);

	// Even when a secondary binding fails to resolve for one frame and its position override is missing, an
	// established anchor is still held at its current position, while the ordinary tail after it stays
	// solver-owned.
	Wrapping.State.SecondarySeedAnchors[0].Mesh.Reset();
	FRopeNodeOverrideFrame MissingBindingFrame;
	Wrapping.ApplyWrappingMotionOverrides(Sim, 0.0f, Ctx, MissingBindingFrame);
	TestTrue(TEXT("missing secondary binding emits no position override"),
		MissingBindingFrame.Flags.IsValidIndex(6) &&
		(MissingBindingFrame.Flags[6] & RopeNodeOverride::Position) == 0);
	Wrapping.ApplyWrappingKinematicMask(Sim, MissingBindingFrame);
	TestTrue(TEXT("missing-binding secondary anchor remains kinematic"),
		MissingBindingFrame.InvMass.IsValidIndex(6) && MissingBindingFrame.InvMass[6] == 0.0f);
	TestTrue(TEXT("tail after missing-binding anchor remains solver-owned"),
		MissingBindingFrame.InvMass.IsValidIndex(8) &&
		MissingBindingFrame.InvMass[7] > 0.0f && MissingBindingFrame.InvMass[8] > 0.0f);
	Wrapping.State.SecondarySeedAnchors[0].Mesh = SecondaryMesh;

	// Three: the commit seed carries the six path anchors plus the one secondary anchor, with the
	// secondary's bone and mesh preserved.
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

	// The wrap axis source setting:
	// - CaptureTravelPlane places the travel plane normal at the centre of the captured contact region,
	//   which composite wrapping uses.
	// - BoneCenteredGuidePlane, the default, places the same normal at the latch bone's location, which
	//   single-bone assisted wrapping uses.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingAxisSourceTest,
	"DynamicRope.Wrapping.AxisSourceConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingAxisSourceTest::RunTest(const FString& Parameters)
{
	// The guide plane normal is Y. CaptureTravelPlane, the explicit composite intent, follows the guide
	// plane. The default mode on a non-skeletal target now prefers the shape-extent axis - the capsule's
	// long axis Z - over the guide plane, so a static prop wraps about what its shape dictates regardless
	// of the throw direction.
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
	TestTrue(FString::Printf(TEXT("default mode prefers the shape-extent axis over the guide plane (dir=%s)"),
			*DefaultWrapping.State.PathAxisDirection.ToString()),
		FMath::Abs(FVector::DotProduct(DefaultWrapping.State.PathAxisDirection, FVector(0, 0, 1))) > 0.99f);
	TestTrue(FString::Printf(TEXT("BoneCenteredGuidePlane uses the latch bone origin (origin=%s)"),
			*DefaultWrapping.State.PathAxisOrigin.ToString()),
		DefaultWrapping.State.PathAxisOrigin.Equals(FVector::ZeroVector, 0.1f));
	return true;
}

	// The shape-extent axis for non-skeletal targets: with no guide plane, a log-shaped convex picks its
	// long axis however the component basis is oriented (the old component-basis rule tied on a top hit
	// and picked X by iteration order), and an end hit rejects the near-normal long axis and falls to the
	// larger cross axis.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingShapeExtentAxisTest,
	"DynamicRope.Wrapping.ShapeExtentAxis",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingShapeExtentAxisTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = MakeMockTarget();
	const FName LogBone(TEXT("log"));

	// A log-shaped convex: local bounds 60 x 200 x 60, long axis Y, at the origin.
	const auto MakeLogConvex = [&]() -> FRopeConvexCollider
	{
		TArray<FPlane> Planes;
		Planes.Add(FPlane(FVector(1, 0, 0), 30.0));
		Planes.Add(FPlane(FVector(-1, 0, 0), 30.0));
		Planes.Add(FPlane(FVector(0, 1, 0), 100.0));
		Planes.Add(FPlane(FVector(0, -1, 0), 100.0));
		Planes.Add(FPlane(FVector(0, 0, 1), 30.0));
		Planes.Add(FPlane(FVector(0, 0, -1), 30.0));
		FRopeConvexCollider Convex(MoveTemp(Planes),
			FBox(FVector(-30.0, -100.0, -30.0), FVector(30.0, 100.0, 30.0)));
		Convex.Bone = LogBone;
		Convex.SourceMesh = Mesh;
		return Convex;
	};

	// 1) A top hit (normal Z): the long axis Y must win. The old basis rule tied X and Y at
	// perpendicularity 0 and took X by iteration order.
	{
		FRopeConvexCollider Convex = MakeLogConvex();
		TArray<IRopeCollider*> Colliders = { &Convex };
		FRopeSimState Sim = RopeTest::MakeStraightRope(9, 160.0f, FVector(0, 0, 31), FVector(1, 0, 0));

		FRopeSurfaceAnchor Latch;
		Latch.NodeIndex = 0;
		Latch.Bone = LogBone;
		Latch.Mesh = Mesh;
		Latch.LocalSurfacePosition = FVector(0, 0, 30);
		Latch.LocalNormal = FVector(0, 0, 1);
		Latch.LocalTangent = FVector(1, 0, 0);
		Latch.StartWorldPosition = FVector(0, 0, 30);
		Latch.SurfaceOffset = 1.0f;

		FRopeWrapConfig Config = MakeTestWrapConfig();
		const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
			/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
			/*bHasGuidePlaneNormal*/ false, FVector::ZeroVector };

		FRopeWrappingPhase Wrapping;
		TestTrue(TEXT("wrapping begins on the log top hit"), Wrapping.Begin(Latch, 0.16f, Sim, Ctx));
		TestTrue(FString::Printf(TEXT("top hit picks the log's long axis Y (dir=%s)"),
				*Wrapping.State.PathAxisDirection.ToString()),
			FMath::Abs(FVector::DotProduct(Wrapping.State.PathAxisDirection, FVector(0, 1, 0))) > 0.99f);
	}

	// 2) An end hit (normal Y, parallel to the long axis): the long axis is rejected by the normal gate
	// and the larger cross axis, X or Z at 60 each, must win - never the near-normal Y.
	{
		FRopeConvexCollider Convex = MakeLogConvex();
		TArray<IRopeCollider*> Colliders = { &Convex };
		FRopeSimState Sim = RopeTest::MakeStraightRope(9, 160.0f, FVector(0, 101, 0), FVector(1, 0, 0));

		FRopeSurfaceAnchor Latch;
		Latch.NodeIndex = 0;
		Latch.Bone = LogBone;
		Latch.Mesh = Mesh;
		Latch.LocalSurfacePosition = FVector(0, 100, 0);
		Latch.LocalNormal = FVector(0, 1, 0);
		Latch.LocalTangent = FVector(1, 0, 0);
		Latch.StartWorldPosition = FVector(0, 100, 0);
		Latch.SurfaceOffset = 1.0f;

		FRopeWrapConfig Config = MakeTestWrapConfig();
		const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
			/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
			/*bHasGuidePlaneNormal*/ false, FVector::ZeroVector };

		FRopeWrappingPhase Wrapping;
		TestTrue(TEXT("wrapping begins on the log end hit"), Wrapping.Begin(Latch, 0.16f, Sim, Ctx));
		TestTrue(FString::Printf(TEXT("end hit rejects the near-normal long axis (dir=%s)"),
				*Wrapping.State.PathAxisDirection.ToString()),
			FMath::Abs(FVector::DotProduct(Wrapping.State.PathAxisDirection, FVector(0, 1, 0))) < 0.5f);
	}


	// 3) A ground-pivoted log (rigid translation lifts the body 50 above the component pivot): the axis
	// origin must sit at the shape's centre, not at the pivot - an axis along the log's bottom edge
	// degenerates the radial/circumferential directions the surface walk derives from it.
	{
		TArray<FPlane> Planes;
		Planes.Add(FPlane(FVector(1, 0, 0), 30.0));
		Planes.Add(FPlane(FVector(-1, 0, 0), 30.0));
		Planes.Add(FPlane(FVector(0, 1, 0), 100.0));
		Planes.Add(FPlane(FVector(0, -1, 0), 100.0));
		Planes.Add(FPlane(FVector(0, 0, 1), 30.0));
		Planes.Add(FPlane(FVector(0, 0, -1), 30.0));
		FRopeConvexCollider Convex(MoveTemp(Planes),
			FBox(FVector(-30.0, -100.0, -30.0), FVector(30.0, 100.0, 30.0)),
			FQuat::Identity, FVector(0.0, 0.0, 50.0));
		Convex.Bone = LogBone;
		Convex.SourceMesh = Mesh;
		TArray<IRopeCollider*> Colliders = { &Convex };
		FRopeSimState Sim = RopeTest::MakeStraightRope(9, 160.0f, FVector(0, 0, 81), FVector(1, 0, 0));

		FRopeSurfaceAnchor Latch;
		Latch.NodeIndex = 0;
		Latch.Bone = LogBone;
		Latch.Mesh = Mesh;
		Latch.LocalSurfacePosition = FVector(0, 0, 80);
		Latch.LocalNormal = FVector(0, 0, 1);
		Latch.LocalTangent = FVector(1, 0, 0);
		Latch.StartWorldPosition = FVector(0, 0, 80);
		Latch.SurfaceOffset = 1.0f;

		FRopeWrapConfig Config = MakeTestWrapConfig();
		const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
			/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
			/*bHasGuidePlaneNormal*/ false, FVector::ZeroVector };

		FRopeWrappingPhase Wrapping;
		TestTrue(TEXT("wrapping begins on the pivoted log"), Wrapping.Begin(Latch, 0.16f, Sim, Ctx));
		TestTrue(FString::Printf(TEXT("axis origin sits at the shape centre, not the pivot (origin=%s)"),
				*Wrapping.State.PathAxisOrigin.ToString()),
			FMath::Abs(Wrapping.State.PathAxisOrigin.Z - 50.0) < 1.0);
	}

	// 4) RopeLog-style collision: one horizontal convex plus two remote supports share one virtual bone.
	// The union still supplies the long Y direction, but its Z centre is far below the contacted bar.
	// The winding axis must pass through the contacted bar, not through the three-piece union centre.
	{
		const auto MakeConvexPiece = [&](const FVector& Extent, const FVector& Translation)
		{
			TArray<FPlane> Planes;
			Planes.Add(FPlane(FVector(1, 0, 0), Extent.X));
			Planes.Add(FPlane(FVector(-1, 0, 0), Extent.X));
			Planes.Add(FPlane(FVector(0, 1, 0), Extent.Y));
			Planes.Add(FPlane(FVector(0, -1, 0), Extent.Y));
			Planes.Add(FPlane(FVector(0, 0, 1), Extent.Z));
			Planes.Add(FPlane(FVector(0, 0, -1), Extent.Z));
			FRopeConvexCollider Piece(MoveTemp(Planes), FBox(-Extent, Extent),
				FQuat::Identity, Translation);
			Piece.Bone = LogBone;
			Piece.SourceMesh = Mesh;
			return Piece;
		};

		FRopeConvexCollider Bar = MakeConvexPiece(FVector(30, 150, 30), FVector(0, 0, 100));
		FRopeConvexCollider LeftSupport = MakeConvexPiece(FVector(20, 20, 100), FVector(0, -120, 0));
		FRopeConvexCollider RightSupport = MakeConvexPiece(FVector(20, 20, 100), FVector(0, 120, 0));
		TArray<IRopeCollider*> Colliders = { &Bar, &LeftSupport, &RightSupport };
		FRopeSimState Sim = RopeTest::MakeStraightRope(9, 160.0f,
			FVector(0, 0, 131), FVector(1, 0, 0));

		FRopeSurfaceAnchor Latch;
		Latch.NodeIndex = 0;
		Latch.Bone = LogBone;
		Latch.Mesh = Mesh;
		Latch.LocalSurfacePosition = FVector(0, 0, 130);
		Latch.LocalNormal = FVector(0, 0, 1);
		Latch.LocalTangent = FVector(1, 0, 0);
		Latch.StartWorldPosition = FVector(0, 0, 130);
		Latch.SurfaceOffset = 1.0f;

		FRopeWrapConfig Config = MakeTestWrapConfig();
		const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
			/*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true,
			/*bHasGuidePlaneNormal*/ false, FVector::ZeroVector };

		FRopeWrappingPhase Wrapping;
		TestTrue(TEXT("wrapping begins on a multi-piece log"), Wrapping.Begin(Latch, 0.16f, Sim, Ctx));
		TestTrue(FString::Printf(TEXT("multi-piece log keeps the union long axis (dir=%s)"),
				*Wrapping.State.PathAxisDirection.ToString()),
			FMath::Abs(FVector::DotProduct(Wrapping.State.PathAxisDirection, FVector(0, 1, 0))) > 0.99f);
		TestTrue(FString::Printf(TEXT("axis origin stays on the contacted bar (origin=%s)"),
				*Wrapping.State.PathAxisOrigin.ToString()),
			FMath::Abs(Wrapping.State.PathAxisOrigin.Z - 100.0) < 1.0);
	}

	return true;
}


	// Consuming the capture snapshot, which applies to CaptureTravelPlane alone:
	// - the axis origin is the contact region centre rather than the latch bone's location, which measures
	//   the radius about the centre of a pair such as two legs;
	// - the winding reference is the capture velocity rather than the latch tangent, so the direction
	//   wrapping starts in matches the real motion;
	// - with no snapshot it falls back to the previous behaviour, meaning the bone origin and tangent
	//   winding.
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

	// The capture snapshot: the contact centre is offset from the bone origin, and the capture velocity
	// points along positive X.
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
	// The latch tangent along positive Y is perpendicular to the circumference, so the previous rule could
	// not determine a sign here. Only with the velocity as the reference does wrapping start towards
	// positive X.
	TestTrue(FString::Printf(TEXT("winding starts along the capture velocity (circ=%s)"),
			*FrameWrapping.State.PathCircumferenceDir.ToString()),
		FVector::DotProduct(FrameWrapping.State.PathCircumferenceDir, FVector(1, 0, 0)) > 0.1f);

	// With no snapshot the origin stays at the latch bone's location, which the mock identity puts at the
	// origin.
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

	// Gap bridging, on a reduced two-leg geometry: a pair of parallel capsules is wrapped about the travel
	// plane axis running through the centre of the pair. Without bridging this path necessarily dies at one
	// of two points, because in the air between the legs the projection either snaps to the far surface,
	// curling the path into the valley, or fails outright and breaks the build. The contract is that
	//  1) the path completes with no failure;
	//  2) chord bridge path points exist and their nodes have no anchor;
	//  3) anchors are created on both capsule surfaces; and
	//  4) the accumulated wrapped angle proves it went round the pair, exceeding 270 degrees.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingGapBridgeTest,
	"DynamicRope.Wrapping.GapBridgeWrapsCapsulePair",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingGapBridgeTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = MakeMockTarget();
	// The two legs share a mesh and a bone name: a non-skeletal mock has no bone graph, so the only
	// candidate is the current bone, and sharing the name reflects that. What this test covers is crossing
	// the gap rather than transitioning between bones.
	FCapsuleCollider LegA(FVector(30, 0, -50), FVector(30, 0, 50), 12.0f, FName("legs"), Mesh);
	FCapsuleCollider LegB(FVector(-30, 0, -50), FVector(-30, 0, 50), 12.0f, FName("legs"), Mesh);
	TArray<IRopeCollider*> Colliders = { &LegA, &LegB };

	// A 280 cm rope of fifteen nodes, long enough to wrap more than once around the pair's hull perimeter.
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

	// The capture snapshot puts the axis origin at the centre of the pair and takes the winding from the
	// capture velocity.
	FRopeCaptureTravelFrame Frame;
	Frame.bValid = true;
	Frame.RegionCenter = FVector::ZeroVector;
	Frame.AverageVelocity = FVector(0, 100, 0);

	FRopeWrapConfig Config = MakeTestWrapConfig();
	Config.WrappingAxisSource = ERopeWrappingAxisSource::CaptureTravelPlane;
	// The chord is 60 cm, but the bridge path detours along an arc at the axis radius, which is longer, so
	// the limit is set with margin.
	Config.WrappingMaxGapBridgeDistance = 120.0f;
	// This test verifies going round within the travel plane alone, so the helical rise along the axis is
	// kept from running off the top of the capsules.
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

	// One: bridging lets it complete with no failure.
	TestTrue(TEXT("path build completes around the pair"), Wrapping.State.bPathBuildComplete);
	TestTrue(TEXT("path build did not fail"), !Wrapping.State.bPathBuildFailed);

	// Two: chord path points exist and their nodes have no anchor.
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

	// Three: anchors were created on both capsule surfaces. With an identity transform, local X
	// distinguishes them directly.
	bool bAnchorOnLegA = false;
	bool bAnchorOnLegB = false;
	for (const FRopeSurfaceAnchor& Anchor : Wrapping.State.Anchors)
	{
		bAnchorOnLegA |= Anchor.LocalSurfacePosition.X > 15.0f;
		bAnchorOnLegB |= Anchor.LocalSurfacePosition.X < -15.0f;
	}
	TestTrue(TEXT("anchors reached the near leg"), bAnchorOnLegA);
	TestTrue(TEXT("anchors reached the far leg"), bAnchorOnLegB);

	// Four: the accumulated wrapped angle proves it went round the pair.
	float AngleDeg = 0.0f;
	TestTrue(TEXT("wrapped angle computable"), Wrapping.ComputeBuiltPathWrapAngle(Sim, Ctx, AngleDeg));
	TestTrue(FString::Printf(TEXT("accumulated angle circles the pair (%.0f deg)"), AngleDeg),
		AngleDeg > 270.0f);

	// Five: the shape-based coverage. Having gone round the pair, no large gap should remain about the axis.
	float CoverageDeg = 0.0f;
	TestTrue(TEXT("enclosure coverage computable"), Wrapping.ComputeWrapEnclosureCoverage(CoverageDeg));
	TestTrue(FString::Printf(TEXT("pair wrap leaves no escape gap (coverage=%.0f deg)"), CoverageDeg),
		CoverageDeg > 300.0f);

	// Six: the real centreline arc-length resampling contract. The path coordinates are multiples of the
	// segment length, and the chord between adjacent nodes cannot exceed the polyline arc between them and
	// therefore never exceeds the segment length.
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

	// Cluster centre axis correction: capture happens the instant the rope touches the first leg, which puts
	// the region centre on that one leg. Standing the axis there makes the field spiral around that leg
	// alone, and with the axis inside a target the winding departure gate stays silent as well, so it never
	// crosses to the other side; measurements showed over 1700 degrees on a single leg.
	// In bridging mode the cluster of nearby collider centres on the same mesh moves the origin to the
	// centre of the pair, so the path goes round both even when contact starts on one side.
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

	// Reproducing a real capture: contact is on the first leg alone, so the region centre lies on its
	// surface.
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

	// The axis origin moves from the first contact point to the collider cluster centre, which is the
	// midpoint of the two capsules.
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

	// The wrap amount limit: instead of a long rope drawing itself in over several turns, the path finishes
	// early as a success once the accumulated wrapped angle reaches the target, with the tail node count
	// reduced to the length of the path that was built, and the front motion does not drag the rope beyond
	// the path, which stays frozen and becomes free after the commit. It is the guard against the
	// multi-thousand-degree runaway spirals seen in measurement.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingWrapAngleCapTest,
	"DynamicRope.Wrapping.WrapAngleCapStopsSpiral",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingWrapAngleCapTest::RunTest(const FString& Parameters)
{
	// A capsule of radius 25 with a 320 cm rope of seventeen nodes, which is long enough to spiral more
	// than twice without a limit.
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

	// Reaching the limit finishes as a success rather than a failure. The path is shorter than the rope, and
	// the tail node count, which is the basis of the front and commit target distance, has to have been
	// reduced to the path's length.
	TestTrue(TEXT("capped path finishes as success"), Wrapping.State.bPathBuildComplete);
	TestTrue(TEXT("capped path did not fail"), !Wrapping.State.bPathBuildFailed);
	TestTrue(FString::Printf(TEXT("path stops short of the rope (%d/17 points)"), Wrapping.State.Path.Num()),
		Wrapping.State.Path.Num() < 17);
	TestEqual(TEXT("NumTailNodes shrinks to the built path"),
		Wrapping.State.NumTailNodes, Wrapping.State.Path.Num());
	TestEqual(TEXT("every capped path point is anchored"),
		Wrapping.State.Anchors.Num(), Wrapping.State.Path.Num());

	// The angle at completion should be just past the target, between the target and the target plus one
	// step, which forbids a multi-turn spiral.
	float AngleDeg = 0.0f;
	TestTrue(TEXT("wrapped angle computable"), Wrapping.ComputeBuiltPathWrapAngle(Sim, Ctx, AngleDeg));
	TestTrue(FString::Printf(TEXT("angle stops just past the cap (%.0f deg)"), AngleDeg),
		AngleDeg >= 360.0f && AngleDeg < 460.0f);

	// The front motion does not drag the rope beyond the path, which stays frozen and becomes a free span
	// after the commit.
	FRopeNodeOverrideFrame Frame;
	for (int32 Step = 0; Step < 64; ++Step)
	{
		Frame.Reset();
		Wrapping.ApplyWrappingMotionOverrides(Sim, 0.05f, Ctx, Frame);
	}
	const int32 LastDrivenNode = Latch.NodeIndex + Wrapping.State.NumTailNodes - 1;
	TestTrue(TEXT("last path node is front-driven"),
		Frame.Flags.IsValidIndex(LastDrivenNode) &&
		(Frame.Flags[LastDrivenNode] & RopeNodeOverride::Position) != 0);
	TestTrue(TEXT("leftover rope beyond the cap stays untouched"),
		Frame.Flags.IsValidIndex(16) &&
		(Frame.Flags[LastDrivenNode + 1] & RopeNodeOverride::Position) == 0 &&
		(Frame.Flags[16] & RopeNodeOverride::Position) == 0);
	Wrapping.ApplyWrappingKinematicMask(Sim, Frame);
	TestTrue(TEXT("last path node remains kinematic while wrapping"),
		Frame.InvMass.IsValidIndex(LastDrivenNode) && Frame.InvMass[LastDrivenNode] == 0.0f);
	TestTrue(TEXT("leftover rope beyond the cap remains solver-owned"),
		Frame.InvMass.IsValidIndex(16) &&
		Frame.InvMass[LastDrivenNode + 1] > 0.0f && Frame.InvMass[16] > 0.0f);
	return true;
}

	// The shape-based binding measure, ComputeWrapEnclosureCoverage: on the same capsule with different rope
	// lengths, a full wrap converges on 360 degrees of coverage, while a half hook leaves the opposite side
	// of the axis entirely empty and its coverage is lower by exactly that much.
	// The contract is that the commit coverage gate uses this value to distinguish being enclosed from
	// merely being caught on.
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
	// The capsule's Z axis is pinned as the reference axis for the coverage comparison, rather than relying
	// on automatic shape axis inference.
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

	// A full wrap of nine nodes leaves a gap no larger than one step, so the coverage converges on 360
	// degrees.
	float FullCoverageDeg = 0.0f;
	TestTrue(TEXT("full wrap builds and measures"), BuildAndMeasure(9, 160.0f, FullCoverageDeg));
	TestTrue(FString::Printf(TEXT("full wrap coverage nears 360 (%.0f deg)"), FullCoverageDeg),
		FullCoverageDeg > 300.0f);

	// A half hook of five nodes ends after about a third of a turn, leaving the opposite side entirely
	// empty.
	float HookCoverageDeg = 0.0f;
	TestTrue(TEXT("hook wrap builds and measures"), BuildAndMeasure(5, 80.0f, HookCoverageDeg));
	TestTrue(FString::Printf(TEXT("hook coverage stays low (%.0f deg)"), HookCoverageDeg),
		HookCoverageDeg < 220.0f);
	TestTrue(TEXT("coverage separates hook from wrap"), FullCoverageDeg > HookCoverageDeg + 90.0f);
	return true;
}

	// Pins that the wrap target gate, URopeComponent::CanWrapTarget, applies to the wrapping path build as
	// well. Applying it only to aiming, the preview and the judgement left the wrapping path taking the
	// collider snapshot unfiltered, which produced the inconsistency of a path being laid over a target
	// aiming had refused. The gate is one filter at the component boundary.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapTargetGateFilterTest,
	"DynamicRope.Wrapping.CanWrapTargetGateFiltersWrappingColliders",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapTargetGateFilterTest::RunTest(const FString& Parameters)
{
	USceneComponent* AllowedMesh = MakeMockTarget();
	USceneComponent* DeniedMesh = MakeMockTarget();

	FCapsuleCollider AllowedCol(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), AllowedMesh);
	FCapsuleCollider DeniedCol(FVector(100, 0, -50), FVector(100, 0, 50), 25.0f, FName("leg"), DeniedMesh);
	// A collider with no attribution, meaning static world geometry, can never be a wrap target and is
	// therefore not subject to the gate.
	FCapsuleCollider WorldGeo(FVector(0, 200, -50), FVector(0, 200, 50), 25.0f, NAME_None, nullptr);

	const TArray<IRopeCollider*> In = { &AllowedCol, &DeniedCol, &WorldGeo };
	TArray<IRopeCollider*> Out;

	// One: the default gate permits everything and leaves the input unchanged, which is the contract that a
	// rope not overriding it behaves identically.
	RopeWrapTargets::FilterWrappableColliders(In,
		[](const USceneComponent*, FName) { return true; }, Out);
	TestEqual(TEXT("the collider set is unchanged under the default gate"), Out.Num(), In.Num());

	// Two: refusing one target removes that collider alone.
	RopeWrapTargets::FilterWrappableColliders(In,
		[DeniedMesh](const USceneComponent* Mesh, FName) { return Mesh != DeniedMesh; }, Out);
	TestEqual(TEXT("the one refused target is removed"), Out.Num(), 2);
	TestTrue(TEXT("a permitted target remains"), Out.Contains(&AllowedCol));
	TestFalse(TEXT("a refused target is invisible to the wrapping path"), Out.Contains(&DeniedCol));
	TestTrue(TEXT("unattributed world geometry remains regardless of the gate"), Out.Contains(&WorldGeo));

	// Three: even refusing everything keeps the unattributed surface geometry, since a rope must not pass
	// through a wall.
	RopeWrapTargets::FilterWrappableColliders(In,
		[](const USceneComponent*, FName) { return false; }, Out);
	TestEqual(TEXT("unattributed geometry remains even when everything is refused"), Out.Num(), 1);
	TestTrue(TEXT("what remains is the world geometry"), Out.Contains(&WorldGeo));

	return true;
}

	// Whether the per-frame path build budget still varies with the configured steps per frame on a large
	// rope, meaning it scales proportionally rather than acting as a simple floor. With 72 tail nodes the
	// size baseline is large, and the ordering by configured value has to survive, which guards against
	// the setting being nullified.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapStepBudgetScalingTest,
	"DynamicRope.Wrapping.StepBudgetScalesWithStepsPerFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapStepBudgetScalingTest::RunTest(const FString& Parameters)
{
	// With 72 tail nodes the size baseline is 36, so four steps per frame gives 18, the default of eight
	// gives 36, and twelve gives 54.
	const int32 Sparse   = FRopeWrappingPhase::ComputePathStepBudget(72, 4);
	const int32 Baseline = FRopeWrappingPhase::ComputePathStepBudget(72, 8);
	const int32 Dense    = FRopeWrappingPhase::ComputePathStepBudget(72, 12);
	TestEqual(TEXT("72 nodes at four steps per frame gives 18"), Sparse, 18);
	TestEqual(TEXT("72 nodes at eight steps per frame gives 36"), Baseline, 36);
	TestEqual(TEXT("72 nodes at twelve steps per frame gives 54"), Dense, 54);
	TestTrue(TEXT("the configured value still varies the budget on a large rope"), Sparse < Baseline && Baseline < Dense);

	// The preview value is effectively unlimited and completes in one frame, since the total work is twice
	// the tail node count.
	TestTrue(TEXT("the preview budget covers the total work"), FRopeWrappingPhase::ComputePathStepBudget(72, 4096) >= 72 * 2);

	// The minimum is one, even at zero or a negative node count.
	TestEqual(TEXT("zero nodes still gives at least one"), FRopeWrappingPhase::ComputePathStepBudget(0, 8), 1);

	return true;
}

#endif
