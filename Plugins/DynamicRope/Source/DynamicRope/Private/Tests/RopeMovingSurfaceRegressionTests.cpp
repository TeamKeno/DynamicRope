// Copyright Epic Games, Inc. All Rights Reserved.
//
// Unit tests for the ragdoll transition matrix.
// Going limp happens outside the plugin, in game code, but the contracts the rope has to honour at that
// moment can be verified at the logic level with no world:
//   (a) Hold follows a bone transform that jumps a long way in one frame, with no velocity injected, by
//       writing the previous position equal to the current one.
//   (b) Hold returns false with no fallback once the wrapped target is destroyed and the weak pointer is
//       null, so the caller releases.
//   (c) Solver friction: the drag from a surface velocity spike is clamped by the Coulomb limit and is
//       therefore not proportional to the size of the spike.
//   (d) The detector: relative motion evaluation subtracts the surface velocity, and a spike does not
//       block the capture gate, which pins the current behaviour and documents that the real defences are
//       the dwell in Contacting and dismissal when the candidates disappear.
// Restarting the dwell when the dominant bone changes belongs to the runtime contact tracker.
// Real physics bodies, partial ragdolls and capsule rebuilds are covered by the play-in-editor checklist.

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
	 * A mock mesh whose transform can be moved. With no skeletal asset, the socket lookup returns the
	 * component transform with no warning, so in these tests the component transform plays the part of the
	 * bone; a ragdoll pose pop is modelled as that transform jumping in one frame. The component is
	 * unregistered, so it is moved directly rather than through the move path.
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

	/** Starts a wrap with one node latched with no bone name, which follows the component transform. */
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

// (a) The ragdoll transition frame: even when the bone, meaning the mock transform, jumps a long way in one
// frame, Hold replaces the node exactly at the new bone position and writes zero velocity. That the jump is
// not injected into the solver as velocity is why the rope does not snap on going limp.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeRagdollHoldFollowsJumpTest,
	"DynamicRope.Ragdoll.HoldFollowsBoneJumpWithZeroVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeRagdollHoldFollowsJumpTest::RunTest(const FString& Parameters)
{
	// The nodes are evenly spaced along X, one of them is latched, and the mesh starts at the identity.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakeMovableMockMesh();
	const FVector NodeWorld = Sim.Positions[3];

	FRopeWrapController Wrap;
	FRopeNodeOverrideFrame BeginFrame;
	BeginMockWrap(Wrap, Sim, Mesh, 3, BeginFrame);
	TestTrue(TEXT("wrap holds after BeginWrap"), Wrap.State.IsWrapped());

	// Before the jump, the anchor stays where it was when the wrap began.
	FRopeNodeOverrideFrame HoldFrame;
	TestTrue(TEXT("Hold succeeds before jump"), Wrap.Hold(Sim, 0.016f, HoldFrame));
	TestTrue(TEXT("node stays at latch position before jump"),
		HoldFrame.Positions[3].Equals(NodeWorld, 0.01f));

	// The ragdoll pose pop: the bone jumps a long way upwards in a single frame.
	const FVector Jump(0.0f, 0.0f, 500.0f);
	TeleportMockMesh(Mesh, Jump);
	TestTrue(TEXT("mock mesh transform actually moved (test rig sanity)"),
		Mesh->GetComponentLocation().Equals(Jump, 0.01f));

	FRopeNodeOverrideFrame JumpFrame;
	TestTrue(TEXT("Hold succeeds on jump frame"), Wrap.Hold(Sim, 0.016f, JumpFrame));
	// The node moves with the bone by exactly that amount.
	TestTrue(FString::Printf(TEXT("node follows bone jump exactly (%s)"), *JumpFrame.Positions[3].ToCompactString()),
		JumpFrame.Positions[3].Equals(NodeWorld + Jump, 0.01f));
	// Zero velocity is recorded, so the jump is not injected as a Verlet velocity.
	TestTrue(TEXT("jump is written with zero velocity (PrevFromPosition)"),
		(JumpFrame.Flags[3] & RopeNodeOverride::PrevFromPosition) != 0);
	// The node is still owned by logic, with an inverse mass of zero.
	TestTrue(TEXT("node stays logic-owned (InvMass override 0)"),
		(JumpFrame.Flags[3] & RopeNodeOverride::InvMass) != 0 && JumpFrame.InvMass[3] == 0.0f);
	return true;
}

// (b) Losing the wrapped target: when the mesh is destroyed, as by a ragdoll death effect, Hold returns
// false so the caller releases, with no dangling dereference and no dragging towards a wrong position.
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

	// Destroy the target, which marks it for collection and makes the weak pointer null.
	Mesh->MarkAsGarbage();
	FRopeNodeOverrideFrame LostFrame;
	TestFalse(TEXT("Hold returns false after mesh loss (caller must release)"),
		Wrap.Hold(Sim, 0.016f, LostFrame));
	TestFalse(TEXT("no node override is produced on the loss frame"), LostFrame.HasAny());
	return true;
}

// (c) A surface velocity spike on the transition frame: when the bone jumps as the ragdoll is enabled, its
// movement relative to the capsule's previous endpoints is large and the surface velocity spikes. The
// friction drag is clamped by the Coulomb limit, so the rope must not be swept along in proportion to that
// spike: multiplying the spike tenfold has to leave the per-frame displacement almost unchanged, which
// proves the clamp is active.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeRagdollFrictionClampTest,
	"DynamicRope.Ragdoll.FrictionClampBoundsSurfaceVelocitySpike",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeRagdollFrictionClampTest::RunTest(const FString& Parameters)
{
	// A sphere whose upper surface sits near the rope's plane, so the nodes around it penetrate within the
	// collision radius and contact and friction engage. Gravity presses the rope onto the surface and builds
	// up the constraint force.
	auto MaxXDisplacementAfterOneStep = [](float SpikeSpeed) -> float
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
		TArray<FVector> InitialPositions = Sim.Positions;

		USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
		RopeTest::FSphereMockCollider Body(FVector(60.0f, 0.0f, -200.0f), 200.0f, FName("body"), Mesh);
		// The pose pop, as a tangential spike.
		Body.SurfaceVelocity = FVector(SpikeSpeed, 0.0f, 0.0f);
		TArray<IRopeCollider*> Colliders = { &Body };

		// With the default friction and gravity. The collision radius is stated explicitly, because the
		// struct default became automatic and automatic resolution exists only at the component boundary, so
		// a direct solver call has to state it.
		FRopeSolverConfig Config;
		Config.CollisionRadius = 2.0f;
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

	// A large spike: with no clamp and full transfer it would sweep the rope a long way per frame.
	const float Disp1x = MaxXDisplacementAfterOneStep(60000.0f);
	// A tenfold spike, which with no clamp would displace it roughly ten times as far.
	const float Disp10x = MaxXDisplacementAfterOneStep(600000.0f);

	TestTrue(TEXT("no NaN with 1x spike"), Disp1x >= 0.0f);
	TestTrue(TEXT("no NaN with 10x spike"), Disp10x >= 0.0f);
	// The smoke bound: less than half of the surface's own movement may be transferred; with no clamp it
	// would be nearly all of it.
	TestTrue(FString::Printf(TEXT("drag is far below full surface transport (%.1fcm < 500cm)"), Disp1x),
		Disp1x < 500.0f);
	// The essential property: the clamp limit depends on the constraint force rather than the spike, so a
	// tenfold spike has to leave the displacement almost identical.
	TestTrue(FString::Printf(TEXT("drag does not scale with spike (1x=%.2fcm, 10x=%.2fcm)"), Disp1x, Disp10x),
		Disp10x < Disp1x * 2.0f + 1.0f);
	return true;
}

// (d) Pinning the current contract of relative motion evaluation and capture:
//  - Relative motion subtracts the surface velocity from the rope's per-frame displacement after
//    converting it by the delta. A stationary rope on a moving surface therefore has a relative tangential
//    speed of the surface speed times the delta, which is what makes grazing detectable even on a moving
//    bone. Omitting that conversion overstated it by the reciprocal of the delta, and this pins the fix.
//  - Capture applies no additional quality filter, so a spike does not prevent a capture, which is pinned
//    as current behaviour. The real defences against a false positive on the transition frame are the dwell
//    in Contacting, dismissal when the candidates disappear, and the clamp in (c).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeRagdollRelativeMotionTest,
	"DynamicRope.Ragdoll.RelativeMotionSubtractsSurfaceVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeRagdollRelativeMotionTest::RunTest(const FString& Parameters)
{
	// A stationary rope, with the previous positions equal to the current ones, on a surface moving along X.
	// The normal is vertical, so the surface motion is entirely tangential.
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
	// The substep delta is stated explicitly to verify the unit conversion.
	Params.SubstepDeltaTime = 0.02f;

	TArray<FRopeContactCandidate> Candidates;
	Candidates.Add(MakeSpikeCandidate(3, 500.0f));
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, Params, Candidates);

	TestTrue(TEXT("candidate stays valid (mock mesh has no bone axis to judge miss cone)"),
		Candidates[0].bValid);
	// A stationary rope on a surface moving at a given speed gives a relative tangential speed of that speed
	// times the substep delta; subtracting the surface velocity without converting it would give the raw
	// speed instead, and preventing that regression is why this assertion exists.
	TestTrue(FString::Printf(TEXT("relative tangential speed equals surface speed x dt for a resting rope (%.2f)"),
		Candidates[0].RelativeTangentialSpeed),
		FMath::IsNearlyEqual(Candidates[0].RelativeTangentialSpeed, 10.0f, 0.05f));

	// Pinning the capture behaviour: even with a surface velocity spike, meeting the minimum latch node
	// count is enough to capture.
	// Any future quality filter has to update this assertion along with its spike cut-off.
	TArray<FRopeContactCandidate> SpikeCandidates;
	SpikeCandidates.Add(MakeSpikeCandidate(3, 60000.0f));
	SpikeCandidates.Add(MakeSpikeCandidate(4, 60000.0f));
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, Params, SpikeCandidates);
	TestTrue(TEXT("capture proceeds despite surface-velocity spike (no additional quality filter)"),
		FRopeFlightContactDetector::EvaluateCapture(SpikeCandidates, Params).bShouldCapture);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
