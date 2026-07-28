// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Solver/RopeXPBDSolver.h"
#include "Collision/RopeCollider.h"
#include "RopeTestHelpers.h"

namespace
{
	FRopeSolverConfig MakeStiffConfig()
	{
		FRopeSolverConfig C;
		C.Substeps = 4;
		C.Iterations = 8;
		// Rigid
		C.StretchCompliance = 0.0f;
		C.BendCompliance = 0.02f;
		C.Gravity = FVector::ZeroVector;
		C.Damping = 0.0f;
		return C;
	}
}

// Does the doubled Free chain converge to the rest segment length after several steps (distance constraint)?
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverDistanceTest,
	"DynamicRope.Solver.DistanceConvergesToRest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverDistanceTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		// 2x stretch
		Sim.Positions[i] *= 2.0f;
		// maintain velocity 0
		Sim.SetStill(i);
	}

	const FRopeSolverConfig Config = MakeStiffConfig();
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;
	for (int32 Frame = 0; Frame < 60; ++Frame)
	{
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
	}

	const float Err = RopeTest::MaxSegmentError(Sim);
	TestTrue(FString::Printf(TEXT("max segment error %.3f cm should be < 1.0"), Err), Err < 1.0f);
	TestFalse(TEXT("no NaN"), RopeTest::AnyNaN(Sim));
	return true;
}

// Strain limiting: When a long chain hangs on a pin and the segment adjacent to the anchor is overstretched, even if the iteration is insufficient (it=1)
// End of substep Does the sequential clamp confine all segments to ≤ MaxStretchRatio×SegmentLength? And if disabled(0)
// Does it exceed the cap under the same conditions (prove by comparison that the clamp is the cause).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverStrainLimitTest,
	"DynamicRope.Solver.StrainLimitBoundsStretch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverStrainLimitTest::RunTest(const FString& Parameters)
{
	auto MakeStretchedPinnedRope = []() -> FRopeSimState
	{
		// 40node, SegmentLength=10. All segments were overstretched by spreading the nodes at three-fold intervals (30cm), and node 0 was pinned.
		FRopeSimState S = RopeTest::MakeStraightRope(40, 390.0f); // seg = 390/39 = 10
		for (int32 i = 0; i < S.Num(); ++i)
		{
			const FVector P = FVector(static_cast<float>(i) * 3.0f * S.SegmentLength, 0.0f, 0.0f);
			S.Positions[i] = P;
			S.PrevPositions[i] = P; // velocity 0
		}
		S.bStartPinned = true;
		S.StartPinPrev = S.Positions[0];
		S.StartPinTarget = S.Positions[0];
		S.InvMass[0] = 0.0f;
		return S;
	};

	auto MaxSegmentLen = [](const FRopeSimState& S) -> float
	{
		float M = 0.0f;
		for (int32 i = 0; i + 1 < S.Num(); ++i)
		{
			M = FMath::Max(M, static_cast<float>(FVector::Dist(S.Positions[i], S.Positions[i + 1])));
		}
		return M;
	};

	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;

	// Bone the sole contribution of the strain limit with a weak solver (it=1).
	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Iterations = 1;
	Config.MaxStretchRatio = 1.5f;

	// (1) strain limit ON: After one step, all segments are ≤ 1.5×seg.
	{
		FRopeSimState Sim = MakeStretchedPinnedRope();
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
		const float MaxLen = MaxSegmentLen(Sim);
		const float Limit = 1.5f * Sim.SegmentLength;
		TestTrue(FString::Printf(TEXT("strain-limited max segment %.2f should be <= %.2f"), MaxLen, Limit * 1.02f),
			MaxLen <= Limit * 1.02f);
		TestFalse(TEXT("no NaN (strain limit on)"), RopeTest::AnyNaN(Sim));
	}

	// (2) Contrast — strain limit OFF(0): With the same weak solver, the cap is greatly exceeded in one step (proving that the clamp is the cause).
	{
		FRopeSolverConfig Off = Config;
		Off.MaxStretchRatio = 0.0f;
		FRopeSimState Sim = MakeStretchedPinnedRope();
		Solver.Step(Sim, Off, NoColliders, 1.0f / 60.0f);
		const float MaxLen = MaxSegmentLen(Sim);
		const float Limit = 1.5f * Sim.SegmentLength;
		TestTrue(FString::Printf(TEXT("without strain limit max segment %.2f should exceed %.2f"), MaxLen, Limit),
			MaxLen > Limit);
	}

	return true;
}

// Does the CPU contact settle on the stationary collider without backlash (outer normal velocity injection)? — The GPU uses VnOut after push-out.
// to remove parity. The restitution memory (CL 189) was judged as “CPU SolveContacts have a constraint-type structure, so mirrors are not required”;
// This test nails the deal and catches regression (rebound/trampoline).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverStaticContactNoReboundTest,
	"DynamicRope.Solver.StaticContactNoRebound",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverStaticContactNoReboundTest::RunTest(const FString& Parameters)
{
	// Pinned node0 to a rest distance just above the sphere surface, and dropped node1 below it onto the sphere by gravity.
	// At surface z=7 (sphere radius 5 + node thickness 2), the rest distance (200) is just right, so the distance constraint force is 0 when settling → pure gravity vs contact.
	FRopeSimState Sim = RopeTest::MakeStraightRope(2, 200.0f); // SegmentLength=200
	Sim.Positions[0]     = FVector(0, 0, 207);
	Sim.PrevPositions[0] = FVector(0, 0, 207);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;
	Sim.Positions[1]     = FVector(0, 0, 100); // Stationary starts on a sphere → gravity falls
	Sim.PrevPositions[1] = FVector(0, 0, 100);

	RopeTest::FSphereMockCollider Sphere(FVector::ZeroVector, 5.0f, FName("static"));
	const TArray<IRopeCollider*> Colliders = { &Sphere };

	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -980.0f); // Maintain contact by pressing against the surface with gravity
	Config.Damping = 0.0f;                          // Does not discriminate against backlash due to damping
	Config.CollisionRadius = 2.0f;                  // node thickness → surface z ≈ 5+2 = 7

	const FRopeXPBDSolver Solver;
	// Turn sufficiently to drop + settle.
	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		Solver.Step(Sim, Config, Colliders, 1.0f / 60.0f);
	}
	// Observation of rebound/trampoline after settling: There should not be an upward velocity bouncing over the surface.
	float MaxZ = -1.0e30f;
	float MaxUpVel = -1.0e30f;
	for (int32 Frame = 0; Frame < 60; ++Frame)
	{
		Solver.Step(Sim, Config, Colliders, 1.0f / 60.0f);
		MaxZ = FMath::Max(MaxZ, static_cast<float>(Sim.Positions[1].Z));
		MaxUpVel = FMath::Max(MaxUpVel, static_cast<float>(Sim.Positions[1].Z - Sim.PrevPositions[1].Z));
	}

	const float SettledZ = static_cast<float>(Sim.Positions[1].Z);
	TestTrue(FString::Printf(TEXT("settles near surface (z=%.2f ~ 7)"), SettledZ), SettledZ > 6.0f && SettledZ < 8.0f);
	TestTrue(FString::Printf(TEXT("never bounces above surface (maxZ=%.2f)"), MaxZ), MaxZ < 8.5f);
	TestTrue(FString::Printf(TEXT("no outward velocity injection (maxUpVel=%.3f cm/substep)"), MaxUpVel), MaxUpVel < 1.0f);
	TestFalse(TEXT("no NaN"), RopeTest::AnyNaN(Sim));
	return true;
}

// InvMass 0 + bStartPinned Does the node maintain its pin position even under gravity?
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverPinTest,
	"DynamicRope.Solver.PinnedStartHeld",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverPinTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(10, 180.0f);
	const FVector Pin = Sim.Positions[0];
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Pin;
	Sim.StartPinTarget = Pin;
	Sim.InvMass[0] = 0.0f;

	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -980.0f);
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;
	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
	}

	TestTrue(TEXT("pinned node stays at pin"), Sim.Positions[0].Equals(Pin, 0.01f));
	TestFalse(TEXT("no NaN under gravity"), RopeTest::AnyNaN(Sim));
	return true;
}

// Does the non-stretch length be maintained without divergence/NaN in long-time simulations under gravity (explosion guard)?
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverStabilityTest,
	"DynamicRope.Solver.StableUnderGravity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverStabilityTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(16, 300.0f);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;

	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -980.0f);
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;
	for (int32 Frame = 0; Frame < 300; ++Frame)
	{
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
	}

	TestFalse(TEXT("no NaN over 300 frames"), RopeTest::AnyNaN(Sim));
	const float Err = RopeTest::MaxSegmentError(Sim);
	TestTrue(FString::Printf(TEXT("segment error %.2f bounded"), Err), Err < Sim.SegmentLength * 2.0f);
	return true;
}

// segment tension(XPBD distance λ → F=max(0,-λ)/h²): In a hanging rope, the upper segment should be larger.
// (there is a lot of mass hanging below), the top tension should be close to the theoretical value (number of nodes below × g). Weightless slack is ~0.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverTensionTest,
	"DynamicRope.Solver.SegmentTensionHangingRope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverTensionTest::RunTest(const FString& Parameters)
{
	constexpr int32 NumNodes = 10;
	constexpr float Gravity = 980.0f;
	FRopeSimState Sim = RopeTest::MakeStraightRope(NumNodes, 180.0f);
	Sim.bStartPinned = true;
	Sim.StartPinPrev = Sim.Positions[0];
	Sim.StartPinTarget = Sim.Positions[0];
	Sim.InvMass[0] = 0.0f;

	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -Gravity);
	// tension(λ) convergence check, so plenty.
	Config.Iterations = 32;
	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;
	for (int32 Frame = 0; Frame < 240; ++Frame)
	{
		Solver.Step(Sim, Config, NoColliders, 1.0f / 60.0f);
	}

	TestTrue(TEXT("tension array sized to segments"), Sim.SegmentTension.Num() == NumNodes - 1);

	// In static equilibrium, tension of segment k = hanging mass × g = (N-1-k) × 980 (node ​​mass 1).
	const float TopExpected = static_cast<float>(NumNodes - 1) * Gravity;
	const float Top = Sim.SegmentTension[0];
	TestTrue(FString::Printf(TEXT("top tension %.0f should be within 50%% of %.0f"), Top, TopExpected),
		Top > TopExpected * 0.5f && Top < TopExpected * 1.5f);

	// Monotonically decreasing from top to bottom (10% convergence error margin).
	for (int32 k = 1; k < Sim.SegmentTension.Num(); ++k)
	{
		TestTrue(FString::Printf(TEXT("tension[%d]=%.0f <= tension[%d]=%.0f (+10%%)"),
			k, Sim.SegmentTension[k], k - 1, Sim.SegmentTension[k - 1]),
			Sim.SegmentTension[k] <= Sim.SegmentTension[k - 1] * 1.1f + 1.0f);
	}

	// Weightless rest length rope(slack) → tension ~0.
	FRopeSimState Slack = RopeTest::MakeStraightRope(8, 140.0f);
	// Gravity = 0
	FRopeSolverConfig SlackConfig = MakeStiffConfig();
	for (int32 Frame = 0; Frame < 30; ++Frame)
	{
		Solver.Step(Slack, SlackConfig, NoColliders, 1.0f / 60.0f);
	}
	float SlackMax = 0.0f;
	for (const float T : Slack.SegmentTension) { SlackMax = FMath::Max(SlackMax, T); }
	TestTrue(FString::Printf(TEXT("slack rope max tension %.1f should be ~0"), SlackMax), SlackMax < 1.0f);
	return true;
}

// list of broad-phase candidates (selected by DetectContacts and selected by SolveContacts/SolveSegmentContacts for each pass)
// iteration reuse change the results? It nails two things:
//  (1) The result should be the same even if distant colliders that are not in contact are inserted in between — candidate slot → collider index
//      If the mapping is misaligned (a mistake that could not have been made in the full loop era), the wrong collider is queried, and this is where the difference occurs.
//  (2) If the collider overlapping on one node exceeds MaxPerItem, the candidate must be given up and all falls back to a loop —
//      If you only look at the MaxPerItem items in front, you will miss the rear collider and go through them.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverColliderCandidateTest,
	"DynamicRope.Solver.ColliderCandidateFiltering",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverColliderCandidateTest::RunTest(const FString& Parameters)
{
	FRopeSolverConfig Config = MakeStiffConfig();
	Config.Gravity = FVector(0.0f, 0.0f, -980.0f);
	Config.CollisionRadius = 2.0f;
	const FRopeXPBDSolver Solver;

	// (1) Distant decoy colliders should not affect the results.
	{
		// Apply strong damping so that the swing stops within 90 frames and settles on the ball — The comparison below shows that the "rope actually
		// It is a setup for presentation because it has meaning when it spans a phrase (it applies equally to both runs, so it has no effect on comparison).
		// Damping is 60fps per-frame rate, so 0.3 => 0.7^60 residual rate per second, that is, close to immediate stationary.
		FRopeSolverConfig DrapeConfig = Config;
		DrapeConfig.Damping = 0.3f;
		// Both ends are pinned + spare length (span 160 < rest 220) → The center stretches over the sphere to create actual contact.
		auto MakeDrapedRope = []() -> FRopeSimState
		{
			FRopeSimState S = RopeTest::MakeStraightRope(12, 220.0f);
			for (int32 i = 0; i < S.Num(); ++i)
			{
				const float Alpha = static_cast<float>(i) / static_cast<float>(S.Num() - 1);
				const FVector P(-80.0f + 160.0f * Alpha, 0.0f, 60.0f);
				S.Positions[i] = P;
				S.PrevPositions[i] = P;
			}
			S.bStartPinned = true;
			S.StartPinPrev = S.Positions[0];
			S.StartPinTarget = S.Positions[0];
			S.InvMass[0] = 0.0f;
			S.InvMass[S.Num() - 1] = 0.0f;
			return S;
		};

		// Arrange them so that they overlap each other — if you drop them, the hanging rope will just slip through the gap in the middle.
		constexpr float SphereRadius = 25.0f;
		RopeTest::FSphereMockCollider RealA(FVector(-20.0f, 0.0f, 0.0f), SphereRadius, FName("a"));
		RopeTest::FSphereMockCollider RealB(FVector( 20.0f, 0.0f, 0.0f), SphereRadius, FName("b"));

		// decoys that are far from rope AABB and are not even considered candidates.
		TArray<RopeTest::FSphereMockCollider> Decoy;
		Decoy.Reserve(8);
		for (int32 i = 0; i < 8; ++i)
		{
			Decoy.Add(RopeTest::FSphereMockCollider(FVector(0.0f, 5000.0f + i * 100.0f, 0.0f), 10.0f, FName("far")));
		}

		const TArray<IRopeCollider*> Bare = { &RealA, &RealB };

		// Insert the decoy back and forth so that the actual collider is at a high index rather than 0/1 (creates a slot≠index situation).
		TArray<IRopeCollider*> Mixed;
		for (int32 i = 0; i < 4; ++i) { Mixed.Add(&Decoy[i]); }
		Mixed.Add(&RealA);
		for (int32 i = 4; i < 8; ++i) { Mixed.Add(&Decoy[i]); }
		Mixed.Add(&RealB);

		FRopeSimState A = MakeDrapedRope();
		FRopeSimState B = MakeDrapedRope();
		for (int32 Frame = 0; Frame < 90; ++Frame)
		{
			Solver.Step(A, DrapeConfig, Bare, 1.0f / 60.0f);
			Solver.Step(B, DrapeConfig, Mixed, 1.0f / 60.0f);
		}

		// The comparison is meaningful only if the rope actually spans a sphere (if both are in Free fall, they are self-evidently equal).
		bool bTouched = false;
		for (int32 i = 0; i < A.Num(); ++i)
		{
			const float DA = static_cast<float>(FVector::Dist(A.Positions[i], RealA.Center));
			const float DB = static_cast<float>(FVector::Dist(A.Positions[i], RealB.Center));
			const float Surface = SphereRadius + DrapeConfig.CollisionRadius + 1.0f;
			bTouched |= (DA < Surface) || (DB < Surface);
		}
		TestTrue(TEXT("rope actually rests on the spheres (otherwise the comparison is vacuous)"), bTouched);

		float MaxDelta = 0.0f;
		for (int32 i = 0; i < A.Num(); ++i)
		{
			MaxDelta = FMath::Max(MaxDelta, static_cast<float>(FVector::Dist(A.Positions[i], B.Positions[i])));
		}
		TestTrue(FString::Printf(TEXT("distant decoy colliders must not change the result (max delta %.4f cm)"), MaxDelta),
			MaxDelta < 0.01f);
		TestFalse(TEXT("no NaN (decoy run)"), RopeTest::AnyNaN(B));
	}

	// (2) Candidate cap exceeded → full loop fallback.
	{
		// 16 overlapping spheres in the same place. Make the last one bigger, so if you only saw the MaxPerItems before, you'd miss that big sphere.
		// Fallback operation is determined by whether the node sinks to the small sphere surface (z≈5).
		constexpr int32 NumSpheres = 16;
		static_assert(NumSpheres > FRopeColliderCandidates::MaxPerItem, "overflow 경로를 타야 의미가 있는 테스트");
		TArray<RopeTest::FSphereMockCollider> Spheres;
		Spheres.Reserve(NumSpheres);
		for (int32 i = 0; i < NumSpheres; ++i)
		{
			Spheres.Add(RopeTest::FSphereMockCollider(FVector::ZeroVector,
				(i == NumSpheres - 1) ? 8.0f : 3.0f, FName("stack")));
		}
		TArray<IRopeCollider*> Colliders;
		Colliders.Reserve(NumSpheres);
		for (RopeTest::FSphereMockCollider& S : Spheres) { Colliders.Add(&S); }

		// 2node fixture like StaticContactNoRebound: pin node0, node1 drops onto pile of spheres.
		FRopeSimState Sim = RopeTest::MakeStraightRope(2, 200.0f);
		Sim.Positions[0] = FVector(0.0f, 0.0f, 207.0f);
		Sim.PrevPositions[0] = Sim.Positions[0];
		Sim.bStartPinned = true;
		Sim.StartPinPrev = Sim.Positions[0];
		Sim.StartPinTarget = Sim.Positions[0];
		Sim.InvMass[0] = 0.0f;
		Sim.Positions[1] = FVector(0.0f, 0.0f, 100.0f);
		Sim.PrevPositions[1] = Sim.Positions[1];

		for (int32 Frame = 0; Frame < 180; ++Frame)
		{
			Solver.Step(Sim, Config, Colliders, 1.0f / 60.0f);
		}

		// Surface of largest sphere = 8 + CollisionRadius 2 = 10. If fallback is broken, it sits near 5.
		const float Z = static_cast<float>(Sim.Positions[1].Z);
		TestTrue(FString::Printf(TEXT("overflowed node still sees the last collider (z=%.2f, expected ~10)"), Z),
			Z > 9.0f && Z < 11.0f);
		TestFalse(TEXT("no NaN (overflow run)"), RopeTest::AnyNaN(Sim));
	}

	return true;
}

// Contact Solve Interval Contract: No matter what interval is given, in the *last* iteration of each collision pass
// The contact must be released. Otherwise distance/bending will substep without a chance to push back what was last pulled.
// ends and remains in a penetrating state. The distance constraint verifies the fixture pulling the node to the *center* of the collider —
// If it is a naive implementation that counts the cycle from the beginning of pass, it breaks here because the end is missing from Interval > Iterations.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSolverContactCadenceTest,
	"DynamicRope.Solver.ContactSolveIntervalKeepsLastIteration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSolverContactCadenceTest::RunTest(const FString& Parameters)
{
	// Sphere radius 5 + node thickness 2 → surface z = 7. Node0 is pinned at z=200 and the rest length is also 200, so the distance constraint is
	// Pulls node1 to z=0 (center of sphere) → The most sensitive condition to cadence, directly competing with collision.
	constexpr float SurfaceZ = 7.0f;

	auto DeepestZ = [](int32 Interval) -> float
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(2, 200.0f);
		Sim.Positions[0] = FVector(0.0f, 0.0f, 200.0f);
		Sim.PrevPositions[0] = Sim.Positions[0];
		Sim.bStartPinned = true;
		Sim.StartPinPrev = Sim.Positions[0];
		Sim.StartPinTarget = Sim.Positions[0];
		Sim.InvMass[0] = 0.0f;
		Sim.Positions[1] = FVector(0.0f, 0.0f, 40.0f);
		Sim.PrevPositions[1] = Sim.Positions[1];

		RopeTest::FSphereMockCollider Sphere(FVector::ZeroVector, 5.0f, FName("static"));
		const TArray<IRopeCollider*> Colliders = { &Sphere };

		FRopeSolverConfig Config = MakeStiffConfig();
		Config.Gravity = FVector(0.0f, 0.0f, -980.0f);
		Config.CollisionRadius = 2.0f;
		Config.ContactSolveInterval = Interval;

		const FRopeXPBDSolver Solver;
		float Deepest = 1.0e30f;
		for (int32 Frame = 0; Frame < 180; ++Frame)
		{
			Solver.Step(Sim, Config, Colliders, 1.0f / 60.0f);
			// Only the section after settling is boned (the initial fall is caught by the swept CCD).
			if (Frame >= 60)
			{
				Deepest = FMath::Min(Deepest, static_cast<float>(Sim.Positions[1].Z));
			}
		}
		return Deepest;
	};

	// MakeStiffConfig Iterations=8. Check even larger cycles (= GPU cadence that falls to 1 per pass).
	for (const int32 Interval : { 1, 2, 3, 8, 16 })
	{
		const float Deepest = DeepestZ(Interval);
		TestTrue(FString::Printf(TEXT("interval=%d keeps the node outside the surface (deepest z=%.3f, surface %.1f)"),
			Interval, Deepest, SurfaceZ), Deepest > SurfaceZ - 0.5f);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

