// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrapController.h"
#include "RopeTestHelpers.h"

// ComputePull: whether it produces, as data, the direction from the first anchor on the hand side towards
// its adjacent node on that side, plus that segment's tension.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapComputePullTest,
	"DynamicRope.Wrap.ComputePullDirectionAndTension",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapComputePullTest::RunTest(const FString& Parameters)
{
	// The nodes are spaced along positive X. With anchors at nodes five and three, the first anchor on the
	// hand side is node three.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	Sim.SegmentTension.SetNumZeroed(Sim.Num() - 1);
	// The segment between the anchor and its adjacent node on the hand side.
	Sim.SegmentTension[2] = 1234.0f;
	// A different segment, which must not be selected.
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

	// With a corner threshold of 30 degrees, a straight rope walks all the way to the hand at node zero and
	// its direction is exactly the chord.
	const float BendDeg = 30.0f;
	FRopePullSample Pull;
	TestTrue(TEXT("ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
	TestTrue(TEXT("pull sample valid"), Pull.bValid);
	TestEqual(TEXT("hand-side head anchor wins"), Pull.AnchorNode, 3);
	TestTrue(TEXT("bone attributed"), Pull.Bone == FName("arm"));
	// On a straight rope every node on the hand side of the anchor lies along negative X, so the look-ahead
	// coincides with the chord.
	TestTrue(FString::Printf(TEXT("direction %s points toward hand (-X)"), *Pull.Direction.ToCompactString()),
		Pull.Direction.Equals(FVector(-1, 0, 0), 0.01f));
	TestEqual(TEXT("tension from anchor-hand segment"), Pull.Tension, 1234.0f);

	// With the only anchor at node zero, the hand pin, there is no segment on the hand side and it is
	// invalid.
	FRopeWrapController WrapAtHand;
	WrapAtHand.State.BoneName = FName("arm");
	FRopeSurfaceAnchor HandAnchor;
	HandAnchor.NodeIndex = 0;
	WrapAtHand.State.Anchors.Add(HandAnchor);
	FRopePullSample InvalidPull;
	TestFalse(TEXT("anchor at hand node yields no pull"), WrapAtHand.ComputePull(Sim, BendDeg, InvalidPull));

	// A bent free span, as at a wall edge: the first leg climbs upwards from the anchor at node four, and at
	// the corner, node two, it turns horizontally towards the hand. The look-ahead direction has to follow
	// the rope's path, meaning the first leg, and be clearly different from the straight anchor-to-hand
	// chord, which is diagonal and cuts across the corner. That difference is the fix for the chord passing
	// through a wall on a rope caught on one.
	FRopeSimState Bent;
	Bent.Positions = {
		// Node zero, the hand.
		FVector(-40, 0, 40),
		// 1
		FVector(-20, 0, 40),
		// Node two, the corner.
		FVector(  0, 0, 40),
		// Node three, on the first leg.
		FVector(  0, 0, 20),
		// Node four, the anchor.
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
	// Walking the first leg upwards from node four, it detects the right-angled turn at node two, stops,
	// and gives an upward direction along the rope's path rather than the chord.
	TestTrue(FString::Printf(TEXT("bent direction %s follows first leg (+Z)"), *BentPull.Direction.ToCompactString()),
		BentPull.Direction.Equals(FVector(0, 0, 1), 0.01f));
	// The diagonal, which cuts through the corner.
	const FVector Chord = (Bent.Positions[0] - Bent.Positions[4]).GetSafeNormal();
	TestFalse(TEXT("bent direction is NOT the straight chord"), BentPull.Direction.Equals(Chord, 0.05f));
	return true;
}

// The whole-chain taut observations from ComputePull: whether the sum of the corner-to-corner leg chords,
// clamped per leg to its rest length, the free span's rest length, and the minimum transmitted tension
// together distinguish whether the rope is straightened out. Four cases are covered: taut and straight,
// slack through compression, taut but caught on a corner, and the regression case of a moving target where
// only the anchor leg is stretched while the rest is slack.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrapComputePullChainTautTest,
	"DynamicRope.Wrap.ComputePullChainTautObservables",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrapComputePullChainTautTest::RunTest(const FString& Parameters)
{
	const float BendDeg = 30.0f;
	auto MakeWrap = [](int32 AnchorNode)
	{
		FRopeWrapController Wrap;
		Wrap.State.BoneName = FName("arm");
		FRopeSurfaceAnchor A; A.NodeIndex = AnchorNode; A.Bone = FName("arm");
		Wrap.State.Anchors.Add(A);
		return Wrap;
	};

	// One: a straight, taut rope. With the anchor at node five and a segment length of twenty, the rest
	// length is 100 and the chord sum is the distance from the anchor to the hand, also 100.
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
		// The minimum transmitted tension looks only at the free span, so a low tension on a segment beyond
		// the anchor is ignored.
		Sim.SegmentTension.Init(500.0f, Sim.Num() - 1);
		Sim.SegmentTension[2] = 50.0f; // The minimum within the free span.
		Sim.SegmentTension[5] = 1.0f;  // Beyond the anchor, which must not be counted.
		FRopeWrapController Wrap = MakeWrap(5);
		FRopePullSample Pull;
		TestTrue(TEXT("straight ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
		TestEqual(TEXT("straight rest length"), Pull.FreeRestLen, 100.0f, 0.1f);
		TestEqual(TEXT("straight chord sum equals rest (taut)"), Pull.TautChordLen, 100.0f, 0.1f);
		TestEqual(TEXT("min transmitted tension reads the hand-side span only"), Pull.MinFreeTension, 50.0f, 0.1f);
		TestEqual(TEXT("straight rope has no sag"), Pull.MaxLegSag, 0.0f, 0.1f);
		TestEqual(TEXT("straight raw chord equals clamped chord"), Pull.PathChordLen, 100.0f, 0.1f);
	}

	// Two: a compressed, slack rope. The nodes are spaced at half their rest length along a straight line,
	// so the chord sum is half the rest length with no bending at all.
	// XPBD does not resist compression, so this is the archetypal shape of a rope that is not straightened
	// out.
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(5, 80.0f);
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.Positions[i] = FVector(10.0f * i, 0, 0);
			Sim.PrevPositions[i] = Sim.Positions[i];
		}
		FRopeWrapController Wrap = MakeWrap(4);
		FRopePullSample Pull;
		TestTrue(TEXT("compressed ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
		TestEqual(TEXT("compressed rest length"), Pull.FreeRestLen, 80.0f, 0.1f);
		TestEqual(TEXT("compressed chord sum is half the rest (slack)"), Pull.TautChordLen, 40.0f, 0.1f);
		TestEqual(TEXT("compressed raw chord matches (slack -> constraint C negative)"), Pull.PathChordLen, 40.0f, 0.1f);
	}

	// Three: caught on a corner but taut in both legs. The two legs sum to the rest length exactly.
	// A corner is not penalized: a taut rope caught on a wall is recognized as taut and the tether and pull
	// fire exactly as before.
	{
		FRopeSimState Bent;
		Bent.Positions = {
			FVector(-40, 0, 40), FVector(-20, 0, 40), FVector(0, 0, 40),
			FVector(0, 0, 20), FVector(0, 0, 0),
		};
		Bent.PrevPositions = Bent.Positions;
		Bent.SegmentLength = 20.0f;
		Bent.InvMass.Init(1.0f, 5);
		FRopeWrapController Wrap = MakeWrap(4);
		FRopePullSample Pull;
		TestTrue(TEXT("bent ComputePull succeeds"), Wrap.ComputePull(Bent, BendDeg, Pull));
		TestEqual(TEXT("bent rest length"), Pull.FreeRestLen, 80.0f, 0.1f);
		TestEqual(TEXT("bent leg chords sum to rest (taut around a corner)"), Pull.TautChordLen, 80.0f, 0.1f);
		// Note the blind spot: slack folded into an L shape in mid-air reads geometrically as the same rest
		// length. Distinguishing a wall corner, where tension is present throughout, from a kink in the air,
		// where it is zero somewhere, is what the minimum transmitted tension is for. Here it is before the
		// solve, so the array is empty and it reads as zero.
		TestEqual(TEXT("bent MinFreeTension is zero without solved tensions"), Pull.MinFreeTension, 0.0f, 0.01f);
	}

	// Four: the regression case with a moving target. Only the leg adjacent to the anchor is stretched
	// straight, with a segment longer than its rest length, while the tail bunches and sags. Looking at that
	// sub-leg alone it appears taut, but the stretched leg's chord is clamped to that leg's rest length, so a
	// stretch cannot mask slack, and the zero tension across the kinked stretch drives the minimum
	// transmitted tension to zero. Those two observations are exactly what identifies the symptom of a slack
	// rope being dragged along.
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(6, 100.0f);
		Sim.Positions = {
			FVector(32, 0, -18), FVector(36, 0, -14), FVector(40, 0, -8),
			FVector(46, 0, 0), FVector(73, 0, 0), FVector(100, 0, 0),
		};
		Sim.PrevPositions = Sim.Positions;
	// Tension exists only on the stretched span near the anchor; the kinked span has none.
		Sim.SegmentTension = { 0.0f, 0.0f, 0.0f, 800.0f, 900.0f };
		FRopeWrapController Wrap = MakeWrap(5);
		FRopePullSample Pull;
		TestTrue(TEXT("stretched-leg ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
		TestEqual(TEXT("stretched-leg rest length"), Pull.FreeRestLen, 100.0f, 0.1f);
		// The first leg still stops just before the tail sags, at node three, so the direction and the aim are
		// produced exactly as before.
		TestEqual(TEXT("first leg still aims at the bend"), Pull.AimNode, 3);
		// After clamping, the sum is the clamped leg plus the sagging tail, which is well below the rest
		// length; counting the stretch at its full length would have hidden that.
		TestTrue(FString::Printf(TEXT("chord sum %.1f stays well below rest 100 (slack chain)"), Pull.TautChordLen),
			Pull.TautChordLen < 70.0f);
		TestEqual(TEXT("crumpled span zeroes the min transmitted tension"), Pull.MinFreeTension, 0.0f, 0.01f);
		// The unclamped sum counts the stretched leg in full, yet is still below the rest length: a partial
		// stretch alone never makes the constraint violation positive, since the whole rope has to be
		// straightened before lambda appears.
		TestEqual(TEXT("raw chord counts the stretched leg yet stays below rest"), Pull.PathChordLen, 76.8f, 0.5f);
	}

	// Five: a gentle catenary sag, meaning a bend below the corner threshold and therefore a single leg. The
	// chord ratio responds only to the square of the sag, so even a noticeable sag passes at 99 percent, and
	// that insensitivity is why a rope reading as almost fully taut could still sag visibly. The maximum leg
	// sag reports the sag in centimetres directly, and is the observation behind the sag gate.
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(5, 80.0f);
		Sim.Positions = {
			FVector(79.0f, 0, 0), FVector(59.5f, 0, -6), FVector(39.5f, 0, -9),
			FVector(19.5f, 0, -6), FVector(0, 0, 0),
		};
		Sim.PrevPositions = Sim.Positions;
		FRopeWrapController Wrap = MakeWrap(4);
		FRopePullSample Pull;
		TestTrue(TEXT("sagging ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
	// The bend is gentle enough that the walk reaches the hand as one leg, so the chord ratio is about 99
	// percent and the ratio gate passes.
		TestEqual(TEXT("gentle sag still walks to the hand"), Pull.AimNode, 0);
		TestTrue(FString::Printf(TEXT("chord ratio %.3f stays above 0.97 (ratio gate blind)"),
			Pull.TautChordLen / Pull.FreeRestLen), Pull.TautChordLen / Pull.FreeRestLen > 0.97f);
		TestEqual(TEXT("max leg sag reads the visible dip"), Pull.MaxLegSag, 9.0f, 0.5f);
	}

	// Six: taut and stretched, with the node spacing above the rest length. This is where the two
	// observations diverge: the taut chord length is clamped per leg to its rest length and stays there, as
	// the gate's convention requires, while the path chord length reports the real path and makes the
	// constraint violation positive, which is the condition for lambda to fire.
	{
		FRopeSimState Sim = RopeTest::MakeStraightRope(6, 100.0f);
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.Positions[i] = FVector(22.0f * i, 0, 0);
			Sim.PrevPositions[i] = Sim.Positions[i];
		}
		FRopeWrapController Wrap = MakeWrap(5);
		FRopePullSample Pull;
		TestTrue(TEXT("stretched ComputePull succeeds"), Wrap.ComputePull(Sim, BendDeg, Pull));
		TestEqual(TEXT("stretched rest length"), Pull.FreeRestLen, 100.0f, 0.1f);
		TestEqual(TEXT("clamped chord stays at rest (gate contract)"), Pull.TautChordLen, 100.0f, 0.1f);
		TestEqual(TEXT("raw chord reads the stretched path (constraint C positive)"), Pull.PathChordLen, 110.0f, 0.1f);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
