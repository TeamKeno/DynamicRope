// Copyright Epic Games, Inc. All Rights Reserved.
//
// Unit tests for the pierce binding mode, which belongs to GuaranteedWrap alone and establishes a single
// anchor at the aim hit point.
// They pin eight contracts: the permitted combinations of resolve mode and binding, the throw phase gate
// restricting GuaranteedWrap to Loaded, the preview builder producing a single anchor, the requirement
// that a preview target come from an aim hit, the single-anchor commit through BeginWrap, entering and
// leaving the guided throw phase, the rejection of targets hidden behind world geometry, and the clamp that
// keeps a throw into open space from ending up under the floor.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Core/RopeTypes.h"
#include "Logic/RopeWrapController.h"
#include "Logic/RopeThrowPreviewBuilder.h"
#include "Components/SkeletalMeshComponent.h"
#include "RopeComponent.h"
#include "RopeTestHelpers.h"

namespace
{
	USkeletalMeshComponent* MakePierceMockMesh()
	{
		return NewObject<USkeletalMeshComponent>();
	}
}

// The throw phase gate: GuaranteedWrap can be thrown from Loaded alone, while the other modes have no
// phase gate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceThrowPhaseGateTest,
	"DynamicRope.Pierce.ThrowPhaseGateContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceThrowPhaseGateTest::RunTest(const FString& Parameters)
{
	using namespace RopeWrapModes;

	// ERopePhase has no count or maximum sentinel, so a newly added phase has to be added here by hand.
	static const ERopePhase AllPhases[] = {
		ERopePhase::Free, ERopePhase::Flight, ERopePhase::Contacting, ERopePhase::Wrapping,
		ERopePhase::Wrapped, ERopePhase::Releasing, ERopePhase::GuidedThrow, ERopePhase::Loaded
	};

	// GuaranteedWrap is restricted to Loaded.
	TestTrue(TEXT("GuaranteedWrap can be thrown from Loaded"),
		CanThrowInPhase(ERopeWrapResolveMode::GuaranteedWrap, ERopePhase::Loaded));
	for (const ERopePhase Phase : AllPhases)
	{
		if (Phase == ERopePhase::Loaded)
		{
			continue;
		}
		TestFalse(*FString::Printf(TEXT("GuaranteedWrap cannot be thrown from phase %d, which is not Loaded"), static_cast<int32>(Phase)),
			CanThrowInPhase(ERopeWrapResolveMode::GuaranteedWrap, Phase));
	}

	// The other modes have no phase gate and are therefore true in every phase.
	// Simplifying the predicate to a direct phase comparison would fail here, which is the regression line
	// stopping the other modes being silently blocked.
	for (const ERopePhase Phase : AllPhases)
	{
		TestTrue(*FString::Printf(TEXT("FullSimulation has no gate in phase %d"), static_cast<int32>(Phase)),
			CanThrowInPhase(ERopeWrapResolveMode::FullSimulation, Phase));
		TestTrue(*FString::Printf(TEXT("AssistedJudged has no gate in phase %d"), static_cast<int32>(Phase)),
			CanThrowInPhase(ERopeWrapResolveMode::AssistedJudged, Phase));
	}
	return true;
}

// The pierce preview builder: whether it produces a single latch anchor from the aim hit candidate, with no
// wrapping helix.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceSingleAnchorPreviewTest,
	"DynamicRope.Pierce.PreviewYieldsSingleAnchor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceSingleAnchorPreviewTest::RunTest(const FString& Parameters)
{
	// The nodes are evenly spaced along X. The aim hits the spine bone, which produces a single anchor.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();

	FRopeThrowPreviewBuilder::FInput Input;
	Input.Sim = &Sim;
	Input.RopeRadius = 2.0f;
	Input.ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Input.ThrowContext.Origin = FVector::ZeroVector;
	Input.ThrowContext.FrameForward = FVector(1, 0, 0);
	Input.ThrowContext.FrameUp = FVector(0, 0, 1);
	Input.ThrowContext.bHasAimGuideHit = true;
	Input.ThrowContext.AimGuideMesh = Mesh;
	Input.ThrowContext.AimGuideBone = FName("spine");
	Input.ThrowContext.AimGuideHitWorldPos = FVector(60, 0, 0);
	Input.ThrowContext.AimGuideNormal = FVector(0, 0, 1);

	FRopePreparedThrowPreview Prepared;
	FString Failure;
	const bool bBuilt = FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, Prepared, &Failure);

	TestTrue(FString::Printf(TEXT("pierce prepared preview builds (%s)"), *Failure), bBuilt);
	TestTrue(TEXT("prepared valid"), Prepared.IsValid());
	TestEqual(TEXT("pierce produces a single anchor"), Prepared.Anchors.Num(), 1);
	TestTrue(TEXT("the anchor's bone is the target bone"), Prepared.Bone == FName("spine"));
	TestTrue(TEXT("the anchor's mesh is the target mesh"), Prepared.Mesh.Get() == Mesh);
	if (Prepared.Anchors.Num() == 1)
	{
		TestTrue(TEXT("the single anchor is the latch anchor's node"), Prepared.Anchors[0].NodeIndex == Prepared.LatchAnchor.NodeIndex);
		// Pierce embeds the tip, which is the last node, so the anchor has to be at the end of the rope for
		// the tip mesh to land at the embed point and the end not to sag.
		TestEqual(TEXT("the pierce anchor is the rope's end node, where the tip is"),
			Prepared.Anchors[0].NodeIndex, Sim.Num() - 1);
	}
	TestTrue(TEXT("the render preview is valid, as a straight centreline"), Prepared.RenderPreview.IsValid());
	return true;
}

// The single-anchor commit: whether BeginWrap pins a one-anchor seed straight to wrapped, regardless of how
// many anchors the commit path normally carries.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceSingleAnchorBeginWrapTest,
	"DynamicRope.Pierce.SingleAnchorBeginWrap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceSingleAnchorBeginWrapTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	const USkeletalMeshComponent* Mesh = MakePierceMockMesh();

	// A single-anchor seed of the same shape pierce produces.
	const int32 PierceNode = 3;
	FRopeWrapState Seed;
	Seed.BoneName = FName("spine");
	Seed.Mesh = Mesh;
	{
		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = PierceNode;
		Anchor.Bone = FName("spine");
		Anchor.Mesh = Mesh;
		Anchor.LocalSurfacePosition = Sim.Positions[PierceNode]; // mock mesh transform = identity
		Seed.Anchors.Add(Anchor);

		FRopeLatchNode Latch;
		Latch.NodeIndex = PierceNode;
		Latch.Bone = FName("spine");
		Seed.Latched.Add(Latch);
	}

	FRopeWrapController Wrap;
	FRopeNodeOverrideFrame Frame;
	Wrap.BeginWrap(Sim, Seed, Frame);

	TestTrue(TEXT("a single anchor establishes a wrap"), Wrap.State.IsWrapped());
	TestEqual(TEXT("the one anchor is kept"), Wrap.State.Anchors.Num(), 1);
	TestTrue(TEXT("the committed bone is the spine"), Wrap.State.BoneName == FName("spine"));
	// The embedded node is pinned with an inverse mass of zero, so it is driven by the bone rather than the
	// solver.
	TestTrue(TEXT("an override frame was produced"), Frame.HasAny());
	if (Frame.InvMass.IsValidIndex(PierceNode))
	{
		TestEqual(TEXT("the embedded node is pinned with a zero inverse mass"), Frame.InvMass[PierceNode], 0.0f);
	}
	return true;
}

// The Loaded transition: a GuaranteedWrap rope moves from Free to Loaded through EnterLoaded(), while other
// modes treat it as a no-op. Throwing is possible from Loaded alone.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceEnterLoadedTest,
	"DynamicRope.Pierce.EnterLoadedTransition",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceEnterLoadedTest::RunTest(const FString& Parameters)
{
	// A GuaranteedWrap rope starts in Free and enters Loaded.
	URopeComponent* Guaranteed = NewObject<URopeComponent>();
	Guaranteed->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	TestEqual(TEXT("the default phase is Free"), Guaranteed->GetPhase(), ERopePhase::Free);
	Guaranteed->EnterLoaded();
	TestEqual(TEXT("EnterLoaded moves a GuaranteedWrap rope to Loaded"), Guaranteed->GetPhase(), ERopePhase::Loaded);

	// On an assisted rope EnterLoaded is a no-op and it stays in Free, since Loaded belongs to
	// GuaranteedWrap alone.
	URopeComponent* Assisted = NewObject<URopeComponent>();
	Assisted->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	Assisted->EnterLoaded();
	TestEqual(TEXT("EnterLoaded is a no-op in AssistedJudged"), Assisted->GetPhase(), ERopePhase::Free);

	// Re-entering Loaded from Loaded keeps it there, which is permitted; from any other phase it does nothing.
	Guaranteed->EnterLoaded();
	TestEqual(TEXT("re-entering from Loaded stays in Loaded"), Guaranteed->GetPhase(), ERopePhase::Loaded);
	return true;
}

// The phase contract for entering and leaving a guided throw: Loaded, then a prepared throw, then
// GuidedThrow, then a manual release into Releasing.
// That manual release is the only public route into the guided throw branch of the release path, meaning a
// release before the commit.
//
// A limit of this test's scope, stated plainly: aborting mid-throw, and the release event it fires, cannot
// be verified here. The functions that would drive it are private with only the subsystem as a friend; the
// release event is a dynamic delegate and needs a UObject listener, which this test module has none of; and
// with no world the central signal is unreachable in the first place. Verifying those events is the
// play-in-editor checklist's job.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceGuidedThrowEntryTest,
	"DynamicRope.Pierce.GuidedThrowEntryAndManualRelease",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceGuidedThrowEntryTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Rope->RopeLength = 140.0f;
	Rope->NumParticles = 8;

	// A prepared throw for a successful aim, assembled with no world, on the same path as the preview test
	// above.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();

	FRopeThrowPreviewBuilder::FInput Input;
	Input.Sim = &Sim;
	Input.RopeRadius = 2.0f;
	Input.ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Input.ThrowContext.Origin = FVector::ZeroVector;
	Input.ThrowContext.FrameForward = FVector(1, 0, 0);
	Input.ThrowContext.FrameUp = FVector(0, 0, 1);
	Input.ThrowContext.bHasAimGuideHit = true;
	Input.ThrowContext.AimGuideMesh = Mesh;
	Input.ThrowContext.AimGuideBone = FName("spine");
	Input.ThrowContext.AimGuideHitWorldPos = FVector(60, 0, 0);
	Input.ThrowContext.AimGuideNormal = FVector(0, 0, 1);

	FRopePreparedThrowPreview Prepared;
	FString Failure;
	if (!TestTrue(FString::Printf(TEXT("prepared preview builds (%s)"), *Failure),
		FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, Prepared, &Failure)))
	{
		return false;
	}

	// Throwing is impossible outside Loaded, which checks the real path of the phase gate contract.
	TestFalse(TEXT("a prepared throw is refused from Free"), Rope->ThrowWithPreparedPreview(Prepared));
	TestEqual(TEXT("the phase is unchanged after the refusal"), Rope->GetPhase(), ERopePhase::Free);

	Rope->EnterLoaded();
	TestTrue(TEXT("a prepared throw succeeds from Loaded"), Rope->ThrowWithPreparedPreview(Prepared));
	TestEqual(TEXT("prepared throw → GuidedThrow"), Rope->GetPhase(), ERopePhase::GuidedThrow);

	// A manual release mid-throw moves it to Releasing. It is before the commit, so no central signal should
	// fire, although that cannot be observed without a world.
	Rope->ReleaseWrap();
	TestEqual(TEXT("releasing during a guided throw moves to Releasing"), Rope->GetPhase(), ERopePhase::Releasing);
	return true;
}
// A preview target is decided by the aim hit alone. Sweeping the area around the throw direction for a
// target when there is no aim hit would embed the rope in a neighbouring target that was never aimed at, so
// the preview does not succeed even with a target immediately beside it.
// That holds both when the aim ray missed and when there was no aiming flow at all, as on a direct
// Blueprint or AI call.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceAimHitRequiredForPreviewTest,
	"DynamicRope.Pierce.AimHitRequiredForPreview",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceAimHitRequiredForPreviewTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();
	// A target placed off the aim axis, at a distance from the origin equal to half the rope's length, which
	// is exactly where an implementation sweeping around the throw direction would pick it up.
	FCapsuleCollider Target(FVector(0, -20, 70), FVector(0, 20, 70), 25.0f, FName("spine"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Target };

	for (const bool bAimRayEvaluated : { true, false })
	{
		FRopeThrowPreviewBuilder::FInput Input;
		Input.Sim = &Sim;
		Input.RopeRadius = 2.0f;
		Input.Colliders = &Colliders;
		Input.ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
		Input.ThrowContext.Origin = FVector::ZeroVector;
		Input.ThrowContext.FrameForward = FVector(1, 0, 0);
		Input.ThrowContext.FrameUp = FVector(0, 0, 1);
		Input.ThrowContext.bAimRayEvaluated = bAimRayEvaluated; // True means the aim missed and false means there was no aiming flow.
		// The aim guide hit flag stays false, which is the premise shared by both branches.

		FRopePreparedThrowPreview Prepared;
		FString Failure;
		TestFalse(*FString::Printf(TEXT("with no aim hit the preview fails even with a target beside it (aimRayEvaluated=%d)"),
			bAimRayEvaluated ? 1 : 0),
			FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, Prepared, &Failure));
		TestFalse(TEXT("the prepared throw is invalid"), Prepared.IsValid());
	}
	return true;
}

namespace
{
	/** A stand-in for the engine line trace URopeComponent injects: it blocks at a fixed distance along the
	 *  ray. It is what lets the two tests below run with no world. */
	FRopeAimTargeting::FQueryContext MakeBlockerContext(const TArray<IRopeCollider*>* Colliders,
		float BlockDistance, float QueryRadius)
	{
		FRopeAimTargeting::FQueryContext Ctx;
		Ctx.Colliders = Colliders;
		Ctx.FallbackQueryRadius = QueryRadius;
		if (BlockDistance > 0.0f)
		{
			Ctx.TraceWorldBlocker = [BlockDistance](const FVector& Start, const FVector& End,
				FVector& OutBlockPoint, float& OutDistance)
			{
				OutBlockPoint = Start + (End - Start).GetSafeNormal() * BlockDistance;
				OutDistance = BlockDistance;
				return true;
			};
		}
		return Ctx;
	}
}

// Aiming through opaque world geometry. A target behind a wall or under the floor is not aimable, so the
// guaranteed throw never starts towards something it cannot reach in a straight line. The blocker itself is
// reported as blocked instead, which is what the aiming HUD draws.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceOccludedAimTargetTest,
	"DynamicRope.Pierce.OccludedAimTargetRejected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceOccludedAimTargetTest::RunTest(const FString& Parameters)
{
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();
	// A wrappable target on the aim axis, centred 200 cm ahead.
	FCapsuleCollider Target(FVector(200, -20, 0), FVector(200, 20, 0), 25.0f, FName("spine"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Target };

	const FVector Origin = FVector::ZeroVector;
	const FVector AimDir = FVector(1, 0, 0);
	const float QueryRadius = 3.0f;
	auto PermitAll = [](const USceneComponent*, FName) { return true; };

	// With nothing in the way the target is acquired, which is the baseline the gate must not disturb.
	{
		FRopeAimRayHitResult Hit;
		FRopeAimRayHitResult Blocked;
		const FRopeAimTargeting::FQueryContext Ctx = MakeBlockerContext(&Colliders, /*BlockDistance*/ 0.0f, QueryRadius);
		TestTrue(TEXT("an unobstructed target is acquired"),
			FRopeAimTargeting::FindAimRayBoneHit(Ctx, Origin, AimDir, 400.0f, QueryRadius, 2.0f,
				PermitAll, Hit, &Blocked));
		TestTrue(TEXT("the acquired bone is the spine"), Hit.Bone == FName("spine"));
	}

	// A wall in front of the target hides it: no hit, and the wall is reported as the blocked result.
	{
		FRopeAimRayHitResult Hit;
		FRopeAimRayHitResult Blocked;
		const FRopeAimTargeting::FQueryContext Ctx = MakeBlockerContext(&Colliders, /*BlockDistance*/ 100.0f, QueryRadius);
		TestFalse(TEXT("a target behind a wall is not acquired"),
			FRopeAimTargeting::FindAimRayBoneHit(Ctx, Origin, AimDir, 400.0f, QueryRadius, 2.0f,
				PermitAll, Hit, &Blocked));
		TestFalse(TEXT("no hit is reported"), Hit.bHit);
		TestTrue(TEXT("the wall is reported as blocked"), Blocked.bHit);
		TestEqual(TEXT("the blocked distance is the wall's"), Blocked.Distance, 100.0f);
	}

	// A blocker behind the target does not hide it. Without this the gate would reject every target that has
	// anything at all behind it, which is nearly all of them.
	{
		FRopeAimRayHitResult Hit;
		FRopeAimRayHitResult Blocked;
		const FRopeAimTargeting::FQueryContext Ctx = MakeBlockerContext(&Colliders, /*BlockDistance*/ 300.0f, QueryRadius);
		TestTrue(TEXT("a target in front of the wall is still acquired"),
			FRopeAimTargeting::FindAimRayBoneHit(Ctx, Origin, AimDir, 400.0f, QueryRadius, 2.0f,
				PermitAll, Hit, &Blocked));
		TestTrue(TEXT("the acquired bone is the spine"), Hit.Bone == FName("spine"));
	}
	return true;
}

// The endpoint of a throw into open space. Aiming at bare floor acquires no target, so the rope is driven
// towards the ray end; that endpoint has to be clamped to the surface, because the guided throw replays node
// positions with the solver switched off and would otherwise carry the rope through the floor.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceFreeThrowEndpointClampTest,
	"DynamicRope.Pierce.FreeThrowEndpointClampedByBlocker",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceFreeThrowEndpointClampTest::RunTest(const FString& Parameters)
{
	const FVector Origin = FVector(0, 0, 200);
	const FVector AimDir = FVector(0, 0, -1); // Straight down at the floor.
	const float RopeLen = 500.0f;
	const float Clearance = 2.0f;

	// With nothing in the way the endpoint stays at the ray end, one rope length ahead.
	{
		const FRopeAimTargeting::FQueryContext Ctx = MakeBlockerContext(nullptr, /*BlockDistance*/ 0.0f, 3.0f);
		const FVector Endpoint = FRopeAimTargeting::ResolveOpenSpaceThrowEndpoint(Ctx, Origin, AimDir, RopeLen, Clearance);
		TestTrue(TEXT("with no blocker the endpoint is the ray end"),
			Endpoint.Equals(Origin + AimDir * RopeLen, 0.01f));
	}

	// A floor 150 cm below pulls the endpoint back to just above it, well short of the ray end at 500 cm.
	{
		const FRopeAimTargeting::FQueryContext Ctx = MakeBlockerContext(nullptr, /*BlockDistance*/ 150.0f, 3.0f);
		const FVector Endpoint = FRopeAimTargeting::ResolveOpenSpaceThrowEndpoint(Ctx, Origin, AimDir, RopeLen, Clearance);
		TestTrue(TEXT("the endpoint is clamped to just in front of the floor"),
			Endpoint.Equals(Origin + AimDir * (150.0f - Clearance), 0.01f));
		TestTrue(TEXT("the endpoint stays above the floor"), Endpoint.Z > Origin.Z - 150.0f);
	}

	// A blocker closer than the clearance clamps to the origin rather than producing a throw that runs
	// backwards through the thrower.
	{
		const FRopeAimTargeting::FQueryContext Ctx = MakeBlockerContext(nullptr, /*BlockDistance*/ 1.0f, 3.0f);
		const FVector Endpoint = FRopeAimTargeting::ResolveOpenSpaceThrowEndpoint(Ctx, Origin, AimDir, RopeLen, Clearance);
		TestTrue(TEXT("a blocker inside the clearance clamps to the origin"), Endpoint.Equals(Origin, 0.01f));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
