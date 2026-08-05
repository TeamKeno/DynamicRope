// Copyright 2026 TeamKeno. All Rights Reserved.
//
// FRopeTipPlacement socket-follow, pierce-embed, and aim-lock unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeTipPlacement.h"

// The placement mathematics of a pierce embed: whether the tip socket embeds at the hit point along the pierce
// direction and the tail socket becomes the rope's attachment point.
// FRopeTipPlacement has no world dependency, so the contract is pinned with socket locals alone.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceEmbedMathTest,
	"DynamicRope.Pierce.EmbedPlacesTipAtHit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceEmbedMathTest::RunTest(const FString& Parameters)
{
	const FVector HitPoint(0, 60, 0);
	const FVector PierceDir(0, 1, 0); // An axis-misaligned direction, so the rotation is verified too.

	// The tip socket is 50 along the mesh's X with a non-trivial rotation, 90 degrees about Z, which confirms the contract is independent of the socket's rotation.
	const FTransform TipSocketLocal(FQuat(FVector(0, 0, 1), HALF_PI), FVector(50, 10, 0));
	const FTransform TailSocketLocal(FQuat::Identity, FVector(-30, -5, 0)); // The tail forms a two-point axis distinct from the head.

	FTransform ComponentWorld;
	FVector TailWorld;
	FRopeTipPlacement::SolvePierceEmbed(HitPoint, PierceDir, TipSocketLocal,
		/*bHasTailSocket*/ true, TailSocketLocal, ComponentWorld, TailWorld);

	const FTransform TipWorld = TipSocketLocal * ComponentWorld;
	TestTrue(TEXT("the tip socket embeds at the hit point"), TipWorld.GetLocation().Equals(HitPoint, 0.01f));
	TestTrue(TEXT("the head-to-tail vector is the pierce direction"),
		(TipWorld.GetLocation() - TailWorld).GetSafeNormal().Equals(PierceDir, 0.01f));

	// The rope's attachment point is the tail socket in world space.
	const FVector ExpectedTail = (TailSocketLocal * ComponentWorld).GetLocation();
	TestTrue(TEXT("the tail socket is the rope attachment point"), TailWorld.Equals(ExpectedTail, 0.01f));

	const FTransform Relative(FQuat::Identity, FVector::ZeroVector, FVector(0.2f, 0.2f, 0.2f));
	const FTransform EffectiveTipSocketLocal = TipSocketLocal * Relative;
	const FTransform EffectiveTailSocketLocal = TailSocketLocal * Relative;
	FTransform ScaledBaseWorld;
	FVector ScaledTailWorld;
	FRopeTipPlacement::SolvePierceEmbed(HitPoint, PierceDir, EffectiveTipSocketLocal,
		/*bHasTailSocket*/ true, EffectiveTailSocketLocal, ScaledBaseWorld, ScaledTailWorld);
	const FTransform ScaledMeshWorld = Relative * ScaledBaseWorld;
	const FTransform ScaledTipWorld = TipSocketLocal * ScaledMeshWorld;
	const FTransform ScaledTailTransform = TailSocketLocal * ScaledMeshWorld;
	TestTrue(TEXT("the tip socket still embeds at the hit point after a relative scale"),
		ScaledTipWorld.GetLocation().Equals(HitPoint, 0.01f));
	TestTrue(TEXT("the tail socket is still the rope attachment point after a relative scale"),
		ScaledTailWorld.Equals(ScaledTailTransform.GetLocation(), 0.01f));
	TestTrue(TEXT("the head-to-tail vector is still the pierce direction after a relative scale"),
		(ScaledTipWorld.GetLocation() - ScaledTailTransform.GetLocation()).GetSafeNormal().Equals(PierceDir, 0.01f));
	TestTrue(TEXT("the relative scale of 0.2 is preserved"),
		ScaledMeshWorld.GetScale3D().Equals(Relative.GetScale3D(), 0.001f));

	// With no tail socket the attachment point is the mesh origin.
	FTransform CW2;
	FVector Tail2;
	FRopeTipPlacement::SolvePierceEmbed(HitPoint, PierceDir, TipSocketLocal,
		/*bHasTailSocket*/ false, FTransform::Identity, CW2, Tail2);
	TestTrue(TEXT("with no tail socket the attachment point is the mesh origin"), Tail2.Equals(CW2.GetLocation(), 0.01f));
	return true;
}

// The mathematics of following the tail socket: the rope's end node has to attach to the tail socket rather than to the mesh origin.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceTailSocketFollowMathTest,
	"DynamicRope.Pierce.TailSocketFollowMath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceTailSocketFollowMathTest::RunTest(const FString& Parameters)
{
	const FVector RopeAttachWorld(10, 25, 40);
	const FVector ForwardDir = FVector(0, 1, 0);
	const FTransform TailSocketLocal(FQuat(FVector(0, 0, 1), HALF_PI), FVector(-30, 5, 2));
	const FTransform HeadSocketLocal(FQuat(FVector(0, 1, 0), 0.4f), FVector(20, 35, 8));

	FTransform ComponentWorld;
	FRopeTipPlacement::SolveSocketFollow(RopeAttachWorld, ForwardDir, TailSocketLocal,
		/*bHasHeadSocket*/ true, HeadSocketLocal, ComponentWorld);

	const FTransform TailWorld = TailSocketLocal * ComponentWorld;
	const FTransform HeadWorld = HeadSocketLocal * ComponentWorld;
	TestTrue(TEXT("the tail socket's position is the rope attachment point"),
		TailWorld.GetLocation().Equals(RopeAttachWorld, 0.01f));
	TestTrue(TEXT("the head-to-tail vector is the direction of the rope's end"),
		(HeadWorld.GetLocation() - TailWorld.GetLocation()).GetSafeNormal().Equals(ForwardDir, 0.01f));

	const FTransform Relative(
		FQuat(FVector(0, 0, 1), 0.35f),
		FVector(4, -8, 3),
		FVector(0.2f, 0.2f, 0.2f));
	const FTransform EffectiveTailSocketLocal = TailSocketLocal * Relative;
	const FTransform EffectiveHeadSocketLocal = HeadSocketLocal * Relative;

	FTransform BaseWorld;
	FRopeTipPlacement::SolveSocketFollow(RopeAttachWorld, ForwardDir, EffectiveTailSocketLocal,
		/*bHasHeadSocket*/ true, EffectiveHeadSocketLocal, BaseWorld);

	const FTransform RenderedMeshWorld = Relative * BaseWorld;
	const FTransform RenderedTailWorld = TailSocketLocal * RenderedMeshWorld;
	const FTransform RenderedHeadWorld = HeadSocketLocal * RenderedMeshWorld;
	TestTrue(TEXT("the tail socket's position is preserved through a relative transform"),
		RenderedTailWorld.GetLocation().Equals(RopeAttachWorld, 0.01f));
	TestTrue(TEXT("the head-to-tail direction is preserved through a relative transform"),
		(RenderedHeadWorld.GetLocation() - RenderedTailWorld.GetLocation()).GetSafeNormal().Equals(ForwardDir, 0.01f));
	TestTrue(TEXT("the relative scale of 0.2 is preserved"),
		RenderedMeshWorld.GetScale3D().Equals(Relative.GetScale3D(), 0.001f));
	return true;
}

// The wrapped restoration round trip: reviving the frozen bone-local transform through the bone transform matches the
// pose at commit. The wrapped branch of UpdateTipMeshTransform depends on that identity.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceEmbedBoneLocalRoundTripTest,
	"DynamicRope.Pierce.EmbedBoneLocalRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceEmbedBoneLocalRoundTripTest::RunTest(const FString& Parameters)
{
	const FVector HitPoint(120, -40, 15);
	const FVector PierceDir = FVector(1, 1, 0).GetSafeNormal();
	const FTransform TipSocketLocal(FQuat(FVector(0, 1, 0), 0.3f), FVector(40, 5, 0));

	FTransform ComponentWorld;
	FVector TailWorld;
	FRopeTipPlacement::SolvePierceEmbed(HitPoint, PierceDir, TipSocketLocal,
		/*bHasTailSocket*/ false, FTransform::Identity, ComponentWorld, TailWorld);

	// An arbitrary bone transform, with a position, a rotation and a scale.
	const FTransform BoneXform(FQuat(FVector(0, 0, 1), 1.1f), FVector(300, 50, 20), FVector(1.0f));

	// Stored at commit: the local mesh transform is the component's world transform relative to the bone.
	const FTransform LocalMeshTransform = ComponentWorld.GetRelativeTransform(BoneXform);
	// Restored every frame: the mesh world transform is the local mesh transform composed with the bone transform, which has to equal the component's world transform or there is a pop and drift.
	const FTransform Restored = LocalMeshTransform * BoneXform;

	TestTrue(TEXT("the restored position matches the position at commit"), Restored.GetLocation().Equals(ComponentWorld.GetLocation(), 0.01f));
	TestTrue(TEXT("the restored rotation matches the rotation at commit"),
		Restored.GetRotation().Equals(ComponentWorld.GetRotation(), 0.001f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceAimYawLockTest,
	"DynamicRope.Pierce.AimYawLockPreservesPitch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceAimYawLockTest::RunTest(const FString& Parameters)
{
	const FVector Up = FVector::UpVector;
	const FVector SourceDir = FVector(0.8f, 0.0f, 0.6f).GetSafeNormal();
	const FVector AimDir = FVector(0.0f, 1.0f, 0.4f).GetSafeNormal();
	const FVector Locked = FRopeTipPlacement::MakeAimYawLockedDirection(SourceDir, AimDir, Up);

	TestTrue(TEXT("the vertical tilt is preserved"),
		FMath::IsNearlyEqual(FVector::DotProduct(Locked, Up), FVector::DotProduct(SourceDir, Up), 0.001f));
	const FVector LockedFlat = FVector::VectorPlaneProject(Locked, Up).GetSafeNormal();
	const FVector AimFlat = FVector::VectorPlaneProject(AimDir, Up).GetSafeNormal();
	TestTrue(TEXT("the horizontal yaw is the aim direction"), LockedFlat.Equals(AimFlat, 0.001f));

	const FVector VerticalAimResult = FRopeTipPlacement::MakeAimYawLockedDirection(
		SourceDir, FVector::UpVector, Up);
	TestTrue(TEXT("with no horizontal aim the original direction is preserved"), VerticalAimResult.Equals(SourceDir, 0.001f));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

