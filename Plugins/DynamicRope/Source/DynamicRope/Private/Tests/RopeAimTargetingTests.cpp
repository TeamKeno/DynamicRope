// Copyright 2026 TeamKeno. All Rights Reserved.
//
// FRopeAimTargeting unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeAimTargeting.h"
#include "Collision/RopeCollider.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "RopeTestHelpers.h"

// The aim lock's permitted range per resolve mode: assisted permits other bones on the same character as multi-bone
// candidates, while guaranteed permits the prepared target's exact bone alone.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimTargetResolvePolicyTest,
	"DynamicRope.FlightContact.AimTargetResolvePolicy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimTargetResolvePolicyTest::RunTest(const FString& Parameters)
{
	const USceneComponent* TargetMesh = NewObject<USceneComponent>();
	const USceneComponent* OtherMesh = NewObject<USceneComponent>();
	const FName PrimaryBone("upperarm_l");
	const FName NeighborBone("clavicle_l");

	FRopeThrowContext ThrowContext;
	ThrowContext.bHasAimGuideHit = true;
	ThrowContext.AimGuideMesh = TargetMesh;
	ThrowContext.AimGuideBone = PrimaryBone;

	FRopeAimTargeting Targeting;
	Targeting.SetWrapTargetLock(ThrowContext);
	TestTrue(TEXT("exact aim bone is primary"), Targeting.IsPrimaryTarget(TargetMesh, PrimaryBone));
	TestTrue(TEXT("neighbor bone on target mesh is allowed in Assisted"),
		Targeting.IsWrapTarget(ERopePhase::Flight, ERopeWrapResolveMode::AssistedJudged,
			TargetMesh, NeighborBone));
	TestFalse(TEXT("same bone on another mesh is rejected in Assisted"),
		Targeting.IsWrapTarget(ERopePhase::Flight, ERopeWrapResolveMode::AssistedJudged,
			OtherMesh, PrimaryBone));
	TestFalse(TEXT("neighbor bone is rejected in Guaranteed"),
		Targeting.IsWrapTarget(ERopePhase::Flight, ERopeWrapResolveMode::GuaranteedWrap,
			TargetMesh, NeighborBone));
	TestTrue(TEXT("exact bone remains allowed in Guaranteed"),
		Targeting.IsWrapTarget(ERopePhase::Flight, ERopeWrapResolveMode::GuaranteedWrap,
			TargetMesh, PrimaryBone));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimRayReachLengthTest,
	"DynamicRope.FlightContact.AimRayReachLengthUsesThrowOrigin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimRayReachLengthTest::RunTest(const FString& Parameters)
{
	const FVector RayDir(1.0f, 0.0f, 0.0f);
	const float RopeReach = 200.0f;

	TestEqual(TEXT("ray at hand uses rope reach"),
		FRopeAimTargeting::ResolveRayLengthForReach(FVector::ZeroVector, RayDir, FVector::ZeroVector, RopeReach),
		200.0f);
	TestEqual(TEXT("ray behind hand extends to hand reach"),
		FRopeAimTargeting::ResolveRayLengthForReach(FVector(-100.0f, 0.0f, 0.0f), RayDir, FVector::ZeroVector, RopeReach),
		300.0f);
	TestTrue(TEXT("lateral offset intersects reach sphere at chord end"),
		FMath::IsNearlyEqual(
			FRopeAimTargeting::ResolveRayLengthForReach(FVector(0.0f, 100.0f, 0.0f), RayDir, FVector::ZeroVector, RopeReach),
			FMath::Sqrt(30000.0f), 0.01f));
	TestEqual(TEXT("ray pointing away from reach sphere has no usable length"),
		FRopeAimTargeting::ResolveRayLengthForReach(FVector(300.0f, 0.0f, 0.0f), RayDir, FVector::ZeroVector, RopeReach),
		0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeAimRayResolveOutputsTest,
	"DynamicRope.FlightContact.AimRayResolveReturnsSingleSweepOutputs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeAimRayResolveOutputsTest::RunTest(const FString& Parameters)
{
	const FName Bone("upperarm_l");
	const USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>();
	RopeTest::FSphereMockCollider Target(FVector(100.0f, 0.0f, 0.0f), 10.0f, Bone, Mesh);
	TArray<IRopeCollider*> Colliders = { &Target };

	FRopeAimTargeting::FQueryContext QueryContext;
	QueryContext.Colliders = &Colliders;
	QueryContext.FallbackRayLength = 200.0f;
	QueryContext.FallbackQueryRadius = 2.0f;

	FRopeAimRayThrowRequest Request;
	Request.BaseContext.Origin = FVector::ZeroVector;
	Request.BaseContext.FrameForward = FVector::RightVector;
	Request.BaseContext.FrameUp = FVector::UpVector;
	Request.BaseContext.FrameRight = FVector::BackwardVector;
	Request.RayOrigin = FVector::ZeroVector;
	Request.RayDirection = FVector::ForwardVector;
	Request.RayLength = 200.0f;
	Request.ReachOrigin = FVector::ZeroVector;
	Request.ReachLength = 200.0f;
	Request.QueryRadius = 2.0f;
	Request.SweepStep = 1.0f;

	FRopeThrowContext Resolved;
	FRopeAimRayHitResult Hit;
	FRopeAimRayHitResult Blocked;
	const bool bResolved = FRopeAimTargeting::ResolveAimRayThrowContext(
		QueryContext, Request,
		[](const USceneComponent*, FName) { return true; },
		Resolved, &Hit, &Blocked);
	TestTrue(TEXT("a wrappable target resolves the context as well"), bResolved);
	TestTrue(TEXT("it returns the hit from the same sweep"), Hit.bHit);
	TestFalse(TEXT("a wrappable target is not blocked"), Blocked.bHit);
	TestTrue(TEXT("the aim guide is set on the context"), Resolved.bHasAimGuideHit);
	TestTrue(TEXT("the hit direction replaces only the runtime aim forward"),
		Resolved.FrameForward.Equals(FVector::ForwardVector, 0.01f));
	TestTrue(TEXT("the pre-hit forward is preserved for the physical whip sweep"),
		Resolved.bHasWhipReferenceFrame &&
		Resolved.WhipReferenceForward.Equals(FVector::RightVector, 0.01f));
	TestTrue(TEXT("the context's mesh matches the hit's"), Resolved.AimGuideMesh.Get() == Hit.Mesh);
	TestEqual(TEXT("the context's bone matches the hit's"), Resolved.AimGuideBone, Hit.Bone);

	Resolved = FRopeThrowContext();
	Hit = FRopeAimRayHitResult();
	Blocked = FRopeAimRayHitResult();
	const bool bRejected = FRopeAimTargeting::ResolveAimRayThrowContext(
		QueryContext, Request,
		[](const USceneComponent*, FName) { return false; },
		Resolved, &Hit, &Blocked);
	TestFalse(TEXT("a target refused by the gate falls back to the plain context"), bRejected);
	TestFalse(TEXT("a target refused by the gate is not a valid hit"), Hit.bHit);
	TestTrue(TEXT("a target refused by the gate is returned as blocked"), Blocked.bHit);
	// It is a wrap target that was refused, not level geometry, so the HUD shows its blocked colour.
	TestTrue(TEXT("a target refused by the gate is a wrap candidate"), Blocked.bWrapCandidate);
	TestFalse(TEXT("the fallback context carries no aim guide"), Resolved.bHasAimGuideHit);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

