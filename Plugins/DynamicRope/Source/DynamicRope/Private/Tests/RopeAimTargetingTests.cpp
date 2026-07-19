// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeAimTargeting unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeAimTargeting.h"
#include "Collision/RopeCollider.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "RopeTestHelpers.h"

// resolve mode별 aim lock 범위: Assisted는 같은 캐릭터의 다른 본까지 multi-bone 후보로 허용하고,
// Guaranteed는 prepared target의 exact bone만 허용한다.
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
	TestTrue(TEXT("wrap 가능 target은 context까지 해석"), bResolved);
	TestTrue(TEXT("같은 sweep의 hit 반환"), Hit.bHit);
	TestFalse(TEXT("wrap 가능 target은 blocked 아님"), Blocked.bHit);
	TestTrue(TEXT("context에 aim guide 설정"), Resolved.bHasAimGuideHit);
	TestTrue(TEXT("context와 hit의 mesh 일치"), Resolved.AimGuideMesh.Get() == Hit.Mesh);
	TestEqual(TEXT("context와 hit의 bone 일치"), Resolved.AimGuideBone, Hit.Bone);

	Resolved = FRopeThrowContext();
	Hit = FRopeAimRayHitResult();
	Blocked = FRopeAimRayHitResult();
	const bool bRejected = FRopeAimTargeting::ResolveAimRayThrowContext(
		QueryContext, Request,
		[](const USceneComponent*, FName) { return false; },
		Resolved, &Hit, &Blocked);
	TestFalse(TEXT("게이트 거부 target은 context fallback"), bRejected);
	TestFalse(TEXT("게이트 거부 target은 valid hit 아님"), Hit.bHit);
	TestTrue(TEXT("게이트 거부 target은 blocked로 반환"), Blocked.bHit);
	TestFalse(TEXT("fallback context에는 aim guide 없음"), Resolved.bHasAimGuideHit);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

