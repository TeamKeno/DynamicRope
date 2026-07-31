// Copyright Epic Games, Inc. All Rights Reserved.
//
// Unit tests for the owner exclusion decision in collider gathering, RopeColliderGather::IsExcludedOwnerBody.
// On paths exempt from per-provider exclusion, such as the static world provider, this decision is what keeps world
// geometry while removing the shapes belonging to the rope's own actor. Widening it wrongly makes the floor
// disappear, and narrowing it wrongly lets the rope's own shapes that follow it around, such as the tether proxy, the
// tip or a weapon, push their own rope and bring the oscillation back. Both failures are pinned here.
// The actor pointers are used for identity comparison alone, so transient objects suffice; they are never dereferenced.

#include "Misc/AutomationTest.h"
#include "Collision/RopeColliderProvider.h"
#include "GameFramework/Actor.h"
#include "UObject/Package.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	// Dummy actors for comparison alone, needing no world. Distinct names give distinct pointers, which is enough for identity.
	AActor* MakeIdentityActor()
	{
		return NewObject<AActor>(GetTransientPackage());
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeColliderGatherOwnerBodyTest,
	"DynamicRope.Collision.Gather.OwnerBodyExclusion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeColliderGatherOwnerBodyTest::RunTest(const FString& Parameters)
{
	AActor* RopeOwner = MakeIdentityActor();
	AActor* WorldProp = MakeIdentityActor();
	AActor* WrapTarget = MakeIdentityActor();

	// The pool: [0] a shape belonging to the rope's owner, such as the tether proxy or the tip, [1] world geometry
	//     such as the floor, [2] the target being wrapped, on another actor, and [3] one of unknown origin, being null.
	const TArray<const AActor*> SourceActors = { RopeOwner, WorldProp, WrapTarget, nullptr };

	// By default, with bIncludeOwnerColliders false, the owner's own body drops out and everything else remains.
	TestTrue(TEXT("a shape belonging to the rope's owner is excluded"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 0, RopeOwner));
	TestFalse(TEXT("world geometry on another actor, such as a floor or a pillar, remains"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 1, RopeOwner));
	TestFalse(TEXT("a cross-actor wrap target remains"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 2, RopeOwner));
	TestFalse(TEXT("an entry of unknown origin, being null, is not excluded"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 3, RopeOwner));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeColliderGatherOwnerBodyFallbackTest,
	"DynamicRope.Collision.Gather.OwnerBodyExclusionFallbacks",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeColliderGatherOwnerBodyFallbackTest::RunTest(const FString& Parameters)
{
	AActor* RopeOwner = MakeIdentityActor();
	const TArray<const AActor*> SourceActors = { RopeOwner, RopeOwner };

	// Opting in, with bIncludeOwnerColliders true and therefore nothing to exclude, excludes nothing at all.
	// It is the escape hatch that lets a rope mounted on a prop keep colliding with its own base, so breaking this property would silently break that configuration.
	TestFalse(TEXT("with owner colliders opted in, the owner's own body remains"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 0, nullptr));

	// A provider that supplies no attribution, such as the skeletal or wrap target providers, gives an empty array, so nothing is excluded and the decision is left to the per-provider filter.
	const TArray<const AActor*> NoAttribution;
	TestFalse(TEXT("a provider supplying no attribution excludes nothing per collider"),
		RopeColliderGather::IsExcludedOwnerBody(NoAttribution, 0, RopeOwner));

	// A mismatched array length, meaning a provider bug: an index out of range is not excluded, so it never fails in the direction of losing a collision.
	TestFalse(TEXT("on a length mismatch an index out of range is not excluded"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 5, RopeOwner));
	TestFalse(TEXT("a negative index is not excluded"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, -1, RopeOwner));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
