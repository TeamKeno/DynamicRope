// Copyright Epic Games, Inc. All Rights Reserved.
//
// Base capsule collider motion and swept-query unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Collision/RopeCollider.h"

// Whether a moving capsule's query reports the contact material point's surface velocity, per the FRopeContact contract: centimetres per second, and zero when static.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCapsuleSurfaceVelocityTest,
	"DynamicRope.Collision.CapsuleSurfaceVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCapsuleSurfaceVelocityTest::RunTest(const FString& Parameters)
{
	// A capsule along Z with a radius of 10 moves 6 cm along positive X in one frame, at 1/60 s. The node overlaps the positive X surface at the middle of the axis.
	FCapsuleCollider Cap(FVector(6, 0, 0), FVector(6, 0, 100), 10.0f, FName(TEXT("bone")));
	Cap.PrevA = FVector(0, 0, 0);
	Cap.PrevB = FVector(0, 0, 100);
	Cap.InvDeltaTime = 60.0f;

	const FRopeContact Contact = Cap.Query(FVector(14, 0, 50), 2.0f);
	TestTrue(TEXT("node overlaps capsule"), Contact.bHit);
	// The material point, on the axis at z = 50, is displaced 6 cm along positive X over the frame, giving a surface velocity of 6 times 60, or 360 cm/s.
	TestTrue(FString::Printf(TEXT("surface velocity %s should be ~(360,0,0)"), *Contact.SurfaceVelocity.ToString()),
		Contact.SurfaceVelocity.Equals(FVector(360, 0, 0), 1.0f));
	TestTrue(TEXT("normal points outward (+X)"), Contact.Normal.Equals(FVector(1, 0, 0), 0.01f));

	// A static capsule, with an inverse delta of zero, has zero surface velocity, which preserves the existing behaviour.
	FCapsuleCollider StaticCap(FVector(6, 0, 0), FVector(6, 0, 100), 10.0f);
	const FRopeContact StaticContact = StaticCap.Query(FVector(14, 0, 50), 2.0f);
	TestTrue(TEXT("static capsule overlaps"), StaticContact.bHit);
	TestTrue(TEXT("static capsule surface velocity is zero"), StaticContact.SurfaceVelocity.IsNearlyZero());
	return true;
}

// Whether a moving capsule's swept query catches a stationary node it overtakes, on the approaching front face, through relative-motion continuous collision detection.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCapsuleSweptRelativeMotionTest,
	"DynamicRope.Collision.CapsuleSweptRelativeMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCapsuleSweptRelativeMotionTest::RunTest(const FString& Parameters)
{
	// A capsule along Z with a radius of 10 moves 80 cm in one frame, from x = -40 to x = +40, while the node sits
	// still at x = 0. Looking at the end pose alone the capsule has already passed the node, at a distance of 40
	// against a reach of 12, and there is no contact at all, which is a pass-through. The relative-motion sweep has to
	// catch the first contact during the approach and report a normal pushing the node out through the capsule's
	// front face, along positive X, in the direction of travel.
	FCapsuleCollider Cap(FVector(40, 0, 0), FVector(40, 0, 100), 10.0f, FName(TEXT("bone")));
	Cap.PrevA = FVector(-40, 0, 0);
	Cap.PrevB = FVector(-40, 0, 100);
	Cap.InvDeltaTime = 60.0f;

	FRopeSweptQuery Q;
	// A stationary node, with no movement.
	Q.WorldStart = FVector(0, 0, 50);
	Q.WorldEnd = FVector(0, 0, 50);
	Q.NodeRadius = 2.0f;
	Q.SweepStep = 2.0f;
	Q.MaxSamples = 64;
	// The whole frame as a single substep.
	Q.SubAlpha0 = 0.0f;
	Q.SubAlpha1 = 1.0f;

	FVector HitPos;
	const FRopeContact Contact = Cap.QuerySwept(Q, HitPos);
	TestTrue(TEXT("overtaking capsule is caught by relative sweep"), Contact.bHit);
	TestTrue(FString::Printf(TEXT("normal %s should push node ahead (+X)"), *Contact.Normal.ToString()),
		Contact.Normal.X > 0.9f);
	// The surface velocity is along the capsule's direction of travel, positive X, at 80 cm per frame times 60, giving 4800 cm/s.
	TestTrue(FString::Printf(TEXT("surface velocity %s should be ~(4800,0,0)"), *Contact.SurfaceVelocity.ToString()),
		Contact.SurfaceVelocity.Equals(FVector(4800, 0, 0), 10.0f));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

