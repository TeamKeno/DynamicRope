// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Unit tests for FRopeBoxCollider. The central point is the exact diagonal normal at corners and edges,
// which is the regression gate for the rope passing through a box corner where the voxel global distance
// field rounded it off.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Collision/RopeStaticCollider.h"
#include "Collision/RopeBodyColliderExtraction.h"
#include "PhysicsEngine/BodySetup.h"
#include "UObject/Package.h"
#include "Solver/RopeXPBDSolver.h"
#include "RopeTestHelpers.h"

// Whether the push-out normal at a diagonal position outside a corner is the exact diagonal direction
// rather than a face normal.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxCornerPushOutTest,
	"DynamicRope.Collision.BoxCornerPushOut",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxCornerPushOutTest::RunTest(const FString& Parameters)
{
	// One: an axis-aligned box, queried from a short diagonal distance outside a corner.
	{
		const FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, FVector(50.0));
		const FVector Corner(50.0, 50.0, 50.0);
		const FVector Dir = FVector(1, 1, 1).GetSafeNormal();
		const FVector P = Corner + Dir * 2.0;

		const FRopeContact C = Box.Query(P, 5.0f);
		TestTrue(TEXT("corner overlap hit"), C.bHit);
		TestTrue(TEXT("normal is unit"), FMath::IsNearlyEqual(static_cast<float>(C.Normal.Size()), 1.0f, 1e-3f));
		// The diagonal normal, which the global distance field would have rounded towards a face normal.
		TestTrue(FString::Printf(TEXT("normal %s should equal corner diagonal %s"), *C.Normal.ToString(), *Dir.ToString()),
			C.Normal.Equals(Dir, 1e-3));
		TestTrue(TEXT("penetration = 5 - 2 = 3"), FMath::IsNearlyEqual(C.Penetration, 3.0f, 1e-3f));
		// After the push-out the node sits exactly the query radius away from the corner.
		const FVector Pushed = P + C.Normal * C.Penetration;
		TestTrue(TEXT("pushed node sits at query radius from corner"),
			FMath::IsNearlyEqual(static_cast<float>(FVector::Dist(Pushed, Corner)), 5.0f, 1e-3f));
		TestTrue(TEXT("surface point is the corner"), C.SurfacePoint.Equals(Corner, 1e-3));
		// The static world geometry contract: no attribution and no surface velocity.
		TestTrue(TEXT("no bone attribution"), C.Bone.IsNone());
		TestTrue(TEXT("no source mesh"), C.SourceMesh == nullptr);
		TestTrue(TEXT("zero surface velocity"), C.SurfaceVelocity.IsNearlyZero());
	}

	// Two: the same has to hold for a rotated and translated box, which verifies the local transform.
	{
		const FQuat Rot(FVector::UpVector, PI / 6.0);
		const FVector Center(100.0, -40.0, 25.0);
		const FRopeBoxCollider Box(Center, Rot, FVector(50.0));
		const FVector CornerW = Center + Rot.RotateVector(FVector(50.0, 50.0, 50.0));
		const FVector DirW = Rot.RotateVector(FVector(1, 1, 1).GetSafeNormal());
		const FVector P = CornerW + DirW * 2.0;

		const FRopeContact C = Box.Query(P, 5.0f);
		TestTrue(TEXT("rotated corner hit"), C.bHit);
		TestTrue(FString::Printf(TEXT("rotated normal %s should equal world diagonal %s"), *C.Normal.ToString(), *DirW.ToString()),
			C.Normal.Equals(DirW, 1e-3));
		TestTrue(TEXT("rotated penetration = 3"), FMath::IsNearlyEqual(C.Penetration, 3.0f, 1e-3f));

		// Whether the oriented box's world AABB contains the rotated corners, which is the broad-phase
		// contract.
		const FBox Bounds = Box.GetWorldBounds();
		TestTrue(TEXT("world bounds contains rotated corner"), Bounds.IsInsideOrOn(CornerW));
	}

	return true;
}

// Whether a node inside the box is pushed out through the least-penetrated face.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxInsideMinFaceTest,
	"DynamicRope.Collision.BoxInsideMinFace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxInsideMinFaceTest::RunTest(const FString& Parameters)
{
	const FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, FVector(50.0));
	// An interior point nearest the positive X face, which is therefore the minimum-penetration face.
	const FVector P(45.0, 10.0, -20.0);

	const FRopeContact C = Box.Query(P, 2.0f);
	TestTrue(TEXT("inside hit"), C.bHit);
	TestTrue(FString::Printf(TEXT("normal %s should be +X face"), *C.Normal.ToString()),
		C.Normal.Equals(FVector(1, 0, 0), 1e-4));
	// The penetration is the node radius plus the depth to that face, so after the push-out the node sits at
	// the surface plus the radius.
	TestTrue(TEXT("penetration = 7"), FMath::IsNearlyEqual(C.Penetration, 7.0f, 1e-3f));
	const FVector Pushed = P + C.Normal * C.Penetration;
	TestTrue(TEXT("pushed to surface + radius"), Pushed.Equals(FVector(52.0, 10.0, -20.0), 1e-3));
	TestTrue(TEXT("surface point on +X face"), C.SurfacePoint.Equals(FVector(50.0, 10.0, -20.0), 1e-3));
	return true;
}

// Whether the default swept query, which falls back to line samples, catches a thin box wall at its entry
// face rather than passing through it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxSweptTunnelingTest,
	"DynamicRope.Collision.BoxQuerySweptTunneling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxSweptTunnelingTest::RunTest(const FString& Parameters)
{
	// It attempts to pass through a thin wall in a single long movement.
	const FRopeBoxCollider Wall(FVector::ZeroVector, FQuat::Identity, FVector(2.0, 100.0, 100.0));

	FRopeSweptQuery Q;
	Q.WorldStart = FVector(-30.0, 0.0, 0.0);
	Q.WorldEnd = FVector(30.0, 0.0, 0.0);
	Q.NodeRadius = 2.0f;
	Q.SweepStep = 2.0f;
	Q.MaxSamples = 64;

	FVector HitPos = FVector::ZeroVector;
	const FRopeContact C = Wall.QuerySwept(Q, HitPos);
	TestTrue(TEXT("swept catches thin wall"), C.bHit);
	TestTrue(FString::Printf(TEXT("hit position x=%.1f should be on entry side (x<0)"), HitPos.X), HitPos.X < 0.0);
	TestTrue(FString::Printf(TEXT("normal %s should face entry side (-X)"), *C.Normal.ToString()),
		C.Normal.Equals(FVector(-1, 0, 0), 1e-3));
	return true;
}

// The configured SweepStep must be a maximum interval, not merely the divisor used to choose a
// point count. With the old 1 + floor(Travel / Step) formula, this 3.9 cm path produced only its two
// endpoints, leaving a 3.9 cm gap in which the thin wall was never queried.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSweepStepMaximumSpacingTest,
	"DynamicRope.Collision.SweepStepIsMaximumSpacing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSweepStepMaximumSpacingTest::RunTest(const FString& Parameters)
{
	const FRopeBoxCollider ThinWall(FVector::ZeroVector, FQuat::Identity, FVector(0.05, 100.0, 100.0));

	FRopeSweptQuery Q;
	Q.WorldStart = FVector(-1.95, 0.0, 0.0);
	Q.WorldEnd = FVector(1.95, 0.0, 0.0);
	Q.NodeRadius = 0.05f;
	Q.SweepStep = 2.0f;
	Q.MaxSamples = 16;

	FVector HitPos = FVector::ZeroVector;
	const FRopeContact C = ThinWall.QuerySwept(Q, HitPos);
	TestTrue(TEXT("sweep samples the thin wall between both endpoints"), C.bHit);
	return true;
}

namespace
{
	// Builds an axis-aligned box, from a centre and half extents, as a six-plane convex. It is the fixture
	// used to compare and verify the convex query against the box.
	// Each face has an outward axis normal, so the positive X face at the box's maximum X becomes a plane
	// with that normal and offset.
	TArray<FPlane> MakeAABoxPlanes(const FVector& Center, const FVector& H)
	{
		TArray<FPlane> P;
		P.Add(FPlane(FVector(1, 0, 0), Center.X + H.X));
		P.Add(FPlane(FVector(-1, 0, 0), -Center.X + H.X));
		P.Add(FPlane(FVector(0, 1, 0), Center.Y + H.Y));
		P.Add(FPlane(FVector(0, -1, 0), -Center.Y + H.Y));
		P.Add(FPlane(FVector(0, 0, 1), Center.Z + H.Z));
		P.Add(FPlane(FVector(0, 0, -1), -Center.Z + H.Z));
		return P;
	}
}

// Whether the maximum-plane test picks exactly the least-penetrated face for a point inside a convex and
// matches the box query, since it is exact inside.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeConvexInsideExactTest,
	"DynamicRope.Collision.ConvexInsideExact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeConvexInsideExactTest::RunTest(const FString& Parameters)
{
	const FVector H(50.0);
	FRopeConvexCollider Convex(MakeAABoxPlanes(FVector::ZeroVector, H), FBox(-H, H));
	FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, H);

	// The interior point is nearest the positive X face. It has to give the same result as the box, because
	// the maximum-plane test is exact inside.
	const FVector P(45.0, 10.0, -20.0);
	const FRopeContact C = Convex.Query(P, 2.0f);
	const FRopeContact B = Box.Query(P, 2.0f);
	TestTrue(TEXT("convex inside hit"), C.bHit);
	TestTrue(FString::Printf(TEXT("normal %s is +X face"), *C.Normal.ToString()), C.Normal.Equals(FVector(1, 0, 0), 1e-4));
	TestTrue(TEXT("penetration = 7 (matches box)"), FMath::IsNearlyEqual(C.Penetration, B.Penetration, 1e-3f));
	TestTrue(TEXT("surface point matches box"), C.SurfacePoint.Equals(B.SurfacePoint, 1e-3));
	TestTrue(TEXT("no bone/mesh (static)"), C.Bone.IsNone() && C.SourceMesh == nullptr);
	return true;
}

// Outside a convex face: an outward normal and a penetration, with no contact beyond the radius.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeConvexOutsideFaceTest,
	"DynamicRope.Collision.ConvexOutsideFace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeConvexOutsideFaceTest::RunTest(const FString& Parameters)
{
	const FVector H(50.0);
	FRopeConvexCollider Convex(MakeAABoxPlanes(FVector::ZeroVector, H), FBox(-H, H));

	// Just outside the positive X face and within the radius, so it contacts with that normal.
	{
		const FRopeContact C = Convex.Query(FVector(52.0, 0, 0), 5.0f);
		TestTrue(TEXT("outside face hit"), C.bHit);
		TestTrue(TEXT("normal +X"), C.Normal.Equals(FVector(1, 0, 0), 1e-4));
		TestTrue(TEXT("penetration = 3"), FMath::IsNearlyEqual(C.Penetration, 3.0f, 1e-3f));
	}
	// Beyond the radius, so there is no contact.
	{
		const FRopeContact C = Convex.Query(FVector(56.0, 0, 0), 5.0f);
		TestFalse(TEXT("beyond radius no hit"), C.bHit);
	}
	return true;
}

// Outside a convex edge: the maximum-plane test underestimates the distance, so contact engages
// conservatively, meaning slightly early, which prevents tunnelling. The normal is still a valid outward
// direction, namely a face normal. This regression test documents that it is an approximation, unlike the
// box's exact diagonal normal.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeConvexEdgeConservativeTest,
	"DynamicRope.Collision.ConvexEdgeConservative",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeConvexEdgeConservativeTest::RunTest(const FString& Parameters)
{
	const FVector H(50.0);
	FRopeConvexCollider Convex(MakeAABoxPlanes(FVector::ZeroVector, H), FBox(-H, H));

	// Diagonally outside the positive X and Y edge: each face is three away while the real edge is further,
	// and the maximum-plane test underestimates it as three.
	const FVector P(53.0, 53.0, 0.0);
	// At this radius the real distance would mean no contact, but the underestimate registers one, which is
	// the conservative behaviour.
	const FRopeContact C = Convex.Query(P, 4.0f);
	TestTrue(TEXT("edge contact triggers early (conservative)"), C.bHit);
	TestTrue(TEXT("penetration = 4 - 3 = 1"), FMath::IsNearlyEqual(C.Penetration, 1.0f, 1e-3f));
	// The normal is one of the two faces, axis aligned, and is a unit vector pointing outwards.
	TestTrue(TEXT("normal is unit"), FMath::IsNearlyEqual(static_cast<float>(C.Normal.Size()), 1.0f, 1e-3f));
	const bool bAxisAligned = C.Normal.Equals(FVector(1, 0, 0), 1e-3) || C.Normal.Equals(FVector(0, 1, 0), 1e-3);
	TestTrue(TEXT("normal is an outward face normal"), bAxisAligned);
	return true;
}

// A dynamic box: whether the surface velocity is derived from the previous frame's transform as the
// movement of the material point over the delta, which is what drags a rope along a moving surface.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxSurfaceVelocityTest,
	"DynamicRope.Collision.BoxSurfaceVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxSurfaceVelocityTest::RunTest(const FString& Parameters)
{
	// The box moved along positive X this frame, with the previous centre offset accordingly.
	FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, FVector(50.0));
	Box.PrevCenter = FVector(-10.0, 0.0, 0.0);
	Box.InvDeltaTime = 60.0f;

	// Contact just outside the positive X face. The material point's world position moved by that same
	// offset, giving a surface velocity along positive X.
	const FRopeContact C = Box.Query(FVector(52.0, 0.0, 0.0), 5.0f);
	TestTrue(TEXT("moving box hit"), C.bHit);
	TestTrue(TEXT("normal +X"), C.Normal.Equals(FVector(1, 0, 0), 1e-4));
	TestTrue(FString::Printf(TEXT("surface velocity %s should be (600,0,0)"), *C.SurfaceVelocity.ToString()),
		C.SurfaceVelocity.Equals(FVector(600.0, 0.0, 0.0), 1e-2));
	return true;
}

// The convex rigid transform: whether a world query is correct given body-local planes plus a rigid
// transform, where world space is the local space composed with it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeConvexRigidTransformTest,
	"DynamicRope.Collision.ConvexRigidTransform",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeConvexRigidTransformTest::RunTest(const FString& Parameters)
{
	// A local box as six planes, with a translation, which puts its positive X face further out in world
	// space.
	const FVector H(50.0);
	FRopeConvexCollider Cv(MakeAABoxPlanes(FVector::ZeroVector, H), FBox(-H, H), FQuat::Identity, FVector(100.0, 0.0, 0.0));

	// A world point just outside that face maps to a local point violating the positive X plane, giving that
	// normal, a surface point on the face, and the expected penetration.
	const FRopeContact C = Cv.Query(FVector(152.0, 0.0, 0.0), 5.0f);
	TestTrue(TEXT("translated convex hit"), C.bHit);
	TestTrue(TEXT("normal +X"), C.Normal.Equals(FVector(1, 0, 0), 1e-3));
	TestTrue(FString::Printf(TEXT("surface point %s should be (150,0,0)"), *C.SurfacePoint.ToString()),
		C.SurfacePoint.Equals(FVector(150.0, 0.0, 0.0), 1e-3));
	TestTrue(TEXT("penetration = 3"), FMath::IsNearlyEqual(C.Penetration, 3.0f, 1e-3f));

	// The same with a rotated rigid transform: a quarter turn about Z plus a translation puts the local
	// positive X face along positive Y in world space.
	{
		const FQuat Rot(FVector::UpVector, HALF_PI);
		FRopeConvexCollider CvR(MakeAABoxPlanes(FVector::ZeroVector, H), FBox(-H, H), Rot, FVector::ZeroVector);
		// The local positive X face maps to positive Y in world space, so a query just outside it gives a
		// world normal along positive Y.
		const FRopeContact CR = CvR.Query(FVector(0.0, 52.0, 0.0), 5.0f);
		TestTrue(TEXT("rotated convex hit"), CR.bHit);
		TestTrue(FString::Printf(TEXT("rotated normal %s should be +Y"), *CR.Normal.ToString()),
			CR.Normal.Equals(FVector(0, 1, 0), 1e-3));
	}
	return true;
}

// A dynamic convex: deriving the surface velocity when the rigid transform moves.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeConvexSurfaceVelocityTest,
	"DynamicRope.Collision.ConvexSurfaceVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeConvexSurfaceVelocityTest::RunTest(const FString& Parameters)
{
	const FVector H(50.0);
	FRopeConvexCollider Cv(MakeAABoxPlanes(FVector::ZeroVector, H), FBox(-H, H), FQuat::Identity, FVector(100.0, 0.0, 0.0));
	// It moved along positive X this frame.
	Cv.PrevTrans = FVector(90.0, 0.0, 0.0);
	Cv.InvDeltaTime = 60.0f;

	// The material point's world position moved by that same offset, giving a surface velocity along
	// positive X.
	const FRopeContact C = Cv.Query(FVector(152.0, 0.0, 0.0), 5.0f);
	TestTrue(TEXT("moving convex hit"), C.bHit);
	TestTrue(FString::Printf(TEXT("surface velocity %s should be (600,0,0)"), *C.SurfaceVelocity.ToString()),
		C.SurfaceVelocity.Equals(FVector(600.0, 0.0, 0.0), 1e-2));
	return true;
}

// Solver integration: whether a rope draped over a box corner stays out of the box's interior after many
// frames. It is the solver-level regression test for the corner rounding that let the rope pass through.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxSolverCornerDrapeTest,
	"DynamicRope.Solver.BoxCornerNoPenetration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxSolverCornerDrapeTest::RunTest(const FString& Parameters)
{
	// The rope lies across the positive X edge above the box and drapes under gravity.
	const FVector HalfExtents(50.0);
	FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, HalfExtents);
	TArray<IRopeCollider*> Colliders;
	Colliders.Add(&Box);

	FRopeSimState Sim = RopeTest::MakeStraightRope(24, 300.0f, FVector(-150.0, 0.0, 55.0), FVector(1, 0, 0));

	FRopeSolverConfig Config;
	Config.Substeps = 8;
	Config.Iterations = 4;
	Config.Gravity = FVector(0.0f, 0.0f, -980.0f);
	Config.CollisionRadius = 2.0f;

	const FRopeXPBDSolver Solver;
	float MaxInsideDepth = 0.0f;
	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		Solver.Step(Sim, Config, Colliders, 1.0f / 60.0f);
		// Checked every frame: no node may penetrate the box's interior, which catches even a momentary
		// pass-through.
		for (const FVector& P : Sim.Positions)
		{
			const FVector A = P.GetAbs();
			if (A.X < HalfExtents.X && A.Y < HalfExtents.Y && A.Z < HalfExtents.Z)
			{
				const float Depth = static_cast<float>(FMath::Min3(
					HalfExtents.X - A.X, HalfExtents.Y - A.Y, HalfExtents.Z - A.Z));
				MaxInsideDepth = FMath::Max(MaxInsideDepth, Depth);
			}
		}
	}

	TestTrue(FString::Printf(TEXT("max inside depth %.3f cm should be < 0.5"), MaxInsideDepth), MaxInsideDepth < 0.5f);
	TestFalse(TEXT("no NaN"), RopeTest::AnyNaN(Sim));
	return true;
}


// ===== Attributed extraction of simple collision, for the full-set wrap =====

// Whether the presence of the attribution parameters decides the bone, source mesh and static status of
// the sphyls and boxes, including the convex OBB fallback.
// The contract of URopeWrapTargetComponent's full-set mode: an attributed set takes part in detection,
// while an unattributed one, as produced by the previous callers, is push-out only.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBodyExtractionAttributionTest,
	"DynamicRope.Collision.BodyExtractionAttribution",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBodyExtractionAttributionTest::RunTest(const FString& Parameters)
{
	UBodySetup* Setup = NewObject<UBodySetup>(GetTransientPackage());

	FKSphylElem Sphyl(10.0f, 60.0f);
	Setup->AggGeom.SphylElems.Add(Sphyl);
	FKSphereElem Sphere(8.0f);
	Setup->AggGeom.SphereElems.Add(Sphere);
	FKBoxElem BoxElem(20.0f, 30.0f, 40.0f);
	Setup->AggGeom.BoxElems.Add(BoxElem);
	// An uncooked convex, with no planes, plus a valid element box takes the OBB fallback path, which is
	// eligible for attribution.
	FKConvexElem Convex;
	Convex.ElemBox = FBox(FVector(-5.0), FVector(5.0));
	Setup->AggGeom.ConvexElems.Add(Convex);

	const FTransform CompTM = FTransform::Identity;
	const FName VirtualBone(TEXT("Prop_RopeWrapAnchor"));

	// One: an attributed call gives every capsule and box, including the convex fallback, a virtual bone and
	// makes them take part in detection.
	{
		TArray<FRopeBoxCollider> Boxes;
		TArray<FRopeStaticCapsuleCollider> Capsules;
		TArray<FRopeConvexCollider> Convexes;
		int32 FallbackCount = 0;
		RopeBodyColliderExtraction::AppendBodyColliders(*Setup, CompTM, CompTM, 0.0f, 32, 32,
			Boxes, Capsules, Convexes, [&FallbackCount](int32) { ++FallbackCount; },
			VirtualBone, /*AttributionMesh*/ nullptr);

		TestEqual(TEXT("attributed capsules (sphyl+sphere)"), Capsules.Num(), 2);
		TestEqual(TEXT("attributed boxes (box+convex fallback)"), Boxes.Num(), 2);
		TestEqual(TEXT("no cooked convex colliders"), Convexes.Num(), 0);
		TestEqual(TEXT("convex fallback fired once"), FallbackCount, 1);
		for (const FRopeStaticCapsuleCollider& Cap : Capsules)
		{
			TestEqual(TEXT("capsule virtual bone"), Cap.Bone, VirtualBone);
			TestFalse(TEXT("attributed capsule joins detect"), Cap.IsWorldStatic());
		}
		for (const FRopeBoxCollider& B : Boxes)
		{
			TestEqual(TEXT("box virtual bone"), B.Bone, VirtualBone);
			TestFalse(TEXT("attributed box joins detect"), B.IsWorldStatic());
		}
	// Whether an attributed capsule's query carries the bone on the contact, which is the input contract of
	// the contact-to-wrap path.
		const FRopeContact C = Capsules[0].Query(Capsules[0].A + FVector(0, 0, 1) * (Capsules[0].Radius + 1.0f), 3.0f);
		TestTrue(TEXT("attributed capsule contact hit"), C.bHit);
		TestEqual(TEXT("contact carries virtual bone"), C.Bone, VirtualBone);
	}

	// Two: an unattributed call, as on the previous static body provider path, leaves everything with no
	// bone, making it push-out only, with behaviour unchanged.
	{
		TArray<FRopeBoxCollider> Boxes;
		TArray<FRopeStaticCapsuleCollider> Capsules;
		TArray<FRopeConvexCollider> Convexes;
		RopeBodyColliderExtraction::AppendBodyColliders(*Setup, CompTM, CompTM, 0.0f, 32, 32,
			Boxes, Capsules, Convexes, [](int32) {});

		for (const FRopeStaticCapsuleCollider& Cap : Capsules)
		{
			TestTrue(TEXT("unattributed capsule stays static"), Cap.Bone.IsNone() && Cap.IsWorldStatic());
		}
		for (const FRopeBoxCollider& B : Boxes)
		{
			TestTrue(TEXT("unattributed box stays static"), B.Bone.IsNone() && B.IsWorldStatic());
		}
	}

	return true;
}


// The convex attribution contract for a wrap target's full set: a virtual bone makes it take part in
// detection and makes its query carry the bone and mesh, and attributed extraction also carries the bone on
// a sheared box, which a rotated element with a non-uniform scale routes to a convex.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeConvexWrapAttributionTest,
	"DynamicRope.Collision.ConvexWrapAttribution",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeConvexWrapAttributionTest::RunTest(const FString& Parameters)
{
	const FName VirtualBone(TEXT("Prop_RopeWrapAnchor"));

	// One: a hand-built box-shaped convex of six planes is static before attribution and takes part in
	// detection, carrying the bone on its query, afterwards.
	{
		TArray<FPlane> Planes;
		Planes.Add(FPlane(FVector(1, 0, 0), 50.0));
		Planes.Add(FPlane(FVector(-1, 0, 0), 50.0));
		Planes.Add(FPlane(FVector(0, 1, 0), 50.0));
		Planes.Add(FPlane(FVector(0, -1, 0), 50.0));
		Planes.Add(FPlane(FVector(0, 0, 1), 50.0));
		Planes.Add(FPlane(FVector(0, 0, -1), 50.0));
		FRopeConvexCollider Cv(MoveTemp(Planes), FBox(FVector(-50.0), FVector(50.0)));
		TestTrue(TEXT("unattributed convex stays static"), Cv.IsWorldStatic());

		Cv.Bone = VirtualBone;
		TestFalse(TEXT("attributed convex joins detect"), Cv.IsWorldStatic());
		const FRopeContact C = Cv.Query(FVector(48.0, 0.0, 0.0), 3.0f);
		TestTrue(TEXT("attributed convex contact hit"), C.bHit);
		TestEqual(TEXT("contact carries virtual bone"), C.Bone, VirtualBone);
	}

	// Two: attributed extraction routes a rotated element with a non-uniform scale to a convex and carries
	// the bone on it.
	{
		UBodySetup* Setup = NewObject<UBodySetup>(GetTransientPackage());
		FKBoxElem BoxElem(20.0f, 30.0f, 40.0f);
		BoxElem.Rotation = FRotator(0.0f, 30.0f, 0.0f);
		Setup->AggGeom.BoxElems.Add(BoxElem);

		FTransform CompTM = FTransform::Identity;
		CompTM.SetScale3D(FVector(1.0, 2.0, 1.0));

		TArray<FRopeBoxCollider> Boxes;
		TArray<FRopeStaticCapsuleCollider> Capsules;
		TArray<FRopeConvexCollider> Convexes;
		RopeBodyColliderExtraction::AppendBodyColliders(*Setup, CompTM, CompTM, 0.0f, 32, 32,
			Boxes, Capsules, Convexes, [](int32) {}, VirtualBone, /*AttributionMesh*/ nullptr);

		TestEqual(TEXT("sheared box routed to convex"), Convexes.Num(), 1);
		if (Convexes.Num() == 1)
		{
			TestEqual(TEXT("extracted convex virtual bone"), Convexes[0].Bone, VirtualBone);
			TestFalse(TEXT("extracted convex joins detect"), Convexes[0].IsWorldStatic());
		}
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
