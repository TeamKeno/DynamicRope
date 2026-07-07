// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeBoxCollider 단위 테스트. 핵심은 모서리/엣지에서의 정확한 대각 normal —
// GDF 복셀 라운딩으로 로프가 박스 모서리를 관통하던 버그의 회귀 게이트다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Collision/RopeStaticCollider.h"
#include "Solver/RopeXPBDSolver.h"
#include "RopeTestHelpers.h"

// 모서리 바깥 사선 위치에서 push-out normal이 (면 법선이 아닌) 정확한 대각 방향인가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxCornerPushOutTest,
	"DynamicRope.Collision.BoxCornerPushOut",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxCornerPushOutTest::RunTest(const FString& Parameters)
{
	// 1) 축 정렬 박스: 코너 (50,50,50) 바깥 대각 2cm, 질의 반경 5cm.
	{
		const FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, FVector(50.0));
		const FVector Corner(50.0, 50.0, 50.0);
		const FVector Dir = FVector(1, 1, 1).GetSafeNormal();
		const FVector P = Corner + Dir * 2.0;

		const FRopeContact C = Box.Query(P, 5.0f);
		TestTrue(TEXT("corner overlap hit"), C.bHit);
		TestTrue(TEXT("normal is unit"), FMath::IsNearlyEqual(static_cast<float>(C.Normal.Size()), 1.0f, 1e-3f));
		// 대각 normal — GDF였다면 복셀 라운딩으로 면 법선 쪽으로 뭉개지던 값.
		TestTrue(FString::Printf(TEXT("normal %s should equal corner diagonal %s"), *C.Normal.ToString(), *Dir.ToString()),
			C.Normal.Equals(Dir, 1e-3));
		TestTrue(TEXT("penetration = 5 - 2 = 3"), FMath::IsNearlyEqual(C.Penetration, 3.0f, 1e-3f));
		// push-out 후 노드는 코너에서 정확히 질의 반경만큼 떨어진다.
		const FVector Pushed = P + C.Normal * C.Penetration;
		TestTrue(TEXT("pushed node sits at query radius from corner"),
			FMath::IsNearlyEqual(static_cast<float>(FVector::Dist(Pushed, Corner)), 5.0f, 1e-3f));
		TestTrue(TEXT("surface point is the corner"), C.SurfacePoint.Equals(Corner, 1e-3));
		// 정적 월드 지오메트리 계약: 귀속 없음 / 표면 속도 0.
		TestTrue(TEXT("no bone attribution"), C.Bone.IsNone());
		TestTrue(TEXT("no source mesh"), C.SourceMesh == nullptr);
		TestTrue(TEXT("zero surface velocity"), C.SurfaceVelocity.IsNearlyZero());
	}

	// 2) 회전 + 평행이동된 박스에서도 동일해야 한다(로컬 변환 검증).
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

		// OBB -> AABB bounds가 회전된 코너를 포함하는가(브로드페이즈 계약).
		const FBox Bounds = Box.GetWorldBounds();
		TestTrue(TEXT("world bounds contains rotated corner"), Bounds.IsInsideOrOn(CornerW));
	}

	return true;
}

// 박스 내부 노드는 침투가 가장 얕은 면의 바깥으로 밀려나는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxInsideMinFaceTest,
	"DynamicRope.Collision.BoxInsideMinFace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxInsideMinFaceTest::RunTest(const FString& Parameters)
{
	const FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, FVector(50.0));
	// 내부점: +X 면까지 5, +Y 면까지 40, -Z 면까지 30 -> 최소 침투 면은 +X.
	const FVector P(45.0, 10.0, -20.0);

	const FRopeContact C = Box.Query(P, 2.0f);
	TestTrue(TEXT("inside hit"), C.bHit);
	TestTrue(FString::Printf(TEXT("normal %s should be +X face"), *C.Normal.ToString()),
		C.Normal.Equals(FVector(1, 0, 0), 1e-4));
	// 침투 = 노드 반지름(2) + 면까지 깊이(5) = 7 -> push-out 후 x = 52 (표면 + 반지름).
	TestTrue(TEXT("penetration = 7"), FMath::IsNearlyEqual(C.Penetration, 7.0f, 1e-3f));
	const FVector Pushed = P + C.Normal * C.Penetration;
	TestTrue(TEXT("pushed to surface + radius"), Pushed.Equals(FVector(52.0, 10.0, -20.0), 1e-3));
	TestTrue(TEXT("surface point on +X face"), C.SurfacePoint.Equals(FVector(50.0, 10.0, -20.0), 1e-3));
	return true;
}

// 기본 QuerySwept(라인 샘플 폴백)가 얇은 박스 벽을 통과하지 않고 진입면에서 잡는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxSweptTunnelingTest,
	"DynamicRope.Collision.BoxQuerySweptTunneling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxSweptTunnelingTest::RunTest(const FString& Parameters)
{
	// 두께 4cm(반폭 2)의 얇은 벽을 60cm 이동으로 관통 시도.
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

// 솔버 통합: 박스 모서리 위로 드레이프된 로프가 여러 프레임 뒤에도 박스 내부로 파고들지 않는가
// (GDF 모서리 라운딩 관통 버그의 솔버 레벨 회귀 테스트).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeBoxSolverCornerDrapeTest,
	"DynamicRope.Solver.BoxCornerNoPenetration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeBoxSolverCornerDrapeTest::RunTest(const FString& Parameters)
{
	// 박스(반폭 50) 위 z=55에서 +X 엣지(x=50)를 가로질러 걸친 로프. 중력으로 낙하/드레이프.
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
		// 매 프레임 검사: 어떤 노드도 박스 내부로 파고들면 안 된다(순간 관통도 잡는다).
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

#endif // WITH_DEV_AUTOMATION_TESTS
