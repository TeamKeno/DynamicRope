// Copyright Epic Games, Inc. All Rights Reserved.
//
// Wrapping 페이즈(FRopeWrappingPhase) 단위 테스트. 정적 랩 대상 경로(가상 본 이름 + 컴포넌트
// 트랜스폼)를 이용해 월드 없이 SurfaceVectorField 경로 빌드를 완주시킨다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrappingPhase.h"
#include "Collision/RopeCollider.h"
#include "Components/SceneComponent.h"
#include "RopeTestHelpers.h"

namespace
{
	// 정적 랩 대상 경로를 그대로 쓰는 mock 대상 — ResolveBindingWorld가 가상 본 이름에 대해
	// 경고 없이 컴포넌트(identity) 트랜스폼을 반환하므로 본 로컬 = 월드가 된다.
	USceneComponent* MakeMockTarget()
	{
		return NewObject<USceneComponent>();
	}
}

// SurfaceVectorField 경로 빌드가 캡슐(원기둥) 주위를 완주하고, 앵커가 표면 위에 놓이며, 빌드 중
// 적분한 누적 감싼 각도가 단일 축 helix 공식과 일치하는가 — rolling axis(본 전환 시 축 재해석)
// 도입 후에도 단일 본(전환 없음) 결과가 기존 공식과 동치임을 고정하는 회귀 계약.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingCapsuleAngleTest,
	"DynamicRope.Wrapping.SurfaceVectorFieldCapsuleWrapAngle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingCapsuleAngleTest::RunTest(const FString& Parameters)
{
	// 캡슐: Z축 정렬(-50~+50), 반지름 25 → 둘레 ≈157cm. 로프 160cm(9노드)면 약 한 바퀴 감는다.
	USceneComponent* Mesh = MakeMockTarget();
	FCapsuleCollider Capsule(FVector(0, 0, -50), FVector(0, 0, 50), 25.0f, FName("arm"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Capsule };

	// 로프는 latch 지점(+X 표면)에서 +Y(원주 방향)로 뻗어 있다.
	FRopeSimState Sim = RopeTest::MakeStraightRope(9, 160.0f, FVector(25, 0, 0), FVector(0, 1, 0));

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = FName("arm");
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(25, 0, 0);
	Latch.LocalNormal = FVector(1, 0, 0);
	Latch.LocalTangent = FVector(0, 1, 0);
	Latch.StartWorldPosition = FVector(25, 0, 0);
	Latch.SurfaceOffset = 1.0f;

	const FRopeWrapConfig Config;
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		ERopeWrappingPathMode::SurfaceVectorField, /*SurfaceOffset*/ 1.0f, TEXT("WrappingTest"), true };

	FRopeWrappingPhase Wrapping;
	TestTrue(TEXT("wrapping begins on capsule"), Wrapping.Begin(Latch, Mesh, FName("arm"), 0.16f, Sim, Ctx));

	for (int32 Iteration = 0; Iteration < 256 && Wrapping.State.bPathBuildActive; ++Iteration)
	{
		Wrapping.AdvancePathBuild(Sim, Ctx);
	}

	TestTrue(TEXT("path build completes around the capsule"), Wrapping.State.bPathBuildComplete);
	TestEqual(TEXT("one anchor per tail node"), Wrapping.State.Anchors.Num(), 9);

	// 모든 anchor는 캡슐 표면(축 반경 25cm) 위 — identity 트랜스폼이라 로컬 XY 거리로 바로 잰다.
	for (const FRopeSurfaceAnchor& Anchor : Wrapping.State.Anchors)
	{
		const float RadialDistance = static_cast<float>(
			FVector2D(Anchor.LocalSurfacePosition.X, Anchor.LocalSurfacePosition.Y).Size());
		TestTrue(FString::Printf(TEXT("anchor %d on capsule surface (r=%.1f)"), Anchor.NodeIndex, RadialDistance),
			FMath::IsNearlyEqual(RadialDistance, 25.0f, 3.0f));
	}

	// 누적 각도 ≈ 단일 축 helix 공식: angle = (거리 / 피치 보정) / 반지름 ≈ 356°.
	// 걷기가 현(chord) 스텝(세그먼트 절반 = 10cm)이라 호 기반 공식보다 스텝당 atan만큼 낮게 적분된다
	// (r=25에서 약 -4.7% ≈ -16°) — 허용 오차는 그 이산화 편차를 포함해 30°로 잡는다.
	float AngleDeg = 0.0f;
	TestTrue(TEXT("wrapped angle computable"), Wrapping.ComputeWrappedAngleAtLastBuiltPoint(Sim, Ctx, AngleDeg));
	const float PitchScale = Config.WrappingHelixPitchScale;
	const float ExpectedDeg = FMath::RadiansToDegrees(
		(160.0f / FMath::Sqrt(1.0f + PitchScale * PitchScale)) / 25.0f);
	TestTrue(FString::Printf(TEXT("accumulated angle matches helix formula (%.0f vs %.0f deg)"), AngleDeg, ExpectedDeg),
		FMath::IsNearlyEqual(AngleDeg, ExpectedDeg, 30.0f));
	return true;
}

#endif
