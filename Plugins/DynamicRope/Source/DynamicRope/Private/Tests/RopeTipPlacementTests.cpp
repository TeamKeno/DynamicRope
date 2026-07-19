// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeTipPlacement socket-follow, pierce-embed, and aim-lock unit tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeTipPlacement.h"

// Pierce 임베드 배치 수학: 팁 소켓이 히트점에 관통 방향으로 박히고, 꼬리 소켓이 로프 연결점이 되는가.
// FRopeTipPlacement는 world 무의존이라 소켓 로컬만으로 계약을 잠근다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceEmbedMathTest,
	"DynamicRope.Pierce.EmbedPlacesTipAtHit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceEmbedMathTest::RunTest(const FString& Parameters)
{
	const FVector HitPoint(0, 60, 0);
	const FVector PierceDir(0, 1, 0); // 축 비정렬 방향으로 회전까지 검증.

	// 팁 소켓은 메쉬 X로 +50, 비자명 회전(Z 90°)까지 줘 소켓 회전에 무관한 계약을 확인한다.
	const FTransform TipSocketLocal(FQuat(FVector(0, 0, 1), HALF_PI), FVector(50, 10, 0));
	const FTransform TailSocketLocal(FQuat::Identity, FVector(-30, -5, 0)); // 꼬리는 Head와 다른 두 점 축을 만든다.

	FTransform ComponentWorld;
	FVector TailWorld;
	FRopeTipPlacement::SolvePierceEmbed(HitPoint, PierceDir, TipSocketLocal,
		/*bHasTailSocket*/ true, TailSocketLocal, ComponentWorld, TailWorld);

	const FTransform TipWorld = TipSocketLocal * ComponentWorld;
	TestTrue(TEXT("팁 소켓이 히트점에 박힘"), TipWorld.GetLocation().Equals(HitPoint, 0.01f));
	TestTrue(TEXT("Head-Tail 벡터 = 관통 방향"),
		(TipWorld.GetLocation() - TailWorld).GetSafeNormal().Equals(PierceDir, 0.01f));

	// 로프 연결점 = 꼬리 소켓 월드.
	const FVector ExpectedTail = (TailSocketLocal * ComponentWorld).GetLocation();
	TestTrue(TEXT("꼬리 소켓이 로프 연결점"), TailWorld.Equals(ExpectedTail, 0.01f));

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
	TestTrue(TEXT("Relative scale 이후에도 팁 소켓이 히트점에 박힘"),
		ScaledTipWorld.GetLocation().Equals(HitPoint, 0.01f));
	TestTrue(TEXT("Relative scale 이후에도 꼬리 소켓이 로프 연결점"),
		ScaledTailWorld.Equals(ScaledTailTransform.GetLocation(), 0.01f));
	TestTrue(TEXT("Relative scale 이후에도 Head-Tail 벡터 = 관통 방향"),
		(ScaledTipWorld.GetLocation() - ScaledTailTransform.GetLocation()).GetSafeNormal().Equals(PierceDir, 0.01f));
	TestTrue(TEXT("Relative scale 0.2 유지"),
		ScaledMeshWorld.GetScale3D().Equals(Relative.GetScale3D(), 0.001f));

	// 꼬리 소켓 없음 → 연결점은 메쉬 원점.
	FTransform CW2;
	FVector Tail2;
	FRopeTipPlacement::SolvePierceEmbed(HitPoint, PierceDir, TipSocketLocal,
		/*bHasTailSocket*/ false, FTransform::Identity, CW2, Tail2);
	TestTrue(TEXT("꼬리 소켓 없으면 연결점=메쉬 원점"), Tail2.Equals(CW2.GetLocation(), 0.01f));
	return true;
}

// Tail 소켓 추종 수학: 로프 끝 노드가 메쉬 원점이 아니라 꼬리 소켓에 붙어야 한다.
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
	TestTrue(TEXT("꼬리 소켓 위치 = 로프 연결점"),
		TailWorld.GetLocation().Equals(RopeAttachWorld, 0.01f));
	TestTrue(TEXT("Head-Tail 벡터 = 로프 끝 방향"),
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
	TestTrue(TEXT("RelativeTransform 이후에도 꼬리 소켓 위치 유지"),
		RenderedTailWorld.GetLocation().Equals(RopeAttachWorld, 0.01f));
	TestTrue(TEXT("RelativeTransform 이후에도 Head-Tail 방향 유지"),
		(RenderedHeadWorld.GetLocation() - RenderedTailWorld.GetLocation()).GetSafeNormal().Equals(ForwardDir, 0.01f));
	TestTrue(TEXT("Relative scale 0.2 유지"),
		RenderedMeshWorld.GetScale3D().Equals(Relative.GetScale3D(), 0.001f));
	return true;
}

// Wrapped 복원 round-trip: 얼린 bone-local(LocalMeshTransform)을 본 트랜스폼으로 되살리면 커밋 자세와 일치.
// UpdateTipMeshTransform의 Wrapped 분기(MeshWorld = LocalMeshTransform * BoneXform)가 이 항등에 의존한다.
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

	// 임의의 본 트랜스폼(위치+회전+스케일).
	const FTransform BoneXform(FQuat(FVector(0, 0, 1), 1.1f), FVector(300, 50, 20), FVector(1.0f));

	// 커밋 시 저장: LocalMeshTransform = ComponentWorld를 본 기준으로.
	const FTransform LocalMeshTransform = ComponentWorld.GetRelativeTransform(BoneXform);
	// 매 프레임 복원: MeshWorld = LocalMeshTransform * BoneXform == ComponentWorld여야 팝/드리프트가 없다.
	const FTransform Restored = LocalMeshTransform * BoneXform;

	TestTrue(TEXT("복원 위치 = 커밋 위치"), Restored.GetLocation().Equals(ComponentWorld.GetLocation(), 0.01f));
	TestTrue(TEXT("복원 회전 = 커밋 회전"),
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

	TestTrue(TEXT("상하 기울기 유지"),
		FMath::IsNearlyEqual(FVector::DotProduct(Locked, Up), FVector::DotProduct(SourceDir, Up), 0.001f));
	const FVector LockedFlat = FVector::VectorPlaneProject(Locked, Up).GetSafeNormal();
	const FVector AimFlat = FVector::VectorPlaneProject(AimDir, Up).GetSafeNormal();
	TestTrue(TEXT("수평 yaw는 aim 방향"), LockedFlat.Equals(AimFlat, 0.001f));

	const FVector VerticalAimResult = FRopeTipPlacement::MakeAimYawLockedDirection(
		SourceDir, FVector::UpVector, Up);
	TestTrue(TEXT("수평 aim이 없으면 원래 방향 유지"), VerticalAimResult.Equals(SourceDir, 0.001f));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

