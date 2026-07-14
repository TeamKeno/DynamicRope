// Copyright Epic Games, Inc. All Rights Reserved.
//
// Pierce(꽂힘) 결착 모드 단위 테스트 — ③ GuaranteedWrap 전용, aim-hit 접점에 단일 앵커로 성립.
// 세 계약을 잠근다: ①②③↔결착 조합 제약, preview 빌더의 단일 앵커 산출, 단일 앵커 커밋(BeginWrap).

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Core/RopeTypes.h"
#include "Logic/RopeWrapController.h"
#include "Logic/RopeThrowPreviewBuilder.h"
#include "Components/SkeletalMeshComponent.h"
#include "RopeTestHelpers.h"

namespace
{
	USkeletalMeshComponent* MakePierceMockMesh()
	{
		return NewObject<USkeletalMeshComponent>();
	}
}

// 조합 제약: ①② = BareWrap 전용, ③ = Pierce/Cinch 전용(무효 조합은 모드 기본으로 보정, ③→Pierce).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceClampEngagementTest,
	"DynamicRope.Pierce.ClampEngagementContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceClampEngagementTest::RunTest(const FString& Parameters)
{
	using namespace RopeWrapModes;

	// ③ GuaranteedWrap = Pierce/Cinch만 허용, BareWrap은 Pierce로 보정.
	TestFalse(TEXT("③+BareWrap 불가"), IsEngagementAllowed(ERopeWrapResolveMode::GuaranteedWrap, ERopeTipEngagement::BareWrap));
	TestTrue(TEXT("③+Pierce 허용"), IsEngagementAllowed(ERopeWrapResolveMode::GuaranteedWrap, ERopeTipEngagement::Pierce));
	TestTrue(TEXT("③+Cinch 허용"), IsEngagementAllowed(ERopeWrapResolveMode::GuaranteedWrap, ERopeTipEngagement::Cinch));
	TestEqual(TEXT("③+BareWrap → Pierce로 보정"),
		ClampEngagement(ERopeWrapResolveMode::GuaranteedWrap, ERopeTipEngagement::BareWrap), ERopeTipEngagement::Pierce);
	TestEqual(TEXT("③+Pierce 유지"),
		ClampEngagement(ERopeWrapResolveMode::GuaranteedWrap, ERopeTipEngagement::Pierce), ERopeTipEngagement::Pierce);

	// ①② AssistedJudged/FullSimulation = BareWrap만, Pierce는 BareWrap으로 보정.
	TestFalse(TEXT("②+Pierce 불가"), IsEngagementAllowed(ERopeWrapResolveMode::AssistedJudged, ERopeTipEngagement::Pierce));
	TestEqual(TEXT("②+Pierce → BareWrap으로 보정"),
		ClampEngagement(ERopeWrapResolveMode::AssistedJudged, ERopeTipEngagement::Pierce), ERopeTipEngagement::BareWrap);
	TestEqual(TEXT("①+Cinch → BareWrap으로 보정"),
		ClampEngagement(ERopeWrapResolveMode::FullSimulation, ERopeTipEngagement::Cinch), ERopeTipEngagement::BareWrap);
	return true;
}

// Pierce preview 빌더: aim-hit 후보에서 감김 나선 없이 단일 앵커(LatchAnchor)만 산출하는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceSingleAnchorPreviewTest,
	"DynamicRope.Pierce.PreviewYieldsSingleAnchor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceSingleAnchorPreviewTest::RunTest(const FString& Parameters)
{
	// 노드 x=0,20,...,140(segLen 20). aim이 (60,0,0)의 "spine" 본을 맞힘 → node 3에 단일 앵커.
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();

	FRopeThrowPreviewBuilder::FInput Input;
	Input.Sim = &Sim;
	Input.RopeLength = 140.0f;
	Input.ReachScale = 1.0f;
	Input.RopeRadius = 2.0f;
	Input.TipEngagement = ERopeTipEngagement::Pierce;
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
	TestEqual(TEXT("Pierce는 단일 앵커"), Prepared.Anchors.Num(), 1);
	TestTrue(TEXT("앵커 본 = 대상 본"), Prepared.Bone == FName("spine"));
	TestTrue(TEXT("앵커 mesh = 대상 mesh"), Prepared.Mesh.Get() == Mesh);
	if (Prepared.Anchors.Num() == 1)
	{
		TestTrue(TEXT("단일 앵커 = LatchAnchor 노드"), Prepared.Anchors[0].NodeIndex == Prepared.LatchAnchor.NodeIndex);
		// Pierce는 창(팁=마지막 노드)이 꽂히는 것 — 거리 기반 안쪽 노드가 아니라 밧줄 끝에 앵커가 박혀야
		// 팁 mesh가 꽂힘 지점에 오고 끝이 처지지 않는다.
		TestEqual(TEXT("Pierce 앵커는 밧줄 끝(팁=창) 노드"),
			Prepared.Anchors[0].NodeIndex, Prepared.PreviewSim.Num() - 1);
	}
	TestTrue(TEXT("렌더 preview 유효(직선 centerline)"), Prepared.RenderPreview.IsValid());
	return true;
}

// 단일 앵커 커밋: 앵커 1개 seed를 BeginWrap이 그대로 Wrapped로 고정하는가(커밋 경로 앵커 개수 무관).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceSingleAnchorBeginWrapTest,
	"DynamicRope.Pierce.SingleAnchorBeginWrap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceSingleAnchorBeginWrapTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	const USkeletalMeshComponent* Mesh = MakePierceMockMesh();

	// Pierce가 만드는 것과 동형인 단일 앵커 seed(노드 3).
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

	TestTrue(TEXT("단일 앵커로 Wrapped 성립"), Wrap.State.IsWrapped());
	TestEqual(TEXT("앵커 1개 유지"), Wrap.State.Anchors.Num(), 1);
	TestTrue(TEXT("커밋 본 = spine"), Wrap.State.BoneName == FName("spine"));
	// 꽂힌 노드는 InvMass=0으로 핀(솔버가 아니라 본이 구동).
	TestTrue(TEXT("override frame이 산출됨"), Frame.HasAny());
	if (Frame.InvMass.IsValidIndex(PierceNode))
	{
		TestEqual(TEXT("꽂힌 노드 InvMass=0 핀"), Frame.InvMass[PierceNode], 0.0f);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
