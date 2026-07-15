// Copyright Epic Games, Inc. All Rights Reserved.
//
// Pierce(꽂힘) 결착 모드 단위 테스트 — ③ GuaranteedWrap 전용, aim-hit 접점에 단일 앵커로 성립.
// 네 계약을 잠근다: ①②③↔결착 조합 제약, throw phase 게이트(③=Reel 전용), preview 빌더의 단일 앵커 산출,
// 단일 앵커 커밋(BeginWrap).

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Core/RopeTypes.h"
#include "Logic/RopeWrapController.h"
#include "Logic/RopeThrowPreviewBuilder.h"
#include "Components/SkeletalMeshComponent.h"
#include "RopeComponent.h"
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

// throw phase 게이트: ③는 Reel에서만 던질 수 있고, ①②는 phase 게이트가 없다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceThrowPhaseGateTest,
	"DynamicRope.Pierce.ThrowPhaseGateContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceThrowPhaseGateTest::RunTest(const FString& Parameters)
{
	using namespace RopeWrapModes;

	// ERopePhase에는 Count/MAX 센티넬이 없다(Reel로 끝남) — 새 phase가 추가되면 여기에 손으로 더해야 한다.
	static const ERopePhase AllPhases[] = {
		ERopePhase::Free, ERopePhase::Flight, ERopePhase::Contacting, ERopePhase::Wrapping,
		ERopePhase::Wrapped, ERopePhase::Releasing, ERopePhase::GuidedThrow, ERopePhase::Reel
	};

	// ③ GuaranteedWrap = Reel 전용.
	TestTrue(TEXT("③+Reel 던지기 성립"),
		CanThrowInPhase(ERopeWrapResolveMode::GuaranteedWrap, ERopePhase::Reel));
	for (const ERopePhase Phase : AllPhases)
	{
		if (Phase == ERopePhase::Reel)
		{
			continue;
		}
		TestFalse(*FString::Printf(TEXT("③+%d 던지기 불가(Reel 아님)"), static_cast<int32>(Phase)),
			CanThrowInPhase(ERopeWrapResolveMode::GuaranteedWrap, Phase));
	}

	// ①② = phase 게이트 없음 → 모든 phase에서 true.
	// 이 술어를 `Phase == Reel`로 "단순화"하면 여기서 터진다 — ①②가 조용히 막히는 회귀 방지선이다.
	for (const ERopePhase Phase : AllPhases)
	{
		TestTrue(*FString::Printf(TEXT("①+%d 게이트 없음"), static_cast<int32>(Phase)),
			CanThrowInPhase(ERopeWrapResolveMode::FullSimulation, Phase));
		TestTrue(*FString::Printf(TEXT("②+%d 게이트 없음"), static_cast<int32>(Phase)),
			CanThrowInPhase(ERopeWrapResolveMode::AssistedJudged, Phase));
	}
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

// Reel(장전) 전이: ③ 로프는 Free에서 EnterReel() → Reel. 비-③는 no-op. 던지기는 Reel에서만(허용 조건).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceEnterReelTest,
	"DynamicRope.Pierce.EnterReelTransition",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceEnterReelTest::RunTest(const FString& Parameters)
{
	// ③ 로프: 기본 Free → EnterReel → Reel.
	URopeComponent* Guaranteed = NewObject<URopeComponent>();
	Guaranteed->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	TestEqual(TEXT("기본 phase는 Free"), Guaranteed->GetPhase(), ERopePhase::Free);
	Guaranteed->EnterReel();
	TestEqual(TEXT("③ EnterReel → Reel"), Guaranteed->GetPhase(), ERopePhase::Reel);

	// 비-③(② Assisted): EnterReel은 no-op → Free 유지(Reel은 ③ 전용).
	URopeComponent* Assisted = NewObject<URopeComponent>();
	Assisted->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	Assisted->EnterReel();
	TestEqual(TEXT("② EnterReel은 no-op"), Assisted->GetPhase(), ERopePhase::Free);

	// Reel 허용 조건: Reel에서 다시 EnterReel은 Reel 유지(재진입 허용), 그 외 phase에선 무효.
	Guaranteed->EnterReel();
	TestEqual(TEXT("Reel에서 재진입해도 Reel"), Guaranteed->GetPhase(), ERopePhase::Reel);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
