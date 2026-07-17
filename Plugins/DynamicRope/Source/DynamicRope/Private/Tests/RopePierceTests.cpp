// Copyright Epic Games, Inc. All Rights Reserved.
//
// Pierce(꽂힘) 결착 모드 단위 테스트 — ③ GuaranteedWrap 전용, aim-hit 접점에 단일 앵커로 성립.
// 다섯 계약을 잠근다: ①②③↔결착 조합 제약, throw phase 게이트(③=Reel 전용), preview 빌더의 단일 앵커
// 산출, 단일 앵커 커밋(BeginWrap), ③ 연출 진입/이탈(Reel→GuidedThrow→Releasing).

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Core/RopeTypes.h"
#include "Logic/RopeWrapController.h"
#include "Logic/RopeThrowPreviewBuilder.h"
#include "Components/SkeletalMeshComponent.h"
#include "RopeComponent.h"
#include "Logic/RopeTipPlacement.h"
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

// ③ 연출 진입/이탈 phase 계약: Reel → (prepared throw) → GuidedThrow → (수동 해제) → Releasing.
// 이 수동 해제가 FinishWrapRelease의 GuidedThrow 분기(커밋 전 release)로 들어가는 유일한 public 진입로다.
//
// [테스트 범위의 한계 — 사실대로 적는다] 연출 중 abort(AbortGuidedThrow → OnRopeReleased)는 여기서
// 검증할 수 없다: ① UpdateGuidedThrow/PrepareSimFrame이 private이고 friend는 URopeSimSubsystem 하나뿐이라
// 구동할 수 없고, ② OnRopeReleased는 dynamic delegate라 UFUNCTION을 가진 UObject 리스너가 필요한데
// Private/Tests에 UCLASS가 없으며, ③ world가 없어 중앙 신호(OnAnyRopeReleased)는 애초에 도달 불가다
// (URopeSimSubsystem::Get(nullptr) == nullptr). 이벤트 발화 검증은 PIE 체크리스트가 담당한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceGuidedThrowEntryTest,
	"DynamicRope.Pierce.GuidedThrowEntryAndManualRelease",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceGuidedThrowEntryTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Rope->TipEngagement = ERopeTipEngagement::Pierce;
	Rope->RopeLength = 140.0f;
	Rope->NumParticles = 8;

	// 조준 성공 상황의 prepared를 world 없이 조립한다(PreviewYieldsSingleAnchor와 같은 경로).
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
	if (!TestTrue(FString::Printf(TEXT("prepared preview builds (%s)"), *Failure),
		FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, Prepared, &Failure)))
	{
		return false;
	}

	// Reel 밖에서는 던질 수 없다(CanThrowInPhase 계약의 실제 경로 확인).
	TestFalse(TEXT("Free에서는 prepared throw 거부"), Rope->ThrowWithPreparedPreview(Prepared));
	TestEqual(TEXT("거부 후 phase 불변"), Rope->GetPhase(), ERopePhase::Free);

	Rope->EnterReel();
	TestTrue(TEXT("Reel에서 prepared throw 성립"), Rope->ThrowWithPreparedPreview(Prepared));
	TestEqual(TEXT("prepared throw → GuidedThrow"), Rope->GetPhase(), ERopePhase::GuidedThrow);

	// 연출 중 수동 해제 → Releasing(커밋 전이므로 중앙 신호는 안 나가야 하지만 world 없이는 관측 불가).
	Rope->ReleaseWrap();
	TestEqual(TEXT("GuidedThrow 중 ReleaseWrap → Releasing"), Rope->GetPhase(), ERopePhase::Releasing);
	return true;
}

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

// ─── preview 대상 선택의 출처 계약 ─────────────────────────────────────────────
// BuildFreePreparedPreview는 aim hit이 없을 때 arc 전체를 재탐색할 수 있다. 이 재탐색은 조준과
// 무관한 옆 대상을 고르므로, 아래 세 갈래를 반드시 구분해야 한다. 셋 다 "aim hit 없음"이지만
// 기대 동작이 다르다 — 이 구분이 무너지면 조준이 빗나가도 엉뚱한 대상에 꽂힌다(실제 발생한 버그).
namespace
{
	// aim 축(+X)에서 비켜난 arc 위 대상. arc는 (AimDir, GuideUp) 평면을 alpha=1(+X)→0(-X)로 쓸고
	// alpha=0.5가 +Z이므로, +Z로 Radius의 절반 거리에 두면 재탐색이 반드시 줍는 위치가 된다
	// (RopeLength 140 × ReachScale 1 = arc 반경 140 → radial 샘플에 70이 포함된다).
	FCapsuleCollider MakeOffAimArcTarget(const USkeletalMeshComponent* Mesh)
	{
		return FCapsuleCollider(FVector(0, -20, 70), FVector(0, 20, 70), 25.0f, FName("spine"), Mesh);
	}

	// 조준 성공 케이스와 같은 fixture. 호출자가 bAimRayEvaluated/TipEngagement만 바꿔 갈래를 만든다.
	FRopeThrowPreviewBuilder::FInput MakeArcSearchInput(const FRopeSimState& Sim)
	{
		FRopeThrowPreviewBuilder::FInput Input;
		Input.Sim = &Sim;
		Input.RopeLength = 140.0f;
		Input.ReachScale = 1.0f;
		Input.RopeRadius = 2.0f;
		Input.ThrowContext.Origin = FVector::ZeroVector;
		Input.ThrowContext.FrameForward = FVector(1, 0, 0);
		Input.ThrowContext.FrameUp = FVector(0, 0, 1);
		// bHasAimGuideHit = false(기본) — 세 갈래 공통 전제.
		return Input;
	}
}

// 갈래 ①(회귀 잠금): 조준 ray가 돌았는데 빗나감 → preview 없음. 대상이 arc 안에 **있는데도** 그렇다.
// 수정 전에는 여기서 arc 재탐색이 옆 대상을 주워 preview가 성립했고, 그게 리포트된 버그였다
// (청록=miss인데 preview가 ray 밖 대상으로 연결됨). 실패 사유까지 확인해 "arc가 아무것도 못 찾아서
// 우연히 false"인 공허한 통과와 구분한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceAimMissYieldsNoPreviewTest,
	"DynamicRope.Pierce.AimMissYieldsNoPreview",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceAimMissYieldsNoPreviewTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();
	FCapsuleCollider Target = MakeOffAimArcTarget(Mesh);
	TArray<IRopeCollider*> Colliders = { &Target };

	FRopeThrowPreviewBuilder::FInput Input = MakeArcSearchInput(Sim);
	Input.Colliders = &Colliders;
	Input.TipEngagement = ERopeTipEngagement::Pierce;
	Input.ThrowContext.bAimRayEvaluated = true; // 조준했고 — 빗나갔다.

	FRopePreparedThrowPreview Prepared;
	FString Failure;
	TestFalse(TEXT("조준 miss면 arc 재탐색 없이 preview 실패"),
		FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, Prepared, &Failure));
	TestFalse(TEXT("prepared 무효"), Prepared.IsValid());
	TestTrue(FString::Printf(TEXT("사유가 조준 게이트여야 한다(arc 미발견이 아니라): %s"), *Failure),
		Failure.Contains(TEXT("aim ray found no target")));
	return true;
}

// 갈래 ②: 조준 자체가 없는 BP 직행/AI라도 Pierce는 arc 재탐색을 하지 않는다. 창은 조준한 곳에
// 꽂히는 것이 전부라, 방향만 보고 최대 SweepAngleDegrees 폭의 옆 대상에 꽂으면 같은 버그가 된다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceDirectThrowRequiresAimHitTest,
	"DynamicRope.Pierce.DirectPierceRequiresAimHit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceDirectThrowRequiresAimHitTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();
	FCapsuleCollider Target = MakeOffAimArcTarget(Mesh);
	TArray<IRopeCollider*> Colliders = { &Target };

	FRopeThrowPreviewBuilder::FInput Input = MakeArcSearchInput(Sim);
	Input.Colliders = &Colliders;
	Input.TipEngagement = ERopeTipEngagement::Pierce;
	Input.ThrowContext.bAimRayEvaluated = false; // 조준 흐름 없음(BP 직행/AI).

	FRopePreparedThrowPreview Prepared;
	FString Failure;
	TestFalse(TEXT("Pierce는 조준 hit 없이 preview가 성립하지 않는다"),
		FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, Prepared, &Failure));
	TestTrue(FString::Printf(TEXT("사유가 pierce 게이트여야 한다: %s"), *Failure),
		Failure.Contains(TEXT("pierce requires an aim hit")));
	return true;
}

// 갈래 ③(보존 잠금): 조준 없는 BP 직행 + Cinch(감김)는 arc 재탐색이 **의도된** 대상 선택 수단이다
// ("이 방향으로 던져 거기 있는 걸 감아라"). 위 두 게이트가 이 경로까지 막으면 안 된다.
// collider를 비워 arc 탐색 자체의 사유("no frame colliders")로 실패시킨다 — 감김 경로 전체를
// 세우지 않고도 "게이트에 막힌 게 아니라 탐색까지 도달했다"만 정확히 확인한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCinchDirectThrowStillSearchesArcTest,
	"DynamicRope.Pierce.DirectCinchStillSearchesArc",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCinchDirectThrowStillSearchesArcTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	TArray<IRopeCollider*> NoColliders;

	FRopeThrowPreviewBuilder::FInput Input = MakeArcSearchInput(Sim);
	Input.Colliders = &NoColliders;
	Input.TipEngagement = ERopeTipEngagement::Cinch;
	Input.ThrowContext.bAimRayEvaluated = false; // 조준 흐름 없음(BP 직행/AI).

	FRopePreparedThrowPreview Prepared;
	FString Failure;
	FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, Prepared, &Failure);
	TestTrue(FString::Printf(TEXT("BP 직행 Cinch는 arc 탐색까지 도달해야 한다(게이트 아님): %s"), *Failure),
		Failure.Contains(TEXT("free search")));
	return true;
}

// arc 탐색이 aim과 같은 wrap 대상 기준을 쓰는가. 종전에는 arc 탐색이 Bone/SourceMesh만 보고
// CanWrapTarget을 몰라서, aim이 금지한 대상을 preview가 주웠다("보이는데 던지면 거부됨").
// 게이트는 주입식이라(FInput.CanWrapTarget) 월드/서브클래스 없이 람다로 계약을 잠글 수 있다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeCinchArcSearchHonorsWrapGateTest,
	"DynamicRope.Pierce.ArcSearchHonorsWrapGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeCinchArcSearchHonorsWrapGateTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();
	FCapsuleCollider Target = MakeOffAimArcTarget(Mesh);
	TArray<IRopeCollider*> Colliders = { &Target };

	// 먼저 게이트 없이: arc 탐색이 이 대상을 실제로 줍는다는 것부터 확인한다(아래 대조군의 전제).
	FRopeThrowPreviewBuilder::FInput Allowed = MakeArcSearchInput(Sim);
	Allowed.Colliders = &Colliders;
	Allowed.TipEngagement = ERopeTipEngagement::Cinch;
	FRopePreparedThrowPreview AllowedPrepared;
	FString AllowedFailure;
	const bool bAllowedBuilt = FRopeThrowPreviewBuilder::BuildFreePreparedPreview(
		Allowed, AllowedPrepared, &AllowedFailure);
	TestTrue(FString::Printf(TEXT("전제: 게이트 미설정이면 arc 탐색이 이 대상을 줍는다 (%s)"), *AllowedFailure),
		bAllowedBuilt);

	// 같은 대상 + 거부 게이트 → 후보에서 빠져야 한다. 위 전제가 성립하므로 이 false는 게이트 때문이다.
	FRopeThrowPreviewBuilder::FInput Denied = MakeArcSearchInput(Sim);
	Denied.Colliders = &Colliders;
	Denied.TipEngagement = ERopeTipEngagement::Cinch;
	int32 GateCalls = 0;
	Denied.CanWrapTarget = [&GateCalls](const USceneComponent*, FName) { ++GateCalls; return false; };

	FRopePreparedThrowPreview DeniedPrepared;
	FString DeniedFailure;
	TestFalse(TEXT("CanWrapTarget이 거부한 대상은 arc 탐색 후보에서 빠진다"),
		FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Denied, DeniedPrepared, &DeniedFailure));
	TestTrue(TEXT("게이트가 실제로 호출됐다"), GateCalls > 0);
	TestTrue(FString::Printf(TEXT("사유가 '유효 접촉 없음'이어야 한다(collider는 있었다): %s"), *DeniedFailure),
		DeniedFailure.Contains(TEXT("no valid contact")));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
