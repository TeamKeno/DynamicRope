// Copyright Epic Games, Inc. All Rights Reserved.
//
// Pierce(꽂힘) 결착 모드 단위 테스트 — ③ GuaranteedWrap 전용, aim-hit 접점에 단일 앵커로 성립.
// 여섯 계약을 잠근다: ①②③↔결착 조합 제약, throw phase 게이트(③=Loaded 전용), preview 빌더의 단일 앵커
// 산출, preview 대상의 출처(aim hit 필수), 단일 앵커 커밋(BeginWrap),
// ③ 연출 진입/이탈(Reel→GuidedThrow→Releasing).

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

// throw phase 게이트: ③는 Loaded에서만 던질 수 있고, ①②는 phase 게이트가 없다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceThrowPhaseGateTest,
	"DynamicRope.Pierce.ThrowPhaseGateContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceThrowPhaseGateTest::RunTest(const FString& Parameters)
{
	using namespace RopeWrapModes;

	// ERopePhase에는 Count/MAX 센티넬이 없다(Reel로 끝남) — 새 phase가 추가되면 여기에 손으로 더해야 한다.
	static const ERopePhase AllPhases[] = {
		ERopePhase::Free, ERopePhase::Flight, ERopePhase::Contacting, ERopePhase::Wrapping,
		ERopePhase::Wrapped, ERopePhase::Releasing, ERopePhase::GuidedThrow, ERopePhase::Loaded
	};

	// ③ GuaranteedWrap = Loaded 전용.
	TestTrue(TEXT("③+Loaded 던지기 성립"),
		CanThrowInPhase(ERopeWrapResolveMode::GuaranteedWrap, ERopePhase::Loaded));
	for (const ERopePhase Phase : AllPhases)
	{
		if (Phase == ERopePhase::Loaded)
		{
			continue;
		}
		TestFalse(*FString::Printf(TEXT("③+%d 던지기 불가(Loaded 아님)"), static_cast<int32>(Phase)),
			CanThrowInPhase(ERopeWrapResolveMode::GuaranteedWrap, Phase));
	}

	// ①② = phase 게이트 없음 → 모든 phase에서 true.
	// 이 술어를 `Phase == Loaded`로 "단순화"하면 여기서 터진다 — ①②가 조용히 막히는 회귀 방지선이다.
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
	Input.RopeRadius = 2.0f;
	Input.ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
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
			Prepared.Anchors[0].NodeIndex, Sim.Num() - 1);
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

// Loaded(장전) 전이: ③ 로프는 Free에서 EnterLoaded() → Loaded. 비-③는 no-op. 던지기는 Loaded에서만(허용 조건).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceEnterLoadedTest,
	"DynamicRope.Pierce.EnterLoadedTransition",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceEnterLoadedTest::RunTest(const FString& Parameters)
{
	// ③ 로프: 기본 Free → EnterLoaded → Loaded.
	URopeComponent* Guaranteed = NewObject<URopeComponent>();
	Guaranteed->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	TestEqual(TEXT("기본 phase는 Free"), Guaranteed->GetPhase(), ERopePhase::Free);
	Guaranteed->EnterLoaded();
	TestEqual(TEXT("③ EnterLoaded → Loaded"), Guaranteed->GetPhase(), ERopePhase::Loaded);

	// 비-③(② Assisted): EnterLoaded은 no-op → Free 유지(Loaded은 ③ 전용).
	URopeComponent* Assisted = NewObject<URopeComponent>();
	Assisted->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	Assisted->EnterLoaded();
	TestEqual(TEXT("② EnterLoaded은 no-op"), Assisted->GetPhase(), ERopePhase::Free);

	// Loaded 허용 조건: Loaded에서 다시 EnterLoaded은 Loaded 유지(재진입 허용), 그 외 phase에선 무효.
	Guaranteed->EnterLoaded();
	TestEqual(TEXT("Loaded에서 재진입해도 Loaded"), Guaranteed->GetPhase(), ERopePhase::Loaded);
	return true;
}

// ③ 연출 진입/이탈 phase 계약: Loaded → (prepared throw) → GuidedThrow → (수동 해제) → Releasing.
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
	Rope->RopeLength = 140.0f;
	Rope->NumParticles = 8;

	// 조준 성공 상황의 prepared를 world 없이 조립한다(PreviewYieldsSingleAnchor와 같은 경로).
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();

	FRopeThrowPreviewBuilder::FInput Input;
	Input.Sim = &Sim;
	Input.RopeRadius = 2.0f;
	Input.ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
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

	// Loaded 밖에서는 던질 수 없다(CanThrowInPhase 계약의 실제 경로 확인).
	TestFalse(TEXT("Free에서는 prepared throw 거부"), Rope->ThrowWithPreparedPreview(Prepared));
	TestEqual(TEXT("거부 후 phase 불변"), Rope->GetPhase(), ERopePhase::Free);

	Rope->EnterLoaded();
	TestTrue(TEXT("Loaded에서 prepared throw 성립"), Rope->ThrowWithPreparedPreview(Prepared));
	TestEqual(TEXT("prepared throw → GuidedThrow"), Rope->GetPhase(), ERopePhase::GuidedThrow);

	// 연출 중 수동 해제 → Releasing(커밋 전이므로 중앙 신호는 안 나가야 하지만 world 없이는 관측 불가).
	Rope->ReleaseWrap();
	TestEqual(TEXT("GuidedThrow 중 ReleaseWrap → Releasing"), Rope->GetPhase(), ERopePhase::Releasing);
	return true;
}
// preview 대상은 **조준 결과(aim hit)** 하나로만 정해진다. aim hit이 없을 때 던지기 방향 주변을 훑어
// 대상을 주우면 조준과 무관한 옆 대상에 꽂히므로, 대상이 바로 옆에 있어도 preview는 성립하지 않는다.
// 조준 ray가 빗나간 경우와 조준 흐름 자체가 없는 BP 직행/AI 경우 모두 같다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePierceAimHitRequiredForPreviewTest,
	"DynamicRope.Pierce.AimHitRequiredForPreview",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePierceAimHitRequiredForPreviewTest::RunTest(const FString& Parameters)
{
	FRopeSimState Sim = RopeTest::MakeStraightRope(8, 140.0f);
	USkeletalMeshComponent* Mesh = MakePierceMockMesh();
	// aim 축(+X)에서 비켜난 곳(원점에서 +Z로 70 = 로프 길이 140의 절반)에 놓인 대상 —
	// 던지기 방향 주변을 훑는 구현이라면 반드시 주울 위치다.
	FCapsuleCollider Target(FVector(0, -20, 70), FVector(0, 20, 70), 25.0f, FName("spine"), Mesh);
	TArray<IRopeCollider*> Colliders = { &Target };

	for (const bool bAimRayEvaluated : { true, false })
	{
		FRopeThrowPreviewBuilder::FInput Input;
		Input.Sim = &Sim;
		Input.RopeRadius = 2.0f;
		Input.Colliders = &Colliders;
		Input.ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
		Input.ThrowContext.Origin = FVector::ZeroVector;
		Input.ThrowContext.FrameForward = FVector(1, 0, 0);
		Input.ThrowContext.FrameUp = FVector(0, 0, 1);
		Input.ThrowContext.bAimRayEvaluated = bAimRayEvaluated; // true=조준 빗나감, false=조준 흐름 없음
		// bHasAimGuideHit = false(기본) — 두 갈래 공통 전제.

		FRopePreparedThrowPreview Prepared;
		FString Failure;
		TestFalse(*FString::Printf(TEXT("aim hit 없으면 옆 대상이 있어도 preview 실패 (aimRayEvaluated=%d)"),
			bAimRayEvaluated ? 1 : 0),
			FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, Prepared, &Failure));
		TestFalse(TEXT("prepared 무효"), Prepared.IsValid());
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
