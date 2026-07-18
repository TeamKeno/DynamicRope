// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopePreset(DataAsset) + URopeComponent::ApplyPreset의 유닛 테스트 — world 없이 NewObject로 검증
// (EnterReel/ApplyPreset이 world 비의존인 것은 RopePierceTests의 phase 계약 테스트와 같은 전제).
//
// [테스트 범위의 한계 — RopePierceTests와 동일한 이유] OnPresetApplied(dynamic delegate) 발화와
// Wielder의 RefreshModeDerivedState 연쇄는 여기서 검증할 수 없다: dynamic delegate는 UFUNCTION을
// 가진 UObject 리스너가 필요한데 Private/Tests에 UCLASS가 없고, Wielder 갱신은 world/BeginPlay가
// 전제다. 이벤트/HUD/preview 갱신 검증은 PIE 체크리스트(Rope.Preset.Cycle)가 담당한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Preset/RopePreset.h"
#include "RopeComponent.h"
#if WITH_EDITOR
#include "Misc/DataValidation.h"
#endif

// RopeComponent.h의 friend 시임 — ApplyPreset 페이즈 게이트 음성 테스트가 금지 페이즈를 강제하는
// 유일한 통로(공개 API로는 world 없이 Wrapped를 만들 수 없다).
struct FRopePresetTestSeam
{
	static void ForcePhase(URopeComponent& Rope, ERopePhase Phase)
	{
		Rope.Phase = Phase;
	}
};

namespace
{
	/** 비기본값이 확실한 ② 프리셋(스탬프 검증용). */
	URopePreset* MakeStampTestPreset()
	{
		URopePreset* Preset = NewObject<URopePreset>();
		Preset->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
		Preset->TipEngagement = ERopeTipEngagement::BareWrap;
		Preset->NumParticles = 32;
		Preset->RopeLength = 555.0f;
		Preset->MinRopeLength = 111.0f;
		Preset->ReelSpeed = 321.0f;
		Preset->SolverConfig.Substeps = 7;
		Preset->SolverConfig.Iterations = 3;
		Preset->Radius = 3.25f;
		Preset->NumSides = 12;
		Preset->bIncludeOwnerColliders = true;
		Preset->bUseWorldGDF = false;
		Preset->PreviewReachScale = 1.5f;
		return Preset;
	}
}

// 기본값 프리셋 = 기본 로프: 조합이 유효하고, 핵심 기본값이 컴포넌트 CDO와 일치해야
// "빈 프리셋 적용 = 회귀 없음" 계약이 성립한다(프리셋 필드는 컴포넌트 미러 — 헤더 주석).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetDefaultsValidTest,
	"DynamicRope.Preset.DefaultsValid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetDefaultsValidTest::RunTest(const FString& Parameters)
{
	const URopePreset* Preset = GetDefault<URopePreset>();
	const URopeComponent* Rope = GetDefault<URopeComponent>();

	TestTrue(TEXT("기본 모드 조합 유효"), RopeWrapModes::IsEngagementAllowed(Preset->ResolveMode, Preset->TipEngagement));
	TestEqual(TEXT("ResolveMode 기본값 일치"), Preset->ResolveMode, Rope->ResolveMode);
	TestEqual(TEXT("TipEngagement 기본값 일치"), Preset->TipEngagement, Rope->TipEngagement);
	TestEqual(TEXT("NumParticles 기본값 일치"), Preset->NumParticles, Rope->NumParticles);
	TestEqual(TEXT("RopeLength 기본값 일치"), Preset->RopeLength, Rope->RopeLength);
	TestEqual(TEXT("MinRopeLength 기본값 일치"), Preset->MinRopeLength, Rope->MinRopeLength);
	TestEqual(TEXT("ReelSpeed 기본값 일치"), Preset->ReelSpeed, Rope->ReelSpeed);
	TestEqual(TEXT("Radius 기본값 일치"), Preset->Radius, Rope->Radius);
	TestEqual(TEXT("NumSides 기본값 일치"), Preset->NumSides, Rope->NumSides);
	TestEqual(TEXT("Substeps 기본값 일치"), Preset->SolverConfig.Substeps, Rope->SolverConfig.Substeps);
	TestEqual(TEXT("bUseWorldGDF 기본값 일치"), Preset->bUseWorldGDF, Rope->bUseWorldGDF);
	TestEqual(TEXT("bUseTipMesh 기본값 일치"), Preset->bUseTipMesh, Rope->bUseTipMesh);
	// 기본 머티리얼도 미러 — 프리셋 None이면 스탬프가 기본 머티리얼을 벗겨 회색 폴백이 된다(생성자 FObjectFinder 동기화).
	TestEqual(TEXT("RopeMaterial 기본값 일치"), Preset->RopeMaterial.Get(), Rope->RopeMaterial.Get());
	return true;
}

#if WITH_EDITOR
// 에셋 저장 검증(IsDataValid): 무효 모드 조합은 Invalid, 유효 조합은 통과. Cinch는 경고만(저장 허용).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetDataValidationTest,
	"DynamicRope.Preset.DataValidationRejectsInvalidCombo",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetDataValidationTest::RunTest(const FString& Parameters)
{
	URopePreset* Preset = NewObject<URopePreset>();

	// ③ + BareWrap = 무효.
	Preset->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Preset->TipEngagement = ERopeTipEngagement::BareWrap;
	{
		FDataValidationContext Context;
		TestEqual(TEXT("③+BareWrap → Invalid"), Preset->IsDataValid(Context), EDataValidationResult::Invalid);
	}

	// ① + Pierce = 무효.
	Preset->ResolveMode = ERopeWrapResolveMode::FullSimulation;
	Preset->TipEngagement = ERopeTipEngagement::Pierce;
	{
		FDataValidationContext Context;
		TestEqual(TEXT("①+Pierce → Invalid"), Preset->IsDataValid(Context), EDataValidationResult::Invalid);
	}

	// ③ + Pierce = 유효.
	Preset->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Preset->TipEngagement = ERopeTipEngagement::Pierce;
	{
		FDataValidationContext Context;
		TestNotEqual(TEXT("③+Pierce → Invalid 아님"), Preset->IsDataValid(Context), EDataValidationResult::Invalid);
	}

	// ② + BareWrap = 유효.
	Preset->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	Preset->TipEngagement = ERopeTipEngagement::BareWrap;
	{
		FDataValidationContext Context;
		TestNotEqual(TEXT("②+BareWrap → Invalid 아님"), Preset->IsDataValid(Context), EDataValidationResult::Invalid);
	}
	return true;
}
#endif // WITH_EDITOR

// 값 스탬프 + 재초기화: 적용 성공 시 필드가 복사되고 InitRope가 실제로 돌아
// 노드 수/길이가 Sim에 반영된다. Free 로프의 ②적용은 Free 유지.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetApplyStampsValuesTest,
	"DynamicRope.Preset.ApplyStampsValues",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetApplyStampsValuesTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopePreset* Preset = MakeStampTestPreset();

	TestTrue(TEXT("Free에서 적용 성공"), Rope->ApplyPreset(Preset));

	TestEqual(TEXT("ResolveMode 스탬프"), Rope->ResolveMode, ERopeWrapResolveMode::AssistedJudged);
	TestEqual(TEXT("NumParticles 스탬프"), Rope->NumParticles, 32);
	TestEqual(TEXT("RopeLength 스탬프"), Rope->RopeLength, 555.0f);
	TestEqual(TEXT("MinRopeLength 스탬프"), Rope->MinRopeLength, 111.0f);
	TestEqual(TEXT("ReelSpeed 스탬프"), Rope->ReelSpeed, 321.0f);
	TestEqual(TEXT("SolverConfig 구조체 스탬프"), Rope->SolverConfig.Substeps, 7);
	TestEqual(TEXT("Radius 스탬프"), Rope->Radius, 3.25f);
	TestEqual(TEXT("NumSides 스탬프"), Rope->NumSides, 12);
	TestEqual(TEXT("bIncludeOwnerColliders 스탬프"), Rope->bIncludeOwnerColliders, true);
	TestEqual(TEXT("bUseWorldGDF 스탬프"), Rope->bUseWorldGDF, false);
	TestEqual(TEXT("PreviewReachScale 스탬프"), Rope->PreviewReachScale, 1.5f);

	// InitRope 실행 증명 — Sim 토폴로지/길이가 새 값으로 재시드됐다.
	TestEqual(TEXT("노드 수 재시드"), Rope->GetNodeCount(), 32);
	TestEqual(TEXT("현재 길이 재시드"), Rope->GetCurrentRopeLength(), 555.0f);

	// ② 프리셋은 페이즈를 옮기지 않는다.
	TestEqual(TEXT("Free 유지"), Rope->GetPhase(), ERopePhase::Free);
	return true;
}

// 무효 조합 심층 방어: 코드로 조작한 ③+BareWrap 프리셋(IsDataValid를 우회한 케이스)을 적용하면
// ClampEngagement 규칙(③→Pierce)으로 보정돼 들어간다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetApplyClampsEngagementTest,
	"DynamicRope.Preset.ApplyClampsEngagement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetApplyClampsEngagementTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopePreset* Preset = NewObject<URopePreset>();
	Preset->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Preset->TipEngagement = ERopeTipEngagement::BareWrap;

	TestTrue(TEXT("적용 성공(보정 포함)"), Rope->ApplyPreset(Preset));
	TestEqual(TEXT("③+BareWrap → Pierce 보정"), Rope->TipEngagement, ERopeTipEngagement::Pierce);
	return true;
}

// 모드-페이즈 정합: Free+③프리셋 → 즉시 Reel(장전) 진입 + 던지기 게이트 통과.
// 이어서 Reel에서 ①프리셋 → 전개 후 Free 복귀(①은 게이트 없음).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetModePhaseReconciliationTest,
	"DynamicRope.Preset.ModePhaseReconciliation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetModePhaseReconciliationTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	URopePreset* Guaranteed = NewObject<URopePreset>();
	Guaranteed->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Guaranteed->TipEngagement = ERopeTipEngagement::Pierce;

	TestTrue(TEXT("Free에서 ③ 적용 성공"), Rope->ApplyPreset(Guaranteed));
	TestEqual(TEXT("③ 적용 → Reel 진입"), Rope->GetPhase(), ERopePhase::Reel);
	TestTrue(TEXT("Reel에서 던지기 게이트 통과"), Rope->CanThrowNow());

	URopePreset* FreeSim = NewObject<URopePreset>();
	FreeSim->ResolveMode = ERopeWrapResolveMode::FullSimulation;
	FreeSim->TipEngagement = ERopeTipEngagement::BareWrap;

	TestTrue(TEXT("Reel에서 ① 적용 성공(Reel은 허용 페이즈)"), Rope->ApplyPreset(FreeSim));
	TestEqual(TEXT("①로 전환 → Free 복귀"), Rope->GetPhase(), ERopePhase::Free);
	TestTrue(TEXT("①은 페이즈 게이트 없음"), Rope->CanThrowNow());
	return true;
}

// 페이즈 게이트 음성: Free/Reel 밖(Wrapped)에서는 false를 반환하고 **아무 필드도 바꾸지 않는다**
// (게이트가 스탬프보다 앞이라는 순서 계약의 증명).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetRejectsOutsideFreeReelTest,
	"DynamicRope.Preset.RejectsOutsideFreeReel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetRejectsOutsideFreeReelTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	const int32 OldNumParticles = Rope->NumParticles;
	const float OldRopeLength = Rope->RopeLength;
	const ERopeWrapResolveMode OldMode = Rope->ResolveMode;

	FRopePresetTestSeam::ForcePhase(*Rope, ERopePhase::Wrapped);

	URopePreset* Preset = MakeStampTestPreset();
	TestFalse(TEXT("Wrapped에서 적용 거부"), Rope->ApplyPreset(Preset));
	TestEqual(TEXT("거부 시 NumParticles 미변경"), Rope->NumParticles, OldNumParticles);
	TestEqual(TEXT("거부 시 RopeLength 미변경"), Rope->RopeLength, OldRopeLength);
	TestEqual(TEXT("거부 시 ResolveMode 미변경"), Rope->ResolveMode, OldMode);
	TestEqual(TEXT("거부 시 페이즈 유지"), Rope->GetPhase(), ERopePhase::Wrapped);

	// null 프리셋도 거부.
	TestFalse(TEXT("null 프리셋 거부"), Rope->ApplyPreset(nullptr));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
