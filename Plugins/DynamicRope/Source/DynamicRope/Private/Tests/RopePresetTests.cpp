// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopePreset(DataAsset) + URopeComponent::ApplyPreset의 유닛 테스트 — world 없이 NewObject로 검증
// (EnterLoaded/ApplyPreset이 world 비의존인 것은 RopePierceTests의 phase 계약 테스트와 같은 전제).
//
// [테스트 범위의 한계 — RopePierceTests와 동일한 이유] OnPresetApplied(dynamic delegate) 발화와
// Wielder의 RefreshModeDerivedState 연쇄는 여기서 검증할 수 없다: dynamic delegate는 UFUNCTION을
// 가진 UObject 리스너가 필요한데 Private/Tests에 UCLASS가 없고, Wielder 갱신은 world/BeginPlay가
// 전제다. 이벤트/HUD/preview 갱신 검증은 PIE 체크리스트(Rope.Preset.Cycle)가 담당한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Preset/RopePreset.h"
#include "RopeComponent.h"
#include "Core/RopeThrowTypes.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/Actor.h"
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

	static void SeedTransientSimState(URopeComponent& Rope)
	{
		Rope.Sim.SegmentTension = { 100.0f, 200.0f };
		Rope.Sim.TimeAccumulator = 0.5f;
	}

	static int32 GetSegmentTensionNum(const URopeComponent& Rope)
	{
		return Rope.Sim.SegmentTension.Num();
	}

	static float GetTimeAccumulator(const URopeComponent& Rope)
	{
		return Rope.Sim.TimeAccumulator;
	}

	// Loaded tip placement: the private compose helper plus the protected socket virtual it builds on,
	// so the offset contract can be checked without a world.
	static FTransform GetLoadedTipBaseWorld(const URopeComponent& Rope)
	{
		return Rope.MakeLoadedTipBaseWorld();
	}

	static FTransform GetLoadedSocketWorld(const URopeComponent& Rope)
	{
		return Rope.GetLoadedTipTransform();
	}

	// 던지기 세기 계약 게이트(private) — 모든 던지기 경로가 상태 변경 전에 공유 호출하는 단일 지점.
	static bool TryThrowSpeed(const URopeComponent& Rope, const FRopeThrowContext& Ctx, float& Out)
	{
		return Rope.TryResolveValidThrowSpeed(Ctx, Out);
	}

	// StartFreshThrow(private) — 무효 속도 reject 경로가 상태 변경 전에 빠져나오는지(state 보존) 검증용.
	static void CallStartFreshThrow(URopeComponent& Rope, const FRopeThrowContext& Ctx)
	{
		Rope.StartFreshThrow(Ctx);
	}
};

namespace
{
	/** 비기본값이 확실한 ② 프리셋(스탬프 검증용). */
	URopePreset* MakeStampTestPreset()
	{
		URopePreset* Preset = NewObject<URopePreset>();
		Preset->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
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

	TestEqual(TEXT("ResolveMode 기본값 일치"), Preset->ResolveMode, Rope->ResolveMode);
	TestEqual(TEXT("NumParticles 기본값 일치"), Preset->NumParticles, Rope->NumParticles);
	TestEqual(TEXT("RopeLength 기본값 일치"), Preset->RopeLength, Rope->RopeLength);
	TestEqual(TEXT("MinRopeLength 기본값 일치"), Preset->MinRopeLength, Rope->MinRopeLength);
	TestEqual(TEXT("ReelSpeed 기본값 일치"), Preset->ReelSpeed, Rope->ReelSpeed);
	TestEqual(TEXT("Radius 기본값 일치"), Preset->Radius, Rope->Radius);
	TestEqual(TEXT("NumSides 기본값 일치"), Preset->NumSides, Rope->NumSides);
	TestEqual(TEXT("Substeps 기본값 일치"), Preset->SolverConfig.Substeps, Rope->SolverConfig.Substeps);
	TestEqual(TEXT("bUseWorldGDF 기본값 일치"), Preset->bUseWorldGDF, Rope->bUseWorldGDF);
	TestEqual(TEXT("bUseTipMesh 기본값 일치"), Preset->bUseTipMesh, Rope->bUseTipMesh);
	TestEqual(TEXT("LoadedHandSocket 기본값 일치"), Preset->LoadedHandSocket, Rope->LoadedHandSocket);
	TestTrue(TEXT("LoadedTipRelativeTransform 기본값 일치"),
		Preset->LoadedTipRelativeTransform.Equals(Rope->LoadedTipRelativeTransform));
	// 소켓 오버라이드는 기본 꺼짐이어야 기존 프리셋 적용이 인스턴스 배선을 지우지 않는다.
	TestFalse(TEXT("bOverrideLoadedHandSocket 기본 꺼짐"), Preset->bOverrideLoadedHandSocket);
	// 기본 머티리얼도 미러 — 프리셋 None이면 스탬프가 기본 머티리얼을 벗겨 회색 폴백이 된다(생성자 FObjectFinder 동기화).
	TestEqual(TEXT("RopeMaterial 기본값 일치"), Preset->RopeMaterial.Get(), Rope->RopeMaterial.Get());
	return true;
}

#if WITH_EDITOR
// 에셋 저장 검증(IsDataValid): 길이 역전은 경고로 알리되 저장 자체는 막지 않는다 —
// SetRopeLength가 [MinRopeLength, RopeLength]로 클램프하므로 역전은 저작 실수지 무효 데이터가 아니다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetDataValidationTest,
	"DynamicRope.Preset.DataValidationWarnsOnLengthInversion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetDataValidationTest::RunTest(const FString& Parameters)
{
	URopePreset* Preset = NewObject<URopePreset>();

	Preset->RopeLength = 300.0f;
	Preset->MinRopeLength = 500.0f;
	{
		FDataValidationContext Context;
		TestNotEqual(TEXT("길이 역전은 경고지 Invalid가 아니다"),
			Preset->IsDataValid(Context), EDataValidationResult::Invalid);
		TestEqual(TEXT("길이 역전 경고 1건"), Context.GetNumWarnings(), 1);
	}

	Preset->MinRopeLength = 100.0f;
	{
		FDataValidationContext Context;
		TestEqual(TEXT("정상 길이면 경고 없음"), Context.GetNumWarnings(), 0);
		TestNotEqual(TEXT("정상 길이는 Invalid 아님"),
			Preset->IsDataValid(Context), EDataValidationResult::Invalid);
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
	FRopePresetTestSeam::SeedTransientSimState(*Rope);

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

	// InitRope 실행 증명 — Sim 토폴로지/길이가 새 값으로 재시드됐다.
	TestEqual(TEXT("노드 수 재시드"), Rope->GetNodeCount(), 32);
	TestEqual(TEXT("현재 길이 재시드"), Rope->GetCurrentRopeLength(), 555.0f);
	TestEqual(TEXT("재초기화 시 이전 장력 제거"), FRopePresetTestSeam::GetSegmentTensionNum(*Rope), 0);
	TestEqual(TEXT("재초기화 시 fixed-step 누적 시간 제거"), FRopePresetTestSeam::GetTimeAccumulator(*Rope), 0.0f);

	// ② 프리셋은 페이즈를 옮기지 않는다.
	TestEqual(TEXT("Free 유지"), Rope->GetPhase(), ERopePhase::Free);
	return true;
}

// 저장 설정이 곧 런타임 값이다 — 중간 해석 단계가 없다. 정밀도/비용 기본값은 문서화된 값을
// 유지해야 하고(회귀 방어), 프리셋은 그 값을 그대로 스탬프한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeStoredConfigIsRuntimeConfigTest,
	"DynamicRope.Preset.StoredConfigIsRuntimeConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeStoredConfigIsRuntimeConfigTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	// 솔버 정밀도/스윕 기본값.
	TestEqual(TEXT("기본 Substeps=12"), Rope->SolverConfig.Substeps, 12);
	TestEqual(TEXT("기본 Iterations=4"), Rope->SolverConfig.Iterations, 4);
	TestEqual(TEXT("기본 SweepStep=2"), Rope->SolverConfig.SweepStep, 2.0f);
	TestEqual(TEXT("기본 MaxSweepSamples=16"), Rope->SolverConfig.MaxSweepSamples, 16);

	// 접촉 감지 스윕 + 감김 경로 빌드 예산 기본값.
	TestEqual(TEXT("기본 ContactSweepStep=2"), Rope->WrapConfig.ContactSweepStep, 2.0f);
	TestEqual(TEXT("기본 ContactMaxSweepSamples=16"), Rope->WrapConfig.ContactMaxSweepSamples, 16);
	TestEqual(TEXT("기본 WrappingPathBuildSteps=8"), Rope->WrapConfig.WrappingPathBuildStepsPerFrame, 8);

	// 프리셋 스탬프 후에도 저장값이 그대로 남는다(적용 이후 재해석 없음).
	URopePreset* Preset = NewObject<URopePreset>();
	Preset->SolverConfig.Substeps = 9;
	Preset->SolverConfig.Iterations = 5;
	TestTrue(TEXT("프리셋 적용 성공"), Rope->ApplyPreset(Preset));
	TestEqual(TEXT("Substeps 스탬프"), Rope->SolverConfig.Substeps, 9);
	TestEqual(TEXT("Iterations 스탬프"), Rope->SolverConfig.Iterations, 5);

	return true;
}

// 던지기 세기 계약: ThrowSpeed는 양수(cm/s)여야 하고, 0/음수는 ThrowParams로 폴백해 해석한다.
// 무효(≥해석 후 <1) 속도면 상태 변경 전에 거부한다(Phase/transient 보존) — "느린데 갑자기 빠른" 모순 제거.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeThrowInvalidSpeedTest,
	"DynamicRope.Throw.InvalidSpeedPreservesState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeThrowInvalidSpeedTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	// 1) Context 0 & ThrowParams 0 → 거부.
	Rope->ThrowParams.ThrowSpeed = 0.0f;
	FRopeThrowContext Ctx;
	Ctx.ThrowSpeed = 0.0f;
	float Out = -1.0f;
	TestFalse(TEXT("0 속도 거부"), FRopePresetTestSeam::TryThrowSpeed(*Rope, Ctx, Out));

	// 2) Context 0 → ThrowParams(1500) 폴백 → 통과.
	Rope->ThrowParams.ThrowSpeed = 1500.0f;
	TestTrue(TEXT("0 컨텍스트는 ThrowParams로 폴백"), FRopePresetTestSeam::TryThrowSpeed(*Rope, Ctx, Out));
	TestEqual(TEXT("폴백 속도=1500"), Out, 1500.0f);

	// 3) 0<속도<1 → 거부(하한 계약).
	Ctx.ThrowSpeed = 0.5f;
	TestFalse(TEXT("0.5 속도 거부"), FRopePresetTestSeam::TryThrowSpeed(*Rope, Ctx, Out));

	// 4) 양수 속도 → 통과, 값 그대로.
	Ctx.ThrowSpeed = 900.0f;
	TestTrue(TEXT("900 속도 통과"), FRopePresetTestSeam::TryThrowSpeed(*Rope, Ctx, Out));
	TestEqual(TEXT("해석 속도=900"), Out, 900.0f);

	// 5) 상태 보존: StartFreshThrow가 무효 속도면 상태 변경(ResetStateForNewThrow) 전에 거부한다 →
	//    Phase 유지 + 이전 transient(장력) 보존(정상 던지기면 InitRope가 이를 비운다).
	Rope->ThrowParams.ThrowSpeed = 0.0f; // 폴백도 무효가 되도록
	FRopePresetTestSeam::ForcePhase(*Rope, ERopePhase::Free);
	FRopePresetTestSeam::SeedTransientSimState(*Rope);
	FRopeThrowContext ZeroCtx;
	ZeroCtx.ThrowSpeed = 0.0f;
	FRopePresetTestSeam::CallStartFreshThrow(*Rope, ZeroCtx);
	TestEqual(TEXT("거부 시 Phase 불변(Free)"), Rope->GetPhase(), ERopePhase::Free);
	TestEqual(TEXT("거부 시 transient 장력 보존"), FRopePresetTestSeam::GetSegmentTensionNum(*Rope), 2);

	return true;
}

// Taut Sensitivity → 슬랙 비율·최대 처짐의 기하 보간(방향/값 검증). 0=느슨(큰 허용), 1=엄격(작은 허용).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTautSensitivityMappingTest,
	"DynamicRope.Taut.SensitivityMapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTautSensitivityMappingTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	Rope->HoldConfig.TautSensitivity = 0.5f;
	TestEqual(TEXT("0.5 슬랙≈0.03"), Rope->GetEffectiveTautSlackRatio(), 0.03f, 0.002f);
	TestEqual(TEXT("0.5 처짐≈20"), Rope->GetEffectiveTautMaxSag(), 20.0f, 0.5f);

	Rope->HoldConfig.TautSensitivity = 0.0f;
	TestEqual(TEXT("0 슬랙≈0.09"), Rope->GetEffectiveTautSlackRatio(), 0.09f, 0.002f);
	TestEqual(TEXT("0 처짐≈80"), Rope->GetEffectiveTautMaxSag(), 80.0f, 0.5f);

	Rope->HoldConfig.TautSensitivity = 1.0f;
	TestEqual(TEXT("1 슬랙≈0.01"), Rope->GetEffectiveTautSlackRatio(), 0.01f, 0.002f);
	TestEqual(TEXT("1 처짐≈5"), Rope->GetEffectiveTautMaxSag(), 5.0f, 0.5f);

	// 방향: 민감도↑ → 허용↓(엄격).
	Rope->HoldConfig.TautSensitivity = 0.3f;
	const float Slack03 = Rope->GetEffectiveTautSlackRatio();
	Rope->HoldConfig.TautSensitivity = 0.7f;
	const float Slack07 = Rope->GetEffectiveTautSlackRatio();
	TestTrue(TEXT("민감도 높을수록 슬랙 허용 작다"), Slack07 < Slack03);

	return true;
}

// 모드-페이즈 정합: Free+③프리셋 → 즉시 Loaded(장전) 진입 + 던지기 게이트 통과.
// 이어서 Loaded에서 ①프리셋 → 전개 후 Free 복귀(①은 게이트 없음).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetModePhaseReconciliationTest,
	"DynamicRope.Preset.ModePhaseReconciliation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetModePhaseReconciliationTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	URopePreset* Guaranteed = NewObject<URopePreset>();
	Guaranteed->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;

	TestTrue(TEXT("Free에서 ③ 적용 성공"), Rope->ApplyPreset(Guaranteed));
	TestEqual(TEXT("③ 적용 → Loaded 진입"), Rope->GetPhase(), ERopePhase::Loaded);
	TestTrue(TEXT("Loaded에서 던지기 게이트 통과"), Rope->CanThrowNow());

	URopePreset* FreeSim = NewObject<URopePreset>();
	FreeSim->ResolveMode = ERopeWrapResolveMode::FullSimulation;

	TestTrue(TEXT("Loaded에서 ① 적용 성공(Loaded은 허용 페이즈)"), Rope->ApplyPreset(FreeSim));
	TestEqual(TEXT("①로 전환 → Free 복귀"), Rope->GetPhase(), ERopePhase::Free);
	TestTrue(TEXT("①은 페이즈 게이트 없음"), Rope->CanThrowNow());
	return true;
}

// 페이즈 게이트 음성: Free/Loaded 밖(Wrapped)에서는 false를 반환하고 **아무 필드도 바꾸지 않는다**
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

// 손 소켓은 소유 스켈레톤에 결합된 인스턴스 배선이라, 옵트인한 프리셋만 덮어쓴다.
// 기본(꺼짐) 프리셋은 소켓을 비워 둔 채로 적용되므로 배선을 지우면 안 된다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetLoadedHandSocketOverrideGateTest,
	"DynamicRope.Preset.LoadedHandSocketOverrideGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetLoadedHandSocketOverrideGateTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->LoadedHandSocket = FName(TEXT("hand_r"));

	// 게이트 꺼짐(기본) — 소켓이 채워져 있어도 인스턴스 값을 보존한다.
	URopePreset* KeepPreset = NewObject<URopePreset>();
	KeepPreset->bUseTipMesh = true;
	KeepPreset->LoadedHandSocket = FName(TEXT("hand_l"));
	TestTrue(TEXT("게이트 꺼짐 프리셋 적용 성공"), Rope->ApplyPreset(KeepPreset));
	TestEqual(TEXT("옵트인 전에는 인스턴스 소켓 보존"), Rope->LoadedHandSocket, FName(TEXT("hand_r")));

	// 게이트 켬 — 프리셋 소켓으로 교체된다.
	URopePreset* OverridePreset = NewObject<URopePreset>();
	OverridePreset->bUseTipMesh = true;
	OverridePreset->bOverrideLoadedHandSocket = true;
	OverridePreset->LoadedHandSocket = FName(TEXT("hand_l"));
	TestTrue(TEXT("게이트 켬 프리셋 적용 성공"), Rope->ApplyPreset(OverridePreset));
	TestEqual(TEXT("옵트인 시 소켓 교체"), Rope->LoadedHandSocket, FName(TEXT("hand_l")));

	// 옵트인 프리셋은 빈 소켓도 그대로 스탬프한다(명시적 초기화 경로).
	URopePreset* ClearPreset = NewObject<URopePreset>();
	ClearPreset->bUseTipMesh = true;
	ClearPreset->bOverrideLoadedHandSocket = true;
	TestTrue(TEXT("빈 소켓 옵트인 프리셋 적용 성공"), Rope->ApplyPreset(ClearPreset));
	TestEqual(TEXT("옵트인 빈 소켓은 초기화"), Rope->LoadedHandSocket, NAME_None);
	return true;
}

// Loaded 배치 오프셋은 무조건 스탬프되고(Identity=현행 동작), 소켓 프레임 기준으로 합성된다.
// 팀 별 소비처(팁 메쉬 배치 / 마지막 노드 핀)가 이 하나를 공유하므로 합성식 자체를 못 박아 둔다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetLoadedTipOffsetStampedTest,
	"DynamicRope.Preset.LoadedTipOffsetStamped",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetLoadedTipOffsetStampedTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	// The socket fallback is the component transform, and an identity socket would make Offset*Socket
	// and Socket*Offset agree - which is exactly the mistake this test exists to catch. Push the
	// component somewhere non-trivial first, and fail loudly if that did not take.
	Rope->SetRelativeTransform(FTransform(FQuat(FVector(1, 0, 0), HALF_PI), FVector(-40.0f, 7.0f, 100.0f)));
	Rope->UpdateComponentToWorld();
	TestFalse(TEXT("소켓 폴백이 비-Identity여야 순서 검증이 성립"),
		FRopePresetTestSeam::GetLoadedSocketWorld(*Rope).Equals(FTransform::Identity));

	// 기본 Identity 오프셋에서는 합성 결과가 소켓 트랜스폼 그대로여야 한다(회귀 방어).
	TestTrue(TEXT("Identity면 소켓 트랜스폼 그대로"),
		FRopePresetTestSeam::GetLoadedTipBaseWorld(*Rope).Equals(
			FRopePresetTestSeam::GetLoadedSocketWorld(*Rope)));

	const FTransform Offset(FQuat(FVector(0, 0, 1), HALF_PI), FVector(12.0f, -3.0f, 5.0f));
	URopePreset* Preset = NewObject<URopePreset>();
	Preset->bUseTipMesh = true;
	Preset->LoadedTipRelativeTransform = Offset;
	TestTrue(TEXT("프리셋 적용 성공"), Rope->ApplyPreset(Preset));
	TestTrue(TEXT("Loaded 오프셋 스탬프"), Rope->LoadedTipRelativeTransform.Equals(Offset));

	// 소켓 로컬 프레임 기준 = Offset * SocketWorld (Offset이 먼저 적용된다).
	const FTransform SocketWorld = FRopePresetTestSeam::GetLoadedSocketWorld(*Rope);
	TestTrue(TEXT("소켓 프레임 기준으로 합성"),
		FRopePresetTestSeam::GetLoadedTipBaseWorld(*Rope).Equals(Offset * SocketWorld));
	// 반대 순서로는 성립하지 않아야 한다 — 합성 순서가 실제로 계약임을 못 박는다.
	TestFalse(TEXT("반대 합성 순서는 불일치"),
		FRopePresetTestSeam::GetLoadedTipBaseWorld(*Rope).Equals(SocketWorld * Offset));
	return true;
}

// 태그 재사용 팁 × 프리셋 전환: 해제(Teardown) 시 저작 상대 트랜스폼이 복원돼, 직전 프리셋의
// 배치 스케일(0.2)이 다음 획득의 저작 기준선으로 오염되지 않는다(스케일 누적 버그 회귀 방어).
// 매 프레임 배치(FinalizeSimFrame)는 테스트에서 안 돌므로, 배치가 덮어쓴 상태를 직접 기록해 재현한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetTagTipTransformRestoreTest,
	"DynamicRope.Preset.TagTipTransformRestoredAcrossPresets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetTagTipTransformRestoreTest::RunTest(const FString& Parameters)
{
	AActor* Owner = NewObject<AActor>();
	UStaticMeshComponent* TagTip = NewObject<UStaticMeshComponent>(Owner);
	TagTip->ComponentTags.Add(FName(TEXT("RopeTipTagTest")));
	TagTip->SetRelativeScale3D(FVector(2.0f));		// 저작 스케일

	URopeComponent* Rope = NewObject<URopeComponent>(Owner);
	Rope->TipMeshComponentTag = FName(TEXT("RopeTipTagTest"));

	// 프리셋 1: 팁 사용 + 배치 스케일 0.2.
	URopePreset* SmallTipPreset = NewObject<URopePreset>();
	SmallTipPreset->bUseTipMesh = true;
	SmallTipPreset->TipMeshRelativeTransform.SetScale3D(FVector(0.2f));
	TestTrue(TEXT("프리셋 1 적용"), Rope->ApplyPreset(SmallTipPreset));
	TestEqual(TEXT("태그 컴포넌트 획득"), Rope->GetTipMeshComponent(), TagTip);

	// 프레임 배치가 월드 스케일을 저작(2)×프리셋(0.2)로 덮은 상태 재현.
	TagTip->SetWorldScale3D(FVector(2.0f * 0.2f));

	// 프리셋 2: 팁 사용 + 기본 Transform(스케일 1). Teardown 복원 → 재획득 기준선이 저작값이어야 한다.
	URopePreset* DefaultTipPreset = NewObject<URopePreset>();
	DefaultTipPreset->bUseTipMesh = true;
	TestTrue(TEXT("프리셋 2 적용"), Rope->ApplyPreset(DefaultTipPreset));
	TestEqual(TEXT("재획득 유지"), Rope->GetTipMeshComponent(), TagTip);
	TestEqual(TEXT("저작 스케일 복원(0.2 오염 없음)"), TagTip->GetRelativeScale3D(), FVector(2.0f));
	return true;
}

// 태그 재사용 팁 × 팁 끔 프리셋: 외부 컴포넌트는 파괴되지 않고(소유 아님), 참조만 해제되며
// 저작 트랜스폼으로 복원된다. 로프의 팁 포인터는 null.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetTagTipSurvivesTipOffTest,
	"DynamicRope.Preset.TagTipSurvivesTipOffPreset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetTagTipSurvivesTipOffTest::RunTest(const FString& Parameters)
{
	AActor* Owner = NewObject<AActor>();
	UStaticMeshComponent* TagTip = NewObject<UStaticMeshComponent>(Owner);
	TagTip->ComponentTags.Add(FName(TEXT("RopeTipTagTest2")));
	TagTip->SetRelativeScale3D(FVector(3.0f));

	URopeComponent* Rope = NewObject<URopeComponent>(Owner);
	Rope->TipMeshComponentTag = FName(TEXT("RopeTipTagTest2"));

	URopePreset* TipOnPreset = NewObject<URopePreset>();
	TipOnPreset->bUseTipMesh = true;
	TipOnPreset->TipMeshRelativeTransform.SetScale3D(FVector(0.5f));
	TestTrue(TEXT("팁 켬 프리셋 적용"), Rope->ApplyPreset(TipOnPreset));
	TagTip->SetWorldScale3D(FVector(3.0f * 0.5f));	// 배치 덮어쓰기 재현

	URopePreset* TipOffPreset = NewObject<URopePreset>();	// 기본값 = bUseTipMesh false
	TestTrue(TEXT("팁 끔 프리셋 적용"), Rope->ApplyPreset(TipOffPreset));
	TestNull(TEXT("로프 팁 참조 해제"), Rope->GetTipMeshComponent());
	TestTrue(TEXT("외부 컴포넌트 생존"), IsValid(TagTip));
	TestEqual(TEXT("저작 트랜스폼 복원"), TagTip->GetRelativeScale3D(), FVector(3.0f));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
