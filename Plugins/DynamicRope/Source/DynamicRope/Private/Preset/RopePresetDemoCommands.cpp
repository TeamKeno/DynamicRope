// Copyright Epic Games, Inc. All Rights Reserved.
//
// 데모/기능 테스트용 프리셋 전환 콘솔 명령(비Shipping 전용) — 2026-07-18 회의 11번 안건의
// "런타임 모드 전환"을 프리셋 순환 하나로 해소한다. 목록 소스는 프로젝트 설정
// UDynamicRopeSettings::DemoPresets. 정식 게임 코드는 URopeComponent::ApplyPreset을 직접 호출한다.

#include "CoreMinimal.h"

#if !UE_BUILD_SHIPPING

#include "DynamicRopeLog.h"
#include "HAL/IConsoleManager.h"
#include "Preset/RopePreset.h"
#include "RopeComponent.h"
#include "Settings/DynamicRopeSettings.h"
#include "UObject/UObjectIterator.h"

namespace RopePresetConsole
{
	// 순환 상태 — 마지막으로 적용한 DemoPresets 인덱스(-1 = 아직 없음). Apply가 이름/인덱스로
	// 적용해도 동기화해, 이어지는 Cycle이 그 다음 항목부터 돈다.
	static int32 GLastAppliedIndex = -1;

	/** 월드 내 모든 로프에 Fn 적용(템플릿/파괴 중 제외). Rope.Ragdoll의 ForEach 관례 미러. */
	static void ForEachRope(UWorld* World, TFunctionRef<void(URopeComponent&)> Fn)
	{
		int32 Count = 0;
		for (TObjectIterator<URopeComponent> It; It; ++It)
		{
			URopeComponent* Rope = *It;
			if (IsValid(Rope) && !Rope->IsTemplate() && Rope->GetWorld() == World)
			{
				Fn(*Rope);
				++Count;
			}
		}
		if (Count == 0)
		{
			UE_LOG(LogDynamicRope, Warning, TEXT("월드에 URopeComponent가 없다."));
		}
	}

	/** DemoPresets[Index]를 동기 로드한다. 실패 시 null + 경고. */
	static URopePreset* LoadDemoPreset(int32 Index)
	{
		const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
		if (!Settings->DemoPresets.IsValidIndex(Index))
		{
			return nullptr;
		}
		// 데모 전용 명령이라 동기 로드 히치를 수용한다(첫 적용 1회).
		URopePreset* Preset = Settings->DemoPresets[Index].LoadSynchronous();
		if (!Preset)
		{
			UE_LOG(LogDynamicRope, Warning, TEXT("DemoPresets[%d] 로드 실패: %s"),
				Index, *Settings->DemoPresets[Index].ToString());
		}
		return Preset;
	}

	/** 프리셋을 월드의 모든 로프에 적용하고 인덱스를 동기화한다. 거부(페이즈)는 ApplyPreset이 로그로 남긴다. */
	static void ApplyToWorld(UWorld* World, URopePreset* Preset, int32 Index)
	{
		if (!Preset)
		{
			return;
		}
		GLastAppliedIndex = Index;
		UE_LOG(LogDynamicRope, Log, TEXT("Rope.Preset: '%s' (DemoPresets[%d]) 적용 시작."), *Preset->GetName(), Index);
		ForEachRope(World, [Preset](URopeComponent& Rope)
		{
			Rope.ApplyPreset(Preset);
		});
	}

	/** 설정 배열이 비었으면 안내 경고 후 false. */
	static bool EnsureDemoPresetList()
	{
		if (UDynamicRopeSettings::Get()->DemoPresets.Num() == 0)
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("DemoPresets가 비어 있다 — Project Settings > Plugins > Dynamic Rope > Demo에 URopePreset 에셋을 등록할 것."));
			return false;
		}
		return true;
	}

	static FAutoConsoleCommandWithWorldAndArgs GCycleCmd(
		TEXT("Rope.Preset.Cycle"),
		TEXT("DemoPresets(프로젝트 설정)의 다음 프리셋을 월드 내 모든 로프에 적용한다. Free/Reel이 아닌 로프는 거부 로그."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			if (!EnsureDemoPresetList())
			{
				return;
			}
			const int32 Num = UDynamicRopeSettings::Get()->DemoPresets.Num();
			const int32 NextIndex = (GLastAppliedIndex + 1) % Num;
			ApplyToWorld(World, LoadDemoPreset(NextIndex), NextIndex);
		}));

	static FAutoConsoleCommandWithWorldAndArgs GApplyCmd(
		TEXT("Rope.Preset.Apply"),
		TEXT("지정 프리셋 적용: 인자 = DemoPresets 인덱스(숫자) 또는 에셋 이름 부분일치. 순환 인덱스도 동기화된다."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			if (Args.Num() == 0)
			{
				UE_LOG(LogDynamicRope, Warning, TEXT("사용법: Rope.Preset.Apply <인덱스|이름부분일치> — 목록은 Rope.Preset.List."));
				return;
			}
			if (!EnsureDemoPresetList())
			{
				return;
			}

			const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
			int32 Index = INDEX_NONE;
			if (Args[0].IsNumeric())
			{
				Index = FCString::Atoi(*Args[0]);
			}
			else
			{
				// 이름 부분일치(대소문자 무시) — soft ref의 에셋 이름으로 검색(로드 없이).
				for (int32 i = 0; i < Settings->DemoPresets.Num(); ++i)
				{
					if (Settings->DemoPresets[i].GetAssetName().Contains(Args[0]))
					{
						Index = i;
						break;
					}
				}
			}

			if (!Settings->DemoPresets.IsValidIndex(Index))
			{
				UE_LOG(LogDynamicRope, Warning, TEXT("'%s'에 해당하는 DemoPresets 항목이 없다(0..%d) — Rope.Preset.List로 확인."),
					*Args[0], Settings->DemoPresets.Num() - 1);
				return;
			}
			ApplyToWorld(World, LoadDemoPreset(Index), Index);
		}));

	static FAutoConsoleCommandWithWorldAndArgs GListCmd(
		TEXT("Rope.Preset.List"),
		TEXT("DemoPresets 목록과 현재 순환 인덱스를 출력한다."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
			UE_LOG(LogDynamicRope, Log, TEXT("DemoPresets %d개 (현재 인덱스=%d):"),
				Settings->DemoPresets.Num(), GLastAppliedIndex);
			for (int32 i = 0; i < Settings->DemoPresets.Num(); ++i)
			{
				// 로드된 에셋이면 모드까지, 아니면 이름만(목록 조회로 로드를 유발하지 않는다).
				const URopePreset* Loaded = Settings->DemoPresets[i].Get();
				if (Loaded)
				{
					UE_LOG(LogDynamicRope, Log, TEXT("  [%d]%s %s (mode=%s)"),
						i, (i == GLastAppliedIndex ? TEXT("*") : TEXT(" ")), *Loaded->GetName(),
						*StaticEnum<ERopeWrapResolveMode>()->GetNameStringByValue((int64)Loaded->ResolveMode));
				}
				else
				{
					UE_LOG(LogDynamicRope, Log, TEXT("  [%d]%s %s (미로드)"),
						i, (i == GLastAppliedIndex ? TEXT("*") : TEXT(" ")), *Settings->DemoPresets[i].GetAssetName());
				}
			}
		}));
}

#endif // !UE_BUILD_SHIPPING
