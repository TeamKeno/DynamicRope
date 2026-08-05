// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Console commands for switching presets in demos and feature tests, in non-shipping builds alone, which reduce
// switching modes at runtime to cycling through presets. The list comes from the project setting
// UDynamicRopeSettings::DemoPresets. Real game code calls URopeComponent::ApplyPreset directly.

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
	// The cycling state, being the index into DemoPresets that was last applied, or minus one for none yet. Applying
	// by name or index synchronizes it too, so a subsequent cycle continues from the next entry.
	static int32 GLastAppliedIndex = -1;

	/** Applies the function to every rope in the world, excluding templates and those being destroyed. Mirrors the ForEach convention of the Rope.Ragdoll commands. */
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
			UE_LOG(LogDynamicRope, Warning, TEXT("There is no URopeComponent in the world."));
		}
	}

	/** Loads DemoPresets[Index] synchronously, returning null with a warning on failure. */
	static URopePreset* LoadDemoPreset(int32 Index)
	{
		const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
		if (!Settings->DemoPresets.IsValidIndex(Index))
		{
			return nullptr;
		}
		// This is a demo-only command, so the hitch of a synchronous load is accepted, once on first application.
		URopePreset* Preset = Settings->DemoPresets[Index].LoadSynchronous();
		if (!Preset)
		{
			UE_LOG(LogDynamicRope, Warning, TEXT("Failed to load DemoPresets[%d]: %s"),
				Index, *Settings->DemoPresets[Index].ToString());
		}
		return Preset;
	}

	/** Applies a preset to every rope in the world and synchronizes the index. A refusal on phase grounds is logged by ApplyPreset. */
	static void ApplyToWorld(UWorld* World, URopePreset* Preset, int32 Index)
	{
		if (!Preset)
		{
			return;
		}
		GLastAppliedIndex = Index;
		UE_LOG(LogDynamicRope, Log, TEXT("Rope.Preset: applying '%s' (DemoPresets[%d])."), *Preset->GetName(), Index);
		ForEachRope(World, [Preset](URopeComponent& Rope)
		{
			Rope.ApplyPreset(Preset);
		});
	}

	/** Returns false, with an explanatory warning, if the settings array is empty. */
	static bool EnsureDemoPresetList()
	{
		if (UDynamicRopeSettings::Get()->DemoPresets.Num() == 0)
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("DemoPresets is empty. Register URopePreset assets under Project Settings > Plugins > Dynamic Rope > Demo."));
			return false;
		}
		return true;
	}

	static FAutoConsoleCommandWithWorldAndArgs GCycleCmd(
		TEXT("Rope.Preset.Cycle"),
		TEXT("Applies the next preset in DemoPresets, from the project settings, to every rope in the world. A rope that is neither Free nor Loaded is refused and logged."),
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
		TEXT("Applies a named preset. The argument is either a numeric index into DemoPresets or a partial match on the asset name. The cycling index is synchronized as well."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			if (Args.Num() == 0)
			{
				UE_LOG(LogDynamicRope, Warning, TEXT("Usage: Rope.Preset.Apply <index|name substring>. Use Rope.Preset.List for the list."));
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
				// A case-insensitive partial name match, searched against the soft reference's asset name without loading it.
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
				UE_LOG(LogDynamicRope, Warning, TEXT("No DemoPresets entry matches '%s' (0..%d). Check Rope.Preset.List."),
					*Args[0], Settings->DemoPresets.Num() - 1);
				return;
			}
			ApplyToWorld(World, LoadDemoPreset(Index), Index);
		}));

	static FAutoConsoleCommandWithWorldAndArgs GListCmd(
		TEXT("Rope.Preset.List"),
		TEXT("Prints the DemoPresets list and the current cycling index."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
			UE_LOG(LogDynamicRope, Log, TEXT("%d DemoPresets (current index %d):"),
				Settings->DemoPresets.Num(), GLastAppliedIndex);
			for (int32 i = 0; i < Settings->DemoPresets.Num(); ++i)
			{
				// A loaded asset also reports its mode; an unloaded one reports its name alone, so listing never triggers a load.
				const URopePreset* Loaded = Settings->DemoPresets[i].Get();
				if (Loaded)
				{
					UE_LOG(LogDynamicRope, Log, TEXT("  [%d]%s %s (mode=%s)"),
						i, (i == GLastAppliedIndex ? TEXT("*") : TEXT(" ")), *Loaded->GetName(),
						*StaticEnum<ERopeWrapResolveMode>()->GetNameStringByValue((int64)Loaded->ResolveMode));
				}
				else
				{
					UE_LOG(LogDynamicRope, Log, TEXT("  [%d]%s %s (not loaded)"),
						i, (i == GLastAppliedIndex ? TEXT("*") : TEXT(" ")), *Settings->DemoPresets[i].GetAssetName());
				}
			}
		}));
}

#endif // !UE_BUILD_SHIPPING
