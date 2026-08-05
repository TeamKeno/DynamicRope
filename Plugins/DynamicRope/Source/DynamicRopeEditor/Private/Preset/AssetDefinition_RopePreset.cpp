// Copyright 2026 TeamKeno. All Rights Reserved.

#include "AssetDefinition_RopePreset.h"
#include "Preset/RopePreset.h"

#define LOCTEXT_NAMESPACE "AssetDefinition_RopePreset"

FText UAssetDefinition_RopePreset::GetAssetDisplayName() const
{
	return LOCTEXT("DisplayName", "Rope Preset");
}

FLinearColor UAssetDefinition_RopePreset::GetAssetColor() const
{
	// A rope, meaning jute, tone, which distinguishes this plugin asset from the SDF data's sky blue.
	return FLinearColor(FColor(214, 154, 62));
}

TSoftClassPtr<UObject> UAssetDefinition_RopePreset::GetAssetClass() const
{
	return URopePreset::StaticClass();
}

TConstArrayView<FAssetCategoryPath> UAssetDefinition_RopePreset::GetAssetCategories() const
{
	// Shares the same "Dynamic Rope" Add menu category as the SDF data.
	static const TArray<FAssetCategoryPath> Categories = { FAssetCategoryPath(LOCTEXT("DynamicRopeCategory", "Dynamic Rope")) };
	return Categories;
}

#undef LOCTEXT_NAMESPACE
