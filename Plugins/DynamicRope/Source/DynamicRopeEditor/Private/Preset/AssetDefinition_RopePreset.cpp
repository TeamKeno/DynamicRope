// Copyright Epic Games, Inc. All Rights Reserved.

#include "AssetDefinition_RopePreset.h"
#include "Preset/RopePreset.h"

#define LOCTEXT_NAMESPACE "AssetDefinition_RopePreset"

FText UAssetDefinition_RopePreset::GetAssetDisplayName() const
{
	return LOCTEXT("DisplayName", "Rope Preset");
}

FLinearColor UAssetDefinition_RopePreset::GetAssetColor() const
{
	// 로프(황마) 계열 — SDF(하늘색)와 구분되는 플러그인 에셋 색.
	return FLinearColor(FColor(214, 154, 62));
}

TSoftClassPtr<UObject> UAssetDefinition_RopePreset::GetAssetClass() const
{
	return URopePreset::StaticClass();
}

TConstArrayView<FAssetCategoryPath> UAssetDefinition_RopePreset::GetAssetCategories() const
{
	// SDF와 같은 "Dynamic Rope" Add 메뉴 카테고리를 공유한다.
	static const TArray<FAssetCategoryPath> Categories = { FAssetCategoryPath(LOCTEXT("DynamicRopeCategory", "Dynamic Rope")) };
	return Categories;
}

#undef LOCTEXT_NAMESPACE
