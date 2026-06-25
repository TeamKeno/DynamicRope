// Copyright Epic Games, Inc. All Rights Reserved.

#include "AssetDefinition_RopeSDFData.h"
#include "Collision/SDF/RopeSDFData.h"

#define LOCTEXT_NAMESPACE "AssetDefinition_RopeSDFData"

FText UAssetDefinition_RopeSDFData::GetAssetDisplayName() const
{
	return LOCTEXT("DisplayName", "Rope SDF Data");
}

FLinearColor UAssetDefinition_RopeSDFData::GetAssetColor() const
{
	return FLinearColor(FColor(120, 200, 255));
}

TSoftClassPtr<UObject> UAssetDefinition_RopeSDFData::GetAssetClass() const
{
	return URopeSDFData::StaticClass();
}

TConstArrayView<FAssetCategoryPath> UAssetDefinition_RopeSDFData::GetAssetCategories() const
{
	static const TArray<FAssetCategoryPath> Categories = { FAssetCategoryPath(LOCTEXT("DynamicRopeCategory", "Dynamic Rope")) };
	return Categories;
}

#undef LOCTEXT_NAMESPACE
