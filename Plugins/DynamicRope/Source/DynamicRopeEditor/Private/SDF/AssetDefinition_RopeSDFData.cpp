// Copyright 2026 TeamKeno. All Rights Reserved.

#include "AssetDefinition_RopeSDFData.h"
#include "Collision/SDF/RopeSDFData.h"
#include "DynamicRopeEditorModule.h"
#include "Modules/ModuleManager.h"

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

EAssetCommandResult UAssetDefinition_RopeSDFData::OpenAssets(const FAssetOpenArgs& OpenArgs) const
{
	// The authoring tab is a single instance, so even with a multiple selection only the first asset is opened, since
	// there is nowhere to open the rest. Inspecting and editing the asset's properties, its source mesh and bone
	// volumes, is the job of the details view built into the panel.
	const TArray<URopeSDFData*> Objects = OpenArgs.LoadObjects<URopeSDFData>();
	if (Objects.Num() > 0)
	{
		FModuleManager::LoadModuleChecked<FDynamicRopeEditorModule>("DynamicRopeEditor")
			.OpenSDFAuthoringTabForAsset(Objects[0]);
	}
	return EAssetCommandResult::Handled;
}

#undef LOCTEXT_NAMESPACE
