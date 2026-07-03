// Copyright Epic Games, Inc. All Rights Reserved.

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
	// 오써링 탭은 단일 인스턴스이므로 다중 선택이어도 첫 에셋만 연다(나머지를 열 곳이 없다).
	// 에셋 프로퍼티(SourceMesh/Bone Volumes) 확인·편집은 패널 내장 디테일 뷰가 담당한다.
	const TArray<URopeSDFData*> Objects = OpenArgs.LoadObjects<URopeSDFData>();
	if (Objects.Num() > 0)
	{
		FModuleManager::LoadModuleChecked<FDynamicRopeEditorModule>("DynamicRopeEditor")
			.OpenSDFAuthoringTabForAsset(Objects[0]);
	}
	return EAssetCommandResult::Handled;
}

#undef LOCTEXT_NAMESPACE
