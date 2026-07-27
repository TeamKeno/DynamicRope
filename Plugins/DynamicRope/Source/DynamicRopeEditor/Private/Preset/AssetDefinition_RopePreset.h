// Copyright Epic Games, Inc. All Rights Reserved.
//
// The content browser representation of URopePreset, being its name, colour and category. A UAssetDefinition CDO is
// discovered automatically, so no manual registration in StartupModule is needed. The generic property editor is
// enough for a double click, so OpenAssets is not overridden: unlike the SDF data there is no dedicated authoring tab.

#pragma once

#include "CoreMinimal.h"
#include "AssetDefinitionDefault.h"
#include "AssetDefinition_RopePreset.generated.h"

UCLASS()
class UAssetDefinition_RopePreset : public UAssetDefinitionDefault
{
	GENERATED_BODY()

public:
	//~ UAssetDefinition
	virtual FText GetAssetDisplayName() const override;
	virtual FLinearColor GetAssetColor() const override;
	virtual TSoftClassPtr<UObject> GetAssetClass() const override;
	virtual TConstArrayView<FAssetCategoryPath> GetAssetCategories() const override;
};
