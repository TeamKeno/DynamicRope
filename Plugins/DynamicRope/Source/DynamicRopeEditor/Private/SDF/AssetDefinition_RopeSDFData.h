// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The content browser representation of URopeSDFData, being its name, colour and category. A UAssetDefinition CDO is
// discovered automatically, so no manual registration in StartupModule is needed.

#pragma once

#include "CoreMinimal.h"
#include "AssetDefinitionDefault.h"
#include "AssetDefinition_RopeSDFData.generated.h"

UCLASS()
class UAssetDefinition_RopeSDFData : public UAssetDefinitionDefault
{
	GENERATED_BODY()

public:
	//~ UAssetDefinition
	virtual FText GetAssetDisplayName() const override;
	virtual FLinearColor GetAssetColor() const override;
	virtual TSoftClassPtr<UObject> GetAssetClass() const override;
	virtual TConstArrayView<FAssetCategoryPath> GetAssetCategories() const override;
	/** On a double click it opens the SDF authoring tab targeting that asset, rather than the generic property editor. */
	virtual EAssetCommandResult OpenAssets(const FAssetOpenArgs& OpenArgs) const override;
};
