// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeSDFData의 Content Browser 표현(이름/색/카테고리). UAssetDefinition CDO는 자동 발견되므로
// StartupModule에서의 수동 등록이 필요 없다.

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
};
