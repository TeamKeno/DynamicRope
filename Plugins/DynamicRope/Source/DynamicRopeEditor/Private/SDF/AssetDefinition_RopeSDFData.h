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
	/** 더블클릭 시 제네릭 프로퍼티 에디터 대신 SDF 오써링 탭을 열어 해당 에셋을 타깃으로 지정한다. */
	virtual EAssetCommandResult OpenAssets(const FAssetOpenArgs& OpenArgs) const override;
};
