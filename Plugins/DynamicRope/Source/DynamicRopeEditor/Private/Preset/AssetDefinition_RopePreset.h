// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopePreset의 Content Browser 표현(이름/색/카테고리). UAssetDefinition CDO는 자동 발견되므로
// StartupModule에서의 수동 등록이 필요 없다. 더블클릭은 제네릭 프로퍼티 에디터로 충분해
// OpenAssets를 오버라이드하지 않는다(SDF와 달리 전용 오써링 탭이 없다).

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
