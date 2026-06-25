// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeSDFData를 Content Browser에서 생성하는 팩토리. UFactory CDO는 자동 발견되므로 별도 등록은
// 필요 없다(Add 메뉴의 "Dynamic Rope" 카테고리는 AssetDefinition_RopeSDFData가 제공).

#pragma once

#include "CoreMinimal.h"
#include "Factories/Factory.h"
#include "RopeSDFDataFactory.generated.h"

UCLASS()
class URopeSDFDataFactory : public UFactory
{
	GENERATED_BODY()

public:
	URopeSDFDataFactory();

	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName,
		EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn) override;
};
