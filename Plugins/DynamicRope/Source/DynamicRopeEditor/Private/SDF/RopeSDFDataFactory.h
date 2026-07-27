// Copyright Epic Games, Inc. All Rights Reserved.
//
// The factory that creates a URopeSDFData from the content browser. A UFactory CDO is discovered automatically, so no
// separate registration is needed; the "Dynamic Rope" category in the Add menu is provided by AssetDefinition_RopeSDFData.

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
