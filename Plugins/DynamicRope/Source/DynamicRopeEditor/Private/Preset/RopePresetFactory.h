// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The factory that creates a URopePreset from the content browser. A UFactory CDO is discovered automatically, so no
// separate registration is needed; the "Dynamic Rope" category in the Add menu is provided by AssetDefinition_RopePreset.

#pragma once

#include "CoreMinimal.h"
#include "Factories/Factory.h"
#include "RopePresetFactory.generated.h"

UCLASS()
class URopePresetFactory : public UFactory
{
	GENERATED_BODY()

public:
	URopePresetFactory();

	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName,
		EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn) override;
};
