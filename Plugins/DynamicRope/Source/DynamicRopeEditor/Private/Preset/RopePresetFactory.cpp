// Copyright 2026 TeamKeno. All Rights Reserved.

#include "RopePresetFactory.h"
#include "Preset/RopePreset.h"

URopePresetFactory::URopePresetFactory()
{
	bCreateNew = true;
	bEditAfterNew = true;
	SupportedClass = URopePreset::StaticClass();
}

UObject* URopePresetFactory::FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName,
	EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn)
{
	return NewObject<URopePreset>(InParent, InClass, InName, Flags);
}
