// Copyright 2026 TeamKeno. All Rights Reserved.

#include "RopeSDFDataFactory.h"
#include "DynamicRopeEditorLog.h"
#include "Collision/SDF/RopeSDFData.h"

URopeSDFDataFactory::URopeSDFDataFactory()
{
	bCreateNew = true;
	bEditAfterNew = true;
	SupportedClass = URopeSDFData::StaticClass();
}

UObject* URopeSDFDataFactory::FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName,
	EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn)
{
	UE_LOG(LogDynamicRopeEditor, Verbose, TEXT("Created Rope SDF Data asset '%s'."), *InName.ToString());
	return NewObject<URopeSDFData>(InParent, InClass, InName, Flags);
}
