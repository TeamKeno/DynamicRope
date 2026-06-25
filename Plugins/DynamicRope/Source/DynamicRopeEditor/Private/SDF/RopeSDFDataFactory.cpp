// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeSDFDataFactory.h"
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
	return NewObject<URopeSDFData>(InParent, InClass, InName, Flags);
}
