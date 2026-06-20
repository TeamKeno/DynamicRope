// Copyright Epic Games, Inc. All Rights Reserved.

#include "Settings/DynamicRopeSettings.h"

UDynamicRopeSettings::UDynamicRopeSettings()
{
	CategoryName = TEXT("Plugins");
	SectionName = TEXT("Dynamic Rope");

	// Sensible starting set of wrappable bones for the standard UE mannequin skeleton.
	WrappableBones = {
		TEXT("upperarm_l"), TEXT("lowerarm_l"),
		TEXT("upperarm_r"), TEXT("lowerarm_r"),
		TEXT("thigh_l"), TEXT("calf_l"),
		TEXT("thigh_r"), TEXT("calf_r"),
		TEXT("spine_03"),
	};
}

const UDynamicRopeSettings* UDynamicRopeSettings::Get()
{
	return GetDefault<UDynamicRopeSettings>();
}
