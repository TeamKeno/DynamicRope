// Copyright Epic Games, Inc. All Rights Reserved.

#include "Settings/DynamicRopeSettings.h"

UDynamicRopeSettings::UDynamicRopeSettings()
{
	CategoryName = TEXT("Plugins");
	SectionName = TEXT("Dynamic Rope");

	// 표준 UE 마네킹 스켈레톤을 위한 적절한 초기 wrappable 본 집합.
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
