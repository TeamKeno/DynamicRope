// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFData.h"

const FRopeBoneSDFVolume* URopeSDFData::FindVolume(FName Bone) const
{
	for (const FRopeBoneSDFVolume& Volume : BoneVolumes)
	{
		if (Volume.Bone == Bone)
		{
			return &Volume;
		}
	}
	return nullptr;
}

bool URopeSDFData::HasAnyBakedVolume() const
{
	for (const FRopeBoneSDFVolume& Volume : BoneVolumes)
	{
		if (Volume.IsBaked())
		{
			return true;
		}
	}
	return false;
}
