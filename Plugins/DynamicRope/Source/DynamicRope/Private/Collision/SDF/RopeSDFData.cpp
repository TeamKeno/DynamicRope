// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFData.h"
#include "HAL/ThreadSafeCounter64.h"

uint64 URopeSDFData::GetRuntimeVolumeId() const
{
	if (RuntimeVolumeId == 0)
	{
		// A process-wide monotonically increasing counter, never reused. Called during collider assembly on the game thread.
		static FThreadSafeCounter64 GRuntimeVolumeIdCounter(0);
		RuntimeVolumeId = static_cast<uint64>(GRuntimeVolumeIdCounter.Increment());
	}
	return RuntimeVolumeId;
}

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
