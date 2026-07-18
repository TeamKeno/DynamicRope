// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFData.h"
#include "HAL/ThreadSafeCounter64.h"

uint64 URopeSDFData::GetRuntimeVolumeId() const
{
	if (RuntimeVolumeId == 0)
	{
		// 프로세스 전역 단조 증가 카운터(재사용 없음). collider 조립(GT)에서 호출된다.
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
