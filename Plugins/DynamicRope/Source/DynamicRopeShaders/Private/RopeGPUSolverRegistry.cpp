// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeGPUSolverRegistry.h"
#include "HAL/CriticalSection.h"

namespace RopeGDF
{
	// 두 맵 모두 game/render 스레드에서 접근되므로 하나의 락으로 보호한다(맵이 작아 경합 무시 가능).
	static FCriticalSection GRegistryCS;
	static TMap<FSceneInterface*, FRopeGPUSolver*> GSolvers;
	static TMap<FSceneInterface*, int32>           GGDFActiveCounts;

	void RegisterSolver(FSceneInterface* Scene, FRopeGPUSolver* Solver)
	{
		if (!Scene || !Solver)
		{
			return;
		}
		FScopeLock Lock(&GRegistryCS);
		GSolvers.Add(Scene, Solver);
	}

	void UnregisterSolver(FSceneInterface* Scene)
	{
		if (!Scene)
		{
			return;
		}
		FScopeLock Lock(&GRegistryCS);
		GSolvers.Remove(Scene);
		GGDFActiveCounts.Remove(Scene);
	}

	FRopeGPUSolver* FindSolver(FSceneInterface* Scene)
	{
		if (!Scene)
		{
			return nullptr;
		}
		FScopeLock Lock(&GRegistryCS);
		return GSolvers.FindRef(Scene);
	}

	void SetGDFActiveCount(FSceneInterface* Scene, int32 Count)
	{
		if (!Scene)
		{
			return;
		}
		FScopeLock Lock(&GRegistryCS);
		if (Count > 0)
		{
			GGDFActiveCounts.Add(Scene, Count);
		}
		else
		{
			GGDFActiveCounts.Remove(Scene);
		}
	}

	bool IsGDFActive(FSceneInterface* Scene)
	{
		if (!Scene)
		{
			return false;
		}
		FScopeLock Lock(&GRegistryCS);
		return GGDFActiveCounts.FindRef(Scene) > 0;
	}
}
