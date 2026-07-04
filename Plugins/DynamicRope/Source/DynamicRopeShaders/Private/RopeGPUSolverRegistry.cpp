// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeGPUSolverRegistry.h"
#include "HAL/CriticalSection.h"
#include "HAL/IConsoleManager.h"

// Phase 1 검증/디버그용: 강제로 GDF 소비자를 켠다(로프 없이도 GDF 빌드 유발). 기본 off.
static TAutoConsoleVariable<int32> CVarForceGDFConsumer(
	TEXT("r.DynamicRope.ForceGDFConsumer"),
	0,
	TEXT("DynamicRope: 1이면 GDF 로프 유무와 무관하게 커스텀 FX 시스템이 GDF를 요구한다(온디맨드 빌드 경로 검증용)."),
	ECVF_RenderThreadSafe);

// GPU 솔브 dispatch 경로: 0=Step() 전용 그래프(현행), 1=뷰 확장에서 씬 렌더러 그래프로 dispatch(GDF 통합).
// GT(서브시스템)와 RT(뷰 확장) 양쪽에서 읽으므로 RenderThreadSafe.
static TAutoConsoleVariable<int32> CVarGDFDispatchInVE(
	TEXT("r.DynamicRope.GDFDispatchInVE"),
	0,
	TEXT("DynamicRope: 0=GPU 솔브를 자체 RDG 그래프에서 실행(기본), 1=씬 뷰 확장(PreRenderBasePass)에서 씬 그래프로 실행."),
	ECVF_RenderThreadSafe);

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
		if (CVarForceGDFConsumer.GetValueOnAnyThread() != 0)
		{
			return true; // 검증용 강제.
		}
		if (!Scene)
		{
			return false;
		}
		FScopeLock Lock(&GRegistryCS);
		return GGDFActiveCounts.FindRef(Scene) > 0;
	}

	bool IsDispatchInVE()
	{
		return CVarGDFDispatchInVE.GetValueOnAnyThread() != 0;
	}
}
