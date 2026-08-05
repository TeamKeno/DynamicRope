// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Isolates the engine version difference in compute PSO precaching in one place. Up to 5.7
// PipelineStateCache::PrecacheComputePipelineState took the compute shader, a debug name and a force flag
// directly; 5.8 replaced that pair of trailing arguments with an FPSOPrecacheRequestParams struct and takes
// an initializer rather than the shader, so the name became RequestParams.ResourceName and the flag became
// RequestParams.bForcePrecache. Both versions copy the initializer, mark it bPSOPrecache and compile async
// on the thread pool, so the wrapper only has to move the same three values into the shape the version
// expects — nothing about the precache behaviour differs. Call sites, being the solver and tube PSO
// precache passes, use this wrapper alone and stay version-clean.

#pragma once

#include "CoreMinimal.h"
#include "PipelineStateCache.h"
#include "RHIResources.h"
#include "Misc/EngineVersionComparison.h"

namespace RopePSO
{
	/**
	 * Precaches one compute shader's PSO. Name is a debug label only, surfaced in the PSO compilation
	 * event. bForcePrecache bypasses the r.PSOPrecaching master switch, which is off in editor and
	 * uncooked -game runs — exactly the runs where the first-dispatch driver compile is felt.
	 */
	inline FPSOPrecacheRequestResult PrecacheCompute(FRHIComputeShader* ComputeShader, const TCHAR* Name,
		bool bForcePrecache)
	{
#if UE_VERSION_OLDER_THAN(5, 8, 0)
		return PipelineStateCache::PrecacheComputePipelineState(ComputeShader, Name, bForcePrecache);
#else
		FPSOPrecacheRequestParams RequestParams;
		RequestParams.ResourceName = Name;
		RequestParams.bForcePrecache = bForcePrecache;
		return PipelineStateCache::PrecacheComputePipelineState(
			FComputePipelineStateInitializer(ComputeShader), RequestParams);
#endif
	}
}
