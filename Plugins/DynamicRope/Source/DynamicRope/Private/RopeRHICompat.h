// Copyright Epic Games, Inc. All Rights Reserved.
//
// Isolates the engine version differences in RHI buffer creation in one place. In 5.6 the FRHIBufferCreateDesc
// builder was introduced and the API became CreateVertex, CreateIndex and CreateStructured followed by CreateBuffer,
// while 5.5 uses the older CreateVertexBuffer, CreateIndexBuffer and CreateStructuredBuffer with
// FRHIResourceCreateInfo. Call sites, being RopeSceneProxy and the GPU tube tests, use these wrappers alone and stay
// version-clean. Creating SRVs and UAVs through FRHIViewDesc exists in 5.5 as well and is left as it is.

#pragma once

#include "CoreMinimal.h"
#include "RHICommandList.h"
#include "RHIResources.h"
#include "Misc/EngineVersionComparison.h"

namespace RopeRHI
{
	/** A vertex buffer, for typed SRVs and UAVs. The size is in total bytes. */
	inline FBufferRHIRef CreateVertexBuffer(FRHICommandListBase& RHICmdList, const TCHAR* Name, uint32 Bytes,
		EBufferUsageFlags Usage)
	{
#if UE_VERSION_OLDER_THAN(5, 6, 0)
		FRHIResourceCreateInfo CreateInfo(Name);
		return RHICmdList.CreateVertexBuffer(Bytes, Usage, CreateInfo);
#else
		return RHICmdList.CreateBuffer(FRHIBufferCreateDesc::CreateVertex(Name, Bytes)
			.AddUsage(Usage)
			.DetermineInitialState());
#endif
	}

	/** An index buffer. The stride is the size of one index and the count is the number of indices. */
	inline FBufferRHIRef CreateIndexBuffer(FRHICommandListBase& RHICmdList, const TCHAR* Name, uint32 Stride,
		uint32 NumIndices, EBufferUsageFlags Usage)
	{
#if UE_VERSION_OLDER_THAN(5, 6, 0)
		FRHIResourceCreateInfo CreateInfo(Name);
		return RHICmdList.CreateIndexBuffer(Stride, Stride * NumIndices, Usage, CreateInfo);
#else
		return RHICmdList.CreateBuffer(FRHIBufferCreateDesc::CreateIndex(Name, Stride * NumIndices, Stride)
			.AddUsage(Usage)
			.DetermineInitialState());
#endif
	}

	/** A structured buffer. The stride is the element size and the count is the number of elements. */
	inline FBufferRHIRef CreateStructuredBuffer(FRHICommandListBase& RHICmdList, const TCHAR* Name, uint32 Stride,
		uint32 Count, EBufferUsageFlags Usage)
	{
#if UE_VERSION_OLDER_THAN(5, 6, 0)
		FRHIResourceCreateInfo CreateInfo(Name);
		return RHICmdList.CreateStructuredBuffer(Stride, Stride * Count, Usage, CreateInfo);
#else
		return RHICmdList.CreateBuffer(FRHIBufferCreateDesc::CreateStructured(Name, Stride * Count, Stride)
			.AddUsage(Usage)
			.DetermineInitialState());
#endif
	}
}
