// Copyright Epic Games, Inc. All Rights Reserved.
//
// RHI 버퍼 생성의 엔진 버전차를 한 곳에 격리한다. 5.6에서 FRHIBufferCreateDesc 빌더가 도입되면서
// CreateVertex/CreateIndex/CreateStructured + CreateBuffer 형태로 바뀌었고, 5.5는 구 CreateVertexBuffer/
// CreateIndexBuffer/CreateStructuredBuffer + FRHIResourceCreateInfo 경로다. 호출부(RopeSceneProxy, GPU 튜브
// 테스트)는 이 래퍼만 쓰고 버전-클린하게 유지한다 — SRV/UAV 생성(FRHIViewDesc)은 5.5에도 있어 그대로 둔다.

#pragma once

#include "CoreMinimal.h"
#include "RHICommandList.h"
#include "RHIResources.h"
#include "Misc/EngineVersionComparison.h"

namespace RopeRHI
{
	/** 버텍스 버퍼(타입드 SRV/UAV용). Bytes = 전체 바이트. */
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

	/** 인덱스 버퍼. Stride = 인덱스 1개 크기, NumIndices = 개수. */
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

	/** 스트럭처드 버퍼. Stride = 원소 크기, Count = 원소 수. */
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
