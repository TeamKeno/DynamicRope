// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU 튜브 메시 생성(M5b). 센터라인 위치 버퍼에서 parallel-transport frame으로 튜브 정점 위치를 계산해
// 외부(렌더러) vertex buffer UAV에 직접 기록한다 → CPU BuildTube/리드백 없이 GPU 상태를 바로 렌더.
// 이 모듈(DynamicRopeShaders)은 DynamicRope 렌더 타입에 의존하지 않는다 → opaque RHI view 핸들만 받는다.

#pragma once

#include "CoreMinimal.h"

class FRHIShaderResourceView;
class FRHIUnorderedAccessView;
class FRHICommandList;

namespace RopeGPU
{
	/**
	 * 렌더 스레드. 센터라인(InCenterlineSRV: R32_FLOAT 타입, ring r 위치 = float[r*3..])에서 튜브 정점
	 * 위치를 OutPositionsUAV(R32_FLOAT, v당 float3)에 기록한다. 로프 1개 = 1 디스패치.
	 * 좌표계 변환 없음 — 입력/출력 동일 공간(B1: component-local). 호출자가 UAV 배리어를 책임진다.
	 * OutTangentsUAV(R32_UINT, v당 uint2 = FPackedNormal TangentX/TangentZ) + OutTexCoordsUAV(R32_FLOAT,
	 * v당 float2)를 주면 tangent/UV도 GPU 생성한다(B2-full, CPU BuildTube 불필요). null이면 위치만.
	 */
	DYNAMICROPESHADERS_API void BuildTube_RenderThread(
		FRHICommandList& RHICmdList,
		FRHIShaderResourceView* InCenterlineSRV,
		FRHIUnorderedAccessView* OutPositionsUAV,
		FRHIUnorderedAccessView* OutTangentsUAV,
		FRHIUnorderedAccessView* OutTexCoordsUAV,
		int32 NumRings, int32 NumSides, float Radius);

	/**
	 * 렌더 스레드(M5b B2-lite / B2-full). 솔버 resident PosBuf(StructuredBuffer<float4>, 월드)에서 직접 튜브
	 * 위치를 OutPositionsUAV에 기록. WorldToLocal로 component-local 변환 → 위치 무지연(리드백 없음).
	 * OutTangentsUAV/OutTexCoordsUAV를 주면 tangent/UV도 GPU 생성(B2-full). 호출자가 UAV 배리어를 책임진다.
	 */
	DYNAMICROPESHADERS_API void BuildTubeFromResident_RenderThread(
		FRHICommandList& RHICmdList,
		FRHIShaderResourceView* InResidentPositionsSRV,
		FRHIUnorderedAccessView* OutPositionsUAV,
		FRHIUnorderedAccessView* OutTangentsUAV,
		FRHIUnorderedAccessView* OutTexCoordsUAV,
		int32 NumRings, int32 NumSides, float Radius,
		const FMatrix44f& WorldToLocal);
}
