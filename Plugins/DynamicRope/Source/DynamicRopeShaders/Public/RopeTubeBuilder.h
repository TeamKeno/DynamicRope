// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU 튜브 메시 생성(M5b). 센터라인 위치 버퍼에서 parallel-transport frame으로 튜브 정점 위치를 계산해
// 외부(렌더러) vertex buffer UAV에 직접 기록한다 → CPU BuildTube/리드백 없이 GPU 상태를 바로 렌더.
// 이 모듈(DynamicRopeShaders)은 DynamicRope 렌더 타입에 의존하지 않는다 → opaque RHI view 핸들만 받는다.

#pragma once

#include "CoreMinimal.h"
#include "RenderGraphFwd.h" // FRDGBuilder / FRDGBufferRef (RDG 튜브 경로)

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
	 * 렌더 스레드(B2-full). 솔버 resident PosBuf(StructuredBuffer<float4>, 월드, 시뮬 노드 NumSrcNodes개)를
	 * GPU에서 Catmull-Rom 스무딩(Subdiv)해 렌더 센터라인(NumRings)을 만든 뒤 튜브 pos/tangent/UV를 생성한다.
	 * CPU 미러 업로드/스무딩 불필요 → 위치 무지연. WorldToLocal로 component-local 변환. 호출자가 UAV 배리어 책임.
	 */
	DYNAMICROPESHADERS_API void BuildTubeFromResident_RenderThread(
		FRHICommandList& RHICmdList,
		FRHIShaderResourceView* InResidentPositionsSRV,
		FRHIUnorderedAccessView* OutPositionsUAV,
		FRHIUnorderedAccessView* OutTangentsUAV,
		FRHIUnorderedAccessView* OutTexCoordsUAV,
		int32 NumRings, int32 NumSides, float Radius,
		int32 NumSrcNodes, int32 Subdiv,
		const FMatrix44f& WorldToLocal);

	/**
	 * 렌더 스레드(Phase 2b). BuildTubeFromResident_RenderThread의 RDG 버전 — 씬 렌더러 그래프에 튜브 생성
	 * 패스를 얹는다(솔브 뒤 자동 정렬, 배리어 RDG 관리). 입력/출력은 전부 RDG 버퍼 핸들이어야 한다:
	 * InResidentPositions=StructuredBuffer<float4>(솔버 resident PosBuf), Out*=typed 정점 스트림 버퍼(UAV).
	 * 호출자가 이후 UseExternalAccessMode로 base pass에 넘긴다.
	 */
	DYNAMICROPESHADERS_API void BuildTubeFromResidentRDG_RenderThread(
		FRDGBuilder& GraphBuilder,
		FRDGBufferRef InResidentPositions,
		FRDGBufferRef OutPositions,
		FRDGBufferRef OutTangents,
		FRDGBufferRef OutTexCoords,
		int32 NumRings, int32 NumSides, float Radius,
		int32 NumSrcNodes, int32 Subdiv,
		const FMatrix44f& WorldToLocal);
}
