// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU tube mesh generation. It computes the tube's vertex positions from a centreline position
// buffer using a parallel-transport frame and writes them straight into the renderer's vertex buffer
// UAV, which renders the GPU state directly with no CPU tube build and no readback.
// This module, DynamicRopeShaders, does not depend on the DynamicRope render types, so it takes
// opaque RHI view handles only.

#pragma once

#include "CoreMinimal.h"

class FRHIShaderResourceView;
class FRHIUnorderedAccessView;
class FRHICommandList;

namespace RopeGPU
{
	/** The maximum number of rings the GPU tube supports, which is the largest thread group bucket. A
	 *  rope with more rings than this must fall back to the CPU tube.
	 *  The caller, the scene proxy, uses it to decide whether the GPU tube applies, which keeps the
	 *  bucket limit and the proxy's gate from diverging. */
	DYNAMICROPESHADERS_API int32 MaxTubeRings();

	/** The smallest thread group bucket at least as large as NumRings, from 64, 128, 256 and 512.
	 *  Returns 0 above the limit, which means falling back to the CPU. The dispatch uses it to select a
	 *  permutation and the debug overlay to show the bucket actually in use, from this single source. */
	DYNAMICROPESHADERS_API int32 TubeRingBucket(int32 NumRings);

	/** Decides the render tube's subdivision. It uses WantedSubdiv, from 1 to 8, reduced as needed so
	 *  that NumRings, which is (NumNodes - 1) * Subdiv + 1, does not exceed MaxTubeRings. A rope with
	 *  many nodes therefore keeps the GPU tube and only loses render smoothing gradually.
	 *  The scene proxy uses it to decide the actual subdivision and the debug overlay to show
	 *  eligibility, from this single source. */
	DYNAMICROPESHADERS_API int32 ComputeTubeSubdiv(int32 NumNodes, int32 WantedSubdiv);

	/**
	 * Render thread. Writes the tube's vertex positions into OutPositionsUAV, which is R32_FLOAT with
	 * three floats per vertex, from the centreline in InCenterlineSRV, which is R32_FLOAT with ring r's
	 * position at float indices r*3 onwards. One rope is one dispatch.
	 * There is no coordinate conversion; the input and output share a space, which is component-local.
	 * The caller is responsible for the UAV barriers.
	 * Supplying OutTangentsUAV, which is R32_UINT with a uint2 per vertex holding the packed tangent
	 * basis, and OutTexCoordsUAV, which is R32_FLOAT with a float2 per vertex, also generates the
	 * tangents and UVs on the GPU, removing the need for the CPU tube build. Pass null for either to
	 * generate positions only.
	 */
	DYNAMICROPESHADERS_API void BuildTube_RenderThread(
		FRHICommandList& RHICmdList,
		FRHIShaderResourceView* InCenterlineSRV,
		FRHIUnorderedAccessView* OutPositionsUAV,
		FRHIUnorderedAccessView* OutTangentsUAV,
		FRHIUnorderedAccessView* OutTexCoordsUAV,
		int32 NumRings, int32 NumSides, float Radius);

	/**
	 * Render thread. Takes the solver's resident position buffer, a StructuredBuffer of float4 in world
	 * space holding NumSrcNodes simulation nodes, smooths it on the GPU with Catmull-Rom interpolation
	 * at the given subdivision to produce the render centreline of NumRings, and then generates the
	 * tube's positions, tangents and UVs.
	 * No CPU mirror upload or smoothing is needed. WorldToLocal converts into component-local space. The
	 * caller is responsible for the UAV barriers.
	 */
	DYNAMICROPESHADERS_API void BuildTubeFromResident_RenderThread(
		FRHICommandList& RHICmdList,
		FRHIShaderResourceView* InResidentPositionsSRV,
		FRHIUnorderedAccessView* OutPositionsUAV,
		FRHIUnorderedAccessView* OutTangentsUAV,
		FRHIUnorderedAccessView* OutTexCoordsUAV,
		int32 NumRings, int32 NumSides, float Radius,
		int32 NumSrcNodes, int32 Subdiv, float SmoothParam,
		const FMatrix44f& WorldToLocal);
}
