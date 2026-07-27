// Copyright Epic Games, Inc. All Rights Reserved.
//
// Editor-only drawing helpers for visualizing an SDF volume, based on the PDI. They draw each bone's SDF volume as
// bounds, a grid, voxels, a slice and gradients, and are what the authoring panel's preview viewport draws with.
// Every coordinate is bone-local and placed into world space through the supplied transform.

#pragma once

#include "CoreMinimal.h"

struct FRopeBoneSDFVolume;
class FPrimitiveDrawInterface;
enum class ERopeSDFSliceAxis : uint8;

namespace RopeSDFDraw
{
	/** The twelve edges of the local AABB. */
	void DrawBounds(FPrimitiveDrawInterface* PDI, const FBox& LocalBounds, const FTransform& Xform, const FLinearColor& Color);

	/**
	 * The narrow-band voxels as coloured points by sign: inside is red, outside is blue and near zero is white, the
	 * same convention as the slice heatmap. Only samples whose absolute distance is within the band are drawn.
	 * Samples saturated against the volume's asymmetric band, at or beyond the outer or inner narrow band, are
	 * skipped, since they are clamped placeholders and raising the band would otherwise pull in the whole plateau. On
	 * the inside the automatic band covers the entire interior, so only the deepest point is affected. Skipping is
	 * disabled if the band is invalid, meaning its range is zero.
	 */
	void DrawVoxels(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform, float Band);

	/**
	 * A distance heatmap on a single-axis slice plane, with negative red, zero white and positive blue. The position
	 * runs from zero to one, the resolution is the grid size, and it saturates at the scale in centimetres.
	 * Samples saturated against the volume's asymmetric band, at or beyond the outer or inner narrow band, are drawn
	 * in dim grey rather than as a heatmap, which distinguishes the meaningless clamped plateau from the meaningful
	 * band covering the surface, the continuous field and the interior, under the same convention as the saturation
	 * skipping in the voxel and gradient draws. If the band is invalid, meaning its range is zero, the whole slice is
	 * drawn as a heatmap. The boundary between grey and colour is the contour of the band.
	 */
	void DrawSlice(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform,
		ERopeSDFSliceAxis Axis, float Pos01, int32 Res, float Scale);

	/**
	 * The gradient direction, being outwards and therefore the query normal, of narrow-band samples, drawn as arrows.
	 * Only samples whose absolute distance is within the band are drawn.
	 * Samples saturated against the volume's asymmetric band, at or beyond the outer or inner narrow band, are
	 * skipped: being clamped they carry no direction information, falling back to the up vector where they are flat
	 * and to noise at the boundary, and would only produce false arrows. Skipping is disabled if the band is invalid,
	 * meaning its range is zero.
	 */
	void DrawGradients(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform,
		float Band, float Length);
}
