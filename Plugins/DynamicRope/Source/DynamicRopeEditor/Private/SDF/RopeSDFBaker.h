// Copyright Epic Games, Inc. All Rights Reserved.
//
// Editor-only CPU baker that fills a URopeSDFData with per-bone signed distance volumes.
// Reads the skeletal mesh's editor source model (WITH_EDITOR), assigns triangles to bones by
// skin weight, transforms them into bone-local space, and voxelizes a narrow-band SDF whose
// sign comes from the generalized winding number (robust on the open, per-bone triangle patch).
//
// Convention (must match the runtime FRopeSDFCollider::Query sampler): samples sit on grid
// CORNERS, so the sample at index (x,y,z) is at LocalBounds.Min + (x,y,z) * VoxelSize, and
// LocalBounds.Max == Min + (Resolution - 1) * VoxelSize. Distance is cm, outside-positive.

#pragma once

#include "CoreMinimal.h"

class USkeletalMesh;
struct FRopeBoneSDFVolume;

/** Designer-facing knobs for a bake run. */
struct FRopeSDFBakeSettings
{
	/** Sample spacing in cm (cubic voxels). Smaller = sharper surface, more memory/time. */
	float VoxelSize = 1.5f;

	/** Per-axis sample cap. If a bone's grid would exceed this, VoxelSize is raised to fit. */
	int32 MaxResolution = 48;

	/** Clamp |distance| to this band (cm). Values past the band are not collision-relevant. */
	float NarrowBand = 6.0f;

	/** Minimum averaged skin weight for a triangle to be assigned to a bone [0..1]. */
	float WeightThreshold = 0.2f;

	/** Pad the bone's triangle AABB before voxelizing (cm), so the band has room outside the skin. */
	float BoundsPadding = 3.0f;
};

/** Stateless per-bone SDF baker. Editor-only (uses the imported source model). */
class FRopeSDFBaker
{
public:
	/**
	 * Bakes one bone-local volume per requested bone into OutVolumes.
	 *  - Bones empty  => every bone that has skinned geometry.
	 *  - A bone with no qualifying triangles is silently skipped.
	 * Returns false only if the mesh exposes no CPU geometry (e.g. cooked/stripped).
	 */
	static bool BakeMesh(USkeletalMesh* Mesh, const TArray<FName>& Bones,
		const FRopeSDFBakeSettings& Settings, TArray<FRopeBoneSDFVolume>& OutVolumes);
};
