// Copyright Epic Games, Inc. All Rights Reserved.
//
// The editor-only CPU baker that fills a URopeSDFData with per-bone signed distance volumes.
// It reads the skeletal mesh's editor source model, assigns triangles to bones by skin weight,
// transforms them into bone-local space, and voxelizes a narrow-band SDF.
// The unsigned distance is measured against that bone's own triangles, which preserves the attribution,
// while the sign, meaning inside or outside, comes from the generalized winding number over the whole
// mesh, treated as a closed surface, using the fast winding implementation in GeometryCore. Integrating
// over a bone's open patch alone would misjudge the interior of a short, wide bone segment as outside, so
// integrating over the global mesh is what makes it robust.
//
// The convention, which must match the runtime sampler in FRopeSDFCollider::Query: samples sit on grid
// corners, so the sample at index (x, y, z) is at the bounds minimum plus (x, y, z) times the voxel size,
// and the bounds maximum equals the minimum plus the resolution minus one, times the voxel size.
// Distances are in centimetres and positive outside.

#pragma once

#include "CoreMinimal.h"
// The progress callback type.
#include "Templates/Function.h"
// FRopeSDFBakeSettings, a runtime struct stored on the asset.
#include "Collision/SDF/RopeSDFData.h"

class USkeletalMesh;
struct FRopeBoneSDFVolume;

// FRopeSDFBakeSettings was promoted to the runtime module, in RopeSDFData.h, so the bake settings can be
// stored on the asset as URopeSDFData::LastBakeSettings and used as the comparison baseline when
// authoring again. It is taken here as the input type directly.

/**
 * The per-bone progress and cancellation callback, invoked once as the voxelization of each target bone
 * begins.
 *  - Done is the zero-based ordinal of the bone about to start.
 *  - Total is the number of target bones.
 *  - Bone is the name of the bone about to start.
 * Returning false aborts the bake immediately, while true continues. Left empty, the default, it bakes
 * to completion with no reporting and no cancellation.
 */
using FRopeSDFBakeProgress = TFunction<bool(int32 /*Done*/, int32 /*Total*/, const FName& /*Bone*/)>;

/**
 * A cancellation poll called frequently during a bake, between the voxel batches within a bone.
 * Returning true aborts immediately.
 * It exists so that voxelizing a heavy bone does not occupy the game thread for a long time: between
 * batches this poll pumps the slow task UI and processes the cancel button. Left empty, the default,
 * only per-bone cancellation through the progress callback's return value applies.
 */
using FRopeSDFBakeCancelPoll = TFunction<bool()>;

/** The result of BakeMesh. */
enum class ERopeSDFBakeResult : uint8
{
	// Completed normally, although the result may contain zero bones.
	Success,
	// There is no CPU geometry, as in a cooked or stripped build, or the mesh is null, so it cannot be
	// baked.
	NoGeometry,
	// A callback requested an abort. The output volumes are incomplete and must not be written to the
	// asset.
	Cancelled,
};

/**
 * A record of one bone whose requested voxel size had to be increased, that is coarsened, because of the
 * maximum resolution limit.
 * It lets the user see in the message log why the spacing they entered was baked more coarsely.
 */
struct FRopeSDFCoarsenedBone
{
	// The bone's name.
	FName      Bone;
	// The voxel size the user entered (cm).
	float      RequestedVoxelSize;
	// The voxel size actually used after coarsening to fit the limit (cm).
	float      ActualVoxelSize;
	// The final grid resolution, as a sample count per axis.
	FIntVector Resolution;
};

/** Aggregate statistics for one BakeMesh call, for editor reporting. A plain type that is not stored on
 *  the asset. */
struct FRopeSDFBakeStats
{
	// The number of bones actually baked into volumes.
	int32 BonesBaked = 0;
	// Only the bones that were coarsened are recorded.
	TArray<FRopeSDFCoarsenedBone> CoarsenedBones;
	// Bones dropped because their girth fell below the configured minimum.
	TArray<FName> DroppedThinBones;
};

/** The stateless per-bone SDF baker. Editor only, since it uses the import source model. */
class FRopeSDFBaker
{
public:
	/**
	 * Bakes one bone-local volume per requested bone into the output array.
	 *  - An empty bone list means every bone with skinned geometry.
	 *  - A bone with no eligible triangles is skipped silently.
	 *  - Where a progress callback is supplied it is called just before each bone is processed, which
	 *    drives the progress display and optional per-bone cancellation.
	 *  - Where a cancellation poll is supplied it is called between the voxel batches within a bone, so a
	 *    cancellation is honoured even part-way through a heavy one.
	 *    A cancellation from either callback returns Cancelled and leaves the output volumes incomplete.
	 *  - Where statistics are supplied they are filled in with the number of bones baked and the list of
	 *    bones that were coarsened, for reporting.
	 * A mesh with no CPU geometry, as in a cooked or stripped build, returns NoGeometry.
	 */
	static ERopeSDFBakeResult BakeMesh(USkeletalMesh* Mesh, const TArray<FName>& Bones,
		const FRopeSDFBakeSettings& Settings, TArray<FRopeBoneSDFVolume>& OutVolumes,
		const FRopeSDFBakeProgress& Progress = FRopeSDFBakeProgress(),
		const FRopeSDFBakeCancelPoll& CancelPoll = FRopeSDFBakeCancelPoll(),
		FRopeSDFBakeStats* OutStats = nullptr);
};
