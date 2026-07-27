// Copyright Epic Games, Inc. All Rights Reserved.
//
// A shared helper that extracts the simple collision of a UBodySetup, that is its sphyls, spheres,
// boxes and convexes, into world-space push-out colliders. It is used both by
// URopeStaticBodyProvider, which finds bodies through channel overlaps, and by
// URopeWrapTargetComponent, which extracts a target's bodies directly. An extracted collider has no
// bone, which makes IsWorldStatic() true and excludes it from detection, leaving it push-out only.

#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"
#include "Collision/RopeStaticCollider.h"

class UBodySetup;
class USceneComponent;

namespace RopeBodyColliderExtraction
{
	/**
	 * Appends the setup's simple collision to the per-type output arrays as world-space colliders. The
	 * budget, MaxColliders, counts the sum of all three arrays, and exceeding it returns false after a
	 * partial extraction.
	 * PrevCompTM and InvDeltaTime supply the surface velocity of a dynamic body; for a static one, pass
	 * PrevCompTM equal to CompTM and an InvDeltaTime of 0.
	 * A convex with more planes than MaxConvexPlanes, or one that is not cooked, falls back to the
	 * element box OBB and reports it through OnConvexFallback, which receives the plane count, for the
	 * caller to log.
	 *
	 * Attribution: with the defaults, meaning no bone and a null mesh, every collider is produced with
	 * no bone, making it push-out only, which is the URopeStaticBodyProvider path. Supplying them
	 * attaches a virtual bone and a source mesh to the box, sphyl and sphere elements, and to the
	 * convex OBB fallback, which makes them wrappable and able to take part in detection; that is the
	 * full-set mode of URopeWrapTargetComponent. Genuine convexes (plane sets) are attributed the
	 * same way - the GPU detect kernel runs a convex loop over the wrappable range, so an attributed
	 * convex joins detection exactly like the box and capsule elements.
	 */
	DYNAMICROPE_API bool AppendBodyColliders(
		const UBodySetup& Setup, const FTransform& CompTM, const FTransform& PrevCompTM,
		float InvDeltaTime, int32 MaxColliders, int32 MaxConvexPlanes,
		TArray<FRopeBoxCollider>& OutBoxes,
		TArray<FRopeStaticCapsuleCollider>& OutCapsules,
		TArray<FRopeConvexCollider>& OutConvexes,
		const TFunctionRef<void(int32 NumPlanes)>& OnConvexFallback,
		FName AttributionBone = NAME_None, const USceneComponent* AttributionMesh = nullptr);
}
