// Copyright Epic Games, Inc. All Rights Reserved.
//
// Pure utilities for sampling an FRopeBoneSDFVolume, shared by the visualization and by
// FRopeSDFCollider::Query. They have no UObject dependency and can be unit tested. Every coordinate
// is in bone-local space.

#pragma once

#include "CoreMinimal.h"

struct FRopeBoneSDFVolume;

namespace RopeSDFSampler
{
	/** The trilinearly interpolated signed distance at a bone-local position (cm, positive outside).
	 *  0 when the volume is not baked. */
	DYNAMICROPE_API float SampleTrilinear(const FRopeBoneSDFVolume& Volume, const FVector& LocalPos);

	/** The normalized central-difference gradient, which is the outward direction and the normal Query
	 *  returns. Falls back to +Z when degenerate. */
	DYNAMICROPE_API FVector SampleGradient(const FRopeBoneSDFVolume& Volume, const FVector& LocalPos);

	/** The boundary-aware gradient used for projection: at the edge of the local bounds it uses
	 *  whichever difference is available. */
	DYNAMICROPE_API FVector SampleProjectionGradient(const FRopeBoneSDFVolume& Volume, const FVector& LocalPos);
}
