// Copyright 2026 TeamKeno. All Rights Reserved.
//
// A helper that fills an FRopeBoneSDFVolume from an analytic SDF. It builds a sphere SDF whose true distances are
// known on the spot, as the fixture for the regression tests, such as RopeSDFSamplerTests, that compare the
// sampler's trilinear and gradient accuracy against golden values; a real baked volume has no known true distance and
// cannot be verified.
// This is test-only code with no part in the runtime path. It is header-only and inline.

#pragma once

#include "CoreMinimal.h"
#include "Collision/SDF/RopeSDFData.h"

namespace RopeSDFSynthetic
{
	/**
	 * Bakes an analytic sphere SDF in bone-local space, where the distance is the length of P minus the centre, less
	 * the radius, giving positive outside and negative inside.
	 * The samples sit on the grid nodes, evenly dividing the minimum to the maximum inclusive, per the RopeSDFSampler convention.
	 */
	inline FRopeBoneSDFVolume MakeSphere(FName Bone, const FVector& Center, float Radius,
		const FIntVector& Resolution, float Padding = 8.0f)
	{
		FRopeBoneSDFVolume V;
		V.Bone = Bone;
		V.Resolution = FIntVector(
			FMath::Max(2, Resolution.X), FMath::Max(2, Resolution.Y), FMath::Max(2, Resolution.Z));

		const float R = Radius + Padding;
		V.LocalBounds = FBox(Center - FVector(R), Center + FVector(R));

		const FVector Min = V.LocalBounds.Min;
		const FVector Size = V.LocalBounds.GetSize();
		V.VoxelSize = static_cast<float>(Size.X / (V.Resolution.X - 1));

		const int32 NX = V.Resolution.X;
		const int32 NY = V.Resolution.Y;
		const int32 NZ = V.Resolution.Z;
		const int32 N = NX * NY * NZ;

		// The first pass gathers the analytic distances as floats into a temporary and finds the maximum absolute
		// distance. Synthetic data is never clamped, so the quantization range, being the narrow band, is fitted to
		// the data's maximum, leaving quantization rounding as the only loss.
		TArray<float> Raw;
		Raw.SetNumUninitialized(N);
		float MaxAbs = KINDA_SMALL_NUMBER;
		for (int32 Z = 0; Z < NZ; ++Z)
		{
			for (int32 Y = 0; Y < NY; ++Y)
			{
				for (int32 X = 0; X < NX; ++X)
				{
					const FVector P(
						Min.X + Size.X * (static_cast<double>(X) / (NX - 1)),
						Min.Y + Size.Y * (static_cast<double>(Y) / (NY - 1)),
						Min.Z + Size.Z * (static_cast<double>(Z) / (NZ - 1)));
					const int32 Idx = X + Y * NX + Z * NX * NY;
					const float D = static_cast<float>((P - Center).Size()) - Radius;
					Raw[Idx] = D;
					MaxAbs = FMath::Max(MaxAbs, FMath::Abs(D));
				}
			}
		}

		// The second pass quantizes. A sphere is symmetric, so both the inner and outer bands are set to the data's
		// maximum absolute distance, giving a symmetric range.
		// Unsigned 8-bit is enough for synthetic test data; this is the default but is stated explicitly.
		V.NarrowBandInner = MaxAbs;
		V.NarrowBandOuter = MaxAbs;
		V.QuantBits = ERopeSDFQuantBits::UInt8;
		const int32 Bpc = V.BytesPerCode();
		V.Distances.SetNumUninitialized(N * Bpc);
		for (int32 i = 0; i < N; ++i)
		{
			FRopeBoneSDFVolume::EncodeInto(V.Distances, i, Raw[i], V.NarrowBandInner, V.NarrowBandOuter, Bpc);
		}
		return V;
	}
}
