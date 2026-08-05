// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Collision/SDF/RopeSDFSampler.h"
#include "Collision/SDF/RopeSDFData.h"

namespace
{
	// The sampling convention: the grid nodes divide the local bounds evenly from the minimum to the maximum
	// inclusive. Node i has a normalized coordinate of i/(Res-1), so the first node is the minimum and the last is
	// the maximum. The baker must follow the same convention.

	/**
	 * A view holding a single volume's decode and indexing constants, extracted in advance.
	 *
	 * Previously every tap had FRopeBoneSDFVolume::DecodeDistance recompute the quantization range, branch on the
	 * bytes per code, divide by the range and the maximum code, and bounds-check the distance array again. One
	 * trilinear sample is eight taps and SampleGradient calls trilinear four times, giving forty taps per contact,
	 * all of them the same computation on the same volume. Those constants are built once on entry and reused.
	 *
	 * Not a single bit of the value changes: the division of the range by the maximum code is deterministic and
	 * gives the same float whether computed once or eight times, and writing a subtraction as an addition of the
	 * negation is identical under IEEE 754.
	 * The bounds check can be omitted because IsBaked() guarantees the distance count equals the product of the three
	 * resolutions and the bytes per code, so an index clamped to the range of each axis is always valid. Every public
	 * entry point checks IsBaked() first.
	 */
	struct FVolumeReader
	{
		const uint8* Data = nullptr;
		int32 ResX = 0;
		int32 ResY = 0;
		int32 ResZ = 0;
		/** The indexing strides, being the X resolution and the product of the X and Y resolutions, extracted to avoid repeating the multiplication per tap. */
		int32 StrideY = 0;
		int32 StrideZ = 0;
		int32 Bpc = 1;
		/** The linear restoration from a code to centimetres, being the code times the scale plus the bias, which is the same expression as DecodeDistance. */
		float DecodeScale = 0.0f;
		float DecodeBias = 0.0f;

		explicit FVolumeReader(const FRopeBoneSDFVolume& V)
			: Data(V.Distances.GetData())
			, ResX(V.Resolution.X)
			, ResY(V.Resolution.Y)
			, ResZ(V.Resolution.Z)
			, StrideY(V.Resolution.X)
			, StrideZ(V.Resolution.X * V.Resolution.Y)
			, Bpc(V.BytesPerCode())
			, DecodeBias(-V.NarrowBandInner)
		{
			const float MaxCodeF = (Bpc >= 2) ? 65535.0f : 255.0f;
			DecodeScale = V.QuantRange() / MaxCodeF;
		}

		/** A voxel index to centimetres. It assumes the index is valid, being baked and clamped per axis, so there is no bounds check. */
		FORCEINLINE float Decode(int32 Index) const
		{
			const int32 Base = Index * Bpc;
			uint32 Code = Data[Base];
			if (Bpc >= 2)
			{
				// Little-endian.
				Code |= static_cast<uint32>(Data[Base + 1]) << 8;
			}
			return static_cast<float>(Code) * DecodeScale + DecodeBias;
		}

		/** Reads a coordinate outside the grid by clamping it to the boundary face, on the path taken by taps landing exactly on the grid's maximum face. */
		FORCEINLINE float FetchClamped(int32 X, int32 Y, int32 Z) const
		{
			X = FMath::Clamp(X, 0, ResX - 1);
			Y = FMath::Clamp(Y, 0, ResY - 1);
			Z = FMath::Clamp(Z, 0, ResZ - 1);
			return Decode(X + Y * StrideY + Z * StrideZ);
		}
	};

	FORCEINLINE double GridCoord(double P, double Mn, double Size, int32 Res)
	{
		if (Size <= KINDA_SMALL_NUMBER || Res < 2)
		{
			return 0.0;
		}
		const double T = FMath::Clamp((P - Mn) / Size, 0.0, 1.0);
		return T * (Res - 1);
	}

	/** The body of the trilinear sample once a reader has been built, reused by callers such as SampleGradient that probe one volume several times. */
	float SampleTrilinearWith(const FVolumeReader& R, const FRopeBoneSDFVolume& V, const FVector& LocalPos)
	{
		const FVector Min = V.LocalBounds.Min;
		const FVector Size = V.LocalBounds.GetSize();

		const double Gx = GridCoord(LocalPos.X, Min.X, Size.X, R.ResX);
		const double Gy = GridCoord(LocalPos.Y, Min.Y, Size.Y, R.ResY);
		const double Gz = GridCoord(LocalPos.Z, Min.Z, Size.Z, R.ResZ);

		const int32 X0 = FMath::FloorToInt(Gx);
		const int32 Y0 = FMath::FloorToInt(Gy);
		const int32 Z0 = FMath::FloorToInt(Gz);
		const float Fx = static_cast<float>(Gx - X0);
		const float Fy = static_cast<float>(Gy - Y0);
		const float Fz = static_cast<float>(Gz - Z0);

		float C000, C100, C010, C110, C001, C101, C011, C111;
		// The fast path: when the cell is inside the grid a tap one further along cannot leave the range, so the
		// per-axis clamp, eight taps across three axes, is skipped entirely and the corner index is addressed
		// directly through the stride offsets. The clamp actually bites only on the grid's maximum face, where the
		// grid coordinate comes out at exactly one less than the resolution, so the slow path below is rarely taken.
		if (X0 + 1 < R.ResX && Y0 + 1 < R.ResY && Z0 + 1 < R.ResZ && X0 >= 0 && Y0 >= 0 && Z0 >= 0)
		{
			const int32 I = X0 + Y0 * R.StrideY + Z0 * R.StrideZ;
			C000 = R.Decode(I);
			C100 = R.Decode(I + 1);
			C010 = R.Decode(I + R.StrideY);
			C110 = R.Decode(I + R.StrideY + 1);
			C001 = R.Decode(I + R.StrideZ);
			C101 = R.Decode(I + R.StrideZ + 1);
			C011 = R.Decode(I + R.StrideZ + R.StrideY);
			C111 = R.Decode(I + R.StrideZ + R.StrideY + 1);
		}
		else
		{
			C000 = R.FetchClamped(X0,     Y0,     Z0);
			C100 = R.FetchClamped(X0 + 1, Y0,     Z0);
			C010 = R.FetchClamped(X0,     Y0 + 1, Z0);
			C110 = R.FetchClamped(X0 + 1, Y0 + 1, Z0);
			C001 = R.FetchClamped(X0,     Y0,     Z0 + 1);
			C101 = R.FetchClamped(X0 + 1, Y0,     Z0 + 1);
			C011 = R.FetchClamped(X0,     Y0 + 1, Z0 + 1);
			C111 = R.FetchClamped(X0 + 1, Y0 + 1, Z0 + 1);
		}

		const float X00 = FMath::Lerp(C000, C100, Fx);
		const float X10 = FMath::Lerp(C010, C110, Fx);
		const float X01 = FMath::Lerp(C001, C101, Fx);
		const float X11 = FMath::Lerp(C011, C111, Fx);
		const float Y0V = FMath::Lerp(X00, X10, Fy);
		const float Y1V = FMath::Lerp(X01, X11, Fy);
		return FMath::Lerp(Y0V, Y1V, Fz);
	}
}

float RopeSDFSampler::SampleTrilinear(const FRopeBoneSDFVolume& V, const FVector& LocalPos)
{
	if (!V.IsBaked())
	{
		return 0.0f;
	}
	return SampleTrilinearWith(FVolumeReader(V), V, LocalPos);
}

FVector RopeSDFSampler::SampleGradient(const FRopeBoneSDFVolume& V, const FVector& LocalPos)
{
	if (!V.IsBaked())
	{
		return FVector::UpVector;
	}

	const FVector Size = V.LocalBounds.GetSize();
	const double Hx = (V.Resolution.X > 1) ? (Size.X / (V.Resolution.X - 1)) : 1.0;
	const double Hy = (V.Resolution.Y > 1) ? (Size.Y / (V.Resolution.Y - 1)) : 1.0;
	const double Hz = (V.Resolution.Z > 1) ? (Size.Z / (V.Resolution.Z - 1)) : 1.0;

	// A forward difference, being the centre plus one sample per axis, giving four, which is two trilinear samples
	// fewer than the previous central difference of six and therefore cheaper on the hot path. The interior of an SDF
	// is monotonic enough for the push-out normal direction, at a slight cost in symmetric accuracy.
	// However, when the probe at plus H falls outside the grid's maximum boundary, the trilinear sample clamps to the
	// boundary face and that axis's difference degenerates to zero, losing the normal's component along that axis and
	// leaving the normal along the tangent. To stop nodes being pushed sideways outside a cut face, meaning a bone
	// seam, that axis alone is replaced with a backward difference; interior points keep the forward difference and
	// the hot path stays at four samples.
	// The reader is shared by all four trilinear samples, since the volume, and therefore the decode constants, is the same.
	const FVolumeReader R(V);
	const float C = SampleTrilinearWith(R, V, LocalPos);
	const auto AxisDeriv = [&R, &V, &LocalPos, C](int32 Axis, double H) -> float
	{
		FVector Pp = LocalPos;
		Pp[Axis] += H;
		if (Pp[Axis] <= V.LocalBounds.Max[Axis])
		{
			return static_cast<float>((SampleTrilinearWith(R, V, Pp) - C) / H);
		}
		FVector Pm = LocalPos;
		Pm[Axis] -= H;
		return static_cast<float>((C - SampleTrilinearWith(R, V, Pm)) / H);
	};

	const FVector Grad(AxisDeriv(0, Hx), AxisDeriv(1, Hy), AxisDeriv(2, Hz));
	const FVector N = Grad.GetSafeNormal();
	return N.IsNearlyZero() ? FVector::UpVector : N;
}

FVector RopeSDFSampler::SampleProjectionGradient(const FRopeBoneSDFVolume& V, const FVector& LocalPos)
{
	if (!V.IsBaked())
	{
		return FVector::ZeroVector;
	}

	const FVector Size = V.LocalBounds.GetSize();
	const FVector H(
		(V.Resolution.X > 1) ? (Size.X / (V.Resolution.X - 1)) : 1.0,
		(V.Resolution.Y > 1) ? (Size.Y / (V.Resolution.Y - 1)) : 1.0,
		(V.Resolution.Z > 1) ? (Size.Z / (V.Resolution.Z - 1)) : 1.0);

	// All six trilinear samples share a single reader.
	const FVolumeReader R(V);
	auto AxisDerivative = [&](int32 Axis, float Step)
	{
		FVector Minus = LocalPos;
		FVector Plus = LocalPos;
		Minus[Axis] = FMath::Max(LocalPos[Axis] - Step, V.LocalBounds.Min[Axis]);
		Plus[Axis] = FMath::Min(LocalPos[Axis] + Step, V.LocalBounds.Max[Axis]);

		const float Span = static_cast<float>(Plus[Axis] - Minus[Axis]);
		if (Span <= KINDA_SMALL_NUMBER)
		{
			return 0.0f;
		}

		return (SampleTrilinearWith(R, V, Plus) - SampleTrilinearWith(R, V, Minus)) / Span;
	};

	return FVector(
		AxisDerivative(0, H.X),
		AxisDerivative(1, H.Y),
		AxisDerivative(2, H.Z)).GetSafeNormal();
}
