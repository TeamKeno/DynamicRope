// Copyright Epic Games, Inc. All Rights Reserved.

#include "SDF/RopeSDFDraw.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFSampler.h"
// ERopeSDFSliceAxis
#include "Collision/SDF/RopeSDFProvider.h"
// FPrimitiveDrawInterface, SDPG_*
#include "SceneManagement.h"

namespace
{
	// Converts normalized coordinates, each from zero to one, into a bone-local position.
	FORCEINLINE FVector LocalFromNorm(const FBox& Local, double Tx, double Ty, double Tz)
	{
		const FVector Mn = Local.Min;
		const FVector Sz = Local.GetSize();
		return FVector(Mn.X + Sz.X * Tx, Mn.Y + Sz.Y * Ty, Mn.Z + Sz.Z * Tz);
	}

	// A diverging heatmap: negative, meaning inside, is red, zero is white and positive, meaning outside, is blue, saturating at the scale in centimetres.
	FLinearColor HeatColor(float D, float Scale)
	{
		const float T = FMath::Clamp(D / FMath::Max(Scale, KINDA_SMALL_NUMBER), -1.0f, 1.0f);
		return (T >= 0.0f)
			? FMath::Lerp(FLinearColor::White, FLinearColor(0.0f, 0.4f, 1.0f), T)
			: FMath::Lerp(FLinearColor::White, FLinearColor::Red, -T);
	}

	// Whether a sample is saturated, meaning a clamped placeholder, against the volume's asymmetric band: outside at
	// or beyond the outer narrow band, or inside at or beyond the inner one. Those regions carry no real distance or
	// direction information. The test is disabled, returning false, if the band range is invalid.
	FORCEINLINE bool IsSaturatedSample(const FRopeBoneSDFVolume& V, float D)
	{
		const float NBIn = V.NarrowBandInner;
		const float NBOut = V.NarrowBandOuter;
		if (NBIn + NBOut <= 0.0f)
		{
			return false;
		}
		return (NBOut > 0.0f && D >= NBOut - KINDA_SMALL_NUMBER)
			|| (NBIn  > 0.0f && D <= -NBIn + KINDA_SMALL_NUMBER);
	}
}

void RopeSDFDraw::DrawBounds(FPrimitiveDrawInterface* PDI, const FBox& Local, const FTransform& Xform, const FLinearColor& Color)
{
	const FVector Mn = Local.Min;
	const FVector Mx = Local.Max;
	const FVector V[8] = {
		Xform.TransformPosition(FVector(Mn.X, Mn.Y, Mn.Z)),
		Xform.TransformPosition(FVector(Mx.X, Mn.Y, Mn.Z)),
		Xform.TransformPosition(FVector(Mx.X, Mx.Y, Mn.Z)),
		Xform.TransformPosition(FVector(Mn.X, Mx.Y, Mn.Z)),
		Xform.TransformPosition(FVector(Mn.X, Mn.Y, Mx.Z)),
		Xform.TransformPosition(FVector(Mx.X, Mn.Y, Mx.Z)),
		Xform.TransformPosition(FVector(Mx.X, Mx.Y, Mx.Z)),
		Xform.TransformPosition(FVector(Mn.X, Mx.Y, Mx.Z)),
	};
	static const int32 Edges[12][2] = {
		{0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4}, {0,4},{1,5},{2,6},{3,7} };
	for (const auto& E : Edges)
	{
		PDI->DrawLine(V[E[0]], V[E[1]], Color, SDPG_World, 0.5f);
	}
}

void RopeSDFDraw::DrawVoxels(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform, float Band)
{
	const FVector Mn = V.LocalBounds.Min;
	const FVector Sz = V.LocalBounds.GetSize();
	const int32 NX = V.Resolution.X;
	const int32 NY = V.Resolution.Y;
	const int32 NZ = V.Resolution.Z;
	if (NX < 2 || NY < 2 || NZ < 2)
	{
		return;
	}

	// Cap total iteration with a stride for very high resolutions.
	const int64 Total = static_cast<int64>(NX) * NY * NZ;
	int32 Stride = 1;
	while ((Total / (static_cast<int64>(Stride) * Stride * Stride)) > 200000)
	{
		++Stride;
	}

	for (int32 Z = 0; Z < NZ; Z += Stride)
	{
		for (int32 Y = 0; Y < NY; Y += Stride)
		{
			for (int32 X = 0; X < NX; X += Stride)
			{
				const int32 Idx = X + Y * NX + Z * NX * NY;
				if (!V.Distances.IsValidIndex(Idx))
				{
					continue;
				}
				const float D = V.DecodeDistance(Idx);
				if (FMath::Abs(D) > Band)
				{
					continue;
				}
				// Saturated samples, having reached the outer or inner narrow band, are clamped placeholders and are
				// skipped, which stops the entire outer plateau being pulled in at the band's maximum and filling the
				// box. On the inside the automatic band covers the interior, so only the deepest point is affected.
				// Skipping is disabled if the band is invalid.
				if (IsSaturatedSample(V, D))
				{
					continue;
				}
				const FVector L(
					Mn.X + Sz.X * (static_cast<double>(X) / (NX - 1)),
					Mn.Y + Sz.Y * (static_cast<double>(Y) / (NY - 1)),
					Mn.Z + Sz.Z * (static_cast<double>(Z) / (NZ - 1)));
				// The same convention as the slice heatmap: inside, being negative, is red, outside, being positive, is blue, and near zero is white.
				const FLinearColor C = (D < -0.01f) ? FLinearColor::Red
					: (D > 0.01f) ? FLinearColor(0.0f, 0.4f, 1.0f)
					: FLinearColor::White;
				PDI->DrawPoint(Xform.TransformPosition(L), C, 4.0f, SDPG_World);
			}
		}
	}
}

void RopeSDFDraw::DrawSlice(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform,
	ERopeSDFSliceAxis Axis, float Pos01, int32 Res, float Scale)
{
	// A dim grey for saturated samples, having reached the outer or inner narrow band, which separates the meaningless plateau visually from the meaningful band.
	static const FLinearColor SaturatedColor(0.15f, 0.15f, 0.15f);
	Res = FMath::Max(2, Res);
	for (int32 I = 0; I < Res; ++I)
	{
		const double U = static_cast<double>(I) / (Res - 1);
		for (int32 J = 0; J < Res; ++J)
		{
			const double W = static_cast<double>(J) / (Res - 1);
			double Tx, Ty, Tz;
			switch (Axis)
			{
			case ERopeSDFSliceAxis::X: Tx = Pos01; Ty = U; Tz = W; break;
			case ERopeSDFSliceAxis::Y: Tx = U; Ty = Pos01; Tz = W; break;
			// Z
			default:                   Tx = U; Ty = W; Tz = Pos01; break;
			}
			const FVector L = LocalFromNorm(V.LocalBounds, Tx, Ty, Tz);
			const float D = RopeSDFSampler::SampleTrilinear(V, L);
			// Saturated samples, at the outer or inner narrow band, are a constant plateau with no real distance
			// information, so they are drawn grey to distinguish them from the meaningful band covering the surface,
			// the continuous field and the interior. With an invalid band everything is drawn as a heatmap as before.
			const FLinearColor C = IsSaturatedSample(V, D) ? SaturatedColor : HeatColor(D, Scale);
			PDI->DrawPoint(Xform.TransformPosition(L), C, 5.0f, SDPG_World);
		}
	}
}

void RopeSDFDraw::DrawGradients(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform,
	float Band, float Length)
{
	const int32 NX = V.Resolution.X;
	const int32 NY = V.Resolution.Y;
	const int32 NZ = V.Resolution.Z;
	if (NX < 2 || NY < 2 || NZ < 2)
	{
		return;
	}
	// Strided to about six per axis, to avoid crowding.
	const int32 SX = FMath::Max(1, (NX - 1) / 6);
	const int32 SY = FMath::Max(1, (NY - 1) / 6);
	const int32 SZ = FMath::Max(1, (NZ - 1) / 6);
	for (int32 Z = 0; Z < NZ; Z += SZ)
	{
		for (int32 Y = 0; Y < NY; Y += SY)
		{
			for (int32 X = 0; X < NX; X += SX)
			{
				const int32 Idx = X + Y * NX + Z * NX * NY;
				if (!V.Distances.IsValidIndex(Idx))
				{
					continue;
				}
				const float D = V.DecodeDistance(Idx);
				if (FMath::Abs(D) > Band)
				{
					continue;
				}
				// Saturated samples, having reached the outer or inner narrow band, carry no direction information,
				// being flat, which falls back to the up vector, or noisy at the boundary, so they are skipped. With
				// an invalid band skipping is disabled and they are drawn as before.
				if (IsSaturatedSample(V, D))
				{
					continue;
				}
				const FVector L = LocalFromNorm(V.LocalBounds,
					static_cast<double>(X) / (NX - 1),
					static_cast<double>(Y) / (NY - 1),
					static_cast<double>(Z) / (NZ - 1));
				const FVector G = RopeSDFSampler::SampleGradient(V, L);
				// Drawn as an arrow with a head so the push-out direction, meaning outwards, is visible. A matrix
				// rotating the positive X axis onto the gradient direction is built with the sample position as its
				// origin, using a no-scale transform in case the scale is non-uniform.
				const FVector WorldDir = Xform.TransformVectorNoScale(G).GetSafeNormal();
				if (WorldDir.IsNearlyZero())
				{
					continue;
				}
				FMatrix ArrowToWorld = FRotationMatrix::MakeFromX(WorldDir);
				ArrowToWorld.SetOrigin(Xform.TransformPosition(L));
				DrawDirectionalArrow(PDI, ArrowToWorld, FLinearColor::Green,
					Length /*Length in centimetres*/, Length * 0.05f /*Arrowhead size*/, SDPG_World, 0.2f /*Thickness*/);
			}
		}
	}
}
