// Copyright Epic Games, Inc. All Rights Reserved.
//
// Shared mathematics and utility helpers internal to the module. If several .cpp files each defined the same helper
// in an anonymous namespace, a unity build, which merges several .cpp files into one translation unit, would merge
// those anonymous namespaces and produce a duplicate definition error, so helpers that need to be shared live here as
// inline functions and are used through RopeMath::.

#pragma once

#include "CoreMinimal.h"

namespace RopeMath
{
	/** A smooth interpolation weight over the range zero to one, being 3t^2 - 2t^3. */
	inline float SmoothStep(float T)
	{
		T = FMath::Clamp(T, 0.0f, 1.0f);
		return T * T * (3.0f - 2.0f * T);
	}

	/** The parameter of the point on the segment nearest to P, clamped to the range zero to one. Used for such things as identifying a capsule contact's material point. */
	inline float ClosestSegmentParam(const FVector& P, const FVector& SegA, const FVector& SegB)
	{
		const FVector Seg = SegB - SegA;
		const float SegSq = static_cast<float>(Seg.SizeSquared());
		return (SegSq > KINDA_SMALL_NUMBER) ? FMath::Clamp(static_cast<float>((P - SegA) | Seg) / SegSq, 0.0f, 1.0f) : 0.0f;
	}

	/** An arbitrary but stable tangent perpendicular to the normal, with a built-in fallback for the degenerate case. */
	inline FVector AnyTangentFromNormal(const FVector& Normal)
	{
		const FVector N = Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		const FVector Reference = FMath::Abs(FVector::DotProduct(N, FVector::UpVector)) < 0.9f
			? FVector::UpVector
			: FVector::RightVector;
		return FVector::CrossProduct(Reference, N).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	}

	/** Normalizes the value, returning the normalized fallback if it is degenerate, meaning a zero vector. This has the same meaning as FRopeWhipGuide::SafeNormalOr. */
	inline FVector SafeNormalOr(const FVector& Value, const FVector& Fallback)
	{
		const FVector Normalized = Value.GetSafeNormal();
		return Normalized.IsNearlyZero() ? Fallback.GetSafeNormal() : Normalized;
	}

	/**
	 * The unit direction at Alpha along the throw preview arc, where zero is the start of the sweep, opposite the aim,
	 * and one is the aim direction.
	 * The raw aim direction and guide up vector are the arc plane's reference axes and may be neither normalized nor
	 * orthogonal, since both are handled internally.
	 */
	inline FVector ArcDirectionAtAlpha(const FVector& AimDirRaw, const FVector& GuideUpRaw,
		float SweepAngleDegrees, float Alpha)
	{
		const FVector Aim = SafeNormalOr(AimDirRaw, FVector::ForwardVector);
		FVector Up = GuideUpRaw - FVector::DotProduct(GuideUpRaw, Aim) * Aim;
		Up = SafeNormalOr(Up, FVector::UpVector);

		const float ClampedAlpha = FMath::Clamp(Alpha, 0.0f, 1.0f);
		const float SweepRadians = FMath::DegreesToRadians(FMath::Clamp(SweepAngleDegrees, 1.0f, 180.0f));
		const float Angle = SweepRadians * (1.0f - ClampedAlpha);
		return (Aim * FMath::Cos(Angle) + Up * FMath::Sin(Angle)).GetSafeNormal();
	}

	/**
	 * Builds the whip guide's time-varying straight centreline. The caller rotates SweepDirection over time;
	 * every sample remains collinear at a fixed normalized time. Throw motion inheritance belongs to the Verlet
	 * velocity injection, not the guide geometry, because distributing drift non-linearly along the rope curves
	 * an otherwise straight Flight guide.
	 */
	inline void BuildWhipGuideRawPoints(const FVector& Origin, const FVector& SweepDirection,
		const FVector& AimDirection, bool /*bHasAimTarget*/, float /*NormalizedTime*/,
		float GuideLength, const FVector& /*InheritedDrift*/, float /*AimSteerStartAlpha*/,
		float /*AimLockAlpha*/, float /*AimDirectionBias*/, int32 RequestedSampleCount, TArray<FVector>& OutPoints)
	{
		OutPoints.Reset();
		const int32 SampleCount = FMath::Max(RequestedSampleCount, 4);
		const float PathLength = FMath::Max(GuideLength, KINDA_SMALL_NUMBER);
		const FVector SweepDir = SafeNormalOr(SweepDirection, AimDirection);

		OutPoints.Reserve(SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			const float RopeAlpha = static_cast<float>(SampleIndex) / static_cast<float>(SampleCount - 1);
			OutPoints.Add(Origin + SweepDir * (RopeAlpha * PathLength));
		}
	}

	/** The index closest to the hand, meaning node zero, among a list of node indices within the range of the positions. INDEX_NONE if there is none. */
	inline int32 HeadValidNodeIndex(const TArray<int32>& NodeIndices, const TArray<FVector>& Positions)
	{
		int32 HeadNodeIndex = INDEX_NONE;
		for (const int32 NodeIndex : NodeIndices)
		{
			if (!Positions.IsValidIndex(NodeIndex))
			{
				continue;
			}

			if (HeadNodeIndex == INDEX_NONE || NodeIndex < HeadNodeIndex)
			{
				HeadNodeIndex = NodeIndex;
			}
		}
		return HeadNodeIndex;
	}

	/** A setter for an optional failure reason out parameter, permitting null. The shared pattern across the preview APIs. */
	inline void SetPreviewFailureReason(FString* OutFailureReason, const FString& Reason)
	{
		if (OutFailureReason)
		{
			*OutFailureReason = Reason;
		}
	}
}
