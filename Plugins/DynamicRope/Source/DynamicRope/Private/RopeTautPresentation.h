// Copyright Epic Games, Inc. All Rights Reserved.
//
// Presentation-only shaping of the rendered centerline while a wrapped hold is taut. The simulation is
// never touched: the component applies this to the copy of the positions it sends to the renderer, so
// gameplay, tension and the tether read the solved pose while the tube draws the shaped one.
//
// Why it exists: a taut leg solved by XPBD keeps a few centimetres of sag and micro-jitter, and on
// camera that residue reads as "the rope is elastic", undermining the tension the gameplay is actually
// applying. Straightening the hand-side free span toward its chord, plus a short standing-wave thrum
// when the chain first snaps taut, is what makes the hold look loaded.

#pragma once

#include "CoreMinimal.h"

namespace RopeTautPresentation
{
	/** First-mode standing wave rate for the snap-taut thrum. Chosen for legibility on 60 fps footage
	 *  rather than physical accuracy — a real steel-cable thrum is far faster and would alias. */
	inline constexpr float ThrumFrequencyHz = 12.0f;

	/** Exponential decay of the thrum amplitude. About one order of magnitude in half a second. */
	inline constexpr float ThrumDecayPerSecond = 4.5f;

	/**
	 * Corner guard. Straightening blends nodes toward the hand→first-anchor chord, which is only honest
	 * while the span actually is nearly straight. When the leg bends over a live pivot — a ledge, a
	 * pulley — the chord cuts through the corner geometry, so the effect fades out between these two
	 * deviation measurements (max node distance off the chord, cm) and a bent span renders untouched.
	 */
	inline constexpr float StraightenDeviationFadeStartCm = 10.0f;
	inline constexpr float StraightenDeviationFadeEndCm = 25.0f;

	struct FParams
	{
		/** Where the shaped span begins. 0 is the hand; a hang grip pin moves it up to the pinned node,
		 *  because the rope below a gripping hand drapes rather than straightens, and a chord anchored
		 *  at the lower hand would pull the drawn rope off the pinned one. */
		int32 StartNode = 0;

		/** The first wrapped node; the shaped span is the open interval (StartNode, EndNode). */
		int32 EndNode = INDEX_NONE;

		/** 0..1 blend of the span's interior nodes toward the chord. */
		float Straighten = 0.0f;

		/** This frame's signed thrum displacement at the antinode, cm. The caller owns the oscillator
		 *  (amplitude decay and phase); this stays a pure function of its inputs. */
		float ThrumOffset = 0.0f;
	};

	/**
	 * Shapes Points in place (any single space — the caller passes world positions) and reports whether
	 * anything moved. Interior nodes are redistributed along the chord by index fraction, which is exact
	 * for the uniform segment lengths the sim maintains, and the thrum is a first-mode standing wave:
	 * both ends pinned, antinode in the middle, perpendicular to the chord.
	 */
	inline bool Apply(TArrayView<FVector> Points, const FParams& Params)
	{
		if (Params.StartNode < 0 || Params.EndNode - Params.StartNode < 2
			|| Params.EndNode >= Points.Num())
		{
			return false;
		}
		const FVector P0 = Points[Params.StartNode];
		const FVector Chord = Points[Params.EndNode] - P0;
		const float ChordLen = Chord.Size();
		if (ChordLen <= UE_SMALL_NUMBER)
		{
			return false;
		}
		const FVector ChordDir = Chord / ChordLen;

		float MaxDeviationSq = 0.0f;
		for (int32 Index = Params.StartNode + 1; Index < Params.EndNode; ++Index)
		{
			const FVector ToNode = Points[Index] - P0;
			const FVector OffChord = ToNode - ChordDir * (ToNode | ChordDir);
			MaxDeviationSq = FMath::Max(MaxDeviationSq, OffChord.SizeSquared());
		}
		const float CornerFade = 1.0f - FMath::Clamp(
			(FMath::Sqrt(MaxDeviationSq) - StraightenDeviationFadeStartCm) /
				(StraightenDeviationFadeEndCm - StraightenDeviationFadeStartCm),
			0.0f, 1.0f);
		const float Straighten = FMath::Clamp(Params.Straighten, 0.0f, 1.0f) * CornerFade;
		const float Thrum = Params.ThrumOffset * CornerFade;
		if (Straighten <= UE_KINDA_SMALL_NUMBER && FMath::Abs(Thrum) <= UE_KINDA_SMALL_NUMBER)
		{
			return false;
		}

		FVector Perp = FVector::CrossProduct(ChordDir, FVector::UpVector);
		if (!Perp.Normalize())
		{
			// A vertical chord: any horizontal direction is as good as another.
			Perp = FVector::CrossProduct(ChordDir, FVector::ForwardVector);
			Perp.Normalize();
		}

		for (int32 Index = Params.StartNode + 1; Index < Params.EndNode; ++Index)
		{
			const float Frac = static_cast<float>(Index - Params.StartNode)
				/ static_cast<float>(Params.EndNode - Params.StartNode);
			FVector& Point = Points[Index];
			Point = FMath::Lerp(Point, P0 + Chord * Frac, Straighten);
			Point += Perp * (Thrum * FMath::Sin(UE_PI * Frac));
		}
		return true;
	}
}
