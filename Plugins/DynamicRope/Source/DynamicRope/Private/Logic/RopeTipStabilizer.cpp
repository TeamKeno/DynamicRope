// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeTipStabilizer.h"

FVector FRopeTipStabilizer::ComputeTailDirection(TConstArrayView<FVector> Positions, int32 SampleCount,
	const FVector& Fallback)
{
	const int32 N = Positions.Num();
	if (N < 2)
	{
		return Fallback;
	}

	// Weight K..1 toward the tip: the newest segment dominates, older ones only steady it.
	const int32 K = FMath::Clamp(SampleCount, 1, N - 1);
	FVector Sum = FVector::ZeroVector;
	for (int32 j = 0; j < K; ++j)
	{
		const FVector SegDir = (Positions[N - 1 - j] - Positions[N - 2 - j]).GetSafeNormal();
		Sum += SegDir * static_cast<float>(K - j);
	}

	const FVector Averaged = Sum.GetSafeNormal();
	if (!Averaged.IsNearlyZero())
	{
		return Averaged;
	}

	// A folded rope can cancel the average out; the raw last segment is still meaningful there.
	const FVector LastSegDir = (Positions[N - 1] - Positions[N - 2]).GetSafeNormal();
	return LastSegDir.IsNearlyZero() ? Fallback : LastSegDir;
}

void FRopeTipStabilizer::Reset()
{
	bHasState = false;
	Held = FSample();
	Smoothed = FSample();
	LastRawPosition = FVector::ZeroVector;
	SlowRawPosition = FVector::ZeroVector;
}

FRopeTipStabilizer::FSample FRopeTipStabilizer::Stabilize(const FSample& Raw, float DeltaTime,
	const FParams& Params)
{
	// A paused or zero-dt frame holds the last output; before any seed there is nothing to hold.
	if (DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return bHasState ? Smoothed : Raw;
	}

	// First sample after a Reset, or a teleport-sized jump: converging across it would drag the tip
	// through space, so reseed onto the input instead.
	const float RawStep = FVector::Dist(Raw.Position, LastRawPosition);
	if (!bHasState || RawStep > Params.TeleportDistance)
	{
		bHasState = true;
		Held = Raw;
		Smoothed = Raw;
		LastRawPosition = Raw.Position;
		SlowRawPosition = Raw.Position;
		return Raw;
	}
	LastRawPosition = Raw.Position;

	// Sustained speed: net travel away from a trailing average, not the per-frame delta. In-place
	// jitter spikes the per-frame delta hardest exactly when filtering matters most, which would
	// collapse the weight below and switch the filter off against itself; oscillation around a spot
	// keeps the trailing average centred, so only actual travel registers. For steady motion at v
	// the average lags by v * Tau, so distance / Tau recovers v.
	const float SpeedEstimateHalfLife = 0.06f;
	const float Tau = SpeedEstimateHalfLife / FMath::Loge(2.0f);
	const float Speed = FVector::Dist(Raw.Position, SlowRawPosition) / Tau;
	const float TrailAlpha = 1.0f - FMath::Exp2(-DeltaTime / SpeedEstimateHalfLife);
	SlowRawPosition = FMath::Lerp(SlowRawPosition, Raw.Position, TrailAlpha);

	// One weight drives the whole filter: 1 at rest, fading to 0 (raw pass-through) as the
	// sustained speed approaches FadeOutSpeed. The fade starts at a quarter of it so slow drifts
	// stay filtered.
	const float FadeStart = 0.25f * Params.FadeOutSpeed;
	const float FadeEnd = FMath::Max(Params.FadeOutSpeed, FadeStart + KINDA_SMALL_NUMBER);
	const float StabilizeWeight = 1.0f - FMath::SmoothStep(FadeStart, FadeEnd, Speed);

	// Deadband latches, one per channel. Comparing against the latch rather than the last output is
	// the hysteresis: noise that stays inside the band can never accumulate into visible motion.
	// The channels latch independently — position creep must not unlatch a steady direction, or the
	// angle band would be dead whenever the rope drifts.
	const float PosDeadband = Params.PositionDeadband * StabilizeWeight;
	const float AngDeadbandRad = FMath::DegreesToRadians(Params.AngleDeadbandDeg) * StabilizeWeight;
	const float PosDelta = FVector::Dist(Raw.Position, Held.Position);
	const float AngDelta = FMath::Acos(FMath::Clamp(
		static_cast<float>(FVector::DotProduct(Raw.Direction, Held.Direction)), -1.0f, 1.0f));
	if (PosDelta > PosDeadband)
	{
		Held.Position = Raw.Position;
	}
	if (AngDelta > AngDeadbandRad)
	{
		Held.Direction = Raw.Direction;
	}

	// Converge toward the latch: exponential with the configured half-life, blended toward an
	// instant snap as the stabilization fades out with speed.
	float Alpha = 1.0f;
	if (Params.SmoothingHalfLife > KINDA_SMALL_NUMBER)
	{
		const float HalfLifeAlpha = 1.0f - FMath::Exp2(-DeltaTime / Params.SmoothingHalfLife);
		Alpha = FMath::Lerp(1.0f, HalfLifeAlpha, StabilizeWeight);
	}
	Smoothed.Position = FMath::Lerp(Smoothed.Position, Held.Position, Alpha);
	const FQuat FullTurn = FQuat::FindBetweenNormals(Smoothed.Direction, Held.Direction);
	const FQuat PartialTurn = FQuat::Slerp(FQuat::Identity, FullTurn, Alpha);
	Smoothed.Direction = PartialTurn.RotateVector(Smoothed.Direction)
		.GetSafeNormal(KINDA_SMALL_NUMBER, Held.Direction);
	return Smoothed;
}
