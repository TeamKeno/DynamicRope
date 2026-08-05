// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeLengthConstraintSolver.h"

namespace RopeLengthConstraint
{
	FResult Solve(const FInput& In, float DeltaTime)
	{
		FResult Out;
		if (DeltaTime <= 1e-4f)
		{
			return Out;
		}

		const float Slop = FMath::Max(In.ActivationSlop, 0.0f);
		const bool bPenetrating = In.Violation > 0.0f;
		const bool bOutwardAtBoundary =
			In.Violation >= -Slop && In.SeparatingSpeed > KINDA_SMALL_NUMBER;
		if (!bPenetrating && !bOutwardAtBoundary)
		{
			return Out;
		}

		const float EffectiveInverseMass = FMath::Max(In.EffectiveInverseMass, 0.0f);
		if (EffectiveInverseMass <= KINDA_SMALL_NUMBER)
		{
			return Out;
		}

		const float Compliance = FMath::Max(In.Compliance, 0.0f);
		if (Compliance > KINDA_SMALL_NUMBER)
		{
			// Backward-Euler Kelvin-Voigt material solve, evaluated at the post-impulse
			// C/Cdot state:
			//   s+ = s - w*lambda, C+ = max(C,0) + dt*s+
			//   lambda = dt * (k*C+ + d*s+)
			// with k=1/alpha and critical d=2*sqrt(k/w). Solving the three equations
			// together is unconditionally stable; unlike explicit T*dt, damping cannot
			// reverse an outward velocity merely because the frame is long.
			//
			// This alpha-scaled form avoids very large k at near-rigid compliance:
			//   r = alpha*d = 2*sqrt(alpha/w)
			//   T = [max(C,0) + (r+dt)*s] /
			//       [alpha + w*dt*(r+dt)]
			const float DampingCompliance =
				2.0f * FMath::Sqrt(Compliance / EffectiveInverseMass);
			const float StepResponse = DampingCompliance + DeltaTime;
			const float Denominator =
				Compliance +
				EffectiveInverseMass * DeltaTime * StepResponse;
			if (Denominator <= SMALL_NUMBER)
			{
				return Out;
			}
			Out.Tension = FMath::Max(
				0.0f,
				(FMath::Max(In.Violation, 0.0f) +
					StepResponse * In.SeparatingSpeed) /
					Denominator);
			if (In.MaxTension > 0.0f)
			{
				Out.Tension = FMath::Min(
					Out.Tension, In.MaxTension);
			}
			Out.Lambda = Out.Tension * DeltaTime;
			Out.bActive = Out.Lambda > KINDA_SMALL_NUMBER;
			if (!Out.bActive)
			{
				Out.Tension = 0.0f;
			}
			return Out;
		}

		const float Beta = FMath::Clamp(In.SettleAlpha, 0.0f, 1.0f);
		Out.BiasSpeed = Beta * FMath::Max(In.Violation, 0.0f) / DeltaTime;
		if (In.MaxBiasSpeed > 0.0f)
		{
			Out.BiasSpeed = FMath::Min(Out.BiasSpeed, In.MaxBiasSpeed);
		}

		// New Cdot after an inward reaction is Cdot' = Cdot - lambda * J M^-1 J^T.
		// Require Cdot' <= -BiasSpeed. A negative numerator means it is already
		// approaching quickly enough, so a unilateral rope must do nothing.
		const float RequiredSpeedChange = In.SeparatingSpeed + Out.BiasSpeed;
		if (RequiredSpeedChange <= 0.0f)
		{
			return Out;
		}

		Out.Lambda = RequiredSpeedChange / EffectiveInverseMass;
		if (In.MaxTension > 0.0f)
		{
			Out.Lambda = FMath::Min(Out.Lambda, In.MaxTension * DeltaTime);
		}
		Out.bActive = Out.Lambda > KINDA_SMALL_NUMBER;
		Out.Tension = Out.bActive ? Out.Lambda / DeltaTime : 0.0f;
		return Out;
	}
}
