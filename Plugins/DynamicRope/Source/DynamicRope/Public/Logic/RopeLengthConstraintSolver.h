// Copyright 2026 TeamKeno. All Rights Reserved.
//
// UObject-free unilateral material-length constraint math.  Elasticity is compliance
// of this constraint; tension is the reaction impulse produced by the same solve.

#pragma once

#include "CoreMinimal.h"

namespace RopeLengthConstraint
{
	struct FInput
	{
		/** C = required length - material length (cm). C <= 0 is feasible. */
		float Violation = 0.0f;

		/** Cdot (cm/s), positive when the endpoints attempt to separate. */
		float SeparatingSpeed = 0.0f;

		/** J M^-1 J^T (1/kg). For two translational endpoints this is wA + wB. */
		float EffectiveInverseMass = 0.0f;

		/** Numerical activation band only; it never enlarges MaterialLength. */
		float ActivationSlop = 0.0f;

		/** Fraction [0..1] of positive position error recovered this step. */
		float SettleAlpha = 1.0f;

		/** Cap for position-bias recovery only (cm/s, 0 = unlimited). */
		float MaxBiasSpeed = 0.0f;

		/**
		 * Material compliance alpha (s^2/kg = inverse spring stiffness). Zero is rigid;
		 * positive values use an implicit critically damped Kelvin-Voigt step.
		 */
		float Compliance = 0.0f;

		/** Reaction-force cap (kg*cm/s^2, 0 = unlimited). */
		float MaxTension = 0.0f;
	};

	struct FResult
	{
		bool bActive = false;
		float Lambda = 0.0f;
		float Tension = 0.0f;
		float BiasSpeed = 0.0f;
	};

	/**
	 * Solves C <= 0 as a unilateral velocity constraint.
	 *
	 * Unlike the legacy tether solve, C == 0 with positive Cdot is active: a rigid
	 * cable can have zero extension and non-zero tension. Inward motion is never
	 * opposed and a point farther inside than ActivationSlop is slack.
	 */
	DYNAMICROPE_API FResult Solve(const FInput& In, float DeltaTime);
}
