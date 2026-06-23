// Copyright Epic Games, Inc. All Rights Reserved.
//
// Position-based (XPBD) rope solver. Operates purely on FRopeSimState with no UObject deps,
// so it is unit-testable and portable to a compute shader later. Runs only in Flight/Contacting.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class IRopeCollider;

class DYNAMICROPE_API FRopeXPBDSolver
{
public:
	/** Advance one frame: substepped integrate + distance/bending/collision constraints. */
	void Step(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, float DeltaSeconds) const;

private:
	void Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const;

	// XPBD distance: enforces segment length with StretchCompliance. Lambda accumulates across the
	// substep's iterations (one entry per segment constraint), making stiffness step/iter-independent.
	void SolveDistance(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	// XPBD bending: i<->i+2 "support stick" (rest = 2*SegmentLength) with BendCompliance.
	void SolveBending(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	void SolveCollisions(FRopeSimState& State, const TArray<IRopeCollider*>& Colliders) const;
};
