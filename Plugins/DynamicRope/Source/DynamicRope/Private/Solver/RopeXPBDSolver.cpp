// Copyright Epic Games, Inc. All Rights Reserved.

#include "Solver/RopeXPBDSolver.h"
#include "Collision/RopeCollider.h"

void FRopeXPBDSolver::Step(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, float DeltaSeconds) const
{
	if (State.Num() < 2)
	{
		return;
	}

	const int32 Sub = FMath::Clamp(Config.Substeps, 1, 16);
	const float SubDt = FMath::Min(DeltaSeconds, 1.0f / 30.0f) / static_cast<float>(Sub);
	const int32 Iters = FMath::Max(1, Config.Iterations);

	for (int32 s = 0; s < Sub; ++s)
	{
		Integrate(State, Config, SubDt);

		for (int32 It = 0; It < Iters; ++It)
		{
			// Alternate sweep direction to remove Gauss-Seidel bias.
			const bool bReverse = (It & 1) != 0;
			SolveDistance(State, Config, SubDt, bReverse);
			SolveBending(State, Config, SubDt, bReverse);
			SolveCollisions(State, Colliders);
		}
	}
}

void FRopeXPBDSolver::Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const
{
	const float Damp = 1.0f - FMath::Clamp(Config.Damping, 0.0f, 1.0f);
	const float Dt2 = SubDt * SubDt;

	for (int32 i = 0; i < State.Num(); ++i)
	{
		if (State.InvMass[i] <= 0.0f)
		{
			continue;
		}
		const FVector Velocity = (State.Positions[i] - State.PrevPositions[i]) * Damp;
		const FVector NewPos = State.Positions[i] + Velocity + Config.Gravity * Dt2;
		State.PrevPositions[i] = State.Positions[i];
		State.Positions[i] = NewPos;
	}
}

void FRopeXPBDSolver::SolveDistance(FRopeSimState& State, const FRopeSolverConfig& /*Config*/, float /*SubDt*/, bool bReverse) const
{
	// TODO(XPBD): use StretchCompliance + per-constraint Lagrange multiplier (α/dt²) for
	// iteration/timestep-independent stiffness. Scaffold uses plain PBD projection.
	const int32 Count = State.Num() - 1;
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		const float WA = State.InvMass[i];
		const float WB = State.InvMass[i + 1];
		const float WSum = WA + WB;
		if (WSum <= 0.0f)
		{
			continue;
		}

		const FVector Delta = State.Positions[i + 1] - State.Positions[i];
		const float Dist = Delta.Size();
		if (Dist <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const FVector Correction = Delta * ((Dist - State.SegmentLength) / Dist);
		State.Positions[i] += Correction * (WA / WSum);
		State.Positions[i + 1] -= Correction * (WB / WSum);
	}
}

void FRopeXPBDSolver::SolveBending(FRopeSimState& /*State*/, const FRopeSolverConfig& /*Config*/, float /*SubDt*/, bool /*bReverse*/) const
{
	// TODO(M1): XPBD bending (or i<->i+2 support stick) using BendCompliance.
}

void FRopeXPBDSolver::SolveCollisions(FRopeSimState& State, const TArray<IRopeCollider*>& Colliders) const
{
	if (Colliders.Num() == 0)
	{
		return;
	}

	for (int32 i = 0; i < State.Num(); ++i)
	{
		if (State.InvMass[i] <= 0.0f)
		{
			continue;
		}
		for (const IRopeCollider* Collider : Colliders)
		{
			if (!Collider)
			{
				continue;
			}
			const FRopeContact Contact = Collider->Query(State.Positions[i], 0.0f);
			if (Contact.bHit)
			{
				State.Positions[i] += Contact.Normal * Contact.Penetration;
			}
		}
	}
}
