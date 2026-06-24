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

	// 제약별 Lagrange multiplier(XPBD). substep마다 리셋되며, 해당 iteration들에 걸쳐 누적된다.
	const int32 NumDist = State.Num() - 1;
	const int32 NumBend = FMath::Max(0, State.Num() - 2);
	TArray<float> LambdaDist;
	TArray<float> LambdaBend;
	LambdaDist.SetNumZeroed(NumDist);
	LambdaBend.SetNumZeroed(NumBend);

	for (int32 s = 0; s < Sub; ++s)
	{
		Integrate(State, Config, SubDt);

		// 이번 substep에서 고정된 시작점을 보간된 target까지 sweep한다(anchor가 점프할 때의 explosion 방지).
		// pin에서 velocity를 0으로 두어 motion을 주입하지 않도록 한다.
		if (State.bStartPinned && State.Num() > 0)
		{
			const float Alpha = static_cast<float>(s + 1) / static_cast<float>(Sub);
			const FVector Pin = FMath::Lerp(State.StartPinPrev, State.StartPinTarget, Alpha);
			State.Positions[0] = Pin;
			State.PrevPositions[0] = Pin;
			State.InvMass[0] = 0.0f;
		}

		// XPBD: lambda는 substep 내에서 누적되므로, 이번 substep의 iteration 전에 0으로 초기화한다.
		for (float& L : LambdaDist) { L = 0.0f; }
		for (float& L : LambdaBend) { L = 0.0f; }

		for (int32 It = 0; It < Iters; ++It)
		{
			// Gauss-Seidel bias를 제거하기 위해 sweep 방향을 번갈아 바꾼다.
			const bool bReverse = (It & 1) != 0;
			SolveDistance(State, Config, SubDt, bReverse, LambdaDist);
			SolveBending(State, Config, SubDt, bReverse, LambdaBend);
			SolveCollisions(State, Colliders);
		}
	}
}

void FRopeXPBDSolver::Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const
{
	const float Damp = 1.0f - FMath::Clamp(Config.Damping, 0.0f, 1.0f);
	const float Dt2 = SubDt * SubDt;
	// substep당 변위를 제한하여 chain이 절대 발산/explode하지 않도록 한다.
	const float MaxStep = FMath::Max(State.SegmentLength * 2.0f, 1.0f);
	const float MaxStepSq = MaxStep * MaxStep;

	for (int32 i = 0; i < State.Num(); ++i)
	{
		if (State.InvMass[i] <= 0.0f)
		{
			continue;
		}
		FVector Velocity = (State.Positions[i] - State.PrevPositions[i]) * Damp;
		if (Velocity.SizeSquared() > MaxStepSq)
		{
			Velocity = Velocity.GetSafeNormal() * MaxStep;
		}
		const FVector NewPos = State.Positions[i] + Velocity + Config.Gravity * Dt2;
		State.PrevPositions[i] = State.Positions[i];
		State.Positions[i] = NewPos;
	}
}

void FRopeXPBDSolver::SolveDistance(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
	TArray<float>& Lambda) const
{
	// XPBD distance 제약 C = |x_{i+1} - x_i| - L 을 compliant Lagrange multiplier로 푼다.
	// alpha_tilde = compliance / dt^2 (0 => rigid PBD). dLambda = (-C - alpha_tilde*Lambda) / (wA+wB+alpha_tilde).
	const float AlphaTilde = (SubDt > KINDA_SMALL_NUMBER) ? (Config.StretchCompliance / (SubDt * SubDt)) : 0.0f;
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

		const FVector N = Delta / Dist;
		const float C = Dist - State.SegmentLength;
		const float DLambda = (-C - AlphaTilde * Lambda[i]) / (WSum + AlphaTilde);
		Lambda[i] += DLambda;

		// grad_i = -N, grad_{i+1} = +N.
		State.Positions[i]     -= N * (WA * DLambda);
		State.Positions[i + 1] += N * (WB * DLambda);
	}
}

void FRopeXPBDSolver::SolveBending(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
	TArray<float>& Lambda) const
{
	// Support-stick bending: rest 길이가 2*SegmentLength인, i..i+2 구간에 걸친 XPBD distance 제약.
	// 곧게 펴지면 => C=0; 접히면 span이 짧아져 => C<0 => 제약이 양 끝을 서로 밀어내어
	// (펴주며), BendCompliance에 따라 부드럽게 작용한다. 1D chain에 대해 저렴하고 안정적이다.
	const int32 Count = State.Num() - 2;
	if (Count <= 0)
	{
		return;
	}
	const float AlphaTilde = (SubDt > KINDA_SMALL_NUMBER) ? (Config.BendCompliance / (SubDt * SubDt)) : 0.0f;
	const float Rest = 2.0f * State.SegmentLength;
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		const float WA = State.InvMass[i];
		const float WB = State.InvMass[i + 2];
		const float WSum = WA + WB;
		if (WSum <= 0.0f)
		{
			continue;
		}

		const FVector Delta = State.Positions[i + 2] - State.Positions[i];
		const float Dist = Delta.Size();
		if (Dist <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const FVector N = Delta / Dist;
		const float C = Dist - Rest;
		const float DLambda = (-C - AlphaTilde * Lambda[i]) / (WSum + AlphaTilde);
		Lambda[i] += DLambda;

		State.Positions[i]     -= N * (WA * DLambda);
		State.Positions[i + 2] += N * (WB * DLambda);
	}
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
