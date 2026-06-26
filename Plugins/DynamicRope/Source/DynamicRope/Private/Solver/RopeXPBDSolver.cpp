// Copyright Epic Games, Inc. All Rights Reserved.

#include "Solver/RopeXPBDSolver.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
#include "ProfilingDebugging/CpuProfilerTrace.h" // TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)

FRopeSubstepSchedule RopeSolverSubsteps(FRopeSimState& State, const FRopeSolverConfig& Config, float DeltaSeconds)
{
	// 고정 timestep: substep 크기를 frame rate와 무관하게 고정한다(Substeps = "60fps frame당 substep 수"로
	// 해석). 실제 경과 시간을 누적해 고정 크기로 소비하므로 저fps면 더 많은 substep을, 고fps면 더 적은
	// substep을 돌린다 → substep당 변위가 항상 일정 → 충돌/터널링이 frame rate에 의존하지 않는다.
	const int32 SubPerRef = FMath::Clamp(Config.Substeps, 1, 16);
	const float FixedDt = (1.0f / 60.0f) / static_cast<float>(SubPerRef);
	const int32 MaxSubsteps = FMath::Clamp(SubPerRef * 2, 1, 32); // spiral-of-death 상한(과부하 시 slow-mo)

	State.TimeAccumulator += DeltaSeconds;
	const float MaxAccum = FixedDt * static_cast<float>(MaxSubsteps);
	if (State.TimeAccumulator > MaxAccum)
	{
		State.TimeAccumulator = MaxAccum; // 초과분 버림: 폭주 대신 가벼운 slow-mo
	}

	const int32 NumSub = FMath::FloorToInt(State.TimeAccumulator / FixedDt);
	if (NumSub <= 0)
	{
		return FRopeSubstepSchedule{ 0, FixedDt }; // 아직 한 substep 분량이 안 모임(고fps) → 다음 frame으로 이월
	}
	State.TimeAccumulator -= static_cast<float>(NumSub) * FixedDt;
	return FRopeSubstepSchedule{ NumSub, FixedDt };
}

void FRopeXPBDSolver::Step(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, float DeltaSeconds) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Step);
	if (State.Num() < 2)
	{
		return;
	}

	const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(State, Config, DeltaSeconds);
	const int32 NumSub = Schedule.NumSub;
	const float FixedDt = Schedule.FixedDt;
	if (NumSub <= 0)
	{
		return;
	}

	const int32 Iters = FMath::Max(1, Config.Iterations);

	// hot-path: 기본 비활성(VeryVerbose). r.LogRopeSolver를 켜야 보인다.
	UE_LOG(LogRopeSolver, VeryVerbose, TEXT("Step: %d node(s), %d substep(s) x %d iter(s), %d collider(s)"),
		State.Num(), NumSub, Iters, Colliders.Num());

	// 제약별 Lagrange multiplier(XPBD). substep마다 리셋되며, 해당 iteration들에 걸쳐 누적된다.
	const int32 NumDist = State.Num() - 1;
	const int32 NumBend = FMath::Max(0, State.Num() - 2);
	TArray<float> LambdaDist;
	TArray<float> LambdaBend;
	LambdaDist.SetNumZeroed(NumDist);
	LambdaBend.SetNumZeroed(NumBend);

	// Broad-phase: collider별 월드 AABB(+CollisionRadius)를 1회만 계산한다. SolveCollisions가
	// node/iteration/substep마다 먼 collider까지 역변환 query하던 비용을 싼 박스 테스트로 컷.
	const float CollRadius = FMath::Max(0.0f, Config.CollisionRadius);
	TArray<FBox> ColliderBounds;
	ColliderBounds.Reserve(Colliders.Num());
	for (const IRopeCollider* Collider : Colliders)
	{
		ColliderBounds.Add(Collider ? Collider->GetWorldBounds().ExpandBy(CollRadius) : FBox(ForceInit));
	}

	for (int32 s = 0; s < NumSub; ++s)
	{
		Integrate(State, Config, FixedDt);

		// 이번 frame의 substep들에 걸쳐 고정된 시작점을 target까지 sweep한다(anchor 점프 시 explosion 방지).
		// pin에서 velocity를 0으로 두어 motion을 주입하지 않도록 한다.
		if (State.bStartPinned && State.Num() > 0)
		{
			const float Alpha = static_cast<float>(s + 1) / static_cast<float>(NumSub);
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
			SolveDistance(State, Config, FixedDt, bReverse, LambdaDist);
			SolveBending(State, Config, FixedDt, bReverse, LambdaBend);
		}

		// 충돌은 substep당 1회(매 iteration이 아니라). Query가 비싸고, push-out이 침투를 한 번에
		// 해소하므로 substep 끝에서 한 번이면 충분하다(다음 substep이 재수렴). friction 과적용도 방지.
		SolveCollisions(State, Config, Colliders, ColliderBounds);
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
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Distance);
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
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Bending);
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

void FRopeXPBDSolver::SolveCollisions(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Collisions);
	if (Colliders.Num() == 0)
	{
		return;
	}

	// 로프의 충돌 두께. 0이면 노드가 표면 안에 들어가야만 hit → 얇은 limb/희소 노드에서 대부분 관통.
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const float Friction = FMath::Clamp(Config.Friction, 0.0f, 1.0f);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();

	// Swept(연속) 충돌: 노드를 점이 아니라 PrevPos->Pos 구간으로 본다. 빠른 노드가 한 substep에 얇은
	// 표면을 가로질러도(이산 점검사로는 터널링) 구간을 따라 샘플해 첫 접촉에서 멈춘다. 느린 접촉(L 작음)은
	// 샘플 1개 = 끝점만 검사하므로 추가 비용이 없다.
	const float SweepStep = FMath::Max(Config.SweepStep, 0.1f);     // 샘플 간격(cm), 디자이너 튜닝
	const int32 MaxSweepSamples = FMath::Max(1, Config.MaxSweepSamples); // 구간당 샘플 상한

	for (int32 i = 0; i < State.Num(); ++i)
	{
		if (State.InvMass[i] <= 0.0f)
		{
			continue;
		}

		const FVector A = State.PrevPositions[i]; // substep 시작 위치
		// 구간 broad-phase용 AABB. collider bounds는 이미 Radius만큼 확장돼 있다.
		FBox SweepBox(ForceInit);
		SweepBox += A;
		SweepBox += State.Positions[i];

		for (int32 c = 0; c < Colliders.Num(); ++c)
		{
			const IRopeCollider* Collider = Colliders[c];
			if (!Collider)
			{
				continue;
			}
			// Broad-phase: 구간이 collider AABB(+Radius)와 안 겹치면 스킵. 끝점만 보면 가로질러 통과한
			// 노드를 놓치므로 반드시 구간 AABB로 판단한다.
			if (bHasBounds && !ColliderBounds[c].Intersect(SweepBox))
			{
				continue;
			}

			const FVector B = State.Positions[i]; // 현재 끝점(앞선 collider가 밀었을 수 있음)
			const double L = FVector::Dist(A, B);
			const int32 NumSamples = FMath::Clamp(1 + FMath::FloorToInt(L / SweepStep), 1, MaxSweepSamples);

			for (int32 k = 0; k < NumSamples; ++k)
			{
				// A->B를 따라 첫 접촉을 찾는다. NumSamples==1이면 끝점만(느린 접촉 = 기존 동작, 추가비용 0).
				const double T = (NumSamples <= 1) ? 1.0 : static_cast<double>(k) / static_cast<double>(NumSamples - 1);
				const FVector P = FMath::Lerp(A, B, T);

				const FRopeContact Contact = Collider->Query(P, Radius);
				if (!Contact.bHit)
				{
					continue;
				}

				// 첫 접촉 지점에서 표면 밖으로 밀어 멈춘다(가로질러 통과하지 못하게).
				State.Positions[i] = P + Contact.Normal * Contact.Penetration;

				// 접선 방향 friction: 변위(Pos-Prev)의 접선 성분을 Friction만큼 깎아 그립을 만든다.
				if (Friction > 0.0f)
				{
					const FVector Delta = State.Positions[i] - State.PrevPositions[i];
					const FVector Tangent = Delta - (Delta | Contact.Normal) * Contact.Normal;
					State.PrevPositions[i] += Tangent * Friction;
				}
				break; // 이 collider에 대한 첫 접촉에서 종료
			}
		}
	}
}
