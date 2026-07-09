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

	// 노드별 캐시된 접촉 제약(substep마다 검출 → iteration마다 강제 → 끝에 마찰 1회).
	TArray<FRopeContactState> Contacts;
	Contacts.SetNum(State.Num());

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
		for (FRopeContactState& C : Contacts) { C.bActive = false; C.Lambda = 0.0f; }

		// 알파: 이 substep이 차지하는 collider 모션 구간(프레임 모션을 substep에 균등 분배).
		const float SubAlpha0 = static_cast<float>(s) / static_cast<float>(NumSub);
		const float SubAlpha1 = static_cast<float>(s + 1) / static_cast<float>(NumSub);

		// CollisionPassesPerSubstep = substep당 접촉 *재검출* 횟수(캐시 평면 갱신). 1=시작에 1회 검출(기본).
		// 검출(swept)은 비싸므로 가끔만, 그 사이 SolveContacts가 매 iteration 캐시 평면을 싸게 강제한다 → K=1에서도
		// collision이 매 iteration distance/bending과 동등하게 경쟁해 장력에 안 밀린다(관통 차단). 노드가 substep
		// 안에서 많이 움직여 평면이 낡으면 K>1로 중간 갱신.
		const int32 CollPasses = FMath::Clamp(Config.CollisionPassesPerSubstep, 1, Iters);
		int32 ItDone = 0;
		for (int32 p = 0; p < CollPasses; ++p)
		{
			DetectContacts(State, Config, Colliders, ColliderBounds, SubAlpha0, SubAlpha1, Contacts);
			const int32 ItTarget = ((p + 1) * Iters) / CollPasses; // 누적 목표(마지막 패스가 Iters를 보장)
			for (; ItDone < ItTarget; ++ItDone)
			{
				// Gauss-Seidel bias를 제거하기 위해 sweep 방향을 번갈아 바꾼다.
				const bool bReverse = (ItDone & 1) != 0;
				SolveDistance(State, Config, FixedDt, bReverse, LambdaDist);
				SolveBending(State, Config, FixedDt, bReverse, LambdaBend);
				SolveContacts(State, Config, Colliders, ColliderBounds, Contacts);
					SolveSegmentContacts(State, Config, Colliders, ColliderBounds, bReverse);
			}
		}

		// 마찰은 substep 끝 1회: 누적된 접촉 법선력(Lambda)으로 Coulomb 한계를 잡는다.
		ApplyContactFriction(State, Config, Contacts, FixedDt);
	}

	// 장력(마지막 substep의 수렴 λ → 힘): XPBD에서 F = λ/h². 스트레치는 C>0 → λ<0이므로 -λ의 양수부만
	// 장력이다(압축/슬랙은 0). 단위는 질량 1 노드 기준 상대 힘 — FRopeSimState::SegmentTension 주석 참고.
	State.SegmentTension.SetNumUninitialized(NumDist);
	const float InvDt2 = 1.0f / (FixedDt * FixedDt);
	for (int32 k = 0; k < NumDist; ++k)
	{
		State.SegmentTension[k] = FMath::Max(0.0f, -LambdaDist[k]) * InvDt2;
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
	// 각도-허용 벤딩(GPU RopeXPBD.usf와 동일): 급한 굽힘(코너/랩 경계)은 펴는 힘을 놔줘 노드가 각지게
	// 튀는 것을 막고, 완만한 굽힘만 곧게 편다. r=Dist/Rest=cos(턴각/2)로 판정. Full은 Release보다 커야
	// smoothstep이 성립하므로 하한을 강제한다(두 값이 같거나 뒤집혀도 안전).
	const float BendRelease = Config.BendReleaseRatio;
	const float BendFull = FMath::Max(Config.BendFullRatio, BendRelease + 1e-4f);
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
		const float BendScale = FMath::SmoothStep(BendRelease, BendFull, Dist / Rest);
		const float DLambda = BendScale * (-C - AlphaTilde * Lambda[i]) / (WSum + AlphaTilde);
		Lambda[i] += DLambda;

		State.Positions[i]     -= N * (WA * DLambda);
		State.Positions[i + 2] += N * (WB * DLambda);
	}
}

void FRopeXPBDSolver::DetectContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
	float SubAlpha0, float SubAlpha1, TArray<FRopeContactState>& Contacts) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Collisions);

	// 재검출: 이전 패스의 활성 플래그만 지운다(Lambda는 substep 시작에만 리셋되어 substep 내 누적 유지).
	for (FRopeContactState& C : Contacts) { C.bActive = false; }
	if (Colliders.Num() == 0)
	{
		return;
	}

	// 로프의 충돌 두께. 0이면 노드가 표면 안에 들어가야만 hit → 얇은 limb/희소 노드에서 대부분 관통.
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();

	// Swept(연속) 충돌: 노드를 점이 아니라 PrevPos->Pos 구간으로 본다. 빠른 노드가 한 substep에 얇은
	// 표면을 가로질러도(이산 점검사로는 터널링) 구간을 따라 샘플해 첫 접촉에서 멈춘다. 느린 접촉(L 작음)은
	// 샘플 1개 = 끝점만 검사하므로 추가 비용이 없다.
	const float SweepStep = FMath::Max(Config.SweepStep, 0.1f);     // 샘플 간격(cm), 디자이너 튜닝
	const int32 MaxSweepSamples = FMath::Max(1, Config.MaxSweepSamples); // 구간당 샘플 상한

	// 이 substep 로프 AABB(콜라이더 1회 컬용). 실제 노드 위치(Prev/Pos) 기반이라 관통 위험 없이,
	// 로프와 안 겹치는 본은 노드 루프/Blend 진입 전에 통째로 스킵한다(매달려도 떨어져 있으면 거의 무비용).
	FBox RopeBounds(ForceInit);
	for (int32 i = 0; i < State.Num(); ++i)
	{
		RopeBounds += State.PrevPositions[i];
		RopeBounds += State.Positions[i];
	}

	// 이번 검출에서 노드가 실제 swept hit를 받았는지. hit가 proximity-watch(아래)를 덮어쓰도록 구분한다.
	TArray<bool> bHitThisDetect;
	bHitThisDetect.Init(false, State.Num());

	// 콜라이더-아우터: substep sub-포즈(움직이는 본의 prev->curr를 알파로 Blend)를 콜라이더당 1회 계산해
	// 노드 루프 밖으로 호이스팅한다(노드마다 Blend 재계산 방지). 한 노드가 여러 collider에 닿으면 마지막 hit가
	// 캐시를 덮어쓴다(노드당 단일 접촉 평면 — pinch 다접촉은 단순화). sub-포즈는 스택 로컬이라 병렬 솔브 안전.
	for (int32 c = 0; c < Colliders.Num(); ++c)
	{
		const IRopeCollider* Collider = Colliders[c];
		if (!Collider)
		{
			continue;
		}
		const FBox ColBounds = bHasBounds ? ColliderBounds[c] : FBox(ForceInit);

		// 이 substep 로프 AABB와 안 겹치는 collider는 통째로 스킵(노드 루프/Blend 진입조차 안 함).
		if (bHasBounds && !ColBounds.Intersect(RopeBounds))
		{
			continue;
		}

		// 이 collider의 이번 substep sub-포즈 1회 계산. 움직이는 본만 Blend(정지면 단일 현재 포즈로 무비용 폴백).
		FRopeSweptQuery SQ;
		SQ.NodeRadius = Radius;
		SQ.SweepStep  = SweepStep;
		SQ.MaxSamples = MaxSweepSamples;
		SQ.SubAlpha0  = SubAlpha0; // 트랜스폼-프리 collider(캡슐)가 자체 prev 상태를 보간하는 데 쓴다.
		SQ.SubAlpha1  = SubAlpha1;
		FTransform PrevX, CurrX;
		if (Collider->GetFrameMotion(PrevX, CurrX) && !PrevX.Equals(CurrX))
		{
			SQ.bUseSubPose = true;
			SQ.SubPoseStart.Blend(PrevX, CurrX, SubAlpha0);
			SQ.SubPoseEnd.Blend(PrevX, CurrX, SubAlpha1);
		}

		for (int32 i = 0; i < State.Num(); ++i)
		{
			if (State.InvMass[i] <= 0.0f)
			{
				continue;
			}

			const FVector A = State.PrevPositions[i];  // substep 시작 위치
			const FVector B = State.Positions[i];      // 끝점(앞선 collider가 밀었을 수 있어 매번 현재값).

			// Broad-phase: 노드 구간 AABB가 collider AABB(+Radius)와 안 겹치면 스킵. 끝점만 보면 가로질러
			// 통과한 노드를 놓치므로 반드시 구간 AABB로 판단한다.
			if (bHasBounds)
			{
				FBox SweepBox(ForceInit);
				SweepBox += A;
				SweepBox += B;
				if (!ColBounds.Intersect(SweepBox))
				{
					continue;
				}
			}

			SQ.WorldStart = A;
			SQ.WorldEnd   = B;

			FVector HitPos;
			const FRopeContact Contact = Collider->QuerySwept(SQ, HitPos);

			FRopeContactState& CC = Contacts[i];
			if (Contact.bHit)
			{
				// CCD: 첫 접촉에서 표면 밖으로 즉시 밀고(가로질러 통과 방지), 이 collider를 접촉으로 확정한다
				// (hit가 proximity-watch를 덮어씀). 이후 SolveContacts가 매 iteration fresh 재질의로 강제.
				State.Positions[i] = HitPos + Contact.Normal * Contact.Penetration;
				CC.bActive        = true;
				CC.ColliderIndex  = c;
				CC.Normal         = Contact.Normal;
				CC.SurfaceVel     = Contact.SurfaceVelocity;
				bHitThisDetect[i] = true;
			}
			else if (!bHitThisDetect[i])
			{
				// swept 미접촉이지만 broad-phase 근접 → watch만 등록한다. iteration 도중 distance/bending이
				// 노드를 표면으로 끌어들이거나(처음엔 밖이라 hit 아님), 분리 휴리스틱이 swept를 억제한 경우라도,
				// SolveContacts가 매 iteration point-query로 실제 침투를 직접 판정해 밀어낸다(밖이면 무동작 —
				// 한쪽 접촉). 이미 hit로 확정된 collider는 덮어쓰지 않는다.
				CC.bActive       = true;
				CC.ColliderIndex = c;
			}
			// CC.Lambda는 그대로 둔다(substep 시작에만 0으로 리셋되어 누적).
		}
	}
}

void FRopeXPBDSolver::SolveContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
	TArray<FRopeContactState>& Contacts) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Contacts);
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();
	const int32 Count = State.Num();
	for (int32 i = 0; i < Count; ++i)
	{
		FRopeContactState& CC = Contacts[i];
		if (!CC.bActive) // DetectContacts가 "이 노드는 어떤 collider엔가 근접" 표시한 노드만.
		{
			continue;
		}
		const float W = State.InvMass[i];
		if (W <= 0.0f)
		{
			continue;
		}

		// 노드에 근접한 *모든* collider를 재질의해 각각 표면 밖으로 민다(겹치는 뼈 다중 접촉을 전부 방어).
		// 캐시된 collider 하나만 보던 버그(다른 뼈 관통을 못 막음, colIdx≠cached로 확인됨) 수정 —
		// 캐시 평면이 아니라 매번 실제 표면을 보므로 곡면/오목에서도 정확. GPU .usf의 노드당 전 collider 루프와 일치.
		const FVector& P = State.Positions[i];
		for (int32 c = 0; c < Colliders.Num(); ++c)
		{
			const IRopeCollider* Collider = Colliders[c];
			if (!Collider)
			{
				continue;
			}
			// broad-phase: collider 월드 bounds(+Radius로 확장됨)에 노드 점이 없으면 스킵(먼 collider 컷).
			if (bHasBounds && !ColliderBounds[c].IsInsideOrOn(P))
			{
				continue;
			}
			const FRopeContact Contact = Collider->Query(P, Radius);
			if (!Contact.bHit)
			{
				continue; // 이 collider 표면 밖 → 접촉력 없음(한쪽 접촉).
			}

			// XPBD rigid 접촉(compliance 0): C = -Penetration(<0). ΔLambda = Penetration/W. Lambda는 >=0 클램프.
			// 위치 갱신 = Normal*Penetration: 노드를 표면으로 재투영. Lambda는 노드별 누적 법선 임펄스(마찰용).
			const float DLambda = Contact.Penetration / W;
			const float NewLambda = FMath::Max(0.0f, CC.Lambda + DLambda);
			const float Applied = NewLambda - CC.Lambda;
			CC.Lambda  = NewLambda;
			CC.Normal  = Contact.Normal;             // 마찰용으로 최신(마지막 접촉) 법선 캐시.
			CC.SurfaceVel = Contact.SurfaceVelocity; // 최신 표면 속도.
			CC.ColliderIndex = c;                    // 마지막 접촉 collider(디버그/마찰 힌트).
			State.Positions[i] += Contact.Normal * (W * Applied);
		}
	}
}

void FRopeXPBDSolver::SolveSegmentContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds, bool bReverse) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_SegContacts);
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();
	const float SweepStep = FMath::Max(Config.SweepStep, 0.1f);       // 샘플 간격(cm) — 노드 점 충돌과 동일 config 재사용.
	const int32 MaxSamples = FMath::Max(1, Config.MaxSweepSamples);   // 세그먼트당 내부 샘플 상한.
	const int32 Count = State.Num() - 1;
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		const float W0 = State.InvMass[i];
		const float W1 = State.InvMass[i + 1];
		if (W0 + W1 <= 0.0f)
		{
			continue; // 양 끝 모두 pin(wrap 구간) → 세그먼트를 못 움직임. 스킵.
		}

		const FVector P0 = State.Positions[i];
		const FVector P1 = State.Positions[i + 1];
		const float SegLen = static_cast<float>(FVector::Dist(P0, P1));
		// 내부 샘플 수(양 끝 노드는 SolveContacts가 이미 처리 → 내부만). 세그먼트가 길수록 촘촘히.
		const int32 NumInner = FMath::Clamp(FMath::FloorToInt(SegLen / SweepStep), 1, MaxSamples);
		for (int32 s = 1; s <= NumInner; ++s)
		{
			const float T = static_cast<float>(s) / static_cast<float>(NumInner + 1); // (0,1) 내부.
			// barycentric 유효 역질량: 내부점을 delta만큼 밀려면 두 끝을 (1-T),(T) 비율로 움직인다.
			const float WEff = (1.0f - T) * (1.0f - T) * W0 + T * T * W1;
			if (WEff <= 0.0f)
			{
				continue;
			}
			// 현재 위치로 매 샘플 재계산(앞 샘플이 끝 노드를 이미 움직였을 수 있음).
			const FVector Mid = FMath::Lerp(State.Positions[i], State.Positions[i + 1], T);
			for (int32 c = 0; c < Colliders.Num(); ++c)
			{
				const IRopeCollider* Collider = Colliders[c];
				if (!Collider)
				{
					continue;
				}
				if (bHasBounds && !ColliderBounds[c].IsInsideOrOn(Mid))
				{
					continue;
				}
				const FRopeContact Contact = Collider->Query(Mid, Radius);
				if (!Contact.bHit)
				{
					continue;
				}
				// 샘플점을 표면 밖으로 Penetration만큼: 보정을 barycentric으로 양 끝에 분배.
				// 이동합 = ((1-T)^2 W0 + T^2 W1)/WEff * Pen = Pen → 내부점이 정확히 표면으로.
				// 속도 중립 보정(GPU 세그먼트 충돌 미러): 현(chord)은 곡면 위 rest에서도 항상 침투해 보정이
				// 계속 발생 → Positions만 밀면 그만큼 바깥 속도가 주입돼 정지 콜라이더 위에서도 노드가 튄다.
				// PrevPositions도 같이 옮겨 위치만 고치고 속도는 보존한다.
				const float DLambda = Contact.Penetration / WEff;
				const FVector D0 = Contact.Normal * ((1.0f - T) * W0 * DLambda);
				const FVector D1 = Contact.Normal * (T * W1 * DLambda);
				State.Positions[i]         += D0;
				State.PrevPositions[i]     += D0;
				State.Positions[i + 1]     += D1;
				State.PrevPositions[i + 1] += D1;
			}
		}
	}
}

void FRopeXPBDSolver::ApplyContactFriction(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<FRopeContactState>& Contacts, float SubDt) const
{
	const float Friction = FMath::Clamp(Config.Friction, 0.0f, 1.0f);
	if (Friction <= 0.0f)
	{
		return;
	}
	const int32 Count = State.Num();
	for (int32 i = 0; i < Count; ++i)
	{
		const FRopeContactState& CC = Contacts[i];
		if (!CC.bActive || CC.Lambda <= 0.0f)
		{
			continue;
		}
		const float W = State.InvMass[i];
		if (W <= 0.0f)
		{
			continue;
		}

		// 노드와 표면의 *상대* 접선 변위(움직이는 표면이 로프를 끌어 쓸어냄). 정지 표면(SurfaceVel 0)이면 노드 변위만.
		const FVector NodeDelta = State.Positions[i] - State.PrevPositions[i];
		const FVector SurfDelta = CC.SurfaceVel * SubDt;
		const FVector RelDelta = NodeDelta - SurfDelta;
		FVector RelTangent = RelDelta - (RelDelta | CC.Normal) * CC.Normal;

		// 자유단 테이퍼: 끝 노드는 장력이 가장 낮아 잘 붙잡히므로 μ를 낮춘다(고정점 frac=0→1, 끝 frac=1→TipFrictionScale).
		const float Frac = (Count > 1) ? (static_cast<float>(i) / static_cast<float>(Count - 1)) : 0.0f;
		const float MuEff = Friction * FMath::Lerp(1.0f, FMath::Clamp(Config.TipFrictionScale, 0.0f, 1.0f), Frac);

		// Coulomb 한계: μ·(누적 접촉 법선력 Lambda)·w. λ는 cm·mass, ×w로 displacement화 → MaxSlip(cm) 동차(매직 상수 없음).
		// 장력이 클수록 Lambda↑(매 iteration 평면을 더 세게 밀어야 하므로) → 그립↑. 그립 초과분은 슬립(영구 그립 방지).
		const float MaxSlip = MuEff * CC.Lambda * W;
		const float TLen = RelTangent.Size();
		if (TLen > MaxSlip && TLen > KINDA_SMALL_NUMBER)
		{
			RelTangent *= (MaxSlip / TLen);
		}
		State.PrevPositions[i] += RelTangent;
	}
}
