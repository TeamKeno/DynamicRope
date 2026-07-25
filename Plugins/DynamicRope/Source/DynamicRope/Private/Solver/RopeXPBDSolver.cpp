// Copyright Epic Games, Inc. All Rights Reserved.

#include "Solver/RopeXPBDSolver.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"

void FRopeColliderCandidates::Reset(int32 NumNodes)
{
	const int32 NumSeg = FMath::Max(0, NumNodes - 1);
	bValid = false;

	// Reset(슬랙 유지) + AddZeroed로 카운트만 비운다 → substep마다 힙을 다시 잡지 않는다.
	// 인덱스 버퍼는 카운트 밖 슬롯을 읽지 않으므로 초기화할 필요가 없다.
	NodeBounds.SetNum(NumNodes, EAllowShrinking::No);
	NodeIndices.SetNumUninitialized(NumNodes * MaxPerItem, EAllowShrinking::No);
	NodeNum.Reset(NumNodes);
	NodeNum.AddZeroed(NumNodes);
	bNodeOverflow.Reset(NumNodes);
	bNodeOverflow.AddZeroed(NumNodes);

	SegIndices.SetNumUninitialized(NumSeg * MaxPerItem, EAllowShrinking::No);
	SegNum.Reset(NumSeg);
	SegNum.AddZeroed(NumSeg);
	bSegOverflow.Reset(NumSeg);
	bSegOverflow.AddZeroed(NumSeg);
}

void FRopeColliderCandidates::AddNode(int32 NodeIndex, int32 ColliderIndex)
{
	int32& Num = NodeNum[NodeIndex];
	if (Num >= MaxPerItem)
	{
		// 이 노드에 겹치는 collider가 상한 초과 → 후보 목록이 불완전하므로 전량 루프로 폴백시킨다
		// (앞의 MaxPerItem개만 보면 검출이 줄어 관통이 난다 — 여기서만은 느린 게 맞다).
		bNodeOverflow[NodeIndex] = true;
		return;
	}
	NodeIndices[NodeIndex * MaxPerItem + Num] = ColliderIndex;
	++Num;
}

void FRopeColliderCandidates::AddSegment(int32 SegIndex, int32 ColliderIndex)
{
	int32& Num = SegNum[SegIndex];
	if (Num >= MaxPerItem)
	{
		bSegOverflow[SegIndex] = true;
		return;
	}
	SegIndices[SegIndex * MaxPerItem + Num] = ColliderIndex;
	++Num;
}

int32 FRopeColliderCandidates::NodeCount(int32 NodeIndex, int32 NumColliders, bool& bOutAll) const
{
	if (!bValid || !bNodeOverflow.IsValidIndex(NodeIndex) || bNodeOverflow[NodeIndex])
	{
		bOutAll = true;
		return NumColliders;
	}
	bOutAll = false;
	return NodeNum[NodeIndex];
}

int32 FRopeColliderCandidates::SegCount(int32 SegIndex, int32 NumColliders, bool& bOutAll) const
{
	if (!bValid || !bSegOverflow.IsValidIndex(SegIndex) || bSegOverflow[SegIndex])
	{
		bOutAll = true;
		return NumColliders;
	}
	bOutAll = false;
	return SegNum[SegIndex];
}

FRopeSubstepSchedule RopeSolverSubsteps(FRopeSimState& State, const FRopeSolverConfig& Config, float DeltaSeconds)
{
	// 고정 timestep: substep 크기를 frame rate와 무관하게 고정한다(Substeps = "60fps frame당 substep 수"로
	// 해석). 실제 경과 시간을 누적해 고정 크기로 소비하므로 저fps면 더 많은 substep을, 고fps면 더 적은
	// substep을 돌린다 → substep당 변위가 항상 일정 → 충돌/터널링이 frame rate에 의존하지 않는다.
	const int32 SubPerRef = FMath::Clamp(Config.Substeps, 1, 16);
	const float FixedDt = (1.0f / 60.0f) / static_cast<float>(SubPerRef);
	// spiral-of-death 상한(과부하 시 slow-mo). ×1.5 = 40fps까지는 실시간 캐치업, 그 밑은 slow-mo.
	// 종전 ×2(30fps까지 캐치업)는 프레임당 substep을 최대 2배로 몰아, 솔브 부하로 프레임이 떨어질수록
	// 다음 프레임 부하가 더 커지는 정귀환(스파이럴)을 키웠다 — 상한을 낮춰 최악 프레임의 솔브 비용을
	// 정상 상태의 1.5배로 묶는다. 긴 정지 뒤 몰아치기(MaxAccum 클램프)도 같은 상한을 따른다.
	const int32 MaxSubsteps = FMath::Clamp((SubPerRef * 3) / 2, 1, 32);

	State.TimeAccumulator += DeltaSeconds;
	const float MaxAccum = FixedDt * static_cast<float>(MaxSubsteps);
	if (State.TimeAccumulator > MaxAccum)
	{
		// 초과분 버림: 폭주 대신 가벼운 slow-mo.
		State.TimeAccumulator = MaxAccum;
	}

	const int32 NumSub = FMath::FloorToInt(State.TimeAccumulator / FixedDt);
	if (NumSub <= 0)
	{
		// 아직 한 substep 분량이 안 모임(고fps) → 다음 frame으로 이월.
		return FRopeSubstepSchedule{ 0, FixedDt };
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

	// hot-path: 기본 비활성(VeryVerbose). 콘솔 "log LogRopeSolver VeryVerbose"로 올려야 보인다.
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

	// 노드/세그먼트별 collider 후보. detect 패스마다 다시 채워지고 그 패스의 iteration들이 재사용한다.
	// 버퍼를 substep 루프 밖에서 한 번만 잡으려고 여기서 선언한다.
	FRopeColliderCandidates Candidates;

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
		// 접촉 해소 주기. 패스의 마지막 iteration부터 거꾸로 세므로 마지막은 항상 포함된다(아래 참조).
		const int32 ContactInterval = FMath::Max(1, Config.ContactSolveInterval);
		int32 ItDone = 0;
		for (int32 p = 0; p < CollPasses; ++p)
		{
			DetectContacts(State, Config, Colliders, ColliderBounds, SubAlpha0, SubAlpha1, Contacts, Candidates);
			// 누적 목표(마지막 패스가 Iters를 보장).
			const int32 ItTarget = ((p + 1) * Iters) / CollPasses;
			for (; ItDone < ItTarget; ++ItDone)
			{
				// Gauss-Seidel bias를 제거하기 위해 sweep 방향을 번갈아 바꾼다.
				const bool bReverse = (ItDone & 1) != 0;
				SolveDistance(State, Config, FixedDt, bReverse, LambdaDist);
				SolveBending(State, Config, FixedDt, bReverse, LambdaBend);

				// 접촉은 ContactInterval마다. 주기를 패스의 *마지막* iteration에서 거꾸로 세는 것이 요점이다 —
				// 나머지 연산이 0이 되는 지점에 마지막이 항상 포함되므로, 어떤 Interval에서도 "distance/bending이
				// 마지막으로 당긴 뒤 되밀지 못한 채 substep이 끝나는" 관통을 구조적으로 막는다.
				// Interval >= 패스당 iteration 수 → 패스당 정확히 1회 = GPU 커널과 같은 cadence.
				if (((ItTarget - 1 - ItDone) % ContactInterval) == 0)
				{
					SolveContacts(State, Config, Colliders, ColliderBounds, Candidates, Contacts);
					SolveSegmentContacts(State, Config, Colliders, ColliderBounds, Candidates, bReverse);
				}
			}
		}

		// 마찰은 substep 끝 1회: 누적된 접촉 법선력(Lambda)으로 Coulomb 한계를 잡는다.
		ApplyContactFriction(State, Config, Contacts, FixedDt);

		// Strain limiting: iteration으로 못 잡은 잔여 과신장(긴 체인이 앵커 핀에 매달릴 때)을 substep 끝에
		// 순차 sweep으로 상한 안에 가둔다(고정 노드에서 자유단으로 보정 전파).
		SolveStrainLimit(State, Config);
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

void FRopeXPBDSolver::SolveStrainLimit(FRopeSimState& State, const FRopeSolverConfig& Config) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_StrainLimit);
	const float MaxRatio = Config.MaxStretchRatio;
	// 0 또는 <1 = 비활성. 1.0 = 완전 비신축.
	if (MaxRatio < 1.0f)
	{
		return;
	}
	const int32 Count = State.Num();
	const float MaxLen = MaxRatio * State.SegmentLength;
	if (Count < 2 || MaxLen <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// FTL(Follow-The-Leader) 지향 클램프: 세그먼트가 MaxLen 초과면 *follower만* leader 쪽으로 당겨 길이를
	// 정확히 MaxLen으로 맞춘다(leader는 안 움직임). 양쪽을 다 움직이면 직전에 맞춘 세그먼트가 다시 흐트러져
	// 한 sweep으로 수렴하지 못한다 — follower만 옮기면 leader(=이미 배치된 노드)가 불변이라 순차 전파가
	// 보존된다. follower가 고정(InvMass 0)이면 못 옮겨 스킵. 위치 이동은 prev도 함께 옮겨 속도 중립(fling 방지).
	auto ClampToward = [&State, MaxLen](int32 Leader, int32 Follower)
	{
		if (State.InvMass[Follower] <= 0.0f)
		{
			return;
		}
		const FVector Delta = State.Positions[Follower] - State.Positions[Leader];
		const float Dist = Delta.Size();
		if (Dist <= MaxLen || Dist <= KINDA_SMALL_NUMBER)
		{
			return;
		}
		const FVector Target = State.Positions[Leader] + (Delta / Dist) * MaxLen;
		const FVector Corr = Target - State.Positions[Follower];
		State.Positions[Follower]     += Corr;
		State.PrevPositions[Follower] += Corr;
	};

	// 전방(node0→N-1, leader=낮은 인덱스: 핀/앵커가 앞에 있는 손쪽 체인 전파) + 후방(N-1→0, leader=높은 인덱스:
	// wrap 앵커가 뒤에 있는 구간 전파)을 2회. 각 방향은 follower만 옮기므로 그 방향 세그먼트를 1패스로 상한
	// 안에 넣고, 두 방향으로 양끝 고정(손 핀·wrap 앵커)을 모두 처리한다(슬랙이 있으면 수렴, 없으면 최소 잔차).
	// GPU RopeXPBD.usf strain-limit 스테이지와 동일 순서.
	const int32 Passes = 2;
	for (int32 p = 0; p < Passes; ++p)
	{
		for (int32 k = 0; k < Count - 1; ++k) { ClampToward(k, k + 1); }      // 전방: leader=k, follower=k+1
		for (int32 k = Count - 2; k >= 0; --k) { ClampToward(k + 1, k); }     // 후방: leader=k+1, follower=k
	}
}

void FRopeXPBDSolver::Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const
{
	// Damping = "60fps 기준 *프레임*당 속도 감소 비율". substep마다 그대로 곱하면 감쇠가 substep 수에 비례해
	// 쌓여(12 substep이면 초당 720회) 유효 항력이 수십 배가 된다 → 종단속도가 1m/s 아래로 내려앉아 로프가
	// 리본처럼 등속으로 떠내려온다. 지수로 substep 크기에 맞춰 나눠 무게감(가속 램프·운동량)을 보존한다.
	const float Damp = FMath::Pow(1.0f - FMath::Clamp(Config.Damping, 0.0f, 1.0f), SubDt * 60.0f);
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
	float SubAlpha0, float SubAlpha1, TArray<FRopeContactState>& Contacts,
	FRopeColliderCandidates& Candidates) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Collisions);

	// 재검출: 이전 패스의 활성 플래그만 지운다(Lambda는 substep 시작에만 리셋되어 substep 내 누적 유지).
	for (FRopeContactState& C : Contacts) { C.bActive = false; }
	// 후보도 같이 비운다(조기 반환 경로에서도 이전 패스 목록이 남지 않도록 먼저).
	Candidates.Reset(State.Num());
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
	// 샘플 간격(cm, 디자이너 튜닝)과 구간당 샘플 상한.
	const float SweepStep = FMath::Max(Config.SweepStep, 0.1f);
	const int32 MaxSweepSamples = FMath::Max(1, Config.MaxSweepSamples);

	Candidates.bValid = bHasBounds;

	// 노드별 sweep AABB(Prev→Pos)를 1회 만들어 둔다. 로프 전체 AABB(콜라이더 1회 컬용)와 세그먼트 구간
	// 박스가 전부 여기서 파생된다. 실제 노드 위치 기반이라 관통 위험 없이, 로프와 안 겹치는 본은 노드
	// 루프/Blend 진입 전에 통째로 스킵한다(매달려도 떨어져 있으면 거의 무비용).
	const int32 NumNodes = State.Num();
	FBox RopeBounds(ForceInit);
	for (int32 i = 0; i < NumNodes; ++i)
	{
		FBox& NodeBox = Candidates.NodeBounds[i];
		NodeBox = FBox(ForceInit);
		NodeBox += State.PrevPositions[i];
		NodeBox += State.Positions[i];
		RopeBounds += NodeBox;
	}

	// 후보 판정에 줄 여유(cm). 이 detect 패스에 뒤따르는 iteration들이 distance/bending/세그먼트 보정으로
	// 노드를 끌어당길 수 있는 범위를 덮어야 후보에서 collider를 놓치지 않는다. 세그먼트 rest 길이면 충분히
	// 보수적이다 — 한 collision 패스 안에서 노드가 그보다 더 재배치되면 이미 폭주 상태라 다음 substep의
	// 재검출이 답이다. rest 길이가 아직 0인 초기화 직후를 위해 Radius/1cm로 하한을 둔다.
	const float CandidateMargin = FMath::Max3(State.SegmentLength, Radius, 1.0f);

	// 이번 검출에서 노드가 실제 swept hit를 받았는지. hit가 proximity-watch(아래)를 덮어쓰도록 구분한다.
	TArray<bool> bHitThisDetect;
	bHitThisDetect.Init(false, NumNodes);

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
		// 알파 구간은 트랜스폼-프리 collider(캡슐)가 자체 prev 상태를 보간하는 데 쓴다.
		SQ.SubAlpha0  = SubAlpha0;
		SQ.SubAlpha1  = SubAlpha1;
		FTransform PrevX, CurrX;
		if (Collider->GetFrameMotion(PrevX, CurrX) && !PrevX.Equals(CurrX))
		{
			SQ.bUseSubPose = true;
			SQ.SubPoseStart.Blend(PrevX, CurrX, SubAlpha0);
			SQ.SubPoseEnd.Blend(PrevX, CurrX, SubAlpha1);
		}

		for (int32 i = 0; i < NumNodes; ++i)
		{
			if (State.InvMass[i] <= 0.0f)
			{
				continue;
			}

			// A = substep 시작 위치, B = 끝점(앞선 collider가 밀었을 수 있어 매번 현재값).
			const FVector A = State.PrevPositions[i];
			const FVector B = State.Positions[i];

			// 앞선 collider의 push-out으로 옮겨간 위치까지 노드 박스에 누적한다. 커지기만 하므로 이 박스에서
			// 파생되는 세그먼트 후보 판정은 계속 보수적이다.
			Candidates.NodeBounds[i] += B;

			// Broad-phase: 노드 구간 AABB가 collider AABB(+Radius)와 안 겹치면 스킵. 끝점만 보면 가로질러
			// 통과한 노드를 놓치므로 반드시 구간 AABB로 판단한다. 판정은 두 겹이다 — Margin만큼 넓힌 쪽은
			// 뒤따르는 iteration들이 쓸 후보 등록용, 좁은 쪽은 지금 swept query를 쏠지 여부용.
			if (bHasBounds)
			{
				FBox SweepBox(ForceInit);
				SweepBox += A;
				SweepBox += B;
				if (!ColBounds.Intersect(SweepBox.ExpandBy(CandidateMargin)))
				{
					continue;
				}
				Candidates.AddNode(i, c);
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
			}
			// CC.Lambda는 그대로 둔다(substep 시작에만 0으로 리셋되어 누적).
		}
	}

	// 세그먼트 후보는 검출이 다 끝난 뒤(push-out까지 반영된 최종 노드 박스로) 한 번에 만든다.
	// 노드 후보의 합집합으로 대신할 수 없다 — 양 끝 박스 어느 쪽과도 안 겹치면서 세그먼트 중간을 가로지르는
	// collider가 있기 때문(박스들의 합집합 ⊊ 합집합의 박스). SolveSegmentContacts의 내부 샘플은 전부 두 끝
	// 사이에 있으므로 이 구간 박스로 거르면 보수적이다.
	if (bHasBounds)
	{
		for (int32 k = 0; k + 1 < NumNodes; ++k)
		{
			const FBox SegBox = (Candidates.NodeBounds[k] + Candidates.NodeBounds[k + 1]).ExpandBy(CandidateMargin);
			for (int32 c = 0; c < Colliders.Num(); ++c)
			{
				if (Colliders[c] && ColliderBounds[c].Intersect(SegBox))
				{
					Candidates.AddSegment(k, c);
				}
			}
		}
	}
}

void FRopeXPBDSolver::SolveContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
	const FRopeColliderCandidates& Candidates, TArray<FRopeContactState>& Contacts) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Contacts);
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();
	const int32 Count = State.Num();
	for (int32 i = 0; i < Count; ++i)
	{
		FRopeContactState& CC = Contacts[i];
		// DetectContacts가 "이 노드는 어떤 collider엔가 근접" 표시한 노드만.
		if (!CC.bActive)
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
		// 다만 "근접"의 범위는 DetectContacts가 이미 가려 뒀으므로 그 후보만 돈다(상한 초과 노드만 전량 폴백).
		const FVector& P = State.Positions[i];
		// 충돌 push-out 직전(거리 제약까지 반영된) 위치. pinch 시 여기로 되돌려 노드를 얼린다(아래 참조).
		const FVector PrePos = State.Positions[i];
		// pinch 감지: 이 노드가 닿은 collider들의 단위 법선 합·개수(GPU RopeXPBD.usf NodeContact 미러).
		FVector ContactNormalSum = FVector::ZeroVector;
		int32 ContactCount = 0;
		bool bAllColliders = true;
		const int32 NumCand = Candidates.NodeCount(i, Colliders.Num(), bAllColliders);
		for (int32 n = 0; n < NumCand; ++n)
		{
			const int32 c = bAllColliders ? n : Candidates.NodeAt(i, n);
			const IRopeCollider* Collider = Colliders[c];
			if (!Collider)
			{
				continue;
			}
			// broad-phase: collider 월드 bounds(+Radius로 확장됨)에 노드 점이 없으면 스킵. 후보는 Margin만큼
			// 넉넉히 뽑혔으므로(iteration 중 이동 대비) 이 정밀 판정은 후보 안에서도 그대로 필요하다.
			if (bHasBounds && !ColliderBounds[c].IsInsideOrOn(P))
			{
				continue;
			}
			const FRopeContact Contact = Collider->Query(P, Radius);
			if (!Contact.bHit)
			{
				// 이 collider 표면 밖 → 접촉력 없음(한쪽 접촉).
				continue;
			}

			// XPBD rigid 접촉(compliance 0): C = -Penetration(<0). ΔLambda = Penetration/W. Lambda는 >=0 클램프.
			// 위치 갱신 = Normal*Penetration: 노드를 표면으로 재투영. Lambda는 노드별 누적 법선 임펄스(마찰용).
			const float DLambda = Contact.Penetration / W;
			const float NewLambda = FMath::Max(0.0f, CC.Lambda + DLambda);
			const float Applied = NewLambda - CC.Lambda;
			// 마찰용으로 최신(마지막 접촉) 법선/표면 속도를 캐시한다.
			CC.Lambda  = NewLambda;
			CC.Normal  = Contact.Normal;
			CC.SurfaceVel = Contact.SurfaceVelocity;
			State.Positions[i] += Contact.Normal * (W * Applied);
			ContactNormalSum += Contact.Normal;
			++ContactCount;
		}

		// Pinch 감쇠(GPU RopeXPBD.usf 미러): 서로 마주 보는 collider에 동시에 눌린 노드(단위 법선 합이
		// 상쇄 = |sum| << 개수)는 빠져나갈 위치가 없다. last-wins push-out으로 한쪽 표면에 밀어붙인 채 두면
		// 다음 substep 거리 제약이 다시 당겨 재관통 → 프레임 간 위치 왕복(지터)/접선 튕김. 그래서 표면으로
		// 민 결과를 버리고 *충돌 직전 위치(PrePos)에 그대로 얼린다* — 빠져나갈 자리가 없으니 제자리가 최선,
		// 순서 무관해 안정적. 속도도 0(gPrev=gPos=PrePos). 임계 0.6*개수 = 두 법선이 ~106° 초과로 벌어진
		// 경우만 발화(진짜 마주 봄) — 단일면·완만한 코너엔 무영향.
		if (ContactCount > 1 && ContactNormalSum.Size() < 0.6f * static_cast<float>(ContactCount))
		{
			State.Positions[i] = PrePos;
			State.PrevPositions[i] = PrePos;
		}
	}
}

void FRopeXPBDSolver::SolveSegmentContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
	const FRopeColliderCandidates& Candidates, bool bReverse) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_SegContacts);
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();
	// 샘플 간격(cm — 노드 점 충돌과 동일 config 재사용)과 세그먼트당 내부 샘플 상한.
	const float SweepStep = FMath::Max(Config.SweepStep, 0.1f);
	const int32 MaxSamples = FMath::Max(1, Config.MaxSweepSamples);
	const int32 Count = State.Num() - 1;
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		const float W0 = State.InvMass[i];
		const float W1 = State.InvMass[i + 1];
		if (W0 + W1 <= 0.0f)
		{
			// 양 끝 모두 pin(wrap 구간) → 세그먼트를 못 움직임. 스킵.
			continue;
		}

		// 이 세그먼트의 collider 후보(DetectContacts가 세그먼트 구간 박스로 추려 둠). 후보가 없으면 샘플
		// 루프 진입 자체를 건너뛴다 — 보통 대부분의 세그먼트가 여기서 끝난다.
		bool bAllColliders = true;
		const int32 NumCand = Candidates.SegCount(i, Colliders.Num(), bAllColliders);
		if (NumCand == 0)
		{
			continue;
		}

		const FVector P0 = State.Positions[i];
		const FVector P1 = State.Positions[i + 1];
		const float SegLen = static_cast<float>(FVector::Dist(P0, P1));
		// 내부 샘플 수(양 끝 노드는 SolveContacts가 이미 처리 → 내부만). 세그먼트가 길수록 촘촘히.
		const int32 NumInner = FMath::Clamp(FMath::FloorToInt(SegLen / SweepStep), 1, MaxSamples);
		for (int32 s = 1; s <= NumInner; ++s)
		{
			// T는 (0,1) 내부 파라미터.
			const float T = static_cast<float>(s) / static_cast<float>(NumInner + 1);
			// barycentric 유효 역질량: 내부점을 delta만큼 밀려면 두 끝을 (1-T),(T) 비율로 움직인다.
			const float WEff = (1.0f - T) * (1.0f - T) * W0 + T * T * W1;
			if (WEff <= 0.0f)
			{
				continue;
			}
			// 현재 위치로 매 샘플 재계산(앞 샘플이 끝 노드를 이미 움직였을 수 있음).
			const FVector Mid = FMath::Lerp(State.Positions[i], State.Positions[i + 1], T);
			for (int32 n = 0; n < NumCand; ++n)
			{
				const int32 c = bAllColliders ? n : Candidates.SegAt(i, n);
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
