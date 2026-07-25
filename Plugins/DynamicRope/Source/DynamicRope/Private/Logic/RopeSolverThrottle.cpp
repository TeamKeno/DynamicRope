// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeSolverThrottle.h"
#include "Collision/RopeCollider.h"

bool FRopeSolverThrottle::UpdateSleepState(ERopePhase Phase, const FRopeSimState& Sim,
	const FRopeSolverConfig& Config, float DeltaTime, bool bHoldAwake)
{
	// Free/Wrapped + 슬립 허용에서만 측정. 그 외에는 누적을 버려 상태 오염을 막는다(캐시는 다음 슬립
	// 페이즈 진입 시 재구축). Wrapped는 로직(Hold/견인/자동 release)이 계속 도는 채 솔브만 쉬는 페이즈라
	// Free와 동일한 침전 측정이 성립한다 — 본이 움직이면 Hold가 쓴 노드 변위가 측정에 그대로 잡힌다.
	const bool bSleepPhase = (Phase == ERopePhase::Free || Phase == ERopePhase::Wrapped);
	if (!bSleepPhase || !Config.bAllowSleep || bAsleep || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		SleepTimer = 0.0f;
		SleepPrevFramePositions.Reset();
		return false;
	}

	// 프레임간 최대 노드 변위 → 속도. Verlet substep 변위가 아니라 프레임 캐시 비교라 substep 수/GPU
	// 미러 지연과 무관하게 동작한다.
	bool bJustSlept = false;
	if (SleepPrevFramePositions.Num() == Sim.Num())
	{
		float MaxDistSq = 0.0f;
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			MaxDistSq = FMath::Max(MaxDistSq, static_cast<float>(FVector::DistSquared(Sim.Positions[i], SleepPrevFramePositions[i])));
		}
		const float MaxSpeed = FMath::Sqrt(MaxDistSq) / DeltaTime;
		// bHoldAwake는 진입만 막는다(타이머 리셋) — 측정 캐시는 계속 갱신해 게이트 해제 시 delay가 깨끗이 재시작.
		SleepTimer = (!bHoldAwake && MaxSpeed < Config.SleepVelocityThreshold) ? SleepTimer + DeltaTime : 0.0f;
		if (SleepTimer >= Config.SleepDelay)
		{
			bAsleep = true;
			SleepPinPos = Sim.StartPinTarget;
			// 드리프트 wake 기준(슬립 중 로직 쓰기 감지 — ShouldWakeFromSleep).
			SleepNodePositions = Sim.Positions;
			// 전이 로그는 호출자(컴포넌트) 담당.
			bJustSlept = true;
		}
	}
	SleepPrevFramePositions = Sim.Positions;
	return bJustSlept;
}

bool FRopeSolverThrottle::ShouldWakeFromSleep(const FRopeSimState& Sim, const FRopeSolverConfig& Config,
	float ReelRate, const TArray<IRopeCollider*>& Colliders) const
{
	if (!Config.bAllowSleep)
	{
		return true;
	}
	// 핀(손)이 슬립 시점에서 이동 — 캐릭터가 움직였다.
	if (FVector::DistSquared(Sim.StartPinTarget, SleepPinPos) > FMath::Square(1.0f))
	{
		return true;
	}
	// 슬립 중 로직 쓰기로 노드가 슬립 시점에서 이동 — Wrapped의 Hold 본 추종이 대표(랩 대상/엘리베이터가
	// 움직이는 중). Free는 슬립 중 노드를 쓰는 주체가 없어 자연히 no-op. 임계 0.5cm(콜라이더 정지 판정과 동일).
	if (SleepNodePositions.Num() == Sim.Num())
	{
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			if (FVector::DistSquared(Sim.Positions[i], SleepNodePositions[i]) > 0.25)
			{
				return true;
			}
		}
	}
	// 되감기/풀기 중.
	if (!FMath::IsNearlyZero(ReelRate))
	{
		return true;
	}
	// 움직이는 collider 근접: 전달된 목록은 이미 로프 bounds로 컬링돼 있어(서브시스템) 근접분만 남는다.
	// 정지 본(prev==curr)은 무시 — 애니 idle 미세 흔들림은 0.5cm 임계로 걸러진다.
	for (const IRopeCollider* Collider : Colliders)
	{
		if (!Collider)
		{
			continue;
		}
		FTransform PrevX, CurrX;
		if (Collider->GetFrameMotion(PrevX, CurrX) && !PrevX.Equals(CurrX, 0.5f))
		{
			return true;
		}
		FVector PrevA, PrevB;
		float InvDt = 0.0f;
		if (Collider->GetGPUCapsuleMotion(PrevA, PrevB, InvDt))
		{
			FVector A, B;
			float R = 0.0f;
			if (Collider->GetGPUCapsule(A, B, R)
				&& (FVector::DistSquared(A, PrevA) > 0.25 || FVector::DistSquared(B, PrevB) > 0.25))
			{
				return true;
			}
		}
	}
	return false;
}

void FRopeSolverThrottle::ComputeSolverLOD(const FRopeSolverConfig& Config, const TOptional<float>& CameraDistance)
{
	SolverLODScale = 1.0f;
	if (!Config.bEnableDistanceLOD || Config.LODStartDistance <= 0.0f || !CameraDistance.IsSet())
	{
		// 비활성 또는 카메라 없음(서버) = 풀 품질.
		return;
	}
	const float Range = FMath::Max(Config.LODEndDistance - Config.LODStartDistance, 1.0f);
	const float Alpha = FMath::Clamp((CameraDistance.GetValue() - Config.LODStartDistance) / Range, 0.0f, 1.0f);
	SolverLODScale = FMath::Lerp(1.0f, FMath::Clamp(Config.LODMinIterationScale, 0.05f, 1.0f), Alpha);
}
