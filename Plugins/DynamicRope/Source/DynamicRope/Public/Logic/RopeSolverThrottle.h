// Copyright Epic Games, Inc. All Rights Reserved.
//
// 솔브 스로틀(UObject-free F-클래스): Free 정지 슬립(솔브/디스패치 스킵) + 거리 LOD(iteration 감쇠).
// 상태(슬립 타이머/측정 캐시/LOD 배율)와 전이·해제 판정을 소유한다. UObject 컨텍스트는 호출마다
// 주입된다 — 카메라 접근(거리 산출)과 슬립 전이 로그는 URopeComponent에 남는다(UpdateSleepState가
// 잠든 프레임에 true를 반환해 로그 시점을 알린다). 월드 없이 단위 테스트 가능.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeConfigTypes.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeSimTypes.h"

class IRopeCollider;

class DYNAMICROPE_API FRopeSolverThrottle
{
public:
	//~ 슬립(Free 전용 — 정지 판정으로 솔브 스킵) -----------------------------
	bool IsAsleep() const { return bAsleep; }

	/** 즉시 깨움(타이머 리셋 포함). 비-Free 페이즈 진입/해제 판정 통과 시 호출. 측정 캐시는
	 *  UpdateSleepState가 비-Free에서 스스로 폐기하므로 여기서 건드리지 않는다. */
	void Wake() { bAsleep = false; SleepTimer = 0.0f; }

	/** 슬립 전이 측정(Finalize, 매 프레임): Free + 슬립 허용에서 프레임간 최대 노드 속도가 임계 미만으로
	 *  SleepDelay 지속되면 잠든다. 그 외 페이즈에서는 누적/캐시를 폐기한다(상태 오염 방지).
	 *  이번 호출에 막 잠들었으면 true — 호출자(컴포넌트)가 전이 로그를 담당한다. */
	bool UpdateSleepState(ERopePhase Phase, const FRopeSimState& Sim, const FRopeSolverConfig& Config, float DeltaTime);

	/** 슬립 해제 판정(Prepare): 핀(손) 이동 / 되감기 중 / 움직이는 근접 collider. Colliders는 이미
	 *  로프 bounds로 컬링된 프레임 스냅샷(SimFrame.FrameColliders)을 기대한다. */
	bool ShouldWakeFromSleep(const FRopeSimState& Sim, const FRopeSolverConfig& Config, float ReelRate,
		const TArray<IRopeCollider*>& Colliders) const;

	//~ 거리 LOD(원거리 iteration 감쇠 — 안정성은 substep이 지배하므로 iteration만) ---
	/** LOD 배율 갱신(Prepare, 매 프레임). CameraDistance 미설정(서버/카메라 없음) = 풀 품질(1). */
	void ComputeSolverLOD(const FRopeSolverConfig& Config, const TOptional<float>& CameraDistance);

	float GetSolverLODScale() const { return SolverLODScale; }

	/** LOD 반영된 유효 iteration(CPU 솔브/GPU 스텝 공용). */
	int32 LODScaledIterations(int32 ConfigIterations) const
	{
		return FMath::Max(1, FMath::RoundToInt(static_cast<float>(ConfigIterations) * SolverLODScale));
	}

private:
	/** Free 정지 판정으로 솔브 스킵 중. */
	bool  bAsleep = false;

	/** 저속 유지 누적(초). */
	float SleepTimer = 0.0f;

	/** 슬립 진입 시 핀 위치(이동 시 wake). */
	FVector SleepPinPos = FVector::ZeroVector;

	/** 프레임간 변위 측정 캐시(Finalize에서 갱신). */
	TArray<FVector> SleepPrevFramePositions;

	/** 거리 LOD iteration 배율(Prepare가 계산, 1=풀). */
	float SolverLODScale = 1.0f;
};
