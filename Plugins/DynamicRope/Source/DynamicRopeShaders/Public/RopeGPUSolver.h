// Copyright Epic Games, Inc. All Rights Reserved.
//
// XPBD 로프 솔버의 GPU(compute) 구현 — M1: 물리 전용(integrate + distance + bending, 충돌 없음).
// CPU FRopeXPBDSolver와 동일 수식이되 제약은 red-black/stride-3 컬러링으로 푼다(병렬 안전).
// 이 모듈(DynamicRopeShaders)은 DynamicRope 런타임 타입에 의존하지 않는다 → FRopeGPUJob은 POD(raw 포인터+스칼라).
// 호출자(런타임 서브시스템)가 FRopeSimState/Config로부터 잡을 채워 넘긴다.

#pragma once

#include "CoreMinimal.h"

/** GPU 충돌(M2)용 해석적 capsule. 월드 공간 세그먼트(A-B) + 반지름. 호출자가 collider에서 추출해 채운다. */
struct FRopeGPUCapsule
{
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;
};

/**
 * GPU 배치 솔브 1건. Positions/PrevPositions는 in/out(리드백 결과를 같은 버퍼에 써넣는다), InvMass는 in.
 * 포인터는 호출자 소유 버퍼(예: FRopeSimState의 TArray<FVector>)를 가리킨다. SolveBatch가 동기라 호출 동안 유효해야 한다.
 */
struct FRopeGPUJob
{
	// 노드 데이터(길이 NumNodes).
	FVector*     Positions = nullptr;
	FVector*     PrevPositions = nullptr;
	const float* InvMass = nullptr;
	int32        NumNodes = 0;

	// sim / config 스칼라.
	float   SegmentLength = 0.0f;
	bool    bStartPinned = false;
	FVector StartPinPrev = FVector::ZeroVector;
	FVector StartPinTarget = FVector::ZeroVector;
	float   StretchCompliance = 0.0f;
	float   BendCompliance = 0.0f;
	float   Damping = 0.0f;
	int32   Iterations = 1;
	FVector Gravity = FVector::ZeroVector;

	// 충돌(M2). 이 로프에 적용할 capsule 목록(호출자 소유, 동기라 호출 동안 유효). 비면 충돌 없음.
	const FRopeGPUCapsule* Capsules = nullptr;
	int32 NumCapsules = 0;
	float CollisionRadius = 0.0f; // 로프 노드 두께(= FRopeSolverConfig::CollisionRadius).
	float Friction = 0.0f;        // 접선 감쇠 [0..1].
	float SweepStep = 2.0f;       // swept 샘플 간격(cm).
	int32 MaxSweepSamples = 16;   // 세그먼트당 샘플 상한.

	// 이번 프레임 substep 스케줄(호출자가 RopeSolverSubsteps로 계산해 전달).
	int32 NumSub = 0;
	float FixedDt = 0.0f;
};

class DYNAMICROPESHADERS_API FRopeGPUSolver
{
public:
	/**
	 * 배치된 모든 로프를 단일 compute 디스패치(로프=스레드그룹)로 풀고, 결과를 각 Job의 Positions/PrevPositions에 써넣는다.
	 *
	 * NOTE(M1 스파이크): 동기 리드백(렌더 커맨드 내 GPU idle 대기 + Lock, 이후 FlushRenderingCommands).
	 * 매 프레임 GT 스톨이며 목적은 파이프라인 + CPU 패리티 입증이다. 성능(리드백 제거/async)은 후속. 충돌 미포함.
	 */
	static void SolveBatch(TArrayView<const FRopeGPUJob> Jobs);
};
