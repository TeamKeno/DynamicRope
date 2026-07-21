// Copyright Epic Games, Inc. All Rights Reserved.
//
// Wrapped 견인/스무딩 상태 묶음. Wrapped 틱 4단계 중 ② 관측치 산출(UpdateWrappedPullSample)과
// ③ 견인 인가(ApplyWrappedTraction: 테더 + 능동 Pull)가 쓰고 갱신하는 프레임 간 상태를 한 타입으로
// 모은다 — 로직은 URopeComponent에 남고(UObject 접근/훅 호출), 여기는 상태와 리셋 규약만 담는다.
// 필드 이름은 컴포넌트 낱개 멤버 시절 그대로다(접근 경로만 PullDrive.X로 변경 — CL 305).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

struct FRopePullDriveState
{
	/**
	 * 이번 프레임 Pull 산출물(Wrapped 동안 매 프레임 산출). BP 조회/디버거 화살표 소스.
	 * Direction은 아래 SmoothedPullDir(시간 스무딩된 방향)으로 매 프레임 덮어써서 소비자(테더/능동 Pull)가
	 * 스무딩된 값을 쓰게 한다.
	 */
	FRopePullSample LastPullSample;

	/**
	 * Pull 방향의 시간 스무딩 상태(EMA). ComputePull의 look-ahead 방향(공간 평균)을 프레임 간 지수이동평균해
	 * 잔여 지터 + GPU 미러 지연 노이즈를 흡수한다. 영벡터 = 미초기화(wrap 시작 후 첫 유효 프레임에 측정값으로
	 * 시드). ResetTransient에서 리셋. 테더/능동 Pull이 이 방향을 공용으로 쓴다.
	 */
	FVector SmoothedPullDir = FVector::ZeroVector;

	/**
	 * wielder 견인 방향(손(노드0)→로프 첫 다리)의 시간 스무딩 상태(EMA — SmoothedPullDir과 동일 상수
	 * PullDirSmoothTime). 영벡터 = 미초기화(첫 유효 프레임에 시드), ResetTransient에서 리셋.
	 * 방향이 프레임마다 튀면 속도 톱업이 매번 다른 축으로 들어가 벡터가 랜덤워크로 불어난다(폭주) —
	 * 방향 안정화가 1차 방어(속력 상한은 ClampInjectedVelocity의 2차 방어).
	 */
	FVector SmoothedWielderPullDir = FVector::ZeroVector;

	/** 스무딩 전 look-ahead 방향(EMA 입력 원본). 디버거가 raw vs smoothed를 나란히 그려 지터 진단에 쓴다. */
	FVector LastPullDirRaw = FVector::ZeroVector;

	/**
	 * 앵커(LastPullSample.WorldPoint)의 월드 속도 추정(cm/s, EMA). 매 Wrapped 유효 프레임에 WorldPoint의
	 * 프레임 간 차분으로 갱신한다. 디버거/BP 관찰용 관측치 — 레거시 피드포워드 소비는 제거됐다(λ 제약은
	 * 끝 속도를 실측해 움직이는 앵커를 자동 추종한다). bPrevAnchorPointValid=false면 미시드(첫 유효
	 * 프레임엔 prev만 채운다). ResetTransient에서 리셋.
	 */
	FVector SmoothedAnchorVelocity = FVector::ZeroVector;
	FVector PrevAnchorPoint = FVector::ZeroVector;
	bool bPrevAnchorPointValid = false;

	/**
	 * Pull 조준 노드의 시간 스무딩 상태(fractional). ComputePull이 고른 정수 AimNode를 float로 EMA해 노드
	 * 사이를 보간 → 방향/tether를 연속화(이산 홉 제거). <0 = 미초기화(wrap 시작 후 첫 유효 프레임에 시드).
	 * ResetTransient에서 -1로 리셋. PullAimSmoothTime이 상수.
	 */
	float SmoothedAimNodeF = -1.0f;

	/** 능동 Pull의 현재 힘(SetActivePull이 설정, 0=꺼짐). Wrapped + 팽팽할 때만 인가된다(아래 bPullTaut 게이트). */
	float ActivePullForce = 0.0f;

	/**
	 * 이번 능동 Pull이 팽팽 게이트를 무시하는가(SetActivePull의 per-call 인자). true면 config
	 * (bActivePullRequiresTaut)와 무관하게 유효 샘플만으로 인가한다 — 애니 pull window의 "팽팽 무시" 구간용.
	 * ActivePullForce와 같은 입력 상태라 ResetTransient에서 남긴다(해제는 SetActivePull의 몫).
	 */
	bool bActivePullIgnoresTaut = false;

	/**
	 * 이번 프레임 팽팽(taut) 게이트 상태(히스테리시스 래치 — RopeTraction::EvaluateTautGate).
	 * UpdateWrappedPullSample(②)이 매 Wrapped 프레임 갱신하고, 능동 Pull 인가(③ ApplyWrappedTraction)와
	 * URopeComponent::IsPullTaut()가 공용으로 읽는다. 유효 Pull 샘플이 없으면(비Wrapped 포함) false.
	 * ResetTransient에서 리셋.
	 */
	bool bPullTaut = false;

	/**
	 * 이번 프레임 전 체인 팽팽(기하) 게이트 상태(히스테리시스 래치 — RopeTraction::EvaluateChainTautGate).
	 * 앵커→손 코너-다리 chord 합 vs 자유 구간 rest 길이의 비교로, 로프 **전체**가 펴져 있는가를 판정한다.
	 * UpdateWrappedPullSample(②)이 매 Wrapped 프레임 갱신하고, 견인 인가(③: 테더 + 능동 Pull)가 선행
	 * 조건으로 읽는다 — 국소 관측치(앵커 인접 장력/sub-leg overshoot)는 슬랙 로프에서도 발생하므로
	 * 이 게이트가 닫혀 있으면 견인하지 않는다. bPullTaut는 이 값 ∧ 장력 임계다. ResetTransient에서 리셋.
	 */
	bool bChainTaut = false;

	// (레거시 서보 시절의 주입 장부(TowedVelDebt)/슬랙 브레이크는 제거됐다 — λ의 위치 회수 항은
	//  MaxBiasSpeed로 유계라 회수할 과잉 주입 자체가 없다. Docs/PoC/05 §3.5.)

	/** 이번 프레임 테더 초과분(cm) — 손~앵커 직선 거리 - 가용 로프 길이(0 미만은 0). 디버거 표시용.
	 *  (Constraint 모드에선 제약 위반 C의 0 클램프 — 산출원만 다르고 의미는 동일하다.) */
	float LastTetherOvershoot = 0.0f;

	/**
	 * (Constraint 모드) 이번 프레임 λ(장력 임펄스, kg·cm/s). 0 = 미발화(슬랙/접근 중/비Constraint 모드).
	 * 테더 장력 관측치 GetTetherTension() = 이 값 / dt — 그 dt를 함께 보관한다. 디버거·BP 조회 소스.
	 * ResetTransient에서 리셋.
	 */
	float LastTetherLambda = 0.0f;
	float LastTetherLambdaDt = 0.0f;

	/**
	 * (Constraint 모드) 직전 프레임의 자유 구간 rest 길이(cm, TetherSlack 포함) — 되감기/SetRopeLength에
	 * 의한 rest 변화율(dRest/dt)을 프레임 차분으로 관측해 벌어짐 속도(s)에 싣는다(감김 = rest 감소 = 벌어짐
	 * 취급 → λ가 당긴다 = 리엘의 유일한 견인 경로). 앵커 노드가 바뀐 프레임은 rest가 불연속이라 차분을
	 * 쓰지 않는다(PrevAnchorNode 비교). bPrevFreeRestValid=false = 미시드. ResetTransient에서 리셋.
	 */
	float PrevFreeRestLen = 0.0f;
	int32 PrevAnchorNode = -1;
	bool bPrevFreeRestValid = false;

	// (Constraint 모드의 스켈레탈 대상 관측/인가 상태는 여기 없다 — 랙돌 절반은 엔진 물리 제약이 담당한다:
	//  URopeComponent::UpdatePhysicalTether. GT 임펄스 관측 상태(점 속도 EMA)는 그 전환으로 폐기됐다.)

	/**
	 * 이번 프레임 유효 대상 몫(shareT) [0..1] — wielder 게이트(URopeWielderComponent::IsWielderTetherActive)와
	 * 디버거가 읽는다. 1이면 wielder 몫 0(전량 대상). UpdateTargetPullable이 끌림 판정의 이진값(가능=1,
	 * 불가=0)으로 채우고, λ가 실제 발화한 프레임엔 UpdateConstraintTether가 역질량비(w_t/w합)로 덮는다.
	 */
	float LastTargetShare = 1.0f;

	/**
	 * 대상을 끌 수 있는가의 sticky 판정 상태(능동 Pull climb-in 방향의 정본) — 대상 유효질량 ≤ wielder
	 * 유효질량이면 true. bTargetPullableInit이 false면 미시드(다음 유효 프레임에 히스테리시스 없이 순수
	 * 비교로 시드), true면 내부 히스테리시스로만 뒤집힌다. ResetTransient에서 미시드로 되돌린다.
	 */
	bool bTargetPullable = true;
	bool bTargetPullableInit = false;

	/** Pull 힘 수신자 없음 경고를 wrap당 1회만 내보내기 위한 래치(ResetTransient에서 리셋). */
	bool bLoggedPullNoReceiver = false;

	/**
	 * 페이즈 전이 시 폐기할 "진행 중 wrap" 일시 상태만 리셋(URopeComponent::ResetTransientPhaseState가 호출).
	 * 의도적으로 남기는 것: ActivePullForce/bActivePullIgnoresTaut(입력 홀드 상태 — 해제는 SetActivePull(0)의 몫),
	 * LastPullDirRaw/LastTetherOvershoot(디버거 표시용 잔상 — 다음 Wrapped 프레임이 덮어쓴다).
	 */
	void ResetTransient()
	{
		LastPullSample = FRopePullSample();
		// 스무딩 상태는 미초기화 값으로 되돌린다 — 다음 wrap 시작 시 측정값으로 다시 시드.
		SmoothedPullDir = FVector::ZeroVector;
		SmoothedWielderPullDir = FVector::ZeroVector;
		SmoothedAimNodeF = -1.0f;
		SmoothedAnchorVelocity = FVector::ZeroVector;
		PrevAnchorPoint = FVector::ZeroVector;
		bPrevAnchorPointValid = false;
		bTargetPullableInit = false; // 다음 wrap 시작 시 순수 비교로 다시 시드.
		bPullTaut = false;
		bChainTaut = false;
		LastTetherLambda = 0.0f;
		LastTetherLambdaDt = 0.0f;
		PrevFreeRestLen = 0.0f;
		PrevAnchorNode = -1;
		bPrevFreeRestValid = false;
		bLoggedPullNoReceiver = false;
	}
};
