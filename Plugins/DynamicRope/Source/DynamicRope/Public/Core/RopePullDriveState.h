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
	 * 프레임 간 차분으로 갱신한다. 움직이는 앵커(비행 몬스터 등)에 매달린 wielder 견인의 **피드포워드** 소스 —
	 * 리엘(overshoot P 제어)은 오차만 닫을 수 있어 순항 중인 앵커를 영영 못 따라잡는다(리엘 상한 < 앵커 속도면
	 * 초과분만 무한히 쌓임). 이 값의 로프 축 성분을 견인 목표 속도에 더해 앵커와 같은 속도로 순항하게 한다.
	 * 정지 앵커는 0이라 동작 불변. bPrevAnchorPointValid=false면 미시드(첫 유효 프레임엔 prev만 채운다 —
	 * 0에서 EMA로 램프업해 wrap 직후 홱 당겨지지 않는다). ResetTransient에서 리셋.
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

	/** 능동 Pull의 현재 힘(SetActivePull이 설정, 0=꺼짐). Wrapped + 팽팽할 때만 인가된다. */
	float ActivePullForce = 0.0f;

	/** 이번 프레임 테더 초과분(cm) — 손~앵커 직선 거리 - 가용 로프 길이(0 미만은 0). 디버거 표시용. */
	float LastTetherOvershoot = 0.0f;

	/**
	 * 테더 대상 몫(shareT)의 시간 스무딩 상태(자동 분배). 접지↔공중/질량 변화로 프레임 간 튀는 것을 EMA로
	 * 흡수한다. <0 = 미초기화(wrap 시작 후 첫 유효 프레임에 측정값으로 시드). ResetTransient에서 -1로 리셋.
	 */
	float SmoothedTargetShare = -1.0f;

	/**
	 * 이번 프레임 실제 사용된 대상 몫(shareT) [0..1] — 자동/수동 공통 최종값. wielder 게이트
	 * (URopeWielderComponent::IsWielderTetherActive)와 디버거가 읽는다. 1이면 wielder 몫 0(전량 대상).
	 * BinaryPullable 모드에선 이진값(끌림 가능=1, 불가=0)만 들어간다.
	 */
	float LastTargetShare = 1.0f;

	/**
	 * (BinaryPullable 전용) 대상을 끌 수 있는가의 sticky 판정 상태 — 대상 유효질량 ≤ wielder 유효질량이면
	 * true. bTargetPullableInit이 false면 미시드(다음 유효 프레임에 히스테리시스 없이 순수 비교로 시드),
	 * true면 TetherPullMassMargin 히스테리시스로만 뒤집힌다. ResetTransient에서 미시드로 되돌린다.
	 */
	bool bTargetPullable = true;
	bool bTargetPullableInit = false;

	/** Pull 힘 수신자 없음 경고를 wrap당 1회만 내보내기 위한 래치(ResetTransient에서 리셋). */
	bool bLoggedPullNoReceiver = false;

	/**
	 * 페이즈 전이 시 폐기할 "진행 중 wrap" 일시 상태만 리셋(URopeComponent::ResetTransientPhaseState가 호출).
	 * 의도적으로 남기는 것: ActivePullForce(입력 홀드 상태 — 해제는 SetActivePull(0)의 몫),
	 * LastPullDirRaw/LastTetherOvershoot(디버거 표시용 잔상 — 다음 Wrapped 프레임이 덮어쓴다).
	 */
	void ResetTransient()
	{
		LastPullSample = FRopePullSample();
		// 스무딩 상태는 미초기화 값으로 되돌린다 — 다음 wrap 시작 시 측정값으로 다시 시드.
		SmoothedPullDir = FVector::ZeroVector;
		SmoothedWielderPullDir = FVector::ZeroVector;
		SmoothedAimNodeF = -1.0f;
		SmoothedTargetShare = -1.0f;
		SmoothedAnchorVelocity = FVector::ZeroVector;
		PrevAnchorPoint = FVector::ZeroVector;
		bPrevAnchorPointValid = false;
		bTargetPullableInit = false; // 다음 wrap 시작 시 순수 비교로 다시 시드.
		bLoggedPullNoReceiver = false;
	}
};
