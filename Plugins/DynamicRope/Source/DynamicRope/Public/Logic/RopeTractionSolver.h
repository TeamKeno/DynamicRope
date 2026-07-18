// Copyright Epic Games, Inc. All Rights Reserved.
//
// 견인(테더 + 능동 Pull) 축 드라이브의 공용 수학(UObject-free). 테더 두 모드 × 양끝과 능동 Pull은 전부
// "로프 축 속도를 목표로 서보한다"는 같은 골격이고, 목표/접근 방식/상한만 다르다 — 그 차이를 FRopeAxisServo로
// 기술하고 산수는 여기서 한 번만 한다. 월드 없이 단위 테스트 가능(Tests/RopeTractionSolverTests.cpp).
//
// 여기 없는 것 = 인가(어떤 UObject API로 꽂는가). 수신자마다 의미가 달라 통일할 수 없다 — 특히 CMC 캐릭터는
// Movement->Velocity 직접 세팅이 *이번 프레임*에 반영되고 Movement->AddImpulse는 *다음 틱*
// (ApplyAccumulatedForces)에 반영돼 서로 교환 불가다. 호출자(URopeComponent)가 여기서 ΔV/임펄스만 받아
// 자기 수신자 방식으로 인가한다.

#pragma once

#include "CoreMinimal.h"

namespace RopeTraction
{
	/**
	 * 축 속도 서보 스펙. 로프 축(인가 방향) 성분만 다룬다 — 직교 성분(중력/스윙)은 호출자가 보존한다.
	 */
	struct FRopeAxisServo
	{
		/** 목표 축 속도(cm/s, +가 인가 방향). */
		float TargetSpeed = 0.0f;
		/** 목표까지 이번 프레임에 접근할 비율 [0..1]. 1 = 정확 도달(감쇠 없음), 작을수록 여러 프레임에 걸쳐 부드럽게. */
		float Alpha = 1.0f;
		/** false = 가속만(목표에 부족할 때만 인가, 감속 안 함) / true = 제동도(목표 초과 관성 제거 — 코스팅·오버슛 방지). */
		bool bBidirectional = false;
		/** true = 바깥(음수) 축 속도를 먼저 0으로 상쇄한 뒤 목표로 접근(CMC walk 상쇄: 즉시·완전, Alpha 무관). */
		bool bCancelOutward = false;
	};

	/**
	 * 이번 프레임 인가할 축 ΔV(cm/s). 0이면 무동작 — 호출자는 그걸로 스킵을 판단한다.
	 * 단방향(bBidirectional=false)은 목표 이상이면 0을 반환해 역추진/제동을 하지 않는다.
	 */
	DYNAMICROPE_API float ComputeAxisDeltaV(float CurAlong, const FRopeAxisServo& Servo);

	/**
	 * ΔV를 실제 임펄스로: J = 질량 × ΔV, 상한이 있으면 ±MaxImpulse로 클램프(양방향 — 가속/제동 대칭).
	 * MaxImpulse = 최대 장력 × dt. 0 = 무제한(질량 무관하게 ΔV를 그대로 내는 정확 서보).
	 * 질량 M이 속도 V에 한 프레임 만에 도달하는 문턱 장력 ≈ M·V·fps — 그보다 작으면 뒤처진다(무게감).
	 */
	DYNAMICROPE_API float ClampAxisImpulse(float DeltaV, float Mass, float MaxImpulse);

	/**
	 * 한 프레임 축 ΔV 절대 상한(가속 상한 × dt). 양방향 정확 서보(bVelChange)는 ΔV = |현재 − 목표|가
	 * 무제한이라, 빠르게 멀어지는 대상(지면 관통으로 이탈하는 Pierce mesh 등)을 한 프레임에 역전
	 * 슬램해 물리를 폭발시킬 수 있다 — 이 클램프가 역전을 여러 프레임에 분산한다(목표 속도 상한과
	 * 별개의 방어: 목표는 유한해도 현재 속도가 무제한이면 ΔV가 무제한이다). MaxAbsDeltaV ≤ 0 = 무제한.
	 */
	DYNAMICROPE_API float ClampAxisDeltaV(float DeltaV, float MaxAbsDeltaV);

	/**
	 * 리엘 목표 속도(cm/s): 고정 ReelSpeed로 감되 경계 근처(Overshoot < TaperDist)에서 선형 감속하고,
	 * 이번 프레임에 남은 overshoot를 넘게 회수하지 않도록 Overshoot/dt로 캡한다(경계 안착 — 지나쳐
	 * 코스팅→재팽팽 진동이 없다). ReelSpeed=0 → 0(리엘 없음). "상한 없음"이 필요한 호출자는 스스로
	 * Overshoot/dt를 쓴다(설정값 0의 의미가 호출자마다 다르다 — TetherReelSpeed=0은 리엘 없음,
	 * TetherMaxSpeed=0은 상한 없음).
	 */
	DYNAMICROPE_API float ComputeReelTargetSpeed(float Overshoot, float ReelSpeed, float TaperDist, float DeltaTime);

	/**
	 * 속도 주입 결과의 절대 속력 상한 = max(SpeedCap, 기존 속력). 방향이 흔들리면 주입이 프레임마다 다른
	 * 축으로 들어가 감쇠 없는 Falling에서 벡터가 계속 커질 수 있다(폭주 2차 방어 — 1차는 방향 EMA).
	 * 기존에 더 빠른 외부 운동(자유낙하 등)은 보존한다. SpeedCap=0(클램프 없음 설정)이면 그대로 통과.
	 */
	DYNAMICROPE_API FVector ClampInjectedVelocity(const FVector& NewVel, const FVector& OldVel, float SpeedCap);

	/** 유효 역질량(w = 1/유효질량). Mass 0(또는 ~0) = 앵커(무한질량) → 0. */
	DYNAMICROPE_API float InvMassFromMass(float Mass);

	/**
	 * 프레임률 독립 지수 스무딩 계수 α = 1 − exp(−dt/Tau). dt가 아무리 커도 α ≤ 1이라 오버슛하지 않고,
	 * 프레임률이 달라져도 같은 시상수(Tau 초)로 수렴한다(α를 상수로 두면 프레임률에 따라 반응이 달라진다).
	 * Tau ≤ 0 = 스무딩 없음(α = 1, 한 프레임에 목표 도달).
	 */
	DYNAMICROPE_API float ExpSmoothAlpha(float Tau, float DeltaTime);

	/**
	 * 방향 EMA(단위 벡터 전용). 견인 방향이 프레임마다 튀면 클램프/톱업이 매번 다른 축으로 들어가 벡터가
	 * 랜덤워크로 불어난다(폭주) — 그 1차 방어다(2차는 ClampInjectedVelocity의 속력 상한).
	 *  - Current가 ~0(미시드)이면 Target으로 시드한다(첫 유효 프레임 래그 없음).
	 *  - 그 외엔 Lerp 후 재정규화. **180° 반전 순간 Lerp가 정확히 상쇄돼 0이 되면 Target으로 재시드한다** —
	 *    재시드가 없으면 방향이 0이 된 채로 남아 축이 사라진다(호출자 폴백에 의존하게 된다).
	 * Target은 단위 벡터라고 가정한다(호출자가 정규화해 넘긴다).
	 */
	DYNAMICROPE_API FVector SmoothDirection(const FVector& Current, const FVector& Target, float Alpha);

	/**
	 * fractional 조준 위치: 조준 노드 사이를 선형 보간한다. 정수 조준 노드를 그대로 쓰면 프레임 간 이산 홉으로
	 * 방향이 통째로 점프하고 초과분이 노드 단위로 뚝뚝 튄다(견인 "뚝뚝 끊김") — 그 연속화다.
	 * AimF는 [0, AnchorNode] 범위로 클램프해 넘긴다. 인덱스가 범위 밖이면 ZeroVector.
	 */
	DYNAMICROPE_API FVector SampleFractionalAim(const TArray<FVector>& Positions, float AimF, int32 AnchorNode);

	/**
	 * MassShare 자동 분배의 raw 대상 몫 [0..1](EMA 전). 역질량에 지수 MassBias를 걸어 질량차 민감도를
	 * 조절한다: 1 = 선형 역질량(무거운 쪽 = 작은 w → 작은 몫), >1 = 무거운 쪽 몫이 더 급격히 감소(극단),
	 * <1 = 완만, 0 = 50:50. 앵커(w=0)는 지수와 무관하게 항상 몫 0(Pow(0,0)=1 함정 회피).
	 * 양끝 다 앵커(합 ~0)면 0을 반환한다 — "아무도 안 움직임"의 처리는 호출자 몫이다(wielder 몫도 0이라야
	 * 하므로 1-share로 유도하면 안 된다).
	 */
	DYNAMICROPE_API float ComputeRawTargetShare(float InvMassTarget, float InvMassWielder, float MassBias);

	/**
	 * 능동 Pull 팽팽(taut) 게이트 판정(히스테리시스 래치). 팽팽 판정의 정본은 **장력**(XPBD λ 유래)이다 —
	 * 테더 overshoot는 기하라 여기 쓰지 않는다. Threshold ≤ 0(기본) = 장력이 조금이라도 있으면 팽팽
	 * (종전 하드코딩 게이트 "Tension > ~0"과 동일 — 동작 불변). Threshold > 0이면 진입은 Threshold 초과,
	 * 유지(bWasTaut=true)는 Threshold×ReleaseRatio 초과로 판정해 임계 경계의 장력 지터로 게이트가
	 * 켜졌다 꺼졌다 퍼덕이는 것을 막는다(ReleaseRatio는 [0..1]로 클램프).
	 */
	DYNAMICROPE_API bool EvaluateTautGate(float Tension, float Threshold, float ReleaseRatio, bool bWasTaut);

	/**
	 * 전 체인 팽팽(taut) 기하 게이트 판정(히스테리시스 래치). 앵커→손 코너-다리 chord 합(ChordLen)이 자유
	 * 구간 rest 길이(RestLen) × (1 − SlackRatio) 이상이면 로프 전체가 팽팽하다 — 처짐은 다리 chord를 rest보다
	 * 짧게 만들고, 코너에 걸린 팽팽한 로프는 다리별 chord가 rest에 근접해 팽팽으로 인정된다(코너는 손해가
	 * 아니다). 앵커 인접 국소 관측치(세그먼트 장력/sub-leg overshoot)는 움직이는 대상이 슬랙 로프에서도
	 * 만들어내므로(핀 노드가 이웃을 순간 스트레치) 그것만으론 "줄이 다 펴졌나"를 판정할 수 없다 — 그 보완이다.
	 * 유지(bWasTaut=true)는 SlackRatio × ReleaseScale(≥1)로 완화해 경계의 chord 지터로 게이트가 퍼덕이는
	 * 것을 막는다. RestLen ≤ 0이면 false(판정 불능).
	 */
	DYNAMICROPE_API bool EvaluateChainTautGate(float ChordLen, float RestLen, float SlackRatio, float ReleaseScale, bool bWasTaut);

	/**
	 * 견인 주입 장부(debt)의 슬랙 회수 한 스텝. 테더가 주입한 속도 변화의 누적(InOutDebt)에서 Alpha 비율만큼을
	 * Velocity에서 빼고 장부를 그만큼 줄여 반환한다 — 주입하지 않은 운동(스윙 접선/에어컨트롤)은 장부에 없어
	 * 건드리지 않는다. **자동 탕감**: 실제 속도의 장부 방향 성분(avail)이 장부보다 작으면(외부 감속이 이미
	 * 소화) 장부를 avail로 먼저 줄인다 — 없는 속도를 빼서 역방향으로 밀어내는 일이 구조적으로 불가능하다.
	 * Alpha는 [0..1] 클램프(ExpSmoothAlpha로 산출해 넘긴다). 장부가 ~0이면 무동작.
	 */
	DYNAMICROPE_API FVector DecayVelocityDebt(const FVector& Velocity, FVector& InOutDebt, float Alpha);
}
