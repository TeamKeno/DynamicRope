// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"

class USceneComponent;
class UPrimitiveComponent;
class UCharacterMovementComponent;
class AActor;

/**
 * 견인 수신자의 종류 — "무엇이 로프 힘을 받는가"의 단일 판정 결과(RopeComponent.cpp ResolveTetherEndpoint).
 * 테더와 능동 Pull이 각자 래더를 걷던 것을 한 해석으로 합친 결과물이라, 질량 분배와 실제 인가 지점이
 * 어긋나는 일이 구조적으로 불가능하다. 확장 훅(ApplyTractionToReceiver)이 수신자를 기술할 때도 쓴다.
 */
enum class ERopeEndpointKind : uint8
{
	None,      // 수신자 없음(Owner도 없음).
	SimBody,   // 물리 시뮬 바디: 스켈레탈 승격 본 / 대상 프리미티브 / 소유 루트.
	Character, // CMC가 실제로 구동 중인 캐릭터(MOVE_None 제외).
	Anchor,    // 정적/키네마틱/MOVE_None/비시뮬 비캐릭터 — 무한질량(움직이려면 위치 폴백뿐).
};

/**
 * 테더/능동 Pull이 공유하는 수신자 해석 결과. UObject 포인터는 한 GT 프레임 동안만 소비하며 소유하지 않는다.
 * 종류·실제 인가점·유효질량을 한 번에 확정해 판정과 인가가 서로 다른 endpoint를 보지 않게 한다.
 */
struct FRopeTetherEndpoint
{
	ERopeEndpointKind Kind = ERopeEndpointKind::None;
	UPrimitiveComponent* Prim = nullptr;
	FName Bone = NAME_None;
	UCharacterMovementComponent* Movement = nullptr;
	AActor* Actor = nullptr;
	float Mass = 0.0f;
};

/** Wrapped 한 프레임 동안 target/wielder endpoint 해석을 공유하는 비소유 캐시. */
struct FRopeResolvedWrappedEndpoints
{
	FRopeTetherEndpoint Target;
	FRopeTetherEndpoint Wielder;
	TWeakObjectPtr<USceneComponent> TargetMesh;
	FName TargetBone = NAME_None;
	bool bValid = false;

	void Reset() { *this = FRopeResolvedWrappedEndpoints(); }
};

/** 이 인가가 어느 견인 경로에서 왔는가 — 서브클래스가 경로별로 다르게 반응할 수 있게 한다. */
enum class ERopeTractionSource : uint8
{
	/** 자동 견인: λ 임펄스 제약이 양끝에 나눠 인가하는 회수(UpdateConstraintTether). */
	Tether,
	/** 사용자 입력 능동 Pull(상수 힘). 대상 인가와 climb-in(wielder 인가) 둘 다 포함. */
	ActivePull,
};

/**
 * 로프가 수신자에 견인을 인가하기 **직전**의 요청 기술(POD, 비소유 포인터 — 호출 동안만 유효).
 * URopeComponent::ApplyTractionToReceiver가 받는 유일한 타입이며, 로프가 만드는 모든 힘/속도 개입이
 * 이 한 타입으로 기술된다(경로는 Source로 구분).
 */
struct FRopeTractionRequest
{
	ERopeTractionSource Source = ERopeTractionSource::Tether;
	ERopeEndpointKind ReceiverKind = ERopeEndpointKind::None;

	/** SimBody: 인가할 프리미티브와 본(본 없으면 컴포넌트 단위). */
	UPrimitiveComponent* Prim = nullptr;
	FName Bone = NAME_None;

	/** Character: 인가할 무브먼트. */
	UCharacterMovementComponent* Movement = nullptr;

	/** 수신자 소유 액터(Kind 무관, 있으면 채움). 커스텀 무브먼트는 보통 여기서 자기 컴포넌트를 찾는다. */
	AActor* Actor = nullptr;

	/** 인가 방향(단위 벡터). */
	FVector Direction = FVector::ZeroVector;

	/**
	 * Source별 크기 — 단위가 다르니 반드시 Source와 함께 읽을 것.
	 *   Tether     = 이번 프레임 축 속도 변화 ΔV = λ×유효 역질량(cm/s)
	 *   ActivePull = 힘의 크기 = 장력 상한(N)
	 */
	float Amount = 0.0f;

	float DeltaTime = 0.0f;

	/** wielder(로프 소유자) 쪽 인가인가. false면 감긴 대상 쪽. */
	bool bWielderSide = false;
};

/**
 * Pull(당김) 샘플: wrap 앵커가 로프로부터 받는 당김을 데이터로 기술한다(Docs/PoC/01_PostWrapModel.md 4.2).
 * FRopeWrapController::ComputePull이 채우고(UObject-free), 컴포넌트가 힘 인가(캐릭터/물리 본)로 변환한다.
 */
struct FRopePullSample
{
	bool    bValid = false;

	/** 손 쪽 첫 앵커 노드(힘 인가 지점의 노드). */
	int32   AnchorNode = INDEX_NONE;

	/** 첫 직선 다리 끝(walk가 멈춘 정수 노드) — 방향의 raw 조준(ComputePull 산출; 디버그/진단). */
	int32   AimNode = INDEX_NONE;

	/** 앵커가 붙은 본(물리 본 힘 인가 대상). */
	FName   Bone = NAME_None;

	/** 앵커 노드 월드 위치(힘 인가점). */
	FVector WorldPoint = FVector::ZeroVector;

	/** 당김 단위 방향(앵커에서 조준 쪽 = 로프 경로 추종; 소비 시 컴포넌트가 fractional+EMA 스무딩). */
	FVector Direction = FVector::ZeroVector;

	/** 앵커-손 쪽 인접 세그먼트 장력(FRopeSimState::SegmentTension 단위). */
	float   Tension = 0.0f;

	/**
	 * 스무딩된 fractional 조준 인덱스([0, AnchorNode); <0 = 미설정). AimPos와 함께 소비자(컴포넌트)가
	 * AimNode를 float로 시간 스무딩해 채운다(ComputePull은 정수 AimNode만 산출) — tether/방향이 이 연속
	 * 값을 써 정수 조준 노드의 프레임 간 이산 홉(방향 점프 + 견인 끊김)을 없앤다.
	 */
	float   AimNodeF = -1.0f;

	/** 노드 사이 보간된 조준 월드 위치(AimNodeF 위치). */
	FVector AimPos = FVector::ZeroVector;

	/**
	 * 앵커→손 코너-다리 chord 합(cm). walk를 첫 코너에서 멈추지 않고 손(노드 0)까지 이어 각 다리의 직선
	 * 거리를 누적한 값 — FreeRestLen과의 비교가 "전 체인 팽팽" 판정의 관측치다(RopeTraction::
	 * EvaluateChainTautGate). 처짐은 chord를 rest보다 짧게 만들고, 코너에 걸린 팽팽한 로프는 다리별 chord가
	 * rest에 근접해 팽팽으로 인정된다. **다리별 chord는 그 다리의 rest 길이로 클램프**한다 — 움직이는
	 * 앵커가 앵커 쪽 다리를 스트레치시키면(세그먼트 > rest) chord가 rest를 초과해, 나머지 로프의 슬랙을
	 * 상쇄·은폐하는 것을 막는다(PIE 실측 620/600cm 사례). 단 지그재그로 구겨진 슬랙은 다리가 잘게 쪼개져
	 * 여전히 rest에 붙는 맹점이 있다 — 그건 MinFreeTension 게이트가 잡는다.
	 *
	 * ⚠ chord 결손은 처짐의 **제곱**으로만 줄어든다(600cm 로프의 chord 590 = 눈에 보이는 처짐 ~45cm) —
	 * "시각적으로 펴졌는가"의 판정자는 이 비율이 아니라 MaxLegSag(cm, 선형)다. 이 값은 완만한 대형 처짐과
	 * 압축(노드 뭉침)의 거친 백스톱으로 남는다.
	 */
	float   TautChordLen = 0.0f;

	/**
	 * 앵커→손 코너-다리 chord 합의 **비클램프** 값(cm) — TautChordLen과 달리 다리별 rest 클램프를 하지
	 * 않아, 스트레치된 다리는 그만큼 합을 키운다. Constraint 테더의 제약 위반 관측치: C = 이 값 −
	 * (FreeRestLen + TetherSlack). (TautChordLen의 클램프는 팽팽 *게이트* 전용 규약이라 제약 위반량으로는
	 * 못 쓴다 — 클램프 합은 정의상 FreeRestLen을 넘지 못해 C가 항상 음수가 된다.)
	 *
	 * ⚠ C > 0은 발화의 **필요조건일 뿐**이다(∧ bChainTaut — 랙돌 PIE 2026-07-20 교훈): 랙돌 본 요동이
	 * 앵커 인접 다리만 strain limit까지 늘리면 나머지가 늘어져 있어도 합이 rest를 넘어 슬랙 로프에서
	 * C > 0이 된다(부분 스트레치 오염). 그 가짜 C에 λ가 발화하면 견인→요동→스트레치의 정귀환 폭주가
	 * 된다 — "전체가 펴졌는가"의 정본은 여전히 3중 팽팽 게이트다.
	 */
	float   PathChordLen = 0.0f;

	/** 자유 구간(손~앵커) rest 길이(cm) = AnchorNode × SegmentLength(되감기 축소 자동 반영). */
	float   FreeRestLen = 0.0f;

	/**
	 * 자유 구간(손~앵커) 세그먼트 장력의 **최솟값**(FRopeSimState::SegmentTension 단위). 팽팽한 로프는
	 * 장력이 앵커에서 손까지 전 구간으로 전달되므로 최솟값이 양수고, 어딘가 한 구간이라도 슬랙이면
	 * (압축/구김 — XPBD 장력은 당김만 계상) 0이다 — chord 합 기하가 못 보는 지그재그 슬랙/부분 스트레치를
	 * 이걸로 판별한다. 아직 솔브 전(배열 비어 있음)이면 0(GPU 로프는 1~2프레임 지연 미러).
	 */
	float   MinFreeTension = 0.0f;

	/**
	 * 다리별 최대 처짐(cm) = 각 코너-다리의 내부 노드가 그 다리 chord 직선에서 벗어난 최대 수직 거리.
	 * "시각적으로 펴졌는가"의 직접 관측치 — chord 비율(처짐의 제곱에만 반응)과 달리 처짐 cm에 **선형**으로
	 * 반응한다(PIE 실측: chord 590/600(98.3%)인 로프의 실제 처짐 ~45cm). 다리 단위라 코너에 걸린 팽팽한
	 * 로프(다리별로 곧음)는 값이 작고, 완만한 catenary 처짐은 그대로 cm로 드러난다.
	 */
	float   MaxLegSag = 0.0f;
};
