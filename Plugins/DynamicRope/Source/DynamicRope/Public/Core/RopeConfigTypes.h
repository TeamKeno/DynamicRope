// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RopeConfigTypes.generated.h"

/**
 * 감김 축을 어느 기준으로 배치할지(FRopeWrapConfig::WrappingAxisSource).
 * 두 모드 모두 로프 진행 평면의 normal을 축 방향으로 쓰되, 축 원점과 winding 기준이 다르다.
 * 뒤쪽 폴백(본→부모 → 컴포넌트 기저 → 본 로컬 X)은 공통이다 — FRopeWrappingPhase::ResolveWrappingAxis.
 */
UENUM(BlueprintType)
enum class ERopeWrappingAxisSource : uint8
{
	/**
	 * 본 중심 가이드 평면: 축 원점은 latch 본 위치, winding은 latch tangent를 기준으로 잡는다.
	 * Assisted resolve의 단일 본 wrapping처럼 본별로 축을 재해석해야 하는 경로에 적합하다.
	 */
	BoneCenteredGuidePlane = 0 UMETA(DisplayName = "Bone-Centered Guide Plane"),

	/**
	 * 캡처 진행 평면: 로프가 날아온 스윙 평면의 normal을 축 방향으로 쓰고, 축 원점은 캡처 접촉 영역과
	 * collider 군집 중심으로 보정한다. winding은 캡처 순간 속도를 기준으로 잡아 Composite wrapping에 적합하다.
	 * 캡처 진행 평면을 만들 수 없으면 공통 본/컴포넌트 축 폴백으로 내려간다.
	 */
	CaptureTravelPlane = 1 UMETA(DisplayName = "Capture Travel Plane")
};

/**
 * 시뮬레이션 품질 프리셋 — 정밀도/성능 균형을 한 값으로 고른다. Custom이 아니면 컴포넌트가
 * 솔버(Substeps/Iterations/스윕)·접촉 감지 스윕·감김 경로 빌드 예산을 이 값으로 해석한다
 * (URopeComponent::GetEffective{Solver,Detect}Config·GetEffectiveWrappingPathBuildSteps, 비파괴 —
 * 저장 원본 불변). 개별 Advanced 필드를 직접 조정하려면 Custom. Medium = 기존 기본값과 동일(무변화).
 */
UENUM(BlueprintType)
enum class ERopeSimQuality : uint8
{
	Low = 0 UMETA(DisplayName = "Low"),
	Medium = 1 UMETA(DisplayName = "Medium"),
	High = 2 UMETA(DisplayName = "High"),
	Custom = 3 UMETA(DisplayName = "Custom")
};

/** XPBD solver 튜닝(디자이너용). */
USTRUCT(BlueprintType)
struct FRopeSolverConfig
{
	GENERATED_BODY()

	/** 프레임당 물리 substep 수(anti-tunneling; "small steps"가 iteration을 늘리는 것보다 낫다). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1", ClampMax = "16"))
	int32 Substeps = 12;

	/** substep당 constraint iteration 수. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1"))
	int32 Iterations = 4;

	/** substep당 충돌 해소 패스 수. 1=substep 끝에 1회(기존 동작, perf 무회귀). sharp한 굴곡에서
	 *  distance/bending이 안쪽으로 당기는 힘을 단일 충돌이 못 이겨 관통할 때, 제약 iteration을 이 수만큼
	 *  나눠 사이사이 충돌을 끼운다 → 더 sharp한 끼인각까지 방어(>1일수록 강하지만 비용↑). Iterations로 상한. */
	int32 CollisionPassesPerSubstep = 1;

	/** 접촉 제약을 몇 iteration마다 풀지(CPU 폴백 전용 — GPU 커널은 원래 collision 패스당 1회다).
	 *  1 = 매 iteration(기본, 기존 동작). N = N iteration마다 1회.
	 *
	 *  **어떤 값을 줘도 각 collision 패스의 *마지막* iteration에는 반드시 푼다.** 이게 계약의 핵심이다 —
	 *  마지막에 안 풀면 그 뒤로 distance/bending이 노드를 표면 안으로 당긴 걸 되밀 기회가 없어 substep이
	 *  관통 상태로 끝난다. 그래서 N을 Iterations 이상으로 주면 "패스당 정확히 1회" = GPU와 같은 cadence가 된다.
	 *
	 *  매 iteration 푸는 편이 sharp한 끼인각에서 더 강하다(충돌이 distance/bending과 매번 경쟁하므로 장력에
	 *  안 밀린다). N을 키우면 그 경쟁 횟수를 줄여 비용을 선형으로 깎는 대신 관통 여유가 줄어든다.
	 *  Query가 비싼 SDF 콜라이더에서 CPU 폴백 비용을 직접 나누는 유일한 손잡이다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "1"))
	int32 ContactSolveInterval = 1;

	/** XPBD stretch compliance(stiffness의 역수). 0 = 신장 불가(inextensible). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0"))
	float StretchCompliance = 0.0f;

	/** Strain limiting(최대 신장 클램프): substep solve 뒤, 핀/앵커 고정 노드에서 체인을 따라 walk하며 각
	 *  세그먼트 길이를 ≤ 이 배율 × SegmentLength로 하드 투영한다(속도 중립 — prev도 함께 이동). XPBD 거리
	 *  제약은 Gauss-Seidel이라 iteration이 적으면 긴 체인(수십 노드)이 앵커 핀에 매달릴 때 보정이 끝까지
	 *  전파되지 못해 앵커 인접 세그먼트에 신장이 폭주(6배+)·거대 장력·접선 휩 지터가 생긴다. 순차 sweep은
	 *  한 번에 체인 전체로 전파돼 이 폭주를 상한 안으로 가둔다(PBD long-range constraint 표준 해법).
	 *  1.5 = 최대 50% 신장 허용(기본). 1.0 = 완전 비신축(가장 빡빡). **0 또는 <1 = 비활성**(strain limit 끔). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0"))
	float MaxStretchRatio = 1.5f;

	/** XPBD bending compliance. 클수록 더 흐물거린다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0"))
	float BendCompliance = 0.02f;

	/** 각도-허용 벤딩: 굽힘이 급할수록 펴는 힘을 놔준다(코너/랩 경계에서 free 노드가 각지게 튀는 것 완화).
	 *  판정값 r = (i↔i+2 거리)/(2*SegmentLength) = cos(턴각/2): 1=직선, 작을수록 급한 굽힘.
	 *  r ≤ BendReleaseRatio면 펴는 힘 0(완전히 놔줌), r ≥ BendFullRatio면 100%(기존 동작), 사이는 smoothstep.
	 *  기본 0.70(≈턴각 91°). 코너가 아직 각지면 올리고(급한 굴곡까지 놔줌), 두 값을 0으로 두면 항상 편다(각도 허용 끔). */
	float BendReleaseRatio = 0.70f;

	/** 각도-허용 벤딩: r ≥ 이 값이면 펴는 힘 100%(완만한 굽힘은 기존처럼 곧게 편다). 기본 0.92(≈턴각 46°).
	 *  자유 로프가 너무 흐물거리면 낮추고, 항상 BendReleaseRatio 이상이어야 한다(솔버가 내부적으로 보장). */
	float BendFullRatio = 0.92f;

	/** collider에 대한 접선 방향 friction [0..1](Coulomb 계수 μ). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Friction = 0.5f;

	/** 자유단(끝)으로 갈수록 friction을 약화시키는 배율(고정점=1, 끝=이 값). 끝 노드는 장력이 가장 낮아
	 *  마찰에 잘 붙잡히므로, 끝쪽 그립만 낮춰 잘 놔주게 한다. 1.0이면 테이퍼 없음(균일 friction). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TipFrictionScale = 1.0f;

	/** 충돌 질의 반지름(cm) — 솔버가 노드를 접촉 표면에서 이만큼 띄운다. **기본 0 = auto: 렌더 튜브
	 *  Radius를 그대로 사용**(반지름 3종 자동 정합 — 2026-07-13 표면 감사 B-2; 명시값을 넣으면 그 값).
	 *  해석은 컴포넌트 경계(GetEffectiveCollisionRadius)에서 1회 — 솔버/GPU step은 해석된 값만 받는다.
	 *  주의: 컴포넌트 없이 이 구조체를 직접 쓰는 소비자(유닛 테스트/커스텀 솔버 호출)에는 auto 해석이
	 *  없다 — 0이면 반지름 0으로 동작하므로 반드시 명시값을 넣을 것. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Collision Radius (0=Auto)"))
	float CollisionRadius = 0.0f;

	// NOTE: bUseWorldGDF는 URopeComponent 직속("Rope|Collision" 카테고리)으로 이사했다
	// (2026-07-13 표면 감사 CL-4 — 충돌 도메인 응집: bIncludeOwnerColliders와 한자리).

	/** Swept collision sample spacing in cm. Lower values reduce tunneling at higher query cost. */
	float SweepStep = 2.0f;

	/** Maximum swept samples per segment, used as a cost cap for very fast nodes. */
	int32 MaxSweepSamples = 16;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	/** 공기 저항. 단위는 **60fps 기준 프레임당 속도 감소 비율**(substep 수와 무관 — Integrate가 지수로 보정).
	 *  0.02면 초당 잔존율 0.98^60 ≈ 0.30, 유효 항력 k ≈ 1.2/s → 자유낙하 종단속도 ≈ g/k ≈ 8m/s.
	 *  올릴수록 종단속도가 낮아지고 운동량이 빨리 죽어 "가벼운 리본"처럼 보인다 — 무게감이 필요하면 낮춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Motion Damping"))
	float Damping = 0.02f;

	//~ 스케일링(슬립/LOD) — 다수 로프가 존재할 때 유휴/원거리 비용을 줄인다 ------------------

	/**
	 * 슬립: Free 페이즈에서 모든 노드 속도가 SleepVelocityThreshold 미만으로 SleepDelay 동안 유지되면
	 * 솔브를 통째로 스킵한다(GPU 로프는 dispatch 자체가 없음). 핀 이동/되감기/움직이는 collider 근접
	 * /페이즈 전환에서 깨어난다. 다른 페이즈(Flight~Releasing)는 항상 활성.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling")
	bool bAllowSleep = true;

	/** 슬립 진입 판정 속도(cm/s) — 프레임간 최대 노드 변위 / dt가 이 값 미만이어야 한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.1"))
	float SleepVelocityThreshold = 3.0f;

	/** 슬립 진입까지 저속 상태가 유지되어야 하는 시간(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "s"))
	float SleepDelay = 0.5f;

	/**
	 * 거리 LOD: 플레이어 카메라와의 거리가 LODStartDistance를 넘으면 constraint iteration을 줄이기
	 * 시작해 LODEndDistance에서 LODMinIterationScale까지 선형 감소한다(멀리서는 수렴 오차가 안 보임).
	 * 카메라가 없으면(데디 서버) 항상 풀 iteration.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling")
	bool bEnableDistanceLOD = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "cm"))
	float LODStartDistance = 3000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.0", Units = "cm"))
	float LODEndDistance = 8000.0f;

	/** 최원거리에서의 iteration 배율(1=감소 없음). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Tuning|Scaling", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float LODMinIterationScale = 0.25f;
};

/**
 * 컨택트 결정 튜닝: 걸쳐진 rope가 언제 사지(limb)에 "wrapped"된 것으로 간주되는가?
 * 파라미터 계층(2026-07-13 회의 결정 F): 기본 노출 필드 = T2(밸런스), AdvancedDisplay 필드 = T3
 * (고급 — ②AssistedJudged × BareWrap 판정 인프라 전용이 대부분. ③Guaranteed는 판정/경로 빌드를
 * 쓰지 않으므로 T3가 전부 무의미하다). 상세는 Docs/PoC/02_WrapResolveModes.md §5~6.
 */
USTRUCT(BlueprintType)
struct FRopeWrapConfig
{
	GENERATED_BODY()

	/**
	 * SurfaceVectorField path point가 latch bone 하나에 고정되지 않고 graph 후보 본으로 넘어갈지 여부.
	 * false면 후보 graph depth/cost가 0이 되어 현재 본만 평가하므로 기존 단일 본 동작에 가깝게 돌아간다.
	 * (MultiBone 하위 그룹을 Tuning 앞에 두려고 선언 순서상 여기에 배치 — 디테일 패널은 선언 순서로 하위 그룹을 정렬.)
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|MultiBone")
	bool bEnableMultiBoneWrapping = true;

	/**
	 * 접촉 *질의* 반지름(cm) — 이름이 말하듯 특정 단계 소유가 아니라 **감지(Flight/Contacting)와
	 * 성립(경로 빌드 투영/스냅 상한/DecideWrap)이 공유하는 표면 질의 프로브 반경**이다(그래서 감지
	 * 4종이 DetectConfig로 분리될 때 여기 남았다 — 표면 감사 B-1 축소안, 구 이름 ContactRadius).
	 * **기본 0 = auto: 렌더 튜브 Radius × 1.5**(반지름 3종 자동 정합; 명시값을 넣으면 그 값). 해석은
	 * 컴포넌트 경계(GetEffectiveContactQueryRadius)에서 — 소비처는 해석된 값을 받는다. 주의: 컴포넌트
	 * 없이 직접 쓰는 소비자(유닛 테스트 등)에는 auto 해석이 없다 — 반드시 명시값을 넣을 것. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Contact Query Radius (0=Auto)"))
	float ContactQueryRadius = 0.0f;

	// NOTE: 순수 감지 튜닝 4종(MinLatchNodes/WrapDecisionTime/PredictiveContactFrames/
	// FlightNoContactReturnTime)은 FRopeDetectConfig("Rope|Detect")로 분리됐다(표면 감사 B-1).

	/**
	 * 한 번의 캡처에서 채택할 수 있는 wrap 시드(접촉 대상) 최대 개수. 1(기본) = 기존 단일 시드 동작.
	 * 2 이상이면 dominant 대상 외에, dominant latch보다 tail 쪽에서 *다른* (mesh, bone)에 dwell을
	 * 채운 접촉이 보조 시드로 함께 감긴다(예: 양다리 — 한쪽 다리를 감고 반대쪽 다리 접촉 노드도
	 * 그 본에 고정). 감김 경로(나선)는 dominant 시드에만 생성되고, 보조 시드는 접촉 노드를 자기
	 * 본에 hold하는 방식이다 — 보조 대상 둘레를 도는 경로까지 만들지는 않는다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "1", ClampMax = "8"))
	int32 MaxWrapSeeds = 1;

	/**
	 * 감김 축 유도 소스. BoneCenteredGuidePlane(기본)은 Assisted 단일 본 wrapping을 위해 latch 본을
	 * 축 원점으로 사용한다. CaptureTravelPlane은 접촉 영역/군집 중심 축으로 Composite wrapping을 지원한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning")
	ERopeWrappingAxisSource WrappingAxisSource = ERopeWrappingAxisSource::BoneCenteredGuidePlane;

	/**
	 * SurfaceVectorField 경로가 표면 없는 허공을 tangent 직진(chord)으로 건널 수 있는 최대 거리(cm).
	 * 0(기본) = 끔 — 투영이 끊기면 종전대로 경로 빌드를 실패 처리한다.
	 * 켜면 두 가지가 달라진다(진행 방향 기반 wrap 4단계, 양다리처럼 대상이 둘로 갈라진 랩의 전제):
	 *  ① 투영이 예측점에서 한 세그먼트 이상 떨어진 표면으로 끌어당기려 하면 스냅을 거부하고 chord로
	 *     간다(끄면 QueryRadius 내 관대한 스냅 그대로 — 기본 동작 불변).
	 *  ② chord 구간의 경로점은 앵커를 만들지 않는다 — 커밋 후 그 노드들은 자유 로프로 남아 solver가
	 *     현수/직선 형태를 잡고, 대상이 벌어지면 장력이 걸린다(묶임의 실제 물리).
	 * 이 거리를 넘겨도 표면에 재진입하지 못하면 종전과 같은 실패 처리로 떨어진다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", Units = "cm"))
	float WrappingMaxGapBridgeDistance = 0.0f;

	/**
	 * 감는 양 상한(도): SurfaceVectorField 경로 빌드의 누적 감싼 각도(rolling axis 적분 —
	 * FRopeWrappingState::PathAccumulatedAngleRad)가 이 값에 닿으면 경로를 *성공*으로 조기 마감한다.
	 * 0(기본) = 무제한 — 남은 로프 전량이 감길 때까지 진행(기존 동작).
	 * 긴 로프가 대상을 여러 바퀴 나선으로 감아 들어가며(실측 3000~4400°) 본 전환 재시드가 목/머리
	 * 등으로 번지는 "문어발 랩"의 방지책. 상한에서 마감된 경로 밖의 남는 로프는 Wrapping 동안
	 * 동결됐다가 커밋 후 자유 구간으로 늘어진다(front 모션도 경로 밖 노드는 끌지 않는다).
	 * 양다리 bola면 400~540°(한 바퀴 + 여유)가 자연스럽다.
	 * Composite AnalyticHelix에는 적용되지 않는다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", Units = "deg"))
	float WrappingMaxWrapAngleDeg = 0.0f;

	/** Wrapping phase must keep the same accumulated latch span stable this long before committing. */
	float WrappingStableTime = 0.10f;

	/** Time used to pull tail nodes onto their generated surface wrap targets. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.01", Units = "s"))
	float WrappingMotionDuration = 0.50f;

	/**
	 * wrapping animation의 easing 적용 전 기준 각속도(deg/s).
	 * Single/Composite 모두 이 값으로 FrontWrapAngleRad를 진행한다. 1100deg/s는 3단계 전
	 * Single 실측 시작 정책(baseSpeed 약 203cm/s / 10.57cm/rad)에 맞춘 공통 기본값이다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap",
		meta = (ClampMin = "1.0", ClampMax = "7200.0", Units = "deg/s", DisplayName = "Wrap Speed"))
	float WrappingAngularSpeedDegPerSec = 1100.0f;

	/** 각도 매핑 front가 angle+distance 목표에 도달한 뒤 Wrapped commit 전에 확보할 짧은
	 *  안정화 시간. 마지막 kinematic node/anchor 전환에서 발생할 수 있는 한 프레임 튐을 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning",
		meta = (ClampMin = "0.0", ClampMax = "1.0", Units = "s"))
	float WrappingPostFrontSettleTime = 0.08f;

	/** tail node가 surface path에 안착하도록 세그먼트마다 추가하던 지연 시간.
	 *  4단계부터 commit 제한 시간으로는 각도 매핑을 쓸 수 없는 DistanceFallback 경로에만 적용한다. */
	float WrappingTailDelayPerSegment = 0.024f;

	/** Wrapping 중 한 프레임에 진행할 surface path 적분 step 수. 높이면 빨라지지만 순간 비용이 커진다. */
	int32 WrappingPathBuildStepsPerFrame = 8;

	/**
	 * Axis distance advanced per circumference distance for Sequential SurfaceVectorField wrapping.
	 * Composite Analytic Helix는 이 값을 사용하지 않고 Contacting 순간 tail 기울기에서 자동 산출한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "-2.0", ClampMax = "2.0"))
	float WrappingHelixPitchScale = 0.25f;

	// NOTE: 아래 멀티본 투영 스코어링 세부(깊이/비용/가중치/보너스/히스테리시스 12종)는 실측 튜닝이
	// 끝난 개발자 상수로 내부화됐다(2026-07-13 표면 감사 B-2 — UPROPERTY 제거, 코드에서만 조정).
	// 켜고 끄는 스위치는 위 bEnableMultiBoneWrapping 하나다. 각 값의 의미는 필드별 주석 유지.

	/**
	 * 현재 본에서 몇 edge까지 후보로 볼지.
	 * 지금은 skeleton parent/child edge만 사용한다. 이후 디자이너 지정 transition을 추가해도
	 * 같은 depth 제한을 통과하므로, 너무 먼 bridge가 한 번에 열리는 것을 막는 1차 안전장치다.
	 */
	int32 MaxBoneTransitionDepth = 3;

	/**
	 * 후보 graph 누적 비용 상한.
	 * depth가 같아도 edge별 penalty가 다르면 비용이 달라질 수 있다. 지금은 parent/child edge 비용만
	 * 누적하지만, 나중에 designer edge / 금지에 가까운 edge를 섞을 때 projection 전에 후보를 잘라내는 역할을 한다.
	 */
	float MaxBoneTransitionCost = 5.0f;

	/**
	 * 자동 parent/child edge 하나를 지날 때의 비용.
	 * 값이 클수록 graph cost가 커져 같은 본 유지가 쉬워지고, 낮추면 parent/child chain을 더 적극적으로 탄다.
	 */
	float AutoParentChildTransitionPenalty = 1.0f;

	/** projection 거리 점수 가중치. 예측 위치에서 표면까지 멀수록 불리하다. */
	float ProjectionDistanceWeight = 0.35f;

	/** 실제 rope node 위치와 projection 표면점 사이 거리 가중치. 로프가 실제로 있는 쪽의 본을 선호한다. */
	float RopeNodeDistanceWeight = 0.25f;

	/** 이전 tangent와 새 tangent가 꺾이는 정도의 가중치. 값이 클수록 부드러운 진행을 선호한다. */
	float TangentContinuityWeight = 8.0f;

	/** 이전 normal과 새 normal이 꺾이는 정도의 가중치. 값이 클수록 표면 normal 연속성을 선호한다. */
	float NormalContinuityWeight = 5.0f;

	/** graph 비용 가중치. parent/child를 많이 건너는 후보일수록 불리하게 만든다. */
	float BoneTransitionPenaltyWeight = 1.0f;

	/** 현재 본 유지 보너스. 동점 근처에서 본이 흔들리는 것을 줄인다. */
	float CurrentBoneBonus = 0.35f;

	/** 새 본이 현재 본보다 이 점수만큼 더 좋아야 전환한다. 전환 hysteresis. */
	float BoneTransitionHysteresis = 0.75f;

	/** 직전 본으로 바로 돌아가는 후보에 더하는 penalty. A->B->A 왕복을 줄인다. */
	float ImmediateBoneReturnPenalty = 1.5f;

	/** 마지막 본 전환 이후 이 거리(cm) 이상 진행해야 다음 전환을 허용한다. 0이면 비활성. */
	float MinBoneTransitionPathDistance = 8.0f;

	/** Upper bound for physics-based wrapping settle before committing the best accumulated anchors. */
	float WrappingMaxSettleTime = 0.90f;

	/**
	 * 경로 생성이 실패한 wrap의 최소 감싼 각도(도). 실패 시점까지 감은 각도가 이 값 미만이면 "조금
	 * 닿았는데 철썩 붙는" 커밋 대신 release한다. 0 = 가드 끔.
	 * 각도 기준인 이유(이전 constexpr "최소 1바퀴" 기준 대체): 한 바퀴는 로프 2πr을 요구해 대상이
	 * 클수록 절대 길이가 폭증한다 — 반지름 100cm 몸통은 한 바퀴에 628cm로 기본 로프(200cm)로는
	 * 물리적으로 불가능해 큰 대상 wrap이 구조적으로 전멸했다. 감싼 각도는 대상 크기와 무관한
	 * "걸림 품질" 척도다(120° = 1/3바퀴 훅).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg"))
	float FailedWrapMinAngleDeg = 120.0f;

	/**
	 * 커밋 품질 하한(도): Wrapped로 커밋되는 *모든* wrap(경로 완료/실패/settle 타임아웃 불문)의 감싼
	 * 각도가 이 값 미만이면 커밋 대신 release한다. 0(기본) = 끔 — 기존 동작 그대로.
	 * FailedWrapMinAngleDeg와의 차이: 그쪽은 "경로 생성이 실패한" wrap 전용 조기 abort, 여기는 커밋
	 * 직전 최종 관문. 경로가 정상 완료돼도 latch가 팁 근처면 경로가 짧아(감은 각도 미미) 철썩 붙는
	 * 커밋이 나올 수 있고, settle 타임아웃 커밋은 앵커 1개로도 통과한다 — 그런 부실 랩을 게임
	 * 규칙으로 거르고 싶을 때 opt-in으로 켠다(팁 살짝 걸침도 유효한 디자인이면 0 유지).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg"))
	float CommitMinWrapAngleDeg = 0.0f;

	/**
	 * 형상 기준 묶임 관문(도): 커밋되는 wrap 경로의 감김 축 둘레 각도 커버리지(경로점 각도들을 정렬해
	 * 360° − 최대 공백)가 이 값 미만이면 커밋 대신 release한다. 0(기본) = 끔 — 기존 동작 그대로.
	 * CommitMinWrapAngleDeg(누적 각도)와의 차이: 누적 각도는 걸은 회전량의 합이라 표면 위 진동/왕복이
	 * 값을 부풀릴 수 있고 여러 바퀴면 360°를 넘는다. 커버리지는 "축 둘레 어느 방향까지 로프가 실제로
	 * 둘러쌌는가"의 순수 기하 척도(0~360°)라 진동에 면역이다 — 대상이 정말 갇혔는지(양다리 bola처럼
	 * 빠져나갈 공백이 없는지)를 묻는 판정. 축이 캡처 시점에 고정되는 CaptureTravelPlane 감김에서 가장
	 * 의미가 정확하다(BoneCenteredGuidePlane의 rolling axis에서는 마지막 축 기준 근사).
	 * 양다리 잠금 용도면 300° 안팎, 느슨한 훅도 허용하려면 0 유지.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|Tuning", meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg"))
	float CommitMinWrapCoverageDeg = 0.0f;

	// NOTE: 종전의 [미배선] WrappingContactGraceTime은 삭제됐다(2026-07-13 표면 감사 B-2 — 소비 코드가
	// 없는 죽은 설정). grace 로직을 실제로 배선할 때 그 CL에서 설정도 함께 되살릴 것.

};

/**
 * Flight/Contacting 감지 튜닝 — "언제 잡혔다고 볼 것인가"의 단계로, 성립(FRopeWrapConfig)과
 * 다른 도메인이라 분리했다(2026-07-13 표면 감사 B-1). 공유 프로브 반경(ContactQueryRadius)은
 * 성립 쪽도 쓰므로 WrapConfig에 남아 있다(그쪽 주석 참고).
 */
USTRUCT(BlueprintType)
struct FRopeDetectConfig
{
	GENERATED_BODY()

	/** 스치는 접촉이 아니라 catch로 간주하기 위해 한 bone에 닿아야 하는 최소 rope 노드 수. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Detect", meta = (ClampMin = "1"))
	int32 MinLatchNodes = 1;

	/** wrap을 확정하기 전에 컨택트가 같은 bone에서 이만큼 지속되어야 한다(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Detect", meta = (ClampMin = "0.0", Units = "s"))
	float WrapDecisionTime = 0.016f;

	/** Extra Flight lookahead in frame-displacements for thin limb/SDF candidate detection. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Detect", meta = (ClampMin = "0.0", ClampMax = "4.0"))
	float PredictiveContactFrames = 1.0f;

	/** Whip 종료 후 이 시간 동안 캡처하지 못하면 Free로 복귀한다. 0이면 기본 실패 복귀 쿨다운을 쓴다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Detect", meta = (ClampMin = "0.0", Units = "s"))
	float FlightNoContactReturnTime = 0.0f;

	/**
	 * 접촉 감지 스윕의 샘플 간격(cm). 노드가 한 프레임에 이동한 경로를 이 간격으로 점질의해 최심 접촉을
	 * 찾는다 — **터널링을 막는 값**이라 대상의 얇은 쪽 두께(팔뚝/난간)보다 작아야 한다.
	 *
	 * 종전에는 간격을 `SegmentLength` 기준으로 잡고 샘플을 4개로 잘랐다. 세그먼트 길이는 대상 두께와
	 * 아무 관계가 없어서(로프 20cm 세그먼트 vs 팔뚝 8cm) 빠른 던지기는 샘플 사이로 대상을 그냥 지나쳤고,
	 * CPU/GPU가 똑같이 틀려 parity 테스트도 통과했다. 이제 cm 단위로 끊는다 — solver 충돌의
	 * FRopeSolverConfig::SweepStep과 같은 사고방식이되, 감지는 Flight에서만 도므로 예산을 따로 둔다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Detect", meta = (ClampMin = "0.1", Units = "cm"))
	float ContactSweepStep = 2.0f;

	/** 위 스윕의 샘플 수 상한(비용 한도). 매우 빠른 노드는 간격이 이 상한에 눌려 넓어진다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Detect", meta = (ClampMin = "1", ClampMax = "64"))
	int32 ContactMaxSweepSamples = 16;
};

/**
 * Wrapped *이후*(유지/당김/풀림)의 튜닝 — 설계 노트 01(Post-Wrap 모델)의 도메인이자, "성립 이후는
 * 도달 모드·결착 모델 무관 공통"(02 문서 §3) 경계와 일치한다. 종전에는 FRopeWrapConfig(성립 판정)에
 * 섞여 있던 것을 분리했다(2026-07-13 표면 감사 B-1 — 기존 BP 튜닝 미승계 클린 브레이크).
 * 소비자: Wrapping/운동 제약 + URopeComponent의 Wrapped 4단계
 * (Hold→Pull 샘플→테더/Pull 인가→자동 release).
 */
USTRUCT(BlueprintType)
struct FRopeHoldConfig
{
	GENERATED_BODY()

	/**
	 * Wrapping이 시작된 순간부터 wielder의 손 쪽 자유 구간을 material rest length 안에 강제한다.
	 * true면 RopeWielder가 CharacterMovement의 최종 이동(입력/root motion/slide 포함)을 같은 PrePhysics
	 * 프레임에 구면 제약으로 투영하고, 일반 Pawn은 movement tick 직후 같은 안전망을 적용한다.
	 *
	 * 이 제약은 SegmentTension/GPU readback/아래 TetherSlack과 무관한 gameplay authority다. 따라서
	 * TetherCompliance=0인 로프가 kinematic Pawn 이동 때문에 먼저 늘어난 뒤 사후 회수되는 것을 막는다.
	 * TetherCompliance>0이면 의도적 탄성을 허용하므로 hard projection은 자동 비활성화된다.
	 * Wielder가 없는 custom movement는 URopeComponent::ConstrainWielderLocation을 이동 적용 전에 호출할 것.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning")
	bool bEnforceWielderLengthConstraint = true;

	/**
	 * Material-length constraint activation tolerance(cm). This is a numerical boundary band:
	 * it allows an outward attempt within this distance to produce a stable reaction, but it is
	 * never added to rope length and therefore cannot make an inextensible rope longer.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning",
		meta = (ClampMin = "0.0", Units = "cm"))
	float LengthConstraintActivationSlop = 0.5f;

	/**
	 * Wrapped 중 authoritative material-constraint 장력(kg·cm/s²)이 이 값을 TensionReleaseTime 동안
	 * 지속해서 넘으면 자동 release한다(ERopeReleaseReason::Tension). 0 = 비활성.
	 * GetConstraintTension/MaxTetherTension과 같은 단위이며 XPBD SegmentTension과 혼용하지 않는다.
	 * (자동 release는 도달 모드 ①②에서만 유효 — ③ Guaranteed는 명시 해제만.)
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release", meta = (ClampMin = "0.0"))
	float TensionReleaseForce = 0.0f;

	/** 장력 release 판정의 지속 시간(초). 순간 스파이크(충격 프레임)로 풀리는 것을 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release", meta = (ClampMin = "0.0", Units = "s"))
	float TensionReleaseTime = 0.05f;

	/**
	 * Legacy soft-tether length allowance. The authoritative material constraint no longer adds
	 * this value to rope length; use LengthConstraintActivationSlop for numerical stabilization.
	 * Kept for existing asset serialization while legacy/custom-mover fallback is retired.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning",
		meta = (ClampMin = "0.0", Units = "cm"))
	float TetherSlack = 5.0f;

	/**
	 * (wielder 쪽 시뮬 루트 한정) λ 임펄스는 로프 축 성분만 만드므로, 방향이 급전환하면 옛 방향 관성이
	 * 직교로 남아 날아간다("관성 과다"). 이 비율(0 = 보존, 1 = 완전 제거)로 그 잔여 관성을 몇 프레임에
	 * 걸쳐 빼 fling을 억제한다. 값은 60fps 기준 프레임당 비율이고 적용 시 dt로 보정된다(프레임률 독립).
	 * **대상 쪽 시뮬 바디(프랍/랙돌)는 물리 제약 테더가 담당해 이 감쇠를 타지 않는다** — 남는 수신자는
	 * wielder가 물리 액터 구성(시뮬 루트)일 때뿐이다. CMC 캐릭터에도 적용하지 않는다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TetherPerpDamping = 0.3f;

	/**
	 * 테더 속도 안전 상한(cm/s) — 인가 결과 속력의 2차 클램프(ClampInjectedVelocity — 기존에 더 빠른 외부
	 * 운동은 보존). 0 = 클램프 없음(비권장). λ의 위치 회수 명령 상한은 별도 노브다(TetherMaxBiasSpeed —
	 * 종전엔 이 값을 재사용해 회수가 사실상 무상한이었다).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "cm/s"))
	float TetherMaxSpeed = 1500.0f;

	/**
	 * λ 위치 회수(bias) 명령 속도의 상한(cm/s) — SolveTetherLambda의 MaxBiasSpeed. 벌어짐 상쇄(SepSpeed)와
	 * 달리 이 항만 운동량으로 남는다(단방향 제약이라 슬랙 전환 후 제동이 없다 — 이 값이 곧 슬랙 코스팅
	 * 속도의 상한). 종전엔 TetherMaxSpeed(1500)를 재사용해 가벼운 대상이 한두 프레임에 15m/s로 가속된 뒤
	 * 슬랙 전환과 함께 그대로 날아갔다("휙") — 안착 회수는 이 값이면 충분하다. 테더는 "벌어짐을 막는 것"이
	 * 본분이고 초과분을 능동적으로 되감는 건 회수 항뿐이므로, 이 상한이 테더의 윈치성(性)을 정한다.
	 * 0 = 회수 없음(벌어짐 저지만 — 초과분은 되감기/자연 접근으로만 준다).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "cm/s"))
	float TetherMaxBiasSpeed = 150.0f;

	/**
	 * 접지(발 디딘) 캐릭터가 자기 Mass의 몇 배까지 마찰로 버티는가(유효질량 = Mass × 이 값). 클수록 단단히
	 * 버텨 무거운 대상도 잘 끌고, 작을수록 쉽게 끌려간다. "대상이 얼마나 무거워야 접지한 나를 끌기
	 * 시작하는가"의 교차점을 정하는 유일한 튜닝 노브 — 기본값으로 대부분 무설정.
	 * λ 분배(유효 역질량)와 끌림 가능 판정(climb-in)이 공용으로 쓴다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "1.0"))
	float GroundBraceFactor = 1.5f;

	// (끌림 가능 판정의 히스테리시스는 비노출 내부 상수다 — 질량 노브는 GroundBraceFactor
	//  하나로 통일. RopeComponent.cpp UpdateTargetPullable의 PullMassHysteresis 참조.)

	/**
	 * 거리 release: Wrapped 중 authoritative material-length 위반량이 이 값(cm)을
	 * 초과하면 자동 release한다(ERopeReleaseReason::Distance). TetherSlack은 포함하지 않는다.
	 * 0 = 비활성(기본). 테더와 함께 쓰면
	 * "테더가 버티다가 이 한계를 넘으면 놓친다"가 된다 — 테더가 충분히 강하면 초과분이 안 쌓여
	 * 발동하지 않고, 테더 없이 쓰면 순수 거리 제한으로 동작한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning|Release", meta = (ClampMin = "0.0", Units = "cm"))
	float DistanceReleaseSlack = 0.0f;

	/**
	 * Pull 방향 코너 판정 임계(도). 당김 방향을 앵커→손 직선(chord)이 아니라, 앵커에서 손 쪽으로 로프를
	 * 따라 걸으며 찾은 "첫 직선 다리"의 끝 노드를 향하도록 잡는다 → 로프가 벽/모서리에 걸려 꺾이면 그
	 * 직전에서 멈춰 첫 다리를 따라 당긴다(직선 chord는 장애물을 관통). 걷는 중 다음 세그먼트가 지금까지의
	 * 누적 다리 방향에서 이 각도 이상 꺾이면 코너로 보고 멈춘다 — 곧으면 손(노드 0)까지 걸어가 정확히
	 * chord가 된다. 크게 잡으면(완만한 굴곡 무시) 더 chord에 가깝고, 작게 잡으면 미세한 꺾임에도 민감.
	 * 팽팽할 때의 처짐/노드 지터는 이 임계 아래이고, 벽 모서리는 위라 구분된다(잔여 지터는 SmoothTime이 흡수).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "1.0", ClampMax = "179.0", Units = "deg"))
	float PullBendThresholdDeg = 30.0f;

	/**
	 * Pull 방향 시간 스무딩 상수(초, EMA time constant). look-ahead 방향의 프레임 간 지터 + GPU 미러 지연
	 * 노이즈를 지수이동평균으로 흡수한다(alpha = 1-exp(-dt/이 값), 프레임레이트 독립). 클수록 매끄럽지만
	 * 반응이 느리고, 0이면 스무딩 없음(원 look-ahead). wrap 시작 시 측정값으로 초기화된다.
	 */
	float PullDirSmoothTime = 0.08f;

	/**
	 * Pull 조준 노드 시간 스무딩 상수(초, EMA time constant). walk가 고른 정수 조준 노드(AimNode)는 로프가
	 * 흔들리면 프레임마다 이산적으로 튀어(방향 통째 점프 + tether 초과분 불연속 = 견인 끊김) 방향 EMA로는
	 * 못 잡는다. 조준 인덱스를 float로 EMA해 노드 사이를 보간하면 방향·tether가 연속이 된다(alpha=1-exp(-dt/이
	 * 값), 프레임레이트 독립). 클수록 매끄럽지만 반응이 느리고, 0이면 스무딩 없음. wrap 시작 시 측정값으로 초기화.
	 */
	float PullAimSmoothTime = 0.08f;

	/**
	 * 능동 Pull(입력 홀드)의 **최대 장력**(견인력의 상한) — SetActivePull에 실리는 기본값. 대상을 목표 속도
	 * (ActivePullMaxLinearSpeed)까지 끌 수 있는 최대 힘이다: 가벼운 대상은 이 장력 안에서 목표 속도에 즉시(오버슛
	 * 없이) 도달하고, 이 장력으로 목표까지 못 끄는 무거운 대상은 뒤처진다(현실적 질량 의존 — 이 값이 "몇 kg부터
	 * 버거운가"를 정한다). 힘 크기는 로프 물리 도메인이라 여기 산다(Wielder PullAction이 이 값을 쓴다).
	 * Wrapped + 팽팽할 때만 실제 인가된다(URopeComponent::SetActivePull 계약 — 팽팽 판정은 아래
	 * bActivePullRequiresTaut/ActivePullTautTension 게이트).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ClampMin = "0.0", DisplayName = "Pull Strength"))
	float PullForce = 100000.0f;

	/**
	 * 능동 Pull을 팽팽(taut)할 때만 인가할지. true(기본) = 로프가 팽팽한 프레임에만 힘이 실린다(늘어진 로프를
	 * 당겨도 반응 없음 — 물리적으로 자연스러움). false = 팽팽함 무시: Wrapped + 유효 Pull 샘플이면 항상 인가
	 * (연출/특수 게임플레이용). 팽팽 판정 자체는 URopeComponent::IsPullTaut()로 항상 조회 가능하다(이 스위치와
	 * 무관하게 갱신 — 애니 pull window 등 외부 판단용).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning")
	bool bActivePullRequiresTaut = true;

	/**
	 * 팽팽(taut) 판정의 선택적 load 임계. 0(기본) = 순수 기하 taut이면 능동 Pull을 시작할 수 있다.
	 * > 0이면 authoritative GetConstraintTension()이 이 값을 넘어야 load-bearing으로 본다.
	 * XPBD SegmentTension은 사용하지 않는다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0"))
	float ActivePullTautTension = 0.0f;

	/**
	 * 전 체인 팽팽(taut) 판정 민감도 [0..1] — 0=느슨(적은 팽팽함에도 견인 시작), 1=엄격(더 확실히 펴져야
	 * 견인). 슬랙 허용 비율과 최대 허용 처짐(cm)을 한 값으로 함께 스케일한다(URopeComponent의
	 * GetEffectiveTautSlackRatio / GetEffectiveTautMaxSag — 기하 보간). 0.5(기본) = 기존 튜닝
	 * (슬랙 3%, 처짐 20cm). 0 → 슬랙 9%·처짐 80cm, 1 → 슬랙 1%·처짐 5cm. 판정 히스테리시스·해제 유예는
	 * 실측 튜닝이 끝난 내부 상수다(RopeComponentTraction.cpp). "시각적으로 펴졌을 때만 끌린다"의 단일 손잡이.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Taut Sensitivity"))
	float TautSensitivity = 0.5f;

	/**
	 * Legacy particle-chord analytic fallback의 오염 방지 임계. Wielder/live material geometry가 없는
	 * 경로에서만 자유 구간 XPBD SegmentTension 최솟값을 검사해 부분 스트레치 정귀환을 차단한다.
	 * 정상 Pawn hard-constraint/Chaos 경로의 taut·장력에는 참여하지 않는다. 0(기본) = 끔. 판정
	 * 히스테리시스는 내부 상수(RopeComponentTraction.cpp).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0"))
	float TautMinTension = 0.0f;

	/**
	 * 비신축(TetherCompliance=0) 제약에서 초과분 C의 위치 회수 시상수(초).
	 * β = 1−exp(−dt/이 값)만큼 매 프레임 C를 닫는
	 * 접근 속도를 명령한다 — 작을수록 단단(즉시 안착), 클수록 부드러운 추종. 0 = 한 프레임 전량(β=1).
	 * 프레임률 독립. 회수 명령 속도의 절대 상한은 TetherMaxBiasSpeed다(SolveTetherLambda의
	 * MaxBiasSpeed — 커밋 직후 C가 큰 프레임의 스파이크 방지이자 슬랙 코스팅 잔류의 상한).
	 * 탄성 모드는 이 값을 쓰지 않고 TetherCompliance의 kC 복원력으로 초과 길이를 회수한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "s"))
	float TetherSettleTime = 0.08f;

	/**
	 * 장력 한계/과부하 기준(kg·cm/s², 0 = 무제한).
	 * TetherCompliance>0인 탄성 모드에서는 λ ≤ 이 값 × dt인 실제 force cap이다.
	 * TetherCompliance=0인 비신축 모드에서는 유한 force cap과 exact length를 동시에 만족할 수 없으므로
	 * 길이를 우선하고 full reaction을 보고한다. 이때 이 값은 debugger의 overload 기준선일 뿐이며,
	 * 실제 끊김/해제는 TensionReleaseForce 또는 별도 게임 규칙으로 명시한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0"))
	float MaxTetherTension = 500000.0f;

	/**
	 * 재료 컴플라이언스 α(s²/kg = 역강성). 0(기본) = 비신축 로프.
	 * > 0이면 k=1/α인 implicit spring + generalized critical damping으로 common game
	 * frame rate에서도 안정적인 의도적 탄성(번지 등)을 만든다. 예: 0.0005 → k=2000 kg/s².
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0"))
	float TetherCompliance = 0.0f;

	/**
	 * Deprecated compatibility value. Physical softness is now derived only from
	 * TetherCompliance (stiffness = 1/compliance), so TetherCompliance=0 always creates
	 * a hard Chaos limit. Kept only so existing assets deserialize without data loss.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0"))
	float PhysicalTetherStiffness = 1000.0f;

	/** Deprecated compatibility value. Compliant damping is now derived from effective mass and stiffness. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0"))
	float PhysicalTetherDamping = 100.0f;

	/**
	 * 능동 Pull의 **견인 목표 속도**(cm/s). 능동 Pull은 대상을 이 속도로 당김 방향을 따라 몰되(장력 상한 PullForce
	 * 내에서), 임펄스를 목표 도달분(질량×ΔV)으로 클램프해 **이 속도를 오버슛하지 않는다** — 가벼운 대상이 상수 힘의
	 * a=F/m로 한 프레임에 목표를 훌쩍 넘겨 튕기던(먼지/턱턱) 문제를 없앤다. 무거운 대상은 장력 한계로 이 속도까지
	 * 못 끌어 뒤처진다(현실적 질량 의존). 0 = 견인 없음. 견인 중에만 적용.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "cm/s"))
	float ActivePullMaxLinearSpeed = 300.0f;

	/**
	 * 능동 Pull 대상 물리 바디의 각속도 상한(deg/s, 0 = 무제한). 힘을 무게중심(AddForce)에 주면 토크가 없어
	 * 스핀 원인이 대부분 사라지지만, 랙돌 관절 다이내믹이 만드는 잔여 스핀을 이 상한이 마저 억제한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold|Tuning", meta = (ClampMin = "0.0", Units = "deg/s"))
	float ActivePullMaxAngularSpeed = 720.0f;
};
