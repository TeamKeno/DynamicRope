// Copyright Epic Games, Inc. All Rights Reserved.
//
// Dynamic Rope 시스템의 핵심 데이터 타입. 핫 루프(sim/contact/wrap 상태)에 있는 것은
// 순수 POD로, 디자이너용 설정에만 USTRUCT를 사용한다.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "RopeTypes.generated.h"

// 랩 대상 추상화(Decision 0): 랩 대상을 실을 수 있는 포인터는 스켈레탈에 국한하지 않고 USceneComponent로
// 일반화한다(정적/무버블 프롭 opt-in — 피드백 5번). 스켈레탈 경로는 필요 지점에서만 Cast로 되찾는다.
class USceneComponent;

/**
 * 라이프사이클 단계. 물리(solver)는 Free/Flight에서 전체를, Wrapping/Wrapped에서는 마스크되지 않은
 * 자유 구간만 굴린다. Contacting/Wrapping/Wrapped/Releasing의 판정·구동은 로직(Logic/ F-클래스) 담당.
 */
UENUM(BlueprintType)
enum class ERopePhase : uint8
{
	Free,
	Flight,

	/** 접촉 후보를 매 프레임 재수집하며 트래커 dwell로 wrap 진입을 판정하는 중. */
	Contacting,

	/** 감기는 중(표면 경로 점진 생성 + front 모션 + 질량 마스크). */
	Wrapping,

	Wrapped,

	/** PreviewPathLocked 전용. 물리 Flight를 타지 않고 cached preview path를 authoritative하게 따라간다. */
	GuidedThrow,

	Releasing
};

/** wrap이 해제된 이유. */
UENUM(BlueprintType)
enum class ERopeReleaseReason : uint8
{
	Manual,

	/** 손~앵커 거리가 가용 로프 길이 + DistanceReleaseSlack 초과(자동). */
	Distance,

	/** 최대 장력이 TensionReleaseForce를 지속 초과(자동). */
	Tension,

	/** 대상 소실/wrap 실패 등 내부 사유. */
	Broken,

	/** 외부 게임플레이가 로프를 절단(URopeComponent::CutRope). */
	Cut
};

/**
 * 감김 해결(도달) 모드 — 이 로프가 던지기~결착 성립까지 무엇을 보장하는지의 계약.
 * 설계 근거/상황별 기대 매트릭스는 Docs/PoC/02_WrapResolveModes.md(2026-07-13 팀 합의).
 * 모드는 새 파이프라인이 아니라 기존 세 경로의 이름표다: 조준·preview의 지위와 판정 관문의
 * 사용 여부를 이 값이 결정하고, Wielder의 조준/던지기 방식도 여기서 유도된다(별도 스위치 없음).
 * Wrapped 성립 이후(Hold/Pull/테더/release)는 모드 무관 공통이다.
 */
UENUM(BlueprintType)
enum class ERopeWrapResolveMode : uint8
{
	/**
	 * ① 전체 시뮬: 날리기부터 결착까지 전부 창발. 조준 보정/preview 없음 — 빗나감·스침·판정
	 * 미달 전부 정상 결과다(현실 대응). 샌드박스/리서치용.
	 */
	FullSimulation UMETA(DisplayName = "Full Simulation"),

	/**
	 * ② 보조+판정(기본): aim ray가 대상을 잠가 명중은 보장하되, 결착 성립은 판정(감싼 각도/
	 * 커버리지 관문)이 결정한다. preview는 표시용(비구속). 실패(release)도 정상 결과. 전투/스킬용.
	 */
	AssistedJudged UMETA(DisplayName = "Assisted (Judged)"),

	/**
	 * ③ 무조건 성립: 입력 순간 확정한 preview가 곧 실행 경로(구속). preview 생성 실패 = 던지기
	 * 거부(Aim 무효)라 던진 뒤의 실패는 없다. 자동 release(장력/거리)도 무효 — 명시 해제만.
	 * 데모/연출/이동기용. BareWrap 결착의 무조건 성립은 Aim 단계에서 걸러진다(회의 결정 B).
	 */
	GuaranteedWrap UMETA(DisplayName = "Guaranteed")
};

/** Wrapping 중 tail node의 목표 surface path를 생성하는 방식. 로프별 선택(FRopeWrapConfig). */
UENUM(BlueprintType)
enum class ERopeWrappingPathMode : uint8
{
	/** bone-parent 축을 기준으로 수학적 helix를 만든 뒤 SDF 표면에 투영한다. 일정한 나선 실루엣을 얻기 쉽다. */
	AnalyticHelix UMETA(DisplayName = "Analytic Helix"),

	/** 매 step마다 축 기준 원주 방향 벡터장을 만들고 SDF tangent plane에 투영한다. 의도적으로 원주를 돌면서 표면 굴곡도 따라간다. */
	SurfaceVectorField UMETA(DisplayName = "Surface Vector Field")
};

/**
 * 감김 축을 어디서 유도할지(FRopeWrapConfig::WrappingAxisSource). 우선순위 체인의 앞부분만 다르고,
 * 뒤쪽 폴백(본→부모 → 컴포넌트 기저 → 본 로컬 X)은 공통이다 — FRopeWrappingPhase::ResolveWrappingAxis.
 * (CL 341이 형상 축을 주석으로 껐다 켰다 하던 실험을 정식 설정으로 승격 — 진행 방향 기반 wrap 1단계.)
 */
UENUM(BlueprintType)
enum class ERopeWrappingAxisSource : uint8
{
	/**
	 * 형상 축 우선(기존 동작): latch 본에 귀속된 collider의 장축 → 진행 방향 축 → 공통 폴백.
	 * 단일 대상(드래곤 몸통/목, 인간형 팔다리 하나)을 그 대상의 실루엣대로 감는 데 정확하다.
	 */
	ShapeAxisFirst UMETA(DisplayName = "Shape Axis First"),

	/**
	 * 진행 방향 축 우선: 로프가 날아온 스윙 평면의 normal(Flight whip guide)을 축으로 앞세운다 →
	 * 형상 축 → 공통 폴백. 감김 원주가 로프의 운동 평면에 놓이므로 여러 본/대상에 걸친 랩(양다리)이
	 * 특정 본 하나의 축에 끌려가지 않는다. 본 전환 시 재시드(rolling axis)에서도 같은 소스가 이겨
	 * 축 방향이 진행 평면에 고정된다. 가이드 평면이 없는 던지기(BP 직행 등)는 형상 축으로 폴백.
	 */
	TravelPlaneFirst UMETA(DisplayName = "Travel Plane First")
};

/**
 * narrow-phase 컨택트: rope 노드 하나 vs collider 하나, IRopeCollider::Query가 반환한다.
 *
 * CONTRACT — FROZEN 2026-06-24 (2026-06-27 SurfaceVelocity 추가: 기본 0인 가산 필드라 하위호환).
 * 모든 IRopeCollider(capsule, bone-SDF, world-GDF)는 이를 반드시 준수해야 한다.
 * 단일 (node, collider) 쌍을 기술한다. 집계는 호출자의 몫이다(solver는 push-out을 합산하고,
 * DecideWrap은 노드별로 penetration이 가장 깊은 bone을 선택한다).
 *
 *   bHit         노드 구체(center = query WorldPos, radius = query Radius)가 collider와 겹친다.
 *                false => 나머지 필드는 모두 정의되지 않음. 호출자는 이를 무시해야 한다.
 *   Normal       UNIT, collider에서 노드를 향해 바깥쪽을 가리킨다(push-out 방향).
 *                불변식: NodePos += Normal*Penetration 은 노드를 표면 위에 올려놓는다.
 *                *** 부호가 load-bearing이다: 안쪽을 향하는 normal은 rope를 몸체 안으로 빨아들인다. ***
 *                축퇴(노드가 medial axis 위에 있음) => 임의의 안정적인 단위 벡터(capsule: +Z).
 *   Penetration  Normal을 따른 overlap 깊이, bHit일 때 > 0. QUERY 반지름 기준으로 측정된다:
 *                (ColliderRadius + QueryRadius) - Distance. 호출자는 solver push-out에는 QueryRadius 0을,
 *                wrap-decision skin에는 WrapConfig.ContactRadius를 전달한다.
 *   SurfacePoint 노드에서 가장 가까운 collider 표면 위의 점(보조/디버그). solver에는 필수가 아니며,
 *                저렴하게 구할 수 있을 때 채운다.
 *   Bone         skeletal collider에서는 반드시 non-None — bone 귀속(attribution)으로 DecideWrap이
 *                wrap을 건다. 멀티-bone SDF는 가장 가까운 표면을 소유한 bone을 반드시 보고해야 한다. world => None.
 *   SourceMesh   Bone을 소유한 skeletal mesh. 액터 간 follow를 전달한다(-> FRopeWrapState::Mesh).
 *                비-skeletal collider에서는 null.
 *   SurfaceVelocity 접촉점에서 collider 표면의 월드 속도(cm/s). solver가 상대 접선 속도 마찰로
 *                로프를 끌고 가는 데 쓴다(움직이는 몸이 정지한 로프를 좌우로 쓸어내게 함).
 *                정적/미지원 collider는 0(= 정적 표면)으로 둔다 — 기존 동작과 동일.
 */
struct FRopeContact
{
	bool    bHit = false;
	FVector Normal = FVector::UpVector;
	float   Penetration = 0.0f;
	FVector SurfacePoint = FVector::ZeroVector;
	FName   Bone = NAME_None;
	const USceneComponent* SourceMesh = nullptr;
	FVector SurfaceVelocity = FVector::ZeroVector;
};

/** latch된 wrap 노드. bone-local 공간에 고정되어 재충돌 없이 skinning을 따라간다. */
struct FRopeLatchNode
{
	int32   NodeIndex = INDEX_NONE;
	FName   Bone = NAME_None;
	FVector BoneLocalPos = FVector::ZeroVector;
};

/** Wrapping/Wrapped가 노드를 표면에 고정할 때 쓰는 표면 앵커(bone-local 프레임 + 진행 눈금). */
struct FRopeSurfaceAnchor
{
	int32 NodeIndex = INDEX_NONE;

	FName Bone = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	/** SDF 표면 기준 bone-local 앵커 프레임(위치/법선/접선). */
	FVector LocalSurfacePosition = FVector::ZeroVector;
	FVector LocalNormal = FVector::UpVector;
	FVector LocalTangent = FVector::ForwardVector;

	/** Wrapping 시작 순간의 월드 위치. front 모션의 Lerp 시작점으로 사용한다. */
	FVector StartWorldPosition = FVector::ZeroVector;

	/** 여러 번 감김/길이 계산이 쓰는 로프 진행 눈금. 커밋 시점 눈금으로 저장되므로 reel(길이 변경) 후엔 stale. */
	float RopeDistance = 0.f;

	/** 표면에서 로프 중심선을 얼마나 띄울지(보통 rope radius). */
	float SurfaceOffset = 0.0f;
};

/** Wrapping 표면 경로의 점 하나(월드 표면 프레임 + 귀속 본/메시 + latch로부터의 진행 거리). */
struct FRopeWrapPathPoint
{
	FVector SurfaceWorld = FVector::ZeroVector;
	FVector NormalWorld = FVector::UpVector;
	FVector TangentWorld = FVector::ForwardVector;

	/**
	 * 이 path point가 투영된 실제 표면 본.
	 * AnalyticHelix는 기존처럼 latch bone을 넣고, SurfaceVectorField는 projection scoring 결과를 넣는다.
	 * 이후 AppendWrappingAnchorFromPathPoint가 이 값을 기준으로 bone-local anchor를 저장한다.
	 */
	FName Bone = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	float DistanceFromLatch = 0.0f;

	/**
	 * 허공 브리지(chord) 경로점(WrappingMaxGapBridgeDistance > 0에서만 발생): 표면 투영 없이
	 * tangent 직진으로 만들어졌다. front 모션의 위치 목표로는 참여하지만 앵커는 만들지 않는다 —
	 * 커밋 후 이 구간 노드는 자유 로프로 남는다.
	 */
	bool bBridge = false;
};

/** Wrapping 페이즈의 작업 상태(FRopeWrappingPhase::State). 경로 빌드 진행/앵커 축적/커밋 판정 재료. */
struct FRopeWrappingState
{
	FName BoneName = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	TArray<FRopeSurfaceAnchor> Anchors;
	FRopeSurfaceAnchor LatchAnchor;
	TArray<FRopeWrapPathPoint> Path;

	/**
	 * 보조 시드 앵커(시드 다중화, MaxWrapSeeds > 1): dominant latch보다 tail 쪽에서 *다른* 본에
	 * 접촉해 채택된 앵커들. 경로 빌드/front 모션의 대상이 아니라 Wrapping 내내 자기 본에 hold되고,
	 * BuildCommitSeed에서 경로 앵커들과 합류한다(BeginWrap/Hold는 앵커별 Bone/Mesh를 이미 지원).
	 * Begin *이전에* 채워야 한다 — BeginProgressiveWrapPathBuild가 경로 길이(NumTailNodes)를
	 * 첫 보조 시드 노드 앞까지로 클램프하는 데 읽는다(경로 앵커와 보조 앵커의 노드 중복 방지).
	 */
	TArray<FRopeSurfaceAnchor> SecondarySeedAnchors;

	int32 NumTailNodes = 0;
	int32 LastAnchoredPathPointCount = 0;
	bool bPathBuildActive = false;
	bool bPathBuildComplete = false;
	bool bPathBuildFailed = false;

	float PathCurrentDistance = 0.0f;
	float FrontDistance = 0.0f;
	ERopeWrappingPathMode PathMode = ERopeWrappingPathMode::SurfaceVectorField;
	FVector PathSurfaceWorld = FVector::ZeroVector;
	FVector PathNormalWorld = FVector::UpVector;
	FVector PathTangentWorld = FVector::ForwardVector;
	FVector PathCircumferenceDir = FVector::ForwardVector;
	FVector PathAxisOrigin = FVector::ZeroVector;
	FVector PathAxisDirection = FVector::ForwardVector;
	FVector PathLatchRadial = FVector::ForwardVector;

	/**
	 * SurfaceVectorField 적분 중 현재 surface point가 어느 본 위에 있는지 추적한다.
	 * 다음 step의 후보 본은 이 값을 중심으로 skeleton graph 근방에서 고른다.
	 */
	FName PathCurrentBone = NAME_None;

	/**
	 * 마지막으로 떠난 본. 새 후보가 바로 이 본이면 A->B->A 왕복 가능성이 높으므로
	 * scoring 단계에서 ImmediateBoneReturnPenalty를 더해 전환 떨림을 줄인다.
	 */
	FName PathPreviousBone = NAME_None;
	TWeakObjectPtr<const USceneComponent> PathCurrentMesh = nullptr;

	/**
	 * 마지막 본 전환 이후 path가 표면을 따라 진행한 거리(cm).
	 * 새 본 후보가 좋아 보여도 MinBoneTransitionPathDistance 전에는 현재 본을 유지해
	 * 한두 step마다 본이 바뀌는 flicker를 막는다.
	 */
	float PathDistanceSinceBoneTransition = 0.0f;

	float PathWindingSign = 1.0f;

	/**
	 * 현재 이어지고 있는 허공 브리지(chord)의 누적 길이(cm). 표면 재진입(스냅 수용) 시 0으로 리셋.
	 * WrappingMaxGapBridgeDistance를 넘기면 경로 빌드가 실패 처리된다(브리징 비활성이면 항상 0).
	 */
	float PathBridgeDistance = 0.0f;

	/**
	 * SurfaceVectorField 경로 빌드 중 적분한 누적 감싼 각도(라디안). 감김 축이 본 전환마다
	 * 재해석되므로(rolling axis) latch 축 하나를 가정하는 helix 공식으로는 전체 각도를 계산할 수
	 * 없다 — 걷기 스텝마다 현재 축 기준 radial 회전량을 더해 둔다. AnalyticHelix 모드는 축이
	 * 고정이라 이 값을 쓰지 않고 기존 helix 공식을 유지한다.
	 */
	float PathAccumulatedAngleRad = 0.0f;

	float Elapsed = 0.0f;
	float Duration = 0.16f;
	float StableTime = 0.0f;

	int32 FirstNode = INDEX_NONE;
	int32 LastNode = INDEX_NONE;
	int32 LastStableFirstNode = INDEX_NONE;
	int32 LastStableLastNode = INDEX_NONE;
	int32 LastStableAnchorCount = 0;

	void Reset()
	{
		*this = FRopeWrappingState();
	}

	bool IsActive() const
	{
		return !BoneName.IsNone() && Anchors.Num() > 0;
	}
};

/**
 * wrap 이후 데이터 모델(바인딩 시맨틱). 물리→로직 핸드오프 시점에 생성된다.
 * TODO: Wrapped까지 surface anchor 기반 정리가 끝나면 legacy Latched 경로를 제거하고 Anchors로 일원화한다.
 */
struct FRopeWrapState
{
	FName                   BoneName = NAME_None;

	/** legacy fallback(bone-local 점 고정, 구 방식). 현행 커밋 경로는 Anchors를 채운다. */
	TArray<FRopeLatchNode>  Latched;

	/** 표면 앵커(현행 방식). */
	TArray<FRopeSurfaceAnchor> Anchors;

	/** 최대 세그먼트 장력(Wrapped 중 매 프레임 갱신). */
	float                   Tension = 0.0f;

	/** 감긴 누적 시간(Hold가 증가 — 포획 성공 판정 등 게임 소비 예정). */
	float                   TimeWrapped = 0.0f;

	// BoneName을 소유한 Mesh. wrap은 이 mesh에 대해 유지/추적된다(rope 소유자와 다른
	// 액터일 수 있음). 결정 시점에 컨택트로부터 해석된다.
	// cross-actor wrap에서는 대상 액터가 Wrapped 도중 파괴될 수 있다. raw 포인터로 보관하면
	// Hold가 매 프레임 dangling 포인터를 역참조(use-after-free)하므로, 파괴 시 안전하게 null이
	// 되는 weak 포인터로 보관한다(POD 유지: hard 레퍼런스가 아니라 GC를 막지 않는다).
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	bool IsWrapped() const { return Anchors.Num() > 0 || Latched.Num() > 0; }
	void Reset() { *this = FRopeWrapState(); }
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
};

/** rope 중심선: 파티클의 체인. solver / 로직 / 렌더의 단일 진실 공급원(single source of truth). */
struct FRopeSimState
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;
	float           SegmentLength = 0.0f;
	float           RopeLength = 0.0f;

	/**
	 * 고정된 시작점(hand/socket). solver는 substep에 걸쳐 Prev->Target으로 쓸어 이동시키므로
	 * 빠른 앵커 점프가 에너지를 주입하는(체인을 폭발시킬) 대신 흡수된다.
	 */
	bool            bStartPinned = false;
	FVector         StartPinPrev = FVector::ZeroVector;
	FVector         StartPinTarget = FVector::ZeroVector;

	/** Fixed timestep accumulator. The solver consumes real frame time in fixed-size substeps. */
	float           TimeAccumulator = 0.0f;

	/**
	 * 세그먼트별 장력(힘, 스트레치=양수만). XPBD distance 제약의 수렴 λ에서 유도: F = max(0, -λ)/h².
	 * 단위는 질량 1 노드 기준 mass·cm/s²(상대값) — 임계치는 실측으로 튜닝한다. CPU 솔버가 Step 끝에
	 * 채우고, GPU 상주 로프는 λ 리드백(1~2프레임 지연)이 채운다. 솔브 없는 프레임은 직전 값 유지.
	 * 크기 = Num()-1(비어 있을 수 있음 — 아직 한 번도 솔브 안 됨).
	 */
	TArray<float>   SegmentTension;

	int32 Num() const { return Positions.Num(); }
	void  Reset() { Positions.Reset(); PrevPositions.Reset(); InvMass.Reset(); SegmentTension.Reset(); TimeAccumulator = 0.0f; }

	//~ Verlet 어휘(순수 인라인 — 컨텍스트/정책 없음). 반복 관용구에 이름을 붙여 부호·차원 실수를 막는다.
	//  솔버 적분 루프(RopeXPBDSolver)와 던지기 속도 주입 루프는 의도적으로 raw 표현을 유지한다 —
	//  전자는 .usf 커널과의 1:1 파리티 대조가 우선, 후자는 누적형(변위 단위 임펄스)이라 형태가 다르다.

	/** 노드 i의 한 프레임 변위(Pos - Prev). Verlet에서 속도 ∝ 변위(dt 나누기 전). */
	FVector Displacement(int32 i) const { return Positions[i] - PrevPositions[i]; }

	/** 노드 i의 한 프레임 이동 거리(cm/프레임). "빠른 노드" 등 임계 판정은 소비자의 정책이다. */
	float NodeSpeed(int32 i) const { return Displacement(i).Size(); }

	/** 노드 i의 속도 0(Prev = Pos). 시드/리시드 경로 전용 — 로직 페이즈의 위치·속도 쓰기는
	 *  FRopeNodeOverrideFrame 단일 통로를 탄다(G2, GPU 상주 동기화). */
	void SetStill(int32 i) { PrevPositions[i] = Positions[i]; }
};

/**
 * FRopeNodeOverrideFrame::Flags의 노드별 비트. ERopeGPUOverride(RopeGPUSolver.h)와 수치 1:1이어야
 * 한다(서브시스템이 검증) — Core는 Shaders 모듈에 의존하지 않으므로 상수를 미러로 둔다.
 */
namespace RopeNodeOverride
{
	/** Pos[i] = Positions[i] */
	constexpr uint8 Position         = 1 << 0;

	/** Prev[i] = PrevPositions[i] (Verlet 속도 주입) */
	constexpr uint8 Prev             = 1 << 1;

	/** Prev[i] = Pos[i] (속도 0; Position 적용 *후* 값) */
	constexpr uint8 PrevFromPosition = 1 << 2;

	/** InvMass[i] = InvMass[i] */
	constexpr uint8 InvMass          = 1 << 3;
}

/**
 * 로직 페이즈의 한 프레임 산출물(G2): "타깃 계산은 GT, 적용은 통로 하나로".
 * Wrapping/Wrapped/Releasing 등 로직이 Sim에 쓰고 싶은 위치·속도·질량을 여기에 scatter하면,
 * PrepareSimFrame 끝에서 CPU Sim에 1회 적용되고(ApplyToSim — 기존 직접 쓰기와 동일한 결과),
 * GPU 상주 로프에는 같은 데이터가 override 패스(FRopeGPUResidentStep)로 실려 재시드 없이
 * 커널에서 적용된다. 같은 노드를 여러 번 채우면 나중 것이 이긴다(순차 Sim 쓰기와 동일).
 * 주의: Prev(명시)와 PrevFromPosition을 한 프레임에 섞어 채우지 말 것 — 커널 적용 순서상
 * PrevFromPosition이 항상 이겨 채운 순서와 무관해진다(로직 페이즈는 PrevFromPosition만 쓴다).
 */
struct FRopeNodeOverrideFrame
{
	/** 노드별 RopeNodeOverride 비트 OR(비어 있으면 이번 프레임 산출물 없음). */
	TArray<uint8>   Flags;
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;

	bool HasAny() const { return Flags.Num() > 0; }

	void Reset()
	{
		Flags.Reset();
		Positions.Reset();
		PrevPositions.Reset();
		InvMass.Reset();
	}

	/** 첫 scatter 시 노드 수만큼 0으로 확보(프레임 내 재호출은 no-op). */
	void EnsureSize(int32 NumNodes)
	{
		if (Flags.Num() != NumNodes)
		{
			Flags.SetNumZeroed(NumNodes);
			Positions.SetNumZeroed(NumNodes);
			PrevPositions.SetNumZeroed(NumNodes);
			InvMass.SetNumZeroed(NumNodes);
		}
	}

	/** 위치 고정: Pos=World, bZeroVelocity면 Prev=Pos(속도 0 — wrapping/hold의 표준 쓰기). */
	void SetPosition(int32 NodeIndex, const FVector& World, bool bZeroVelocity)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::Position | (bZeroVelocity ? RopeNodeOverride::PrevFromPosition : 0);
			Positions[NodeIndex] = World;
		}
	}

	/** 질량 덮어쓰기(마스크/복원). */
	void SetInvMass(int32 NodeIndex, float Value)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::InvMass;
			InvMass[NodeIndex] = Value;
		}
	}

	/** 속도 제거만(Prev=현재 Pos — 위치는 그대로). release 계열의 튐 방지. */
	void SetPrevFromPosition(int32 NodeIndex)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::PrevFromPosition;
		}
	}

	/** CPU 적용 — GPU 커널의 override 스테이지와 같은 순서(Pos → Prev → Prev=Pos → InvMass). */
	void ApplyToSim(FRopeSimState& Sim) const
	{
		const int32 N = FMath::Min(Flags.Num(), Sim.Num());
		for (int32 i = 0; i < N; ++i)
		{
			const uint8 F = Flags[i];
			if (F == 0)
			{
				continue;
			}
			if (F & RopeNodeOverride::Position)         { Sim.Positions[i] = Positions[i]; }
			if (F & RopeNodeOverride::Prev)             { Sim.PrevPositions[i] = PrevPositions[i]; }
			if (F & RopeNodeOverride::PrevFromPosition) { Sim.PrevPositions[i] = Sim.Positions[i]; }
			if (F & RopeNodeOverride::InvMass)          { Sim.InvMass[i] = InvMass[i]; }
		}
	}
};

/** XPBD solver 튜닝(디자이너용). */
USTRUCT(BlueprintType)
struct FRopeSolverConfig
{
	GENERATED_BODY()

	/** 프레임당 물리 substep 수(anti-tunneling; "small steps"가 iteration을 늘리는 것보다 낫다). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "1", ClampMax = "16"))
	int32 Substeps = 12;

	/** substep당 constraint iteration 수. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "1"))
	int32 Iterations = 4;

	/** substep당 충돌 해소 패스 수. 1=substep 끝에 1회(기존 동작, perf 무회귀). sharp한 굴곡에서
	 *  distance/bending이 안쪽으로 당기는 힘을 단일 충돌이 못 이겨 관통할 때, 제약 iteration을 이 수만큼
	 *  나눠 사이사이 충돌을 끼운다 → 더 sharp한 끼인각까지 방어(>1일수록 강하지만 비용↑). Iterations로 상한. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Solver", meta = (ClampMin = "1", ClampMax = "16"))
	int32 CollisionPassesPerSubstep = 1;

	/** XPBD stretch compliance(stiffness의 역수). 0 = 신장 불가(inextensible). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0"))
	float StretchCompliance = 0.0f;

	/** XPBD bending compliance. 클수록 더 흐물거린다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0"))
	float BendCompliance = 0.02f;

	/** 각도-허용 벤딩: 굽힘이 급할수록 펴는 힘을 놔준다(코너/랩 경계에서 free 노드가 각지게 튀는 것 완화).
	 *  판정값 r = (i↔i+2 거리)/(2*SegmentLength) = cos(턴각/2): 1=직선, 작을수록 급한 굽힘.
	 *  r ≤ BendReleaseRatio면 펴는 힘 0(완전히 놔줌), r ≥ BendFullRatio면 100%(기존 동작), 사이는 smoothstep.
	 *  기본 0.70(≈턴각 91°). 코너가 아직 각지면 올리고(급한 굴곡까지 놔줌), 두 값을 0으로 두면 항상 편다(각도 허용 끔). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BendReleaseRatio = 0.70f;

	/** 각도-허용 벤딩: r ≥ 이 값이면 펴는 힘 100%(완만한 굽힘은 기존처럼 곧게 편다). 기본 0.92(≈턴각 46°).
	 *  자유 로프가 너무 흐물거리면 낮추고, 항상 BendReleaseRatio 이상이어야 한다(솔버가 내부적으로 보장). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BendFullRatio = 0.92f;

	/** collider에 대한 접선 방향 friction [0..1](Coulomb 계수 μ). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Friction = 0.5f;

	/** 자유단(끝)으로 갈수록 friction을 약화시키는 배율(고정점=1, 끝=이 값). 끝 노드는 장력이 가장 낮아
	 *  마찰에 잘 붙잡히므로, 끝쪽 그립만 낮춰 잘 놔주게 한다. 1.0이면 테이퍼 없음(균일 friction). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TipFrictionScale = 1.0f;

	/** Collision query radius in cm. The solver keeps nodes this far off contact surfaces. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", Units = "cm"))
	float CollisionRadius = 2.0f;

	/**
	 * 엔진 Global Distance Field로 정적 월드 지오메트리(벽/바닥)에서 로프를 밀어낸다. GPU 경로(씬 그래프
	 * dispatch)에서만 동작. 프로젝트에 Generate Mesh Distance Fields 필요.
	 * 본 귀속·표면속도 없음(정적 월드 광역 밀어내기 보완재) — per-bone SDF의 대체가 아니다. 켜져 있는 동안
	 * 엔진이 GDF를 온디맨드로 빌드한다. 밀어내기 반경/마찰은 CollisionRadius/Friction/TipFrictionScale 공유.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver")
	bool bUseWorldGDF = false;

	/** Swept collision sample spacing in cm. Lower values reduce tunneling at higher query cost. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Solver", meta = (ClampMin = "0.1", Units = "cm"))
	float SweepStep = 2.0f;

	/** Maximum swept samples per segment, used as a cost cap for very fast nodes. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Solver", meta = (ClampMin = "1", ClampMax = "64"))
	int32 MaxSweepSamples = 16;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.02f;

	//~ 스케일링(슬립/LOD) — 다수 로프가 존재할 때 유휴/원거리 비용을 줄인다 ------------------

	/**
	 * 슬립: Free 페이즈에서 모든 노드 속도가 SleepVelocityThreshold 미만으로 SleepDelay 동안 유지되면
	 * 솔브를 통째로 스킵한다(GPU 로프는 dispatch 자체가 없음). 핀 이동/되감기/움직이는 collider 근접
	 * /페이즈 전환에서 깨어난다. 다른 페이즈(Flight~Releasing)는 항상 활성.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Scaling")
	bool bAllowSleep = true;

	/** 슬립 진입 판정 속도(cm/s) — 프레임간 최대 노드 변위 / dt가 이 값 미만이어야 한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Scaling", meta = (ClampMin = "0.1"))
	float SleepVelocityThreshold = 3.0f;

	/** 슬립 진입까지 저속 상태가 유지되어야 하는 시간(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Scaling", meta = (ClampMin = "0.0", Units = "s"))
	float SleepDelay = 0.5f;

	/**
	 * 거리 LOD: 플레이어 카메라와의 거리가 LODStartDistance를 넘으면 constraint iteration을 줄이기
	 * 시작해 LODEndDistance에서 LODMinIterationScale까지 선형 감소한다(멀리서는 수렴 오차가 안 보임).
	 * 카메라가 없으면(데디 서버) 항상 풀 iteration.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Scaling")
	bool bEnableDistanceLOD = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Scaling", meta = (ClampMin = "0.0", Units = "cm"))
	float LODStartDistance = 3000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Scaling", meta = (ClampMin = "0.0", Units = "cm"))
	float LODEndDistance = 8000.0f;

	/** 최원거리에서의 iteration 배율(1=감소 없음). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver|Scaling", meta = (ClampMin = "0.05", ClampMax = "1.0"))
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

	/** 컨택트 결정 query에 사용하는 노드 반지름(cm). 시각용 튜브 반지름과는 별개. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm"))
	float ContactRadius = 3.0f;

	/** 스치는 접촉이 아니라 catch로 간주하기 위해 한 bone에 닿아야 하는 최소 rope 노드 수. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "1"))
	int32 MinLatchNodes = 1;

	/** wrap을 확정하기 전에 컨택트가 같은 bone에서 이만큼 지속되어야 한다(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrapDecisionTime = 0.016f;

	/**
	 * 한 번의 캡처에서 채택할 수 있는 wrap 시드(접촉 대상) 최대 개수. 1(기본) = 기존 단일 시드 동작.
	 * 2 이상이면 dominant 대상 외에, dominant latch보다 tail 쪽에서 *다른* (mesh, bone)에 dwell을
	 * 채운 접촉이 보조 시드로 함께 감긴다(예: 양다리 — 한쪽 다리를 감고 반대쪽 다리 접촉 노드도
	 * 그 본에 고정). 감김 경로(나선)는 dominant 시드에만 생성되고, 보조 시드는 접촉 노드를 자기
	 * 본에 hold하는 방식이다 — 보조 대상 둘레를 도는 경로까지 만들지는 않는다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap", meta = (ClampMin = "1", ClampMax = "8"))
	int32 MaxWrapSeeds = 1;

	/**
	 * Wrapping 경로 생성 방식(AnalyticHelix / SurfaceVectorField). 종전에는 프로젝트 전역
	 * (UDynamicRopeSettings) 설정이었으나 로프별 값으로 이동했다(2026-07-13 회의 결정 G-마이그레이션,
	 * 묵은 "per-rope화" P2 정리) — 전역 필드는 제거됨, 기존 전역 튜닝은 승계하지 않는 클린 브레이크.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap")
	ERopeWrappingPathMode WrappingPathMode = ERopeWrappingPathMode::SurfaceVectorField;

	/**
	 * 감김 축 유도 소스. ShapeAxisFirst(기본) = 기존 동작(latch 본 collider 장축 우선).
	 * TravelPlaneFirst = 로프 진행(스윙) 평면 normal을 축으로 앞세운다 — 여러 본에 걸친 랩(양다리)
	 * 대비 진행 방향 기반 wrap의 1단계. 상세는 ERopeWrappingAxisSource 주석.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap")
	ERopeWrappingAxisSource WrappingAxisSource = ERopeWrappingAxisSource::ShapeAxisFirst;

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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm"))
	float WrappingMaxGapBridgeDistance = 0.0f;

	/**
	 * 감는 양 상한(도): SurfaceVectorField 경로 빌드의 누적 감싼 각도(rolling axis 적분 —
	 * FRopeWrappingState::PathAccumulatedAngleRad)가 이 값에 닿으면 경로를 *성공*으로 조기 마감한다.
	 * 0(기본) = 무제한 — 남은 로프 전량이 감길 때까지 진행(기존 동작).
	 * 긴 로프가 대상을 여러 바퀴 나선으로 감아 들어가며(실측 3000~4400°) 본 전환 재시드가 목/머리
	 * 등으로 번지는 "문어발 랩"의 방지책. 상한에서 마감된 경로 밖의 남는 로프는 Wrapping 동안
	 * 동결됐다가 커밋 후 자유 구간으로 늘어진다(front 모션도 경로 밖 노드는 끌지 않는다).
	 * 양다리 bola면 400~540°(한 바퀴 + 여유)가 자연스럽다.
	 * AnalyticHelix 모드에는 적용되지 않는다(각도 적분이 SurfaceVectorField 전용).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "deg"))
	float WrappingMaxWrapAngleDeg = 0.0f;

	/** Wrapping phase must keep the same accumulated latch span stable this long before committing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrappingStableTime = 0.10f;

	/** Time used to pull tail nodes onto their generated surface wrap targets. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.01", Units = "s"))
	float WrappingMotionDuration = 0.50f;

	/** Per-segment delay while tail nodes settle onto the surface path. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrappingTailDelayPerSegment = 0.024f;

	/** Wrapping 중 한 프레임에 진행할 surface path 적분 step 수. 높이면 빨라지지만 순간 비용이 커진다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "1", ClampMax = "256"))
	int32 WrappingPathBuildStepsPerFrame = 8;

	/** Axis distance advanced per circumference distance for analytic helix wrapping. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "-2.0", ClampMax = "2.0"))
	float WrappingHelixPitchScale = 0.25f;

	/**
	 * SurfaceVectorField path point가 latch bone 하나에 고정되지 않고 graph 후보 본으로 넘어갈지 여부.
	 * false면 후보 graph depth/cost가 0이 되어 현재 본만 평가하므로 기존 단일 본 동작에 가깝게 돌아간다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap|MultiBone")
	bool bEnableMultiBoneWrapping = true;

	/**
	 * 현재 본에서 몇 edge까지 후보로 볼지.
	 * 지금은 skeleton parent/child edge만 사용한다. 이후 디자이너 지정 transition을 추가해도
	 * 같은 depth 제한을 통과하므로, 너무 먼 bridge가 한 번에 열리는 것을 막는 1차 안전장치다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0", ClampMax = "16"))
	int32 MaxBoneTransitionDepth = 3;

	/**
	 * 후보 graph 누적 비용 상한.
	 * depth가 같아도 edge별 penalty가 다르면 비용이 달라질 수 있다. 지금은 parent/child edge 비용만
	 * 누적하지만, 나중에 designer edge / 금지에 가까운 edge를 섞을 때 projection 전에 후보를 잘라내는 역할을 한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float MaxBoneTransitionCost = 5.0f;

	/**
	 * 자동 parent/child edge 하나를 지날 때의 비용.
	 * 값이 클수록 graph cost가 커져 같은 본 유지가 쉬워지고, 낮추면 parent/child chain을 더 적극적으로 탄다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float AutoParentChildTransitionPenalty = 1.0f;

	/** projection 거리 점수 가중치. 예측 위치에서 표면까지 멀수록 불리하다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float ProjectionDistanceWeight = 0.35f;

	/** 실제 rope node 위치와 projection 표면점 사이 거리 가중치. 로프가 실제로 있는 쪽의 본을 선호한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float RopeNodeDistanceWeight = 0.25f;

	/** 이전 tangent와 새 tangent가 꺾이는 정도의 가중치. 값이 클수록 부드러운 진행을 선호한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float TangentContinuityWeight = 8.0f;

	/** 이전 normal과 새 normal이 꺾이는 정도의 가중치. 값이 클수록 표면 normal 연속성을 선호한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float NormalContinuityWeight = 5.0f;

	/** graph 비용 가중치. parent/child를 많이 건너는 후보일수록 불리하게 만든다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float BoneTransitionPenaltyWeight = 1.0f;

	/** 현재 본 유지 보너스. 동점 근처에서 본이 흔들리는 것을 줄인다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float CurrentBoneBonus = 0.35f;

	/** 새 본이 현재 본보다 이 점수만큼 더 좋아야 전환한다. 전환 hysteresis. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float BoneTransitionHysteresis = 0.75f;

	/** 직전 본으로 바로 돌아가는 후보에 더하는 penalty. A->B->A 왕복을 줄인다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0"))
	float ImmediateBoneReturnPenalty = 1.5f;

	/** 마지막 본 전환 이후 이 거리(cm) 이상 진행해야 다음 전환을 허용한다. 0이면 비활성. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap|MultiBone", meta = (ClampMin = "0.0", Units = "cm"))
	float MinBoneTransitionPathDistance = 8.0f;

	/** Upper bound for physics-based wrapping settle before committing the best accumulated anchors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrappingMaxSettleTime = 0.90f;

	/**
	 * 경로 생성이 실패한 wrap의 최소 감싼 각도(도). 실패 시점까지 감은 각도가 이 값 미만이면 "조금
	 * 닿았는데 철썩 붙는" 커밋 대신 release한다. 0 = 가드 끔.
	 * 각도 기준인 이유(이전 constexpr "최소 1바퀴" 기준 대체): 한 바퀴는 로프 2πr을 요구해 대상이
	 * 클수록 절대 길이가 폭증한다 — 반지름 100cm 몸통은 한 바퀴에 628cm로 기본 로프(200cm)로는
	 * 물리적으로 불가능해 큰 대상 wrap이 구조적으로 전멸했다. 감싼 각도는 대상 크기와 무관한
	 * "걸림 품질" 척도다(120° = 1/3바퀴 훅).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg"))
	float FailedWrapMinAngleDeg = 120.0f;

	/**
	 * 커밋 품질 하한(도): Wrapped로 커밋되는 *모든* wrap(경로 완료/실패/settle 타임아웃 불문)의 감싼
	 * 각도가 이 값 미만이면 커밋 대신 release한다. 0(기본) = 끔 — 기존 동작 그대로.
	 * FailedWrapMinAngleDeg와의 차이: 그쪽은 "경로 생성이 실패한" wrap 전용 조기 abort, 여기는 커밋
	 * 직전 최종 관문. 경로가 정상 완료돼도 latch가 팁 근처면 경로가 짧아(감은 각도 미미) 철썩 붙는
	 * 커밋이 나올 수 있고, settle 타임아웃 커밋은 앵커 1개로도 통과한다 — 그런 부실 랩을 게임
	 * 규칙으로 거르고 싶을 때 opt-in으로 켠다(팁 살짝 걸침도 유효한 디자인이면 0 유지).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg"))
	float CommitMinWrapAngleDeg = 0.0f;

	/**
	 * 형상 기준 묶임 관문(도): 커밋되는 wrap 경로의 감김 축 둘레 각도 커버리지(경로점 각도들을 정렬해
	 * 360° − 최대 공백)가 이 값 미만이면 커밋 대신 release한다. 0(기본) = 끔 — 기존 동작 그대로.
	 * CommitMinWrapAngleDeg(누적 각도)와의 차이: 누적 각도는 걸은 회전량의 합이라 표면 위 진동/왕복이
	 * 값을 부풀릴 수 있고 여러 바퀴면 360°를 넘는다. 커버리지는 "축 둘레 어느 방향까지 로프가 실제로
	 * 둘러쌌는가"의 순수 기하 척도(0~360°)라 진동에 면역이다 — 대상이 정말 갇혔는지(양다리 bola처럼
	 * 빠져나갈 공백이 없는지)를 묻는 판정. 축이 캡처 시점에 고정되는 TravelPlaneFirst 감김에서 가장
	 * 의미가 정확하다(ShapeAxisFirst의 rolling axis에서는 마지막 축 기준 근사).
	 * 양다리 잠금 용도면 300° 안팎, 느슨한 훅도 허용하려면 0 유지.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "360.0", Units = "deg"))
	float CommitMinWrapCoverageDeg = 0.0f;

	/** [미배선] Wrapping 중 일시적 접촉 상실을 이만큼 유예한다는 의도였으나, 소비하는 코드가 아직 없다.
	 *  현재 Wrapping 중단 판정은 IsStillValid(mesh 생존/bone 유효)와 경로 빌드 실패 경로뿐 — 값을 바꿔도
	 *  아무 효과가 없다. grace 로직을 실제로 배선하기 전까지 튜닝 대상이 아니다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrappingContactGraceTime = 0.20f;

	/** Extra Flight lookahead in frame-displacements for thin limb/SDF candidate detection. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "4.0"))
	float PredictiveContactFrames = 1.0f;

	/** Whip 종료 후 이 시간 동안 캡처하지 못하면 Free로 복귀한다. 0이면 기본 실패 복귀 쿨다운을 쓴다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float FlightNoContactReturnTime = 0.0f;

	/**
	 * Wrapped 중 로프 최대 장력(FRopeSimState::SegmentTension 단위 — 질량 1 노드 기준 상대 힘)이 이 값을
	 * TensionReleaseTime 동안 지속해서 넘으면 자동 release한다(ERopeReleaseReason::Tension). 0 = 비활성.
	 * 값 감: 매달린 노드 1개의 중력 하중이 약 980이므로, 로프 전체 무게의 몇 배를 버틸지로 잡는다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0"))
	float TensionReleaseForce = 0.0f;

	/** 장력 release 판정의 지속 시간(초). 순간 스파이크(충격 프레임)로 풀리는 것을 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float TensionReleaseTime = 0.05f;

	/**
	 * 자동 견인(테더) 반응 [0..1]. Wrapped 중 손~앵커 직선 거리가 가용 로프 길이(앵커까지 세그먼트 수 ×
	 * SegmentLength + TetherSlack)를 넘으면 견인이 켜진다. 0 = 비활성(기본).
	 * 견인 *속도* 는 TetherReelSpeed가 정하고, 이 값은 **보정 강성(임계 감쇠)** — 로프 축 속도를 목표로 매
	 * 프레임 이 비율만큼만 접근시킨다. 1 = 즉시(하드 — 진행 속도를 뚝 끊어 "턱턱"), 작을수록(예 0.1~0.3)
	 * 몇 프레임에 걸쳐 부드럽게 감속. 크기는 TetherReelSpeed, 부드러움은 이 값으로 역할이 나뉜다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TetherResponse = 0.0f;

	/** 테더 발동 전 허용 여유(cm). 경계 지터/미세 슬랙에서 발동하는 것을 막는다(가용 길이에 더해짐). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm"))
	float TetherSlack = 5.0f;

	/**
	 * 팽팽한 동안 테더가 대상/wielder를 로프 쪽으로 되돌리는 *고정* 견인 속도(cm/s). overshoot가 TetherSettleDist보다
	 * 크면 항상 이 속도로 당기고(마스 분배로 양끝에 ShareT:ShareW 비율로 나뉨), 한계 근처에선 부드럽게 감속해
	 * 안착한다. 속도 ∝ overshoot가 아니라 고정이라 견인이 일정하다. 0 = 견인 없음.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm/s"))
	float TetherReelSpeed = 400.0f;

	/**
	 * 경계 근처 감속(taper) 구간(cm). overshoot가 이 값보다 작아지면 견인 속도가 0으로 선형 감속해 로프 한계에
	 * 부드럽게 안착한다(임계 감쇠). 작을수록 작은 overshoot에서도 곧바로 고정 ReelSpeed(빠른 견인)에 도달하고
	 * 마지막 이 구간만 감속 — 너무 크면 평소 드래그(overshoot가 작음)가 내내 감속 구간에 들어가 견인이 느려진다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap", meta = (ClampMin = "0.01", Units = "cm"))
	float TetherSettleDist = 1.5f;

	/**
	 * 테더 회수 최대 속도(cm/s) — 안전 상한. TetherReelSpeed가 이보다 크면 이 값으로 클램프해 초과분 스파이크에도
	 * 대상이 튕겨나가지 않게 한다(수렴 보장). 0 = 클램프 없음(비권장).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm/s"))
	float TetherMaxSpeed = 1500.0f;

	/**
	 * 테더 회수 분배를 자동으로 정할지(기본 켜짐). 켜면 양끝의 유효 역질량(w=1/유효질량)으로 초과분을
	 * 나눈다 — 무거울수록/앵커일수록 덜 움직인다(PBD 역질량 가중과 동일). 접지 캐릭터는 무한이 아니라
	 * 유한 브레이스(질량 × GroundBraceFactor)로 저항하고, 공중이면 그냥 질량, MOVE_None/정적 비시뮬은
	 * 앵커(무한질량). 물리 질량은 UE가 자동 유지하므로 별도 세팅이 필요 없다. 끄면 아래 TetherTargetShare
	 * 고정 비율을 쓴다(오버라이드가 자동보다 항상 우선).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	bool bAutoTetherShare = true;

	/**
	 * 접지(발 디딘) 캐릭터가 자기 Mass의 몇 배까지 마찰로 버티는가(유효질량 = Mass × 이 값). 클수록 단단히
	 * 버텨 무거운 대상도 잘 끌고, 작을수록 쉽게 끌려간다. "대상이 얼마나 무거워야 접지한 나를 끌기
	 * 시작하는가"의 교차점을 정하는 유일한 튜닝 노브 — 기본값으로 대부분 무설정. bAutoTetherShare 전용.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap", meta = (ClampMin = "1.0", EditCondition = "bAutoTetherShare"))
	float GroundBraceFactor = 4.0f;

	/**
	 * (bAutoTetherShare=false일 때만) 테더 회수 고정 분배: 초과분 중 감긴 *대상*이 회수하는 비율.
	 * 1(기본) = 전량 대상(질량 무관 강제 — wielder가 대상을 전부 끌고 옴, 대상만 이동), 0 = 전량 wielder
	 * (로프 owner가 앵커 쪽으로 끌려간다 — 고정 앵커 매달리기/등반, 되감기와 조합하면 입체기동식 "감으면
	 * 끌려 올라감"), 중간 = 비율 분할(양끝이 서로에게 끌린다). 양끝 이동의 합이 초과분을 넘지 않아 과수렴이
	 * 없다. 자기 자신에 감긴 로프(owner==대상)는 무시하고 전량 대상 경로.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "!bAutoTetherShare"))
	float TetherTargetShare = 1.0f;

	/**
	 * 거리 release: Wrapped 중 손~앵커 직선 거리가 가용 로프 길이(+TetherSlack)를 이만큼(cm) 더
	 * 초과하면 자동 release한다(ERopeReleaseReason::Distance). 0 = 비활성(기본). 테더와 함께 쓰면
	 * "테더가 버티다가 이 한계를 넘으면 놓친다"가 된다 — 테더가 충분히 강하면 초과분이 안 쌓여
	 * 발동하지 않고, 테더 없이 쓰면 순수 거리 제한으로 동작한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm"))
	float DistanceReleaseSlack = 0.0f;

	/**
	 * Pull 방향 코너 판정 임계(도). 당김 방향을 앵커→손 직선(chord)이 아니라, 앵커에서 손 쪽으로 로프를
	 * 따라 걸으며 찾은 "첫 직선 다리"의 끝 노드를 향하도록 잡는다 → 로프가 벽/모서리에 걸려 꺾이면 그
	 * 직전에서 멈춰 첫 다리를 따라 당긴다(직선 chord는 장애물을 관통). 걷는 중 다음 세그먼트가 지금까지의
	 * 누적 다리 방향에서 이 각도 이상 꺾이면 코너로 보고 멈춘다 — 곧으면 손(노드 0)까지 걸어가 정확히
	 * chord가 된다. 크게 잡으면(완만한 굴곡 무시) 더 chord에 가깝고, 작게 잡으면 미세한 꺾임에도 민감.
	 * 팽팽할 때의 처짐/노드 지터는 이 임계 아래이고, 벽 모서리는 위라 구분된다(잔여 지터는 SmoothTime이 흡수).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "1.0", ClampMax = "179.0", Units = "deg"))
	float PullBendThresholdDeg = 30.0f;

	/**
	 * Pull 방향 시간 스무딩 상수(초, EMA time constant). look-ahead 방향의 프레임 간 지터 + GPU 미러 지연
	 * 노이즈를 지수이동평균으로 흡수한다(alpha = 1-exp(-dt/이 값), 프레임레이트 독립). 클수록 매끄럽지만
	 * 반응이 느리고, 0이면 스무딩 없음(원 look-ahead). wrap 시작 시 측정값으로 초기화된다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float PullDirSmoothTime = 0.08f;

	/**
	 * Pull 조준 노드 시간 스무딩 상수(초, EMA time constant). walk가 고른 정수 조준 노드(AimNode)는 로프가
	 * 흔들리면 프레임마다 이산적으로 튀어(방향 통째 점프 + tether 초과분 불연속 = 견인 끊김) 방향 EMA로는
	 * 못 잡는다. 조준 인덱스를 float로 EMA해 노드 사이를 보간하면 방향·tether가 연속이 된다(alpha=1-exp(-dt/이
	 * 값), 프레임레이트 독립). 클수록 매끄럽지만 반응이 느리고, 0이면 스무딩 없음. wrap 시작 시 측정값으로 초기화.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float PullAimSmoothTime = 0.08f;
};

/** 던질 때 기준축을 어느 좌표계에서 가져올지. */
UENUM(BlueprintType)
enum class ERopeThrowFrameMode : uint8
{
	World = 0 UMETA(DisplayName = "World"),
	Owner = 1 UMETA(DisplayName = "Owner"),
	OwnerCamera = 3 UMETA(DisplayName = "Owner Camera"),
	Socket = 2 UMETA(DisplayName = "Socket"),
	Custom = 4 UMETA(DisplayName = "Custom")
};

/** AimDir과 조합해 스윙 호가 놓일 평면/방향을 고르는 5개 드롭다운. */
UENUM(BlueprintType)
enum class ERopeSwingPlane : uint8
{
	AimAndFrameUp UMETA(DisplayName = "Aim + Frame Up"),
	AimAndFrameDown UMETA(DisplayName = "Aim + Frame Down"),
	AimAndFrameRight UMETA(DisplayName = "Aim + Frame Right"),
	AimAndFrameLeft UMETA(DisplayName = "Aim + Frame Left"),
	CustomNormal UMETA(DisplayName = "Custom Plane Normal")
};

// 아래 정의 — MakeDefault가 설정 스냅샷으로 받는다.
struct FRopeThrowParams;

/** throw 순간 Wielder/Component가 계산해 넘기는 런타임 값. 설정값(FRopeThrowParams)과 분리한다. */
USTRUCT(BlueprintType)
struct FRopeThrowContext
{
	GENERATED_BODY()

	/**
	 * 컴포넌트 트랜스폼 + 던지기 설정에서 기본 컨텍스트를 조립한다(throw당 1회, GT).
	 * URopeComponent::Throw(AimDir) 편의 진입점의 기본 구현이 사용한다 — Wielder처럼 컨텍스트를
	 * 직접 만드는 호출자는 무관. 프레임 기저 규약(FrameMode별):
	 *   World = 월드 축 · Owner/Socket = 컴포넌트 기저 · OwnerCamera = owner의 첫 카메라
	 *   (없으면 컴포넌트 기저 폴백) · Custom = Params의 커스텀 축(원값 — 정규화/직교
	 *   폴백은 ResolveThrowContext 책임). 구현은 RopeTypes.cpp.
	 */
	static FRopeThrowContext MakeDefault(const USceneComponent& RopeComponent, const FRopeThrowParams& Params);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector Origin = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameForward = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameUp = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameRight = FVector::RightVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector OwnerVelocity = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector SocketVelocity = FVector::ZeroVector;

	/** Wielder가 소유하는 던지기 속도. 0 이하이면 RopeComponent의 fallback 값을 사용한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float ThrowSpeed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeThrowFrameMode FrameMode = ERopeThrowFrameMode::Owner;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeSwingPlane SwingPlane = ERopeSwingPlane::AimAndFrameUp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector CustomSwingPlaneNormal = FVector::RightVector;

	/** AimRayHitDirection에서 유효한 본 hit을 확보했는지 나타낸다. */
	bool bHasAimGuideHit = false;

	/** Flight 이후 wrap을 허용할 대상 본이다. 다른 본 접촉은 궤적을 유지한 채 무시한다. */
	FName AimGuideBone = NAME_None;

	/** 대상 본의 transform과 SDF를 해석할 mesh/component이다. */
	TWeakObjectPtr<const USceneComponent> AimGuideMesh = nullptr;

	/** ray 중심선이 처음 target SDF 안으로 들어간 월드 위치이다. */
	FVector AimGuideHitWorldPos = FVector::ZeroVector;

	/** SDF 투영으로 구한 실제 표면점이다. */
	FVector AimGuideSurfacePoint = FVector::ZeroVector;

	/** 표면점에서 얻은 바깥쪽 법선이다. */
	FVector AimGuideNormal = FVector::UpVector;

	/** ray origin부터 hit까지 거리이며 preview 후보 노드 선택에 사용한다. */
	float AimGuideDistance = 0.0f;

	/**
	 * 로프 길이상 hit 방향 보간을 시작/완료할 구간이다.
	 * 공간 보간은 Flight 시간 보간과 곱하므로 throw가 끝나기 전에 hit 방향에 고정되는 노드는 없다.
	 */
	float AimGuideSteerStartAlpha = 0.25f;
	float AimGuideLockAlpha = 0.50f;
};

/** 던지기 전 미리보기 호를 정의하는 런타임 데이터. 렌더 컴포넌트는 이 값만 받아 그린다. */
USTRUCT(BlueprintType)
struct FRopeArcPreviewData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	FVector Origin = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	FVector AimDir = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	FVector GuideUp = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "0.0", Units = "cm"))
	float Radius = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "1.0", ClampMax = "180.0", Units = "deg"))
	float SweepAngleDegrees = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "1", ClampMax = "128"))
	int32 SegmentCount = 32;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	bool bBlocked = false;

	/** 0~1. 이 각도 비율부터 blocked material로 그린다. 1이면 막힌 구간 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BlockedStartAlpha = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	FVector HitPoint = FVector::ZeroVector;
};

/** Runtime centerline data for the pre-wrapped rope preview. */
USTRUCT(BlueprintType)
struct FRopeWrapPreviewData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	TArray<FVector> Points;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "0.1", Units = "cm"))
	float Radius = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "3", ClampMax = "32"))
	int32 NumSides = 8;

	bool IsValid() const
	{
		return Points.Num() >= 2 && Radius > KINDA_SMALL_NUMBER;
	}
};

/** 미리보기 호가 현재 collider 스냅샷에 닿았는지와, 닿은 각도 비율. */
USTRUCT(BlueprintType)
struct FRopeArcPreviewHitResult
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Preview")
	bool bHit = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Preview")
	FVector HitPoint = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Preview")
	float AngleAlpha = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Preview")
	float DistanceAlpha = 1.0f;
};

/** flight 단계의 Throw / launch 파라미터. */
USTRUCT(BlueprintType)
struct FRopeThrowParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeThrowFrameMode FrameMode = ERopeThrowFrameMode::Owner;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeSwingPlane SwingPlane = ERopeSwingPlane::AimAndFrameUp;

	/** Wielder를 거치지 않고 RopeComponent::Throw를 직접 호출할 때 쓰는 fallback 속도. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float ThrowSpeed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.1"))
	float TipMass = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float OwnerVelocityScale = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float SocketVelocityScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameForward = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameUp = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameRight = FVector::RightVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "SwingPlane == ERopeSwingPlane::CustomNormal"))
	FVector CustomSwingPlaneNormal = FVector::RightVector;
};

/** 던지기 초반 채찍 스윙(FRopeWhipGuide) 튜닝 값. 런타임 상태는 URopeComponent::WhipGuide가 소유한다. */
USTRUCT(BlueprintType)
struct FRopeWhipConfig
{
	GENERATED_BODY()

	/** 스윙 전체 시간(s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.01", ClampMax = "1.0", Units = "s"))
	float Duration = 0.35f;

	/** 가이드가 잡는 로프 길이 비율(0~1). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.1", ClampMax = "0.95"))
	float GuidedLength = 0.65f;

	/** 시작 각도(조준 반대편)에서 조준 방향까지의 스윕 각. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "1.0", ClampMax = "180.0", Units = "deg"))
	float SweepAngleDegrees = 180.0f;

	/** Aim-hit Flight에서 손 쪽 guide를 solver에 넘기는 로프 길이 비율. 0이면 중앙 spline이 손 바로 옆까지 지배한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip|Aim Hit", meta = (ClampMin = "0.0", ClampMax = "0.45"))
	float AimHitRootSolverFraction = 0.20f;

	/** Hit direction 보간 편향. 1은 선형 강도, 클수록 같은 Flight 시점에서 spline이 더 빨리 hit 방향을 향한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip|Aim Hit", meta = (ClampMin = "1.0", ClampMax = "4.0"))
	float AimHitDirectionBias = 2.0f;

	/** Aim-hit Flight에서 자유단 쪽 guide를 solver에 넘기는 로프 길이 비율. 클수록 끝이 더 관성적으로 움직인다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip|Aim Hit", meta = (ClampMin = "0.0", ClampMax = "0.45"))
	float AimHitTipSolverFraction = 0.25f;

	/** Aim-hit Flight에서 거리/굽힘/감쇠 solver는 유지하고 collider push-out만 끈다. 접촉 감지는 계속 동작한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip|Aim Hit")
	bool bAimHitCollisionFreeSolve = true;

	/** 현재 런타임 미사용 — 에디터 배치 가이드(FRopeComponentVisualizer)의 던지기 아크 표시 전용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.0", Units = "cm"))
	float ArcHeight = 120.0f;

	/** 현재 런타임 미사용 — 에디터 배치 가이드(FRopeComponentVisualizer)의 던지기 아크 표시 전용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.0", Units = "cm"))
	float SideOffset = 35.0f;
};

/**
 * Flight 접촉 후보의 출처(비트 조합 가능 — 같은 노드×본이 여러 경로로 잡히면 SourceMask에 OR).
 * Actual = 이번 프레임 이동 경로(Prev→Pos)의 실제 스윕 접촉,
 * PredictiveFree = 자유 노드의 관성 외삽 예측 접촉, PredictiveGuided = whip 가이드 타깃 외삽 예측 접촉.
 */
enum class ERopeContactCandidateSource : uint8
{
	Actual = 1,
	PredictiveFree = 2,
	PredictiveGuided = 4
};

/** Flight/Contacting이 소비하는 접촉 후보 1건(노드×본). FRopeContact + 상대운동 평가 산출물. */
struct FRopeContactCandidate
{
	bool bValid = false;
	int32 NodeIndex = INDEX_NONE;
	FName Bone = NAME_None;
	const USceneComponent* Mesh = nullptr;
	ERopeContactCandidateSource Source = ERopeContactCandidateSource::Actual;
	uint8 SourceMask = static_cast<uint8>(ERopeContactCandidateSource::Actual);

	FVector WorldPoint = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	FVector SurfaceVelocity = FVector::ZeroVector;

	float Penetration = 0.0f;
	float RelativeTangentialSpeed = 0.0f;

	/** 감김 방향이면 +, 반대면 -. */
	float WrapDirectionScore = 0.0f;
};

/** Wielder의 PreviewPathLocked 흐름이 입력 순간 확정하는 prepared preview(렌더 + GuidedThrow/Wrapped 진입 재료). */
struct FRopePreparedThrowPreview
{
	bool bValid = false;

	/** preview를 만들 때 쓴 throw 기준. montage가 있어도 입력 순간의 frame/origin을 보존하기 위해 저장한다. */
	FRopeThrowContext ThrowContext;

	/** 화면에 보이는 preview centerline. GuidedThrow에서는 이 점들을 실제 노드 목표 위치로도 사용한다. */
	FRopeWrapPreviewData RenderPreview;

	/** AimRayHitDirection처럼 소켓 애니메이션에서 독립시킬 필요가 있는 path는 owner 기준 로컬로도 보관한다. */
	bool bUseGuideFrameLocal = false;
	TWeakObjectPtr<const USceneComponent> GuideFrameComponent = nullptr;
	TArray<FVector> GuideFrameLocalPoints;
	FVector GuideFrameLocalOrigin = FVector::ZeroVector;

	/** preview build 시 만든 가상 로프 상태와 접촉 후보. 디버그/후속 고도화용으로 보존한다. */
	FRopeSimState PreviewSim;
	FRopeContactCandidate Contact;

	/** 최종 Wrapped 진입에 필요한 bone-local 고정 정보. Points만으로는 캐릭터 움직임을 따라갈 수 없다. */
	FRopeSurfaceAnchor LatchAnchor;
	TArray<FRopeSurfaceAnchor> Anchors;

	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;
	FName Bone = NAME_None;
	double BuildTimeSeconds = 0.0;

	void Reset()
	{
		*this = FRopePreparedThrowPreview();
	}

	bool IsValid() const
	{
		return bValid && RenderPreview.IsValid() && Mesh.IsValid() && !Bone.IsNone() && LatchAnchor.NodeIndex != INDEX_NONE;
	}

	/** 생성 당시 월드 preview를 wielder owner 기준 로컬 좌표로 저장해 소켓 애니메이션에서 분리한다. */
	void StoreGuideFrameLocal(const USceneComponent* InGuideFrame);

	/** owner-local guide frame과 로컬 점 데이터가 모두 유효한지 확인한다. */
	bool HasGuideFrameLocal() const;

	/** 저장한 owner-local origin을 현재 owner transform 기준 월드 좌표로 복원한다. */
	FVector ResolveGuideOriginWorld() const;

	/** 지정한 owner-local spline 점을 현재 owner transform 기준 월드 좌표로 복원한다. */
	FVector ResolveGuidePointWorld(int32 PointIndex) const;

	/** 렌더용 전체 preview를 현재 owner transform에 맞춘 월드 데이터로 해석한다. */
	FRopeWrapPreviewData ResolveRenderPreviewWorld() const;
};

/** GuidedThrow 페이즈의 작업 상태: cached preview path를 authoritative하게 따라가는 진행분. */
struct FRopeGuidedThrowState
{
	bool bActive = false;

	/** Wielder가 확정한 prepared preview. 이 phase에서는 접촉 탐색을 다시 하지 않고 이 데이터만 따른다. */
	FRopePreparedThrowPreview Prepared;

	/** GuidedThrow 시작 순간의 실제 rope 위치. RenderPreview.Points로 전체 노드를 lerp하는 시작점이다. */
	TArray<FVector> StartPositions;
	float Elapsed = 0.0f;
	float Duration = 0.18f;

	void Reset()
	{
		*this = FRopeGuidedThrowState();
	}
};

/**
 * 캡처(Flight→Contacting) 순간의 로프 진행 좌표계 스냅샷(진행 방향 기반 wrap 2단계).
 * Contacting부터는 솔브가 없어 노드가 정지하므로, "로프가 어느 방향으로 날아와 어떻게 누웠는가"는
 * 이 순간에만 잴 수 있다 — BuildContactingState가 채우고 ResetTransientPhaseState가 폐기한다.
 * 소비자: TravelPlaneFirst 축(가이드 평면이 없는 던지기의 폴백 normal, 3단계에서 축 origin으로
 * RegionCenter 사용 예정). GPU 상주 로프는 CPU 미러가 1~2프레임 낡을 수 있으나 방향 성분은 충분하다.
 */
struct FRopeCaptureTravelFrame
{
	bool bValid = false;

	/** 접촉 후보 표면점(WorldPoint)들의 평균 — 접촉 영역 중심(월드). */
	FVector RegionCenter = FVector::ZeroVector;

	/** 접촉 노드들의 평균 Verlet 속도(cm/s). dt<=0이면 Zero. */
	FVector AverageVelocity = FVector::ZeroVector;

	/** 접촉 span의 head→tail 단위 방향(로프가 누운 방향). 접촉이 한 노드뿐이면 이웃 노드로 넓혀 잰다. */
	FVector SpanDirection = FVector::ZeroVector;

	/** AverageVelocity × SpanDirection 유도 성공 여부(속도 0/평행이면 false — 자연 폴백 신호). */
	bool bHasPlaneNormal = false;

	/** 유도된 진행 평면 normal(단위). 부호는 소비자(winding/OrientAxisByTail)가 해석한다. */
	FVector PlaneNormal = FVector::ZeroVector;

	void Reset()
	{
		*this = FRopeCaptureTravelFrame();
	}

	/** 캡처 순간의 Sim/후보에서 스냅샷을 계산한다(UObject-free). 구현은 RopeTypes.cpp. */
	static FRopeCaptureTravelFrame Compute(const FRopeSimState& Sim,
		const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);
};

/** 트래커가 dominant 외에도 유지하는 (Mesh, Bone) 대상별 접촉 집계(시드 다중화 재료). */
struct FRopeTrackedContactTarget
{
	FName Bone = NAME_None;
	const USceneComponent* Mesh = nullptr;

	/** 이번 프레임 이 대상에 닿은 노드들(매 갱신 최신 후보로 교체). */
	TArray<int32> Nodes;

	/** 이 대상의 지속 접촉 시간. 접촉이 끊긴 프레임에는 같은 양만큼 감쇠하고 0이 되면 목록에서 빠진다. */
	float DwellTime = 0.0f;
};

/**
 * 접촉 후보들에서 dominant 대상 — (Mesh, Bone) 쌍 — 을 추적하는 POD 트래커. 본 이름만 키로 쓰면
 * 같은 스켈레톤을 쓰는 두 액터가 동시에 닿을 때 후보가 합산/오귀속되므로 mesh까지 키에 포함한다.
 * Flight의 캡처 판정(ShouldCapture)과 Contacting의 체류 추적이 공용으로 쓴다.
 * 동률은 head(손 쪽) 노드가 앞선 대상 → 점수(관통+감김 방향) 순으로 깨져 프레임 간 안정적이다.
 * 대상이 바뀌면(본 또는 mesh) DwellTime이 0부터 다시 쌓인다(전이 프레임 오탐 방어 — 랙돌 테스트 (c)가
 * 고정하는 계약).
 * dominant와 별개로 접촉 중인 모든 (Mesh, Bone) 대상을 Targets에 dwell과 함께 유지한다 —
 * 시드 다중화(MaxWrapSeeds > 1)가 보조 시드 후보를 고르는 재료다. dominant 선정/리셋 계약은
 * Targets 도입과 무관하게 종전과 동일하다.
 */
struct FRopeContactTracker
{
	FName CandidateBone = NAME_None;
	const USceneComponent* CandidateMesh = nullptr;

	TArray<int32> CandidateNodes;
	float DwellTime = 0.0f;

	/** 접촉 중인 모든 대상의 (Mesh, Bone)별 집계. dominant도 포함된다(같은 키로 조회 가능). */
	TArray<FRopeTrackedContactTarget> Targets;

	void Reset()
	{
		CandidateBone = NAME_None;
		CandidateMesh = nullptr;
		CandidateNodes.Reset();
		DwellTime = 0.0f;
		Targets.Reset();
	}

	void BeginOrUpdate(const TArray<FRopeContactCandidate>& Candidates)
	{
		Update(Candidates, 0.0f);
	}

	void Decay(float DeltaTime)
	{
		DwellTime = FMath::Max(0.0f, DwellTime - DeltaTime);

		// 접촉이 전무한 프레임: 모든 대상의 dwell을 같은 비율로 소진시킨다(짧은 플리커 관용은 동일).
		// 노드 목록은 이번 프레임 접촉이 아니므로 비워 stale 소비를 막는다.
		for (int32 Index = Targets.Num() - 1; Index >= 0; --Index)
		{
			Targets[Index].DwellTime -= DeltaTime;
			Targets[Index].Nodes.Reset();
			if (Targets[Index].DwellTime <= 0.0f)
			{
				Targets.RemoveAt(Index);
			}
		}

		if (DwellTime <= 0.0f)
		{
			Reset();
		}
	}

	/** 후보를 (Mesh, Bone) 쌍별 집계해 dominant 대상/노드/체류 시간을 갱신한다. 구현은 RopeTypes.cpp. */
	void Update(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);
};
