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

/** 라이프사이클 단계. Free/Flight/Contacting = 물리(solver). Wrapped/Releasing = 로직(wrap 컨트롤러). */
UENUM(BlueprintType)
enum class ERopePhase : uint8
{
	Free,
	Flight,
	Contacting,		//접촉 후보 감지
	Wrapping,		//감기는 중
	Wrapped,
	GuidedThrow,	//PreviewPathLocked 전용. 물리 Flight를 타지 않고 cached preview path를 authoritative하게 따라간다.
	Releasing
};

/** wrap이 해제된 이유. */
UENUM(BlueprintType)
enum class ERopeReleaseReason : uint8
{
	Manual,
	Distance,	// 손~앵커 거리가 가용 로프 길이 + DistanceReleaseSlack 초과(자동)
	Tension,	// 최대 장력이 TensionReleaseForce를 지속 초과(자동)
	Broken,		// 대상 소실/wrap 실패 등 내부 사유
	Cut			// 외부 게임플레이가 로프를 절단(URopeComponent::CutRope)
};

/** Wrapping 중 tail node의 목표 surface path를 생성하는 방식. Project Settings에서 전역 선택한다. */
UENUM(BlueprintType)
enum class ERopeWrappingPathMode : uint8
{
	/** bone-parent 축을 기준으로 수학적 helix를 만든 뒤 SDF 표면에 투영한다. 일정한 나선 실루엣을 얻기 쉽다. */
	AnalyticHelix UMETA(DisplayName = "Analytic Helix"),

	/** 매 step마다 축 기준 원주 방향 벡터장을 만들고 SDF tangent plane에 투영한다. 의도적으로 원주를 돌면서 표면 굴곡도 따라간다. */
	SurfaceVectorField UMETA(DisplayName = "Surface Vector Field")
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

struct FRopeSurfaceAnchor
{
	int32 NodeIndex = INDEX_NONE;

	FName Bone = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	//SDF 표면 기준 bone-local anchor;
	FVector LocalSurfacePosition = FVector::ZeroVector;
	FVector LocalNormal = FVector::UpVector;
	FVector LocalTangent = FVector::ForwardVector;

	//Wrapping 시작 순간의 월드 위치 Lerp 시작점으로 사용
	FVector StartWorldPosition = FVector::ZeroVector;

	//나중에 여러 번 감김/ 길이 계산이 사용할 값. 커밋 시점 눈금으로 저장되므로 reel(길이 변경) 후엔 stale.
	float RopeDistance = 0.f;

	//표면에서 로프 중심선을 얼마나 띄울지. 보통 rope radius
	float SurfaceOffset = 0.0f;
};

struct FRopeWrapPathPoint
{
	FVector SurfaceWorld = FVector::ZeroVector;
	FVector NormalWorld = FVector::UpVector;
	FVector TangentWorld = FVector::ForwardVector;

	// 이 path point가 투영된 실제 표면 본.
	// AnalyticHelix는 기존처럼 latch bone을 넣고, SurfaceVectorField는 projection scoring 결과를 넣는다.
	// 이후 AppendWrappingAnchorFromPathPoint가 이 값을 기준으로 bone-local anchor를 저장한다.
	FName Bone = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	float DistanceFromLatch = 0.0f;
};

struct FRopeWrappingState
{
	FName BoneName = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	TArray<FRopeSurfaceAnchor> Anchors;
	FRopeSurfaceAnchor LatchAnchor;
	TArray<FRopeWrapPathPoint> Path;

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

	// SurfaceVectorField 적분 중 현재 surface point가 어느 본 위에 있는지 추적한다.
	// 다음 step의 후보 본은 이 값을 중심으로 skeleton graph 근방에서 고른다.
	FName PathCurrentBone = NAME_None;

	// 마지막으로 떠난 본. 새 후보가 바로 이 본이면 A->B->A 왕복 가능성이 높으므로
	// scoring 단계에서 ImmediateBoneReturnPenalty를 더해 전환 떨림을 줄인다.
	FName PathPreviousBone = NAME_None;
	TWeakObjectPtr<const USceneComponent> PathCurrentMesh = nullptr;

	// 마지막 본 전환 이후 path가 표면을 따라 진행한 거리(cm).
	// 새 본 후보가 좋아 보여도 MinBoneTransitionPathDistance 전에는 현재 본을 유지해
	// 한두 step마다 본이 바뀌는 flicker를 막는다.
	float PathDistanceSinceBoneTransition = 0.0f;

	float PathWindingSign = 1.0f;

	float Elapsed = 0.0f;
	float Duration = 0.16f;
	float StableTime = 0.0f;
	float LostContactTime = 0.0f;

	int32 WindingSign = 1;

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

/** wrap 이후 데이터 모델(바인딩 시맨틱). 물리→로직 핸드오프 시점에 생성된다. */
//TODO 추후 수정사항 : 나중에 Wrapped까지 surface anchor 기반으로 갈아엎을 때 FRopeWrapState의 Latched를 Anchors로 바꾸면 돼.
struct FRopeWrapState
{
	FName                   BoneName = NAME_None;

	TArray<FRopeLatchNode>  Latched;// 기존 fallback용
	TArray<FRopeSurfaceAnchor> Anchors; // 새 방식

	float                   Tension = 0.0f;     // 최대 세그먼트 장력(Wrapped 중 매 프레임 갱신)
	float                   TimeWrapped = 0.0f; // 감긴 누적 시간(Hold가 증가 — 포획 성공 판정 등 게임 소비 예정)

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
	int32   AnchorNode = INDEX_NONE;          // 손 쪽 첫 앵커 노드(힘 인가 지점의 노드)
	int32   AimNode = INDEX_NONE;             // 첫 직선 다리 끝(walk가 멈춘 정수 노드) — 방향의 raw 조준(ComputePull 산출; 디버그/진단)
	FName   Bone = NAME_None;                 // 앵커가 붙은 본(물리 본 힘 인가 대상)
	FVector WorldPoint = FVector::ZeroVector; // 앵커 노드 월드 위치(힘 인가점)
	FVector Direction = FVector::ZeroVector;  // 당김 단위 방향(앵커에서 조준 쪽 = 로프 경로 추종; 소비 시 컴포넌트가 fractional+EMA 스무딩)
	float   Tension = 0.0f;                   // 앵커-손 쪽 인접 세그먼트 장력(FRopeSimState::SegmentTension 단위)
	// 아래 둘은 소비자(컴포넌트)가 AimNode를 float로 시간 스무딩해 채운다(ComputePull은 정수 AimNode만 산출).
	// tether/방향이 이 연속 값을 써 정수 조준 노드의 프레임 간 이산 홉(방향 점프 + 견인 끊김)을 없앤다.
	float   AimNodeF = -1.0f;                 // 스무딩된 fractional 조준 인덱스([0, AnchorNode); <0 = 미설정)
	FVector AimPos = FVector::ZeroVector;     // 노드 사이 보간된 조준 월드 위치(AimNodeF 위치)
};

/** rope 중심선: 파티클의 체인. solver / 로직 / 렌더의 단일 진실 공급원(single source of truth). */
struct FRopeSimState
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;
	float           SegmentLength = 0.0f;
	float           RopeLength = 0.0f;

	// 고정된 시작점(hand/socket). solver는 substep에 걸쳐 Prev->Target으로 쓸어 이동시키므로
	// 빠른 앵커 점프가 에너지를 주입하는(체인을 폭발시킬) 대신 흡수된다.
	bool            bStartPinned = false;
	FVector         StartPinPrev = FVector::ZeroVector;
	FVector         StartPinTarget = FVector::ZeroVector;

	// Fixed timestep accumulator. The solver consumes real frame time in fixed-size substeps.
	float           TimeAccumulator = 0.0f;

	// 세그먼트별 장력(힘, 스트레치=양수만). XPBD distance 제약의 수렴 λ에서 유도: F = max(0, -λ)/h².
	// 단위는 질량 1 노드 기준 mass·cm/s²(상대값) — 임계치는 실측으로 튜닝한다. CPU 솔버가 Step 끝에
	// 채우고, GPU 상주 로프는 λ 리드백(1~2프레임 지연)이 채운다. 솔브 없는 프레임은 직전 값 유지.
	// 크기 = Num()-1(비어 있을 수 있음 — 아직 한 번도 솔브 안 됨).
	TArray<float>   SegmentTension;

	int32 Num() const { return Positions.Num(); }
	void  Reset() { Positions.Reset(); PrevPositions.Reset(); InvMass.Reset(); SegmentTension.Reset(); TimeAccumulator = 0.0f; }
};

/**
 * FRopeNodeOverrideFrame::Flags의 노드별 비트. ERopeGPUOverride(RopeGPUSolver.h)와 수치 1:1이어야
 * 한다(서브시스템이 검증) — Core는 Shaders 모듈에 의존하지 않으므로 상수를 미러로 둔다.
 */
namespace RopeNodeOverride
{
	constexpr uint8 Position         = 1 << 0; // Pos[i]  = Positions[i]
	constexpr uint8 Prev             = 1 << 1; // Prev[i] = PrevPositions[i] (Verlet 속도 주입)
	constexpr uint8 PrevFromPosition = 1 << 2; // Prev[i] = Pos[i] (속도 0; Position 적용 *후* 값)
	constexpr uint8 InvMass          = 1 << 3; // InvMass[i] = InvMass[i]
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
	TArray<uint8>   Flags;         // 노드별 RopeNodeOverride 비트 OR(비어 있으면 이번 프레임 산출물 없음)
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

/** 컨택트 결정 튜닝: 걸쳐진 rope가 언제 사지(limb)에 "wrapped"된 것으로 간주되는가? */
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

	/** Temporary contact loss tolerated while the rope is settling into a wrap. */
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
	 * SegmentLength + TetherSlack)를 넘으면 초과분 × 이 값만큼 대상을 손 쪽으로 되돌린다(프레임당).
	 * 힘이 아니라 위치/속도 동기라 장력→힘→스트레치→장력 피드백 폭주가 없다(초과분이 줄면 보정도
	 * 준다 — 수렴). 1 = 즉시 스냅, 0 = 비활성(기본). 물리 시뮬 대상은 같은 수렴을 속도 주입으로 만든다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TetherResponse = 0.0f;

	/** 테더 발동 전 허용 여유(cm). 경계 지터/미세 슬랙에서 발동하는 것을 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm"))
	float TetherSlack = 5.0f;

	/**
	 * 테더 회수 최대 속도(cm/s). 물리 대상은 속도를 누적하지 않고 이 값으로 상한된 목표 속도까지만 톱업하고,
	 * 비물리 대상은 프레임당 위치 보정을 이 값×dt로 클램프한다 → 초과분 스파이크나 임펄스 누적으로 대상이
	 * 튕겨나가는 것을 원천 차단한다(수렴 보장). 0 = 클램프 없음(비권장).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm/s"))
	float TetherMaxSpeed = 1500.0f;

	/**
	 * 테더 회수 분배: 초과분 중 감긴 *대상*이 회수하는 비율. 1(기본) = 전량 대상(기존 동작),
	 * 0 = 전량 wielder(로프 owner가 앵커 쪽으로 끌려간다 — 고정 앵커 매달리기/등반, 되감기와 조합하면
	 * 입체기동식 "감으면 끌려 올라감"), 중간 = 비율 분할(양끝이 서로에게 끌린다). 양끝 이동의 합이
	 * 초과분을 넘지 않아 과수렴이 없다. 자기 자신에 감긴 로프(owner==대상)는 무시하고 전량 대상 경로.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "1.0"))
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

struct FRopeThrowParams; // 아래 정의 — MakeDefault가 설정 스냅샷으로 받는다.

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

	/** 현재 런타임 미사용 — 에디터 배치 가이드(FRopeComponentVisualizer)의 던지기 아크 표시 전용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.0", Units = "cm"))
	float ArcHeight = 120.0f;

	/** 현재 런타임 미사용 — 에디터 배치 가이드(FRopeComponentVisualizer)의 던지기 아크 표시 전용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.0", Units = "cm"))
	float SideOffset = 35.0f;
};

//TODO 주석 추가
enum class ERopeContactCandidateSource : uint8
{
	Actual = 1,
	PredictiveFree = 2,
	PredictiveGuided = 4
};

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
	float WrapDirectionScore = 0.0f; // 감김 방향이면 +, 반대면 -
};

struct FRopePreparedThrowPreview
{
	bool bValid = false;

	// preview를 만들 때 쓴 throw 기준. montage가 있어도 입력 순간의 frame/origin을 보존하기 위해 저장한다.
	FRopeThrowContext ThrowContext;

	// 화면에 보이는 preview centerline. GuidedThrow에서는 이 점들을 실제 노드 목표 위치로도 사용한다.
	FRopeWrapPreviewData RenderPreview;

	// preview build 시 만든 가상 로프 상태와 접촉 후보. 디버그/후속 고도화용으로 보존한다.
	FRopeSimState PreviewSim;
	FRopeContactCandidate Contact;

	// 최종 Wrapped 진입에 필요한 bone-local 고정 정보. Points만으로는 캐릭터 움직임을 따라갈 수 없다.
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
};

struct FRopeGuidedThrowState
{
	bool bActive = false;

	// Wielder가 확정한 prepared preview. 이 phase에서는 접촉 탐색을 다시 하지 않고 이 데이터만 따른다.
	FRopePreparedThrowPreview Prepared;

	// GuidedThrow 시작 순간의 실제 rope 위치. RenderPreview.Points로 전체 노드를 lerp하는 시작점이다.
	TArray<FVector> StartPositions;
	float Elapsed = 0.0f;
	float Duration = 0.18f;

	void Reset()
	{
		*this = FRopeGuidedThrowState();
	}
};

/**
 * 접촉 후보들에서 dominant 본(가장 많은 노드가 닿은 본)을 추적하는 POD 트래커.
 * Flight의 캡처 판정(ShouldCapture)과 Contacting의 체류 추적이 공용으로 쓴다.
 * 동률은 head(손 쪽) 노드가 앞선 본 → 점수(관통+감김 방향) 순으로 깨져 프레임 간 안정적이다.
 * 본이 바뀌면 DwellTime이 0부터 다시 쌓인다(전이 프레임 오탐 방어 — 랙돌 테스트 (c)가 고정하는 계약).
 */
struct FRopeContactTracker
{
	FName CandidateBone = NAME_None;
	const USceneComponent* CandidateMesh = nullptr;

	TArray<int32> CandidateNodes;
	float DwellTime = 0.0f;

	void Reset()
	{
		CandidateBone = NAME_None;
		CandidateMesh = nullptr;
		CandidateNodes.Reset();
		DwellTime = 0.0f;
	}

	void BeginOrUpdate(const TArray<FRopeContactCandidate>& Candidates)
	{
		Update(Candidates, 0.0f);
	}

	void Decay(float DeltaTime)
	{
		DwellTime = FMath::Max(0.0f, DwellTime - DeltaTime);
		if (DwellTime <= 0.0f)
		{
			Reset();
		}
	}

	/** 후보를 본별 집계해 dominant 본/노드/체류 시간을 갱신한다. 구현은 RopeTypes.cpp. */
	void Update(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);
};
