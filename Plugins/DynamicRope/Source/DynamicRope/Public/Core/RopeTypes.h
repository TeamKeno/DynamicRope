// Copyright Epic Games, Inc. All Rights Reserved.
//
// Dynamic Rope 시스템의 핵심 데이터 타입. 핫 루프(sim/contact/wrap 상태)에 있는 것은
// 순수 POD로, 디자이너용 설정에만 USTRUCT를 사용한다.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "RopeTypes.generated.h"

class USkeletalMeshComponent;

/** 라이프사이클 단계. Free/Flight/Contacting = 물리(solver). Wrapped/Releasing = 로직(wrap 컨트롤러). */
UENUM(BlueprintType)
enum class ERopePhase : uint8
{
	Free,
	Flight,
	Contacting,		//접촉 후보 감지
	Wrapping,		//감기는 중
	Wrapped,
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
	const USkeletalMeshComponent* SourceMesh = nullptr;
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
	TWeakObjectPtr<const USkeletalMeshComponent> Mesh = nullptr;

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
	TWeakObjectPtr<const USkeletalMeshComponent> Mesh = nullptr;

	float DistanceFromLatch = 0.0f;
};

struct FRopeWrappingState
{
	FName BoneName = NAME_None;
	TWeakObjectPtr<const USkeletalMeshComponent> Mesh = nullptr;

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
	TWeakObjectPtr<const USkeletalMeshComponent> PathCurrentMesh = nullptr;

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
	TWeakObjectPtr<const USkeletalMeshComponent> Mesh = nullptr;

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
	FName   Bone = NAME_None;                 // 앵커가 붙은 본(물리 본 힘 인가 대상)
	FVector WorldPoint = FVector::ZeroVector; // 앵커 노드 월드 위치(힘 인가점)
	FVector Direction = FVector::ZeroVector;  // 당김 단위 방향(앵커 → 손 직선 chord — 세그먼트 방향은 지터로 부적합)
	float   Tension = 0.0f;                   // 앵커-손 쪽 인접 세그먼트 장력(FRopeSimState::SegmentTension 단위)
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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "1", ClampMax = "16"))
	int32 CollisionPassesPerSubstep = 1;

	/** XPBD stretch compliance(stiffness의 역수). 0 = 신장 불가(inextensible). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0"))
	float StretchCompliance = 0.0f;

	/** XPBD bending compliance. 클수록 더 흐물거린다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0"))
	float BendCompliance = 0.02f;

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
	 * 엔진 Global Distance Field로 정적 월드 지오메트리(벽/바닥)에서 로프를 밀어낸다. GPU 경로 + 씬 그래프
	 * dispatch(r.DynamicRope.GDFDispatchInVE=1)에서만 동작. 프로젝트에 Generate Mesh Distance Fields 필요.
	 * 본 귀속·표면속도 없음(정적 월드 광역 밀어내기 보완재) — per-bone SDF의 대체가 아니다. 켜져 있는 동안
	 * 엔진이 GDF를 온디맨드로 빌드한다. 밀어내기 반경/마찰은 CollisionRadius/Friction/TipFrictionScale 공유.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver")
	bool bUseWorldGDF = false;

	/** Swept collision sample spacing in cm. Lower values reduce tunneling at higher query cost. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.1", Units = "cm"))
	float SweepStep = 2.0f;

	/** Maximum swept samples per segment, used as a cost cap for very fast nodes. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "1", ClampMax = "64"))
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
	 * 거리 release: Wrapped 중 손~앵커 직선 거리가 가용 로프 길이(+TetherSlack)를 이만큼(cm) 더
	 * 초과하면 자동 release한다(ERopeReleaseReason::Distance). 0 = 비활성(기본). 테더와 함께 쓰면
	 * "테더가 버티다가 이 한계를 넘으면 놓친다"가 된다 — 테더가 충분히 강하면 초과분이 안 쌓여
	 * 발동하지 않고, 테더 없이 쓰면 순수 거리 제한으로 동작한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm"))
	float DistanceReleaseSlack = 0.0f;
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

/** 현재 런타임 미사용. 나중에 swing arc의 시작 방향을 어떻게 정의할지 고를 때 쓸 자리만 미리 열어둔다. */
UENUM(BlueprintType)
enum class ERopeSwingStartMode : uint8
{
	FrameAngle UMETA(DisplayName = "Frame Angle"),
	RopePose UMETA(DisplayName = "Rope Pose"),
	AnimationVector UMETA(DisplayName = "Animation Vector")
};

/** 현재 런타임 미사용. angle 폴리싱 때 시작 방향/각도/애니메이션 벡터를 한 덩어리로 넘기기 위한 초안이다. */
USTRUCT(BlueprintType)
struct FRopeSwingArcDraft
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Draft")
	ERopeSwingStartMode StartMode = ERopeSwingStartMode::FrameAngle;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Draft", meta = (ClampMin = "0.0", ClampMax = "180.0", Units = "deg"))
	float ArcAngleDegrees = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Draft")
	FVector AnimationStartDirection = FVector::ZeroVector;
};

/** throw 순간 Wielder/Component가 계산해 넘기는 런타임 값. 설정값(FRopeThrowParams)과 분리한다. */
USTRUCT(BlueprintType)
struct FRopeThrowContext
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector Origin = FVector::ZeroVector;

	/** Legacy 입력값. 현재 throw 방향은 FrameForward를 사용한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FVector AimDirection = FVector::ForwardVector;

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
	const USkeletalMeshComponent* Mesh = nullptr;
	ERopeContactCandidateSource Source = ERopeContactCandidateSource::Actual;
	uint8 SourceMask = static_cast<uint8>(ERopeContactCandidateSource::Actual);

	FVector WorldPoint = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	FVector SurfaceVelocity = FVector::ZeroVector;

	float Penetration = 0.0f;
	float RelativeTangentialSpeed = 0.0f;
	float WrapDirectionScore = 0.0f; // 감김 방향이면 +, 반대면 -
};

//TODO 주석 추가
struct FRopeContactTracker
{
	FName CandidateBone = NAME_None;
	const USkeletalMeshComponent* CandidateMesh = nullptr;

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

	void Update(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime)
	{
		if (Candidates.Num() == 0)
		{
			Decay(DeltaTime);
			return;
		}

		TMap<FName, TArray<int32>> NodesByBone;
		TMap<FName, const USkeletalMeshComponent*> MeshByBone;
		TMap<FName, float> ScoreByBone;
		TMap<FName, int32> HeadNodeByBone;
		for (const FRopeContactCandidate& Candidate : Candidates)
		{
			if (!Candidate.bValid || Candidate.Bone.IsNone())
			{
				continue;
			}

			NodesByBone.FindOrAdd(Candidate.Bone).Add(Candidate.NodeIndex);
			MeshByBone.FindOrAdd(Candidate.Bone) = Candidate.Mesh;
			ScoreByBone.FindOrAdd(Candidate.Bone) += Candidate.Penetration + FMath::Max(0.0f, Candidate.WrapDirectionScore);
			if (int32* ExistingHeadNode = HeadNodeByBone.Find(Candidate.Bone))
			{
				*ExistingHeadNode = FMath::Min(*ExistingHeadNode, Candidate.NodeIndex);
			}
			else
			{
				HeadNodeByBone.Add(Candidate.Bone, Candidate.NodeIndex);
			}
		}

		FName BestBone = NAME_None;
		int32 BestCount = 0;
		int32 BestHeadNode = INDEX_NONE;
		float BestScore = 0.0f;
		for (const TPair<FName, TArray<int32>>& Pair : NodesByBone)
		{
			const float Score = ScoreByBone.FindRef(Pair.Key);
			const int32 HeadNode = HeadNodeByBone.FindRef(Pair.Key);
			if (Pair.Value.Num() > BestCount ||
				(Pair.Value.Num() == BestCount &&
					(BestHeadNode == INDEX_NONE || HeadNode < BestHeadNode ||
						(HeadNode == BestHeadNode && Score > BestScore))))
			{
				BestBone = Pair.Key;
				BestCount = Pair.Value.Num();
				BestHeadNode = HeadNode;
				BestScore = Score;
			}
		}

		if (BestBone.IsNone())
		{
			Decay(DeltaTime);
			return;
		}

		if (BestBone == CandidateBone)
		{
			DwellTime += DeltaTime;
		}
		else
		{
			CandidateBone = BestBone;
			DwellTime = 0.0f;
		}

		CandidateMesh = MeshByBone.FindRef(BestBone);
		CandidateNodes = NodesByBone.FindRef(BestBone);
	}
};
