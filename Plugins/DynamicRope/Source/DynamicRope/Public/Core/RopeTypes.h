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
	Distance,
	Tension,
	Broken
};

/** Wrapping 중 tail node의 목표 surface path를 생성하는 방식. Project Settings에서 전역 선택한다. */
UENUM(BlueprintType)
enum class ERopeWrappingPathMode : uint8
{
	/** 최초 latch tangent 방향으로 SDF 표면을 한 걸음씩 따라간다. 가장 보수적인 기본 폴백 경로. */
	SurfaceWalk UMETA(DisplayName = "Surface Walk"),

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

	//나중에 여러 번 감김/ 길이 계산이 사용할 값
	float RopeDistance = 0.f;
	float WindingAngle = 0.f;

	//표면에서 로프 중심선을 얼마나 띄울지. 보통 rope radius
	float SurfaceOffset = 0.0f;
};

struct FRopeWrappingState
{
	FName BoneName = NAME_None;
	TWeakObjectPtr<const USkeletalMeshComponent> Mesh = nullptr;

	TArray<FRopeSurfaceAnchor> Anchors;

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

	float                   WrapTurns = 0.0f;
	float                   Tension = 0.0f;
	float                   TimeWrapped = 0.0f;
	float                   AnchorDistance = 0.0f;

	// BoneName을 소유한 Mesh. wrap은 이 mesh에 대해 유지/추적된다(rope 소유자와 다른
	// 액터일 수 있음). 결정 시점에 컨택트로부터 해석된다.
	// cross-actor wrap에서는 대상 액터가 Wrapped 도중 파괴될 수 있다. raw 포인터로 보관하면
	// Hold가 매 프레임 dangling 포인터를 역참조(use-after-free)하므로, 파괴 시 안전하게 null이
	// 되는 weak 포인터로 보관한다(POD 유지: hard 레퍼런스가 아니라 GC를 막지 않는다).
	TWeakObjectPtr<const USkeletalMeshComponent> Mesh = nullptr;

	bool IsWrapped() const { return Anchors.Num() > 0 || Latched.Num() > 0; }
	void Reset() { *this = FRopeWrapState(); }
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

	int32 Num() const { return Positions.Num(); }
	void  Reset() { Positions.Reset(); PrevPositions.Reset(); InvMass.Reset(); TimeAccumulator = 0.0f; }
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
	float WrapDecisionTime = 0.15f;

	/** Wrapping phase must keep the same accumulated latch span stable this long before committing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrappingStableTime = 0.10f;

	/** Time used to pull tail nodes onto their generated surface wrap targets. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.01", Units = "s"))
	float WrappingMotionDuration = 0.25f;

	/** Per-segment delay while tail nodes settle onto the surface path. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrappingTailDelayPerSegment = 0.012f;

	/** Axis distance advanced per circumference distance for analytic helix wrapping. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "-2.0", ClampMax = "2.0"))
	float WrappingHelixPitchScale = 0.25f;

	/** Upper bound for physics-based wrapping settle before committing the best accumulated anchors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrappingMaxSettleTime = 0.45f;

	/** Temporary contact loss tolerated while the rope is settling into a wrap. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrappingContactGraceTime = 0.20f;

	/** Extra Flight lookahead in frame-displacements for thin limb/SDF candidate detection. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", ClampMax = "4.0"))
	float PredictiveContactFrames = 1.0f;

	/** Flight returns to Free after this much post-whip time with no contact candidates. 0 disables. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float FlightNoContactReturnTime = 0.0f;
};

/** flight 단계의 Throw / launch 파라미터. */
USTRUCT(BlueprintType)
struct FRopeThrowParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float ThrowSpeed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.1"))
	float TipMass = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0", Units = "cm"))
	float AimAssistRadius = 100.0f;
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
	float BestWrapScore = 0.0f;

	void Reset()
	{
		CandidateBone = NAME_None;
		CandidateMesh = nullptr;
		CandidateNodes.Reset();
		DwellTime = 0.0f;
		BestWrapScore = 0.0f;
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
		BestWrapScore = BestScore;
	}
};
