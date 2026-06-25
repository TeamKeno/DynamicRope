// Copyright Epic Games, Inc. All Rights Reserved.
//
// Dynamic Rope 시스템의 핵심 데이터 타입. 핫 루프(sim/contact/wrap 상태)에 있는 것은
// 순수 POD로, 디자이너용 설정에만 USTRUCT를 사용한다.

#pragma once

#include "CoreMinimal.h"
#include "RopeTypes.generated.h"

class USkeletalMeshComponent;

/** 라이프사이클 단계. Free/Flight/Contacting = 물리(solver). Wrapped/Releasing = 로직(wrap 컨트롤러). */
UENUM(BlueprintType)
enum class ERopePhase : uint8
{
	Free,
	Flight,
	Contacting,
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

/**
 * narrow-phase 컨택트: rope 노드 하나 vs collider 하나, IRopeCollider::Query가 반환한다.
 *
 * CONTRACT — FROZEN 2026-06-24. 모든 IRopeCollider(capsule, bone-SDF, world-GDF)는 이를 반드시 준수해야 한다.
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
 */
struct FRopeContact
{
	bool    bHit = false;
	FVector Normal = FVector::UpVector;
	float   Penetration = 0.0f;
	FVector SurfacePoint = FVector::ZeroVector;
	FName   Bone = NAME_None;
	const USkeletalMeshComponent* SourceMesh = nullptr;
};

/** latch된 wrap 노드. bone-local 공간에 고정되어 재충돌 없이 skinning을 따라간다. */
struct FRopeLatchNode
{
	int32   NodeIndex = INDEX_NONE;
	FName   Bone = NAME_None;
	FVector BoneLocalPos = FVector::ZeroVector;
};

/** wrap 이후 데이터 모델(바인딩 시맨틱). 물리→로직 핸드오프 시점에 생성된다. */
struct FRopeWrapState
{
	FName                   BoneName = NAME_None;
	TArray<FRopeLatchNode>  Latched;
	float                   WrapTurns = 0.0f;
	float                   Tension = 0.0f;
	float                   TimeWrapped = 0.0f;
	float                   AnchorDistance = 0.0f;

	// BoneName을 소유한 Mesh. wrap은 이 mesh에 대해 유지/추적된다(rope 소유자와 다른
	// 액터일 수 있음). 결정 시점에 컨택트로부터 해석된다.
	const USkeletalMeshComponent* Mesh = nullptr;

	bool IsWrapped() const { return Latched.Num() > 0; }
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

	/** collider에 대한 접선 방향 friction [0..1]. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Friction = 0.5f;

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
	int32 MinLatchNodes = 3;

	/** wrap을 확정하기 전에 컨택트가 같은 bone에서 이만큼 지속되어야 한다(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrapDecisionTime = 0.15f;
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
struct FRopeContactCandidate
{
	bool bValid = false;
	int32 NodeIndex = INDEX_NONE;
	FName Bone = NAME_None;
	const USkeletalMeshComponent* Mesh = nullptr;

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
		for (const FRopeContactCandidate& Candidate : Candidates)
		{
			if (!Candidate.bValid || Candidate.Bone.IsNone())
			{
				continue;
			}

			NodesByBone.FindOrAdd(Candidate.Bone).Add(Candidate.NodeIndex);
			MeshByBone.FindOrAdd(Candidate.Bone) = Candidate.Mesh;
			ScoreByBone.FindOrAdd(Candidate.Bone) += Candidate.Penetration + FMath::Max(0.0f, Candidate.WrapDirectionScore);
		}

		FName BestBone = NAME_None;
		int32 BestCount = 0;
		float BestScore = 0.0f;
		for (const TPair<FName, TArray<int32>>& Pair : NodesByBone)
		{
			const float Score = ScoreByBone.FindRef(Pair.Key);
			if (Pair.Value.Num() > BestCount || (Pair.Value.Num() == BestCount && Score > BestScore))
			{
				BestBone = Pair.Key;
				BestCount = Pair.Value.Num();
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
