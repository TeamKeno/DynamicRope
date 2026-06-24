// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — 실험용이며 출시 대상 아님. PoC/ 아래의 모든 것은 폐기 가능한 코드다.
// S0: 직선 rope PBD/Verlet solver + spline-mesh 렌더링. 아직 바디 collision 없음.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PoC/RopeCapsuleProvider.h"
#include "RopePoCActor.generated.h"

class USplineMeshComponent;
class UStaticMesh;
class UMaterialInterface;
class ARopePoCCapsuleActor;

/**
 * Proof-of-concept 직선 rope.
 * Verlet integration + Position-Based-Dynamics distance constraint로 파티클 체인을
 * 시뮬레이션하고, 그 결과를 spline mesh 체인으로 렌더링한다.
 *
 * 빠른 반복 작업을 위해 (PIE 없이) 에디터 뷰포트에서 tick한다.
 */
UCLASS()
class DYNAMICROPE_API ARopePoCActor : public AActor
{
	GENERATED_BODY()

public:
	ARopePoCActor();

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual bool ShouldTickIfViewportsOnly() const override { return true; } // 에디터 뷰포트에서 tick
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	//~ Setup -------------------------------------------------------------
	/** rope를 따라 시뮬레이션되는 파티클 개수(>= 2). */
	UPROPERTY(EditAnywhere, Category = "Rope|Setup", meta = (ClampMin = "2", UIMin = "2"))
	int32 NumParticles = 24;

	/** rope의 전체 rest length(cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Setup", meta = (ClampMin = "1.0", UIMin = "1.0", Units = "cm"))
	float RopeLength = 200.0f;

	//~ Solver ------------------------------------------------------------
	/** 프레임당 constraint solver 반복 횟수. 클수록 더 뻣뻣하고/더 안정적. */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "1", UIMin = "1"))
	int32 SolverIterations = 12;

	/**
	 * 프레임당 물리 substep. 프레임을 N개의 step으로 분할해, 각 step마다 pinned 끝과
	 * capsule을 이전 포즈와 현재 포즈 사이로 sweep한다. 클수록 고속에서 tunneling이
	 * 없어진다("빠른 rope가 관통하는" 문제의 핵심 해법). 비용은 대략 선형으로 증가한다.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "1", ClampMax = "16", UIMin = "1", UIMax = "16"))
	int32 SimSubsteps = 4;

	/** free 파티클에 적용되는 중력. */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	/** 프레임당 속도 damping [0..1]. 0 = damping 없음. */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.02f;

	/**
	 * Jakobsen support stick(i↔i+2 distance constraint)을 통한 bending stiffness [0..1].
	 * 0 = 흐물거리는 체인, 1 = 휘는 것을 버티며 형태를 유지하는 뻣뻣한 rope. "면발이 아니라
	 * rope처럼 보이게" 하는 핵심 노브(RDR2급 품질과의 격차).
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BendStiffness = 0.3f;

	//~ Endpoints ---------------------------------------------------------
	/** 첫 번째 파티클을 이 액터의 원점에 pin한다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Endpoints")
	bool bPinStart = true;

	/** 마지막 파티클을 EndAnchorActor에 pin한다(설정된 경우). 해당 액터를 드래그하면 rope의 free end가 움직인다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Endpoints")
	bool bPinEnd = false;

	/** bPinEnd가 true일 때 마지막 파티클이 pin되는 선택적 액터. */
	UPROPERTY(EditAnywhere, Category = "Rope|Endpoints")
	TObjectPtr<AActor> EndAnchorActor = nullptr;

	//~ Collision ---------------------------------------------------------
	/** 명시적 capsule provider(테스트용 capsule 액터). 비워두고 bAutoFindColliders에 의존해도 된다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision")
	TArray<TObjectPtr<ARopePoCCapsuleActor>> Colliders;

	/** 레벨에서 발견되는 모든 IRopeCapsuleProvider(테스트용 capsule, skeletal limb)와도 collision한다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision")
	bool bAutoFindColliders = true;

	/** collision push-out에 사용되는 rope의 접촉 두께(cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float RopeCollisionRadius = 2.0f;

	/**
	 * (노드뿐 아니라) rope SEGMENT를 capsule에 대해 collision시키고, push-out을 양쪽 끝
	 * 노드에 분배한다(Jakobsen §5.2). 휘어진 limb에서 rope가 노드 사이로 파고들어가는 것을
	 * 막아준다 — "wrap이 그럴듯해 보이는" 핵심 디테일. 끄면 legacy 노드 전용 테스트.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision")
	bool bUseSegmentCollision = true;

	/**
	 * 접촉 중인 rope가 (움직이는) capsule 표면에 얼마나 강하게 달라붙는지.
	 * 0 = 마찰 없음(미끄러져 빠짐), 1 = 완전히 grip(limb와 함께 움직임 → wrap 유지). "감김 유지"의 핵심 노브.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WrapFriction = 0.6f;

	/** capsule 표면을 넘어서도 friction에서 여전히 "접촉 중"으로 취급하는 거리(cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float FrictionContactBand = 2.0f;

	//~ Pull / two-way coupling (S4) --------------------------------------
	/**
	 * S4: rope의 접촉 반작용을 capsule provider로 되먹임해, rope의 끝을 당기면 실제로
	 * 감긴 limb가 끌려오게 한다. draggable provider(테스트용 capsule)가 필요하다.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Pull")
	bool bEnableTwoWayPull = true;

	/** 반작용(collision push-out + latch된 노드의 tension)을 capsule에 전달되는 impulse로 스케일한다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Pull", meta = (EditCondition = "bEnableTwoWayPull", ClampMin = "0.0"))
	float PullReactionGain = 8.0f;

	//~ Wrap latch / Hold state (S4 — doc 4.1) ----------------------------
	/**
	 * 접촉 노드가 capsule 위에 충분히 오래 머무르면, 그 표면에 데이터로 latch한다:
	 * 그 이후로는 tension/중력과 무관하게 wrap을 유지하고(프로덕션 "Hold" 모델),
	 * 움직이는 limb를 따라가며, pull을 capsule로 전달한다. "감아도 바로 풀림"의 해법.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Wrap")
	bool bEnableWrapLatch = true;

	/** 노드가 latch되기 전까지의 연속 접촉 시간(s). */
	UPROPERTY(EditAnywhere, Category = "Rope|Wrap", meta = (EditCondition = "bEnableWrapLatch", ClampMin = "0.0", Units = "s"))
	float LatchContactTime = 0.15f;

	/** latch 판정 시 capsule 표면을 넘어서도 여전히 접촉으로 인정하는 추가 거리(cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Wrap", meta = (EditCondition = "bEnableWrapLatch", ClampMin = "0.0", Units = "cm"))
	float LatchContactBand = 1.5f;

	/** 인접 segment가 rest length의 이 배수를 넘어 늘어나면 latch를 해제한다(뜯겨 빠짐). */
	UPROPERTY(EditAnywhere, Category = "Rope|Wrap", meta = (EditCondition = "bEnableWrapLatch", ClampMin = "1.0"))
	float LatchReleaseStrain = 1.8f;

	/** latch된 모든 wrap을 해제한다(예: "unwrap" 입력 시). doc 4.3의 명시적 해제. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Wrap")
	void ReleaseAllWraps();

	//~ Render ------------------------------------------------------------
	/** segment마다 사용되는 mesh. 비워두면 엔진 cylinder가 기본값. */
	UPROPERTY(EditAnywhere, Category = "Rope|Render")
	TObjectPtr<UStaticMesh> RopeMesh = nullptr;

	/** rope segment에 적용되는 material. 비워두면 basic material이 기본값. */
	UPROPERTY(EditAnywhere, Category = "Rope|Render")
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	/** 시각적 rope radius(cm). base radius 50의 cylinder mesh를 가정한다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Render", meta = (ClampMin = "0.1", UIMin = "0.1", Units = "cm"))
	float RopeRadius = 2.0f;

	//~ Debug -------------------------------------------------------------
	/** 파티클 체인을 debug line/point로 그린다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Debug")
	bool bDrawDebug = true;

private:
	/** rope 전체를 배치/이동할 수 있게 해주는 root. */
	UPROPERTY()
	TObjectPtr<USceneComponent> RopeRoot = nullptr;

	/** segment당 하나의 spline mesh(NumParticles - 1). Transient — 재생성되며 저장되지 않음. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<USplineMeshComponent>> SegmentMeshes;

	// --- Transient 시뮬레이션 상태(월드 공간) ---
	TArray<FVector> Positions;
	TArray<FVector> OldPositions;
	TArray<float>   InvMasses;
	float           SegmentLength = 0.0f;
	bool            bInitialized = false;

	// --- Wrap latch 상태(파티클별) ---
	/** 이 파티클이 latch된 capsule 인덱스, free면 -1. */
	TArray<int32>   LatchCapsule;
	/** capsule 기준으로 표현한 latch 접촉: A로부터 axis를 따라가는 거리... */
	TArray<float>   LatchAlong;
	/** ...그리고 radial 방향 + 거리(월드 공간, 매 step axis를 따라 회전됨). */
	TArray<FVector> LatchRadialDir;
	TArray<float>   LatchRadialDist;
	/** 직전 업데이트 시점의 capsule axis. limb가 움직일 때 증분 회전을 계산하는 데 사용. */
	TArray<FVector> LatchAxis;
	/** 각 파티클이 얼마나 연속으로 접촉해 왔는지(latch dwell 판정용). */
	TArray<float>   ContactDwell;

	// S2 GO/NO-GO budget 체크를 위한 성능 readout.
	float           LastSolveMs = 0.0f;
	float           AvgSolveMs = 0.0f;

	/** 이번 실행에 사용된 capsule provider(명시적 리스트 + auto-found), init 시 resolve됨. */
	TArray<TWeakObjectPtr<UObject>> CapsuleProviders;

	/** 프레임당 한 번 provider에서 수집한 capsule(현재 포즈). */
	TArray<FRopeCapsule> FrameCapsules;

	/** 각 FrameCapsules 항목을 만든 provider 인덱스(CapsuleProviders 기준). */
	TArray<int32> FrameCapsuleOwner;

	/** 직전 프레임의 capsule(같은 순서). substep이 보간을 시작하는 start 포즈. */
	TArray<FRopeCapsule> PrevFrameCapsules;

	/** 현재 substep의 capsule(Prev→Frame 보간); collision/friction이 읽는 대상. */
	TArray<FRopeCapsule> ActiveCapsules;
	/** 직전 substep의 capsule. friction의 표면 속도 추정용. */
	TArray<FRopeCapsule> PrevActiveCapsules;

	/** 직전 프레임의 pinned 끝 target. substep이 끝을 sweep할 수 있게 한다(anti-tunneling). */
	FVector PrevStartWorld = FVector::ZeroVector;
	FVector PrevEndWorld = FVector::ZeroVector;
	bool    bHasPrevPins = false;

	// --- S4 pull 반작용 누산기(FrameCapsules 항목별, 매 프레임 reset) ---
	/** 이번 프레임에 rope가 각 capsule에 가하는 반작용 impulse의 합. */
	TArray<FVector> CapsuleReaction;
	/** 각 capsule 반작용의 접촉 가중 적용점(월드 공간). */
	TArray<FVector> CapsuleReactionPoint;
	/** 적용점을 평균내기 위한 전체 접촉 가중치. */
	TArray<float> CapsuleReactionWeight;

	void InitializeRope();
	void RebuildSegmentMeshes();
	void GatherProviders();
	void BuildFrameCapsules();
	void SimulateStep(float DeltaSeconds);
	void SolveConstraints(bool bReverse);
	void SolveBendingConstraints(bool bReverse);
	void SolveCollisions();
	void ApplyFriction();
	void ApplyPullReaction();
	void ApplyPinning();
	/** 끝점을 명시적인 월드 target에 pin한다(substep 전반에 걸쳐 끝을 sweep할 때 사용). */
	void SetPinnedTargets(const FVector& StartW, const FVector& EndW);

	/** latch된 파티클을 (움직이는) capsule 표면에 다시 배치하고 pin을 유지한다. */
	void UpdateLatchedPositions();
	/** 새로 오래 접촉한 노드를 latch하고, 과도하게 늘어난 것은 해제한다. 프레임당 한 번. */
	void ManageWrapLatch(float FrameDt);
	/** latch된 노드의 tension을 pull 반작용으로 capsule에 되먹인다. 프레임당 한 번. */
	void AccumulateLatchReaction();
	void UpdateSegmentMeshes();
	void DrawDebugRope() const;

	FVector GetStartWorld() const;
	FVector GetEndWorld() const;
};
