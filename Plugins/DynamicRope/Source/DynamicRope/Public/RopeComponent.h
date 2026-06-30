// Copyright Epic Games, Inc. All Rights Reserved.
//
// 단일 UE 통합 지점(Facade). sim 상태, solver, wrap controller, 그리고 physics와 logic을
// 분기하는 phase state machine을 소유한다. 캐릭터에 붙이고 Throw()로 사용한다.

#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "Core/RopeTypes.h"
#include "Solver/RopeXPBDSolver.h"
#include "Logic/RopeWrapController.h"
#include "RopeComponent.generated.h"

class AActor;
class IRopeCollider;
class IRopeColliderProvider;
class UMaterialInterface;
class USkeletalMeshComponent;
class FRegisterComponentContext;
struct FRopeDebugSnapshot;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnWrapped, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnCaptured, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnReleased, FName, Bone, ERopeReleaseReason, Reason);

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeComponent : public UMeshComponent
{
	GENERATED_BODY()

	// 서브시스템이 GPU 배치 솔브를 위해 Sim/SolverConfig/bSolveThisFrame에 직접 접근한다(CPU 경로는 SolveSimFrame 사용).
	friend class URopeSimSubsystem;

public:
	URopeComponent();

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void SendRenderDynamicData_Concurrent() override;
	// 에디터(서브시스템 틱 없음)·스폰 직후에도 로프가 보이도록: 등록 시 Sim을 초기화하고,
	// 렌더 상태 생성 직후 센터라인을 1회 푸시한다(틱 없이도 BuildTube가 돌아 bHasData=true).
	virtual void OnRegister() override;
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
#if WITH_EDITOR
	// 에디터에서 NumParticles/RopeLength를 바꾸면 Sim을 새 값으로 재구성한다(프록시 토폴로지와 매칭).
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	/**
	 * 시뮬레이션 한 프레임을 3단계로 나눠 URopeSimSubsystem이 구동한다(컴포넌트는 직접 tick하지 않음).
	 *  Prepare(GT)  : init/pin/provider gather + collider 스냅샷 + 로직 phase 처리.
	 *  Solve(병렬)  : Free/Flight의 Solver.Step만 — POD + const collider라 스레드 안전.
	 *  Finalize(GT) : Flight 접촉 감지/캡처(UObject·이벤트) + 렌더 dirty.
	 */
	void PrepareSimFrame(float DeltaTime);
	void SolveSimFrame(float DeltaTime);
	void FinalizeSimFrame(float DeltaTime);

	//~ UPrimitiveComponent / UMeshComponent
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

	//~ Setup(설정) -------------------------------------------------------
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "2"))
	int32 NumParticles = 24;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeSolverConfig SolverConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeThrowParams ThrowParams;

	/** physics → logic (wrap) 핸드오프를 위한 contact-decision 튜닝 값. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	FRopeWrapConfig WrapConfig;

	/** rope가 wrap될 수 있는 skeletal mesh. null로 두면 owner로부터 자동 해석된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	TObjectPtr<USkeletalMeshComponent> WrapTargetMesh = nullptr;

	/**
	 * 기본적으로 rope는 월드의 모든 collider provider와 충돌하되 **자기 owner(던진 본인)의 provider는 제외**한다
	 * — throw 시 늘어진 로프가 던진 사람 팔다리에 엉키는 것을 막기 위함. cross-actor wrap(다른 액터 body 잡기)은
	 * 그 액터가 "전체"에 포함되므로 자동으로 동작한다.
	 * 켜면 owner provider도 포함한다(로프가 자기 owner 몸을 일부러 감아야 하는 드문 경우).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	bool bIncludeOwnerColliders = false;

	//~ Render(렌더) ------------------------------------------------------
	/** 시각적 tube 반지름(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render", meta = (ClampMin = "0.1", Units = "cm"))
	float Radius = 2.0f;

	/** tube 단면의 변 개수. 높을수록 더 둥글어진다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render", meta = (ClampMin = "3", ClampMax = "32"))
	int32 NumSides = 8;

	/** rope tube에 적용되는 material. 설정하지 않으면 엔진 기본 material을 사용한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render")
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

#if WITH_EDITORONLY_DATA
	/** 에디터에서 이 로프 액터를 선택했을 때 배치-보조 가이드(앵커·조준·도달범위·wrap 타깃·던지기 아크)를
	 *  FRopeComponentVisualizer가 그릴지 여부. 레벨 에디터 전용(런타임/쿠킹 제외). */
	UPROPERTY(EditAnywhere, Category = "Rope|Debug")
	bool bShowPlacementGuides = true;
#endif

	//~ API ---------------------------------------------------------------
	/** rope를 발사한다: AimDir 방향의 초기 tip 속도를 가지고 Flight phase로 진입한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw(const FVector& AimDir);

	/** 현재 wrap을 수동으로 해제한다(Releasing phase). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ReleaseWrap();

	UFUNCTION(BlueprintCallable, Category = "Rope")
	ERopePhase GetPhase() const { return Phase; }

	FName GetWrappedBoneName() const { return WrapController.State.BoneName; }

	const TArray<FVector>& GetCenterlinePositions() const { return Sim.Positions; }

	/**
	 * Debug: sustained-contact gate(MinLatchNodes / WrapDecisionTime)를 우회하여, rope가 현재 가장
	 * 가깝거나 접촉 중인 bone에 즉시 wrap을 commit한다. throw를 튜닝하지 않고도 BeginWrap 핸드오프와
	 * Hold(bone-follow)를 관찰할 수 있게 해 준다. 접촉이 없으면 false를 반환한다.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope|Debug")
	bool DebugForceWrap();

	//~ Events(이벤트) ----------------------------------------------------
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnWrapped OnRopeWrapped;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnCaptured OnRopeCaptured;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnReleased OnRopeReleased;

	//whip swing
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Whip")
	float WhipElapsed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.01", ClampMax = "1.0", Units = "s"))
	float WhipDuration = 0.35f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.1", ClampMax = "0.95"))
	float WhipGuidedLength = 0.65f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "1.0", ClampMax = "180.0", Units = "deg"))
	float WhipSweepAngleDegrees = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.0"))
	float WhipFollowRate = 18.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.01", ClampMax = "1.0", Units = "s"))
	float WhipWaveTravelTime = 0.18f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.0", Units = "cm"))
	float WhipArcHeight = 120.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.0", Units = "cm"))
	float WhipSideOffset = 35.0f;
private:
	ERopePhase Phase = ERopePhase::Free;

	// Non-UObject sim/solver/logic — 값으로 소유하며, GC 추적 대상이 아니다(POD).
	FRopeSimState       Sim;
	FRopeXPBDSolver     Solver;
	FRopeWrapController WrapController;
	FRopeContactTracker ContactTracker;
	FRopeWrapState      PendingWrapSeed;	//초기 연결용, 임시 Seed
	FRopeWrappingState  WrappingState;

	float ReleaseCooldown = 0.0f;
	float ContactingElapsed = 0.0f;
	float FlightNoContactElapsed = 0.0f;
	float WrappedSwayTime = 0.0f;
	FVector WrappedSwayImpulse = FVector::ZeroVector;

	//whip swing
	bool bWhipSwingActive = false;

	FVector WhipAimDir = FVector::ForwardVector;
	FVector WhipGuideOrigin = FVector::ZeroVector;
	FVector WhipGuideForward = FVector::ForwardVector;
	FVector WhipGuideUp = FVector::UpVector;

	TArray<int32> DebugWhipGuideNodeIndices;
	TArray<FVector> DebugWhipGuideTargets;
	TArray<FVector> PreviousWhipGuideTargets;
	TArray<FVector> WhipGuidePrevTargetsThisFrame;
	TArray<FVector> WhipGuideCurrentTargetsThisFrame;
	TArray<uint8> WhipGuidedNodesThisFrame;

	// 한 프레임 collider 스냅샷. RopeSimSubsystem이 Tick에서 중앙 수집해 채운다(provider 레지스트리 → 로프 필터).
	// Solve/Finalize에서 read. provider 소유라 raw 포인터(해당 프레임 동안 유효).
	TArray<IRopeCollider*> FrameColliders;

	// 이번 프레임에 Solver.Step을 돌릴지(Free/Flight만 true).
	bool bSolveThisFrame = false;

	// GPU 상주 솔버(M5)용 시드 generation. Sim을 out-of-band로 바꾼 시점(init/throw/logic phase/whip)에
	// 증가시킨다 → 서브시스템이 변화를 감지해 GPU 영속 버퍼를 재시드한다. 정상 Free/Flight(비-whip)에선 불변(상주 유지).
	uint32 SimGeneration = 0;

	// 이번 프레임에 이 로프가 실제로 GPU에서 step됐는가(서브시스템이 매 프레임 설정). M5b: GPU 튜브 렌더가
	// resident PosBuf를 직접 읽을지(true) CPU Sim 미러로 그릴지(false, whip/CPU-폴백/솔버 off) 가른다.
	bool bGpuSteppedThisFrame = false;

	void InitRope();

#if WITH_GAMEPLAY_DEBUGGER
	// 디버그 캡처 대상일 때 centerline/wrapped/collider 공통 필드를 스냅샷에 채운다(FinalizeSimFrame에서 호출).
	void FillDebugSnapshot(FRopeDebugSnapshot& Snapshot) const;
#endif

	//TODO 주석 추가
	void EnsureRopeInitialized(){if (Sim.Num() == 0)InitRope();	}

	/** rope가 wrap할 skeletal mesh를 해석(및 캐싱)한다: 명시적 WrapTargetMesh 또는 owner의 것. */
	USkeletalMeshComponent* ResolveWrapTargetMesh();

#pragma region Throw 관련 함수
	void StartFreshThrow(const FVector& AimDir);

	void ThrowFreeSpanWhileWrapped(const FVector& AimDir);

	void BuildWhipGuideTargets(float NormalizedTime, int32 LastGuidedNode, TArray<FVector>& OutTargets) const;

	void ResampleGuideByNodeSpacing(const TArray<FVector>& SourcePoints, float TotalLength, int32 NodeCount,
		int32 DesiredPointCount, TArray<FVector>& OutPoints) const;

	float TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const;

#pragma endregion

#pragma region Flight 관련 함수
	void ApplyWhipSwing(float DeltaTime);

	void DetectContactCandidates(const TArray<FVector>& PrevPositions, const TArray<FVector>& Positions,
		const TArray<IRopeCollider*>& Colliders, TArray<FRopeContactCandidate>& OutCandidates) const;

	void AddPredictedContactCandidates(TArray<FRopeContactCandidate>& InOutCandidates, float DeltaTime) const;

	void EvaluateRelativeMotion(TArray<FRopeContactCandidate>& Candidates) const;

	FVector ExpectedWrapTangent(const FRopeContactCandidate& Candidate) const;

	bool ShouldCapture(const TArray<FRopeContactCandidate>& Candidates) const;

	void BuildContactingState(const TArray<FRopeContactCandidate>& Candidates);

	bool IsTailNode(int32 NodeIndex) const;

	bool IsWhipGuidedNodeThisFrame(int32 NodeIndex) const;

	bool ShouldRunPredictiveContactForNode(int32 NodeIndex, bool bHasGuidedNodes, const FVector& FrameDisplacement) const;

	float NodeSpeed(int32 NodeIndex) const;

	bool IsNearAnyColliderSegment(const FVector& PrevPosition, const FVector& Position, const TArray<IRopeCollider*>& Colliders) const;

	void GatherNearbyColliders(const FVector& PrevPosition, const FVector& Position,
		const TArray<IRopeCollider*>& Colliders, TArray<IRopeCollider*>& OutNearbyColliders) const;

	FRopeContact SweepOrSampleContact(const FVector& PrevPosition, const FVector& Position, const TArray<IRopeCollider*>& Colliders) const;

	FRopeContactCandidate MakeCandidate(int32 NodeIndex, const FRopeContact& Contact) const;

	bool IsWrappableBone(FName Bone) const { return !Bone.IsNone(); }

#pragma endregion

#pragma region Contacting 관련 함수

	void AdvanceWrappingMotion(float DeltaTime);

	bool ShouldDismissContacting() const;

	// 기존(legacy)
	//bool ShouldFinishWrapping() const;

	// 변경
	bool ShouldStartWrapping() const;

	FRopeWrapState BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const;

	bool ShouldCommitWrap(const FRopeContactTracker& Tracker) const;

#pragma endregion

#pragma region Wrapping 관련 함수

	void UpdateContacting(float DeltaTime);

	void StartWrappingFromContacting();

	void UpdateWrapping(float DeltaTime);

	bool IsWrappingStillValid() const;

	bool BuildWrappingAnchorsFromLatch(const FRopeSurfaceAnchor& LatchAnchor);

	bool ComputeWrapSurfaceTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
		FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const;

	/*
	 * Surface Walk은 Project Settings 선택지에서 제거했다.
	 * 최초 latch tangent만 따라가서 원주를 의도적으로 도는 힘이 약하므로,
	 * 현재는 Analytic Helix / Surface Vector Field 두 방식만 사용한다.
	bool ComputeSurfaceWalkWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
		FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const;
	 */

	bool ComputeAnalyticHelixWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
		FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const;

	bool ComputeSurfaceVectorFieldWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
		FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const;

	bool ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor,
		FVector& OutAxisOrigin, FVector& OutAxisDirection) const;

	ERopeWrappingPathMode GetWrappingPathMode() const;

	bool ProjectWrapPointToSurface(FName Bone, const USkeletalMeshComponent* Mesh,
		FVector& InOutSurfaceWorld, FVector& InOutNormalWorld) const;

	void ApplyWrappingTargetMotion(float DeltaTime);

	bool UpdateWrappingAnchorsFromCandidates(const TArray<FRopeContactCandidate>& Candidates);

	void ApplyWrappingMassMask();

	void CommitWrapping();

	void AbortWrapping(ERopeReleaseReason Reason);

#pragma endregion

#pragma region Wrapped 관련 함수

	void UpdateWrappedKinematicShape(float DeltaTime);

	void ApplyWrappedMassMask();

#pragma endregion
};
