// Copyright Epic Games, Inc. All Rights Reserved.
//
// 단일 UE 통합 지점(Facade). sim 상태(FRopeSimState)와 솔버, 페이즈별 로직 F-클래스들
// (WhipGuide/WrappingPhase/WrapController)을 값으로 소유하고, physics와 logic을 분기하는
// phase state machine(ERopePhase)을 굴린다. 페이즈별 실제 동작은 Logic/ 클래스에 있고,
// 이 컴포넌트에는 전이·이벤트 브로드캐스트를 결정하는 오케스트레이션만 남는다.
// 캐릭터에 붙이고 Throw()로 사용한다.

#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "Core/RopeTypes.h"
#include "Solver/RopeXPBDSolver.h"
#include "Logic/RopeWrapController.h"
#include "Logic/RopeWhipGuide.h"
#include "Logic/RopeFlightContactDetector.h"
#include "Logic/RopeWrappingPhase.h"
#include "RopeComponent.generated.h"

class AActor;
class IRopeCollider;
class IRopeColliderProvider;
class UMaterialInterface;
class UMaterialInstanceDynamic;
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

	// 서브시스템이 프레임 구동을 위해 Sim/SolverConfig/Phase/bSolveThisFrame/SimGeneration/
	// bGpuSteppedThisFrame/WhipGuide에 직접 접근한다(GPU 배치 솔브 포함; CPU 경로는 SolveSimFrame 사용).
	friend class URopeSimSubsystem;

public:
	URopeComponent();

	//~ Setup(설정) -------------------------------------------------------
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "2"))
	int32 NumParticles = 24;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 200.0f;

	/** 되감기(reel-in)로 줄일 수 있는 최소 길이(cm). RopeLength(초기)가 상한이다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "10.0", Units = "cm"))
	float MinRopeLength = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeSolverConfig SolverConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeThrowParams ThrowParams;

	/** physics → logic (wrap) 핸드오프를 위한 contact-decision 튜닝 값. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	FRopeWrapConfig WrapConfig;

	/**
	 * 에디터 배치 가이드용(비주얼라이저가 이 메시로 wrap 타깃 링크를 그린다) + bIncludeOwnerColliders로
	 * 자기 몸을 감는 드문 케이스의 명시 지정용. 런타임에 실제로 감기는 메시는 이 값이 아니라 접촉에서
	 * 확정된다(FRopeContact.SourceMesh → PendingWrapSeed → FRopeWrapState.Mesh) — cross-actor 포함.
	 */
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

	//~ Whip(던지기 스윙 설정) ----------------------------------------------
	/** 던지기 초반 채찍 스윙 튜닝. 런타임 상태는 WhipGuide가 소유하고, 호출 시
	 *  MakeWhipGuideConfig()로 스냅샷을 만들어 넘긴다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip")
	FRopeWhipConfig WhipConfig;

	/** WhipGuide.GetElapsed()의 BP 노출용 미러(매 프레임 갱신). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Whip")
	float WhipElapsed = 0.0f;

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

	/** 꼬임(strand) 패턴 밀도를 rope length에 비례시켜 자동 조정할지. 켜면 런타임에 dynamic material instance로
	 *  머티리얼이 저작한 TwistTurns에 (RopeLength / 기준 200cm)를 곱해 세팅한다 → 로프가 길어져도 꼬임 간격이
	 *  일정하고, 프리셋별 상대 밀도(예: 파라코드가 더 촘촘)는 보존된다. 끄면 머티리얼 원본을 그대로 사용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render")
	bool bScaleTwistByLength = true;

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

	/** Wielder가 origin/frame/속도까지 계산해 넘기는 확장 throw 진입점. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowWithContext(const FRopeThrowContext& ThrowContext);

	/** 로프 길이와 현재 whip/swing 설정을 반영한 던지기 전 미리보기 호 데이터를 만든다. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	bool BuildThrowArcPreview(const FRopeThrowContext& ThrowContext, float ReachScale, int32 SegmentCount,
		FRopeArcPreviewData& OutPreview) const;

	/** 현재 프레임 collider 스냅샷 기준으로 미리보기 호가 막히는 첫 각도를 찾는다. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	bool FindThrowArcPreviewHit(const FRopeArcPreviewData& Preview, float SampleStep, float QueryRadius,
		FRopeArcPreviewHitResult& OutHit) const;

	/** Builds the current pre-wrapped rope centerline preview from the active/contacting wrap seed. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	bool BuildWrappingPreview(FRopeWrapPreviewData& OutPreview) const;

	/** Builds a pre-wrapped preview for idle/flight aiming using the same throw context as ThrowWithContext. */
	bool BuildWrappingPreview(const FRopeThrowContext& ThrowContext, float ReachScale, int32 SegmentCount,
		float SampleStep, float QueryRadius, FRopeWrapPreviewData& OutPreview) const;

	/** 현재 진행 중인 잡기/감기(Contacting/Wrapping/Wrapped)를 수동으로 해제한다(Releasing phase). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ReleaseWrap();

	/**
	 * 로프 절단(외부 게임플레이 — 칼질/데미지 등): 진행 중인 잡기/감기를 ERopeReleaseReason::Cut으로
	 * 강제 해제한다. 흐름은 ReleaseWrap과 같고 사유만 달라 게임이 구분 반응(로프 파괴 연출 등)할 수
	 * 있다. 로프 자체를 두 조각으로 나누는 물리적 절단은 후속(길이 변경/분할 시뮬 필요).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void CutRope();

	UFUNCTION(BlueprintCallable, Category = "Rope")
	ERopePhase GetPhase() const { return Phase; }

	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool IsTensioned(float SlackTolerance = 5.0f) const;

	/**
	 * 세그먼트(SegmentIndex = 노드 i~i+1) 장력. 솔버의 XPBD distance λ에서 유도한 힘(F=max(0,-λ)/h²,
	 * 질량 1 노드 기준 상대 단위 — 매달린 노드 1개의 중력 하중 ≈ 980). 스트레치만 양수, 슬랙/압축 = 0.
	 * GPU 상주 로프는 1~2프레임 지연 미러. 솔브가 없는 페이즈(Contacting/Releasing)는 직전 값 유지.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	float GetSegmentTension(int32 SegmentIndex) const;

	/** 전체 세그먼트 중 최대 장력. Wrapped 중에는 매 프레임 FRopeWrapState::Tension에도 반영된다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	float GetMaxTension() const;

	/**
	 * 이번 프레임 Pull(당김) 데이터: 손 쪽 첫 앵커가 받는 당김 방향(단위)과 그 세그먼트 장력.
	 * Wrapped 동안 매 프레임 산출된다. 게임 효과(포획 진행도, 이동 방해 등) 판정용.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool GetPullSample(FVector& OutDirection, float& OutTension) const
	{
		OutDirection = LastPullSample.Direction;
		OutTension = LastPullSample.Tension;
		return LastPullSample.bValid;
	}

	/**
	 * 능동 Pull(당김) 힘 설정 — Wrapped + 로프가 팽팽할 때 매 프레임 이 크기의 *상수* 힘을 감긴
	 * 대상에 인가한다(장력과 무관 → 피드백 폭주 없음). 0 = 정지. 입력 홀드 동안 켜고 떼면 끄는
	 * 용도(URopeWielderComponent의 PullAction이 이걸 호출). 캐릭터 대상은 CharacterMovement가
	 * 질량으로 나누고 지면 마찰과 경쟁하므로 수만~수십만 단위가 체감 구간이다.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetActivePull(float Force);

	/** 현재(런타임) 로프 길이(cm). 되감기/풀기로 변한다 — 초기값/상한은 RopeLength. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	float GetCurrentRopeLength() const { return Sim.RopeLength; }

	/**
	 * 로프 길이를 직접 설정(되감기/풀기의 즉시형). [MinRopeLength, RopeLength(초기)]로 클램프.
	 * 노드 수는 유지되고 세그먼트 rest 길이가 균일하게 변한다 — 재시드 없이 솔버(CPU/GPU 동일)에
	 * 다음 프레임부터 반영된다. Wrapped 중 줄이면 가용 로프 길이가 줄어 테더가 대상을 끌어오고,
	 * 테더가 없으면 장력이 오른다(TensionRelease와 조합 가능).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetRopeLength(float NewLength);

	/**
	 * 되감기 속도 설정(cm/s). 양수 = 감기(짧아짐), 음수 = 풀기(길어짐, 초기 길이까지), 0 = 정지.
	 * Free/Flight/Wrapped에서 매 프레임 적용된다(Contacting/Wrapping/Releasing은 일시 보류 —
	 * 경로 생성이 SegmentLength에 의존). 입력 홀드 용도(URopeWielderComponent의 ReelIn/Out 액션).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetReelRate(float CmPerSecond);

	/** 슬립 중인가(Free 페이즈에서 정지 판정 — 솔브 스킵 상태). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool IsSleeping() const { return bAsleep; }

	/** 현재 거리 LOD의 iteration 배율(1=풀 품질). 디버그/프로파일 확인용. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	float GetSolverLODScale() const { return SolverLODScale; }

	FName GetWrappedBoneName() const { return WrapController.State.BoneName; }

	const TArray<FVector>& GetCenterlinePositions() const { return Sim.Positions; }

	//~ Events(이벤트) ----------------------------------------------------
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnWrapped OnRopeWrapped;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnCaptured OnRopeCaptured;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnReleased OnRopeReleased;

private:
	/**
	 * 시뮬레이션 한 프레임을 3단계로 나눠 URopeSimSubsystem이 구동한다(friend 접근;
	 * 컴포넌트는 직접 tick하지 않고, 외부 게임 코드가 부를 일도 없어 private).
	 *  Prepare(GT)  : init/pin 전진 + 로직 phase 처리. collider 스냅샷(FrameColliders)은
	 *                 서브시스템이 이 호출 전에 중앙 수집해 채워 둔다.
	 *  Solve(병렬)  : bSolveThisFrame(Free/Flight/Wrapped)일 때 Solver.Step — POD + const collider라
	 *                 스레드 안전. Wrapped는 latch 노드가 InvMass=0이라 자유 구간만 물리로 움직인다.
	 *  Finalize(GT) : Flight 접촉 감지/캡처(UObject·이벤트) + 렌더 dirty.
	 */
	void PrepareSimFrame(float DeltaTime);
	void SolveSimFrame(float DeltaTime);
	void FinalizeSimFrame(float DeltaTime);

public:
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

	//~ UPrimitiveComponent / UMeshComponent
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

private:
	// 길이 의존 머티리얼 파라미터(꼬임 밀도)를 dynamic material instance로 갱신한다:
	// 저작된 TwistTurns × (RopeLength / 기준 200cm) → rope가 길어져도 꼬임 간격이 일정(프리셋 밀도 보존).
	// RopeMaterial/RopeLength/bScaleTwistByLength 변경 시 호출. GetMaterial은 이 MID를 우선 반환한다.
	void UpdateRopeMaterialDynamicParams();

	// UpdateRopeMaterialDynamicParams가 만드는 런타임 인스턴스(부모 = RopeMaterial/프리셋). 길이 의존 파라미터용.
	// bScaleTwistByLength=false거나 RopeMaterial에 TwistTurns가 없으면 nullptr(원본 머티리얼을 그대로 사용).
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInstanceDynamic> RopeMID = nullptr;

	//~ 페이즈 상태 머신 ----------------------------------------------------
	ERopePhase Phase = ERopePhase::Free;

	/**
	 * Phase 대입의 단일 지점. 전이 로그("[이름] Old -> New (Reason)")를 일원화한다.
	 * Reason은 로그용 부가 설명(nullptr이면 생략). 전이에 딸린 이벤트 브로드캐스트와
	 * cleanup은 전이마다 다르므로 호출자가 결정한다 — 여기서 암묵적으로 하지 않는다.
	 */
	void SetPhase(ERopePhase NewPhase, const TCHAR* Reason = nullptr);

	/**
	 * 페이즈 전이 시 함께 폐기해야 하는 "진행 중 작업" 일시 상태 세트를 리셋한다:
	 * ContactTracker / PendingWrapSeed / WrappingPhase.State / ContactingElapsed / FlightNoContactElapsed / TensionOverTime.
	 * 유휴 상태의 멤버에 대해서는 no-op이라 어떤 전이에서 불러도 안전하다.
	 * (ReleaseCooldown은 전이마다 값이 달라 호출자가 직접 설정한다.)
	 */
	void ResetTransientPhaseState();

	//~ 시뮬레이션 상태 + 페이즈별 로직 소유물 -------------------------------
	// Non-UObject — 값으로 소유하며 GC 추적 대상이 아니다(POD/약참조만 보유).
	// 아래 로직 4개는 로프 수명 순서와 1:1 대응한다: Throw/Flight → Contacting → Wrapping → Wrapped.
	FRopeSimState       Sim;				// 단일 진실: 솔버/로직/렌더가 공유하는 파티클 체인
	FRopeXPBDSolver     Solver;				// XPBD 물리(Free/Flight/Wrapped 자유 구간)
	FRopeWhipGuide      WhipGuide;			// Throw/Flight: 채찍 스윙(가이드 타깃 계산+적용)
	FRopeContactTracker ContactTracker;		// Flight/Contacting: 접촉 후보의 dominant bone 추적
	FRopeWrapState      PendingWrapSeed;	// Contacting: 캡처 시 만들어 둔 wrap 시드(Wrapping 진입 재료)
	FRopeWrappingPhase  WrappingPhase;		// Wrapping: 경로 점진 생성+front 모션+마스크(작업 상태는 .State)
	FRopeWrapController WrapController;		// Wrapped: bone-local latch 유지/해제

	//~ 페이즈 타이머 --------------------------------------------------------
	float ContactingElapsed = 0.0f;	// Contacting 체류 시간(WrapDecisionTime 판정)
	float FlightNoContactElapsed = 0.0f;	// Whip 종료 후 캡처 없이 Flight에 머문 시간
	float ReleaseCooldown = 0.0f;	// Releasing → Free 복귀까지 남은 시간
	float TensionOverTime = 0.0f;	// Wrapped 중 최대 장력이 TensionReleaseForce를 연속 초과한 시간

	// 이번 프레임 Pull 산출물(Wrapped 동안 매 프레임 산출). BP 조회/디버거 화살표 소스.
	FRopePullSample LastPullSample;

	// 능동 Pull의 현재 힘(SetActivePull이 설정, 0=꺼짐). Wrapped + 팽팽할 때만 인가된다.
	float ActivePullForce = 0.0f;

	// 되감기 속도(cm/s, +감기/-풀기, 0=정지). SetReelRate가 설정, UpdateReel이 프레임마다 적용.
	float ReelRate = 0.0f;

	// 되감기 프레임 적용(Prepare 초입): 허용 페이즈에서 ReelRate × dt만큼 길이를 조정한다.
	void UpdateReel(float DeltaTime);

	//~ 슬립/LOD(스케일링) ---------------------------------------------------
	bool  bAsleep = false;                    // Free 정지 판정으로 솔브 스킵 중
	float SleepTimer = 0.0f;                  // 저속 유지 누적(초)
	FVector SleepPinPos = FVector::ZeroVector; // 슬립 진입 시 핀 위치(이동 시 wake)
	TArray<FVector> SleepPrevFramePositions;  // 프레임간 변위 측정 캐시(Finalize에서 갱신)
	float SolverLODScale = 1.0f;              // 거리 LOD iteration 배율(Prepare가 계산, 1=풀)

	// 거리 LOD 배율 계산(Prepare, GT — 카메라 접근). 카메라 없으면(서버) 1 유지.
	void ComputeSolverLOD();
	// 슬립 전이 측정(Finalize, Free 전용): 프레임간 최대 노드 속도가 임계 미만이 SleepDelay 지속 → 슬립.
	void UpdateSleepState(float DeltaTime);
	// 슬립 해제 판정(Prepare): 핀 이동/되감기/움직이는 근접 collider.
	bool ShouldWakeFromSleep() const;
	// LOD 반영된 유효 iteration(CPU 솔브/GPU 스텝 공용).
	int32 GetLODScaledIterations() const
	{
		return FMath::Max(1, FMath::RoundToInt(static_cast<float>(SolverConfig.Iterations) * SolverLODScale));
	}

	// 이번 프레임 테더 초과분(cm) — 손~앵커 직선 거리 - 가용 로프 길이(0 미만은 0). 디버거 표시용.
	float LastTetherOvershoot = 0.0f;

	// Pull 힘 수신자 없음 경고를 wrap당 1회만 내보내기 위한 래치(ResetTransientPhaseState에서 리셋).
	bool bLoggedPullNoReceiver = false;

	// 동작 1 — 자동 견인(테더): 가용 로프 길이 초과분을 위치/속도 동기로 회수(수렴, 폭주 없음).
	void UpdateTether(float DeltaTime);

	// 모든 release 트리거의 공용 마무리(페이즈 전환+노드 반환+일시 상태 폐기+쿨다운+이벤트).
	void FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog);

	// ReleaseWrap/CutRope 공용 본체: 진행 중인 잡기/감기를 주어진 사유로 해제(본 귀속 해석 포함).
	void ReleaseWrapAs(ERopeReleaseReason Reason);

	// 동작 2 — Pull 힘 인가(GT, UObject): 물리 시뮬 본 → 캐릭터 무브먼트 → 시뮬 루트 순으로 시도한다.
	void ApplyPullForce(const FVector& Force, const FRopePullSample& Pull);

	//~ 서브시스템 프레임 계약(RopeSimSubsystem이 쓰거나 읽는다) --------------
	// 한 프레임 collider 스냅샷. RopeSimSubsystem이 Tick에서 중앙 수집해 채운다(provider 레지스트리 → 로프 필터).
	// Prepare/Solve/Finalize에서 read. provider 소유라 raw 포인터(해당 프레임 동안 유효).
	TArray<IRopeCollider*> FrameColliders;

	// 이번 프레임에 Solver.Step을 돌릴지. Free/Flight/Wrapped true(Wrapped는 latch 노드 InvMass=0),
	// Contacting/Wrapping/Releasing은 로직 구동이라 false.
	bool bSolveThisFrame = false;

	// 로직 페이즈의 한 프레임 산출물(G2). Prepare 동안 로직(Wrapping/Wrapped/Releasing 등)이 위치·질량을
	// 여기에 scatter하면 Prepare 끝에서 CPU Sim에 1회 적용되고, GPU 상주 로프에는 서브시스템이 같은
	// 데이터를 override 패스로 실어 재시드 없이 커널에서 적용한다. 매 Prepare 시작에 리셋(프레임 스코프).
	FRopeNodeOverrideFrame OverrideFrame;

	// GPU 상주 솔버(M5)용 시드 generation. 진짜 시드(init/throw/노드 수 변경)에만 증가한다 →
	// 서브시스템이 변화를 감지해 GPU 영속 버퍼를 재시드한다. 로직 페이즈/whip의 위치·질량 쓰기는
	// override 패스로 주입되므로(G1/G2) 재시드하지 않는다 — 상주가 페이즈 전체에 걸쳐 유지된다.
	uint32 SimGeneration = 0;

	// 이번 프레임에 이 로프가 실제로 GPU에서 step됐는가(서브시스템이 매 프레임 설정). M5b: GPU 튜브 렌더가
	// resident PosBuf를 직접 읽을지(true) CPU Sim 미러로 그릴지(false, CPU-폴백/솔버 off) 가른다.
	// whip 프레임도 G1부터 GPU(override 주입)라 true — PosBuf가 가이드 타깃을 같은 프레임에 반영한다.
	bool bGpuSteppedThisFrame = false;

	// GPU 접촉 감지(G3) 귀속 테이블: GPU가 emit한 콜라이더 인덱스 → (bone, mesh) 복원용.
	// 서브시스템이 GPU step 프레임마다 Step.Capsules/SDFColliders와 같은 순서로 채운다. mesh는 지연
	// 동안 파괴될 수 있어 weak. (콜라이더 집합이 프레임 간 바뀌면 인덱스가 어긋날 수 있으나 순서가
	//  안정적이고 범위 밖은 무시 → stale 미러 감지와 동일한 관용도. 최악의 경우 한 프레임 오귀속, 자기수정.)
	struct FGpuColliderAttribution
	{
		FName Bone = NAME_None;
		TWeakObjectPtr<const USkeletalMeshComponent> Mesh;
	};
	TArray<FGpuColliderAttribution> GpuCapsuleAttribution; // GPU Capsules와 평행
	TArray<FGpuColliderAttribution> GpuSdfAttribution;     // GPU SDFColliders와 평행

	// GPU 감지(G3) 프레임 산출: 서브시스템이 GetLatestContacts를 귀속해 Finalize 전에 채운다.
	// bValid면 FinalizeSimFrame의 Flight 접촉 소스가 CPU 스윕 대신 이 후보들을 쓴다(GPU 경로).
	TArray<FRopeContactCandidate> GpuFlightCandidates;
	bool bGpuContactsThisFrame = false;

	//~ 초기화/유틸 ----------------------------------------------------------
	void InitRope();

	/** Sim이 비어 있으면 1회 초기화한다(OnRegister/Throw/Prepare 초입의 안전 가드). */
	void EnsureRopeInitialized() { if (Sim.Num() == 0) { InitRope(); } }

#if WITH_GAMEPLAY_DEBUGGER
	// 디버그 캡처 대상일 때 centerline/wrapped/collider 공통 필드를 스냅샷에 채운다(FinalizeSimFrame에서 호출).
	void FillDebugSnapshot(FRopeDebugSnapshot& Snapshot) const;
#endif

	//~ Throw ----------------------------------------------------------------
	FRopeThrowContext MakeDefaultThrowContext(const FVector& AimDir) const;

	void StartFreshThrow(const FRopeThrowContext& ThrowContext);

	FRopeThrowContext ResolveThrowContext(const FRopeThrowContext& ThrowContext) const;

	FVector ComputeThrowInheritedVelocity(const FRopeThrowContext& ThrowContext) const;

	/** WhipGuide에 넘길 설정 스냅샷을 Rope|Whip UPROPERTY들로부터 만든다. */
	FRopeWhipGuide::FConfig MakeWhipGuideConfig() const;

	/** 던지기 임펄스의 tail 가중치(FirstTailNode부터 끝까지 0→1 스무스). */
	float TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const;

	//~ Flight ---------------------------------------------------------------
	// 접촉 감지 파이프라인 자체는 FRopeFlightContactDetector(정적, UObject 비의존)로 분리됐다.
	// 여기엔 UObject 컨텍스트가 필요한 조립 코드만 남는다.

	/** 검출기에 넘길 파라미터 스냅샷(WrapConfig + 튜브 반지름 + 컴포넌트 전방). */
	FRopeFlightContactDetector::FParams MakeFlightDetectParams() const;

	/** 캡처 확정 시 Contacting 진입 상태(ContactTracker/PendingWrapSeed/타이머)를 구성한다. */
	void BuildContactingState(const TArray<FRopeContactCandidate>& Candidates);

	//~ Contacting -----------------------------------------------------------
	void UpdateContacting(float DeltaTime);

	void AdvanceWrappingMotion(float DeltaTime);

	bool ShouldDismissContacting() const;

	bool ShouldStartWrapping() const;

	FRopeWrapState BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const;

	//~ Wrapping -------------------------------------------------------------
	// 경로 생성/front 모션/마스크 등 Wrapping 페이즈의 실제 로직은 FRopeWrappingPhase(WrappingPhase)로
	// 분리됐다. 여기엔 페이즈 전이·이벤트를 결정하는 오케스트레이션만 남는다.

	void StartWrappingFromContacting();

	void UpdateWrapping(float DeltaTime);

	/** 프로젝트 설정(UDynamicRopeSettings)에서 감김 경로 모드를 해석한다. */
	ERopeWrappingPathMode GetWrappingPathMode() const;

	/** WrappingPhase에 넘길 호출 컨텍스트(WrapConfig/collider 스냅샷/경로 모드/튜브 반지름/로그 이름). */
	FRopeWrappingPhase::FContext MakeWrappingContext() const;

	bool BuildWrappingPreviewFromCandidate(const FRopeContactCandidate& Candidate, const FRopeSimState& SourceSim,
		FRopeWrapPreviewData& OutPreview) const;

	bool BuildFreeWrappingPreview(const FRopeThrowContext& ThrowContext, float ReachScale, int32 SegmentCount,
		float SampleStep, float QueryRadius, FRopeWrapPreviewData& OutPreview) const;

	bool BuildFlightWrappingPreview(FRopeWrapPreviewData& OutPreview) const;

	void CommitWrapping();

	void AbortWrapping(ERopeReleaseReason Reason);

	//~ Wrapped --------------------------------------------------------------
	/** latch/anchor 노드 InvMass=0, 나머지 1 — Wrapped 중 자유 구간만 솔버가 움직이게. */
	void ApplyWrappedMassMask(bool bResetDynamicNodeVelocity = false);

	bool ComputeTensionSlack(float& OutSlack, float& OutStraightDistance, float& OutAvailableLength) const;
};
