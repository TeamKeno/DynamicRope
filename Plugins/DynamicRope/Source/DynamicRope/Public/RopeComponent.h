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
class USceneComponent; // 랩 대상 추상화(Decision 0): 랩 대상 mesh를 USceneComponent로 일반화
class FRegisterComponentContext;
struct FRopeDebugSnapshot;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnWrapped, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnCaptured, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnReleased, FName, Bone, ERopeReleaseReason, Reason);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnPhaseChanged, ERopePhase, OldPhase, ERopePhase, NewPhase);

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
	// 아래 초기화 전용 값들(NumParticles/RopeLength/MinRopeLength)은 InitRope 시점에만 소비된다 —
	// 런타임 쓰기는 재초기화 전까지 무효라 BlueprintReadOnly(함정 방지). 런타임 길이 변경은
	// SetRopeLength/SetReelRate를 쓴다.

	// ClampMax 512 = FRopeGPUSolver::MaxNodes(GPU 솔버 스레드그룹 상한). 초과하면 조용히 CPU 솔브+튜브
	// 폴백이 되어 성능 절벽 + 저작 무신호라 에디터에서 막는다(BP/코드 경로는 InitRope가 하드 클램프).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "2", ClampMax = "512"))
	int32 NumParticles = 24;

	/** 초기(최대) 로프 길이(cm). 런타임 현재 길이는 GetCurrentRopeLength/SetRopeLength. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 200.0f;

	/** 되감기(reel-in)로 줄일 수 있는 최소 길이(cm). RopeLength(초기)가 상한이다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "10.0", Units = "cm"))
	float MinRopeLength = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeSolverConfig SolverConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FRopeThrowParams ThrowParams;

	/** physics → logic (wrap) 핸드오프를 위한 contact-decision 튜닝 값. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	FRopeWrapConfig WrapConfig;

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

	/** 현재 whip 스윙 경과 시간(s). 스윙 비활성 시 0. */
	UFUNCTION(BlueprintPure, Category = "Rope|Whip")
	float GetWhipElapsed() const { return WhipGuide.GetElapsed(); }

	//~ Render(렌더) ------------------------------------------------------
	// 아래 렌더 값들(Radius/NumSides/TubeSmoothing*)은 씬 프록시 생성 시 1회 읽혀 굳는다 — 런타임 쓰기는
	// 프록시 재생성 전까지 반영되지 않아 BlueprintReadOnly(에디터 변경은 렌더 상태 재생성으로 반영됨).

	/** 시각적 tube 반지름(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (ClampMin = "0.1", Units = "cm"))
	float Radius = 2.0f;

	/** tube 단면의 변 개수. 높을수록 더 둥글어진다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (ClampMin = "3", ClampMax = "32"))
	int32 NumSides = 8;

	/** 렌더 튜브 스무딩: 세그먼트당 Catmull-Rom 서브분할 수(1=끔). 시뮬 노드는 그대로 두고 렌더 센터라인만
	 *  이웃 노드로 곡률을 추정해 매끄럽게 편다(물리와 분리 — 렌더 전용). 기본 1인 이유: 보간 링은 노드
	 *  폴리라인 바깥으로 부풀 수 있어(특히 벽을 짚는 구간) 노드가 촘촘하면 직선 연결이 더 정확하다.
	 *  성긴 로프만 올려 둥글게 보이게 하고, 오버슈트는 TubeSmoothingAlpha(centripetal)로 줄인다.
	 *  NumRings=(NumParticles-1)*Subdiv+1이 GPU 튜브 링 상한을 넘으면 프록시가 자동 하향한다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (ClampMin = "1", ClampMax = "8"))
	int32 TubeSmoothingSubdiv = 1;

	/** 렌더 튜브 스무딩의 Catmull-Rom knot α: 0=uniform, 0.5=centripetal(급한 코너에서 접선 오버슈트↓ —
	 *  벽을 짚는 구간의 중간 링이 벽 밖으로 덜 부푼다), 1=chordal. CPU 스무딩과 GPU resident 스무딩이
	 *  같은 값을 써 렌더가 일관된다. TubeSmoothingSubdiv=1이면 효과 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TubeSmoothingAlpha = 0.5f;

	/** rope tube에 적용되는 material. 설정하지 않으면 엔진 기본 material을 사용한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render")
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	/** 꼬임(strand) 패턴 밀도를 rope length에 비례시켜 자동 조정할지. 켜면 런타임에 dynamic material instance로
	 *  머티리얼이 저작한 TwistTurns에 (RopeLength / 기준 200cm)를 곱해 세팅한다 → 로프가 길어져도 꼬임 간격이
	 *  일정하고, 프리셋별 상대 밀도(예: 파라코드가 더 촘촘)는 보존된다. 끄면 머티리얼 원본을 그대로 사용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render")
	bool bScaleTwistByLength = true;

#if WITH_EDITORONLY_DATA
	/** 에디터에서 이 로프 액터를 선택했을 때 배치-보조 가이드(앵커·조준·도달범위·던지기 아크)를
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

	//~ Wielder 계약(C++ 전용) ----------------------------------------------
	// URopeWielderComponent의 조준/PreviewPathLocked 흐름이 쓰는 진입점들. 일반 사용자 API가 아니라
	// BP 미노출 — 게임 코드에서 직접 부를 일은 보통 없다(Wielder를 붙이거나 같은 계약을 재구현할 때만).

	/** Builds a pre-wrapped preview for idle/flight aiming using the same throw context as ThrowWithContext. */
	bool BuildWrappingPreview(const FRopeThrowContext& ThrowContext, float ReachScale, int32 SegmentCount,
		float SampleStep, float QueryRadius, FRopeWrapPreviewData& OutPreview,
		FString* OutFailureReason = nullptr) const;

	/** PreviewPathLocked용 preview build. 렌더 centerline뿐 아니라 실제 GuidedThrow/Wrapped 진입에 필요한 contact/anchor도 반환한다. */
	bool BuildPreparedWrappingPreview(const FRopeThrowContext& ThrowContext, float ReachScale, int32 SegmentCount,
		float SampleStep, float QueryRadius, FRopePreparedThrowPreview& OutPrepared,
		FString* OutFailureReason = nullptr) const;

	/** Prepared preview를 권위 있는 경로로 사용해 던진다. Flight/Contacting 재탐색을 타지 않고 GuidedThrow로 진입한다. */
	bool ThrowWithPreparedPreview(const FRopePreparedThrowPreview& Prepared);

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

	UFUNCTION(BlueprintPure, Category = "Rope")
	ERopePhase GetPhase() const { return Phase; }

	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsTensioned(float SlackTolerance = 5.0f) const;

	/**
	 * 세그먼트(SegmentIndex = 노드 i~i+1) 장력. 솔버의 XPBD distance λ에서 유도한 힘(F=max(0,-λ)/h²,
	 * 질량 1 노드 기준 상대 단위 — 매달린 노드 1개의 중력 하중 ≈ 980). 스트레치만 양수, 슬랙/압축 = 0.
	 * GPU 상주 로프는 1~2프레임 지연 미러. 솔브가 없는 페이즈(Contacting/Releasing)는 직전 값 유지.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetSegmentTension(int32 SegmentIndex) const;

	/** 전체 세그먼트 중 최대 장력. Wrapped 중에는 매 프레임 FRopeWrapState::Tension에도 반영된다. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetMaxTension() const;

	/**
	 * 이번 프레임 Pull(당김) 데이터: 손 쪽 첫 앵커가 받는 당김 방향(단위)과 그 세그먼트 장력.
	 * Wrapped 동안 매 프레임 산출된다. 게임 효과(포획 진행도, 이동 방해 등) 판정용.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool GetPullSample(FVector& OutDirection, float& OutTension) const
	{
		OutDirection = LastPullSample.Direction;
		OutTension = LastPullSample.Tension;
		return LastPullSample.bValid;
	}

	/** 이번 프레임 테더 초과분(cm): 손~앵커 직선 거리 - 가용 로프 길이(0 미만은 0). Wrapped 동안
	 *  매 프레임 산출된다(테더 off여도 계산). wielder 견인/지상 이탈 판정 등 게임 반응용. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetTetherOvershoot() const { return LastTetherOvershoot; }

	/**
	 * 능동 Pull(당김) 힘 설정 — Wrapped + 로프가 팽팽할 때 매 프레임 이 크기의 *상수* 힘을 감긴
	 * 대상에 인가한다(장력과 무관 → 피드백 폭주 없음). 0 = 정지. 입력 홀드 동안 켜고 떼면 끄는
	 * 용도(URopeWielderComponent의 PullAction이 이걸 호출). 캐릭터 대상은 CharacterMovement가
	 * 질량으로 나누고 지면 마찰과 경쟁하므로 수만~수십만 단위가 체감 구간이다.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetActivePull(float Force);

	/** 현재(런타임) 로프 길이(cm). 되감기/풀기로 변한다 — 초기값/상한은 RopeLength. */
	UFUNCTION(BlueprintPure, Category = "Rope")
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
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsSleeping() const { return bAsleep; }

	/** 현재 거리 LOD의 iteration 배율(1=풀 품질). 디버그/프로파일 확인용. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetSolverLODScale() const { return SolverLODScale; }

	/** 이번 프레임 이 로프가 GPU 솔버로 step됐는가(false면 CPU 폴백/솔버 off). 디버그 확인용. */
	bool IsGpuSteppedThisFrame() const { return bGpuSteppedThisFrame; }

	/** 현재 감고 있는 본 이름(Wrapped 동안 유효, 아니면 None). 이벤트 파라미터 없이도 조회 가능하게 노출. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	FName GetWrappedBoneName() const { return WrapController.State.BoneName; }

	/** 현재 감고 있는 스켈레탈 메시(Wrapped 동안 유효, 아니면 null). 대상 액터 반응은 GetOwner()로 이어간다.
	 *  내부 보관은 이제 const USceneComponent weak(정적 랩 대비 일반화) — 여기서는 스켈레탈만 반환하고,
	 *  정적 대상이면 null이다. 본문은 Cast가 필요해 .cpp에 정의(헤더 무거운 include 회피). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	USkeletalMeshComponent* GetWrappedMesh() const;

	/** 센터라인 노드 수(= NumParticles, 시뮬 초기화 후). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	int32 GetNodeCount() const { return Sim.Num(); }

	/** 센터라인 노드의 월드 위치(0=손/앵커, GetNodeCount()-1=끝). 범위 밖 인덱스는 ZeroVector.
	 *  로프 끝에 이펙트/사운드를 붙이는 등 BP 소비용 — C++은 GetCenterlinePositions()가 무복사. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetNodePosition(int32 NodeIndex) const
	{
		return Sim.Positions.IsValidIndex(NodeIndex) ? Sim.Positions[NodeIndex] : FVector::ZeroVector;
	}

	const TArray<FVector>& GetCenterlinePositions() const { return Sim.Positions; }

	//~ Events(이벤트) ----------------------------------------------------
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnWrapped OnRopeWrapped;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnCaptured OnRopeCaptured;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnReleased OnRopeReleased;

	/** 모든 페이즈 전이 알림(같은 페이즈 재설정은 제외). Wrapped/Captured/Released보다 세밀한 상태 연동(UI/SFX)용.
	 *  전이 처리 도중(SetPhase 내부)에 브로드캐스트되므로 핸들러에서 로프 상태를 바꾸는 호출(ReleaseWrap 등)은
	 *  지원하지 않는다 — 그런 반응은 OnRopeWrapped/OnRopeReleased(전이 마무리 후 발화)에 바인딩할 것. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnPhaseChanged OnRopePhaseChanged;

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

protected:
	//~ 확장 훅(서브클래스용) -------------------------------------------------
	// 전부 게임 스레드에서 프레임 단위(콜드 패스)로만 불린다 — 병렬로 도는 Solve 단계에는 훅이 없다.
	// 노드 단위 핫 루프(솔버/로직 F-클래스)는 POD·GPU 파리티 기준점이라 virtual 확장 지점이 아니다.
	// 훅을 추가할 때는 호출 스레드/페이즈/빈도를 주석에 명시하는 것을 계약의 일부로 삼는다.

	/** 페이즈 전이 직후, OnRopePhaseChanged 브로드캐스트 직전에 호출(전이당 1회, 같은 페이즈 재설정 제외). */
	virtual void OnPhaseChanged(ERopePhase OldPhase, ERopePhase NewPhase) {}

	/**
	 * wrap 대상 게이트. Flight의 접촉 후보 산출 프레임마다(후보별) + prepared preview throw 진입 시 1회
	 * 호출된다. false면 그 (Mesh, Bone) 후보는 없는 것으로 취급된다 — 팀/태그 등 게임 규칙으로 감을 수
	 * 있는 대상을 제한할 때 오버라이드. 기본 true(모두 허용). 주의: Wielder의 조준 preview 빌드는 이
	 * 게이트를 통과하지 않으므로(정적 빌더), 금지 대상이 preview에 보일 수는 있다 — throw가 거부한다.
	 */
	virtual bool CanWrapTarget(const USceneComponent* Mesh, FName Bone) const { return true; }

	//~ 이벤트 네이티브 훅: 각 델리게이트 브로드캐스트 직전에 호출(엔진 Notify 관례). C++ 서브클래스가
	//  자기 델리게이트에 바인딩하는 우회 없이 반응할 수 있다.
	virtual void NotifyCaptured(FName Bone) {}
	virtual void NotifyWrapped(FName Bone) {}
	virtual void NotifyReleased(FName Bone, ERopeReleaseReason Reason) {}

	/** Throw(AimDir) 편의 진입점이 만드는 기본 컨텍스트(throw당 1회). 조준 규약을 바꾸려면 오버라이드.
	 *  기본 구현은 FRopeThrowContext::MakeDefault(공용 조립 — 프레임 기저 규약은 그쪽 주석 참고) 위임. */
	virtual FRopeThrowContext MakeDefaultThrowContext(const FVector& AimDir) const;

	/** throw 컨텍스트 최종 해석(throw당 1회 — 실제 던지기+프리뷰 빌드가 전부 이 관문을 지난다):
	 *  프레임을 정규직교(오른손계)로 재구성(Forward 기준, Up 직교화, Right = Up×Forward 재유도 —
	 *  입력 Right 무시), 속도·원점 폴백. 에임 어시스트 등 커스텀 지점(오버라이드 시 프리뷰와 실제
	 *  던지기가 자동으로 일치). */
	virtual FRopeThrowContext ResolveThrowContext(const FRopeThrowContext& ThrowContext) const;

	/**
	 * Pull 힘 인가(Wrapped + 팽팽 + 능동 Pull 활성인 프레임마다). 기본 수신자 체인:
	 * 물리 시뮬 본 → CharacterMovement → 물리 시뮬 루트. 커스텀 무브먼트(Mover 등)/탈것/특수 대상은 오버라이드.
	 */
	virtual void ApplyPullForce(const FVector& Force, const FRopePullSample& Pull);

	// 시뮬 상태 읽기 전용 접근(서브클래스용). 변경은 공개 API(Throw·Set 계열)를 통해서만.
	const FRopeSimState& GetSimState() const { return Sim; }

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
	FRopeGuidedThrowState GuidedThrowState;	// PreviewPathLocked: cached preview path를 authoritative하게 구동

	//~ 페이즈 타이머 --------------------------------------------------------
	float ContactingElapsed = 0.0f;	// Contacting 체류 시간(WrapDecisionTime 판정)
	float FlightNoContactElapsed = 0.0f;	// Whip 종료 후 캡처 없이 Flight에 머문 시간
	float ReleaseCooldown = 0.0f;	// Releasing → Free 복귀까지 남은 시간
	float TensionOverTime = 0.0f;	// Wrapped 중 최대 장력이 TensionReleaseForce를 연속 초과한 시간

	// 이번 프레임 Pull 산출물(Wrapped 동안 매 프레임 산출). BP 조회/디버거 화살표 소스.
	// Direction은 아래 SmoothedPullDir(시간 스무딩된 방향)으로 매 프레임 덮어써서 소비자(테더/능동 Pull)가
	// 스무딩된 값을 쓰게 한다.
	FRopePullSample LastPullSample;

	// Pull 방향의 시간 스무딩 상태(EMA). ComputePull의 look-ahead 방향(공간 평균)을 프레임 간 지수이동평균해
	// 잔여 지터 + GPU 미러 지연 노이즈를 흡수한다. 영벡터 = 미초기화(wrap 시작 후 첫 유효 프레임에 측정값으로
	// 시드). ResetTransientPhaseState에서 리셋. 테더/능동 Pull이 이 방향을 공용으로 쓴다.
	FVector SmoothedPullDir = FVector::ZeroVector;

	// wielder 견인 방향(손(노드0)→로프 첫 다리)의 시간 스무딩 상태(EMA — SmoothedPullDir과 동일 상수
	// PullDirSmoothTime). 영벡터 = 미초기화(첫 유효 프레임에 시드), ResetTransientPhaseState에서 리셋.
	// 방향이 프레임마다 튀면 속도 톱업이 매번 다른 축으로 들어가 벡터가 랜덤워크로 불어난다(폭주) —
	// 방향 안정화가 1차 방어(속력 상한은 ApplyNonSimCorrection의 2차 방어).
	FVector SmoothedWielderPullDir = FVector::ZeroVector;

	// 스무딩 전 look-ahead 방향(EMA 입력 원본). 디버거가 raw vs smoothed를 나란히 그려 지터 진단에 쓴다.
	FVector LastPullDirRaw = FVector::ZeroVector;

	// Pull 조준 노드의 시간 스무딩 상태(fractional). ComputePull이 고른 정수 AimNode를 float로 EMA해 노드
	// 사이를 보간 → 방향/tether를 연속화(이산 홉 제거). <0 = 미초기화(wrap 시작 후 첫 유효 프레임에 시드).
	// ResetTransientPhaseState에서 -1로 리셋. PullAimSmoothTime이 상수.
	float SmoothedAimNodeF = -1.0f;

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

	// (ApplyPullForce — 동작 2, Pull 힘 인가 — 는 protected 확장 훅으로 이동.)

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
		TWeakObjectPtr<const USceneComponent> Mesh;
	};
	TArray<FGpuColliderAttribution> GpuCapsuleAttribution; // GPU Capsules와 평행
	TArray<FGpuColliderAttribution> GpuSdfAttribution;     // GPU SDFColliders와 평행
	TArray<FGpuColliderAttribution> GpuBoxAttribution;     // GPU Boxes와 평행(랩 가능 박스 감지 귀속)

	// GPU 감지(G3) 프레임 산출: 서브시스템이 GetLatestContacts를 귀속해 Finalize 전에 채운다.
	// bValid면 FinalizeSimFrame의 Flight 접촉 소스가 CPU 스윕 대신 이 후보들을 쓴다(GPU 경로).
	TArray<FRopeContactCandidate> GpuFlightCandidates;
	bool bGpuContactsThisFrame = false;

	//~ 초기화/유틸 ----------------------------------------------------------
	void InitRope();

	/** Sim이 비어 있으면 1회 초기화한다(OnRegister/Throw/Prepare 초입의 안전 가드). */
	void EnsureRopeInitialized();

#if WITH_GAMEPLAY_DEBUGGER
	// 디버그 캡처 대상일 때 centerline/wrapped/collider 공통 필드를 스냅샷에 채운다(FinalizeSimFrame에서 호출).
	void FillDebugSnapshot(FRopeDebugSnapshot& Snapshot) const;
#endif

	//~ Throw ----------------------------------------------------------------
	// (MakeDefaultThrowContext/ResolveThrowContext는 protected 확장 훅으로 이동.)
	void StartFreshThrow(const FRopeThrowContext& ThrowContext);

	/** GuidedThrow phase 한 프레임 진행. preview centerline으로 노드를 이동시키며 solver는 끈다. */
	void UpdateGuidedThrow(float DeltaTime);

	/** GuidedThrow 완료 시 prepared anchor를 FRopeWrapState로 변환해 바로 Wrapped로 커밋한다. */
	void FinishGuidedThrow();

	FVector ComputeThrowInheritedVelocity(const FRopeThrowContext& ThrowContext) const;

	/** WhipGuide에 넘길 설정 스냅샷을 Rope|Whip UPROPERTY들로부터 만든다. */
	FRopeWhipGuide::FConfig MakeWhipGuideConfig() const;

	/** 던지기 임펄스의 tail 가중치(FirstTailNode부터 끝까지 0→1 스무스). */
	float TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const;

	//~ Flight ---------------------------------------------------------------
	// 접촉 감지 파이프라인 자체는 FRopeFlightContactDetector(정적, UObject 비의존)로 분리됐다.
	// 여기엔 UObject 컨텍스트가 필요한 조립 코드만 남는다.

	/** 검출기에 넘길 파라미터 스냅샷(WrapConfig + 튜브 반지름 + 컴포넌트 전방). */
	// DeltaTime: 이번 프레임 dt — 상대운동 평가의 SurfaceVelocity(cm/s→cm/프레임) 환산에 쓰인다.
	FRopeFlightContactDetector::FParams MakeFlightDetectParams(float DeltaTime) const;

	/** 캡처 확정 시 Contacting 진입 상태(ContactTracker/PendingWrapSeed/타이머)를 구성한다. */
	void BuildContactingState(const TArray<FRopeContactCandidate>& Candidates);

	//~ Contacting -----------------------------------------------------------
	// 매 프레임 실제 접촉을 재수집해 트래커 dwell을 갱신한다: 지속 접촉 → Wrapping, 접촉 소실 →
	// dismiss(Flight), dwell이 임계에 못 미친 정체 → 안전망 타임아웃(Flight). 판정 기준은 총 경과가
	// 아니라 트래커 dwell(지배 본이 바뀌면 리셋)이다.
	void UpdateContacting(float DeltaTime);

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

	void CommitWrapping();

	void AbortWrapping(ERopeReleaseReason Reason);

	//~ Wrapped --------------------------------------------------------------
	/** latch/anchor 노드 InvMass=0, 나머지 1 — Wrapped 중 자유 구간만 솔버가 움직이게. */
	void ApplyWrappedMassMask(bool bResetDynamicNodeVelocity = false);

	bool ComputeTensionSlack(float& OutSlack, float& OutStraightDistance, float& OutAvailableLength) const;
};
