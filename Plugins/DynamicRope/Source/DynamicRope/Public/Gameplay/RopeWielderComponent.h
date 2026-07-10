// Copyright Epic Games, Inc. All Rights Reserved.
//
// 캐릭터가 로프를 "들고 던지게" 해 주는 게임플레이 컴포넌트. URopeComponent를 손 소켓에 붙이고,
// 던지기 입력/조준을 한 곳에 모은다. 캐릭터에 이 컴포넌트 하나만 붙이면(+ 로프 컴포넌트) 셋업 끝.
//
// 수동으로 하던 것: 로프를 Hand_r에 reparent → BP에서 F키→Throw 배선. 이 컴포넌트가 둘 다 대신한다.
//  - 소켓 자동 부착: BeginPlay에 owner의 SkeletalMesh를 찾아 로프를 HandSocketName에 attach.
//  - Throw()/Release()/ToggleThrow() BlueprintCallable. 조준 방향은 AimSource(컨트롤 회전/카메라/액터)에서 계산.
//  - 선택적 Enhanced Input 자동 바인딩: ThrowAction/ReleaseAction(+ MappingContext)을 꽂으면 BeginPlay에 바인딩.
//    안 꽂으면 Throw()를 직접 호출하면 된다.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Core/RopeTypes.h"
#include "Engine/EngineTypes.h"
#include "RopeWielderComponent.generated.h"

class URopeComponent;
class URopePreviewComponent;
struct FRopeAimRayThrowRequest;
class USkeletalMeshComponent;
class UInputAction;
class UInputMappingContext;
class UAnimMontage;

/** 던질 때 조준 방향을 어디서 가져올지. */
UENUM(BlueprintType)
enum class ERopeAimSource : uint8
{
	/** 컨트롤러 회전(보통 카메라/조준 방향). 기본. */
	ControlRotation,
	/** owner의 UCameraComponent forward(있으면). 없으면 ControlRotation 폴백. */
	CameraForward,
	/** owner 액터 forward. */
	ActorForward
};

UENUM(BlueprintType)
enum class ERopeWielderThrowMode : uint8
{
	/** 기존 던지기 방식. 입력이 들어오면 RopeComponent가 Flight로 진입하고 실제 물리/접촉 감지가 결과를 결정한다. */
	PhysicsSimulation UMETA(DisplayName = "Physics Simulation"),

	/** Preview가 성공한 경로를 권위 있는 결과로 사용한다. Preview 실패 상태에서는 던지기 입력 자체를 무시한다. */
	PreviewPathLocked UMETA(DisplayName = "Preview Path Locked")
};

UENUM(BlueprintType)
enum class ERopeWielderAimMode : uint8
{
	/** ThrowFrameMode가 만든 Forward를 그대로 사용한다. */
	FrameForward UMETA(DisplayName = "Frame Forward"),

	/** Forward 방향으로 rope 길이만큼 SDF/collider ray를 쏘고, 본에 맞으면 Origin->Hit 방향을 Forward로 사용한다. */
	AimRayHitDirection UMETA(DisplayName = "Aim Ray Hit Direction")
};

UENUM(BlueprintType)
enum class ERopeAimRayOriginMode : uint8
{
	/** 들고 있는 SkeletalMesh bounds 중심. 특정 bone 이름을 하드코딩하지 않고 몸통/골반 근처에서 시작한다. */
	AttachMeshBoundsCenter UMETA(DisplayName = "Attach Mesh Bounds Center"),

	/** 지정한 socket/bone 위치. 정확히 pelvis 같은 기준이 필요하면 이름을 지정한다. */
	AttachSocketOrBone UMETA(DisplayName = "Attach Socket Or Bone"),

	/** Owner actor 위치. Character에서는 보통 capsule 중심에 가깝다. */
	OwnerActorLocation UMETA(DisplayName = "Owner Actor Location"),

	/** Pawn view/camera 위치. 머리/눈높이 기준이 필요할 때만 사용한다. */
	ViewLocation UMETA(DisplayName = "View Location")
};

/** 던지기 입력이 실행되지 못한 사유. OnThrowRejected로 전달된다(UI 피드백/게임 반응용). */
UENUM(BlueprintType)
enum class ERopeThrowRejectReason : uint8
{
	/** CanThrow() 게이트(서브클래스 게임 규칙 — 스태미나/상태 등)가 거부. */
	Gated,
	/** PreviewPathLocked인데 유효한 prepared preview가 없어 입력을 버림. */
	NoPreparedPreview,
	/** 실행 시점(몽타주 notify 등)에 보존해 둔 prepared preview가 무효화됨. */
	PreparedInvalid,
	/** RopeComponent가 prepared preview throw를 거부함(CanWrapTarget 게이트 포함). */
	RopeRejected
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FRopeWielderOnThrown);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeWielderOnThrowRejected, ERopeThrowRejectReason, Reason);

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeWielderComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeWielderComponent();

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	//~ Setup --------------------------------------------------------------
	/** 들 로프. 비우면 BeginPlay에 owner의 URopeComponent를 자동 탐색한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	TObjectPtr<URopeComponent> Rope = nullptr;

	/** 로프를 붙일 스켈레탈 메시. 비우면 owner의 첫 USkeletalMeshComponent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	TObjectPtr<USkeletalMeshComponent> AttachMesh = nullptr;

	/** 로프를 붙일 소켓/본 이름. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	FName HandSocketName = TEXT("hand_r");

	/** BeginPlay에 로프를 소켓에 자동 부착할지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	bool bAttachOnBeginPlay = true;

	//~ Aim ----------------------------------------------------------------
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim")
	ERopeAimSource AimSource = ERopeAimSource::ControlRotation;

	/** FrameForward 또는 throw 시점의 SDF ray hit 방향 중 실제 spline 기준을 선택한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim")
	ERopeWielderAimMode AimMode = ERopeWielderAimMode::FrameForward;

	/** Aim ray 시작점을 mesh bounds 중심, attach component, socket/bone 중에서 선택한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (EditCondition = "AimMode == ERopeWielderAimMode::AimRayHitDirection"))
	ERopeAimRayOriginMode AimRayOriginMode = ERopeAimRayOriginMode::AttachMeshBoundsCenter;

	/** AttachSocketOrBone 모드에서 ray origin으로 사용할 socket 또는 bone 이름이다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (EditCondition = "AimMode == ERopeWielderAimMode::AimRayHitDirection && AimRayOriginMode == ERopeAimRayOriginMode::AttachSocketOrBone"))
	FName AimRayOriginSocketName = NAME_None;

	/** SDF ray march의 샘플 간격이다. 작을수록 얇은 팔/다리 충돌 정확도가 높아진다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (ClampMin = "0.5", Units = "cm", EditCondition = "AimMode == ERopeWielderAimMode::AimRayHitDirection"))
	float AimRaySweepStep = 2.0f;

	/** 중심선 주변을 함께 검사할 반경이다. 0이면 Rope Radius와 Contact Radius 중 큰 값을 기본 반경으로 사용한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (ClampMin = "0.0", Units = "cm", EditCondition = "AimMode == ERopeWielderAimMode::AimRayHitDirection"))
	float AimRayQueryRadius = 0.0f;

	/** 로프 길이상 현재 스윙 방향을 hit 방향으로 보간하기 시작하는 비율. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (ClampMin = "0.0", ClampMax = "0.9", EditCondition = "AimMode == ERopeWielderAimMode::AimRayHitDirection"))
	float AimRayGuideSteerStartAlpha = 0.25f;

	/** 로프 길이상 hit 방향 공간 보간이 최대가 되는 비율. Flight 시간 보간 전에는 완전히 고정되지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (ClampMin = "0.05", ClampMax = "1.0", EditCondition = "AimMode == ERopeWielderAimMode::AimRayHitDirection"))
	float AimRayGuideLockAlpha = 0.50f;

	/** SDF 검사 ray와 hit 지점/법선을 월드에 디버그 드로우한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim|Debug", meta = (EditCondition = "AimMode == ERopeWielderAimMode::AimRayHitDirection"))
	bool bDrawAimRayDebug = true;

	//~ Throw --------------------------------------------------------------
	/** 던질 때 Up/Right 기준축을 어디서 가져올지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeThrowFrameMode ThrowFrameMode = ERopeThrowFrameMode::Owner;

	/** AimDir과 기준축을 조합해 스윙 호가 놓일 평면을 고른다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeSwingPlane SwingPlane = ERopeSwingPlane::AimAndFrameUp;

	/** Wielder가 책임지는 던지기 속도. ThrowContext를 통해 RopeComponent로 전달된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float ThrowSpeed = 1500.0f;

	/** 던지기 확정 방식을 고른다. PreviewPathLocked는 매 프레임 만든 prepared preview가 있어야만 던질 수 있다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeWielderThrowMode ThrowMode = ERopeWielderThrowMode::PhysicsSimulation;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "ThrowFrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameForward = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "ThrowFrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameUp = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "ThrowFrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameRight = FVector::RightVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "SwingPlane == ERopeSwingPlane::CustomNormal"))
	FVector CustomSwingPlaneNormal = FVector::RightVector;

	//~ Preview ------------------------------------------------------------
	/** 비어 있으면 owner에서 찾는다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Preview", meta = (UseComponentPicker, AllowedClasses = "/Script/DynamicRope.RopePreviewComponent,/Script/DynamicRope.RopeArcPreviewComponent", DisplayName = "Preview Component"))
	FComponentReference PreviewComponentReference;

	UPROPERTY(Transient)
	TObjectPtr<URopePreviewComponent> PreviewComponent = nullptr;

	/** Free/Releasing 상태에서만 preview를 표시한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	bool bPreviewOnlyWhenIdle = true;

	/** Preview 충돌 검사 갱신 주기. 0이면 매 프레임 검사하므로 SDF 대상이 많을 때는 매우 비싸다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "0.0", Units = "s", DisplayName = "Preview Update Interval"))
	float PreviewUpdateInterval = 0.1f;

	/** PreviewPathLocked가 Wrapped로 확정된 뒤에도 preview path를 잠깐 남길 시간. 0이면 Wrapped 진입 시 즉시 지운다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "0.0", Units = "s", DisplayName = "Locked Wrapped Preview Hold Time"))
	float LockedWrappedPreviewHoldTime = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Debug")
	bool bLogPreviewBuildAttempts = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Preview")
	bool bLastPreviewBlocked = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Preview")
	FVector LastPreviewHitPoint = FVector::ZeroVector;

	//~ Input(선택) — 비우면 무시, Throw()를 직접 호출하면 된다 ------------
	/** Action/MappingContext가 설정돼 있으면 BeginPlay에 자동 바인딩할지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	bool bAutoBindInput = true;

	/** 플레이어에 추가할 Input Mapping Context(있으면). Action이 트리거되려면 활성 IMC에 들어 있어야 한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputMappingContext> MappingContext = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input", meta = (EditCondition = "MappingContext != nullptr"))
	int32 MappingPriority = 0;

	/** 던지기 액션. Started에 Throw(). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ThrowAction = nullptr;

	/** 해제 액션. Started에 Release(). 비우면 ThrowAction을 ToggleThrow로 쓸 수도 있다(bThrowActionToggles). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReleaseAction = nullptr;

	/** ReleaseAction이 비었을 때, ThrowAction을 던지기/해제 토글로 사용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	bool bThrowActionToggles = true;

	/** 능동 Pull 액션(홀드). 누르는 동안 PullForce로 감긴 대상을 끌어당기고 떼면 멈춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> PullAction = nullptr;

	/**
	 * 능동 Pull의 힘(상수 — 장력과 무관해 피드백 폭주 없음). Wrapped + 로프가 팽팽할 때만 인가된다.
	 * 캐릭터 대상은 CharacterMovement가 질량으로 나누고 지면 마찰과 경쟁하므로 수만~수십만이 체감 구간.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input", meta = (ClampMin = "0.0"))
	float PullForce = 100000.0f;

	/** 되감기 액션(홀드). 누르는 동안 ReelSpeed로 로프를 감고(짧아짐) 떼면 멈춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReelInAction = nullptr;

	/** 풀기 액션(홀드). 누르는 동안 ReelSpeed로 로프를 풀고(초기 길이까지) 떼면 멈춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReelOutAction = nullptr;

	/** 되감기/풀기 속도(cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input", meta = (ClampMin = "0.0"))
	float ReelSpeed = 150.0f;

	//~ Tension(장력 — wielder 몫 테더와 조합) ------------------------------
	// WrapConfig.TetherTargetShare < 1이면 로프가 wielder를 앵커 쪽으로 끌어당긴다(수렴형 테더 분배).
	// 이 섹션은 그 견인의 캐릭터 이동 정책: 물리(플러그인 코어)가 아니라 게임 반응이라 wielder에 둔다.

	/**
	 * 로프가 위로 당기는데 지상(walking 계열)이면 발이 땅에 붙어 상승을 막는다 — 견인의 상향 성분이
	 * 충분하고 초과분이 쌓여 있으면 자동으로 Falling 전환해 몸이 뜨게 한다(착지 복귀는 엔진이 처리).
	 * 되감기(ReelIn)와 조합하면 입체기동식 "감으면 끌려 올라감"이 된다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tension")
	bool bAutoGroundExitOnUpwardPull = true;

	/** 상향 판정 임계: 견인 방향(손→앵커, 단위 벡터)의 Z 성분이 이 값 이상일 때만 지상 이탈. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tension", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bAutoGroundExitOnUpwardPull"))
	float GroundExitUpDot = 0.35f;

	/** 지상 이탈에 필요한 최소 테더 초과분(cm). 경계 지터로 모드가 퍼덕이는 것을 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tension", meta = (ClampMin = "0.0", Units = "cm", EditCondition = "bAutoGroundExitOnUpwardPull"))
	float GroundExitMinOvershoot = 10.0f;

	/**
	 * 스윙 중(Wrapped + 공중 + wielder 몫 테더 활성) 에어컨트롤을 SwingAirControl로 올려 조향을
	 * 살린다. CharacterMovement 기본 AirControl(0.05)로는 스윙 방향을 거의 못 바꾼다. 스윙이 끝나면
	 * (착지/release) 저장해 둔 원래 값으로 복원한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tension")
	bool bBoostAirControlWhileSwinging = true;

	/** 스윙 중 적용할 AirControl(0~1). 0.35~1 권장 — 1이면 공중에서 지상급 조향. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tension", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bBoostAirControlWhileSwinging"))
	float SwingAirControl = 1.0f;

	//~ Animation(선택) ----------------------------------------------------
	/**
	 * 설정하면 Throw()가 즉시 던지지 않고 이 몽타주를 재생한다. 실제 로프 던지기는 몽타주 안에 배치한
	 * UAnimNotify_RopeThrow가 ThrowNow()를 호출해 일어난다(던지는 모션의 손 떼는 순간에 맞춤).
	 * 비우면 Throw()가 즉시 던진다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation")
	TObjectPtr<UAnimMontage> ThrowMontage = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation", meta = (ClampMin = "0.1"))
	float ThrowMontagePlayRate = 1.0f;

	//~ API ----------------------------------------------------------------
	/**
	 * 던지기 시작. ThrowMontage가 설정돼 있으면 몽타주를 재생(실제 던지기는 몽타주의 UAnimNotify_RopeThrow가
	 * ThrowNow() 호출), 없으면 즉시 ThrowNow(). 입력/게임플레이가 호출하는 진입점.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw();

	/** 실제 로프 던지기를 *지금* 실행한다. 방향은 ThrowFrameMode의 Forward를 사용한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowNow();

	/** 현재 Wielder/Rope 설정으로 throw 순간의 origin/frame/속도 context를 만든다(던지기당 1회, GT).
	 *  유효한 AimDir이 들어오면 설정된 frame forward를 대체한다. 조립 규칙을 바꾸려면 오버라이드(확장 훅). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	virtual FRopeThrowContext BuildThrowContext(const FVector& AimDir) const;

	/** Legacy API. AimDir은 더 이상 주 방향이 아니며, 실제 방향은 ThrowFrameMode의 Forward를 사용한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowInDirection(const FVector& AimDir);

	/** ThrowMontage를 owner 메시의 AnimInstance에서 재생한다(설정돼 있을 때). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void PlayThrowMontage();

	/** 현재 wrap을 해제한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Release();

	/** 능동 Pull 시작(PullForce로 견인 — Wrapped + 팽팽할 때만 실제 인가). 입력 홀드/게임플레이용. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartPull();

	/** 능동 Pull 정지. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StopPull();

	/** 로프 절단(ERopeReleaseReason::Cut으로 강제 해제 — 게임플레이 절단 이벤트용 패스스루). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Cut();

	/** 되감기 시작(로프가 ReelSpeed로 짧아짐). 입력 홀드/게임플레이용. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartReelIn();

	/** 풀기 시작(로프가 ReelSpeed로 초기 길이까지 길어짐). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartReelOut();

	/** 되감기/풀기 정지. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StopReel();

	/** wrap/contact 중이면 Release, 아니면 Throw. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ToggleThrow();

	/** 들고 있는 로프(없으면 null). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	URopeComponent* GetRope() const { return Rope; }

	/** 현재 AimSource 기준 조준 방향(정규화). 던지기/preview 틱에서 호출된다(GT, 콜드).
	 *  락온·에임 어시스트·AI 조준 등 커스텀 조준은 이걸 오버라이드(확장 훅). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	virtual FVector GetAimDirection() const;

	/** 입력을 수동으로 바인딩한다. 자동 바인딩이 타이밍상 실패하면(InputComponent 미준비) Pawn의
	 *  SetupPlayerInputComponent에서 호출하라. 이미 바인딩됐으면 무시. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Input")
	void BindInput();

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void SetThrowPreviewEnabled(bool bEnabled);

	/** 던지기 preview가 현재 켜져 있는가(런타임 상태 — BeginPlay 자동 결정 + SetThrowPreviewEnabled 토글). */
	UFUNCTION(BlueprintPure, Category = "Rope|Preview")
	bool IsThrowPreviewEnabled() const { return bShowThrowPreview; }

	//~ Events(이벤트) ------------------------------------------------------
	/** 던지기가 실제로 실행된 직후(즉시/몽타주 notify 경로 모두). */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeWielderOnThrown OnThrown;

	/** 던지기 입력이 실행되지 못했을 때(사유 포함). PreviewPathLocked의 조용한 입력 버림도 여기로 알린다. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeWielderOnThrowRejected OnThrowRejected;

protected:
	//~ 확장 훅(서브클래스용) ------------------------------------------------
	// URopeComponent와 같은 원칙: 전부 게임 스레드·프레임 단위(콜드 패스)에서만 불린다.
	// 훅을 추가할 때는 호출 시점/빈도를 주석에 명시하는 것을 계약의 일부로 삼는다.
	// (public의 GetAimDirection/BuildThrowContext도 virtual 확장 훅이다.)

	/**
	 * 던지기 입력 게이트: Throw() 진입 시 1회 호출. false면 입력을 버리고 Gated 사유로 알린다.
	 * 스태미나/상태 등 게임 규칙으로 던지기를 제한할 때 오버라이드. 기본 true.
	 * 몽타주 경로의 ThrowNow()(AnimNotify 호출)는 이미 게이트를 통과한 확정 던지기라 재검사하지 않는다.
	 */
	virtual bool CanThrow() const { return true; }

	//~ 이벤트 네이티브 훅: 각 델리게이트 브로드캐스트 직전에 호출(엔진 Notify 관례).
	virtual void NotifyThrown() {}
	virtual void NotifyThrowRejected(ERopeThrowRejectReason Reason) {}

private:
	void ResolveRefs();        // Rope/AttachMesh 해석(미설정 시 owner에서 탐색).
	void AttachRopeToSocket(); // Rope를 AttachMesh의 HandSocketName에 부착.
	void AddMappingContext();  // MappingContext를 로컬 플레이어 Enhanced Input 서브시스템에 추가.

	void ResolvePreviewComponent(bool bAllowAutoCreate);
	// wielder가 테더 몫을 실제로 받는 상태인가(Wrapped + TetherResponse>0 + TargetShare<1 + 셀프랩 아님).
	bool IsWielderTetherActive() const;
	// wielder 몫 테더가 위로 당길 때 walking이면 Falling으로 전환한다(매 틱, GT — 위 Tension 섹션 참고).
	void UpdateGroundExit();
	// 스윙 판정에 따라 AirControl을 부스트/복원한다(매 틱, GT).
	void UpdateSwingAirControl();
	void UpdateThrowPreview();
	// 멀리 있는 target SDF도 수집되도록 ray 구간을 collider query bounds에 포함한다.
	void UpdateAimRayColliderQueryBounds();
	void ClearThrowPreview();
	// collider/SDF side effect 없이 origin/frame/속도만 계산한다.
	FRopeThrowContext BuildBaseThrowContext(const FVector& AimDir) const;
	// 실제 ray를 새로 검사하고 throw 순간에 고정할 context를 구성한다.
	FRopeThrowContext BuildThrowContextInternal(const FVector& AimDir) const;
	// 입력 순간의 base frame과 ray 설정을 값 타입 요청으로 캡처한다.
	FRopeAimRayThrowRequest BuildAimRayThrowRequest(const FVector& AimDir) const;
	// 선택한 origin 모드를 월드 위치로 해석한다.
	FVector GetAimRayOrigin() const;
	// 현재 시뮬레이션 길이와 설정 길이 중 큰 값으로 ray 길이를 계산한다.
	float GetAimRayLength() const;
	void LogPreviewBuildResult(bool bSucceeded, const FString& Reason);
	// Aim hit prepared spline을 wielder owner-local 좌표로 저장해 손 소켓 애니메이션에서 분리한다.
	void StoreAimGuideFrameIfNeeded(FRopePreparedThrowPreview& Prepared) const;
	// 저장된 owner-local prepared spline을 현재 owner transform 기준으로 렌더한다.
	FRopeWrapPreviewData ResolvePreparedPreviewForDisplay(const FRopePreparedThrowPreview& Prepared) const;
	bool ShouldHoldPreparedPreview();
	// 현재 Rope phase에서 새 preview path를 계산해도 되는지 판단한다. false면 비싼 build 경로에 들어가지 않는다.
	bool ShouldUpdateThrowPreviewForPhase(ERopePhase Phase) const;
	// PreviewPathLocked가 이미 확정한 path를 GuidedThrow/Wrapped 동안 렌더 유지한다. 처리했으면 true를 반환한다.
	bool UpdateHeldPreparedPreviewForPhase(ERopePhase Phase);

	void OnThrowInput();
	void OnAimRayThrowResolved();
	void OnReleaseInput();
	void OnPullInputStarted();
	void OnPullInputCompleted();
	void OnReelInStarted();
	void OnReelOutStarted();
	void OnReelCompleted();

	bool bInputBound = false;
	// AirControl 부스트 원복용 저장 상태(스윙 진입 시 저장, 종료/EndPlay 시 복원).
	bool bAirControlBoosted = false;
	float SavedAirControl = 0.0f;
	// preview 켜짐 상태(디자이너 설정 아님 — BeginPlay가 PreviewComponent 유무로 자동 결정하고
	// SetThrowPreviewEnabled가 토글). 조회는 IsThrowPreviewEnabled().
	bool bShowThrowPreview = false;
	float PreviewUpdateCooldown = 0.0f;
	bool bLastPreviewBuildSucceeded = false;
	bool bHasLastPreviewBuildResult = false;
	FString LastPreviewBuildReason;

	// 마지막 preview tick에서 성공한 prepared 결과. PreviewPathLocked 모드에서 "지금 던질 수 있는가"를 판정한다.
	FRopePreparedThrowPreview LastPreparedPreview;

	// 몽타주를 쓰는 경우 입력 시점의 preview를 고정해 두고, AnimNotify_RopeThrow가 ThrowNow를 부를 때 소비한다.
	FRopePreparedThrowPreview PendingPreparedThrow;

	// PreviewPathLocked 실행 중(GuidedThrow 포함) 화면에 유지할 확정 path.
	FRopeWrapPreviewData HeldPreparedPreview;
	// Wrapped 후 preview path를 잠깐 남길 때 사용하는 만료 시각. LockedWrappedPreviewHoldTime이 0이면 즉시 만료된다.
	float HeldPreviewExpireTimeSeconds = 0.0f;
	// Wrapped 진입 순간을 감지하기 위한 마지막 preview 처리 phase.
	ERopePhase LastPreviewPhase = ERopePhase::Free;
};
