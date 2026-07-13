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

// NOTE: 종전의 ERopeWielderThrowMode(PhysicsSimulation/PreviewPathLocked)와
// ERopeWielderAimMode(FrameForward/AimRayHitDirection)는 제거됐다(2026-07-13 회의 결정 F).
// 조준의 '의미'(aim ray 사용)와 던지기 확정 방식(preview 구속)은 이제 로프의
// URopeComponent::ResolveMode에서 유도된다 — UsesAimRay()/UsesLockedPreview() 참조.
// ① FullSimulation = 자유 조준 + 물리 결과, ② AssistedJudged = aim ray + 물리/판정,
// ③ GuaranteedWrap = aim ray + preview 구속. (종전의 금지 조합 PreviewPathLocked+FrameForward는
// 표현 자체가 불가능해졌다.)

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
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeWielderOnAimTargetChanged, USceneComponent*, Mesh, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FRopeWielderOnAimTargetLost);

class URopeAimWidget;

/**
 * 조준 HUD용 프레임 샘플: aim ray가 지금 어떤 감김 가능 대상을 겨누고 있는가.
 * Aim ray 모드일 때 wielder가 틱마다 캐시한다(스윕 레이 1회/프레임 — preview 경로와 같은 비용 등급).
 * 소비자(URopeAimWidget/BP)는 읽기 전용 — Mesh는 표시/식별 용도로만 쓸 것.
 */
USTRUCT(BlueprintType)
struct FRopeAimHudSample
{
	GENERATED_BODY()

	/** 이번 프레임 aim ray가 감김 가능 본에 걸려 있는가. false면 (bBlocked가 아닌 한) 나머지 필드는 무의미. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	bool bHasTarget = false;

	/**
	 * 이번 프레임 aim ray가 뭔가에 걸렸지만 wrap은 불가능한가(월드 정적/본 없음/CanWrapTarget 거부).
	 * bHasTarget과 배타 — 감길 대상이 있으면 그쪽이 우선. true면 HUD를 빨갛게 표시하고, 이때
	 * TargetWorldPos/HitWorldPos/TargetRadius/Distance는 걸린 지점 기준으로 채워진다(Bone/Mesh는 없을 수 있음).
	 */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	bool bBlocked = false;

	/** 겨누고 있는 본(가상 본 포함). */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FName Bone = NAME_None;

	/** 본을 소유한 대상 컴포넌트(스켈레탈/정적 랩 대상). */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	TObjectPtr<USceneComponent> Mesh = nullptr;

	/** 강조 링 중심 — 본 바인딩 위치(ResolveBindingWorld). */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector TargetWorldPos = FVector::ZeroVector;

	/** ray가 실제로 맞은 월드 지점(이펙트 스폰 등 정밀 위치가 필요할 때). */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector HitWorldPos = FVector::ZeroVector;

	/** 대상 콜라이더의 월드 반경 근사(bounds 반대각) — 링 크기 산정용. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	float TargetRadius = 0.0f;

	/** ray origin에서 hit까지 거리. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	float Distance = 0.0f;

	/** 이번 프레임 실제 aim ray 시작점. AimRayOriginMode가 선택한 위치와 동일하다. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector RayOrigin = FVector::ZeroVector;

	/** 실제 aim ray 방향(정규화). */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector RayDirection = FVector::ForwardVector;

	/** 실제 aim ray 길이. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	float RayLength = 0.0f;

	/** 조준원 투영 위치. hit가 있으면 실제 hit, 없으면 aim ray 끝점이다. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector AimWorldPos = FVector::ZeroVector;
};

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
	// Wielder는 조준의 '출처'(카메라/소켓/원점)만 소유한다. aim ray를 쓸지(조준의 '의미')는
	// 로프의 ResolveMode가 결정한다 — UsesAimRay() 참조. 아래 AimRay* 세부는 T3(고급) 튜닝.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim")
	ERopeAimSource AimSource = ERopeAimSource::ControlRotation;

	/** Aim ray 시작점을 mesh bounds 중심, attach component, socket/bone 중에서 선택한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim")
	ERopeAimRayOriginMode AimRayOriginMode = ERopeAimRayOriginMode::AttachMeshBoundsCenter;

	/** AttachSocketOrBone 모드에서 ray origin으로 사용할 socket 또는 bone 이름이다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (EditCondition = "AimRayOriginMode == ERopeAimRayOriginMode::AttachSocketOrBone"))
	FName AimRayOriginSocketName = NAME_None;

	/** SDF ray march의 샘플 간격이다. 작을수록 얇은 팔/다리 충돌 정확도가 높아진다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Aim", meta = (ClampMin = "0.5", Units = "cm"))
	float AimRaySweepStep = 2.0f;

	/** 중심선 주변을 함께 검사할 반경이다. 0이면 Rope Radius와 Contact Radius 중 큰 값을 기본 반경으로 사용한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Aim", meta = (ClampMin = "0.0", Units = "cm"))
	float AimRayQueryRadius = 0.0f;

	/** 로프 길이상 현재 스윙 방향을 hit 방향으로 보간하기 시작하는 비율. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Aim", meta = (ClampMin = "0.0", ClampMax = "0.9"))
	float AimRayGuideSteerStartAlpha = 0.25f;

	/** 로프 길이상 hit 방향 공간 보간이 최대가 되는 비율. Flight 시간 보간 전에는 완전히 고정되지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Rope|Aim", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float AimRayGuideLockAlpha = 0.50f;

	/** SDF 검사 ray와 hit 지점/법선을 월드에 디버그 드로우한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim|Debug")
	bool bDrawAimRayDebug = true;

	/**
	 * 데모 조준 HUD(십자선 + 감김 가능 본 강조 링) 위젯을 로컬 플레이어 뷰포트에 자동으로 띄울지.
	 * 위젯 클래스는 Project Settings > Dynamic Rope > AimHudWidgetClass가 정한다(기본 = C++ URopeAimWidget,
	 * WBP 서브클래스로 리스타일 가능). Aim ray 모드(= 로프 ResolveMode가 ①이 아닐 때)에서만 의미가 있다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim|HUD")
	bool bShowAimHudWidget = true;

	/**
	 * 이 wielder의 조준이 aim ray(대상 잠금)를 쓰는가 — 로프 ResolveMode에서 유도된다
	 * (②AssistedJudged/③GuaranteedWrap = true, ①FullSimulation·로프 없음 = false).
	 * 종전 AimMode 스위치의 대체(2026-07-13 회의 결정 F: 조준의 '의미'는 로프 모드가 소유).
	 */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim")
	bool UsesAimRay() const;

	/**
	 * 이 wielder의 던지기가 preview 구속(PreviewPathLocked 흐름)인가 — 로프 ResolveMode에서
	 * 유도된다(③GuaranteedWrap = true). preview 생성 실패 시 던지기 입력은 거부된다
	 * (OnThrowRejected·NoPreparedPreview). 종전 ThrowMode 스위치의 대체.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope|Throw")
	bool UsesLockedPreview() const;

	//~ Throw --------------------------------------------------------------
	// NOTE: 종전의 던지기 파라미터 사본 7종(ThrowFrameMode/SwingPlane/ThrowSpeed/CustomFrame*/
	// CustomSwingPlaneNormal)은 제거됐다(2026-07-13 표면 감사 A-1). 단일 소스는 로프의
	// URopeComponent::ThrowParams(FRopeThrowParams)다 — Wielder 경유 던지기에서 로프 설정이
	// 무시되던 이중을 해소. Wielder는 조준 방향과 손 소켓 원점 등 "출처"만 컨텍스트에 얹는다.

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

	// NOTE: 힘/속도 수치(PullForce/ReelSpeed)는 로프로 이사했다(2026-07-13 표면 감사 A-2 —
	// 물리 수치는 로프 도메인): Pull 힘 = HoldConfig.PullForce, 릴 속도 = URopeComponent::ReelSpeed.
	// 이 섹션에는 입력 바인딩만 남는다.

	/** 능동 Pull 액션(홀드). 누르는 동안 로프의 HoldConfig.PullForce로 끌어당기고 떼면 멈춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> PullAction = nullptr;

	/** 되감기 액션(홀드). 누르는 동안 로프의 ReelSpeed로 로프를 감고(짧아짐) 떼면 멈춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReelInAction = nullptr;

	/** 풀기 액션(홀드). 누르는 동안 로프의 ReelSpeed로 로프를 풀고(초기 길이까지) 떼면 멈춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReelOutAction = nullptr;

	//~ Tension(장력 — wielder 몫 테더와 조합) ------------------------------
	// HoldConfig.TetherTargetShare < 1이면 로프가 wielder를 앵커 쪽으로 끌어당긴다(수렴형 테더 분배).
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

	/** 실제 로프 던지기를 *지금* 실행한다. 방향은 로프 ThrowParams.FrameMode의 Forward를 사용한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowNow();

	/** 현재 Wielder/Rope 설정으로 throw 순간의 origin/frame/속도 context를 만든다(던지기당 1회, GT).
	 *  유효한 AimDir이 들어오면 설정된 frame forward를 대체한다. 조립 규칙을 바꾸려면 오버라이드(확장 훅). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	virtual FRopeThrowContext BuildThrowContext(const FVector& AimDir) const;

	/** Legacy API. AimDir은 더 이상 주 방향이 아니며, 실제 방향은 로프 ThrowParams.FrameMode의 Forward를 사용한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowInDirection(const FVector& AimDir);

	/** ThrowMontage를 owner 메시의 AnimInstance에서 재생한다(설정돼 있을 때). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void PlayThrowMontage();

	/** 현재 wrap을 해제한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Release();

	/** 능동 Pull 시작(로프 HoldConfig.PullForce로 견인 — Wrapped + 팽팽할 때만 실제 인가). 입력 홀드/게임플레이용. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartPull();

	/** 능동 Pull 정지. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StopPull();

	/** 로프 절단(ERopeReleaseReason::Cut으로 강제 해제 — 게임플레이 절단 이벤트용 패스스루). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Cut();

	/** 되감기 시작(로프의 ReelSpeed로 짧아짐). 입력 홀드/게임플레이용. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartReelIn();

	/** 풀기 시작(로프의 ReelSpeed로 초기 길이까지 길어짐). */
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

	/** aim ray가 새 대상(Mesh, Bone)에 걸린 순간(진입/전환). HUD 연출·사운드 트리거용. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Aim HUD")
	FRopeWielderOnAimTargetChanged OnAimTargetChanged;

	/** aim ray가 대상을 잃은 순간. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Aim HUD")
	FRopeWielderOnAimTargetLost OnAimTargetLost;

	/** 이번 프레임 조준 HUD 샘플(aim ray 모드에서 틱마다 갱신 — 그 외 모드에서는 빈 샘플). */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	const FRopeAimHudSample& GetAimHudSample() const { return AimHudSample; }

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
	/** 이번 프레임 조준 HUD 샘플(Tick에서 UpdateAimHudSample이 갱신). */
	FRopeAimHudSample AimHudSample;

	/** 자동 생성한 조준 HUD 위젯(로컬 플레이어 전용). bShowAimHudWidget/모드 변경에 따라 생성·제거. */
	UPROPERTY(Transient)
	TObjectPtr<URopeAimWidget> AimHudWidget = nullptr;

	/**
	 * aim ray 스윕 1회로 조준 HUD 샘플을 갱신하고, 대상 (Mesh, Bone) 변화 시
	 * OnAimTargetChanged/OnAimTargetLost를 발화한다(Tick, aim ray 모드 전용).
	 */
	void UpdateAimHudSample();

	/** 조준 HUD 위젯 생성/제거(lazy — 로컬 PlayerController가 준비된 뒤 Tick에서). */
	void UpdateAimHudWidget();

	/** Rope/AttachMesh 해석(미설정 시 owner에서 탐색). */
	void ResolveRefs();

	/** Rope를 AttachMesh의 HandSocketName에 부착. */
	void AttachRopeToSocket();

	/** MappingContext를 로컬 플레이어 Enhanced Input 서브시스템에 추가. */
	void AddMappingContext();

	void ResolvePreviewComponent(bool bAllowAutoCreate);

	/** wielder가 테더 몫을 실제로 받는 상태인가(Wrapped + TetherResponse>0 + TargetShare<1 + 셀프랩 아님). */
	bool IsWielderTetherActive() const;

	/** wielder 몫 테더가 위로 당길 때 walking이면 Falling으로 전환한다(매 틱, GT — 위 Tension 섹션 참고). */
	void UpdateGroundExit();

	/** 스윙 판정에 따라 AirControl을 부스트/복원한다(매 틱, GT). */
	void UpdateSwingAirControl();

	void UpdateThrowPreview();

	/** 멀리 있는 target SDF도 수집되도록 ray 구간을 collider query bounds에 포함한다. */
	void UpdateAimRayColliderQueryBounds();

	void ClearThrowPreview();

	/** collider/SDF side effect 없이 origin/frame/속도만 계산한다. */
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
