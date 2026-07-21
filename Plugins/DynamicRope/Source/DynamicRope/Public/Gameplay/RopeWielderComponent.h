// Copyright Epic Games, Inc. All Rights Reserved.
//
// 캐릭터가 로프를 "들고 던지게" 해 주는 게임플레이 컴포넌트. URopeComponent를 손 소켓에 붙이고,
// 던지기 입력/조준을 한 곳에 모은다. 캐릭터에 이 컴포넌트 하나만 붙이면(+ 로프 컴포넌트) 셋업 끝.
//
// 수동으로 하던 것: 로프를 Hand_r에 reparent → BP에서 F키→Throw 배선. 이 컴포넌트가 둘 다 대신한다.
//  - 소켓 자동 부착: BeginPlay에 owner의 SkeletalMesh를 찾아 로프를 HandSocketName에 attach.
//  - Throw()/Release()/ToggleThrow() BlueprintCallable. 조준 방향은 로프 ThrowParams.FrameMode에서 계산.
//  - 선택적 Enhanced Input 자동 바인딩: ThrowAction/ReleaseAction(+ MappingContext)을 꽂으면 BeginPlay에 바인딩.
//    안 꽂으면 Throw()를 직접 호출하면 된다.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Core/RopeTypes.h"
#include "Engine/EngineTypes.h"
#include "RopeWielderComponent.generated.h"

class URopeComponent;
class URopePreset;
class URopePreviewComponent;
struct FRopeAimRayThrowRequest;
class USkeletalMeshComponent;
class UInputAction;
class UInputMappingContext;
class UEnhancedInputLocalPlayerSubsystem;
class UAnimMontage;
class UMaterialInstanceDynamic;

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

	/** Pawn 눈높이(GetPawnViewLocation). 단, 로프 ThrowParams.FrameMode가 OwnerCamera면 카메라 위치를
	 *  쓴다 — 방향과 원점이 같은 기준을 보게 하기 위해서다. 머리/눈높이 기준이 필요할 때만 사용한다. */
	ViewLocation UMETA(DisplayName = "View Location")
};

/** 던지기 입력이 실행되지 못한 사유. OnThrowRejected로 전달된다(UI 피드백/게임 반응용). */
// NOTE: 값 번호를 못 박는다 — 1/2는 삭제된 NoPreparedPreview/PreparedInvalid의 **영구 결번**이다.
// 그냥 지우고 뒤를 당기면 이미 저장된 BP switch 핀이 조용히 다른 사유로 재매핑된다(에러 없이 오동작).
// 두 사유가 사라진 이유: 2026-07-14 재정의로 prepared preview 없는 던지기는 거부가 아니라 레이 끝점
// 아치 폴백이 됐다 — 발화 지점이 코드에서 사라졌다.
UENUM(BlueprintType)
enum class ERopeThrowRejectReason : uint8
{
	/** CanThrow() 게이트(서브클래스 게임 규칙 — 스태미나/상태 등)가 거부. */
	Gated = 0,
	/** RopeComponent가 prepared preview throw를 거부함(CanWrapTarget 게이트 포함). */
	RopeRejected = 3,
	/** ③을 Reel(장전) 밖에서 던지려 함 — EnterReel()이 먼저다. */
	NotInReel = 4
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FRopeWielderOnThrown);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeWielderOnThrowRejected, ERopeThrowRejectReason, Reason);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeWielderOnAimTargetChanged, USceneComponent*, Mesh, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FRopeWielderOnAimTargetLost);
/** Pull 장전 토글이 바뀐 순간(① 해제 ↔ ② 장전). */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeWielderOnPullArmedChanged, bool, bArmed);
/** Pull 발동 래치가 바뀐 순간(② 대기 ↔ ③ 발동). Tension은 발동 시점 관측 장력(해제 시 0). */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeWielderOnPullEngagedChanged, bool, bEngaged, float, Tension);

class URopeAimWidget;
class URopePullGaugeWidget;

/**
 * 조준 HUD용 프레임 샘플: aim ray가 지금 어떤 감김 가능 대상을 겨누고 있는가.
 * Aim ray 모드일 때 wielder가 요청을 등록하고, subsystem의 정상 collider gather 직후 해석한 결과를
 * 다음 틱에 캐시한다(최대 1프레임 지연 — HUD 때문에 provider를 추가 수집하지 않는다).
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

	/** 이번 프레임 스윕에 **실제로 쓰인** 질의 반경. AimRayQueryRadius가 0(기본)이면 로프/접촉 폴백 반경이
	 *  들어가므로 설정값과 다를 수 있다 — 조준이 검사하는 실제 두께다. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	float QueryRadius = 0.0f;

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
	// Wielder는 조준 원점 세부만 소유한다. aim ray 방향은 Rope ThrowParams.FrameMode(Owner/OwnerCamera/Socket 등)를
	// 따른다. aim ray를 쓸지(조준의 '의미')는 로프의 ResolveMode가 결정한다 — UsesAimRay() 참조.
	// NOTE: 종전의 ERopeAimSource/AimSource(ControlRotation/CameraForward/ActorForward)와 조회 헬퍼
	// GetAimDirection()은 제거됐다 — FrameMode가 방향의 단일 소스가 된 뒤로 던지기/조준 방향에 아무 영향이
	// 없는 죽은 배선이었다(설정해도 결과가 안 바뀜). 유일하게 살아 있던 소비처인 ViewLocation 원점의 카메라
	// 폴백은 FrameMode == OwnerCamera가 직접 판정하도록 옮겼다 — 원점이 방향과 같은 기준을 본다.

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

	// NOTE: aim ray 시각화 플래그(bDrawAimRayDebug)는 사라졌다 — 시각화는 Gameplay Debugger의 Rope
	// 카테고리([J]aim)가 아래 GetAimHudSample()을 읽어 그린다. 디버그 진입점은 그 카테고리 하나다.

	/**
	 * 데모 조준 HUD(십자선 + 감김 가능 본 강조 링) 위젯을 로컬 플레이어 뷰포트에 자동으로 띄울지.
	 * 위젯 클래스는 Project Settings > Dynamic Rope > AimHudWidgetClass가 정한다(기본 = C++ URopeAimWidget,
	 * WBP 서브클래스로 리스타일 가능). Aim ray 모드(= 로프 ResolveMode가 ①이 아닐 때)에서만 의미가 있다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim|HUD")
	bool bShowAimHudWidget = true;

	/**
	 * Pull 장전/발동 게이지를 로컬 플레이어 뷰포트에 자동으로 띄울지. 위젯 클래스는
	 * Project Settings > Dynamic Rope > PullGaugeWidgetClass가 정한다(기본 = C++ URopePullGaugeWidget).
	 * 게이지는 장전 상태에서만 그려지므로(해제 시 빈 화면) 평상시 화면을 가리지 않는다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull|Feedback")
	bool bShowPullGaugeWidget = true;

	/** 조준에 aim ray(대상 잠금)를 쓰는가 — 로프 ResolveMode에서 유도된다(②③ = true, ① = false).
	 *  **모드 단위** 판정이라 phase와 무관하다. 지금 조준이 살아 있는지는 IsAimActive(). */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim")
	bool UsesAimRay() const;

	/** 지금 조준이 의미가 있는가 = UsesAimRay() && 로프가 던질 수 있는 phase(CanThrowNow).
	 *  UsesAimRay()의 **프레임 단위 쌍둥이** — ③은 Reel에서만 true, ①②는 UsesAimRay()와 동치.
	 *  조준 HUD·위젯·디버거 시각화가 전부 이 하나를 게이트로 쓴다. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim")
	bool IsAimActive() const;

	/** 던지기가 preview 구속인가 — 로프 ResolveMode에서 유도된다(③ = true).
	 *  ③은 조준이 잡히면 그 경로로 무조건 꽂고, 안 잡히면 레이 끝점 아치로 폴백한다(거부 아님). */
	UFUNCTION(BlueprintPure, Category = "Rope|Throw")
	bool UsesLockedPreview() const;

	//~ Throw --------------------------------------------------------------
	// NOTE: 종전의 던지기 파라미터 사본 7종(ThrowFrameMode/SwingPlane/ThrowSpeed/CustomFrame*/
	// CustomSwingPlaneNormal)은 제거됐다(2026-07-13 표면 감사 A-1). 단일 소스는 로프의
	// URopeComponent::ThrowParams(FRopeThrowParams)다 — Wielder 경유 던지기에서 로프 설정이
	// 무시되던 이중을 해소. Wielder는 조준 방향과 손 소켓 원점 등 "출처"만 컨텍스트에 얹는다.

	//~ Preview(GuaranteedWrap 모드 전용) ----------------------------------
	// preview는 GuaranteedWrap만 쓴다 — Reel에서 조준한 대상을 확정 throw로 던지기 위한 prepared path를
	// 만든다. FullSimulation/AssistedJudged는 감김이 판정/창발이라 던지기 전에 확정할 경로가 없어 preview가
	// 없다(AssistedJudged의 조준 표시는 aim ray HUD가 담당). 아래 필드는 전부 GuaranteedWrap의 표시/보류 정책이다.
	/** 비어 있으면 owner에서 찾는다. */
	UPROPERTY(EditAnywhere, Category = "Rope|Preview", meta = (UseComponentPicker, AllowedClasses = "/Script/DynamicRope.RopePreviewComponent", DisplayName = "Preview Component"))
	FComponentReference PreviewComponentReference;

	UPROPERTY(Transient)
	TObjectPtr<URopePreviewComponent> PreviewComponent = nullptr;

	/** Guaranteed가 Wrapped로 확정된 뒤에도 preview path를 잠깐 남길 시간. 0이면 Wrapped 진입 시 즉시 지운다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "0.0", Units = "s", DisplayName = "Locked Wrapped Preview Hold Time"))
	float LockedWrappedPreviewHoldTime = 0.0f;

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

	/** 능동 Pull 액션(토글): 누르면 장전, 다시 누르면 해제. 발동 시점은 장력 임계(PullEngageTension)가 정한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> PullAction = nullptr;

	/** 되감기 액션(홀드). 누르는 동안 로프의 ReelSpeed로 로프를 감고(짧아짐) 떼면 멈춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReelInAction = nullptr;

	/** 풀기 액션(홀드). 누르는 동안 로프의 ReelSpeed로 로프를 풀고(초기 길이까지) 떼면 멈춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReelOutAction = nullptr;

	/** 장전 액션. Started에 Rope->EnterReel() — ③(Guaranteed) 로프를 던지기 준비(Reel) 상태로 전환한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReloadAction = nullptr;

	//~ Movement(테더 견인에 대한 캐릭터 이동 반응) --------------------------
	// 종전 카테고리명 "Rope|Tension"은 로프 장력 설정과 혼동돼 개명(2026-07-13 표면 감사 C).
	// 테더의 wielder 몫(λ 역질량비)이 0보다 크면 로프가 wielder를 앵커 쪽으로 끌어당긴다.
	// 이 섹션은 그 견인의 캐릭터 이동 정책: 물리(플러그인 코어)가 아니라 게임 반응이라 wielder에 둔다.

	/**
	 * 로프가 위로 당기는데 지상(walking 계열)이면 발이 땅에 붙어 상승을 막는다 — 견인의 상향 성분이
	 * 충분하고 초과분이 쌓여 있으면 자동으로 Falling 전환해 몸이 뜨게 한다(착지 복귀는 엔진이 처리).
	 * 되감기(ReelIn)와 조합하면 입체기동식 "감으면 끌려 올라감"이 된다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Movement")
	bool bAutoGroundExitOnUpwardPull = true;

	/** 상향 판정 임계: 견인 방향(손→앵커, 단위 벡터)의 Z 성분이 이 값 이상일 때만 지상 이탈. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Movement", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bAutoGroundExitOnUpwardPull"))
	float GroundExitUpDot = 0.35f;

	/** 지상 이탈에 필요한 최소 테더 초과분(cm). 경계 지터로 모드가 퍼덕이는 것을 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Movement", meta = (ClampMin = "0.0", Units = "cm", EditCondition = "bAutoGroundExitOnUpwardPull"))
	float GroundExitMinOvershoot = 10.0f;

	/**
	 * 스윙 중(Wrapped + 공중 + wielder 몫 테더 활성) 에어컨트롤을 SwingAirControl로 올려 조향을
	 * 살린다. CharacterMovement 기본 AirControl(0.05)로는 스윙 방향을 거의 못 바꾼다. 스윙이 끝나면
	 * (착지/release) 저장해 둔 원래 값으로 복원한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Movement")
	bool bBoostAirControlWhileSwinging = true;

	/** 스윙 중 적용할 AirControl(0~1). 0.35~1 권장 — 1이면 공중에서 지상급 조향. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Movement", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bBoostAirControlWhileSwinging"))
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

	/**
	 * 설정하면 pull 발동 시 이 몽타주를 **단일 재생**한다: 힘은 몽타주 안에 배치한 UAnimNotifyState_RopePull
	 * window만 싣는다(ThrowMontage의 UAnimNotify_RopeThrow와 같은 계약: notify 미배치면 pull이 일어나지
	 * 않는다). 반복/중단 관리는 없다 — 발동(장전 상태에서 장력 임계 최초 돌파, UpdatePullEngage)마다 1회
	 * 재생하고 자연 종료에 맡긴다. 비우면 발동이 즉시 StartPullNow()로 힘을 장전한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation")
	TObjectPtr<UAnimMontage> PullMontage = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation", meta = (ClampMin = "0.1"))
	float PullMontagePlayRate = 1.0f;

	//~ Pull(능동 견인 정책) --------------------------------------------------
	// 힘의 크기는 로프 도메인이다(HoldConfig.PullForce) — 여기 있는 것은 "언제 발동하는가"뿐이다.
	/**
	 * Pull 발동 임계 장력(FRopeSimState::SegmentTension 단위 — ActivePullTautTension/TensionReleaseForce와
	 * 같은 단위계, 매달린 노드 1개의 중력 하중 ≈ 980). Pull 입력으로 **장전**해 두면(토글 on) Wrapped에서
	 * 장력이 이 값을 처음 넘는 순간 발동한다(0 = 팽팽 판정(IsPullTaut)만으로) — "제대로 당겨졌을 때만
	 * 끌려가기 시작"의 게임플레이 임계. 몽타주 유무와 무관하게 발동을 지배하므로 Animation이 아니라
	 * 여기 있다(무애니면 발동이 즉시 힘을 장전한다). 발동/수명 규칙은 UpdatePullEngage 주석 참조.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull", meta = (ClampMin = "0.0"))
	float PullEngageTension = 0.0f;

	/**
	 * 장전~발동 진행도를 로프 머티리얼에 밀어 넣을지(기본 켬). 로프는 플레이어 시선에 항상 있어서
	 * HUD보다 놓치기 어려운 피드백 채널이다 — M_RopeDefault의 PullGlow 스칼라가 이 값을 받는다.
	 * 실제 MID는 **처음 장전할 때** 만든다(그 전까지는 머티리얼 배선을 건드리지 않는다).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull|Feedback")
	bool bDrivePullGlowMaterial = true;

	/** 진행도를 받을 스칼라 파라미터 이름. 머티리얼에 없으면 무시된다(무해). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull|Feedback", meta = (EditCondition = "bDrivePullGlowMaterial"))
	FName PullGlowParameterName = TEXT("PullGlow");

	/** 발동(③) 상태에서 파라미터에 실을 값. 1보다 크게 두면 발동이 대기보다 확실히 밝다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull|Feedback", meta = (ClampMin = "0.0", EditCondition = "bDrivePullGlowMaterial"))
	float PullGlowEngagedValue = 1.5f;

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

	/** 방향을 명시해 던진다. AimDir이 유효하면 frame forward를 대체하고(aim ray도 같은 방향으로 쏜다),
	 *  ZeroVector면 로프 ThrowParams.FrameMode의 Forward를 그대로 쓴다(= ThrowNow와 동일). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowInDirection(const FVector& AimDir);

	/** ThrowMontage를 owner 메시의 AnimInstance에서 재생한다(설정돼 있을 때). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void PlayThrowMontage();

	/** PullMontage를 owner 메시의 AnimInstance에서 재생한다(설정돼 있을 때). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void PlayPullMontage();

	/** 현재 wrap을 해제한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Release();

	/**
	 * 능동 Pull **장전**(토글 on) — 입력/게임플레이가 호출하는 진입점. 힘/몽타주는 여기서 시작하지 않고,
	 * Wrapped에서 장력이 PullEngageTension을 처음 넘는 순간 발동한다(UpdatePullEngage — 몽타주 단일 재생
	 * 또는 즉시 힘). 감기 전에 장전해 두면 감겨서 당겨지는 순간 자동 발동한다. 해제는 StopPull.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartPull();

	/** 실제 능동 Pull을 *지금* 시작한다(로프 HoldConfig.PullForce로 견인 — Wrapped + 팽팽할 때만 실제 인가.
	 *  팽팽 판정/게이트는 로프 HoldConfig의 bActivePullRequiresTaut/ActivePullTautTension, 조회는 IsPullTaut()).
	 *  bIgnoreTautGate=true면 이번 Pull은 팽팽함을 무시한다(per-call 우회 — pull window의 연출 구간용).
	 *  몽타주 notify가 호출하는 실행 지점(ThrowNow 대응). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartPullNow(bool bIgnoreTautGate = false);

	/** 능동 Pull 힘만 정지한다(몽타주는 건드리지 않음 — pull window의 NotifyEnd가 호출: 창이 닫혀도
	 *  몽타주의 후속 구간(회수 모션 등)은 계속 재생돼야 한다). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StopPullNow();

	/** 능동 Pull 장전 해제(토글 off) + 힘 정지 + 발동 래치 리셋. 몽타주는 중단하지 않는다(단일 재생 — 자연 종료). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StopPull();

	/** Pull 장전(토글) 상태 — ① 해제 / ② 장전. 힘이 실렸는지는 IsPullEngaged. */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull")
	bool IsPullArmed() const { return bPullArmed; }

	/** Pull이 발동해 힘이 실린 상태인가(③). wrap이 풀리면 false로 돌아가며 장전은 유지된다. */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull")
	bool IsPullEngaged() const { return bPullEngaged; }

	/**
	 * 발동 임계까지의 진행도 0..1 — "눌렀는데 왜 안 당겨지지"를 게이지로 답하는 값.
	 * 발동 후에는 1. Wrapped가 아니면 0(감기기 전에는 임계 자체가 성립하지 않는다).
	 * PullEngageTension이 0(팽팽 판정만으로 발동)이면 연속값이 없으므로 IsPullTaut 기준 0 또는 1이다.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull")
	float GetPullEngageProgress() const;

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

	/** 입력을 수동으로 바인딩한다. 자동 바인딩이 타이밍상 실패하면(InputComponent 미준비) Pawn의
	 *  SetupPlayerInputComponent에서 호출하라. 이미 바인딩됐으면 무시. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Input")
	void BindInput();

	/**
	 * 던지기 preview를 **표시**할지 여부(디자이너 설정). 표시 전용 플래그다 — 끄더라도 ③ GuaranteedWrap의
	 * 던지기 계산(prepared preview)은 그대로 돌아가므로 던지기 동작에는 영향이 없다.
	 * 켜져 있고 ③이면 PreviewComponent가 BeginPlay에서 자동 생성된다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Preview")
	bool bShowThrowPreview = true;

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void SetThrowPreviewEnabled(bool bEnabled);

	/** 던지기 preview 표시가 현재 켜져 있는가. */
	UFUNCTION(BlueprintPure, Category = "Rope|Preview")
	bool IsThrowPreviewEnabled() const { return bShowThrowPreview; }

	/**
	 * 로프 ResolveMode에서 유도되는 상태(preview 자동 생성/표시, 컴포넌트 틱 활성, 조준 HUD)를
	 * 현재 모드로 재동기화한다. preview 생성과 틱 활성은 BeginPlay에서만 계산되므로 런타임에
	 * 모드가 바뀌면(URopeComponent::ApplyPreset — OnPresetApplied 구독으로 자동 호출) 이걸로 다시
	 * 계산한다. 게임 코드가 ResolveMode를 직접 바꿨을 때 수동 호출해도 된다(GT, 콜드 패스).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void RefreshModeDerivedState();

	//~ Events(이벤트) ------------------------------------------------------
	/** 던지기가 실제로 실행된 직후(즉시/몽타주 notify 경로 모두). */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeWielderOnThrown OnThrown;

	/** 던지기 입력이 실행되지 못했을 때(사유 포함) — UI 피드백용. ③을 Reel 밖에서 던지려 한 경우도 여기로 온다. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeWielderOnThrowRejected OnThrowRejected;

	/** aim ray가 새 대상(Mesh, Bone)에 걸린 순간(진입/전환). HUD 연출·사운드 트리거용. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Aim HUD")
	FRopeWielderOnAimTargetChanged OnAimTargetChanged;

	/** aim ray가 대상을 잃은 순간. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Aim HUD")
	FRopeWielderOnAimTargetLost OnAimTargetLost;

	/**
	 * Pull 장전 토글 변화(StartPull/StopPull — ① 해제 ↔ ② 장전). 장전음/UI 표시 전환용.
	 * 발동(힘이 실제로 실리는 순간)은 아래 OnPullEngagedChanged다 — 둘은 다른 사건이다.
	 */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Pull")
	FRopeWielderOnPullArmedChanged OnPullArmedChanged;

	/**
	 * Pull 발동 래치 변화(② 대기 ↔ ③ 발동). true = 장력 임계를 넘어 힘이 실린 순간(wrap당 1회),
	 * false = wrap이 풀려 재무장했거나 장전을 해제한 순간. **UI는 반드시 false도 처리해야 한다** —
	 * 발동 표시가 켜진 채 남는 흔한 버그가 여기서 갈린다.
	 */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Pull")
	FRopeWielderOnPullEngagedChanged OnPullEngagedChanged;

	/** 최근 정상 collider gather에서 확정한 조준 HUD 샘플(aim ray 모드에서 틱마다 소비·갱신). */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	const FRopeAimHudSample& GetAimHudSample() const { return AimHudSample; }

protected:
	//~ 확장 훅(서브클래스용) ------------------------------------------------
	// URopeComponent와 같은 원칙: 전부 게임 스레드·프레임 단위(콜드 패스)에서만 불린다.
	// 훅을 추가할 때는 호출 시점/빈도를 주석에 명시하는 것을 계약의 일부로 삼는다.
	// (public의 BuildThrowContext도 virtual 확장 훅이다.)

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
	/** 최근 정상 collider gather의 조준 HUD 샘플(Tick에서 UpdateAimHudSample이 결과를 소비해 갱신). */
	FRopeAimHudSample AimHudSample;

	/** 최근 정상 gather가 확정한 throw context. Preview는 HUD가 소비한 같은 Wielder tick에 이를 재사용한다. */
	FRopeThrowContext AimRayFrameThrowContext;
	uint64 AimRayFrameContextStamp = 0;
	bool bHasAimRayFrameThrowContext = false;

	/** 자동 생성한 조준 HUD 위젯(로컬 플레이어 전용). bShowAimHudWidget/모드 변경에 따라 생성·제거. */
	UPROPERTY(Transient)
	TObjectPtr<URopeAimWidget> AimHudWidget = nullptr;

	/** 자동 생성한 Pull 게이지 위젯(로컬 플레이어 전용). bShowPullGaugeWidget에 따라 생성·제거. */
	UPROPERTY(Transient)
	TObjectPtr<URopePullGaugeWidget> PullGaugeWidget = nullptr;

	/**
	 * aim 요청을 등록하고 정상 gather의 최근 결과로 HUD 샘플을 갱신한 뒤, 대상 (Mesh, Bone) 변화 시
	 * OnAimTargetChanged/OnAimTargetLost를 발화한다(Tick, aim ray 모드 전용, 최대 1프레임 지연).
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

	/** 컴포넌트 틱이 필요한가 — preview(③)/지상 이탈/스윙 에어컨트롤/aim ray(②③) 중 하나라도.
	 *  BeginPlay · SetThrowPreviewEnabled · RefreshModeDerivedState가 공유하는 단일 식. */
	bool ComputeDesiredTickEnabled() const;

	/** 로프 ApplyPreset 성공 신호(OnPresetApplied) 핸들러 — 모드 유도 상태 재동기화. */
	UFUNCTION()
	void HandleRopePresetApplied(const URopePreset* Preset);

	/** wielder가 테더 몫을 실제로 받는 상태인가(Wrapped + 유효 대상 몫 < 1 + 셀프랩 아님). */
	bool IsWielderTetherActive() const;

	/** wielder 몫 테더가 위로 당길 때 walking이면 Falling으로 전환한다(매 틱, GT — 위 Tension 섹션 참고). */
	void UpdateGroundExit();

	/** 스윙 판정에 따라 AirControl을 부스트/복원한다(매 틱, GT). */
	void UpdateSwingAirControl();

	/**
	 * 장전된 Pull(bPullArmed)의 발동 판정(매 틱, GT). Wrapped + 장력 조건(임계 0 = IsPullTaut, > 0 =
	 * GetMaxTension ≥ PullEngageTension)을 처음 만족하는 순간 발동한다(래치 — wrap당 1회): 몽타주 셋업이면
	 * PullMontage **단일 재생**(힘은 window notify가 싣는다 — 반복/중단 관리 없음), 무애니면 즉시 힘 장전.
	 * 비Wrapped가 되면 힘을 끄고 재무장한다(장전 유지 — 다음 wrap에서 재발동).
	 */
	void UpdatePullEngage();

	void UpdateThrowPreview();

	/** 주어진 centerline을 preview 컴포넌트에 넘겨 그린다(표시 OFF/컴포넌트 없음이면 no-op). */
	void DisplayPreviewCenterline(const FRopeWrapPreviewData& Centerline);

	/** 확정된 HeldPreparedPreview를 새 build 없이 그대로 유지 표시한다(표시 OFF면 no-op). */
	void DisplayHeldPreparedPreview();

	/** 표시만 정리한다 — prepared(던지기용 데이터)는 유지된다. */
	void ClearPreviewDisplay();

	/** 던지기용 prepared 상태만 정리한다 — 표시는 건드리지 않는다. */
	void ClearPreparedThrow();

	/** prepared + 표시를 모두 정리한다. */
	void ClearThrowPreview();

	/** collider/SDF side effect 없이 origin/frame/속도만 계산한다. */
	FRopeThrowContext BuildBaseThrowContext(const FVector& AimDir) const;
	// 실제 ray를 새로 검사하고 throw 순간에 고정할 context를 구성한다.
	FRopeThrowContext BuildThrowContextInternal(const FVector& AimDir) const;
	// 같은 Wielder tick에 HUD가 소비한 최근 gather의 aim context가 있으면 Preview가 재사용한다.
	bool TryGetCachedAimRayThrowContext(const FVector& AimDir, FRopeThrowContext& OutContext) const;
	// 입력 순간의 base frame과 ray 설정을 값 타입 요청으로 캡처한다.
	FRopeAimRayThrowRequest BuildAimRayThrowRequest(const FVector& AimDir) const;
	// ③ 실제 입력 ray를 정상 gather 직후 prepared로 확정하도록 큐에 넣는다.
	bool QueueGuaranteedAimThrow(const FVector& AimDir, bool bExecuteWhenReady);
	// 선택한 origin 모드를 월드 위치로 해석한다.
	FVector GetAimRayOrigin() const;
	float GetAimReachLength() const;
	// Aim hit prepared spline을 wielder owner-local 좌표로 저장해 손 소켓 애니메이션에서 분리한다.
	void StoreAimGuideFrameIfNeeded(FRopePreparedThrowPreview& Prepared) const;
	// 저장된 owner-local prepared spline을 현재 owner transform 기준으로 렌더한다.
	FRopeWrapPreviewData ResolvePreparedPreviewForDisplay(const FRopePreparedThrowPreview& Prepared) const;
	bool ShouldHoldPreparedPreview();
	// 현재 Rope phase에서 새 preview path를 계산해도 되는지 판단한다. false면 비싼 build 경로에 들어가지 않는다.
	bool ShouldUpdateThrowPreviewForPhase(ERopePhase Phase) const;
	// ③이 이미 확정한 path를 GuidedThrow/Wrapped 동안 렌더 유지한다. 처리했으면 true를 반환한다.
	bool UpdateHeldPreparedPreviewForPhase(ERopePhase Phase);

	void OnThrowInput();
	void OnGuaranteedAimPrepared(FRopePreparedThrowPreview& Prepared);
	void OnAimRayThrowResolved();
	void OnAimRayThrowRejected();
	void OnReleaseInput();
	void OnPullInputStarted();
	void OnReelInStarted();
	void OnReelOutStarted();
	void OnReelCompleted();
	void OnReloadInput();

	/**
	 * possession 변화 훅 — 늦은 빙의(스폰 직후 컨트롤러 없음)와 재빙의에서 입력을 다시 건다.
	 * 종전에는 BeginPlay에서 딱 한 번만 시도해, 그 시점에 컨트롤러가 없으면 IMC가 영영 안 붙고
	 * InputComponent가 없으면 바인딩도 영영 안 걸렸다(Throw/Pull/Reel 무반응). bAutoBindInput일 때만
	 * 구독한다 — 수동 바인딩 게임은 SetupPlayerInputComponent가 재빙의마다 다시 불려 이미 안전하다.
	 */
	UFUNCTION()
	void HandlePawnControllerChanged(APawn* OwnerPawn, AController* OldController, AController* NewController);

	/** InputComponent는 PawnClientRestart에서 만들어진다 — 컨트롤러가 붙은 직후엔 아직 없을 수 있어
	 *  restart 시점에도 한 번 더 시도한다(둘 중 늦은 쪽이 실제로 바인딩을 성사시킨다). */
	UFUNCTION()
	void HandlePawnRestarted(APawn* OwnerPawn);

	/** IMC 재부착 + 액션 재바인딩(현재 소유 폰 기준). BeginPlay와 위 두 훅의 공용 경로. */
	void RefreshInputRegistration();

	/** AddMappingContext가 꽂은 IMC를 캐시된 서브시스템에서 뗀다(possession 전환/EndPlay 공용). */
	void RemoveMappingContext();

	bool bInputBound = false;
	// 실제로 바인딩을 건 InputComponent. 재빙의로 새 InputComponent가 생기면 bInputBound만으로는
	// "어디에 걸었는지"를 알 수 없어, 새 컴포넌트엔 안 걸린 채 true가 유지됐다(입력 영구 누락).
	TWeakObjectPtr<UInputComponent> BoundInputComponent;
	// AddMappingContext가 IMC를 꽂은 로컬 플레이어 Enhanced Input 서브시스템(weak). IMC는 Pawn이 아니라
	// LocalPlayer에 등록되므로, EndPlay가 폰의 현재 컨트롤러에 의존하지 않고 여기서 possession 무관하게
	// 제거한다(#11 — 폰이 먼저 unpossess된 뒤 파괴돼도 IMC가 로컬 플레이어에 잔류하는 것 방지). LP 파괴 시 null.
	TWeakObjectPtr<UEnhancedInputLocalPlayerSubsystem> MappedInputSubsystem;
	/** bPullArmed 변경 + OnPullArmedChanged 브로드캐스트(변화가 있을 때만). */
	void SetPullArmed(bool bNewArmed);

	/** bPullEngaged 변경 + OnPullEngagedChanged 브로드캐스트(변화가 있을 때만). */
	void SetPullEngaged(bool bNewEngaged, float Tension);

	/** 진행도를 로프 MID의 스칼라 파라미터로 민다(bDrivePullGlowMaterial일 때, 첫 장전 이후에만). */
	void UpdatePullGlowMaterial();

	/** Pull 게이지 위젯 수명 관리(로컬 플레이어 준비 후 생성, 토글 꺼짐/EndPlay에 제거). */
	void UpdatePullGaugeWidget();

	// pull 장전 상태(StartPull~StopPull 사이 토글). 발동 판정은 UpdatePullEngage.
	bool bPullArmed = false;
	// pull 발동 래치(장전 중 장력 임계 최초 돌파 시 true — 몽타주 단일 재생/힘 장전 완료). wrap 해제 시 재무장.
	bool bPullEngaged = false;
	// 로프 머티리얼에 진행도를 싣기 위해 만든 MID. 프리셋 적용 등으로 로프 머티리얼이 교체되면
	// GetMaterial(0)과 어긋나므로, 그때 다시 만든다(매 틱 확인 — 포인터 비교 1회).
	TWeakObjectPtr<UMaterialInstanceDynamic> PullGlowMID;
	// AirControl 부스트 원복용 저장 상태(스윙 진입 시 저장, 종료/EndPlay 시 복원).
	bool bAirControlBoosted = false;
	float SavedAirControl = 0.0f;

	// 마지막 preview tick에서 성공한 prepared 결과. ③에서 "지금 조준이 잡혔는가"를 판정한다.
	FRopePreparedThrowPreview LastPreparedPreview;

	// ③ 입력 ray가 정상 gather에서 prepared로 확정되거나 몽타주 notify 실행을 기다리는 동안 true.
	bool bGuaranteedAimThrowQueued = false;

	// ③ 실행 중(GuidedThrow 포함) 화면에 유지할 확정 path.
	FRopeWrapPreviewData HeldPreparedPreview;
	// Wrapped 후 preview path를 잠깐 남길 때 사용하는 만료 시각. LockedWrappedPreviewHoldTime이 0이면 즉시 만료된다.
	float HeldPreviewExpireTimeSeconds = 0.0f;
	// Wrapped 진입 순간을 감지하기 위한 마지막 preview 처리 phase.
	ERopePhase LastPreviewPhase = ERopePhase::Free;
};
