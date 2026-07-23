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
#include "Core/RopeSimFrameIO.h"
#include "Core/RopePullDriveState.h"
// FRopeAimRayHitResult/FRopeAimRayThrowRequest + 조준 로직/상태.
#include "Logic/RopeAimTargeting.h"
// 슬립 + 거리 LOD(솔브 스로틀).
#include "Logic/RopeSolverThrottle.h"
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
class URopePreset;
class USkeletalMeshComponent;
class UStaticMesh;
class UStaticMeshComponent;
// 랩 대상 추상화(Decision 0): 랩 대상 mesh를 USceneComponent로 일반화.
class USceneComponent;
class FRegisterComponentContext;
struct FRopeDebugSnapshot;
// 디버거 노드별 flight 시각화 항목(Debug/RopeDebugSnapshot.h).
struct FRopeFlightNodeDebug;
// 디버거 캡처 범위 비트(Debug/RopeDebugSnapshot.h) — 켜진 보기만 수집하도록 캡처 측에 전달된다.
enum class ERopeDebugCapture : uint8;

// Wrapped 성립 이벤트는 본 이름 하나에서 구조체 페이로드로 확장됐다(2026-07-13 회의 결정 G —
// 결착/판정값/복수 본. 기존 BP 바인딩은 재연결 필요, 클린 브레이크 승인 사항).
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnWrapped, const FRopeWrappedEventInfo&, Info);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnCaptured, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnReleased, FName, Bone, ERopeReleaseReason, Reason);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnPhaseChanged, ERopePhase, OldPhase, ERopePhase, NewPhase);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnPresetApplied, const URopePreset*, Preset);

// (FRopeAimRayHitResult / FRopeAimRayThrowRequest는 Logic/RopeAimTargeting.h로 이동 — 위 include로 계속 노출된다.)

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeComponent : public UMeshComponent
{
	GENERATED_BODY()

	// 서브시스템이 프레임 구동을 위해 Sim/SolverConfig/Phase/WhipGuide + SimFrame(프레임 계약 묶음 —
	// FRopeSimFrameIO 주석 참조)에 직접 접근한다(GPU 배치 솔브 포함; CPU 경로는 SolveSimFrame 사용).
	friend class URopeSimSubsystem;

#if WITH_DEV_AUTOMATION_TESTS
	// 테스트 시임: ApplyPreset 페이즈 게이트 음성 테스트(RopePresetTests)가 SetPhase를 강제하기 위한 최소 접근.
	friend struct FRopePresetTestSeam;
	// 테스트 시임: Contacting 시드의 anchor 불변식과 synthetic latch fallback 진입을 검증하는 최소 접근.
	friend struct FRopeWrappingFallbackTestSeam;
	// 테스트 시임: virtual bridge 수명과 GuidedThrow 공통 진입 상태를 검증하는 최소 접근.
	friend struct FRopeComponentRefactorTestSeam;
	// 테스트 시임: Wielder 입력/당김 수명주기와 self-wrap 판정을 world 없이 재현하기 위한 최소 접근.
	friend struct FRopeWielderComponentTestSeam;
#endif

public:
	URopeComponent();

	//~ Setup(설정) -------------------------------------------------------

	// 이 로프의 최상위 계약 — 무엇을 보장하는지, 조준·preview의 지위, 판정 관문 사용 여부를 이 값
	// 하나가 결정한다. **정본은 로프다**: Wielder의 조준/던지기 방식은 여기서 유도되고, BP 직행/AI는
	// Wielder 없이 이 값만으로 완결된다. 모드별 계약은 ERopeWrapResolveMode 열거자 주석 참조.

	/** 감김 해결(도달) 모드 — 던지기~결착까지 무엇을 보장하는가. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	//~ Tip(팁 부착물 — 창날/작살/추) ----------------------------------------
	// 밧줄 자유단(GetNodeCount()-1)에 붙는 표시 전용 StaticMesh. 질량·충돌 없음(팁 질량 솔버 반영
	// 안 함 — 2026-07-14 확정). **결착 모델 무관 공통 기능**이다(2026-07-17): bUseTipMesh 하나로
	// 켜고, 수명은 전 모드 BeginPlay~EndPlay로 통일한다 — (우리가 스폰한 경우만) EndPlay에 파괴하고,
	// 외부(태그로 찾은) 컴포넌트는 파괴하지 않는다.
	// 예외는 소켓 보정(bUseTipMeshSockets) 하나 — Head를 꽂힘 지점에 맞춘다는 개념이 ③에만 있다.
	// 활성 조건의 단일 소스는 IsTipSocketPlacementActive()이고, 존재 확인/읽기는 HasTipSocket/ReadTipSocketLocal이 맡는다.
	//
	// 폴백(소켓 보정이 꺼졌거나 ③이 아니거나 Head 소켓이 없을 때): 메쉬 원점이 로프 끝 노드에, X축이
	// 마지막 세그먼트 방향에 놓인다 — 팁이 대상에 파묻혀도 보정하지 않는다(의도된 무보정). Wrapped에서도
	// 끝 노드가 bone-local 앵커라 애니메이션은 계속 따라가고, 회전만 얼린 자세가 아닌 세그먼트 유도가 된다.
	// Head만 있고 Tail이 없으면 로프는 메쉬 원점에 연결된다.
	//
	// 주의: 아래 /** */는 그대로 에디터 툴팁이 된다 — 한 줄로 짧게 유지하고, 상세는 이 블록에 적을 것.

	/** 팁 부착물을 사용한다. 끄면 아래 Tip 설정이 전부 무시되고 팁 없는 일반 로프가 된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip")
	bool bUseTipMesh = false;

	/** 팁에 스폰할 StaticMesh. 비어 있고 태그로도 못 찾으면 팁 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	TObjectPtr<UStaticMesh> TipMesh = nullptr;

	/** Owner에 이미 붙은 이 태그의 StaticMeshComponent를 팁으로 재사용(스폰보다 우선, 파괴 안 함). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	FName TipMeshComponentTag = NAME_None;

	/** 팁 배치 오프셋(팁 노드 프레임 기준). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	FTransform TipMeshRelativeTransform = FTransform::Identity;

	// 팁(창날/작살/추)은 자유단을 매 프레임 따라가는 표시 전용 메쉬라, 충돌 바디가 켜져 있으면 캐릭터
	// 캡슐/월드와 부딪히거나 로프의 충돌 질의와 간섭해 로프 거동이 튄다 — **기본 꺼짐**. 켜면 우리가
	// 스폰한 팁은 전체 충돌(QueryAndPhysics)을, 태그로 재사용한 외부 컴포넌트는 저작 충돌(획득 시점 값)을
	// 갖는다. 적용 시점은 팁 확보(EnsureTipMesh)와 에디터/PIE 편집(PostEditChangeProperty)이다.

	/** 팁 메쉬의 충돌을 켠다. **기본 꺼짐** — 표시 전용 팁의 충돌이 로프/캐릭터와 간섭하는 것을 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	bool bTipMeshCollision = false;

	// 끔 = 게임 코드가 GetTipMeshComponent()로 Free 배치를 직접 구동하라는 확장점(로프는 손대지 않는다).
	// Free 외 페이즈(Flight/GuidedThrow/Wrapping/Wrapped/Releasing/Loaded)는 이 값과 무관하게 항상 추종한다.

	/** Free에서 팁을 매 프레임 로프 끝에 맞춘다. 끄면 Free 동안 팁을 건드리지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	bool bSyncTipMeshOnFree = true;

	// 기본 GetLoadedTipTransform() 구현이 사용한다 — 배치 규약을 바꾸려면 그 virtual을 override.

	/** Loaded(장전)에서 팁을 붙일 Owner 스켈레탈 메시 소켓. 없으면 컴포넌트(손) 트랜스폼. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	FName LoadedHandSocket = NAME_None;

	// 켬 = Tail이 로프 끝에, Head가 꽂힘 지점에 오도록 메쉬 원점을 역산하고 그 자세를 bone-local로 얼려
	// 대상 애니메이션을 따라간다. 끔 = 소켓을 일절 읽지 않는다(위 폴백). ①②에는 무의미.

	/** ③(Guaranteed) 전용 — Head/Tail 소켓으로 팁을 정밀 배치한다. 끄면 메쉬 원점이 로프 끝에 놓인다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap"))
	bool bUseTipMeshSockets = false;

	/** Head 소켓 — 팁의 뾰족한 끝. 이 소켓이 조준 히트점에 박힌다. 없으면 위 보정 비활성. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap"))
	FName TipSocketName = NAME_None;

	/** Tail 소켓 — 로프 자유단이 연결될 지점. 없으면 메쉬 원점에 연결. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap"))
	FName TipRopeSocketName = NAME_None;

	// 아래 초기화 전용 값들(NumParticles/RopeLength/MinRopeLength)은 InitRope 시점에만 소비된다 —
	// 런타임 쓰기는 재초기화 전까지 무효라 BlueprintReadOnly(함정 방지). 런타임 길이 변경은
	// SetRopeLength/SetReelRate를 쓴다.

	// ClampMax 512 = FRopeGPUSolver::MaxNodes(GPU 솔버 스레드그룹 상한). 초과하면 조용히 CPU 솔브+튜브
	// 폴백이 되어 성능 절벽 + 저작 무신호라 에디터에서 막는다(BP/코드 경로는 InitRope가 하드 클램프).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "2", ClampMax = "512"))
	int32 NumParticles = 72;

	/** 초기(최대) 로프 길이(cm). 런타임 현재 길이는 GetCurrentRopeLength/SetRopeLength. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 600.0f;

	/** 되감기(reel-in)로 줄일 수 있는 최소 길이(cm). RopeLength(초기)가 상한이다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "10.0", Units = "cm"))
	float MinRopeLength = 100.0f;

	/** 되감기/풀기 입력이 쓰는 기본 릴 속도(cm/s). 길이 변경은 로프 도메인이라 여기 산다
	 *  (2026-07-13 표면 감사 A-2 — Wielder에서 이사; Wielder Reel 액션이 이 값으로 SetReelRate 호출). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "0.0", Units = "cm/s"))
	float ReelSpeed = 150.0f;

	// Loaded(장전) 동안의 로프 튜브 가시성. 기본 OnEnterLoaded() 구현이 소비하는 값이라, 그 훅을 override해
	// 자체 연출을 넣으면 이 값은 무시된다. 끔 = 창만 손 소켓에 보이는 연출, 켬 = 손~창 사이 늘어진
	// 로프가 그대로 보인다(Loaded에서도 솔브는 돌아 로프가 처진다).
	// 직접 대입은 Loaded 중이면 반영되지 않으므로(가시성 적용 시점이 Loaded 진입 에지) BlueprintReadOnly +
	// SetShowRopeWhenLoaded/ToggleShowRopeWhenLoaded 세터를 쓴다(RopeMaterial과 같은 이유).

	/** Loaded(장전) 상태에서 로프 튜브를 보인다. ③(GuaranteedWrap) 전용 연출 스위치. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope",
		meta = (EditCondition = "ResolveMode == ERopeWrapResolveMode::GuaranteedWrap"))
	bool bShowRopeWhenLoaded = false;

	// 아래 도메인별 설정 구조체는 전부 ShowOnlyInnerProperties로 노출한다 — 디테일 패널에서 카테고리
	// 헤더(Rope|Solver / Rope|Throw / …) 바로 아래에 필드가 펼쳐지므로, "카테고리 → 구조체 이름 →
	// 필드"의 이중 확장 없이 한 단계로 편집된다. BP/직렬화에는 영향이 없다(구조체는 그대로 하나의
	// BlueprintReadWrite 변수). 각 필드의 세부 카테고리(Rope|Solver|Scaling 등)는 구조체 내부에서 유지된다.

	/** XPBD 솔버 튜닝(Free/Flight 물리). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ShowOnlyInnerProperties))
	FRopeSolverConfig SolverConfig;

	/** 던지기/발사 파라미터(Flight 진입). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ShowOnlyInnerProperties))
	FRopeThrowParams ThrowParams;

	/** physics → logic (wrap) 핸드오프 — *성립*(경로 빌드/판정/커밋) 튜닝. 감지는 DetectConfig. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ShowOnlyInnerProperties))
	FRopeWrapConfig WrapConfig;

	/** Flight/Contacting *감지*(언제 잡혔다고 볼 것인가) 튜닝 — 성립(WrapConfig)과 분리된 도메인
	 *  (2026-07-13 표면 감사 B-1). 공유 프로브 반경(ContactQueryRadius)은 WrapConfig 소유. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Detect", meta = (ShowOnlyInnerProperties))
	FRopeDetectConfig DetectConfig;

	/** Wrapped *이후*(유지/당김/풀림) 튜닝 — 성립 판정(WrapConfig)과 분리된 Post-Wrap 도메인
	 *  (2026-07-13 표면 감사 B-1; 설계 노트 01 도메인, 도달 모드·결착 모델 무관 공통). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold", meta = (ShowOnlyInnerProperties))
	FRopeHoldConfig HoldConfig;

	//~ Collision(충돌 도메인) ----------------------------------------------
	// 흩어져 있던 충돌 관련 스위치를 한자리에 응집(2026-07-13 표면 감사 CL-4). 반지름 자체는
	// SolverConfig.CollisionRadius / WrapConfig.ContactQueryRadius에 있고, 0(기본)=auto — 아래
	// GetEffective* 헬퍼가 렌더 Radius에서 유도한다(반지름 3종 자동 정합).

	/**
	 * 기본적으로 rope는 월드의 모든 collider provider와 충돌하되 **자기 owner(던진 본인)의 것은 제외**한다
	 * — throw 시 늘어진 로프가 던진 사람 팔다리에 엉키는 것을 막기 위함. cross-actor wrap(다른 액터 body 잡기)은
	 * 그 액터가 "전체"에 포함되므로 자동으로 동작한다.
	 *
	 * 제외 범위는 두 갈래다:
	 *  - 스켈레톤/랩 대상 provider는 **provider 단위**(owner의 provider를 통째로 건너뜀).
	 *  - 정적 월드 provider(URopeStaticBodyProvider)는 **바디 단위** — 월드를 훑다가 잡은 셰이프 중 출처
	 *    액터가 owner인 것만 뺀다(테더 프록시·팁 메쉬·든 무기 등이 로프를 따라다니며 제 로프를 미는 것 방지).
	 *    바닥/기둥 등 다른 액터의 월드 지오메트리는 그대로 남는다.
	 *
	 * **로프를 프롭에 얹은 구성(기둥·크레인·앵커 액터에 URopeComponent를 붙인 경우)은 이 값을 켜야 한다** —
	 * 끄면 그 받침대의 콜리전도 owner 소유라 제외되어 로프가 받침대를 통과한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision",
		meta = (ToolTip = "끄면(기본) 자기 owner의 콜라이더를 제외합니다 — 정적 월드 provider가 잡은 owner 소유 셰이프(테더 프록시/팁/무기)도 바디 단위로 빠집니다. 로프를 기둥 등 프롭 액터에 붙였다면 켜세요(안 켜면 받침대를 통과)."))
	bool bIncludeOwnerColliders = false;

	/**
	 * 엔진 Global Distance Field로 정적 월드 지오메트리(벽/바닥)에서 로프를 밀어낸다. GPU 경로(씬 그래프
	 * dispatch)에서만 동작. 프로젝트에 Generate Mesh Distance Fields 필요 — GDF가 무효면 조용히 no-op이라
	 * **기본 켜짐**(벽/바닥 뚫림이 기본 방어되는 쪽이 안전; GDF 온디맨드 빌드 비용을 아끼려면 끔).
	 * 본 귀속·표면속도 없음(정적 월드 광역 밀어내기 보완재) — per-bone SDF의 대체가 아니다. 켜져 있는 동안
	 * 엔진이 GDF를 온디맨드로 빌드한다. 밀어내기 반경/마찰은 CollisionRadius/Friction/TipFrictionScale 공유.
	 * (SolverConfig에서 컴포넌트 직속으로 이사 — 충돌 도메인 응집.)
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
	bool bUseWorldGDF = true;

	/** 해석된 솔버 충돌 반지름: SolverConfig.CollisionRadius(0=auto → 렌더 Radius). 솔브/GPU step 경계에서 소비. */
	float GetEffectiveCollisionRadius() const
	{
		return SolverConfig.CollisionRadius > 0.0f ? SolverConfig.CollisionRadius : Radius;
	}

	/** FullSimulation/Assisted의 Flight/Wrapping 및 커밋 프레임에 사용할 최대 신장 배율.
	 *  가이드/래핑 override가 만든 간격을 탄성 strain으로 저장하지 않되, 안정된 Wrapped는 설정값으로 복귀한다. */
	float GetEffectiveMaxStretchRatio() const;

	/** 해석된 접촉 질의 반지름: WrapConfig.ContactQueryRadius(0=auto → 렌더 Radius × 1.5). 감지/랩 경로 경계에서 소비. */
	float GetEffectiveContactQueryRadius() const
	{
		return WrapConfig.ContactQueryRadius > 0.0f ? WrapConfig.ContactQueryRadius : Radius * 1.5f;
	}

	//~ Whip(던지기 스윙 설정) ----------------------------------------------
	/** 던지기 초반 채찍 스윙 튜닝. 런타임 상태는 WhipGuide가 소유하고, 호출 시
	 *  MakeWhipGuideConfig()로 스냅샷을 만들어 넘긴다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ShowOnlyInnerProperties))
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
	UPROPERTY(EditAnywhere, BlueprintReadOnly, AdvancedDisplay, Category = "Rope|Render", meta = (ClampMin = "1", ClampMax = "8"))
	int32 TubeSmoothingSubdiv = 1;

	/** 렌더 튜브 스무딩의 Catmull-Rom knot α: 0=uniform, 0.5=centripetal(급한 코너에서 접선 오버슈트↓ —
	 *  벽을 짚는 구간의 중간 링이 벽 밖으로 덜 부푼다), 1=chordal. CPU 스무딩과 GPU resident 스무딩이
	 *  같은 값을 써 렌더가 일관된다. TubeSmoothingSubdiv=1이면 효과 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, AdvancedDisplay, Category = "Rope|Render", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TubeSmoothingAlpha = 0.5f;

	/** rope tube에 적용되는 material. 설정하지 않으면 엔진 기본 material을 사용한다.
	 *  런타임 교체는 SetMaterial(0, M)으로 할 것 — 씬 프록시가 생성 시점에 머티리얼을 캡처하므로 직접 대입하면
	 *  MarkRenderStateDirty가 없어 다음 프록시 재생성 전까지 교체가 반영되지 않는다.
	 *  BP의 직접 Set은 후킹할 수 없어 BlueprintReadWrite가 아니다 — RopeLength가 BlueprintReadOnly +
	 *  SetRopeLength인 것과 같은 이유. 에디터 디테일 패널 편집은 PostEditChangeProperty가 처리한다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render")
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	//~ API ---------------------------------------------------------------

	/**
	 * 프리셋(URopePreset) 통째 적용 — 값 복사(스탬프) 후 로프를 재초기화한다. **Free/Loaded에서만**
	 * 성립하고 그 외 페이즈(날아가거나 감고 있는 중)는 false를 반환하며 아무것도 바꾸지 않는다.
	 * 적용 시: Sim 재시드(InitRope) + 렌더/MID 재구성 + 팁 재확보 + 모드-페이즈 정합(③이면 Loaded
	 * 진입, Loaded이었는데 ①②가 되면 Free 복귀).
	 * TipMeshComponentTag 등 인스턴스 배선 값은 프리셋 밖이라 유지된다 — 태그로 잡은 외부 팁을
	 * bUseTipMesh=false 프리셋이 숨겨 주지는 않는다(인스턴스 책임). 리플리케이션 없음(로컬 스탬프).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool ApplyPreset(const URopePreset* Preset);

	/** rope를 발사한다. ①②는 초기 tip 속도를 받아 물리 Flight로, ③은 Loaded에서만 성립하며 확정 경로를
	 *  따라가는 GuidedThrow로 진입한다(모드가 경로를 정한다). 실제 방향은 ThrowParams.FrameMode의
	 *  Forward가 단일 소스다. 방향을 직접 지정하려면 ThrowWithContext(FRopeThrowContext)를 쓸 것. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw();

	/** Wielder가 origin/frame/속도까지 계산해 넘기는 확장 throw 진입점. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowWithContext(const FRopeThrowContext& ThrowContext);

	/** 요청 반경(0=미지정)을 이 로프의 폴백 규약으로 해석한 **실제 스윕 반경**. 조준 시각화가 질의와
	 *  같은 치수를 그리도록 쓴다 — 0이 기본값이라 실무에선 거의 항상 폴백이 걸린다. */
	float GetAimRayEffectiveQueryRadius(float RequestedRadius) const;

	/** Aim ray가 검사할 월드 구간을 collider subsystem의 조준 수집 region으로 등록한다. */
	void SetAimRayColliderQueryBounds(const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius);
	/** Aim ray 모드가 끝났을 때 조준 수집 region, 스냅샷, pending/캐시 결과를 함께 비운다. */
	void ClearAimRayColliderQueryBounds();
	/** HUD/preview 요청을 등록한다. 정상 subsystem collider gather 직후 해석되며 즉시 재수집하지 않는다. */
	void QueueAimRayQuery(const FRopeAimRayThrowRequest& Request);
	/** 정상 gather에서 확정한 최근 HUD/preview 결과. 다음 Wielder tick이 소비하므로 최대 1프레임 지연된다. */
	bool GetLatestAimRayQueryResult(FRopeAimRayQueryResult& OutResult) const;
	/** 호환용 레거시 API. 즉시 provider 재수집은 하지 않고 QueueAimRayQuery로 전환하며 항상 false를 반환한다. */
	UE_DEPRECATED(5.7, "Use QueueAimRayQuery/GetLatestAimRayQueryResult. Immediate collider refresh was removed.")
	bool RefreshAimRayQueryColliders(const FRopeAimRayThrowRequest& Request);
	/** 현재 조준 collider 목록으로 Aim 요청을 해석한다. hit이 없으면 OutContext는 BaseContext fallback이다. */
	bool ResolveAimRayThrowContext(const FRopeAimRayThrowRequest& Request, FRopeThrowContext& OutContext,
		FRopeAimRayHitResult* OutHit = nullptr, FRopeAimRayHitResult* OutBlockedHit = nullptr) const;
	/** 실제 throw를 최신 collider 수집 직후 확정하도록 요청을 큐에 넣는다. */
	void QueueAimRayThrow(const FRopeAimRayThrowRequest& Request);

	//~ Preview 탐색(Arc Search) 파라미터 ------------------------------------
	// GuaranteedWrap prepared 빌드가 쓰는 아크 탐색 튜닝의 **단일 소스**. Wielder 경로와 BP 직행 Throw()
	// 경로가 같은 값을 봐야 하므로 로프가 소유한다(종전엔 URopePreviewComponent에 있고 ThrowWithContext가
	// 같은 값을 하드코딩 복사해 조용히 발산할 수 있었다). 표시 전용 값(반지름/변 수/머티리얼)은
	// 렌더 쪽(URopePreviewComponent)에 남는다.

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Arc Search", meta = (ClampMin = "0.0", DisplayName = "Arc Reach Scale"))
	float PreviewReachScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Arc Search", meta = (ClampMin = "1", ClampMax = "128", DisplayName = "Arc Segment Count"))
	int32 PreviewSegmentCount = 32;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Arc Search", meta = (ClampMin = "1.0", Units = "cm", DisplayName = "Arc Sample Step"))
	float PreviewSampleStep = 80.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Arc Search", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Arc Query Radius"))
	float PreviewQueryRadius = 0.0f;

	//~ Wielder 계약(C++ 전용) ----------------------------------------------
	// URopeWielderComponent의 조준/GuaranteedWrap preview 구속 흐름이 쓰는 진입점. 일반 사용자 API가 아니라
	// BP 미노출 — 게임 코드에서 직접 부를 일은 보통 없다(Wielder를 붙이거나 같은 계약을 재구현할 때만).
	// 아크 탐색 튜닝은 인자가 아니라 위 Preview 파라미터(멤버)를 읽는다 — 호출처마다 값이 갈리지 않게.

	/** GuaranteedWrap용 preview build. 렌더 centerline뿐 아니라 실제 GuidedThrow/Wrapped 진입에 필요한 contact/anchor도 반환한다. */
	bool BuildPreparedWrappingPreview(const FRopeThrowContext& ThrowContext, FRopePreparedThrowPreview& OutPrepared,
		FString* OutFailureReason = nullptr) const;

	/** Prepared preview를 권위 있는 경로로 사용해 던진다. Flight/Contacting 재탐색을 타지 않고 GuidedThrow로 진입한다. */
	bool ThrowWithPreparedPreview(const FRopePreparedThrowPreview& Prepared);

	/**
	 * Guaranteed 입력 순간의 aim 요청을 정상 collider gather 직후 prepared path로 확정한다. 몽타주가 없으면
	 * bExecuteWhenReady=true로 즉시 실행하고, 몽타주 경로는 false로 큐에 둔 뒤 notify에서
	 * RequestExecuteQueuedGuaranteedAimThrow를 호출한다. OnPrepared → 실제 실행 → OnResolved 순서다.
	 */
	bool QueueGuaranteedAimThrow(const FRopeAimRayThrowRequest& Request, bool bExecuteWhenReady);
	/** 큐 결과가 준비됐으면 즉시 실행하고, 아직 gather 전이면 준비 직후 실행하도록 표시한다. */
	bool RequestExecuteQueuedGuaranteedAimThrow();
	/** 몽타주 취소/모드 변경/EndPlay에서 아직 실행하지 않은 Guaranteed 요청을 버린다. */
	void CancelQueuedGuaranteedAimThrow();

	/**
	 * 던지기 준비(Loaded/장전) 상태로 진입한다 — 창(팁)을 손 소켓에 든다(로프 튜브 표시는
	 * bShowRopeWhenLoaded, 기본 숨김). **③ 전용**이고
	 * **Free/Loaded에서만** 유효하다(그 외엔 no-op — 날아가거나 꽂혀 있는 중엔 장전할 수 없다).
	 * ③ 로프는 BeginPlay에서 Loaded로 시작한다. 던지기는 이 상태에서만 성립(CanThrowNow).
	 * 장전 입력 바인딩은 사용자 몫이다(이 API를 호출).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void EnterLoaded();

	/** Loaded(장전) 중 로프 튜브 표시를 설정한다. Loaded 중이면 즉시 반영되고, 그 외 페이즈에서는
	 *  다음 Loaded 진입부터 적용된다(전개 상태의 가시성은 건드리지 않는다). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetShowRopeWhenLoaded(bool bShow);

	/** Loaded 로프 표시를 뒤집는다(입력 한 키에 물리는 용도). 반환값 = 뒤집은 뒤의 값. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool ToggleShowRopeWhenLoaded();

	/** Loaded 중 로프 튜브를 보이도록 설정돼 있는가. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsShowRopeWhenLoaded() const { return bShowRopeWhenLoaded; }

	/** 지금 이 로프에 throw가 성립하는가(모드 × 현재 phase). ③은 Loaded에서만, ①②는 항상 true.
	 *  던지기 진입과 조준 HUD가 공유하는 게이트다. 게임 규칙(스태미나 등)은 별개 — Wielder의 CanThrow(). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool CanThrowNow() const { return RopeWrapModes::CanThrowInPhase(ResolveMode, Phase); }

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
		OutDirection = PullDrive.LastPullSample.Direction;
		OutTension = PullDrive.LastPullSample.Tension;
		return PullDrive.LastPullSample.bValid;
	}

	/**
	 * 이번 프레임 로프가 팽팽(taut)한가 — 능동 Pull 게이트와 같은 판정: 전 체인 기하(코너-다리 chord 합 vs
	 * 자유 구간 rest 길이, HoldConfig.TautSlackRatio) ∧ 장력 임계(Pull 샘플 장력 vs
	 * HoldConfig.ActivePullTautTension), 둘 다 히스테리시스 포함. Wrapped 동안 매 프레임 갱신되며 그 외
	 * phase는 false. bActivePullRequiresTaut=false여도 판정 자체는 계속 갱신된다 — 애니메이션 pull window/
	 * BP가 "지금 당겨도 되는 구간인가"를 물을 때 이 하나를 읽는다.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsPullTaut() const { return PullDrive.bPullTaut; }

	/**
	 * 이번 프레임 전 체인이 팽팽한가 — 세 관측치의 AND(모두 히스테리시스 포함): 처짐(다리별 chord 직선
	 * 이탈 cm vs HoldConfig.TautMaxSag — "시각적 펴짐"의 정본) ∧ 기하(코너-다리 chord 합 vs 자유 구간
	 * rest 길이, HoldConfig.TautSlackRatio — 대형 처짐/압축 백스톱) ∧ 최소 전달 장력(자유 구간 세그먼트
	 * 장력 최솟값 vs HoldConfig.TautMinTension — 지그재그 구김/부분 스트레치 거름). 견인(테더 + 능동 Pull)
	 * 의 공용 선행 조건 — 테더 overshoot는 슬랙 체인에서도 sub-leg 스트레치로 >0일 수 있으므로,
	 * overshoot 소비자는 이 게이트를 함께 봐야 한다. IsPullTaut = 이 값 ∧ 앵커 인접 장력 임계.
	 * Wrapped 동안 매 프레임 갱신되며 그 외 phase는 false.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsChainTaut() const { return PullDrive.bChainTaut; }

	/** 이번 프레임 테더 초과분(cm): 손~앵커 직선 거리 - 가용 로프 길이(0 미만은 0). Wrapped 동안
	 *  매 프레임 산출된다(테더 off여도 계산). wielder 견인/지상 이탈 판정 등 게임 반응용. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetTetherOvershoot() const { return PullDrive.LastTetherOvershoot; }

	/** 이번 프레임 실제 사용된 테더 대상 몫(shareT) [0..1]. 자동(질량 기반)/수동 공통 최종값 —
	 *  1이면 wielder 몫 0(전량 대상), 0이면 전량 wielder. wielder 견인 활성 판정/디버그용. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetEffectiveTetherTargetShare() const { return PullDrive.LastTargetShare; }

	/**
	 * (Constraint 테더 모드) 이번 프레임 테더 장력 = λ/dt(kg·cm/s² — HoldConfig.MaxTetherTension과 같은
	 * 단위계라 직접 비교 가능). 슬랙/비Constraint 모드/비Wrapped면 0. 절단·연출 임계 판정과 디버거의
	 * 관측치. (레거시 장력 관측 GetMaxTension은 XPBD 세그먼트 λ 유래로 단위계가 다르다 — 혼용 금지.)
	 */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetTetherTension() const
	{
		return (PullDrive.LastTetherLambdaDt > 1e-4f) ? (PullDrive.LastTetherLambda / PullDrive.LastTetherLambdaDt) : 0.0f;
	}

	/**
	 * 끌림 가능 판정(능동 Pull climb-in 방향의 정본) — 순수 함수(UObject 무의존, 유닛 테스트 가능).
	 * 대상 유효질량 EffMassTarget ≤ wielder 유효질량 EffMassWielder이면 "끌림 가능". bPrev(직전 sticky
	 * 판정)에서 뒤집으려면 반대편 질량이 MarginRatio(≥1)배만큼 더 커야 한다(경계 flapping 방지).
	 * 무한질량(앵커)은 +BIG_NUMBER로 넘긴다(무한 대상 = 끌림 불가, 무한 wielder = 대상 끌림 가능).
	 */
	static bool DecideTargetPullable(float EffMassTarget, float EffMassWielder, bool bPrev, float MarginRatio);

	/**
	 * 능동 Pull(당김) 힘 설정 — Wrapped + 로프가 팽팽할 때 매 프레임 이 크기의 *상수* 힘을 감긴
	 * 대상에 인가한다(장력과 무관 → 피드백 폭주 없음). 0 = 정지. 입력 홀드 동안 켜고 떼면 끄는
	 * 용도(URopeWielderComponent의 PullAction이 이걸 호출). 캐릭터 대상은 CharacterMovement가
	 * 질량으로 나누고 지면 마찰과 경쟁하므로 수만~수십만 단위가 체감 구간이다.
	 * 팽팽 판정/게이트는 HoldConfig(bActivePullRequiresTaut/ActivePullTautTension) — IsPullTaut()로 조회.
	 * bIgnoreTautGate=true면 이번 Pull은 config와 무관하게 팽팽함을 무시하고 인가한다(per-call 우회 —
	 * 애니 pull window의 "팽팽 무시" 구간용, UAnimNotifyState_RopePull이 넘긴다).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetActivePull(float Force, bool bIgnoreTautGate = false);

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
	bool IsSleeping() const { return Throttle.IsAsleep(); }

	/** 현재 거리 LOD의 iteration 배율(1=풀 품질). 디버그/프로파일 확인용. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetSolverLODScale() const { return Throttle.GetSolverLODScale(); }

	/** 이번 프레임 이 로프가 GPU로 디스패치됐는가. **물리 솔브만을 뜻하지 않는다** — 서브시스템은
	 *  솔브 프레임과 override-only 프레임(Wrapping/Releasing/GuidedThrow)을 똑같이 GPU에 싣는다
	 *  (TryBuildResidentStep의 bSolveThisFrame || OverrideFrame.HasAny()). 따라서 이 값 하나로
	 *  "GPU 솔브 중"이라고 읽으면 안 되고, 아래 두 게터와 조합해야 한다. 디버그 확인용. */
	bool IsGpuSteppedThisFrame() const { return SimFrame.bGpuSteppedThisFrame; }

	/** 이번 프레임 이 로프가 (CPU/GPU 무관) 실제로 물리 솔브 스텝을 밟았는가. 슬립·Contacting·Releasing·
	 *  로직 override-only 프레임은 false. IsGpuSteppedThisFrame()과 조합하면 CPU 폴백 솔브를 가려낸다
	 *  (WasSolvedThisFrame() && !IsGpuSteppedThisFrame()). 디버그/프로파일용. */
	bool WasSolvedThisFrame() const { return SimFrame.bSolveThisFrame; }

	/** 이번 프레임 로직 페이즈(Wrapping/Wrapped/Releasing/GuidedThrow 등)가 노드 override를 산출했는가
	 *  = 솔브는 안 밟았지만 위치가 갱신된 프레임. 위 두 게터와 합쳐 솔브 경로를 6종으로 가른다:
	 *  SLEEP / GPU_SOLVE / GPU_OVERRIDE / CPU_SOLVE / CPU_OVERRIDE / IDLE. GPU 경로는
	 *  IsGpuSteppedThisFrame()만으로 override가 드러나지만, CPU 경로에서 override와 idle을 구별하려면
	 *  이 값이 필요하다. 디버그/프로파일용. */
	bool HadLogicOverrideThisFrame() const { return SimFrame.OverrideFrame.HasAny(); }

	/** 현재 감고 있는 본 이름(Wrapped 동안 유효, 아니면 None). 이벤트 파라미터 없이도 조회 가능하게 노출. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	FName GetWrappedBoneName() const { return WrapController.State.BoneName; }

	/** 현재 감고 있는 원본 컴포넌트(스켈레탈/정적 공용). Wrapped가 아니거나 대상이 소실되면 null. */
	const USceneComponent* GetWrappedComponent() const { return WrapController.State.Mesh.Get(); }

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

	// bSyncTipMeshOnFree=false일 때 Free 배치를 게임 코드가 직접 구동하기 위한 확장점 — 그 경우
	// 로프는 Free 동안 이 컴포넌트의 트랜스폼을 건드리지 않는다.

	/** 팁 부착물 컴포넌트(팁을 안 쓰거나 확보 실패면 null). */
	UFUNCTION(BlueprintPure, Category = "Rope|Tip")
	UStaticMeshComponent* GetTipMeshComponent() const { return TipMeshComponent; }

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

	/** ApplyPreset 성공 직후 발화(거부 시 미발화). Wielder가 모드 유도 상태(preview/틱) 재동기화에
	 *  구독하고, 게임 코드도 프리셋 전환 반응(UI 갱신 등)에 쓸 수 있다. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnPresetApplied OnPresetApplied;

private:
	/**
	 * 시뮬레이션 한 프레임을 3단계로 나눠 URopeSimSubsystem이 구동한다(friend 접근;
	 * 컴포넌트는 직접 tick하지 않고, 외부 게임 코드가 부를 일도 없어 private).
	 *
	 * 분리 계약 — 가운데 Solve가 로프 간 병렬(CPU) 또는 GPU 디스패치라 갈라진 것이며 임의 분류가 아니다.
	 * "무엇이 어느 쪽인가"의 판정 기준은 딱 하나: 솔브 결과가 필요한가.
	 *  Prepare(GT)  : 솔브 *입력* 생산 — pin 타깃 전진, whip 타깃 계산, 로직 페이즈(Contacting/Wrapping/
	 *                 Wrapped/Releasing) 처리 + OverrideFrame 산출, bSolveThisFrame 결정. 솔브 결과가
	 *                 필요 없는 로직은 전부 여기다(솔브 전 UObject/이벤트를 만질 수 있는 마지막 지점).
	 *                 collider 스냅샷(FrameColliders)은 서브시스템이 이 호출 전에 중앙 수집해 둔다.
	 *  Solve(병렬)  : POD(Sim) + const collider만 — bSolveThisFrame(Free/Flight/Wrapping/Wrapped)일 때
	 *                 Solver.Step. UObject/이벤트/전이 금지(스레드 안전 경계). Wrapped는 latch 노드가
	 *                 InvMass=0이라 자유 구간만 물리로 움직인다.
	 *  Finalize(GT) : 솔브 *출력* 소비 — Flight 접촉 감지는 노드 이동 경로(Prev→Pos), 즉 솔브 산출물이
	 *                 입력이라 여기 있을 수밖에 없다("로직은 Prepare, 감지만 Finalize"인 비대칭의 근거).
	 *                 전이/이벤트 브로드캐스트 + 렌더 push + 관측(스탯/디버거 스냅샷)도 여기.
	 */
	void PrepareSimFrame(float DeltaTime, const TOptional<FVector>& LODCameraLocation);
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

	//~ Loaded(장전) 연출 훅 — 전부 게임 스레드(콜드 패스). 기본 구현을 override해 연출을 커스텀한다.

	/** Loaded 중 창(팁)을 놓을 월드 트랜스폼. 기본: Owner 스켈레탈 메시의 LoadedHandSocket 소켓(없으면 컴포넌트 트랜스폼).
	 *  ⚠ 전이당이 아니라 **Reel인 동안 매 프레임 2회** 불린다(노드 구동 + 팁 메쉬 배치) — 무거운 계산은 캐시할 것. */
	virtual FTransform GetLoadedTipTransform() const;

	/** Loaded 진입 **에지에서만** 1회(이미 Reel일 때 EnterLoaded()을 다시 불러도 재발화하지 않는다 —
	 *  프리셋 적용이 ③ 로프에 EnterLoaded()을 무조건 호출하기 때문). 기본: bShowRopeWhenLoaded에 따라
	 *  로프 튜브 렌더를 켜거나 끈다. OnDeployFromLoaded과 1:1로 짝지어진다. */
	virtual void OnEnterLoaded();

	/** Reel을 벗어나는 순간 1회(throw 성립 또는 프리셋으로 ①②가 될 때). 기본: 로프 튜브를 다시 표시하고
	 *  전체 길이(RopeLength)를 복원한다.
	 *  ⚠ 호출 시점의 GetPhase()는 **아직 Loaded**이다(페이즈 전이는 이 훅 뒤에 일어난다). */
	virtual void OnDeployFromLoaded();

	/**
	 * wrap 대상 게이트. false면 그 (Mesh, Bone) 후보는 없는 것으로 취급된다 — 팀/태그 등 게임 규칙으로
	 * 감을 수 있는 대상을 제한할 때 오버라이드. 기본 true(모두 허용).
	 *
	 * **대상을 고르는 모든 경로가 이 게이트 하나를 공유한다** — 조준/preview/판정이 갈리면 "조준은
	 * 거부했는데 preview는 고르는" 불일치가 된다. 호출 지점:
	 *   - Flight 후보 산출(프레임마다·후보별) + Contacting 재수집 — RemoveNonWrappableCandidates
	 *   - 조준 ray 질의 — FRopeAimTargeting::FindAimRayBoneHit
	 *     (금지 대상은 blocked로 잡힌다 = aim hit 없음)
	 *   - preview arc 탐색 — FRopeThrowPreviewBuilder::FInput::CanWrapTarget 주입
	 *     (빌더가 UObject-free라 virtual을 직접 못 부르므로 호출자가 람다로 넣어준다)
	 *   - prepared throw 진입 — ThrowWithPreparedPreview (마지막 방어선)
	 * 새 대상 선택 경로를 추가하면 이 게이트도 함께 태울 것.
	 */
	virtual bool CanWrapTarget(const USceneComponent* Mesh, FName Bone) const { return true; }

	//~ 이벤트 네이티브 훅: 각 델리게이트 브로드캐스트 직전에 호출(엔진 Notify 관례). C++ 서브클래스가
	//  자기 델리게이트에 바인딩하는 우회 없이 반응할 수 있다.
	virtual void NotifyCaptured(FName Bone) {}
	virtual void NotifyWrapped(const FRopeWrappedEventInfo& Info) {}
	virtual void NotifyReleased(FName Bone, ERopeReleaseReason Reason) {}
	/** ApplyPreset 성공 직후, OnPresetApplied 브로드캐스트 직전 호출(GT, 콜드 패스 — 적용당 1회). */
	virtual void NotifyPresetApplied(const URopePreset* Preset) {}

	/**
	 * ③ 연출(GuidedThrow) 중 매 프레임 호출되는 인터럽트 판단 훅(GT, 콜드 패스 — 연출은 ~0.2초).
	 * 기본은 항상 false = "그래도 보장"(2026-07-13 회의 결정 G). 대상 사망/텔레포트 같은 게임 규칙으로
	 * 보장을 깨야 하면 오버라이드해 true 반환 — 연출이 중단되고 OnRopeReleased(ThrowAborted)가 발화한다
	 * (내부 실패는 Broken이라 소비자가 구분할 수 있다). 대상 mesh 소실은 훅과 무관하게 항상 중단된다.
	 * **조준 던지기에서만 폴링된다** — 허공 던지기(대상 없음)는 깰 보장이 없고 Prepared가 stub이라
	 * 부르지 않는다. 정책 훅이라 C++ 전용이다(반응은 OnRopeReleased로 BP에서).
	 */
	virtual bool ShouldAbortGuaranteedThrow(const FRopePreparedThrowPreview& Prepared) const { return false; }

	/**
	 * throw 컨텍스트 최종 해석 — **던지기 컨텍스트를 손댈 수 있는 유일한 확장 훅**(throw당 1회).
	 * 프레임을 정규직교(오른손계)로 재구성(Forward 기준, Up 직교화, Right = Up×Forward 재유도 —
	 * 입력 Right 무시), 속도·원점 폴백. 에임 어시스트 등 커스텀 지점(오버라이드하면 프리뷰와 실제
	 * 던지기가 자동으로 일치한다).
	 *
	 * **모든 던지기가 이 관문을 지난다** — 컨텍스트 생산자가 무엇이든(Throw 편의 진입점의
	 * FRopeThrowContext::MakeDefault / URopeWielderComponent::BuildThrowContext / BP 직접 호출)
	 * 여기로 수렴한다. ③ prepared 경로는 preview 빌드 시점에 한 번 지나고 그 결과를 재사용한다.
	 * 과거엔 "조준 규약을 바꾸는" 훅이 MakeDefaultThrowContext에도 있었으나, Wielder 경로가 자체
	 * 컨텍스트를 만들어 그 훅을 지나지 않아 오버라이드해도 무효였다 → 훅을 이쪽 하나로 일원화했다.
	 *
	 * **오버라이드는 순수(pure)해야 한다** — 같은 입력에 항상 같은 출력, 상태 변경 없음. ③은 preview
	 * 빌드 시점에 해석한 컨텍스트를 던지기가 그대로 재사용한다. 여기서 난수(조준 산포 등)를 쓰면
	 * 반복 preview 질의 사이의 결과가 달라지거나 preview가 보여준 궤적과 실제 던지기가 갈라진다.
	 */
	virtual FRopeThrowContext ResolveThrowContext(const FRopeThrowContext& ThrowContext) const;

	/**
	 * Pull 힘 인가(Wrapped + 팽팽 + 능동 Pull 활성인 프레임마다). 기본 수신자 체인:
	 * 물리 시뮬 본 → CharacterMovement → 물리 시뮬 루트. 커스텀 무브먼트(Mover 등)/탈것/특수 대상은 오버라이드.
	 * Force = 당김 방향 × 최대 장력(|Force| = 장력 상한). 물리 바디는 장력 상한 속도 드라이브로 인가한다
	 * (ApplyPullVelocityDrive). DeltaTime은 임펄스 상한(장력×dt) 산정에 쓴다.
	 *
	 * 이건 능동 Pull *정책*(무엇을 얼마나 당기나) 훅이다. 수신자 단위로 가로채려면 아래
	 * ApplyTractionToReceiver를 쓸 것 — 이 함수의 기본 구현도 실제 인가 직전 그 관문을 지난다.
	 */
	virtual void ApplyPullForce(const FVector& Force, const FRopePullSample& Pull, float DeltaTime);

	/**
	 * 로프가 수신자에 견인을 인가하기 직전의 **단일 관문**(GT, 프레임당 최대 수 회 — 콜드 패스).
	 * true를 반환하면 "서브클래스가 처리했다"로 보고 기본 인가를 생략한다. 기본 false = 내장 인가.
	 *
	 * **로프가 만드는 모든 힘/속도 개입이 여기를 지난다** — 자동 테더(양끝), 능동 Pull(대상), climb-in
	 * (wielder), 슬랙 브레이크. 그래서 커스텀 무브먼트(Mover 등)·탈것·특수 수신자는 이것 하나만
	 * 오버라이드하면 로프 견인 전부를 자기 이동 시스템으로 가져갈 수 있다.
	 *
	 * 과거엔 능동 Pull만 훅(ApplyPullForce)이 있고 테더/climb-in/슬랙 브레이크는 수신자에 직접
	 * 임펄스·속도를 꽂아, ApplyPullForce를 오버라이드해도 테더가 그대로 밀어붙이는 상태였다.
	 *
	 * Request.Amount의 단위는 Request.Source마다 다르다(FRopeTractionRequest 주석 참고).
	 * true를 반환해도 로프 내부 관측/상태 갱신은 동일하게 일어난다 — 서브클래스가 처리 여부를 바꿔도
	 * 로프 상태가 갈라지지 않게 하기 위함이다.
	 */
	virtual bool ApplyTractionToReceiver(const FRopeTractionRequest& Request) { return false; }

	// 시뮬 상태 읽기 전용 접근(서브클래스용). 변경은 공개 API(Throw·Set 계열)를 통해서만.
	const FRopeSimState& GetSimState() const { return Sim; }

private:
	//~ 팁 부착물 런타임 상태 -----------------------------------------------
	// UObject라 값 타입 sim 멤버와 달리 GC 추적이 필요하다(Transient UPROPERTY).
	// 던지기 진입에 EnsureTipMesh가 확보하고, FinalizeSimFrame이 매 프레임 자유단으로 추종시킨다.
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> TipMeshComponent = nullptr;

	// 우리가 스폰했는가 — EndPlay에서 스폰분만 파괴하기 위한 소유권 플래그(외부 컴포넌트 보호).
	bool bTipMeshSpawnedByUs = false;

	// 태그로 재사용한 팁 StaticMeshComponent의 기존 월드 스케일. SetWorldTransform으로 덮어도 비주얼 크기를 보존한다.
	FVector TipMeshAuthoredScale = FVector::OneVector;

	// 태그 재사용 팁의 획득 시점 상대 트랜스폼(저작 원본). 해제(Teardown) 시 이 값으로 복원해, 재획득
	// (프리셋 전환)이 매 프레임 배치가 덮어쓴 트랜스폼을 저작 기준선으로 오캡처하는 것(스케일 누적 오염)을 막는다.
	FTransform TipMeshAuthoredRelative = FTransform::Identity;

	// 태그 재사용 팁의 획득 시점 충돌 설정(저작 원본). bTipMeshCollision=false로 껐다가 Teardown에서
	// 이 값으로 되돌린다(외부 컴포넌트 소유 존중). 스폰분에는 무의미(우리가 만든 것).
	TEnumAsByte<ECollisionEnabled::Type> TipMeshAuthoredCollision = ECollisionEnabled::QueryAndPhysics;

	// 존재 여부만 필요한 분기가 socket transform까지 읽지 않도록 분리한 경량 질의.
	bool HasTipSocket(FName Socket) const;

	// 팁 부착물을 BeginPlay~EndPlay 단위로 확보/파괴/추종한다(bUseTipMesh가 켜진 경우만 동작).
	void EnsureTipMesh();
	void TeardownSpawnedTipMesh();
	void UpdateTipMeshTransform();

	// bTipMeshCollision을 현재 팁 컴포넌트에 반영한다(팁이 없으면 no-op). 확보 시점과 편집 시점이 호출.
	void ApplyTipMeshCollision();

	//~ Pierce 임베드(소켓 기반) 헬퍼 -------------------------------------------
	// 소켓 배치 활성 조건의 **단일 소스** = 팁 사용 + 소켓 옵트인 + ③(Guaranteed). Head를 꽂힘 지점에
	// 맞춘다는 개념이 ③에만 있으므로, ①②는 소켓 이름이 채워져 있어도 읽지 않는다
	// (세그먼트 추종으로 통일). HasTipSocket이 이 술어를 태우므로 소켓 경로 전체가 함께 꺼진다.
	bool IsTipSocketPlacementActive() const
	{
		return bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap;
	}
	// 팁 StaticMesh의 소켓을 컴포넌트-로컬 트랜스폼으로 읽는다. 소켓 배치가 비활성이거나 소켓이 없으면
	// false(호출부가 폴백) — 소켓 읽기의 유일한 관문이다.
	bool ReadTipSocketLocal(FName Socket, FTransform& OutLocal) const;
	// 태그 컴포넌트의 기존 스케일과 TipMeshRelativeTransform을 함께 담은 팁 배치 로컬.
	FTransform MakeTipPlacementTransform() const;
	// 최종 팁 배치까지 포함한 "배치 기준" 소켓 로컬. 실제 SetWorldTransform은 MakeTipWorldTransform(BaseWorld)를 쓴다.
	FTransform MakeTipPlacementSocketLocal(const FTransform& SocketLocal) const;
	FTransform MakeTipWorldTransform(const FTransform& BaseWorld) const;
	// 로프 연결점(꼬리 소켓, 없으면 메쉬 원점)이 RopeAttachWorld에 오도록 팁 메쉬 기준 월드 트랜스폼을 역산한다.
	void ComputeTipFollowTransform(const FVector& RopeAttachWorld, const FVector& ForwardDir,
		FTransform& OutComponentWorld) const;
	// 주어진 기준 월드 트랜스폼에서 로프가 붙어야 할 실제 월드 위치(꼬리 소켓, 없으면 메쉬 원점)를 얻는다.
	FVector ResolveTipRopeAttachWorld(const FTransform& ComponentWorld) const;
	// owner-local로 보관된 prepared path를 throw 시점 월드 스냅샷으로 확정한 뒤 Pierce 소켓 목표를 반영한다.
	void ApplyPierceSocketTargetsToPrepared(FRopePreparedThrowPreview& InOutPrepared) const;
	// Prepared의 단일 Pierce 앵커에서 현재 월드 히트점을 복원한다(가능하면 bone-local 앵커 기준).
	bool ResolvePreparedPierceHitPoint(const FRopePreparedThrowPreview& Prepared, FVector& OutHitPoint) const;
	// 팁 소켓을 HitPoint에 두고 Tail->Head 소켓 벡터가 관통 방향(PierceDir)을 보도록 메쉬 원점(컴포넌트) 월드
	// 트랜스폼을 역산한다. 꼬리 소켓이 있으면 로프 연결점(월드)도 함께 낸다(없으면 메쉬 원점).
	// TipSocketName 소켓이 없으면 false(Pierce 임베드 비활성). 순수 배치 수학은 FRopeTipPlacement가 소유한다.
	bool ComputePierceEmbed(const FVector& HitPoint, const FVector& PierceDir,
		FTransform& OutComponentWorld, FVector& OutTailWorld) const;

	//~ 페이즈 상태 머신 ----------------------------------------------------
	ERopePhase Phase = ERopePhase::Free;
	// Prepare 도중 Contacting 등에서 Flight로 돌아온 프레임은 Flight의 Advance/Solve를 거치지 않았다.
	// Finalize 접촉 감지를 한 프레임 미뤄 stale guide 후보로 즉시 재캡처되는 것을 막는다.
	bool bEnteredFlightDuringPrepareThisFrame = false;

#if WITH_GAMEPLAY_DEBUGGER
	// 프레임 시작 시점 phase. 전이는 프레임 곳곳에서 일어나 프레임 끝의 Phase만으로는 "무엇에서 무엇으로
	// 갔는지"를 알 수 없으므로, 디버그 스냅샷이 전이 전후를 함께 담도록 보존한다.
	ERopePhase DebugPhaseAtFrameStart = ERopePhase::Free;
	// 위 값을 기록한 프레임(GFrameCounter). 프레임당 최초 1회만 쓰기 위한 것 — 서브시스템은 Prepare보다
	// **앞서** ResolvePendingAimThrow를 돌리고 그 경로가 StartFreshThrow로 Flight 전이를 만들 수 있어,
	// Prepare에서만 잡으면 그 전이가 이미 지나가 버린다.
	uint64 DebugPhaseFrameStamp = 0;
	// 이번 Wrapped 프레임에 능동 Pull이 팽팽 게이트를 통과해 실제로 인가됐는가. SetActivePull이 저장한
	// 요청값만 보면 "입력은 있으나 게이트에 막힌" 프레임과 구분되지 않는다.
	bool DebugActivePullPassedGate = false;

public:
	/** 이 프레임 첫 접점에서 프레임 시작 phase를 굳힌다(프레임당 1회, 이후 호출은 no-op).
	 *  서브시스템이 로프를 건드리기 전에 부르고, 그 경로를 타지 않는 로프를 위해 Prepare에서도 부른다. */
	void CaptureDebugFrameStartPhase()
	{
		if (DebugPhaseFrameStamp != GFrameCounter)
		{
			DebugPhaseFrameStamp = GFrameCounter;
			DebugPhaseAtFrameStart = Phase;
		}
	}

private:
#endif

	/**
	 * Phase 대입의 단일 지점. 전이 로그("[이름] Old -> New (Reason)")를 일원화한다.
	 * Reason은 로그용 부가 설명(nullptr이면 생략). 전이에 딸린 이벤트 브로드캐스트와
	 * cleanup은 전이마다 다르므로 호출자가 결정한다 — 여기서 암묵적으로 하지 않는다.
	 */
	void SetPhase(ERopePhase NewPhase, const TCHAR* Reason = nullptr);

	/**
	 * 페이즈 전이 시 함께 폐기해야 하는 "진행 중 작업" 일시 상태 세트를 리셋한다:
	 * ContactTracker / PendingWrapSeed / CaptureTravelFrame / WrappingPhase.State / ContactingElapsed /
	 * FlightNoContactElapsed / TensionOverTime.
	 * 유휴 상태의 멤버에 대해서는 no-op이라 어떤 전이에서 불러도 안전하다.
	 * (ReleaseCooldown은 전이마다 값이 달라 호출자가 직접 설정한다.)
	 */
	void ResetTransientPhaseState();

	// Aim-ray 조준 로직/상태는 FRopeAimTargeting(AimTargeting 멤버)으로 분리됐다. 여기엔 서브시스템
	// 프레임 계약 진입점만 남는다 — StartFreshThrow 전이(오케스트레이션)와 SimFrame 접근이 걸려
	// 있어 컴포넌트가 소유한다.
	// Subsystem이 AimFrameColliders를 채운 직후 HUD/preview pending query를 결과 캐시로 확정한다.
	void ResolvePendingAimQuery();
	// Guaranteed 입력 요청을 같은 조준 목록으로 prepared path까지 확정하고, 실행 대기 상태면 즉시 던진다.
	void ResolvePendingGuaranteedAimThrow();
	// Subsystem이 FrameColliders를 채운 직후 호출해 pending request를 hit/fallback context로 확정한다.
	void ResolvePendingAimThrow();
	// aim ray throw가 지정한 mesh+bone만 contact/wrap 후보로 유지한다.
	// collision-free Aim Flight에서는 solver가 이 목록을 의도적으로 무시하지만, 실제/예측 contact와
	// wrapping path는 필터된 목록을 계속 사용한다. 일반 Flight solver도 같은 목록을 사용한다.
	void FilterFrameCollidersForAimWrapTarget();
	/** FRopeAimTargeting 질의에 넘길 컨텍스트 스냅샷(collider 스냅샷 + 폴백 치수). */
	FRopeAimTargeting::FQueryContext MakeAimQueryContext() const;
	/**
	 * 조준 계열 질의(aim ray hit, GuaranteedWrap preview 아크 탐색)가 쓸 collider 목록.
	 * 조준 중이면 조준 전용 스냅샷(로프 AABB ∪ ray 영역 — 물리 목록엔 원거리 대상이 안 들어온다),
	 * 조준 중이 아니면 물리 스냅샷. 후자는 Wielder 조준 흐름 없이 Throw()가 직접 불린 BP/AI 경로다 —
	 * ray region이 없으니 로프 주변 목록이 유일한 소스다.
	 */
	const TArray<IRopeCollider*>& GetAimQueryColliders() const;

	//~ 시뮬레이션 상태 + 페이즈별 로직 소유물 -------------------------------
	// Non-UObject — 값으로 소유하며 GC 추적 대상이 아니다(POD/약참조만 보유).
	// 아래 로직 4개는 로프 수명 순서와 1:1 대응한다: Throw/Flight → Contacting → Wrapping → Wrapped.
	/** 단일 진실: 솔버/로직/렌더가 공유하는 파티클 체인. */
	FRopeSimState       Sim;

	/** XPBD 물리(Free/Flight 및 Wrapping/Wrapped의 solver-owned 자유 구간). */
	FRopeXPBDSolver     Solver;

	/** Throw/Flight: 채찍 스윙(가이드 타깃 계산+적용). */
	FRopeWhipGuide      WhipGuide;

	/** Flight/Contacting: 접촉 후보의 dominant bone 추적. */
	FRopeContactTracker ContactTracker;

	// CPU Flight fallback과 Contacting 재검출이 번갈아 쓰는 후보 저장소. 감지기는 append하므로
	// 각 경로가 사용하기 직전에 Reset한다. GPU Flight는 SimFrame.GpuFlightCandidates를 직접 소비한다.
	TArray<FRopeContactCandidate> ContactCandidateScratch;

	// CPU Flight predictive contact의 다음 프레임 whip 타깃. GPU 경로는 subsystem이 dispatch payload에
	// 별도 소유 배열을 실으므로 이 scratch를 사용하지 않는다.
	TArray<FVector> NextGuideTargetScratch;

	/** Contacting: 캡처 시 만들어 둔 wrap 시드(Wrapping 진입 재료). */
	FRopeWrapState      PendingWrapSeed;
	/** GPU Flight 캡처 직후 pending RT step과 CPU Sim을 아직 권위 있게 맞추지 못한 상태. 이 동안
	 *  Contacting의 dwell/dismiss/seed 판정을 전부 보류하고 SyncGpuPositionsForHandoff를 재시도한다. */
	bool bPendingGpuCaptureHandoff = false;

	/** Contacting~Wrapping: 캡처 순간의 로프 진행 좌표계 스냅샷(속도/누운 방향/진행 평면 normal —
	 *  Contacting부터는 노드가 정지해 이 순간에만 잴 수 있다). CaptureTravelPlane 축의 가이드 평면 폴백. */
	FRopeCaptureTravelFrame CaptureTravelFrame;

	/** Wrapping: 경로 점진 생성+front 모션+마스크(작업 상태는 .State). */
	FRopeWrappingPhase  WrappingPhase;

	/** Wrapped: bone-local latch 유지/해제. */
	FRopeWrapController WrapController;

	/**
	 * Composite Analytic Helix의 bounded no-anchor 구간을 양쪽 실제 anchor 사이 직선으로 유지하는
	 * 컴포넌트 전용 런타임 상태. 특정 본 하나에 귀속하지 않고 두 surface binding을 매 프레임 함께 해석한다.
	 */
	struct FKinematicVirtualBridge
	{
		/** 양쪽 실제 표면점 사이에서 SDF projection에 실패해 anchor가 없는 내부 노드들. */
		TArray<int32> NodeIndices;
		/** 매 프레임 bone-local binding으로 다시 해석할 왼쪽/오른쪽 실제 표면 anchor. */
		FRopeSurfaceAnchor LeftAnchor;
		FRopeSurfaceAnchor RightAnchor;
		/** 양 끝 노드를 포함한 원래 세그먼트 수 × SegmentLength. 과도한 직선 신장 진단 기준이다. */
		float RestSpanLength = 0.0f;
		/** Wrapping front가 오른쪽 실제 anchor까지 도달했을 때 bridge를 켜기 위한 경로 거리. */
		float ActivationFrontDistance = 0.0f;
		/** 커밋 전 점진 등록된 bridge는 false로 대기하고, 양쪽 실제 anchor가 고정된 순간 true가 된다. */
		bool bActive = true;
		/** 같은 bridge의 과신장 경고가 매 프레임 반복되지 않도록 하는 1회성 로그 래치. */
		bool bLoggedStretchWarning = false;
	};

	/** 양쪽 실제 anchor가 있는 virtual run. Wrapping 중 front 도달 뒤부터 Wrapped까지 직선 고정된다. */
	TArray<FKinematicVirtualBridge> KinematicVirtualBridges;
	/** WrappingPhase가 한 번 산출한 run 중 component bridge로 동기화한 prefix 길이. */
	int32 KinematicVirtualBridgeRunCursor = 0;

	/** ③ GuidedThrow 구동 상태: 확정 preview path(조준) 또는 레이 끝점 아치(허공, bFreeThrow). */
	FRopeGuidedThrowState GuidedThrowState;

	// Flight 시작 때 확정된 whip guide spline 평면 normal. Contacting을 거쳐 Wrapping에 들어갈 때
	// bone 위치에 세운 가상 wrapping axis의 방향으로 재사용한다.
	bool bHasFlightGuidePlaneNormal = false;
	FVector FlightGuidePlaneNormal = FVector::RightVector;

	// Aim-ray 조준 상태(throw당 wrap 대상 잠금 + pending HUD/preview query/result + aim throw 큐).
	// 질의/잠금 판정 로직 포함 —
	// FRopeAimTargeting(Logic/RopeAimTargeting.h) 주석 참조.
	FRopeAimTargeting AimTargeting;

	/** 입력 ray를 정상 gather에서 한 번 확정한 뒤 즉시 실행하거나 몽타주 notify까지 보관하는 ③ 전용 상태. */
	struct FPendingGuaranteedAimThrow
	{
		FRopeAimRayThrowRequest Request;
		FRopeThrowContext ResolvedContext;
		FRopePreparedThrowPreview Prepared;
		bool bQueued = false;
		bool bResolved = false;
		bool bExecuteWhenReady = false;

		void Reset() { *this = FPendingGuaranteedAimThrow(); }
	};
	FPendingGuaranteedAimThrow PendingGuaranteedAimThrow;

	/** 확정된 ③ 요청을 재질의 없이 prepared 또는 입력 시점 free-arc로 실행하고 완료/거부 콜백을 보낸다. */
	bool ExecutePendingGuaranteedAimThrow();

	//~ 페이즈 타이머 --------------------------------------------------------
	/** Contacting 체류 시간(WrapDecisionTime 판정). */
	float ContactingElapsed = 0.0f;

	/** Whip 종료 후 캡처 없이 Flight에 머문 시간. */
	float FlightNoContactElapsed = 0.0f;

	/** Releasing → Free 복귀까지 남은 시간. */
	float ReleaseCooldown = 0.0f;

	/** Wrapped 중 최대 장력이 TensionReleaseForce를 연속 초과한 시간. */
	float TensionOverTime = 0.0f;

	// Wrapped 견인/스무딩 상태 묶음(Pull 샘플/EMA 3종/능동 Pull/테더 초과분/경고 래치). 멤버별 의미와
	// 전이 시 리셋 규약(무엇이 살아남는가)은 FRopePullDriveState(Core/RopePullDriveState.h) 주석 참조.
	FRopePullDriveState PullDrive;
	FRopeResolvedWrappedEndpoints WrappedEndpointCache;

	// 되감기 속도(cm/s, +감기/-풀기, 0=정지). SetReelRate가 설정, UpdateReel이 프레임마다 적용.
	float ReelRate = 0.0f;

	// 되감기 프레임 적용(Prepare 초입): 허용 페이즈에서 ReelRate × dt만큼 길이를 조정한다.
	void UpdateReel(float DeltaTime);

	//~ 슬립/LOD(스케일링) ---------------------------------------------------
	// 상태·판정은 FRopeSolverThrottle(Logic/RopeSolverThrottle.h)로 분리 — 컴포넌트에는 카메라 접근(GT)과
	// 슬립 전이 로그만 남는다.
	FRopeSolverThrottle Throttle;

	// 거리 LOD 배율 계산(Prepare, GT): 서브시스템이 프레임당 한 번 구한 카메라 위치를 거리로 바꿔 Throttle에 위임.
	void ComputeSolverLOD(const TOptional<FVector>& CameraLocation);
	// LOD 반영된 유효 iteration(CPU 솔브/GPU 스텝 공용 — 서브시스템이 호출).
	int32 GetLODScaledIterations() const { return Throttle.LODScaledIterations(SolverConfig.Iterations); }

	// 동작 1 — 자동 견인(테더, Docs/PoC/05): 관측(전 체인 C·벌어짐 속도)→λ 솔브→양끝 임펄스 쌍 인가.
	// ApplyWrappedTraction이 매 Wrapped 프레임 호출한다. λ/장력 관측치는 PullDrive.LastTetherLambda(+Dt).
	void UpdateConstraintTether(float DeltaTime);

	// (Constraint 테더 — 랙돌 대상 절반) 엔진 물리 제약: 코너의 키네마틱 프록시 ↔ 감긴 본의 앵커 점을
	// 다리 rest 길이의 구면 리밋으로 묶는다. 대상은 **시뮬 바디 전부**(스켈레탈 본 + 컴포넌트 바디):
	// GT 프레임당 속도 임펄스는 관절체의 "전신 크기 kick → 폭주" vs "본 크기 λ → 견인력 붕괴" 딜레마
	// (2026-07-20 Pierce 실측 반복)에 더해, 공중 하중(매달린 프랍)에서도 구조적으로 진다 — 중력·스윙이
	// 물리 서브스텝에서 진행되는 동안 GT는 한 박자 늦게 사후 상쇄만 하므로 부유·진자 펌핑·직교 감쇠
	// 의존이 생긴다(2026-07-22 PIE). Chaos 제약은 서브스텝에서 중력·관절·접촉과 함께 푼다(Docs/PoC/05
	// §3.4-1·§9). 갱신은 UpdateConstraintTether가 매 Wrapped 프레임, 해체는 phase 전이
	// (ResetTransientPhaseState)/대상·본 변경/EndPlay에서.
	void UpdatePhysicalTether(class UPrimitiveComponent* TargetPrim, FName Bone,
		const FVector& AnchorWorld, const FVector& CornerWorld, float LegRestLen, float DeltaTime);
	void TeardownPhysicalTether();

	/** 물리 제약 테더의 키네마틱 프록시(코너 추종)와 제약 — 런타임 전용, 시뮬 바디 대상에서만 산다. */
	UPROPERTY(Transient)
	TObjectPtr<class USphereComponent> PhysicalTetherProxy;
	UPROPERTY(Transient)
	TObjectPtr<class UPhysicsConstraintComponent> PhysicalTetherConstraint;
	// 제약이 묶은 대상/본(변경 감지 → 재생성)과 현재 리밋(cm — 갱신 스킵용, <0 = 미설정).
	TWeakObjectPtr<class UPrimitiveComponent> PhysicalTetherTarget;
	FName PhysicalTetherBone = NAME_None;
	float PhysicalTetherLimit = -1.0f;
	// 생성 시 고정한 바디-로컬 앵커(제약 Frame2) — wrap 앵커가 같은 (대상,본) 안에서 재배치되면
	// 드리프트를 감지해 재생성하는 가드의 기준값.
	FVector PhysicalTetherAnchorLocal = FVector::ZeroVector;

	// (테더) wielder 견인 방향(손(노드0)→로프 첫 다리 = 앵커 쪽)을 산출해 PullDrive.SmoothedWielderPullDir로
	// EMA 스무딩(PullDirSmoothTime)해 반환 — 방향 지터로 인가 축이 튀는 것을 막는다(180° 반전 축퇴는 raw 재시드).
	// bInstantaneous면 EMA를 생략하고 순간 기하를 그대로 쓴다(공중 스윙 — 궤도 회전을 EMA가 못 따라와
	// 래그 방향의 접선 오차가 스윙 조작을 방해한다; 상태는 계속 시드해 착지 시 EMA 재진입이 연속).
	FVector ComputeSmoothedWielderDir(const FVector& Aim, const FVector& DirToAim, float DeltaTime, bool bInstantaneous);

	// 이번 Wrapped 프레임의 끌림 가능 판정을 overshoot와 무관하게 갱신한다 — 능동 Pull의 climb-in 방향과
	// 분배 관측(LastTargetShare 이진값)이 PullDrive.bTargetPullable을 공유. 양끝 유효질량 비교 + 히스테리시스.
	void UpdateTargetPullable();

	// Wrapped 견인 구간에서 target/wielder를 지연 해석하고 같은 프레임의 판정·테더·기본 Pull이 공유한다.
	const FRopeResolvedWrappedEndpoints* GetOrResolveWrappedEndpoints();

	// (not pullable) 능동 Pull 힘을 wielder(로프 owner)에 인가 — 대상이 무거워
	// wielder가 앵커 쪽으로 끌려가는 climb-in. ApplyPullForce의 owner 쪽 미러(시뮬 루트 → CharacterMovement).
	void ApplyPullForceToWielder(const FVector& Force, float DeltaTime);

	// 능동 Pull 장력 상한 속도 드라이브: 대상 물리 바디를 당김 방향(Dir)을 따라 목표 속도(ActivePullMaxLinearSpeed)로
	// 몰되, 임펄스를 J = min(질량×ΔV, MaxTension×dt)로 클램프한다. 가벼운 대상은 목표 속도에 즉시(오버슛 없음),
	// 무거운 대상은 장력 한계로 뒤처진다(현실적 질량 의존). 상수 힘(a=F/m)의 오버슛·먼지·턱턱을 없앤다.
	void ApplyPullVelocityDrive(UPrimitiveComponent* Prim, FName BoneName, const FVector& Dir, float MaxTension, float DeltaTime) const;

	// 능동 Pull 대상 물리 바디의 각속도를 HoldConfig 상한으로 클램프(잔여 랙돌 스핀 안전망 — 힘을 무게중심에
	// 주므로 pull 토크는 이미 없음). ApplyPullForce가 힘 인가 뒤 호출. BoneName None이면 컴포넌트 단위.
	void ClampPulledBodyVelocity(UPrimitiveComponent* Prim, FName BoneName) const;

	// 모든 release 트리거의 공용 마무리(페이즈 전환+노드 반환+일시 상태 폐기+쿨다운+이벤트).
	void FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog);

	// 성립 전(Captured~Wrapping) 이탈의 공용 마무리: Flight 전이 + 일시 상태 폐기 후에 per-instance release를
	// 통지한다(정리 후 통지 — DispatchReleased 재진입 계약). dismiss/stall/wrapping-abort 4곳 공용. Bone은
	// 호출 시점(Reset 전)에 값으로 캡처된다. 커밋 전이라 bWasWrapped=false(중앙 신호 없이 per-instance만).
	void FinishPreCommitReleaseToFlight(FName Bone, const TCHAR* PhaseLog);

	// ReleaseWrap/CutRope 공용 본체: 진행 중인 잡기/감기를 주어진 사유로 해제(본 귀속 해석 포함).
	void ReleaseWrapAs(ERopeReleaseReason Reason);

	// (ApplyPullForce — 동작 2, Pull 힘 인가 — 는 protected 확장 훅으로 이동.)

	//~ 서브시스템 프레임 계약(RopeSimSubsystem이 쓰거나 읽는다) --------------
	// 프레임 단위 시뮬 입출력 묶음. 멤버별 의미/수명 규약은 FRopeSimFrameIO(Core/RopeSimFrameIO.h) 주석 참조.
	// 필드 이름은 낱개 멤버 시절 그대로라 접근 경로만 SimFrame.X다(CL 303).
	FRopeSimFrameIO SimFrame;

	// 마지막 렌더 push 상태. 정지 로프는 dynamic-data/transform dirty를 건너뛰되 GPU resident 전환과
	// component transform 변경은 반드시 새 WorldToLocal/로컬 centerline을 밀도록 비교한다.
	FTransform LastRenderDataComponentTransform = FTransform::Identity;
	bool bHasLastRenderDataComponentTransform = false;
	bool bLastRenderDataGpuResident = false;

	//~ 초기화/유틸 ----------------------------------------------------------
	void InitRope();

	/** Sim이 비어 있으면 1회 초기화한다(OnRegister/Throw/Prepare 초입의 안전 가드). */
	void EnsureRopeInitialized();

#if WITH_GAMEPLAY_DEBUGGER
	// 디버그 캡처 대상일 때 centerline/wrapped/collider 공통 필드를 스냅샷에 채운다(FinalizeSimFrame에서 호출).
	/** 헤더 요약은 항상, 나머지 섹션은 CaptureMask에 든 것만 채운다(꺼진 보기의 수집 비용을 내지 않는다). */
	void FillDebugSnapshot(FRopeDebugSnapshot& Snapshot, ERopeDebugCapture CaptureMask) const;
#endif

	//~ Throw ----------------------------------------------------------------
	// (MakeDefaultThrowContext/ResolveThrowContext는 protected 확장 훅으로 이동.)
	/** 이미 ResolveThrowContext를 통과한 컨텍스트로 Guaranteed preview를 만든다.
	 *  ThrowWithContext의 성공/실패 경로가 같은 해석 결과를 공유하기 위한 내부 진입점이다. */
	bool BuildPreparedWrappingPreviewFromResolvedContext(const FRopeThrowContext& ResolvedThrowContext,
		FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason) const;

	// 던지기 시작은 아래 4단계 헬퍼의 고정 순서로 읽는다(StartFreshThrow가 오케스트레이션만).
	void StartFreshThrow(const FRopeThrowContext& ThrowContext);

	/** ① 모든 throw의 공통 사전 정리: active wrap은 정상 release 통지와 함께 해제하고,
	 *  bridge/페이즈 일시 상태를 폐기한 뒤 쿨다운 없이 새 throw를 시작할 수 있게 한다. */
	void ResetStateForNewThrow();

	/** Prepared/Free 공용 GuidedThrow 상태와 시작 노드 pin을 구성한다. */
	bool BeginGuidedThrowState(FRopePreparedThrowPreview&& Prepared, bool bFreeThrow);

	/** ② 체인 리셋: 손(노드 0)을 원점에 핀, 전 노드 속도 0(Prev=Pos), GPU 상주 버퍼 재시드 세대 증가. */
	void ResetChainForThrow(const FVector& HandOrigin);

	/** ③ 채찍 스윙 시작: 스윙 기저/상속 속도를 ResolvedThrow에서 조립해 WhipGuide 활성화 + T=0 스냅. */
	void BeginWhipSwingFromThrow(const FRopeThrowContext& ResolvedThrow);

	/** ④ Verlet 속도 주입: PrevPositions를 조준 반대 방향으로 밀어 던지기 속도를 싣는다
	 *  (Verlet에서 속도 = (Pos-Prev)/dt — Prev만 밀면 위치 변화 없이 순수 속도 주입).
	 *  ③이 확정한 조준 방향(WhipGuide.GetAimDir)을 쓰므로 반드시 ③ 뒤에 호출. */
	void InjectThrowVelocityIntoVerlet(const FRopeThrowContext& ResolvedThrow);

	/** GuidedThrow phase 한 프레임 진행. preview centerline으로 노드를 이동시키며 solver는 끈다. */
	void UpdateGuidedThrow(float DeltaTime);

	/** GuidedThrow 완료 시 prepared anchor를 FRopeWrapState로 변환해 바로 Wrapped로 커밋한다. */
	void FinishGuidedThrow();

	/** Captured 통지의 단일 지점(네이티브 훅 → BP 델리게이트). ①② Flight 캡처와 ③ 도달이 공유한다 —
	 *  경로마다 인라인 브로드캐스트를 두면 한쪽이 빠져도 안 보인다(실제로 ③이 그랬다). */
	void DispatchCaptured(FName Bone);

	/** 허공(대상 없음) 던지기: 레이 끝점(EndpointWorld)을 향한 아치 GuidedThrow를 시작한다(꽂힘 없이 완료 시 Free). */
	void StartFreeGuidedThrow(const FRopeThrowContext& ThrowContext, const FVector& EndpointWorld);

	FVector ComputeThrowInheritedVelocity(const FRopeThrowContext& ThrowContext) const;

	/** WhipGuide에 넘길 설정 스냅샷을 Rope|Whip UPROPERTY들로부터 만든다. */
	FRopeWhipGuide::FConfig MakeWhipGuideConfig() const;

	/** 던지기 임펄스의 tail 가중치(FirstTailNode부터 끝까지 0→1 스무스). */
	float TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const;

	//~ Flight ---------------------------------------------------------------
	// 접촉 감지 파이프라인 자체는 FRopeFlightContactDetector(정적, UObject 비의존)로 분리됐다.
	// 여기엔 UObject 컨텍스트가 필요한 조립 코드만 남는다.

	/** 검출기에 넘길 파라미터 스냅샷(WrapConfig + 튜브 반지름 + 컴포넌트 전방 + substep dt + 프레임 dt).
	 *  SubstepDeltaTime은 SolverConfig.Substeps에서 유도한 FixedDt로 상대운동 평가의 SurfaceVelocity(cm/s→변위)
	 *  환산에, FrameDeltaTime은 예측 접촉 외삽의 substep→프레임 변위 환산에 쓰인다. */
	FRopeFlightContactDetector::FParams MakeFlightDetectParams(float DeltaTime) const;

	// FinalizeSimFrame의 Flight 블록은 아래 단계 헬퍼의 고정 순서로 읽는다:
	// ① 후보 산출 → ② 캡처 판정/전이 → ③ 관측(스탯/디버거 — 판정과 분리된 읽기 전용 소비).

	/** CanWrapTarget 게이트를 후보 리스트에 적용한다(금지 대상 제거). Flight 산출과 Contacting 재수집이
	 *  같은 판정 집합을 쓰도록 필터를 한 곳에 둔다 — 조건을 고치면 두 페이즈가 함께 움직인다. */
	void RemoveNonWrappableCandidates(TArray<FRopeContactCandidate>& Candidates) const;

	/** ① 이번 프레임 Flight 후보 선택/산출. GPU 결과는 SimFrame 배열을 복사 없이 직접 소비하고,
	 *  CPU fallback은 ContactCandidateScratch에 actual→predicted→상대운동 순으로 만든다. */
	TArray<FRopeContactCandidate>& GetOrBuildFlightContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams);

	/** CPU Flight fallback 전용 후보 산출. OutCandidates와 NextGuideTargetScratch는 호출자가 미리 Reset한다. */
	void BuildCpuFlightContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeContactCandidate>& OutCandidates);

	/**
	 * Assisted aim lock 전용 동기 보완: GPU 비동기 결과와 별개로 CPU whip target의 실제 경로를
	 * 정확히 잠근 본 collider에 검사한다. readback 중간 프레임 유실과 same-mesh 깊은 이웃 본의
	 * primary 가림을 막는다. Flight에서는 예측 경로도 포함하고 Contacting에서는 actual-only로 유지한다.
	 * 일반 Full/비조준 Flight에는 비용을 추가하지 않는다.
	 */
	void AddSynchronousAssistedAimContactCandidates(float DeltaTime,
		const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeContactCandidate>& InOutCandidates);

	/** ②a 후보를 한 번 집계해 캡처 판정과 관측이 공유할 frame-local 결과를 만든다. */
	FRopeFlightCaptureEvaluation EvaluateFlightCapture(const TArray<FRopeContactCandidate>& Candidates,
		const FRopeFlightContactDetector::FParams& DetectParams) const;

	/** ②b 평가 결과를 게임 상태에 적용한다. 캡처면 Tracker를 ContactTracker로 이동해 Contacting에
	 *  진입하고, 아니면 whip 종료 후 실패 타이머를 굴린다. 실제 캡처 여부를 반환한다. */
	bool ApplyFlightCaptureEvaluation(float DeltaTime, const TArray<FRopeContactCandidate>& Candidates,
		FRopeFlightCaptureEvaluation& Evaluation);

	/** ③ 관측: stat 카운터(수집 중일 때만) + 디버거 스냅샷(OutSnapshot != null일 때 — 디버거 대상
	 *  로프만 넘어온다). 판정(①②)에 관여하지 않는 읽기 전용 소비를 전부 여기 가둔다 —
	 *  FinalizeSimFrame 본문에 디버그/스탯 코드가 남지 않게 하는 것이 목적. */
	void RecordFlightObservation(const FRopeFlightContactDetector::FParams& DetectParams,
		const TArray<FRopeContactCandidate>& Candidates, const FRopeContactTracker& FrameTracker,
		bool bShouldCapture, FRopeDebugSnapshot* OutSnapshot);

#if WITH_GAMEPLAY_DEBUGGER
	/** ③ 관측 보조(디버거 대상 로프 전용): 노드별 감지 입력/판정 시각화 데이터 수집. 본 파이프라인과
	 *  별개로 감지기를 재질의한다(전 노드 스윕) — 대상 1개 로프만 비용을 내는 의도된 중복. */
	void GatherFlightNodeDebug(const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeFlightNodeDebug>& OutNodeDebug) const;
#endif

	/** 캡처 확정 시 평가 Tracker를 소유 상태로 이동하고 Contacting 진입 상태
	 *  (PendingWrapSeed/CaptureTravelFrame/타이머)를 구성한다. */
	void BuildContactingState(FRopeContactTracker&& EvaluatedTracker,
		const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);

	//~ Contacting -----------------------------------------------------------
	// 매 프레임 실제 접촉을 재수집해 트래커 dwell을 갱신한다: 지속 접촉 → Wrapping, 접촉 소실 →
	// dismiss(Flight), dwell이 임계에 못 미친 정체 → 안전망 타임아웃(Flight). 판정 기준은 총 경과가
	// 아니라 트래커 dwell(지배 본이 바뀌면 리셋)이다.
	void UpdateContacting(float DeltaTime);

	bool ShouldDismissContacting() const;

	bool ShouldStartWrapping() const;

	FRopeWrapState BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const;

	/**
	 * 한 (Bone, Mesh) 대상의 시드 latch/anchor를 구성한다(시드 다중화로 dominant/보조가 공용).
	 * OutLatch는 항상 채워지고, 접촉 후보에서 표면 프레임을 얻어 anchor까지 만들었으면 true.
	 * OutMesh는 트래커 mesh가 없을 때 후보의 mesh로 폴백된 결과(양쪽 다 없으면 null).
	 */
	bool BuildSeedLatchForTarget(const TArray<FRopeContactCandidate>& Candidates,
		FName Bone, const USceneComponent* TrackedMesh, int32 NodeIndex, float RopeDistance,
		FRopeLatchNode& OutLatch, FRopeSurfaceAnchor& OutAnchor, const USceneComponent*& OutMesh) const;

	//~ Wrapping -------------------------------------------------------------
	// 경로 생성/front 모션/마스크 등 Wrapping 페이즈의 실제 로직은 FRopeWrappingPhase(WrappingPhase)로
	// 분리됐다. 여기엔 페이즈 전이·이벤트를 결정하는 오케스트레이션만 남는다.

	void StartWrappingFromContacting();

	void UpdateWrapping(float DeltaTime);

	/** WrappingPhase에 넘길 호출 컨텍스트(WrapConfig/collider 스냅샷/튜브 반지름/로그 이름). */
	FRopeWrappingPhase::FContext MakeWrappingContext() const;

	/**
	 * MakeWrappingContext가 넘기는 collider 목록의 저장소 — CanWrapTarget 게이트를 통과한 것만 담는다
	 * (FContext가 배열을 *참조*로 들고 있어 호출보다 오래 사는 저장소가 필요하다).
	 * 금지 대상이 감김 경로 빌드의 표면/귀속 후보로 올라오는 것을 막는 관문이며, 게이트를
	 * 오버라이드하지 않은 로프에서는 FrameColliders와 내용이 같다(동작 불변).
	 */
	mutable TArray<IRopeCollider*> WrappableColliders;

	void CommitWrapping();

	/** Wrapped 성립 이벤트 페이로드 조립(커밋 시드 + 판정값 → NotifyWrapped/OnRopeWrapped 공용). */
	FRopeWrappedEventInfo MakeWrappedEventInfo(const FRopeWrapState& Seed, float AngleDeg, float CoverageDeg) const;

	/** wrap 성립 단일 브로드캐스트: 네이티브 훅 + per-instance BP 델리게이트 + 서브시스템 중앙 신호(③/판정 공용). */
	void DispatchWrapped(const FRopeWrappedEventInfo& Info);

	/** release 단일 브로드캐스트. per-instance(NotifyReleased + OnRopeReleased)는 항상 발화 — engagement가
	 *  끝날 때마다 짝을 맞춘다(Contacting/Wrapping abort·destroy 포함). engagement를 여는 것은 Captured,
	 *  Wrapped, **또는 조준된 ③ 던지기**(ThrowWithPreparedPreview 성공 — start 이벤트는 없지만 대상을 잡은
	 *  시점부터 engagement다). **허공 ③ 던지기(bFreeThrow)는 대상이 없어 아무것도 열지 않으므로 release도
	 *  없다** — 착지는 그냥 Free다. 중앙 OnAnyRopeReleased는 **커밋된 wrap(bWasWrapped)일 때만** 발화한다 —
	 *  성립 전 abort에서 쏘면 다른 로프가 감아 랙돌시킨 대상을 잘못 복구시킨다. WrappedMesh는 중앙 신호
	 *  페이로드(성립 전엔 nullptr). */
	void DispatchReleased(const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason, bool bWasWrapped);

	/**
	 * Wrapped 통지 도중 들어온 release 통지를 담아두는 큐 — **순서 역전 방지**가 목적이다.
	 * 핸들러가 통지 안에서 ReleaseWrap()을 부르면 release 통지가 중첩되어 먼저 끝나 버려, 구독자는
	 * Released → Wrapped 순으로 받는다(랙돌 대상이 복구를 먼저 무시하고 그 뒤 Wrapped만 받아 영구 고착).
	 * 그래서 Wrapped 통지가 다 끝날 때까지 release 통지를 미뤄 **항상 Wrapped → Released** 순을 보장한다.
	 * 상태 변경(ReleaseWrap 자체)은 미루지 않는다 — 미루는 것은 통지뿐이다.
	 */
	struct FDeferredReleaseNotice
	{
		TWeakObjectPtr<USceneComponent> WrappedMesh;
		FName Bone = NAME_None;
		ERopeReleaseReason Reason = ERopeReleaseReason::Manual;
		bool bWasWrapped = false;
	};

	/** DispatchWrapped 중첩 깊이(>0이면 release 통지를 큐에 넣는다). */
	int32 WrappedDispatchDepth = 0;
	TArray<FDeferredReleaseNotice> DeferredReleaseNotices;

	/** 큐에 밀린 release 통지를 순서대로 흘려보낸다(Wrapped 통지가 완전히 끝난 뒤에만 호출). */
	void FlushDeferredReleaseNotices();

	void AbortWrapping(ERopeReleaseReason Reason);

	/** ③ 연출(GuidedThrow) 중단 공용 마무리: Releasing 전환 + 일시 상태 폐기 + 쿨다운 + release 이벤트.
	 *  조준 던지기에서만 이벤트를 쏜다 — 허공 던지기(bFreeThrow)는 연 engagement가 없어 짝이 안 맞는다. */
	void AbortGuidedThrow(ERopeReleaseReason Reason, const TCHAR* ReasonLog);

	//~ Wrapped --------------------------------------------------------------
	// PrepareSimFrame의 Wrapped 케이스는 아래 4단계 헬퍼의 고정 순서로 읽는다.

	/** ① 본 추종: Hold(스킨 본 위 재배치 — 속도 주입 없음) + 질량 마스크. 대상 mesh 소실이면
	 *  Broken release를 마치고 false — 호출자는 이 프레임을 여기서 끝낸다. */
	bool HoldWrappedNodesToBone(float DeltaTime);

	/** ② 관측치 산출: wrap 장력(GetMaxTension) + Pull 샘플(ComputePull) + 2단 스무딩(조준 fractional
	 *  EMA → 방향 EMA). 견인(③)/release 판정(④)/디버거/BP가 공용으로 읽는 입력을 만든다. */
	void UpdateWrappedPullSample(float DeltaTime);

	/** ③ 견인 인가: 테더(λ 임펄스 제약 + 랙돌 물리 제약) + 능동 Pull(팽팽할 때 상수 힘/climb-in). */
	void ApplyWrappedTraction(float DeltaTime);

	/** ④ 자동 release 판정: 장력 지속 초과(TensionRelease*) / 거리 초과(DistanceReleaseSlack —
	 *  ③의 테더가 갱신한 초과분 소비). release가 일어났으면 true — 호출자는 솔브를 건너뛴다. */
	bool CheckWrappedAutoRelease(float DeltaTime);

	/** latch/anchor 노드 InvMass=0, 나머지 1 — Wrapped 중 자유 구간만 솔버가 움직이게. */
	void ApplyWrappedMassMask(bool bResetDynamicNodeVelocity = false);
	// 전체 질량 마스크는 topology/binding 변경 때만 다시 만든다. 매 프레임 Hold는 고정 노드 위치/InvMass만 갱신.
	bool bWrappedMassMaskDirty = true;

	/** WrappingPhase가 새로 산출한 run을 bridge로 한 번만 등록하고 front 도달 시 활성화한다. */
	void UpdateWrappingKinematicVirtualBridges(const TArray<FRopeVirtualBridgeRun>& Runs,
		const TArray<FRopeSurfaceAnchor>& Anchors, float FrontDistance);

	/** 기존 bridge를 최종 commit anchor로 재검증·갱신하고 활성화한다. 재생성하지 않는다. */
	bool FinalizeKinematicVirtualBridges(const TArray<FRopeVirtualBridgeRun>& Runs,
		const TArray<FRopeSurfaceAnchor>& CommitAnchors);

	/** 양쪽 anchor의 현재 월드 위치 사이에 bridge 노드를 균등 배치하고 hard kinematic override를 쓴다. */
	void HoldKinematicVirtualBridges();

	/** release/rethrow/non-composite 진입에서 이전 bridge binding을 폐기한다. */
	void ResetKinematicVirtualBridges();

	/** 활성 bridge 노드를 solver 질량으로 되돌리고 속도를 제거한 뒤 binding과 scan 상태를 폐기한다. */
	void ReleaseKinematicVirtualBridgesToSolver();

};
