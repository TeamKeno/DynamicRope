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
class UMaterialInstanceDynamic;
class USkeletalMeshComponent;
class UStaticMesh;
class UStaticMeshComponent;
// 랩 대상 추상화(Decision 0): 랩 대상 mesh를 USceneComponent로 일반화.
class USceneComponent;
class FRegisterComponentContext;
struct FRopeDebugSnapshot;
// 디버거 노드별 flight 시각화 항목(Debug/RopeDebugSnapshot.h).
struct FRopeFlightNodeDebug;

// Wrapped 성립 이벤트는 본 이름 하나에서 구조체 페이로드로 확장됐다(2026-07-13 회의 결정 G —
// 결착/판정값/복수 본. 기존 BP 바인딩은 재연결 필요, 클린 브레이크 승인 사항).
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnWrapped, const FRopeWrappedEventInfo&, Info);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnCaptured, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnReleased, FName, Bone, ERopeReleaseReason, Reason);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnPhaseChanged, ERopePhase, OldPhase, ERopePhase, NewPhase);

/** RopePreviewComponent가 자기 preview를 만들 때 필요한 읽기 전용 스냅샷. */
struct FRopePreviewBuildContext
{
	FRopeThrowContext ThrowContext;
	FRopeWhipGuide::FSwingBasis SwingBasis;
	FRopeWhipGuide::FConfig WhipConfig;
	// BuildPreviewContext 호출 직후의 동기 preview 생성 중에만 유효하다. 호출 범위 밖에 저장하면 안 된다.
	const TArray<IRopeCollider*>* Colliders = nullptr;
	FVector InheritedVelocity = FVector::ZeroVector;
	float RopeLength = 0.0f;
	float SegmentLength = 0.0f;
	float RopeRadius = 0.0f;
	int32 RopeNumSides = 8;
	int32 NodeCount = 0;
	ERopePhase Phase = ERopePhase::Free;
};

// (FRopeAimRayHitResult / FRopeAimRayThrowRequest는 Logic/RopeAimTargeting.h로 이동 — 위 include로 계속 노출된다.)

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeComponent : public UMeshComponent
{
	GENERATED_BODY()

	// 서브시스템이 프레임 구동을 위해 Sim/SolverConfig/Phase/WhipGuide + SimFrame(프레임 계약 묶음 —
	// FRopeSimFrameIO 주석 참조)에 직접 접근한다(GPU 배치 솔브 포함; CPU 경로는 SolveSimFrame 사용).
	friend class URopeSimSubsystem;

public:
	URopeComponent();

	//~ Setup(설정) -------------------------------------------------------

	/**
	 * 감김 해결(도달) 모드 — 이 로프의 최상위 계약(T1). 던지기~결착 성립까지 무엇을 보장하는지,
	 * 조준·preview의 지위, 판정 관문 사용 여부를 이 값 하나가 결정한다(ERopeWrapResolveMode 주석,
	 * Docs/PoC/02_WrapResolveModes.md). 정본은 로프다: Wielder의 조준/던지기 방식(aim ray 사용,
	 * preview 구속)은 여기서 유도되고, BP 직행/AI는 Wielder 없이 이 값만으로 완결된다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	/**
	 * 결착 모델(T1) — 팁이 닿는 순간 무엇이 성립하는가(ERopeTipEngagement 주석 참고).
	 * 도달 모드와 조합이 제약된다: ①②=BareWrap 전용, ③=Pierce/Cinch 전용. 무효 조합은
	 * 에디터 편집 시(모드가 정본 — TipEngagement가 보정됨)와 던지기 진입 시 자동 보정된다.
	 * Pierce/Cinch의 실행 배선은 후속 CL — 현재는 계약 선언과 이벤트 표기만.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	ERopeTipEngagement TipEngagement = ERopeTipEngagement::BareWrap;

	//~ Tip(팁 부착물 — 결착 모델 Pierce/Cinch용 창날/작살/추) --------------
	// 밧줄 자유단(GetNodeCount()-1)에 붙는 표시 전용 StaticMesh. 질량·충돌 없음(팁 질량 솔버 반영
	// 안 함 — 2026-07-14 확정). 던지기~해제 단위 수명: 던지기 진입에 확보, release/cut·EndPlay에
	// (우리가 스폰한 경우만) 파괴. 외부(태그로 찾은) 컴포넌트는 파괴하지 않는다.

	/** 팁에 스폰할 StaticMesh 에셋. 비어 있고 TipMeshComponentTag로도 못 찾으면 팁 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip")
	TObjectPtr<UStaticMesh> TipMesh = nullptr;

	/** 설정 시, Owner에 이미 붙은 이 태그의 StaticMeshComponent를 팁으로 재사용한다(스폰보다 우선, 파괴 안 함). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip")
	FName TipMeshComponentTag = NAME_None;

	/** 팁 노드(자유단) 프레임 기준 배치 오프셋(로컬 → 월드는 UpdateTipMeshTransform이 적용). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip")
	FTransform TipMeshRelativeTransform = FTransform::Identity;

	/** Reel(장전) 상태에서 창(팁)을 붙일 Owner 스켈레탈 메시의 소켓 이름. 비어 있거나 소켓이 없으면 컴포넌트(손) 트랜스폼.
	 *  기본 GetReelTipTransform() 구현이 사용한다 — 배치 규약을 바꾸려면 그 virtual을 override. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Tip")
	FName ReelHandSocket = NAME_None;

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

	/** 되감기/풀기 입력이 쓰는 기본 릴 속도(cm/s). 길이 변경은 로프 도메인이라 여기 산다
	 *  (2026-07-13 표면 감사 A-2 — Wielder에서 이사; Wielder Reel 액션이 이 값으로 SetReelRate 호출). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "0.0", Units = "cm/s"))
	float ReelSpeed = 150.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeSolverConfig SolverConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	FRopeThrowParams ThrowParams;

	/** physics → logic (wrap) 핸드오프 — *성립*(경로 빌드/판정/커밋) 튜닝. 감지는 DetectConfig. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	FRopeWrapConfig WrapConfig;

	/** Flight/Contacting *감지*(언제 잡혔다고 볼 것인가) 튜닝 — 성립(WrapConfig)과 분리된 도메인
	 *  (2026-07-13 표면 감사 B-1). 공유 프로브 반경(ContactQueryRadius)은 WrapConfig 소유. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Detect")
	FRopeDetectConfig DetectConfig;

	/** Wrapped *이후*(유지/당김/풀림) 튜닝 — 성립 판정(WrapConfig)과 분리된 Post-Wrap 도메인
	 *  (2026-07-13 표면 감사 B-1; 설계 노트 01 도메인, 도달 모드·결착 모델 무관 공통). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Hold")
	FRopeHoldConfig HoldConfig;

	//~ Collision(충돌 도메인) ----------------------------------------------
	// 흩어져 있던 충돌 관련 스위치를 한자리에 응집(2026-07-13 표면 감사 CL-4). 반지름 자체는
	// SolverConfig.CollisionRadius / WrapConfig.ContactQueryRadius에 있고, 0(기본)=auto — 아래
	// GetEffective* 헬퍼가 렌더 Radius에서 유도한다(반지름 3종 자동 정합).

	/**
	 * 기본적으로 rope는 월드의 모든 collider provider와 충돌하되 **자기 owner(던진 본인)의 provider는 제외**한다
	 * — throw 시 늘어진 로프가 던진 사람 팔다리에 엉키는 것을 막기 위함. cross-actor wrap(다른 액터 body 잡기)은
	 * 그 액터가 "전체"에 포함되므로 자동으로 동작한다.
	 * 켜면 owner provider도 포함한다(로프가 자기 owner 몸을 일부러 감아야 하는 드문 경우).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Collision")
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

	/** 해석된 접촉 질의 반지름: WrapConfig.ContactQueryRadius(0=auto → 렌더 Radius × 1.5). 감지/랩 경로 경계에서 소비. */
	float GetEffectiveContactQueryRadius() const
	{
		return WrapConfig.ContactQueryRadius > 0.0f ? WrapConfig.ContactQueryRadius : Radius * 1.5f;
	}

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
	UPROPERTY(EditAnywhere, BlueprintReadOnly, AdvancedDisplay, Category = "Rope|Render", meta = (ClampMin = "1", ClampMax = "8"))
	int32 TubeSmoothingSubdiv = 1;

	/** 렌더 튜브 스무딩의 Catmull-Rom knot α: 0=uniform, 0.5=centripetal(급한 코너에서 접선 오버슈트↓ —
	 *  벽을 짚는 구간의 중간 링이 벽 밖으로 덜 부푼다), 1=chordal. CPU 스무딩과 GPU resident 스무딩이
	 *  같은 값을 써 렌더가 일관된다. TubeSmoothingSubdiv=1이면 효과 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, AdvancedDisplay, Category = "Rope|Render", meta = (ClampMin = "0.0", ClampMax = "1.0"))
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

	/** 현재 FrameColliders를 swept SDF 질의해 ray에서 가장 가까운 wrap 가능 mesh+bone을 찾는다.
	 *  OutBlockedHit(옵션): ray는 맞았지만 wrap 불가한 가장 가까운 hit(조준 HUD "빨강" 표시용). */
	bool FindAimRayBoneHit(const FVector& Origin, const FVector& AimDir, float RayLength,
		float QueryRadius, float SweepStep, bool bDrawDebug, FRopeAimRayHitResult& OutHit,
		FRopeAimRayHitResult* OutBlockedHit = nullptr) const;

	/** Aim ray가 검사할 월드 구간을 collider subsystem의 로프별 수집 bounds에 등록한다. */
	void SetAimRayColliderQueryBounds(const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius);
	/** Aim ray 모드가 끝났을 때 이전 프레임의 추가 collider 수집 bounds를 제거한다. */
	void ClearAimRayColliderQueryBounds();
	/** 현재 FrameColliders로 Aim 요청을 해석한다. hit이 없으면 OutContext는 BaseContext fallback이다. */
	bool ResolveAimRayThrowContext(const FRopeAimRayThrowRequest& Request, FRopeThrowContext& OutContext) const;
	/** 실제 throw를 최신 collider 수집 직후 확정하도록 요청을 큐에 넣는다. */
	void QueueAimRayThrow(const FRopeAimRayThrowRequest& Request);

	/** PreviewComponent 전용: 현재 throw/whip/sim/collider 읽기 스냅샷을 만든다. */
	bool BuildPreviewContext(const FRopeThrowContext& ThrowContext, FRopePreviewBuildContext& OutContext) const;

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

	/**
	 * 던지기 준비(Reel/장전) 상태로 진입한다. ③(GuaranteedWrap) 전용 — 창(팁)을 손 소켓에 들고 로프를 숨긴다.
	 * 꽂힌 뒤 release로 Free가 된 상태에서만 유효(그 외엔 no-op). ③ 로프는 BeginPlay에서 자동으로 Reel로 시작한다.
	 * throw는 이 Reel 상태에서만 성립한다. 장전 입력 바인딩은 사용자 몫(이 API를 호출).
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void EnterReel();

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
		OutDirection = PullDrive.LastPullSample.Direction;
		OutTension = PullDrive.LastPullSample.Tension;
		return PullDrive.LastPullSample.bValid;
	}

	/** 이번 프레임 테더 초과분(cm): 손~앵커 직선 거리 - 가용 로프 길이(0 미만은 0). Wrapped 동안
	 *  매 프레임 산출된다(테더 off여도 계산). wielder 견인/지상 이탈 판정 등 게임 반응용. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetTetherOvershoot() const { return PullDrive.LastTetherOvershoot; }

	/** 이번 프레임 실제 사용된 테더 대상 몫(shareT) [0..1]. 자동(질량 기반)/수동 공통 최종값 —
	 *  1이면 wielder 몫 0(전량 대상), 0이면 전량 wielder. wielder 견인 활성 판정/디버그용. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetEffectiveTetherTargetShare() const { return PullDrive.LastTargetShare; }

	/**
	 * (BinaryPullable 테더 모드) 끌림 가능 판정 — 순수 함수(UObject 무의존, 유닛 테스트 가능).
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
	bool IsSleeping() const { return Throttle.IsAsleep(); }

	/** 현재 거리 LOD의 iteration 배율(1=풀 품질). 디버그/프로파일 확인용. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetSolverLODScale() const { return Throttle.GetSolverLODScale(); }

	/** 이번 프레임 이 로프가 GPU 솔버로 step됐는가(false면 CPU 폴백/솔버 off). 디버그 확인용. */
	bool IsGpuSteppedThisFrame() const { return SimFrame.bGpuSteppedThisFrame; }

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

	//~ Reel(장전) 연출 훅 — 전부 게임 스레드, 전이당 1회(콜드 패스). 기본 구현을 override해 연출을 커스텀한다.
	/** Reel 중 창(팁)을 놓을 월드 트랜스폼. 기본: Owner 스켈레탈 메시의 ReelHandSocket 소켓(없으면 컴포넌트 트랜스폼). */
	virtual FTransform GetReelTipTransform() const;
	/** EnterReel() 진입 시 1회. 기본: 로프 튜브 렌더를 숨긴다(SetVisibility(false)). */
	virtual void OnEnterReel();
	/** Reel에서 나가는 throw 성립 직전 1회. 기본: 로프 튜브를 다시 표시하고 전체 길이(RopeLength)를 복원한다. */
	virtual void OnDeployFromReel();

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
	virtual void NotifyWrapped(const FRopeWrappedEventInfo& Info) {}
	virtual void NotifyReleased(FName Bone, ERopeReleaseReason Reason) {}

	/**
	 * ③ GuaranteedWrap 연출(GuidedThrow 재생) 중 매 프레임 호출되는 인터럽트 판단 훅(GT, 콜드 패스 —
	 * 연출은 ~0.2초). 기본은 항상 false = "그래도 보장"(2026-07-13 회의 결정 G). 대상 사망/텔레포트
	 * 같은 게임 규칙으로 보장을 깨야 하면 오버라이드해 true 반환 — 로프가 연출을 중단하고 Releasing으로
	 * 빠진다. 대상 mesh 소실은 훅과 무관하게 항상 중단된다(Prepared.IsValid()).
	 */
	virtual bool ShouldAbortGuaranteedThrow(const FRopePreparedThrowPreview& Prepared) const { return false; }

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
	 * Force = 당김 방향 × 최대 장력(|Force| = 장력 상한). 물리 바디는 장력 상한 속도 드라이브로 인가한다
	 * (ApplyPullVelocityDrive). DeltaTime은 임펄스 상한(장력×dt) 산정에 쓴다.
	 */
	virtual void ApplyPullForce(const FVector& Force, const FRopePullSample& Pull, float DeltaTime);

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

	//~ 팁 부착물 런타임 상태 -----------------------------------------------
	// UObject라 값 타입 sim 멤버와 달리 GC 추적이 필요하다(Transient UPROPERTY).
	// 던지기 진입에 EnsureTipMesh가 확보하고, FinalizeSimFrame이 매 프레임 자유단으로 추종시킨다.
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> TipMeshComponent = nullptr;

	// 우리가 스폰했는가 — release/cut·EndPlay에서 스폰분만 파괴하기 위한 소유권 플래그(외부 컴포넌트 보호).
	bool bTipMeshSpawnedByUs = false;

	// 팁 부착물을 던지기~해제 단위로 확보/파괴/추종한다(TipMesh/TipMeshComponentTag가 설정된 경우만 동작).
	void EnsureTipMesh();
	void TeardownSpawnedTipMesh();
	void UpdateTipMeshTransform();

	//~ 페이즈 상태 머신 ----------------------------------------------------
	ERopePhase Phase = ERopePhase::Free;
	// Prepare 도중 Contacting 등에서 Flight로 돌아온 프레임은 Flight의 Advance/Solve를 거치지 않았다.
	// Finalize 접촉 감지를 한 프레임 미뤄 stale guide 후보로 즉시 재캡처되는 것을 막는다.
	bool bEnteredFlightDuringPrepareThisFrame = false;

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
	// 프레임 계약 진입점 2개만 남는다 — StartFreshThrow 전이(오케스트레이션)와 SimFrame 접근이 걸려
	// 있어 컴포넌트가 소유한다.
	// Subsystem이 FrameColliders를 채운 직후 호출해 pending request를 hit/fallback context로 확정한다.
	void ResolvePendingAimThrow();
	// AimRayHitDirection throw가 지정한 mesh+bone만 contact/wrap 후보로 유지한다.
	// collision-free Aim Flight에서는 solver가 이 목록을 의도적으로 무시하지만, 실제/예측 contact와
	// wrapping path는 필터된 목록을 계속 사용한다. 일반 Flight solver도 같은 목록을 사용한다.
	void FilterFrameCollidersForAimWrapTarget();
	/** FRopeAimTargeting 질의에 넘길 컨텍스트 스냅샷(collider 스냅샷 + 폴백 치수). */
	FRopeAimTargeting::FQueryContext MakeAimQueryContext() const;

	//~ 시뮬레이션 상태 + 페이즈별 로직 소유물 -------------------------------
	// Non-UObject — 값으로 소유하며 GC 추적 대상이 아니다(POD/약참조만 보유).
	// 아래 로직 4개는 로프 수명 순서와 1:1 대응한다: Throw/Flight → Contacting → Wrapping → Wrapped.
	/** 단일 진실: 솔버/로직/렌더가 공유하는 파티클 체인. */
	FRopeSimState       Sim;

	/** XPBD 물리(Free/Flight/Wrapped 자유 구간). */
	FRopeXPBDSolver     Solver;

	/** Throw/Flight: 채찍 스윙(가이드 타깃 계산+적용). */
	FRopeWhipGuide      WhipGuide;

	/** Flight/Contacting: 접촉 후보의 dominant bone 추적. */
	FRopeContactTracker ContactTracker;

	/** Contacting: 캡처 시 만들어 둔 wrap 시드(Wrapping 진입 재료). */
	FRopeWrapState      PendingWrapSeed;

	/** Contacting~Wrapping: 캡처 순간의 로프 진행 좌표계 스냅샷(속도/누운 방향/진행 평면 normal —
	 *  Contacting부터는 노드가 정지해 이 순간에만 잴 수 있다). TravelPlaneFirst 축의 가이드 평면 폴백. */
	FRopeCaptureTravelFrame CaptureTravelFrame;

	/** Wrapping: 경로 점진 생성+front 모션+마스크(작업 상태는 .State). */
	FRopeWrappingPhase  WrappingPhase;

	/** Wrapped: bone-local latch 유지/해제. */
	FRopeWrapController WrapController;

	/** PreviewPathLocked: cached preview path를 authoritative하게 구동. */
	FRopeGuidedThrowState GuidedThrowState;

	// Flight 시작 때 확정된 whip guide spline 평면 normal. Contacting을 거쳐 Wrapping에 들어갈 때
	// bone 위치에 세운 가상 wrapping axis의 방향으로 재사용한다.
	bool bHasFlightGuidePlaneNormal = false;
	FVector FlightGuidePlaneNormal = FVector::RightVector;

	// Aim-ray 조준 상태(throw당 wrap 대상 잠금 + pending aim throw 큐). 질의/잠금 판정 로직 포함 —
	// FRopeAimTargeting(Logic/RopeAimTargeting.h) 주석 참조.
	FRopeAimTargeting AimTargeting;

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

	// 되감기 속도(cm/s, +감기/-풀기, 0=정지). SetReelRate가 설정, UpdateReel이 프레임마다 적용.
	float ReelRate = 0.0f;

	// 되감기 프레임 적용(Prepare 초입): 허용 페이즈에서 ReelRate × dt만큼 길이를 조정한다.
	void UpdateReel(float DeltaTime);

	//~ 슬립/LOD(스케일링) ---------------------------------------------------
	// 상태·판정은 FRopeSolverThrottle(Logic/RopeSolverThrottle.h)로 분리 — 컴포넌트에는 카메라 접근(GT)과
	// 슬립 전이 로그만 남는다.
	FRopeSolverThrottle Throttle;

	// 거리 LOD 배율 계산(Prepare, GT): 카메라 거리만 여기서 산출해 Throttle에 위임. 카메라 없으면(서버) 1 유지.
	void ComputeSolverLOD();
	// LOD 반영된 유효 iteration(CPU 솔브/GPU 스텝 공용 — 서브시스템이 호출).
	int32 GetLODScaledIterations() const { return Throttle.LODScaledIterations(SolverConfig.Iterations); }

	// 동작 1 — 자동 견인(테더): 가용 로프 길이 초과분을 위치/속도 동기로 회수(수렴, 폭주 없음).
	void UpdateTether(float DeltaTime);

	// (테더 공용) wielder 견인 방향(손(노드0)→로프 첫 다리 = 앵커 쪽)을 산출해 PullDrive.SmoothedWielderPullDir로
	// EMA 스무딩(PullDirSmoothTime)해 반환. MassShare/BinaryPullable의 wielder 몫이 공유 — 방향 지터로 클램프
	// 축이 튀는 것을 막는다(180° 반전 축퇴는 raw로 재시드).
	FVector ComputeSmoothedWielderDir(const FVector& Aim, const FVector& DirToAim, float DeltaTime);

	// (BinaryPullable 전용) 이번 Wrapped 프레임의 끌림 가능 판정을 overshoot와 무관하게 갱신한다 —
	// 테더 회수(UpdateTether)와 능동 Pull 방향(ApplyWrappedTraction)이 PullDrive.bTargetPullable을 공유.
	// 양끝 유효질량 비교 + TetherPullMassMargin 히스테리시스. LastTargetShare(이진 0/1)도 여기서 채운다.
	void UpdateTargetPullable();

	// (BinaryPullable + not pullable) 능동 Pull 힘을 wielder(로프 owner)에 인가 — 대상이 무거워
	// wielder가 앵커 쪽으로 끌려가는 climb-in. ApplyPullForce의 owner 쪽 미러(시뮬 루트 → CharacterMovement).
	void ApplyPullForceToWielder(const FVector& Force);

	// 능동 Pull 장력 상한 속도 드라이브: 대상 물리 바디를 당김 방향(Dir)을 따라 목표 속도(ActivePullMaxLinearSpeed)로
	// 몰되, 임펄스를 J = min(질량×ΔV, MaxTension×dt)로 클램프한다. 가벼운 대상은 목표 속도에 즉시(오버슛 없음),
	// 무거운 대상은 장력 한계로 뒤처진다(현실적 질량 의존). 상수 힘(a=F/m)의 오버슛·먼지·턱턱을 없앤다.
	void ApplyPullVelocityDrive(UPrimitiveComponent* Prim, FName BoneName, const FVector& Dir, float MaxTension, float DeltaTime) const;

	// 능동 Pull 대상 물리 바디의 각속도를 HoldConfig 상한으로 클램프(잔여 랙돌 스핀 안전망 — 힘을 무게중심에
	// 주므로 pull 토크는 이미 없음). ApplyPullForce가 힘 인가 뒤 호출. BoneName None이면 컴포넌트 단위.
	void ClampPulledBodyVelocity(UPrimitiveComponent* Prim, FName BoneName) const;

	// 모든 release 트리거의 공용 마무리(페이즈 전환+노드 반환+일시 상태 폐기+쿨다운+이벤트).
	void FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog);

	// ReleaseWrap/CutRope 공용 본체: 진행 중인 잡기/감기를 주어진 사유로 해제(본 귀속 해석 포함).
	void ReleaseWrapAs(ERopeReleaseReason Reason);

	// (ApplyPullForce — 동작 2, Pull 힘 인가 — 는 protected 확장 훅으로 이동.)

	//~ 서브시스템 프레임 계약(RopeSimSubsystem이 쓰거나 읽는다) --------------
	// 프레임 단위 시뮬 입출력 묶음. 멤버별 의미/수명 규약은 FRopeSimFrameIO(Core/RopeSimFrameIO.h) 주석 참조.
	// 필드 이름은 낱개 멤버 시절 그대로라 접근 경로만 SimFrame.X다(CL 303).
	FRopeSimFrameIO SimFrame;

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
	// 던지기 시작은 아래 4단계 헬퍼의 고정 순서로 읽는다(StartFreshThrow가 오케스트레이션만).
	void StartFreshThrow(const FRopeThrowContext& ThrowContext);

	/** ① 이전 상태 정리: 잡고 있던 wrap 수동 해제 + 페이즈 일시 상태 폐기 + 쿨다운 0(즉시 재던지기). */
	void AbandonActiveStateForRethrow();

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

	FVector ComputeThrowInheritedVelocity(const FRopeThrowContext& ThrowContext) const;

	/** WhipGuide에 넘길 설정 스냅샷을 Rope|Whip UPROPERTY들로부터 만든다. */
	FRopeWhipGuide::FConfig MakeWhipGuideConfig() const;

	/** 던지기 임펄스의 tail 가중치(FirstTailNode부터 끝까지 0→1 스무스). */
	float TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const;

	//~ Flight ---------------------------------------------------------------
	// 접촉 감지 파이프라인 자체는 FRopeFlightContactDetector(정적, UObject 비의존)로 분리됐다.
	// 여기엔 UObject 컨텍스트가 필요한 조립 코드만 남는다.

	/** 검출기에 넘길 파라미터 스냅샷(WrapConfig + 튜브 반지름 + 컴포넌트 전방 + substep dt). SubstepDeltaTime은
	 *  SolverConfig.Substeps에서 유도한 FixedDt로, 상대운동 평가의 SurfaceVelocity(cm/s→변위) 환산에 쓰인다. */
	FRopeFlightContactDetector::FParams MakeFlightDetectParams() const;

	// FinalizeSimFrame의 Flight 블록은 아래 단계 헬퍼의 고정 순서로 읽는다:
	// ① 후보 산출 → ② 캡처 판정/전이 → ③ 관측(스탯/디버거 — 판정과 분리된 읽기 전용 소비).

	/** CanWrapTarget 게이트를 후보 리스트에 적용한다(금지 대상 제거). Flight 산출과 Contacting 재수집이
	 *  같은 판정 집합을 쓰도록 필터를 한 곳에 둔다 — 조건을 고치면 두 페이즈가 함께 움직인다. */
	void RemoveNonWrappableCandidates(TArray<FRopeContactCandidate>& Candidates) const;

	/** ① 이번 프레임 접촉 후보 산출: whip 예측 뷰 조립 + GPU 감지 산출물 회수(상대운동 평가만 GT)
	 *  또는 CPU 감지 파이프라인(actual→predicted→상대운동), 마지막에 CanWrapTarget 게이트. */
	void BuildFlightContactCandidates(float DeltaTime, const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeContactCandidate>& OutCandidates);

	/** ② 캡처 판정/전이: 캡처면 Contacting 진입(이벤트 브로드캐스트 포함), 아니면 whip 종료 후 실패
	 *  타이머를 굴려 FlightNoContactReturnTime 초과 시 Free 복귀. 캡처 여부를 반환한다(③ 관측 소비용). */
	bool TryCaptureFlightContacts(float DeltaTime, const TArray<FRopeContactCandidate>& Candidates,
		const FRopeFlightContactDetector::FParams& DetectParams);

	/** ③ 관측: stat 카운터(수집 중일 때만) + 디버거 스냅샷(OutSnapshot != null일 때 — 디버거 대상
	 *  로프만 넘어온다). 판정(①②)에 관여하지 않는 읽기 전용 소비를 전부 여기 가둔다 —
	 *  FinalizeSimFrame 본문에 디버그/스탯 코드가 남지 않게 하는 것이 목적. */
	void RecordFlightObservation(const FRopeFlightContactDetector::FParams& DetectParams,
		const TArray<FRopeContactCandidate>& Candidates, bool bShouldCapture, FRopeDebugSnapshot* OutSnapshot);

#if WITH_GAMEPLAY_DEBUGGER
	/** ③ 관측 보조(디버거 대상 로프 전용): 노드별 감지 입력/판정 시각화 데이터 수집. 본 파이프라인과
	 *  별개로 감지기를 재질의한다(전 노드 스윕) — 대상 1개 로프만 비용을 내는 의도된 중복. */
	void GatherFlightNodeDebug(const FRopeFlightContactDetector::FParams& DetectParams,
		TArray<FRopeFlightNodeDebug>& OutNodeDebug) const;
#endif

	/** 캡처 확정 시 Contacting 진입 상태(ContactTracker/PendingWrapSeed/CaptureTravelFrame/타이머)를
	 *  구성한다. DeltaTime은 travel frame의 Verlet 속도 환산용(캡처 프레임의 dt). */
	void BuildContactingState(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);

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

	/** 로프별 감김 경로 모드(WrapConfig.WrappingPathMode — 전역 설정에서 per-rope로 이동, 2026-07-13). */
	ERopeWrappingPathMode GetWrappingPathMode() const;

	/** WrappingPhase에 넘길 호출 컨텍스트(WrapConfig/collider 스냅샷/경로 모드/튜브 반지름/로그 이름). */
	FRopeWrappingPhase::FContext MakeWrappingContext() const;

	void CommitWrapping();

	/** Wrapped 성립 이벤트 페이로드 조립(커밋 시드 + 판정값 → NotifyWrapped/OnRopeWrapped 공용). */
	FRopeWrappedEventInfo MakeWrappedEventInfo(const FRopeWrapState& Seed, float AngleDeg, float CoverageDeg) const;

	/** wrap 성립 단일 브로드캐스트: 네이티브 훅 + per-instance BP 델리게이트 + 서브시스템 중앙 신호(③/판정 공용). */
	void DispatchWrapped(const FRopeWrappedEventInfo& Info);

	/** release 단일 브로드캐스트. per-instance(NotifyReleased + OnRopeReleased)는 항상 발화 — Captured/Wrapped로
	 *  시작된 engagement가 끝날 때마다 짝을 맞춘다(Contacting/Wrapping abort·destroy 포함). 중앙 OnAnyRopeReleased는
	 *  **커밋된 wrap(bWasWrapped)일 때만** 발화한다 — 성립 전 abort에서 쏘면 다른 로프가 감아 랙돌시킨 대상을
	 *  잘못 복구시킨다. WrappedMesh는 중앙 신호 페이로드(성립 전엔 nullptr). */
	void DispatchReleased(const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason, bool bWasWrapped);

	void AbortWrapping(ERopeReleaseReason Reason);

	//~ Wrapped --------------------------------------------------------------
	// PrepareSimFrame의 Wrapped 케이스는 아래 4단계 헬퍼의 고정 순서로 읽는다.

	/** ① 본 추종: Hold(스킨 본 위 재배치 — 속도 주입 없음) + 질량 마스크. 대상 mesh 소실이면
	 *  Broken release를 마치고 false — 호출자는 이 프레임을 여기서 끝낸다. */
	bool HoldWrappedNodesToBone(float DeltaTime);

	/** ② 관측치 산출: wrap 장력(GetMaxTension) + Pull 샘플(ComputePull) + 2단 스무딩(조준 fractional
	 *  EMA → 방향 EMA). 견인(③)/release 판정(④)/디버거/BP가 공용으로 읽는 입력을 만든다. */
	void UpdateWrappedPullSample(float DeltaTime);

	/** ③ 견인 인가: 테더(초과분 위치/속도 동기 — TetherTargetShare 분배) + 능동 Pull(팽팽할 때 상수 힘). */
	void ApplyWrappedTraction(float DeltaTime);

	/** ④ 자동 release 판정: 장력 지속 초과(TensionRelease*) / 거리 초과(DistanceReleaseSlack —
	 *  ③의 테더가 갱신한 초과분 소비). release가 일어났으면 true — 호출자는 솔브를 건너뛴다. */
	bool CheckWrappedAutoRelease(float DeltaTime);

	/** latch/anchor 노드 InvMass=0, 나머지 1 — Wrapped 중 자유 구간만 솔버가 움직이게. */
	void ApplyWrappedMassMask(bool bResetDynamicNodeVelocity = false);

	bool ComputeTensionSlack(float& OutSlack, float& OutStraightDistance, float& OutAvailableLength) const;
};
