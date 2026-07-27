// Copyright Epic Games, Inc. All Rights Reserved.
//
// 로프 프리셋(DataAsset) — URopeComponent의 "행동 정체성"(모드/물성/감지/유지/팁/렌더) 값 묶음.
// **스탬프형(copy-on-apply)**: URopeComponent::ApplyPreset()이 값을 컴포넌트로 복사하고 끝난다 —
// 적용 후 컴포넌트 개별 튜닝은 자유고, 프리셋 에셋과의 라이브 링크는 없다(참조형의 per-field
// override 표면 폭발을 의도적으로 배제 — 2026-07-18 회의 11번 안건).
//
// 담지 않는 것(인스턴스 배선 — 액터/스켈레톤에 결합된 값이라 프리셋이 덮으면 배선이 깨진다):
// TipMeshComponentTag(소유 액터의 컴포넌트 태그), Wielder 쪽 전부(AttachMesh/입력/Movement — v1 범위 밖).
// LoadedHandSocket(소유 스켈레톤 소켓)만 예외로, bOverrideLoadedHandSocket을 켜둔 프리셋만 덮어쓴다
// (기본 꺼짐 = 배선 보존).
//
// 필드는 URopeComponent의 동명 프로퍼티와 1:1 미러다 — 기본값·Clamp meta·툴팁은 물론 **디테일 패널
// 표면까지** 컴포넌트와 같게 유지할 것: 같은 카테고리 이름(Rope|…), 같은 DisplayName, 같은
// EditCondition, 설정 구조체는 ShowOnlyInnerProperties, 선언 순서도 컴포넌트와 동일. 그래야 프리셋
// 에셋을 열었을 때 컴포넌트의 Rope 섹션과 같은 모양·같은 순서로 읽힌다. 구조체 *안쪽* 필드의
// 노출 여부/표시 이름은 정의부(RopeConfigTypes.h·RopeThrowTypes.h)를 공유하므로 자동으로 따라오고,
// 구조체를 넘어가는 ③ 게이트 하나(HoldConfig의 자동 release 3종)는 에디터 커스터마이즈가 맡는다
// (DynamicRopeEditor의 RopeResolveModeDetails.h — 컴포넌트와 같은 클래스를 공유한다).
// (기본값 프리셋 적용 = 기본 로프 = 회귀 없음이 계약.)

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Core/RopeTypes.h"
#include "RopePreset.generated.h"

class UMaterialInterface;
class UStaticMesh;

/**
 * 로프 프리셋 — 검증된 로프 구성(그래플링 훅/포획 로프/자유 시뮬 등) 한 벌을 에셋으로 담아
 * URopeComponent::ApplyPreset()으로 통째 적용한다(Free/Loaded 페이즈에서만).
 */
UCLASS(BlueprintType, meta = (ToolTip = "A complete rope configuration (mode, physics, detection, hold, tip, render) applied to a URopeComponent as one stamp via ApplyPreset(). Values are copied — no live link. Only applies while the rope is in Free or Loaded phase."))
class DYNAMICROPE_API URopePreset : public UDataAsset
{
	GENERATED_BODY()

public:
	// 기본값을 컴포넌트 생성자와 동기화한다(RopeMaterial = 플러그인 기본 헴프 밧줄 — URopeComponent
	// 생성자와 같은 FObjectFinder). None으로 두면 "빈 프리셋 적용 = 기본 로프" 계약이 머티리얼에서
	// 깨진다(스탬프가 기본 머티리얼을 벗겨 회색 폴백이 된다).
	URopePreset();

	//~ Setup(설정) ---------------------------------------------------------

	/** 감김 해결(도달) 모드 — 던지기~결착까지 무엇을 보장하는가. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (DisplayName = "Wrap Mode"))
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	//~ Tip(팁 부착물) -------------------------------------------------------
	// TipMeshComponentTag은 인스턴스 배선이라 프리셋에 없고, LoadedHandSocket은 옵트인 스탬프다
	// (파일 머리 주석). 적용 시 기존 팁(우리가 스폰한 것만)은 파괴 후 새 설정으로 재확보된다.

	/** 팁 부착물을 사용한다. 끄면 아래 Tip 설정이 전부 무시되고 팁 없는 일반 로프가 된다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip")
	bool bUseTipMesh = false;

	/** 팁에 스폰할 StaticMesh. 비어 있으면(대상 컴포넌트의 태그 재사용도 없을 때) 팁 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Mesh"))
	TObjectPtr<UStaticMesh> TipMesh = nullptr;

	/** 팁 배치 오프셋(팁 노드 프레임 기준). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Relative Transform"))
	FTransform TipMeshRelativeTransform = FTransform::Identity;

	/** 팁 메쉬의 충돌을 켠다. **기본 꺼짐** — 표시 전용 팁의 충돌이 로프/캐릭터와 간섭하는 것을 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Enable Collision"))
	bool bTipMeshCollision = false;

	/** Free에서 팁을 매 프레임 로프 끝에 맞춘다. 끄면 Free 동안 팁을 건드리지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Sync While Free"))
	bool bSyncTipMeshOnFree = true;

	// Loaded (hand) placement. The socket name is wiring bound to the owning skeleton, so it is stamped
	// only when opted in; the offset defaults to Identity - which is exactly the current behaviour - and
	// is stamped unconditionally like the rest of the Tip fields. The opt-in toggle has no component
	// counterpart (it governs the stamp, not the rope), so it renders under its own name.

	/** Stamp Loaded Hand Socket onto the component. Leave off to keep the instance's own socket wiring. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	bool bOverrideLoadedHandSocket = false;

	/** Socket on the owner's skeletal mesh the tip is held at while Loaded. Empty falls back to the component transform. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bOverrideLoadedHandSocket"))
	FName LoadedHandSocket = NAME_None;

	/** Offset applied to the tip while Loaded, expressed in the hand socket's frame. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh", DisplayName = "Loaded Relative Transform"))
	FTransform LoadedTipRelativeTransform = FTransform::Identity;

	/** ③(Guaranteed) 전용 — Head/Tail 소켓으로 팁을 정밀 배치한다. 끄면 메쉬 원점이 로프 끝에 놓인다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Use Sockets"))
	bool bUseTipMeshSockets = false;

	/** Head 소켓 — 팁의 뾰족한 끝. 이 소켓이 조준 히트점에 박힌다. 없으면 소켓 보정 비활성. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Tip Socket"))
	FName TipSocketName = NAME_None;

	/** Tail 소켓 — 로프 자유단이 연결될 지점. 없으면 메쉬 원점에 연결. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Rope Socket"))
	FName TipRopeSocketName = NAME_None;

	//~ Rope(기본 물성) -----------------------------------------------------

	/** 노드(파티클) 수. 적용 시 로프가 재초기화된다. 상한 512 = GPU 솔버 스레드그룹 한계. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "2", ClampMax = "512", DisplayName = "Node Count"))
	int32 NumParticles = 72;

	/** 초기(최대) 로프 길이(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 600.0f;

	/** 되감기(reel-in)로 줄일 수 있는 최소 길이(cm). RopeLength(초기)가 상한이다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "10.0", Units = "cm"))
	float MinRopeLength = 100.0f;

	/** 되감기/풀기 입력이 쓰는 기본 릴 속도(cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "0.0", Units = "cm/s"))
	float ReelSpeed = 150.0f;

	/** Loaded(장전) 상태에서 로프 튜브를 보인다. ③(GuaranteedWrap) 전용 연출 스위치. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope",
		meta = (EditCondition = "ResolveMode == ERopeWrapResolveMode::GuaranteedWrap"))
	bool bShowRopeWhenLoaded = false;

	//~ 페이즈별 config 구조체(컴포넌트와 동일 도메인 분할) --------------------
	// 컴포넌트와 같이 ShowOnlyInnerProperties로 노출한다 — 카테고리 헤더(Rope|Solver / Rope|Throw / …)
	// 바로 아래에 필드가 펼쳐지므로 "카테고리 → 구조체 이름 → 필드"의 이중 확장이 없고, 각 필드의
	// 세부 카테고리(Rope|Solver|Tuning 등)는 구조체 내부 정의가 그대로 유지한다.

	/** 솔버(XPBD) 튜닝 — substep/iteration/컴플라이언스/마찰/중력/슬립/LOD. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Solver", meta = (ShowOnlyInnerProperties))
	FRopeSolverConfig SolverConfig;

	/** 던지기 파라미터. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Throw", meta = (ShowOnlyInnerProperties))
	FRopeThrowParams ThrowParams;

	// ③(GuaranteedWrap)는 preview 기반이라 감김 판정/경로 빌드가 없고 채찍 Flight 대신 GuidedThrow 아치를
	// 타므로, WrapConfig/WhipConfig는 ③ 프리셋에서 무의미하다 → 컴포넌트와 같은 구조체-멤버 EditCondition으로
	// 회색처리한다(ResolveMode가 같은 클래스의 형제 멤버라 여기서도 보인다). ShowOnlyInnerProperties로 승격된
	// 인라인 자식까지 edit-const가 전파되고, 값은 보존된 채 편집만 막힌다.

	/** physics → logic (wrap) 핸드오프 — 캡처 판정 문턱과 *성립*(경로 빌드/판정/커밋) 튜닝. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Wrap",
		meta = (ShowOnlyInnerProperties, EditCondition = "ResolveMode != ERopeWrapResolveMode::GuaranteedWrap"))
	FRopeWrapConfig WrapConfig;

	/** Wrapped *이후*(유지/당김/풀림) 튜닝. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Hold", meta = (ShowOnlyInnerProperties))
	FRopeHoldConfig HoldConfig;

	//~ Collision(충돌) -----------------------------------------------------

	/** 자기 owner의 collider provider도 충돌에 포함할지(기본 제외 — 던진 사람 몸에 엉킴 방지). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Collision",
		meta = (ToolTip = "끄면(기본) 자기 owner의 콜라이더를 제외합니다 — 정적 월드 provider가 잡은 owner 소유 셰이프(테더 프록시/팁/무기)도 바디 단위로 빠집니다. 로프를 기둥 등 프롭 액터에 붙였다면 켜세요(안 켜면 받침대를 통과).", DisplayName = "Collide With Owner"))
	bool bIncludeOwnerColliders = false;

	/** 엔진 Global Distance Field로 정적 월드 지오메트리(벽/바닥)에서 로프를 밀어낼지. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Collision", meta = (DisplayName = "Use World Distance Field"))
	bool bUseWorldGDF = true;

	//~ Whip(던지기 스윙) ----------------------------------------------------

	/** 던지기 초반 채찍 스윙 튜닝. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Whip",
		meta = (ShowOnlyInnerProperties, EditCondition = "ResolveMode != ERopeWrapResolveMode::GuaranteedWrap"))
	FRopeWhipConfig WhipConfig;

	//~ Render(렌더) --------------------------------------------------------
	// 컴포넌트에서는 프록시 생성 시 1회 소비되는 값들 — ApplyPreset이 렌더 상태 재생성까지 책임진다.

	/** 시각적 tube 반지름(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (ClampMin = "0.1", Units = "cm", DisplayName = "Rope Radius"))
	float Radius = 2.0f;

	/** rope tube에 적용되는 material. 비우면 엔진 기본 material. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (DisplayName = "Material"))
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	/** tube 단면의 변 개수. 높을수록 더 둥글어진다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "3", ClampMax = "32", DisplayName = "Sides"))
	int32 NumSides = 8;

	/** 렌더 튜브 스무딩: 세그먼트당 Catmull-Rom 서브분할 수(1=끔). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "1", ClampMax = "8", DisplayName = "Smoothing Subdivisions"))
	int32 TubeSmoothingSubdiv = 1;

	/** 렌더 튜브 스무딩의 Catmull-Rom knot α: 0=uniform, 0.5=centripetal, 1=chordal. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Smoothing Strength"))
	float TubeSmoothingAlpha = 0.5f;

#if WITH_EDITOR
	//~ 에디터 검증 — 저작 실수(길이 역전 등)를 에셋 저장 시점에 알린다.
	virtual EDataValidationResult IsDataValid(FDataValidationContext& Context) const override;
#endif
};
