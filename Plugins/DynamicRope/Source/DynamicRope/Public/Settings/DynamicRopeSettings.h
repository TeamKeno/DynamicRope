// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeConfigTypes.h"
#include "Engine/DeveloperSettings.h"
class URopePullGaugeWidget;

#include "DynamicRopeSettings.generated.h"

class ARopeController;
class URopeAimWidget;
class URopePreset;

/**
 * Dynamic Rope 플러그인의 프로젝트 전역 설정.
 * Project Settings > Plugins > Dynamic Rope에서 편집 가능하며 DefaultGame.ini에 저장된다.
 */
UCLASS(config = Game, defaultconfig, meta = (DisplayName = "Dynamic Rope"))
class DYNAMICROPE_API UDynamicRopeSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UDynamicRopeSettings();

	/** 활성 (CDO) 설정 객체에 접근하기 위한 편의 접근자. */
	static const UDynamicRopeSettings* Get();

	// NOTE: 이전의 config 필드(DefaultRopeLength/DefaultTension/MaxWrapTurnsPerBodyPart/
	// WrappableBones/AutoUnwrapDistance/bEnableDebugDraw)는 어디에서도 읽히지 않는 dead 설정이라
	// 제거했다. 새 전역 설정을 추가할 때는 반드시 소비처를 함께 연결할 것(Get()으로 접근).
	// WrappingPathMode도 로프별 값(FRopeWrapConfig::WrappingPathMode)으로 이동해 제거됨
	// (2026-07-13 회의 — 기존 전역 튜닝 미승계 클린 브레이크).

	/**
	 * 게임/PIE 월드 시작 시 URopeSimSubsystem이 자동 스폰하는 로프 매니저 액터 클래스. 이 액터가
	 * 정적 월드 충돌용 URopeStaticBodyProvider를 품는다 — 레벨마다 프로바이더를 수동 배치하지 않아도
	 * "월드당 정확히 1개"를 보장한다. ARopeController를 서브클래스해 MaxColliders 등을 조정할 수 있다.
	 * 비우면(None) 자동 스폰을 끈다(수동 배치를 원하는 프로젝트용).
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ToolTip = "게임 시작 시 자동 스폰되는 로프 매니저 액터 클래스(정적 월드 충돌 프로바이더 호스트). 비우면 자동 스폰을 끕니다."))
	TSoftClassPtr<ARopeController> StaticBodyControllerClass;

	/**
	 * 정적 바디 프로바이더가 프레임당 추출할 콜라이더 전역 상한 — 밀집 콜리전 지대에서 추출/컬 비용이
	 * 폭주하는 것을 막는 "안전밸브"다(정상 씬은 여기 닿지 않아야 정상). 실제 로프별 솔브 예산은 아래
	 * StaticBodyMaxCollidersPerRope가 담당하므로, 이 값은 (예상 최대 로프 수 × per-rope 예산)보다
	 * 넉넉히 잡아 전역 추출 단계에서 로프가 굶지 않게 한다. 프로바이더가 매 프레임 직접 읽는 단일 소스.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ClampMin = "1", ToolTip = "정적 바디 프로바이더의 프레임당 전역 추출 상한(밀집 씬 폭주 방지 안전밸브). 로프별 예산은 StaticBodyMaxCollidersPerRope."))
	int32 StaticBodyMaxColliders = 256;

	/**
	 * 한 로프가 솔브에 실을 수 있는 정적 월드 콜라이더 상한(로프별). 로프별 컬링 후 이 수를 넘으면 그 로프에서
	 * 가장 먼 콜라이더부터 버린다 — GPU 커널이 노드×substep마다 콜라이더를 루프하므로 로프별 솔브 비용을
	 * 직접 바운드한다. 전역 상한(StaticBodyMaxColliders)과 달리 로프마다 독립이라, 멀리 있는 로프의 콜라이더가
	 * 가까운 로프의 예산을 잡아먹지 않는다(order-independent). 스켈레톤 콜라이더는 이 예산과 무관하게 항상 포함.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ClampMin = "1", ToolTip = "로프 1개가 솔브에 실을 정적 월드 콜라이더 상한(로프별). 초과 시 가장 먼 것부터 드롭. 스켈레톤은 항상 포함."))
	int32 StaticBodyMaxCollidersPerRope = 32;

	/**
	 * 정적 바디 프로바이더의 컨벡스 1개당 평면 수 상한. 이 수를 넘는 복잡한 컨벡스는 ElemBox OBB로 폴백한다.
	 * StaticBodyMaxColliders와 마찬가지로 프로바이더가 직접 읽는 단일 소스다. (전체 콜라이더 개수는 위에서 별도 제한.)
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ClampMin = "4", ToolTip = "컨벡스당 평면 수 상한(초과분은 OBB 폴백). 단일 소스 — 프로바이더가 직접 읽습니다."))
	int32 StaticBodyMaxConvexPlanes = 32;

	/**
	 * 정적 바디 프로바이더가 WorldStatic 외에 WorldDynamic 오브젝트도 수집할지. 켜면 움직이는 물리/키네마틱
	 * 바디(엘리베이터·문·플랫폼 등)도 로프 충돌에 참여한다. 프로바이더가 이전 프레임 트랜스폼을 추적해
	 * 표면 속도를 산출하므로 움직이는 표면이 로프를 끌고 substep CCD로 터널링을 막는다.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ToolTip = "WorldStatic 외에 WorldDynamic 바디도 수집합니다(움직이는 플랫폼/문 등). 이전 프레임 트랜스폼으로 표면 속도/CCD 처리."))
	bool bIncludeWorldDynamic = true;

	/**
	 * Aim ray 조준(②③)의 데모 조준 HUD(십자선 + 감김 가능 본 강조 링) 위젯 클래스.
	 * URopeWielderComponent가 bShowAimHudWidget일 때 이 클래스를 생성해 로컬 플레이어 뷰포트에 올린다.
	 * 기본값 = C++ URopeAimWidget(에셋 없이 동작). URopeAimWidget을 부모로 한 WBP로 교체해 리스타일
	 * 가능(내장 페인트를 끄고 자체 비주얼도 가능 — RopeAimWidget.h 참조). 비우면 HUD를 띄우지 않는다.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Demo", meta = (ToolTip = "Aim-ray demo HUD widget class (crosshair + wrappable-bone highlight ring). Spawned by URopeWielderComponent for the local player when bShowAimHudWidget is on. Defaults to the C++ URopeAimWidget (works with no assets); point it at a WBP subclass to restyle. Clear it to disable the HUD."))
	TSoftClassPtr<URopeAimWidget> AimHudWidgetClass;

	/**
	 * Pull 장전/발동 게이지 위젯 클래스(장전 임계까지의 진행 링).
	 * URopeWielderComponent가 bShowPullGaugeWidget일 때 생성해 로컬 플레이어 뷰포트에 올린다.
	 * 기본값 = C++ URopePullGaugeWidget(에셋 없이 동작). WBP 서브클래스로 리스타일 가능.
	 * 비우면 게이지를 띄우지 않는다(게터/이벤트는 그대로 쓸 수 있다).
	 */
	UPROPERTY(config, EditAnywhere, Category = "Demo", meta = (ToolTip = "Pull gauge widget class (ring showing progress toward the pull engage tension). Spawned by URopeWielderComponent for the local player when bShowPullGaugeWidget is on. Defaults to the C++ URopePullGaugeWidget (works with no assets); point it at a WBP subclass to restyle. Clear it to disable the gauge."))
	TSoftClassPtr<URopePullGaugeWidget> PullGaugeWidgetClass;

	/**
	 * 데모 프리셋 순환 목록 — 소비처는 콘솔 명령 Rope.Preset.Cycle / .Apply / .List
	 * (RopePresetDemoCommands.cpp, 비Shipping 전용)뿐이다. 데모/기능 테스트에서 월드의 로프에
	 * 순서대로 적용할 URopePreset 에셋들을 등록한다(soft — 명령 실행 시에만 로드). Shipping
	 * 빌드에서는 명령이 빠져 미소비가 되는 **데모 전용 설정**이다(죽은 설정 아님 — 위 규약의
	 * 의도된 예외). 게임 코드는 URopeComponent::ApplyPreset을 직접 호출하면 된다.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Demo", meta = (ToolTip = "Demo preset cycle list consumed only by the non-shipping console commands Rope.Preset.Cycle / .Apply / .List. Soft references - loaded when a command runs. Game code should call URopeComponent::ApplyPreset directly."))
	TArray<TSoftObjectPtr<URopePreset>> DemoPresets;

	/**
	 * 로프 튜브가 velocity 버퍼에 기록할지. 로프는 매 프레임 정점을 in-place 갱신하지만 per-vertex 변형
	 * velocity가 없어(Movable transform 기반 velocity만 찍힘) 빠른 이동 프레임에 per-object 모션블러가
	 * 로프를 번지게 한다(잔상). 기본 false = velocity 미출력 → 모션블러 대상에서 제외. 트레이드오프:
	 * false면 TSR이 이 픽셀을 카메라 재투영으로 처리해 정지 카메라 + 빠른 로프에서 약한 TSR 고스팅이
	 * 생길 수 있다 → true로 A/B 비교. 씬 프록시 생성 시 1회 읽힌다(변경은 재PIE/렌더 상태 재생성 후 반영).
	 */
	UPROPERTY(config, EditAnywhere, Category = "Rendering", meta = (ToolTip = "로프 튜브의 velocity 버퍼 기록 여부. 끄면(기본) 빠른 이동 시 모션블러 잔상이 사라지고, 켜면 레거시(velocity 출력) 동작. 정지 카메라+빠른 로프의 TSR 고스팅과의 트레이드오프 A/B용."))
	bool bWriteVelocity = false;
};
