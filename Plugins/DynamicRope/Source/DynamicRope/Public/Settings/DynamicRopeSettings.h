// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"
#include "Engine/DeveloperSettings.h"
#include "DynamicRopeSettings.generated.h"

class ARopeController;

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

	/** Wrapping 상태로 진입한 뒤 tail 방향 node들의 목표 surface path를 만드는 전역 방식. */
	UPROPERTY(config, EditAnywhere, Category = "Wrapping", meta = (ToolTip = "Wrapping 상태에서 tail node들을 어떤 surface path로 감기게 만들지 선택합니다. 기본값은 의도적으로 원주를 돌면서 SDF 굴곡을 따라가는 Surface Vector Field입니다."))
	ERopeWrappingPathMode WrappingPathMode = ERopeWrappingPathMode::SurfaceVectorField;

	/**
	 * 게임/PIE 월드 시작 시 URopeSimSubsystem이 자동 스폰하는 로프 매니저 액터 클래스. 이 액터가
	 * 정적 월드 충돌용 URopeStaticBodyProvider를 품는다 — 레벨마다 프로바이더를 수동 배치하지 않아도
	 * "월드당 정확히 1개"를 보장한다. ARopeController를 서브클래스해 MaxColliders 등을 조정할 수 있다.
	 * 비우면(None) 자동 스폰을 끈다(수동 배치를 원하는 프로젝트용).
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ToolTip = "게임 시작 시 자동 스폰되는 로프 매니저 액터 클래스(정적 월드 충돌 프로바이더 호스트). 비우면 자동 스폰을 끕니다."))
	TSoftClassPtr<ARopeController> StaticBodyControllerClass;

	/**
	 * 정적 바디 프로바이더가 프레임당 수집할 콜라이더 상한(GPU solve 커널이 노드×substep마다 콜라이더 전량을
	 * 루프하므로 밀집 씬 폭주 방지). 프로바이더가 매 프레임 이 값을 직접 읽는 단일 소스 — 중복 방지 가드가
	 * "월드당 프로바이더 1개"를 강제하므로 컴포넌트별 예산은 불필요하다. 런타임 변경도 즉시 반영된다.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ClampMin = "1", ToolTip = "정적 바디 프로바이더의 프레임당 콜라이더 상한(단일 소스 — 월드당 프로바이더 1개라 전역 관리)."))
	int32 StaticBodyMaxColliders = 128;

	/**
	 * 정적 바디 프로바이더의 컨벡스 1개당 평면 수 상한. 이 수를 넘는 복잡한 컨벡스는 ElemBox OBB로 폴백한다.
	 * StaticBodyMaxColliders와 마찬가지로 프로바이더가 직접 읽는 단일 소스다. (전체 콜라이더 개수는 위에서 별도 제한.)
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ClampMin = "4", ToolTip = "컨벡스당 평면 수 상한(초과분은 OBB 폴백). 단일 소스 — 프로바이더가 직접 읽습니다."))
	int32 StaticBodyMaxConvexPlanes = 32;
};
