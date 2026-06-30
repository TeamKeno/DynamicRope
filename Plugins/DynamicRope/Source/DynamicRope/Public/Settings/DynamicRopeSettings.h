// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"
#include "Engine/DeveloperSettings.h"
#include "DynamicRopeSettings.generated.h"

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
};
