// Copyright Epic Games, Inc. All Rights Reserved.

#include "Settings/DynamicRopeSettings.h"
#include "Collision/RopeController.h"
#include "UI/RopeAimWidget.h"

UDynamicRopeSettings::UDynamicRopeSettings()
{
	CategoryName = TEXT("Plugins");
	SectionName = TEXT("Dynamic Rope");

	// 기본값: 플러그인 내장 ARopeController. 프로젝트가 서브클래스로 덮거나 None으로 자동 스폰을 끌 수 있다
	// (config 필드라 DefaultGame.ini에 저장된 값이 있으면 로드 시 이 기본값을 덮어쓴다).
	StaticBodyControllerClass = ARopeController::StaticClass();

	// 기본값: 에셋 없이 동작하는 C++ 조준 HUD 위젯. WBP 서브클래스로 교체하거나 None으로 끌 수 있다.
	AimHudWidgetClass = URopeAimWidget::StaticClass();
}

const UDynamicRopeSettings* UDynamicRopeSettings::Get()
{
	return GetDefault<UDynamicRopeSettings>();
}
