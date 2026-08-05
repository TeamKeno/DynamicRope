// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Settings/DynamicRopeSettings.h"
#include "Collision/RopeController.h"
#include "UI/RopeAimWidget.h"
#include "UI/RopePullGaugeWidget.h"

UDynamicRopeSettings::UDynamicRopeSettings()
{
	CategoryName = TEXT("Plugins");
	SectionName = TEXT("Dynamic Rope");

	// The default is the plugin's built-in ARopeController. A project can override it with a subclass or set it to
	// none to disable the automatic spawn. It is a config field, so a value saved in DefaultGame.ini overwrites this default on load.
	StaticBodyControllerClass = ARopeController::StaticClass();

	// The default is the C++ aim HUD widget, which works with no assets. It can be replaced with a widget Blueprint subclass or set to none to disable it.
	AimHudWidgetClass = URopeAimWidget::StaticClass();

	// The default is the C++ pull gauge widget, which works with no assets. It can be replaced with a widget Blueprint or set to none to disable it.
	PullGaugeWidgetClass = URopePullGaugeWidget::StaticClass();
}

const UDynamicRopeSettings* UDynamicRopeSettings::Get()
{
	return GetDefault<UDynamicRopeSettings>();
}
