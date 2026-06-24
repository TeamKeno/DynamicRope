// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/** Dynamic Rope 플러그인의 에디터 module. 에디터 툴링, 커스터마이제이션, 비주얼라이저를 호스팅한다. */
class FDynamicRopeEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
