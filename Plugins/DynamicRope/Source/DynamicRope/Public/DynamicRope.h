// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

class FDynamicRopeModule : public IModuleInterface
{
public:

	/** IModuleInterface 구현 */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
