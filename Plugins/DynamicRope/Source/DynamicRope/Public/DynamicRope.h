// Copyright 2026 TeamKeno. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

class FDynamicRopeModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation. */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
