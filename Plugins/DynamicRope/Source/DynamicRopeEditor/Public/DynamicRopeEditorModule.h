// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class SDockTab;
class FSpawnTabArgs;

/** Editor module for the Dynamic Rope plugin. Hosts editor tooling, customizations, and visualizers. */
class FDynamicRopeEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	/** Registers the SDF authoring tab entry point under the Tools menu (callback once ToolMenus is ready). */
	void RegisterMenus();

	/** Builds the content of the SDF authoring dock tab. */
	TSharedRef<SDockTab> SpawnSDFAuthoringTab(const FSpawnTabArgs& Args);
};
