// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class SDockTab;
class FSpawnTabArgs;
class URopeSDFData;

/** Editor module for the Dynamic Rope plugin. Hosts editor tooling and customizations. */
class FDynamicRopeEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	/** Opens the SDF authoring tab, focusing it if it is already open, and points it at the given
	    asset. This is the entry point for double-clicking an asset.
	    The nomad tab is a single instance, so an asset already open there is replaced. */
	void OpenSDFAuthoringTabForAsset(URopeSDFData* InData);

private:
	/** Registers the SDF authoring tab entry point under the Tools menu (callback once ToolMenus is ready). */
	void RegisterMenus();

	/** Builds the content of the SDF authoring dock tab. */
	TSharedRef<SDockTab> SpawnSDFAuthoringTab(const FSpawnTabArgs& Args);
};
