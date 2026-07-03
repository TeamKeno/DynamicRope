// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class SDockTab;
class FSpawnTabArgs;
class URopeSDFData;

/** Editor module for the Dynamic Rope plugin. Hosts editor tooling, customizations, and visualizers. */
class FDynamicRopeEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	/** SDF 오써링 탭을 열고(이미 열려 있으면 포커스) 타깃 에셋을 지정한다. 에셋 더블클릭 진입점.
	    노마드 탭은 단일 인스턴스 — 다른 에셋이 열려 있었다면 타깃이 교체된다. */
	void OpenSDFAuthoringTabForAsset(URopeSDFData* InData);

private:
	/** Registers the SDF authoring tab entry point under the Tools menu (callback once ToolMenus is ready). */
	void RegisterMenus();

	/** Builds the content of the SDF authoring dock tab. */
	TSharedRef<SDockTab> SpawnSDFAuthoringTab(const FSpawnTabArgs& Args);
};
