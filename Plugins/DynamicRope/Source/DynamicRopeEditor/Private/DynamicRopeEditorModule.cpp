// Copyright Epic Games, Inc. All Rights Reserved.

#include "DynamicRopeEditorModule.h"
#include "SDF/SRopeSDFAuthoringPanel.h"
#include "SDF/RopeSDFVisualizer.h"
#include "Visualizers/RopeComponentVisualizer.h"
#include "RopeComponent.h"
#include "Collision/SDF/RopeSDFProvider.h"

#include "Framework/Docking/TabManager.h"
#include "Widgets/Docking/SDockTab.h"
#include "Textures/SlateIcon.h"
#include "Styling/AppStyle.h"
#include "ToolMenus.h"
#include "WorkspaceMenuStructure.h"
#include "WorkspaceMenuStructureModule.h"
#include "Editor/UnrealEdEngine.h"
#include "UnrealEdGlobals.h"

#define LOCTEXT_NAMESPACE "FDynamicRopeEditorModule"

// ID of the plugin-specific nomad dock tab launched from the top-left menu.
static const FName RopeSDFAuthoringTabId(TEXT("DynamicRopeSDFAuthoring"));

void FDynamicRopeEditorModule::StartupModule()
{
	// Register the plugin-specific nomad tab. Visible in the Window menu (Tools category) and openable via TryInvokeTab.
	FGlobalTabmanager::Get()->RegisterNomadTabSpawner(
			RopeSDFAuthoringTabId,
			FOnSpawnTab::CreateRaw(this, &FDynamicRopeEditorModule::SpawnSDFAuthoringTab))
		.SetDisplayName(LOCTEXT("SDFAuthoringTabTitle", "Rope SDF Authoring"))
		.SetTooltipText(LOCTEXT("SDFAuthoringTabTooltip", "DynamicRope: per-bone SDF authoring/baking"))
		.SetGroup(WorkspaceMenu::GetMenuStructure().GetToolsCategory())
		.SetIcon(FSlateIcon(FAppStyle::GetAppStyleSetName(), "ClassIcon.DataAsset"));

	// Add the Tools menu entry once ToolMenus is ready.
	UToolMenus::RegisterStartupCallback(
		FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FDynamicRopeEditorModule::RegisterMenus));

	// Editor viewport visualizers (drawn on selection).
	if (GUnrealEd)
	{
		GUnrealEd->RegisterComponentVisualizer(URopeComponent::StaticClass()->GetFName(),
			MakeShared<FRopeComponentVisualizer>());
		GUnrealEd->RegisterComponentVisualizer(URopeSDFProvider::StaticClass()->GetFName(),
			MakeShared<FRopeSDFVisualizer>());
	}
}

void FDynamicRopeEditorModule::ShutdownModule()
{
	UToolMenus::UnRegisterStartupCallback(this);
	UToolMenus::UnregisterOwner(this);

	if (GUnrealEd)
	{
		GUnrealEd->UnregisterComponentVisualizer(URopeComponent::StaticClass()->GetFName());
		GUnrealEd->UnregisterComponentVisualizer(URopeSDFProvider::StaticClass()->GetFName());
	}

	if (FSlateApplication::IsInitialized())
	{
		FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(RopeSDFAuthoringTabId);
	}
}

void FDynamicRopeEditorModule::RegisterMenus()
{
	FToolMenuOwnerScoped OwnerScoped(this);

	UToolMenu* ToolsMenu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Tools");
	FToolMenuSection& Section = ToolsMenu->FindOrAddSection("DynamicRope");
	Section.AddMenuEntry(
		"OpenRopeSDFAuthoring",
		LOCTEXT("OpenSDFAuthoring", "Rope SDF Authoring"),
		LOCTEXT("OpenSDFAuthoringTooltip", "Open the per-bone SDF authoring/baking panel."),
		FSlateIcon(FAppStyle::GetAppStyleSetName(), "ClassIcon.DataAsset"),
		FUIAction(FExecuteAction::CreateLambda([]()
		{
			FGlobalTabmanager::Get()->TryInvokeTab(RopeSDFAuthoringTabId);
		})));
}

TSharedRef<SDockTab> FDynamicRopeEditorModule::SpawnSDFAuthoringTab(const FSpawnTabArgs& Args)
{
	return SNew(SDockTab)
		.TabRole(ETabRole::NomadTab)
		[
			SNew(SRopeSDFAuthoringPanel)
		];
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FDynamicRopeEditorModule, DynamicRopeEditor)
