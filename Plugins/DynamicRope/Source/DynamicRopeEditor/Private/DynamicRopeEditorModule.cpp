// Copyright Epic Games, Inc. All Rights Reserved.

#include "DynamicRopeEditorModule.h"
#include "DynamicRopeEditorLog.h"
#include "SDF/SRopeSDFAuthoringPanel.h"
#include "SDF/RopeBoneSDFVolumeCustomization.h"
#include "Details/RopeResolveModeDetails.h"
#include "RopeComponent.h"
#include "Preset/RopePreset.h"
#include "Collision/SDF/RopeSDFProvider.h"
#include "Collision/SDF/RopeSDFData.h"

#include "PropertyEditorModule.h"
#include "Modules/ModuleManager.h"
#include "MessageLogModule.h"

#include "Framework/Application/SlateApplication.h"
#include "Framework/Docking/TabManager.h"
#include "Widgets/Docking/SDockTab.h"
#include "Textures/SlateIcon.h"
#include "Styling/AppStyle.h"
#include "ToolMenus.h"
#include "WorkspaceMenuStructure.h"
#include "WorkspaceMenuStructureModule.h"

// Editor log category definitions (declarations in DynamicRopeEditorLog.h).
DEFINE_LOG_CATEGORY(LogDynamicRopeEditor);
DEFINE_LOG_CATEGORY(LogRopeSDFBake);

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

	// The details panel property type customization that shows the bone name on an FRopeBoneSDFVolume array element's header.
	{
		FPropertyEditorModule& PropertyModule =
			FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
		PropertyModule.RegisterCustomPropertyTypeLayout(
			FRopeBoneSDFVolume::StaticStruct()->GetFName(),
			FOnGetPropertyTypeCustomizationInstance::CreateStatic(&FRopeBoneSDFVolumeCustomization::MakeInstance));

		// The URopeComponent and URopePreset details panels, gated on the resolve mode; see the comments in RopeResolveModeDetails.
		// The preset mirrors the component's properties and therefore shares the same customization.
		PropertyModule.RegisterCustomClassLayout(
			URopeComponent::StaticClass()->GetFName(),
			FOnGetDetailCustomizationInstance::CreateStatic(&TRopeResolveModeDetails<URopeComponent>::MakeInstance));
		PropertyModule.RegisterCustomClassLayout(
			URopePreset::StaticClass()->GetFName(),
			FOnGetDetailCustomizationInstance::CreateStatic(&TRopeResolveModeDetails<URopePreset>::MakeInstance));

		PropertyModule.NotifyCustomizationModuleChanged();
	}

	// Registers listening on the message log used to report bake results, such as the list of coarsened bones.
	{
		FMessageLogModule& MessageLogModule = FModuleManager::LoadModuleChecked<FMessageLogModule>("MessageLog");
		FMessageLogInitializationOptions Options;
		// A separate page per bake, which keeps the history.
		Options.bShowPages = true;
		Options.bAllowClear = true;
		Options.bShowFilters = true;
		MessageLogModule.RegisterLogListing(RopeSDFMessageLogName, LOCTEXT("RopeSDFLogLabel", "Dynamic Rope SDF"), Options);
	}

	UE_LOG(LogDynamicRopeEditor, Log, TEXT("DynamicRopeEditor module started (SDF authoring tab registered)."));
}

void FDynamicRopeEditorModule::ShutdownModule()
{
	UToolMenus::UnRegisterStartupCallback(this);
	UToolMenus::UnregisterOwner(this);

	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule =
			FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
		PropertyModule.UnregisterCustomPropertyTypeLayout(FRopeBoneSDFVolume::StaticStruct()->GetFName());
		PropertyModule.UnregisterCustomClassLayout(URopeComponent::StaticClass()->GetFName());
		PropertyModule.UnregisterCustomClassLayout(URopePreset::StaticClass()->GetFName());
		PropertyModule.NotifyCustomizationModuleChanged();
	}

	if (FSlateApplication::IsInitialized())
	{
		FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(RopeSDFAuthoringTabId);
	}

	if (FModuleManager::Get().IsModuleLoaded("MessageLog"))
	{
		FModuleManager::GetModuleChecked<FMessageLogModule>("MessageLog")
			.UnregisterLogListing(RopeSDFMessageLogName);
	}

	UE_LOG(LogDynamicRopeEditor, Log, TEXT("DynamicRopeEditor module shut down."));
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

void FDynamicRopeEditorModule::OpenSDFAuthoringTabForAsset(URopeSDFData* InData)
{
	// Opens the tab, creating it if there is none, brings it to the front and then sets the target on the content panel.
	// SpawnSDFAuthoringTab always puts an SRopeSDFAuthoringPanel in as the content, so the cast is safe.
	if (TSharedPtr<SDockTab> Tab = FGlobalTabmanager::Get()->TryInvokeTab(RopeSDFAuthoringTabId))
	{
		StaticCastSharedRef<SRopeSDFAuthoringPanel>(Tab->GetContent())->SetTargetAsset(InData);
	}
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FDynamicRopeEditorModule, DynamicRopeEditor)
