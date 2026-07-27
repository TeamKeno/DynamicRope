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

	// 디테일 패널 프로퍼티 타입 커스터마이즈: FRopeBoneSDFVolume 배열 요소 헤더에 본 이름 표시.
	{
		FPropertyEditorModule& PropertyModule =
			FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
		PropertyModule.RegisterCustomPropertyTypeLayout(
			FRopeBoneSDFVolume::StaticStruct()->GetFName(),
			FOnGetPropertyTypeCustomizationInstance::CreateStatic(&FRopeBoneSDFVolumeCustomization::MakeInstance));

		// URopeComponent / URopePreset 디테일 패널: ResolveMode에 따라 게이트(RopeResolveModeDetails 주석 참조).
		// 프리셋은 컴포넌트 프로퍼티의 미러라 같은 커스터마이즈를 공유한다.
		PropertyModule.RegisterCustomClassLayout(
			URopeComponent::StaticClass()->GetFName(),
			FOnGetDetailCustomizationInstance::CreateStatic(&TRopeResolveModeDetails<URopeComponent>::MakeInstance));
		PropertyModule.RegisterCustomClassLayout(
			URopePreset::StaticClass()->GetFName(),
			FOnGetDetailCustomizationInstance::CreateStatic(&TRopeResolveModeDetails<URopePreset>::MakeInstance));

		PropertyModule.NotifyCustomizationModuleChanged();
	}

	// 베이크 결과(coarsening된 본 목록 등)를 보고할 Message Log 리스닝 등록.
	{
		FMessageLogModule& MessageLogModule = FModuleManager::LoadModuleChecked<FMessageLogModule>("MessageLog");
		FMessageLogInitializationOptions Options;
		// 베이크마다 페이지를 분리해 이력을 남긴다.
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
	// 탭을 열거나(없으면 생성) 앞으로 가져온 뒤 콘텐츠 패널에 타깃을 지정한다.
	// SpawnSDFAuthoringTab이 콘텐츠로 항상 SRopeSDFAuthoringPanel을 넣으므로 캐스트는 안전하다.
	if (TSharedPtr<SDockTab> Tab = FGlobalTabmanager::Get()->TryInvokeTab(RopeSDFAuthoringTabId))
	{
		StaticCastSharedRef<SRopeSDFAuthoringPanel>(Tab->GetContent())->SetTargetAsset(InData);
	}
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FDynamicRopeEditorModule, DynamicRopeEditor)
