// Copyright 2026 TeamKeno. All Rights Reserved.

#include "DynamicRope.h"
#include "DynamicRopeLog.h"

// The runtime log category definition; the declaration is in DynamicRopeLog.h.
DEFINE_LOG_CATEGORY(LogDynamicRope);
DEFINE_LOG_CATEGORY(LogRopeSolver);
DEFINE_LOG_CATEGORY(LogRopeWrap);
DEFINE_LOG_CATEGORY(LogRopeCollision);

#if WITH_GAMEPLAY_DEBUGGER
#include "GameplayDebugger.h"
#include "Debug/GameplayDebuggerCategory_Rope.h"
#include "Debug/GameplayDebuggerCategory_RopePerf.h"
#endif

#define LOCTEXT_NAMESPACE "FDynamicRopeModule"

void FDynamicRopeModule::StartupModule()
{
	// This code runs after the module is loaded into memory. The exact timing is specified per module in the .uplugin file.
	UE_LOG(LogDynamicRope, Log, TEXT("DynamicRope runtime module started."));
#if WITH_GAMEPLAY_DEBUGGER
	IGameplayDebugger& GameplayDebugger = IGameplayDebugger::Get();
	GameplayDebugger.RegisterCategory(TEXT("Rope"),
		IGameplayDebugger::FOnGetCategory::CreateStatic(&FGameplayDebuggerCategory_Rope::MakeInstance),
		EGameplayDebuggerCategoryState::EnabledInGameAndSimulate);
	// The world-wide rope performance and throttle overview, as a separate category. Off by default and toggled on when needed.
	GameplayDebugger.RegisterCategory(TEXT("RopePerf"),
		IGameplayDebugger::FOnGetCategory::CreateStatic(&FGameplayDebuggerCategory_RopePerf::MakeInstance),
		EGameplayDebuggerCategoryState::Disabled);
	GameplayDebugger.NotifyCategoriesChanged();
	UE_LOG(LogDynamicRope, Verbose, TEXT("Registered GameplayDebugger category 'Rope'."));
#endif
}

void FDynamicRopeModule::ShutdownModule()
{
	// This function may be called to clean the module up during shutdown. For a module that supports dynamic
	// reloading, it is called before the module is unloaded.
#if WITH_GAMEPLAY_DEBUGGER
	if (IGameplayDebugger::IsAvailable())
	{
		IGameplayDebugger::Get().UnregisterCategory(TEXT("Rope"));
		IGameplayDebugger::Get().UnregisterCategory(TEXT("RopePerf"));
	}
#endif
	UE_LOG(LogDynamicRope, Log, TEXT("DynamicRope runtime module shut down."));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FDynamicRopeModule, DynamicRope)