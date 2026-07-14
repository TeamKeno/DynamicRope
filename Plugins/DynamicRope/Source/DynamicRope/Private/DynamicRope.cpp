// Copyright Epic Games, Inc. All Rights Reserved.

#include "DynamicRope.h"
#include "DynamicRopeLog.h"

// 런타임 로그 카테고리 정의(선언은 DynamicRopeLog.h).
DEFINE_LOG_CATEGORY(LogDynamicRope);
DEFINE_LOG_CATEGORY(LogRopeSolver);
DEFINE_LOG_CATEGORY(LogRopeWrap);
DEFINE_LOG_CATEGORY(LogRopeCollision);

#if WITH_GAMEPLAY_DEBUGGER
#include "GameplayDebugger.h"
#include "Debug/GameplayDebuggerCategory_Rope.h"
#endif

#define LOCTEXT_NAMESPACE "FDynamicRopeModule"

void FDynamicRopeModule::StartupModule()
{
	// 이 코드는 module이 메모리에 로드된 후 실행된다. 정확한 시점은 .uplugin 파일에 module별로 지정된다
	UE_LOG(LogDynamicRope, Log, TEXT("DynamicRope runtime module started."));
#if WITH_GAMEPLAY_DEBUGGER
	IGameplayDebugger& GameplayDebugger = IGameplayDebugger::Get();
	GameplayDebugger.RegisterCategory(TEXT("Rope"),
		IGameplayDebugger::FOnGetCategory::CreateStatic(&FGameplayDebuggerCategory_Rope::MakeInstance),
		EGameplayDebuggerCategoryState::EnabledInGameAndSimulate);
	GameplayDebugger.NotifyCategoriesChanged();
	UE_LOG(LogDynamicRope, Verbose, TEXT("Registered GameplayDebugger category 'Rope'."));
#endif
}

void FDynamicRopeModule::ShutdownModule()
{
	// 이 함수는 shutdown 중에 module을 정리하기 위해 호출될 수 있다.  동적 리로딩을 지원하는 module의 경우,
	// module을 언로드하기 전에 이 함수를 호출한다.
#if WITH_GAMEPLAY_DEBUGGER
	if (IGameplayDebugger::IsAvailable())
	{
		IGameplayDebugger::Get().UnregisterCategory(TEXT("Rope"));
	}
#endif
	UE_LOG(LogDynamicRope, Log, TEXT("DynamicRope runtime module shut down."));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FDynamicRopeModule, DynamicRope)