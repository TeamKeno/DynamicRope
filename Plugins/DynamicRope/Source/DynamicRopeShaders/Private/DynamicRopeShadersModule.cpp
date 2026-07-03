// Copyright Epic Games, Inc. All Rights Reserved.

#include "Modules/ModuleManager.h"
#include "DynamicRopeShadersLog.h"
#include "RopeGDFFXSystem.h"       // 커스텀 FX 시스템(GDF 온디맨드 소비자)
#include "RopeGDFViewExtension.h"  // GDF 씬 뷰 확장
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Misc/CoreDelegates.h"    // OnPostEngineInit (뷰 확장 생성 타이밍)
#include "FXSystem.h"              // FFXSystemInterface::RegisterCustomFXSystem
#include "ShaderCore.h" // AddShaderSourceDirectoryMapping

DEFINE_LOG_CATEGORY(LogDynamicRopeGPU);

// 얇은 모듈: 유일한 책임은 글로벌 셰이더 컴파일 이전에 .usf 가상경로를 매핑하는 것.
// (글로벌 셰이더 타입 IMPLEMENT_GLOBAL_SHADER는 RopeGPUSolver.cpp의 static init에서 등록된다 —
//  이 모듈이 PostConfigInit에 로드되므로 InitializeShaderTypes 이전에 타입+매핑이 모두 준비된다.)
class FDynamicRopeShadersModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		// 가상 "/Plugin/DynamicRope" -> <플러그인>/Shaders. 실제 셰이더는 Shaders/Private/*.usf 이며
		// IMPLEMENT_GLOBAL_SHADER는 "/Plugin/DynamicRope/Private/RopeXPBD.usf"로 참조한다(엔진 /Engine/Private 관례).
		if (TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("DynamicRope")))
		{
			const FString ShaderDir = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders"));
			AddShaderSourceDirectoryMapping(TEXT("/Plugin/DynamicRope"), ShaderDir);
			UE_LOG(LogDynamicRopeGPU, Verbose, TEXT("Mapped shader virtual dir /Plugin/DynamicRope -> %s"), *ShaderDir);
		}
		else
		{
			UE_LOG(LogDynamicRopeGPU, Warning, TEXT("Could not find 'DynamicRope' plugin to map shader directory — GPU solver shaders will fail to compile."));
		}

		// 커스텀 FX 시스템을 등록한다(GDF 온디맨드 소비자). 씬의 FFXSystemSet은 월드/씬 생성 시 1회 빌드되므로
		// PostConfigInit(월드 생성 전)에 등록해야 이후 모든 씬에 sibling으로 들어간다.
		FFXSystemInterface::RegisterCustomFXSystem(
			FRopeGDFFXSystem::Name,
			FCreateCustomFXSystemDelegate::CreateStatic(&CreateRopeGDFFXSystem));

		// 뷰 확장은 GEngine이 필요하므로 엔진 초기화 이후 생성한다(PostConfigInit은 너무 이름).
		FCoreDelegates::OnPostEngineInit.AddStatic(&FRopeGDFViewExtension::EnsureRegistered);
	}

	virtual void ShutdownModule() override
	{
		FRopeGDFViewExtension::Shutdown();
		FFXSystemInterface::UnregisterCustomFXSystem(FRopeGDFFXSystem::Name);
	}
};

IMPLEMENT_MODULE(FDynamicRopeShadersModule, DynamicRopeShaders)
