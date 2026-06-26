// Copyright Epic Games, Inc. All Rights Reserved.

#include "Modules/ModuleManager.h"
#include "DynamicRopeShadersLog.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
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
	}

	virtual void ShutdownModule() override
	{
	}
};

IMPLEMENT_MODULE(FDynamicRopeShadersModule, DynamicRopeShaders)
