// Copyright Epic Games, Inc. All Rights Reserved.

#include "Modules/ModuleManager.h"
#include "DynamicRopeShadersLog.h"
// The custom FX system, the on-demand consumer of the global distance field.
#include "RopeGDFFXSystem.h"
// The global distance field scene view extension.
#include "RopeGDFViewExtension.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
// OnPostEngineInit, for the timing of the view extension's creation.
#include "Misc/CoreDelegates.h"
// FFXSystemInterface::RegisterCustomFXSystem
#include "FXSystem.h"
// AddShaderSourceDirectoryMapping
#include "ShaderCore.h"

DEFINE_LOG_CATEGORY(LogDynamicRopeGPU);

// A thin module whose only responsibility is mapping the shader virtual path before the global shaders are compiled.
// The global shader types declared through IMPLEMENT_GLOBAL_SHADER are registered by static initialization in
// RopeGPUSolver.cpp, and because this module loads at PostConfigInit both the types and the mapping are ready before
// InitializeShaderTypes runs.
class FDynamicRopeShadersModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		// Maps the virtual "/Plugin/DynamicRope" to the plugin's Shaders directory. The actual shaders are under
		// Shaders/Private/*.usf and IMPLEMENT_GLOBAL_SHADER refers to them as
		// "/Plugin/DynamicRope/Private/RopeXPBD.usf", following the engine's /Engine/Private convention.
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

		// Registers the custom FX system, the on-demand consumer of the global distance field. A scene's FFXSystemSet
		// is built once when the world and scene are created, so registering at PostConfigInit, before any world
		// exists, is what makes it a sibling in every scene thereafter.
		FFXSystemInterface::RegisterCustomFXSystem(
			FRopeGDFFXSystem::Name,
			FCreateCustomFXSystemDelegate::CreateStatic(&CreateRopeGDFFXSystem));

		// The view extension needs GEngine, so it is created after engine initialization; PostConfigInit is too early.
		FCoreDelegates::OnPostEngineInit.AddStatic(&FRopeGDFViewExtension::EnsureRegistered);
	}

	virtual void ShutdownModule() override
	{
		FRopeGDFViewExtension::Shutdown();
		FFXSystemInterface::UnregisterCustomFXSystem(FRopeGDFFXSystem::Name);
	}
};

IMPLEMENT_MODULE(FDynamicRopeShadersModule, DynamicRopeShaders)
