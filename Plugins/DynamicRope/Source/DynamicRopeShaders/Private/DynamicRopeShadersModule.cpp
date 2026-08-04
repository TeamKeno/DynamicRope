// Copyright Epic Games, Inc. All Rights Reserved.

#include "Modules/ModuleManager.h"
#include "DynamicRopeShadersLog.h"
// The custom FX system, the on-demand consumer of the global distance field.
#include "RopeGDFFXSystem.h"
// The global distance field scene view extension.
#include "RopeGDFViewExtension.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
// The post-engine-init delegate, for the timing of the view extension's creation.
#include "Misc/CoreDelegates.h"
// UE_VERSION_OLDER_THAN, the engine version guard, since the delegate's accessor changed in 5.8.
#include "Misc/EngineVersionComparison.h"
// FFXSystemInterface::RegisterCustomFXSystem
#include "FXSystem.h"
// AddShaderSourceDirectoryMapping
#include "ShaderCore.h"
// RopeGPU::IsRuntimeSupported / PrecacheSolverComputePSOs / PrecacheTubeComputePSOs
#include "RopeGPUSolver.h"
#include "RopeTubeBuilder.h"

DEFINE_LOG_CATEGORY(LogDynamicRopeGPU);

// Warms every compute PSO the rope can dispatch — solver, contact detect and tube build, in all their
// permutations — as async precompiles at PostEngineInit, when the global shader map and RHI exist.
// A pipeline missing from the cache is otherwise created by the driver at its first dispatch, inside
// RDG execution, stalling the render/RHI threads for hundreds of milliseconds; the buckets follow the
// rope's node count, so that stall surfaces exactly on a preset switch or the first throw after one.
static void PrecacheRopeComputePSOs()
{
	// The same gate as the runtime GPU path: no renderable RHI (cook, -nullrhi, server) or a feature
	// level below SM5 means none of these kernels can ever dispatch, so there is nothing to warm.
	if (!RopeGPU::IsRuntimeSupported())
	{
		return;
	}
	RopeGPU::PrecacheSolverComputePSOs();
	RopeGPU::PrecacheTubeComputePSOs();
}

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
		// In 5.8 the public OnPostEngineInit member was replaced by the GetOnPostEngineInit() accessor, hence the version guard.
#if UE_VERSION_OLDER_THAN(5, 8, 0)
		FCoreDelegates::OnPostEngineInit.AddStatic(&FRopeGDFViewExtension::EnsureRegistered);
		FCoreDelegates::OnPostEngineInit.AddStatic(&PrecacheRopeComputePSOs);
#else
		FCoreDelegates::GetOnPostEngineInit().AddStatic(&FRopeGDFViewExtension::EnsureRegistered);
		FCoreDelegates::GetOnPostEngineInit().AddStatic(&PrecacheRopeComputePSOs);
#endif
	}

	virtual void ShutdownModule() override
	{
		FRopeGDFViewExtension::Shutdown();
		FFXSystemInterface::UnregisterCustomFXSystem(FRopeGDFFXSystem::Name);
	}
};

IMPLEMENT_MODULE(FDynamicRopeShadersModule, DynamicRopeShaders)
