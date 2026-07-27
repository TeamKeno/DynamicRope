// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

// A thin module holding only the GPU solver, being global compute shaders on RDG, and the shader virtual path mapping.
// Its loading phase in the .uplugin is PostConfigInit, so it loads before the global shaders are compiled in
// InitializeShaderTypes and registers the shader directory mapping. Keeping it separate from the gameplay and runtime
// module, DynamicRope at the Default phase, makes its initialization timing independent.
// It does not depend on DynamicRope's runtime types, working from the POD FRopeGPUJob, so there is no circular dependency.
public class DynamicRopeShaders : ModuleRules
{
	public DynamicRopeShaders(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
			}
			);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				// The global shader and RDG implementations are used from private sources alone.
				"RenderCore",
				// GPU buffers and readback.
				"RHI",
				// IPluginManager, for the shader virtual path mapping.
				"Projects",
				// FSceneViewExtension and FFXSystemInterface, for world collision against the global distance field.
				"Engine",
				// UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData + FGlobalDistanceFieldParameters2
				"Renderer",
			}
			);
	}
}
