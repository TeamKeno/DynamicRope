// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

// A thin module holding the GPU solver, being global compute shaders on RDG, the shader virtual path mapping, and the
// rope's vertex factory, whose type must likewise register before shader types initialize.
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
				// The public RopeVertexFactory.h derives from FLocalVertexFactory (Engine), which
				// itself pulls in RenderCore and RHI types, so all three propagate to consumers.
				"Engine",
				"RenderCore",
				"RHI",
			}
			);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				// IPluginManager, for the shader virtual path mapping.
				"Projects",
				// UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData + FGlobalDistanceFieldParameters2
				"Renderer",
			}
			);
	}
}
