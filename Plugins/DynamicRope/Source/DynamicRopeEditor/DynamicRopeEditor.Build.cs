// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class DynamicRopeEditor : ModuleRules
{
	public DynamicRopeEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Slate",
			"SlateCore",
			"UnrealEd",
			"DynamicRope",
		});
	}
}
