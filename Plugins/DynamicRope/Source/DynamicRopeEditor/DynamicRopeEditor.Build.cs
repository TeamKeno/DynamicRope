// Copyright 2026 TeamKeno. All Rights Reserved.

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
			// The Tools menu entry.
			"ToolMenus",
			// The dock tab's Window menu category.
			"WorkspaceMenuStructure",
			// The plugin's resource and style paths.
			"Projects",
			"InputCore",
			// The authoring panel's details view, for future use.
			"PropertyEditor",
			// Asset registration, through the factory.
			"AssetTools",
			// UAssetDefinition, for the content browser category and colour.
			"AssetDefinition",
			// The authoring panel's 3D preview viewport, providing the lit preview scene with a floor.
			"AdvancedPreviewScene",
			// The fast winding number used to decide the sign during a bake, being FDynamicMesh3 with its AABB and winding trees.
			"GeometryCore",
			// Listening to the message log, used to report the bake result as the list of coarsened bones.
			"MessageLog",
		});
	}
}
