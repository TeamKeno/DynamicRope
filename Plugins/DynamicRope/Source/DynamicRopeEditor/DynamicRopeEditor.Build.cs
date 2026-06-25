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
			"ToolMenus",            // Tools 메뉴 엔트리
			"WorkspaceMenuStructure", // 도크탭 Window 메뉴 카테고리
			"Projects",             // 플러그인 리소스/스타일 경로
			"InputCore",
			"PropertyEditor",       // 오써링 패널 디테일 뷰(향후)
			"AssetTools",           // 에셋 등록(팩토리)
			"AssetDefinition",      // UAssetDefinition (Content Browser 카테고리/색)
		});
	}
}
