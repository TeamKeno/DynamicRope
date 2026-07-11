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
			// Tools 메뉴 엔트리
			"ToolMenus",
			// 도크탭 Window 메뉴 카테고리
			"WorkspaceMenuStructure",
			// 플러그인 리소스/스타일 경로
			"Projects",
			"InputCore",
			// 오써링 패널 디테일 뷰(향후)
			"PropertyEditor",
			// 에셋 등록(팩토리)
			"AssetTools",
			// UAssetDefinition (Content Browser 카테고리/색)
			"AssetDefinition",
			// 오써링 패널 3D 프리뷰 뷰포트(조명/바닥 프리뷰 씬)
			"AdvancedPreviewScene",
			// 베이크 부호 판정용 fast winding number(FDynamicMesh3 + AABB/winding 트리)
			"GeometryCore",
			// 베이크 결과 보고(coarsening된 본 목록)용 메시지 로그 리스닝
			"MessageLog",
		});
	}
}
