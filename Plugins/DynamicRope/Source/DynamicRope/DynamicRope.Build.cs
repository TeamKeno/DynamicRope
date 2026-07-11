// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class DynamicRope : ModuleRules
{
	public DynamicRope(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"DeveloperSettings",
				// ... add other public dependencies that you statically link with here ...
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				// RopeSceneProxy
				"RenderCore",
				// RopeSceneProxy
				"RHI",
				// GPU 솔버(FRopeGPUSolver) — 별도 PostConfigInit 모듈
				"DynamicRopeShaders",
				// URopeWielderComponent 선택적 입력 자동 바인딩
				"EnhancedInput",
				"Slate",
				"SlateCore",
				// UI/ — 플러그인 설명 HUD(ARopePluginInfoHUD + URopePluginInfoWidget)
				"UMG",
				// UI/ — HUD 토글 키(FKey/EKeys, FKey::GetDisplayName)
				"InputCore",
				// ... add private dependencies that you statically link with here ...
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);

		// Gameplay Debugger 카테고리(rope 인트로스펙션) — 의존성 + WITH_GAMEPLAY_DEBUGGER 매크로를
		// 타깃에 맞게 설정한다(shipping에서는 자동으로 빠진다).
		SetupGameplayDebuggerSupport(Target);
	}
}
