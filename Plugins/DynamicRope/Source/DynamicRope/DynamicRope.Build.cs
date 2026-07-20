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
				// [규약] public 헤더가 include하는 모듈은 반드시 여기(public)에 둔다 — private에 두면
				// 그 헤더를 include하는 하위(게임) 모듈이 include 경로를 못 받아 컴파일에 실패한다.
				"Core",
				"DeveloperSettings",
				// UObject/Interface.h(IRopeColliderProvider), 전 public 헤더의 UObject 계열
				"CoreUObject",
				// Components/MeshComponent.h(URopeComponent), GameFramework/Actor.h(ARopeController) 등
				"Engine",
				// Subsystem/RopeSimSubsystem.h가 RopeGPUSolver.h를 include(FRopeGPUSolver 값 멤버) —
				// 이 헤더가 유일한 공개 확장 seam(RegisterColliderProvider)을 담고 있다.
				"DynamicRopeShaders",
				// UI/ — Blueprint/UserWidget.h(URopeAimWidget, URopePluginInfoWidget)
				"UMG",
				// UI/ — Slate 타입(위젯 페인트 시그니처: FGeometry/FSlateRect/FSlateWindowElementList)
				"Slate",
				"SlateCore",
				// UI/RopePluginInfoHUD.h — InputCoreTypes.h(FKey)
				"InputCore",
			}
			);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				// RopeSceneProxy
				"RenderCore",
				// RopeSceneProxy
				"RHI",
				// URopeWielderComponent 선택적 입력 자동 바인딩(public 헤더는 전방선언만 — private 유지)
				"EnhancedInput",
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
