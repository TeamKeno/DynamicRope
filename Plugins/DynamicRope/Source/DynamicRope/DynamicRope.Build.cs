// Copyright 2026 TeamKeno. All Rights Reserved.

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
				// Convention: a module included by a public header must be listed here, publicly. Listing it privately
				// leaves a downstream game module that includes that header without the include path, and it fails to compile.
				"Core",
				"DeveloperSettings",
				// UObject/Interface.h for IRopeColliderProvider, and the UObject family across every public header.
				"CoreUObject",
				// Components/MeshComponent.h for URopeComponent, GameFramework/Actor.h for ARopeController and so on.
				"Engine",
				// Subsystem/RopeSimSubsystem.h includes RopeGPUSolver.h, holding an FRopeGPUSolver by value.
				// That header carries the only public extension seam, RegisterColliderProvider.
				"DynamicRopeShaders",
				// UI/ — Blueprint/UserWidget.h(URopeAimWidget, URopePluginInfoWidget)
				"UMG",
				// UI/ — the Slate types in the widget paint signatures: FGeometry, FSlateRect and FSlateWindowElementList.
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
				// The optional automatic input binding of URopeWielderComponent; the public header forward-declares it alone, so this stays private.
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

		// The gameplay debugger category, for rope introspection. This sets both the dependency and the
		// WITH_GAMEPLAY_DEBUGGER macro to match the target, so it drops out automatically in shipping.
		SetupGameplayDebuggerSupport(Target);
	}
}
