// Copyright 2026 TeamKeno. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class DynamicRopeProjectEditorTarget : TargetRules
{
	public DynamicRopeProjectEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V6;

		// Compile sources as UTF-8 (Korean comments). Without a BOM or this flag MSVC reads source
		// in the system code page, which breaks per-locale. Clang (Mac/Linux) is UTF-8 by default.
		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			bOverrideBuildEnvironment = true;
			AdditionalCompilerArguments = "/utf-8";
		}

		ExtraModuleNames.AddRange( new string[] { "DynamicRopeProject" } );
	}
}
