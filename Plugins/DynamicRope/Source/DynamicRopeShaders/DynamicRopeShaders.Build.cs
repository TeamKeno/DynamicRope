// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

// GPU 솔버(글로벌 컴퓨트 셰이더 + RDG)와 .usf 가상경로 매핑만 담는 얇은 모듈.
// LoadingPhase=PostConfigInit(.uplugin): 글로벌 셰이더 컴파일(InitializeShaderTypes) 이전에 로드되어
// 셰이더 디렉터리 매핑을 등록한다. 게임플레이/런타임 모듈(DynamicRope, Default)과 분리해 초기화 시점을 독립시킨다.
// DynamicRope 런타임 타입에 의존하지 않는다(POD FRopeGPUJob) → 순환 의존 없음.
public class DynamicRopeShaders : ModuleRules
{
	public DynamicRopeShaders(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				// public 헤더(RopeGPUSolver.h)가 RenderGraphFwd.h를 include하므로 public이어야 한다 —
				// private로 두면 이 모듈의 헤더를 include하는 하위 모듈이 컴파일되지 않는다.
				"RenderCore",
			}
			);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				// GPU 버퍼 / 리드백
				"RHI",
				// IPluginManager — .usf 가상경로 매핑
				"Projects",
				// FSceneViewExtension / FFXSystemInterface (GDF 월드 충돌)
				"Engine",
				// UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData + FGlobalDistanceFieldParameters2
				"Renderer",
			}
			);
	}
}
