// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeGDFFXSystem.h"
#include "RopeGPUSolverRegistry.h"

const FName FRopeGDFFXSystem::Name(TEXT("DynamicRopeGDF"));

bool FRopeGDFFXSystem::UsesGlobalDistanceField() const
{
	// 씬은 FFXSystemSet이 생성 직후 SetSceneInterface로 채운다. 이 씬에 GDF 로프가 활성이면 GDF 빌드 요구.
	return RopeGDF::IsGDFActive(GetSceneInterface());
}

FFXSystemInterface* CreateRopeGDFFXSystem(ERHIFeatureLevel::Type InFeatureLevel, EShaderPlatform InShaderPlatform, FGPUSortManager* InGPUSortManager)
{
	// FFXSystemSet이 소유권을 가져가며(shared ptr + custom deleter), 파괴는 엔진이 렌더 스레드에서 처리한다.
	return new FRopeGDFFXSystem(InFeatureLevel, InShaderPlatform, InGPUSortManager);
}
