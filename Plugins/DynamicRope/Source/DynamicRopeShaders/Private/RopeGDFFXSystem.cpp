// Copyright 2026 TeamKeno. All Rights Reserved.

#include "RopeGDFFXSystem.h"
#include "RopeGPUSolverRegistry.h"

const FName FRopeGDFFXSystem::Name(TEXT("DynamicRopeGDF"));

bool FRopeGDFFXSystem::UsesGlobalDistanceField() const
{
	// The scene is filled in by FFXSystemSet through SetSceneInterface immediately after construction. If this scene has an active rope using the global distance field, the field's build is requested.
	return RopeGDF::IsGDFActive(GetSceneInterface());
}

FFXSystemInterface* CreateRopeGDFFXSystem(ERHIFeatureLevel::Type InFeatureLevel, EShaderPlatform InShaderPlatform, FGPUSortManager* InGPUSortManager)
{
	// FFXSystemSet takes ownership, as a shared pointer with a custom deleter, and the engine destroys it on the render thread.
	return new FRopeGDFFXSystem(InFeatureLevel, InShaderPlatform, InGPUSortManager);
}
