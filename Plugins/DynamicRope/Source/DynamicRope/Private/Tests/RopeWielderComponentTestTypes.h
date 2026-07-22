// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#if WITH_TESTS

#include "Gameplay/RopeWielderComponent.h"
#include "RopeWielderComponentTestTypes.generated.h"

/** 실제 throw 경로가 공개 virtual BuildThrowContext 확장 훅을 통과하는지 관찰하는 테스트 probe. */
UCLASS()
class URopeWielderBuildContextProbe final : public URopeWielderComponent
{
	GENERATED_BODY()

public:
	mutable int32 BuildContextCalls = 0;

	virtual FRopeThrowContext BuildThrowContext(const FVector& AimDir) const override
	{
		++BuildContextCalls;
		FRopeThrowContext Result = Super::BuildThrowContext(AimDir);
		Result.ThrowSpeed = 123.0f;
		return Result;
	}
};

#endif // WITH_TESTS
