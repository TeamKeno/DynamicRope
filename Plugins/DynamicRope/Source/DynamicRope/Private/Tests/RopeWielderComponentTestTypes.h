// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#if WITH_TESTS

#include "Gameplay/RopeWielderComponent.h"
#include "RopeWielderComponentTestTypes.generated.h"

/** A test probe observing whether the real throw path goes through the public virtual BuildThrowContext extension hook. */
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
