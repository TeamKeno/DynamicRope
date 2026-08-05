// Copyright 2026 TeamKeno. All Rights Reserved.

#pragma once

// The probe is declared unconditionally on purpose. UnrealHeaderTool understands only a fixed set of
// preprocessor conditions, and WITH_TESTS is not one of them before 5.7: on an older engine UHT skips the
// whole block, never registers the class, and GENERATED_BODY() is then left with nothing to expand to.
// Guarding a UCLASS declaration this way is therefore not portable across engine versions, so the class is
// always compiled and its only caller stays behind WITH_DEV_AUTOMATION_TESTS instead. It declares no
// properties, and the class specifiers below keep it out of the editor's pickers.

#include "Gameplay/RopeWielderComponent.h"
#include "RopeWielderComponentTestTypes.generated.h"

/** A test probe observing whether the real throw path goes through the public virtual BuildThrowContext
 *  extension hook, and which reason a discarded throw reports. */
UCLASS(NotBlueprintable, Transient, meta = (Hidden))
class URopeWielderBuildContextProbe final : public URopeWielderComponent
{
	GENERATED_BODY()

public:
	mutable int32 BuildContextCalls = 0;
	int32 ThrowRejectCalls = 0;
	ERopeThrowRejectReason LastThrowRejectReason = ERopeThrowRejectReason::Gated;

	virtual FRopeThrowContext BuildThrowContext(const FVector& AimDir) const override
	{
		++BuildContextCalls;
		FRopeThrowContext Result = Super::BuildThrowContext(AimDir);
		Result.ThrowSpeed = 123.0f;
		return Result;
	}

	virtual void NotifyThrowRejected(ERopeThrowRejectReason Reason) override
	{
		++ThrowRejectCalls;
		LastThrowRejectReason = Reason;
	}
};
