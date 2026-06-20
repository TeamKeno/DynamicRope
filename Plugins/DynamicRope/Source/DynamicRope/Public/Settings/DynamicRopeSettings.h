// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "DynamicRopeSettings.generated.h"

/**
 * Project-wide configuration for the Dynamic Rope plugin.
 * Editable under Project Settings > Plugins > Dynamic Rope and saved to DefaultGame.ini.
 */
UCLASS(config = Game, defaultconfig, meta = (DisplayName = "Dynamic Rope"))
class DYNAMICROPE_API UDynamicRopeSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UDynamicRopeSettings();

	/** Convenience accessor for the active (CDO) settings object. */
	static const UDynamicRopeSettings* Get();

	//~ Rope ---------------------------------------------------------------

	/** Default rest length for a newly spawned straight rope. */
	UPROPERTY(EditAnywhere, config, Category = "Rope", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float DefaultRopeLength = 200.0f;

	/** Default tension along the rope. Higher values keep the rope tauter while wrapped. */
	UPROPERTY(EditAnywhere, config, Category = "Rope", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float DefaultTension = 1.0f;

	//~ Wrapping -----------------------------------------------------------

	/** Maximum number of turns a rope may wrap around a single body part before further wrapping is blocked. */
	UPROPERTY(EditAnywhere, config, Category = "Wrapping", meta = (ClampMin = "1", UIMin = "1"))
	int32 MaxWrapTurnsPerBodyPart = 3;

	/** Skeletal-mesh bones that ropes are allowed to wrap around (arms, legs, torso, ...). */
	UPROPERTY(EditAnywhere, config, Category = "Wrapping")
	TArray<FName> WrappableBones;

	/** Distance at which a wrapped rope auto-releases when the character moves away. */
	UPROPERTY(EditAnywhere, config, Category = "Wrapping", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float AutoUnwrapDistance = 500.0f;

	//~ Debug --------------------------------------------------------------

	/** Draw debug visualization of contact points and wrap state in-game. */
	UPROPERTY(EditAnywhere, config, Category = "Debug")
	bool bEnableDebugDraw = false;
};
