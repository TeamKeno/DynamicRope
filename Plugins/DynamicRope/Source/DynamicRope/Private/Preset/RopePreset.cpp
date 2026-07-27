// Copyright Epic Games, Inc. All Rights Reserved.

#include "Preset/RopePreset.h"

#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

URopePreset::URopePreset()
{
	// The same default material as URopeComponent's constructor, being hemp rope, which is a mirroring contract
	// described in the header. If the asset is missing, meaning the find did not succeed, it stays null and falls back
	// to the same grey as the component, which is safe for builds and cooking.
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> DefaultRopeMaterial(
		TEXT("/DynamicRope/Materials/M_RopeDefault.M_RopeDefault"));
	if (DefaultRopeMaterial.Succeeded())
	{
		RopeMaterial = DefaultRopeMaterial.Object;
	}
}

#if WITH_EDITOR
#include "Misc/DataValidation.h"

EDataValidationResult URopePreset::IsDataValid(FDataValidationContext& Context) const
{
	EDataValidationResult Result = Super::IsDataValid(Context);

	// SetRopeLength clamps to the range from the minimum to the maximum rope length, so an inverted range is an authoring mistake.
	if (MinRopeLength > RopeLength)
	{
		Context.AddWarning(FText::Format(
			NSLOCTEXT("RopePreset", "MinLengthAboveMax",
				"MinRopeLength ({0}) is greater than RopeLength ({1}) - reel-in clamping will misbehave."),
			FText::AsNumber(MinRopeLength), FText::AsNumber(RopeLength)));
	}

	return Result;
}
#endif // WITH_EDITOR
