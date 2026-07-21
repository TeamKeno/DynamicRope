// Copyright Epic Games, Inc. All Rights Reserved.

#include "Preset/RopePreset.h"

#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

URopePreset::URopePreset()
{
	// URopeComponent 생성자와 같은 기본 머티리얼(헴프 밧줄) — 미러 계약(헤더 주석). 에셋이
	// 없으면(.Succeeded()==false) null 유지 → 컴포넌트와 같은 회색 폴백(빌드/쿠킹 안전).
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

	// SetRopeLength가 [MinRopeLength, RopeLength]로 클램프하므로 역전 구간은 저작 실수다.
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
