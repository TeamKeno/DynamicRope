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

	// 모드 조합 계약(①②=BareWrap만, ③=Pierce/Cinch만) — 단일 소스 RopeWrapModes::.
	if (!RopeWrapModes::IsEngagementAllowed(ResolveMode, TipEngagement))
	{
		Context.AddError(FText::Format(
			NSLOCTEXT("RopePreset", "InvalidModeCombo",
				"Invalid mode combination: {0} + {1}. FullSimulation/AssistedJudged allow BareWrap only; GuaranteedWrap allows Pierce/Cinch only."),
			UEnum::GetDisplayValueAsText(ResolveMode), UEnum::GetDisplayValueAsText(TipEngagement)));
		Result = EDataValidationResult::Invalid;
	}

	// Cinch는 미구현(BareWrap 경로 폴백) — 저장은 막지 않되 저작자에게 알린다.
	if (TipEngagement == ERopeTipEngagement::Cinch)
	{
		Context.AddWarning(NSLOCTEXT("RopePreset", "CinchNotImplemented",
			"Cinch engagement is not implemented yet - it falls back to the BareWrap wrapping path at runtime."));
	}

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

void URopePreset::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	const FName PropertyName = PropertyChangedEvent.GetPropertyName();
	if (PropertyName == GET_MEMBER_NAME_CHECKED(URopePreset, ResolveMode) ||
		PropertyName == GET_MEMBER_NAME_CHECKED(URopePreset, TipEngagement))
	{
		TipEngagement = RopeWrapModes::ClampEngagement(ResolveMode, TipEngagement);
	}
	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif // WITH_EDITOR
