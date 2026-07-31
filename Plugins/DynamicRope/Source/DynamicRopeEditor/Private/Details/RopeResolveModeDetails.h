// Copyright Epic Games, Inc. All Rights Reserved.
//
// A class customization that removes tuning made meaningless by a resolve mode of GuaranteedWrap from the details panel.
// It is registered for both URopeComponent and URopePreset: the preset is a one-to-one mirror of the component's
// properties, with ResolveMode and HoldConfig as sibling members of the same name, so the rule holds identically, and
// keeping it in one place stops the two panels disagreeing. That is why the class is templated on its owning type.
//
// The division of responsibility:
//  - WrapConfig and WhipConfig are struct members directly on the class and can therefore see the resolve mode, so
//    they are greyed out by a struct-member EditCondition in each header, RopeComponent.h and RopePreset.h, and are
//    not touched here. The edit-const propagates down the property node tree to the inline children promoted by
//    ShowOnlyInnerProperties as well.
//  - The only thing this class handles is the three release fields inside HoldConfig. They are inside a struct, so
//    their EditCondition cannot see the resolve mode outside it, and they are hidden here through HideProperty. The
//    automatic release is meaningful under FullSimulation and AssistedJudged alone, since GuaranteedWrap releases
//    explicitly, and there is no clean way to grey them in place, so hiding them was chosen.
//    The rest of the hold configuration, being the pull, tether and taut settings, is meaningful under GuaranteedWrap
//    too and is left alone.
//
// FullSimulation and AssistedJudged remain fully editable.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "IDetailCustomization.h"

#include "Core/RopeConfigTypes.h"
#include "Core/RopeLifecycleTypes.h"

#include "DetailLayoutBuilder.h"
#include "PropertyHandle.h"

/** TRopeOwner is a class with ResolveMode and HoldConfig as members, being URopeComponent or URopePreset. */
template <typename TRopeOwner>
class TRopeResolveModeDetails : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance()
	{
		return MakeShared<TRopeResolveModeDetails<TRopeOwner>>();
	}

	//~ IDetailCustomization
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override
	{
		// Changing the resolve mode changes what is hidden, so the panel is force-refreshed.
		// (Unlike an EditCondition, a customization is not re-run by a value change alone.)
		const TSharedRef<IPropertyHandle> ResolveModeHandle =
			DetailBuilder.GetProperty(GET_MEMBER_NAME_CHECKED(TRopeOwner, ResolveMode));
		if (ResolveModeHandle->IsValidHandle())
		{
			ResolveModeHandle->SetOnPropertyValueChanged(FSimpleDelegate::CreateLambda(
				[&DetailBuilder]() { DetailBuilder.ForceRefreshDetails(); }));
		}

		TArray<TWeakObjectPtr<UObject>> Objects;
		DetailBuilder.GetObjectsBeingCustomized(Objects);

		int32 OwnerCount = 0;
		bool bAllGuaranteed = true;
		for (const TWeakObjectPtr<UObject>& Object : Objects)
		{
			if (const TRopeOwner* Owner = Cast<TRopeOwner>(Object.Get()))
			{
				++OwnerCount;
				bAllGuaranteed &= (Owner->ResolveMode == ERopeWrapResolveMode::GuaranteedWrap);
			}
		}

		// Gated only when every selected object is GuaranteedWrap. If even one FullSimulation or AssistedJudged is mixed in, as with a multiple selection, it is conservatively left alone.
		if (OwnerCount == 0 || !bAllGuaranteed)
		{
			return;
		}

		const TSharedRef<IPropertyHandle> HoldHandle =
			DetailBuilder.GetProperty(GET_MEMBER_NAME_CHECKED(TRopeOwner, HoldConfig));
		static const FName ReleaseFieldNames[] = {
			GET_MEMBER_NAME_CHECKED(FRopeHoldConfig, TensionReleaseForce),
			GET_MEMBER_NAME_CHECKED(FRopeHoldConfig, TensionReleaseTime),
			GET_MEMBER_NAME_CHECKED(FRopeHoldConfig, DistanceReleaseSlack),
		};
		for (const FName& FieldName : ReleaseFieldNames)
		{
			const TSharedPtr<IPropertyHandle> FieldHandle = HoldHandle->GetChildHandle(FieldName);
			if (FieldHandle.IsValid())
			{
				DetailBuilder.HideProperty(FieldHandle);
			}
		}
	}
};
