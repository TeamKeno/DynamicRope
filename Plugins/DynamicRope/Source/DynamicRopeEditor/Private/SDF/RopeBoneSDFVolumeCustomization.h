// Copyright 2026 TeamKeno. All Rights Reserved.
//
// A details panel property type customization for FRopeBoneSDFVolume. It shows the bone name alongside the index on
// an array element's header row, before it is expanded. The UPROPERTY meta TitleProperty="Bone" is not reflected in
// the header in this context, so the header is drawn directly instead. The child rows, being the bone, bounds,
// resolution and so on, use the default display.

#pragma once

#include "CoreMinimal.h"
#include "IPropertyTypeCustomization.h"

class FRopeBoneSDFVolumeCustomization : public IPropertyTypeCustomization
{
public:
	static TSharedRef<IPropertyTypeCustomization> MakeInstance();

	//~ IPropertyTypeCustomization
	virtual void CustomizeHeader(TSharedRef<IPropertyHandle> PropertyHandle, FDetailWidgetRow& HeaderRow,
		IPropertyTypeCustomizationUtils& CustomizationUtils) override;
	virtual void CustomizeChildren(TSharedRef<IPropertyHandle> PropertyHandle, IDetailChildrenBuilder& ChildBuilder,
		IPropertyTypeCustomizationUtils& CustomizationUtils) override;
};
