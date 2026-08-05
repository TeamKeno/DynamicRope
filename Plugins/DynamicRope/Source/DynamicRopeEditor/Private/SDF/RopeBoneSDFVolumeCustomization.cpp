// Copyright 2026 TeamKeno. All Rights Reserved.

#include "SDF/RopeBoneSDFVolumeCustomization.h"
#include "Collision/SDF/RopeSDFData.h"

#include "DetailWidgetRow.h"
#include "IDetailChildrenBuilder.h"
#include "PropertyHandle.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "RopeBoneSDFVolumeCustomization"

TSharedRef<IPropertyTypeCustomization> FRopeBoneSDFVolumeCustomization::MakeInstance()
{
	return MakeShared<FRopeBoneSDFVolumeCustomization>();
}

void FRopeBoneSDFVolumeCustomization::CustomizeHeader(TSharedRef<IPropertyHandle> PropertyHandle,
	FDetailWidgetRow& HeaderRow, IPropertyTypeCustomizationUtils& CustomizationUtils)
{
	// The bone name child handle, captured by value into the lambda so the header text follows it live.
	const TSharedPtr<IPropertyHandle> BoneHandle =
		PropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(FRopeBoneSDFVolume, Bone));

	HeaderRow
	.NameContent()
	[
	// The default name widget is the array element's "Index [n]".
		PropertyHandle->CreatePropertyNameWidget()
	]
	.ValueContent()
	.HAlign(HAlign_Left)
	.MinDesiredWidth(180.0f)
	[
	// Showing the bone name in the value column makes the header row read as "Index [n]    <bone name>".
		SNew(STextBlock)
		.Font(CustomizationUtils.GetRegularFont())
		.Text_Lambda([BoneHandle]()
		{
			FString BoneStr;
			if (BoneHandle.IsValid()
				&& BoneHandle->GetValueAsFormattedString(BoneStr) == FPropertyAccess::Success
				&& !BoneStr.IsEmpty() && BoneStr != TEXT("None"))
			{
				return FText::FromString(BoneStr);
			}
			return LOCTEXT("NoBone", "(no bone)");
		})
	];
}

void FRopeBoneSDFVolumeCustomization::CustomizeChildren(TSharedRef<IPropertyHandle> PropertyHandle,
	IDetailChildrenBuilder& ChildBuilder, IPropertyTypeCustomizationUtils& CustomizationUtils)
{
	// When expanded, every member of the struct is shown in the default way: the bone, local bounds, resolution, voxel size and distances.
	uint32 NumChildren = 0;
	PropertyHandle->GetNumChildren(NumChildren);
	for (uint32 Index = 0; Index < NumChildren; ++Index)
	{
		if (const TSharedPtr<IPropertyHandle> Child = PropertyHandle->GetChildHandle(Index))
		{
			if(Child->GetProperty()->GetFName() == GET_MEMBER_NAME_CHECKED(FRopeBoneSDFVolume, Bone))
			{
				// The bone is already shown on the header row, so it is omitted from the expanded child rows.
				continue;
			}
			ChildBuilder.AddProperty(Child.ToSharedRef());
		}
	}
}

#undef LOCTEXT_NAMESPACE
