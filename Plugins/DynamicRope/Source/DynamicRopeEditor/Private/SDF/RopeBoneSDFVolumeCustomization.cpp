// Copyright Epic Games, Inc. All Rights Reserved.

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
	// 본 이름 자식 핸들. 람다에 값으로 캡처해 헤더 텍스트를 실시간으로 따라가게 한다.
	const TSharedPtr<IPropertyHandle> BoneHandle =
		PropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(FRopeBoneSDFVolume, Bone));

	HeaderRow
	.NameContent()
	[
		// 기본 이름 위젯 = 배열 요소의 "Index [n]".
		PropertyHandle->CreatePropertyNameWidget()
	]
	.ValueContent()
	.HAlign(HAlign_Left)
	.MinDesiredWidth(180.0f)
	[
		// 값 칸에 본 이름을 표시 → 헤더 행이 "Index [n]    <본 이름>" 형태로 보인다.
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
	// 펼쳤을 때는 구조체의 모든 멤버를 기본 방식으로 보여준다(Bone/LocalBounds/Resolution/VoxelSize/Distances).
	uint32 NumChildren = 0;
	PropertyHandle->GetNumChildren(NumChildren);
	for (uint32 Index = 0; Index < NumChildren; ++Index)
	{
		if (const TSharedPtr<IPropertyHandle> Child = PropertyHandle->GetChildHandle(Index))
		{
			ChildBuilder.AddProperty(Child.ToSharedRef());
		}
	}
}

#undef LOCTEXT_NAMESPACE
