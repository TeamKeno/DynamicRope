// Copyright Epic Games, Inc. All Rights Reserved.

#include "Details/RopeComponentDetails.h"

#include "RopeComponent.h"
#include "Core/RopeConfigTypes.h"
#include "Core/RopeLifecycleTypes.h"

#include "DetailLayoutBuilder.h"
#include "PropertyHandle.h"

TSharedRef<IDetailCustomization> FRopeComponentDetails::MakeInstance()
{
	return MakeShared<FRopeComponentDetails>();
}

void FRopeComponentDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	// ResolveMode가 바뀌면 회색처리 대상이 달라지므로 패널을 강제 리프레시한다.
	// (customization은 EditCondition과 달리 값 변경만으로 재실행되지 않는다.)
	const TSharedRef<IPropertyHandle> ResolveModeHandle =
		DetailBuilder.GetProperty(GET_MEMBER_NAME_CHECKED(URopeComponent, ResolveMode));
	if (ResolveModeHandle->IsValidHandle())
	{
		ResolveModeHandle->SetOnPropertyValueChanged(FSimpleDelegate::CreateLambda(
			[&DetailBuilder]() { DetailBuilder.ForceRefreshDetails(); }));
	}

	// WrapConfig/WhipConfig의 ③ 회색처리는 구조체-멤버 EditCondition(RopeComponent.h)이 담당한다.
	// 여기서는 메타로 닿을 수 없는 대상 하나만 처리한다: HoldConfig 내부의 Release 필드 3종. 이들은
	// 구조체 *안*이라 EditCondition이 컴포넌트의 ResolveMode를 볼 수 없어 여기서 숨긴다. ShowOnlyInnerProperties
	// 구조체의 내부 일부 필드는 제자리 회색처리가 마땅치 않아, ③에서 무의미한(자동 release는 ①②만 유효 —
	// ③은 명시 해제만) 이 세 값은 숨기는 쪽으로 정했다(2026-07-25 결정).
	TArray<TWeakObjectPtr<UObject>> Objects;
	DetailBuilder.GetObjectsBeingCustomized(Objects);

	int32 RopeCount = 0;
	bool bAllGuaranteed = true;
	for (const TWeakObjectPtr<UObject>& Object : Objects)
	{
		if (const URopeComponent* Rope = Cast<URopeComponent>(Object.Get()))
		{
			++RopeCount;
			bAllGuaranteed &= (Rope->ResolveMode == ERopeWrapResolveMode::GuaranteedWrap);
		}
	}

	// 선택된 로프가 전부 ③일 때만 게이트한다. ①②가 하나라도 섞이면(다중 선택) 보수적으로 그대로 둔다.
	if (RopeCount == 0 || !bAllGuaranteed)
	{
		return;
	}

	const TSharedRef<IPropertyHandle> HoldHandle =
		DetailBuilder.GetProperty(GET_MEMBER_NAME_CHECKED(URopeComponent, HoldConfig));
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
