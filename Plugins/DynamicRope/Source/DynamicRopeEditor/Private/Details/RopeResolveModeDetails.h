// Copyright Epic Games, Inc. All Rights Reserved.
//
// ResolveMode == ③ GuaranteedWrap일 때 의미 없는 튜닝을 디테일 패널에서 걷어내는 클래스 커스터마이즈.
// URopeComponent와 URopePreset **양쪽에 등록한다** — 프리셋은 컴포넌트 프로퍼티의 1:1 미러라
// (ResolveMode/HoldConfig가 같은 이름의 형제 멤버) 규칙이 그대로 성립하고, 두 패널이 어긋나지 않도록
// 규칙을 한 곳에만 둔다. 그래서 클래스가 소유 타입으로 템플릿화되어 있다.
//
// 역할 분담:
//  - WrapConfig / WhipConfig: 클래스 직속 struct 멤버라 ResolveMode를 볼 수 있어 각 헤더
//    (RopeComponent.h / RopePreset.h)의 구조체-멤버 EditCondition으로 회색처리한다(여기서 손대지 않음).
//    ShowOnlyInnerProperties로 승격된 인라인 자식까지 edit-const가 프로퍼티 노드 트리를 타고 함께 회색이 된다.
//  - 이 클래스가 담당하는 유일한 대상: HoldConfig 내부의 Release 필드 3종. 구조체 *안*이라 EditCondition이
//    바깥의 ResolveMode를 볼 수 없으므로, 여기서 HideProperty로 숨긴다(자동 release는 ①②만 유효 —
//    ③은 명시 해제만이라 무의미. 제자리 회색처리가 마땅치 않아 숨김으로 정함 — 2026-07-25 결정).
//    Hold의 나머지(Pull/Tether/Taut)는 ③에서도 유효하므로 건드리지 않는다.
//
// ①FullSimulation·②AssistedJudged는 전부 그대로 편집 가능하다.

#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

#include "Core/RopeConfigTypes.h"
#include "Core/RopeLifecycleTypes.h"

#include "DetailLayoutBuilder.h"
#include "PropertyHandle.h"

/** TRopeOwner: ResolveMode와 HoldConfig를 멤버로 갖는 클래스(URopeComponent / URopePreset). */
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
		// ResolveMode가 바뀌면 숨김 대상이 달라지므로 패널을 강제 리프레시한다.
		// (customization은 EditCondition과 달리 값 변경만으로 재실행되지 않는다.)
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

		// 대상이 전부 ③일 때만 게이트한다. ①②가 하나라도 섞이면(다중 선택) 보수적으로 그대로 둔다.
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
