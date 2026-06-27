// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeBoneSDFVolume의 디테일 패널 프로퍼티 타입 커스터마이즈. 배열 요소 헤더 행(펼치기 전)에
// 인덱스와 함께 본 이름을 표시한다. UPROPERTY meta=(TitleProperty="Bone")가 이 환경에서 헤더에
// 반영되지 않아, 헤더를 직접 그리는 방식으로 대체한다. 자식 행(Bone/Bounds/Resolution/…)은 기본 표시.

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
