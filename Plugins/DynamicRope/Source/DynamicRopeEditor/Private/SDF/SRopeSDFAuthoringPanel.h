// Copyright Epic Games, Inc. All Rights Reserved.
//
// SDF 오써링 도크탭의 콘텐츠 패널. 타깃 URopeSDFData를 골라 본별 SDF를 베이크하고, 베이크 전
// 설정(VoxelSize 등)을 조정하는 컨트롤을 호스팅한다.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "RopeSDFBaker.h"

class URopeSDFData;
struct FAssetData;

class SRopeSDFAuthoringPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SRopeSDFAuthoringPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	/** 선택된 타깃에 대해 본별 베이크를 실행하고, 결과를 다시 써넣은 뒤 (임시로) 저장한다. */
	FReply OnBakeClicked();

	/** 에셋 입력 박스에서 선택한 타깃 URopeSDFData. */
	TWeakObjectPtr<URopeSDFData> Target;

	/** 베이크 설정 값. */
	FRopeSDFBakeSettings Settings;

	/** 베이크할 본. 비우면 = 스킨된 모든 본(v1). 본 선택 UI는 추후. */
	TArray<FName> BoneFilter;

	FString GetTargetPath() const;
	void OnTargetChanged(const FAssetData& InAssetData);
	bool CanBake() const;

	/** FRopeSDFBakeSettings 멤버에 바인딩된 라벨+숫자 입력 행을 만든다(필드별 중복 제거). */
	TSharedRef<class SWidget> MakeFloatRow(const FText& Label, float FRopeSDFBakeSettings::* Member, float MinVal, float MaxVal);
	TSharedRef<class SWidget> MakeIntRow(const FText& Label, int32 FRopeSDFBakeSettings::* Member, int32 MinVal, int32 MaxVal);
};
