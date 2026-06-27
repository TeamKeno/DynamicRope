// Copyright Epic Games, Inc. All Rights Reserved.
//
// SDF 오써링 도크탭의 콘텐츠 패널. 타깃 URopeSDFData를 골라 본별 SDF를 베이크하고, 베이크 전
// 설정(VoxelSize 등)을 조정하는 컨트롤을 호스팅한다.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "RopeSDFBaker.h"

class URopeSDFData;
class SRopeSDFPreviewViewport;
struct FAssetData;
struct FRopeSDFPreviewDrawOptions;

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

	/** true면 현재 베이크 설정으로 (자산에 쓰지 않고) 구운 결과를 프리뷰에 표시하고, 설정 변경 시 갱신한다. */
	bool bLivePreview = false;

	FString GetTargetPath() const;
	void OnTargetChanged(const FAssetData& InAssetData);
	bool CanBake() const;

	/** Live Preview 체크박스 상태/토글. 켜면 설정대로 미저장 베이크를 돌려 프리뷰에 띄운다. */
	ECheckBoxState IsLivePreviewChecked() const;
	void OnLivePreviewChanged(ECheckBoxState NewState);

	/** bLivePreview일 때 현재 설정으로 미저장 베이크를 돌려 프리뷰 볼륨을 갱신한다(아니면 프리뷰 클리어). */
	void RebuildPreviewBake();

	/** 베이크 설정 값이 commit될 때 호출 — Live Preview가 켜져 있으면 다시 굽는다. */
	void OnBakeSettingCommitted();

	/** 현재 타깃의 SourceMesh를 동기 로드해 프리뷰 뷰포트에 반영한다(없으면 빈 뷰 + 안내). */
	void RefreshPreviewMesh();

	/** 프리뷰할 메시가 없을 때만 보이는 안내 오버레이의 가시성. */
	EVisibility GetPreviewHintVisibility() const;

	/** 우측 3D 프리뷰 뷰포트(베이크 대상 메시 + 향후 SDF 오버레이). */
	TSharedPtr<SRopeSDFPreviewViewport> PreviewViewport;

	/** FRopeSDFBakeSettings 멤버에 바인딩된 라벨+숫자 입력 행을 만든다(필드별 중복 제거). */
	TSharedRef<class SWidget> MakeFloatRow(const FText& Label, float FRopeSDFBakeSettings::* Member, float MinVal, float MaxVal);
	TSharedRef<class SWidget> MakeIntRow(const FText& Label, int32 FRopeSDFBakeSettings::* Member, int32 MinVal, int32 MaxVal);

	//~ 프리뷰 오버레이(패널 로컬 상태 = PreviewViewport->AccessDrawOptions())에 바인딩되는 컨트롤들.
	/** 오버레이 토글 체크박스 행(bounds/voxels/slice/gradient). */
	TSharedRef<class SWidget> MakeOverlayToggleRow(const FText& Label, bool FRopeSDFPreviewDrawOptions::* Member);
	/** 오버레이 float/int 파라미터 행(band/slice/gradient). */
	TSharedRef<class SWidget> MakePreviewFloatRow(const FText& Label, float FRopeSDFPreviewDrawOptions::* Member, float MinVal, float MaxVal);
	TSharedRef<class SWidget> MakePreviewIntRow(const FText& Label, int32 FRopeSDFPreviewDrawOptions::* Member, int32 MinVal, int32 MaxVal);
	/** slice 축을 X→Y→Z로 순환시키는 버튼 + 현재 축 라벨. */
	FReply OnCycleSliceAxis();
	FText GetSliceAxisLabel() const;
};
