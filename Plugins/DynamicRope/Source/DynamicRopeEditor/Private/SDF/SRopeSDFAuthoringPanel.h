// Copyright Epic Games, Inc. All Rights Reserved.
//
// SDF 오써링 도크탭의 콘텐츠 패널. 타깃 URopeSDFData를 골라 본별 SDF를 베이크하고, 베이크 전
// 설정(VoxelSize 등)을 조정하는 컨트롤을 호스팅한다.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "RopeSDFBaker.h"

class IDetailsView;
class URopeSDFData;
class SRopeSDFPreviewViewport;
struct FAssetData;
struct FPropertyChangedEvent;
struct FRopeSDFPreviewDrawOptions;

class SRopeSDFAuthoringPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SRopeSDFAuthoringPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** 타깃 에셋을 외부에서 지정한다(에셋 더블클릭 → 탭 연결 진입점). nullptr이면 타깃 해제.
	    베이크 설정 복원/프리뷰 갱신 등 픽커로 고른 것과 동일한 경로를 탄다. */
	void SetTargetAsset(URopeSDFData* InData);

private:
	/** 선택된 타깃에 대해 본별 베이크를 실행해 결과를 자산 메모리에 써넣는다(디스크 저장은 Save 버튼). */
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

	//~ Save / Refresh 버튼.
	/** 저장할 변경이 있는가(타깃 패키지가 dirty인가). Save 버튼 활성화 및 "Save *" 표시 기준. */
	bool CanSave() const;
	/** 저장이 필요하면 "Save *", 아니면 "Save". */
	FText GetSaveButtonText() const;
	/** 타깃 패키지를 디스크에 저장한다(베이크 결과 커밋). 성공 시 패키지 dirty가 해제된다. */
	FReply OnSaveClicked();
	/** 프리뷰 뷰포트를 다시 그릴 수 있는가(타깃 + SourceMesh 존재). */
	bool CanRefresh() const;
	/** 현재 베이크된 데이터 기준으로 프리뷰 뷰포트 오버레이를 다시 그린다(카메라는 유지). */
	FReply OnRefreshClicked();
	/** 오버레이 갱신 본체(베이크 직후 자동 반영 + Refresh 버튼 공용). */
	void RefreshPreviewOverlay();

	/** 현재 타깃의 SourceMesh를 동기 로드해 프리뷰 뷰포트에 반영한다(없으면 빈 뷰 + 안내). */
	void RefreshPreviewMesh();

	/** 내장 디테일 뷰에서 에셋 프로퍼티가 바뀌었을 때 — SourceMesh 변경이면 프리뷰 메시를 갱신한다. */
	void OnAssetPropertyChanged(const FPropertyChangedEvent& Event);

	/** 타깃 에셋의 원본 프로퍼티(SourceMesh/Bone Volumes)를 보여주는 내장 디테일 뷰.
	    더블클릭이 제네릭 프로퍼티 에디터 대신 이 탭을 열므로, 확인·편집은 여기서 한다. */
	TSharedPtr<IDetailsView> DetailsView;

	/** 프리뷰할 메시가 없을 때만 보이는 안내 오버레이의 가시성. */
	EVisibility GetPreviewHintVisibility() const;

	/** 우측 3D 프리뷰 뷰포트(베이크 대상 메시 + 향후 SDF 오버레이). */
	TSharedPtr<SRopeSDFPreviewViewport> PreviewViewport;

	/** FRopeSDFBakeSettings 멤버에 바인딩된 라벨+숫자 입력 행을 만든다(필드별 중복 제거).
	    Tip을 주면 행 전체에 hover 툴팁을 단다(비우면 툴팁 없음 — 오써링 탭은 커스텀 Slate라
	    UPROPERTY ToolTip 메타를 읽지 않으므로 여기서 직접 단다). */
	TSharedRef<class SWidget> MakeFloatRow(const FText& Label, float FRopeSDFBakeSettings::* Member, float MinVal, float MaxVal, const FText& Tip = FText::GetEmpty());
	TSharedRef<class SWidget> MakeIntRow(const FText& Label, int32 FRopeSDFBakeSettings::* Member, int32 MinVal, int32 MaxVal, const FText& Tip = FText::GetEmpty());

	/** Settings.Quantization(uint8/uint16) 선택 행: 라벨 + 두 개의 상호배타 옵션(라디오처럼 동작). */
	TSharedRef<class SWidget> MakeQuantizationRow();

	//~ 프리뷰 오버레이(패널 로컬 상태 = PreviewViewport->AccessDrawOptions())에 바인딩되는 컨트롤들.
	/** 오버레이 토글 체크박스 행(bounds/voxels/slice/gradient). */
	TSharedRef<class SWidget> MakeOverlayToggleRow(const FText& Label, bool FRopeSDFPreviewDrawOptions::* Member);
	/** 오버레이 float/int 파라미터 행(band/slice/gradient). */
	TSharedRef<class SWidget> MakePreviewFloatRow(const FText& Label, float FRopeSDFPreviewDrawOptions::* Member, float MinVal, float MaxVal);
	TSharedRef<class SWidget> MakePreviewIntRow(const FText& Label, int32 FRopeSDFPreviewDrawOptions::* Member, int32 MinVal, int32 MaxVal);
	/** slice 축을 X→Y→Z로 순환시키는 버튼 + 현재 축 라벨. */
	FReply OnCycleSliceAxis();
	FText GetSliceAxisLabel() const;

	//~ 오버레이 활성/가시성(베이크 데이터 유무 + 토글 종속).
	/** 오버레이 컨트롤을 편집할 수 있는가(프리뷰 볼륨 스냅샷 존재). 없으면 섹션 전체를 비활성(회색)한다. */
	bool CanEditOverlay() const;
	/** 편집 불가일 때만 보이는 안내("Bake + Refresh") 텍스트의 가시성. */
	EVisibility GetOverlayDisabledHintVisibility() const;
	/** 토글 종속 그룹(설명/범례/수치)의 가시성: 해당 토글이 켜져 있으면 Visible, 아니면 Collapsed. */
	EVisibility GetToggleGroupVisibility(bool FRopeSDFPreviewDrawOptions::* Member) const;

	/** 오버레이 설명 한 줄(연한 텍스트). 토글 그룹 안에 들어가 무엇을 그리는지 알려준다. */
	TSharedRef<class SWidget> MakeOverlayDescription(const FText& Text);
	/** 색상 범례 한 줄: 색 스와치 + 라벨. 디버그 색이 무엇을 뜻하는지 알려준다. */
	TSharedRef<class SWidget> MakeLegendRow(const FLinearColor& Color, const FText& Label);

	/**
	 * Band Threshold 행. 상한을 베이크 당시 NarrowBand로 제한한다 — 그 밖은 ±NarrowBand로 포화돼
	 * 방향/거리 정보가 없으므로 더 올려봐야 무의미하다. Voxels·Gradients 그룹 양쪽에서 같은 멤버를 공유.
	 */
	TSharedRef<class SWidget> MakeBandThresholdRow();
	/** Band Threshold 상한 = 타깃의 LastBakeSettings.NarrowBand(타깃 없으면 폴백). */
	float GetBandThresholdMax() const;
	TOptional<float> GetBandThresholdMaxOpt() const;
};
