// Copyright Epic Games, Inc. All Rights Reserved.

#include "SRopeSDFAuthoringPanel.h"
#include "SRopeSDFPreviewViewport.h"
#include "RopeSDFBaker.h"
#include "DynamicRopeEditorLog.h"
#include "Collision/SDF/RopeSDFData.h"

#include "Logging/MessageLog.h"             // 베이크 결과(coarsening) 보고
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SSplitter.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SExpandableArea.h" // 고급 베이크 설정 접이식 섹션
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Colors/SColorBlock.h"     // 범례 색 스와치
#include "Widgets/Layout/SBox.h"            // 색 스와치 크기 고정
#include "Styling/AppStyle.h"
#include "PropertyCustomizationHelpers.h"   // SObjectPropertyEntryBox
#include "Engine/SkeletalMesh.h"
#include "Misc/ScopedSlowTask.h"
#include "Framework/Notifications/NotificationManager.h"
#include "Widgets/Notifications/SNotificationList.h"

// 베이크 후 임시 자동저장.
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "Misc/PackageName.h"

#define LOCTEXT_NAMESPACE "RopeSDFAuthoring"

void SRopeSDFAuthoringPanel::Construct(const FArguments& InArgs)
{
	ChildSlot
	[
		SNew(SSplitter)
		.Orientation(Orient_Horizontal)

		// 좌: 컨트롤(타깃 피커 + 베이크 + 설정).
		+ SSplitter::Slot()
		.Value(0.4f)
		[
		SNew(SBorder)
		.BorderImage(FAppStyle::GetBrush("ToolPanel.GroupBorder"))
		.Padding(12.0f)
		[
			SNew(SVerticalBox)

			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 8.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("Title", "Rope SDF Authoring"))
			]

			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(STextBlock)
				.AutoWrapText(true)
				.Text(LOCTEXT("Help",
					"Pick a Rope SDF Data asset (its SourceMesh must be set), then Bake."))
			]

			// 타깃 에셋 피커.
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 8.0f)
			[
				SNew(SObjectPropertyEntryBox)
				.AllowedClass(URopeSDFData::StaticClass())
				.ObjectPath(this, &SRopeSDFAuthoringPanel::GetTargetPath)
				.OnObjectChanged(this, &SRopeSDFAuthoringPanel::OnTargetChanged)
				.DisplayThumbnail(false)
				.AllowClear(true)
			]

			+ SVerticalBox::Slot()
			.AutoHeight()
			[
				SNew(SHorizontalBox)

				// Bake: 현재 설정으로 본별 SDF를 굽는다(자산 메모리에만 반영 — 디스크 저장은 Save).
				+ SHorizontalBox::Slot()
				.FillWidth(1.0f)
				.Padding(0.0f, 0.0f, 4.0f, 0.0f)
				[
					SNew(SButton)
					.HAlign(HAlign_Center)
					.Text(LOCTEXT("Bake", "Bake"))
					.IsEnabled(this, &SRopeSDFAuthoringPanel::CanBake)
					.OnClicked(this, &SRopeSDFAuthoringPanel::OnBakeClicked)
				]

				// Save: 베이크 결과를 디스크에 쓴다. 저장할 변경이 있을 때만 활성화되고 "Save *"로 표시.
				+ SHorizontalBox::Slot()
				.FillWidth(1.0f)
				.Padding(0.0f, 0.0f, 4.0f, 0.0f)
				[
					SNew(SButton)
					.HAlign(HAlign_Center)
					.Text(this, &SRopeSDFAuthoringPanel::GetSaveButtonText)
					.IsEnabled(this, &SRopeSDFAuthoringPanel::CanSave)
					.OnClicked(this, &SRopeSDFAuthoringPanel::OnSaveClicked)
				]

				// Refresh: 현재 베이크된 데이터 기준으로 프리뷰 뷰포트를 다시 그린다.
				+ SHorizontalBox::Slot()
				.FillWidth(1.0f)
				[
					SNew(SButton)
					.HAlign(HAlign_Center)
					.Text(LOCTEXT("Refresh", "Refresh"))
					.IsEnabled(this, &SRopeSDFAuthoringPanel::CanRefresh)
					.OnClicked(this, &SRopeSDFAuthoringPanel::OnRefreshClicked)
				]
			]

			// 베이크 설정(베이크 전 편집 가능).
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 14.0f, 0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("SettingsHeader", "Bake Settings (last bake)"))
			]

			// 메인 노브: 일상 사용자가 다루는 값(품질 목표 + 충돌 밴드 + 가는 본 drop)을 노출한다.
			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("VoxelSize", "Voxel Size (cm)"), &FRopeSDFBakeSettings::VoxelSize, 0.25f, 10.0f,
				LOCTEXT("VoxelSizeTip", "Sample spacing in cm (cube voxel). Smaller sharpens the surface but increases memory and bake time.")) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("NarrowBand", "Narrow Band - outward (cm)"), &FRopeSDFBakeSettings::NarrowBand, 1.0f, 50.0f,
				LOCTEXT("NarrowBandTip", "Outward (free-space) detection band in cm: how far outside the surface the rope starts reacting to the body. Contact happens at CollisionRadius, so ~2-3x that is stable. The inward (inside-body) band is auto-sized per bone to the deepest interior distance at bake, so the whole interior is covered - no setting needed.")) ]

			// 가는 본 drop 임계값. 단면 girth가 이 값 미만인 본은 baking에서 제외 → 손가락 등 군더더기 볼륨 제거.
			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("MinGirth", "Min Bone Girth (cm)"), &FRopeSDFBakeSettings::MinBoneGirth, 0.0f, 20.0f,
				LOCTEXT("MinGirthTip", "Bones whose cross-section girth is thinner than this (cm) are dropped from baking (not merged into the parent). A rope cannot catch features finer than its radius, so set this near (or above) the CollisionRadius of the thinnest rope that will use this SDF. 0 bakes every bone.")) ]

			// 고급 설정: Max Resolution(메모리/시간 상한 — Voxel Size를 덮어쓸 수 있음)과
			// 잘 안 건드리는 튜닝값들은 기본 접힘으로 숨긴다.
			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[
				SNew(SExpandableArea)
				.InitiallyCollapsed(true)
				.HeaderContent()
				[
					SNew(STextBlock).Text(LOCTEXT("AdvancedHdr", "Advanced"))
				]
				.BodyContent()
				[
					SNew(SVerticalBox)

					// 양자화 비트수(uint8/uint16). 출력 용량/정밀도 트레이드오프 — 기본값으로 충분해 고급으로 숨긴다.
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
					[ MakeQuantizationRow() ]

					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
					[ MakeIntRow(LOCTEXT("MaxRes", "Max Resolution"), &FRopeSDFBakeSettings::MaxResolution, 8, 256,
						LOCTEXT("MaxResTip", "Maximum samples per axis. If a bone's grid would exceed this, Voxel Size is increased to fit (memory/time cap).")) ]

					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
					[ MakeFloatRow(LOCTEXT("WeightThresh", "Weight Threshold"), &FRopeSDFBakeSettings::WeightThreshold, 0.0f, 1.0f,
						LOCTEXT("WeightThreshTip", "Minimum average skin weight [0..1] for a triangle to be assigned to a bone.")) ]

					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
					[ MakeFloatRow(LOCTEXT("BoundsPad", "Bounds Padding (cm)"), &FRopeSDFBakeSettings::BoundsPadding, 0.0f, 20.0f,
						LOCTEXT("BoundsPadTip", "Expands each bone's triangle AABB by this much (cm) before voxelizing, leaving band margin beyond the skin.")) ]
				]
			]

			// 프리뷰 오버레이(자산 비변경 · 패널 로컬). RopeSDFDraw 헬퍼로 그린다.
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 14.0f, 0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("OverlayHeader", "Preview Overlay"))
			]

			// 베이크 데이터(프리뷰 볼륨 스냅샷)가 없으면 컨트롤은 보이되 비활성. 이유를 안내한다.
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Visibility(this, &SRopeSDFAuthoringPanel::GetOverlayDisabledHintVisibility)
				.AutoWrapText(true)
				.ColorAndOpacity(FSlateColor::UseSubduedForeground())
				.Text(LOCTEXT("OverlayDisabledHint",
					"No baked data to preview. Bake the asset and press Refresh to enable these overlays."))
			]

			// 오버레이 컨트롤 전체를 한 컨테이너로 감싼다. 프리뷰 볼륨이 없으면 IsEnabled가 자식 전체로
			// 전파되어 통째로 회색 비활성된다(어떤 디버그가 있는지는 계속 보이되 못 쓰는 상태).
			+ SVerticalBox::Slot()
			.AutoHeight()
			[
				SNew(SVerticalBox)
				.IsEnabled(this, &SRopeSDFAuthoringPanel::CanEditOverlay)

				// ── Bounds ──
				+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
				[ MakeOverlayToggleRow(LOCTEXT("DrawBounds", "Bounds"), &FRopeSDFPreviewDrawOptions::bDrawBounds) ]

				+ SVerticalBox::Slot().AutoHeight().Padding(16.0f, 0.0f, 0.0f, 4.0f)
				[
					SNew(SVerticalBox)
					.Visibility_Lambda([this]() { return GetToggleGroupVisibility(&FRopeSDFPreviewDrawOptions::bDrawBounds); })

					+ SVerticalBox::Slot().AutoHeight()
					[ MakeOverlayDescription(LOCTEXT("BoundsDesc",
						"Wireframe box of each baked volume's bone-local bounds.")) ]
				]

				// ── Voxels ──
				+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
				[ MakeOverlayToggleRow(LOCTEXT("DrawVoxels", "Voxels (narrow band)"), &FRopeSDFPreviewDrawOptions::bDrawVoxels) ]

				+ SVerticalBox::Slot().AutoHeight().Padding(16.0f, 0.0f, 0.0f, 4.0f)
				[
					SNew(SVerticalBox)
					.Visibility_Lambda([this]() { return GetToggleGroupVisibility(&FRopeSDFPreviewDrawOptions::bDrawVoxels); })

					+ SVerticalBox::Slot().AutoHeight()
					[ MakeOverlayDescription(LOCTEXT("VoxelsDesc",
						"Narrow-band samples drawn as points, colored by signed distance.")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f, 0.0f, 0.0f)
					[ MakeLegendRow(FLinearColor::Red, LOCTEXT("LegendInside", "Inside (negative)")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 1.0f, 0.0f, 0.0f)
					[ MakeLegendRow(FLinearColor::White, LOCTEXT("LegendSurface", "Surface (~0)")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 1.0f, 0.0f, 0.0f)
					[ MakeLegendRow(FLinearColor(0.0f, 0.4f, 1.0f), LOCTEXT("LegendOutside", "Outside (positive)")) ]

					// Band Threshold는 Voxels·Gradients 공용 파라미터다. 양쪽 그룹에 함께 노출하되 같은
					// 멤버(BandThreshold)에 바인딩되므로 한쪽을 바꾸면 다른 쪽도 자동으로 따라온다.
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 4.0f, 0.0f, 0.0f)
					[ MakeBandThresholdRow() ]
				]

				// ── Slice ──
				+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
				[ MakeOverlayToggleRow(LOCTEXT("DrawSlice", "Slice heatmap"), &FRopeSDFPreviewDrawOptions::bDrawSlice) ]

				+ SVerticalBox::Slot().AutoHeight().Padding(16.0f, 0.0f, 0.0f, 4.0f)
				[
					SNew(SVerticalBox)
					.Visibility_Lambda([this]() { return GetToggleGroupVisibility(&FRopeSDFPreviewDrawOptions::bDrawSlice); })

					+ SVerticalBox::Slot().AutoHeight()
					[ MakeOverlayDescription(LOCTEXT("SliceDesc",
						"Heatmap of a sampled plane cutting through each volume.")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f, 0.0f, 0.0f)
					[ MakeLegendRow(FLinearColor::Red, LOCTEXT("LegendInside2", "Inside (negative)")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 1.0f, 0.0f, 0.0f)
					[ MakeLegendRow(FLinearColor::White, LOCTEXT("LegendSurface2", "Surface (~0)")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 1.0f, 0.0f, 0.0f)
					[ MakeLegendRow(FLinearColor(0.0f, 0.4f, 1.0f), LOCTEXT("LegendOutside2", "Outside (positive)")) ]

					// Slice 파라미터: 축(순환 버튼) / 위치 / 해상도 / 색 스케일.
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 4.0f, 0.0f, 0.0f)
					[
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
						[ SNew(STextBlock).Text(LOCTEXT("SliceAxisLabel", "Slice Axis")) ]
						+ SHorizontalBox::Slot().FillWidth(0.45f)
						[
							SNew(SButton)
							.HAlign(HAlign_Center)
							.OnClicked(this, &SRopeSDFAuthoringPanel::OnCycleSliceAxis)
							[
								SNew(STextBlock).Text(this, &SRopeSDFAuthoringPanel::GetSliceAxisLabel)
							]
						]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
					[ MakePreviewFloatRow(LOCTEXT("SlicePos", "Slice Position (0-1)"), &FRopeSDFPreviewDrawOptions::SlicePosition, 0.0f, 1.0f) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
					[ MakePreviewIntRow(LOCTEXT("SliceRes", "Slice Resolution"), &FRopeSDFPreviewDrawOptions::SliceResolution, 2, 128) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
					[ MakePreviewFloatRow(LOCTEXT("SliceScale", "Slice Color Scale (cm)"), &FRopeSDFPreviewDrawOptions::SliceColorScale, 0.1f, 50.0f) ]
				]

				// ── Gradients ──
				+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
				[ MakeOverlayToggleRow(LOCTEXT("DrawGradient", "Gradients"), &FRopeSDFPreviewDrawOptions::bDrawGradient) ]

				+ SVerticalBox::Slot().AutoHeight().Padding(16.0f, 0.0f, 0.0f, 4.0f)
				[
					SNew(SVerticalBox)
					.Visibility_Lambda([this]() { return GetToggleGroupVisibility(&FRopeSDFPreviewDrawOptions::bDrawGradient); })

					+ SVerticalBox::Slot().AutoHeight()
					[ MakeOverlayDescription(LOCTEXT("GradientDesc",
						"Arrows showing the outward distance gradient at narrow-band samples.")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f, 0.0f, 0.0f)
					[ MakeLegendRow(FLinearColor::Green, LOCTEXT("LegendGradient", "Gradient direction (outward)")) ]

					// Voxels 그룹과 공유하는 Band Threshold(같은 멤버 바인딩 → 자동 동기화).
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 4.0f, 0.0f, 0.0f)
					[ MakeBandThresholdRow() ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f, 0.0f, 0.0f)
					[ MakePreviewFloatRow(LOCTEXT("GradLen", "Gradient Length (cm)"), &FRopeSDFPreviewDrawOptions::GradientLength, 0.5f, 20.0f) ]
				]
			]
		]
		]

		// 우: 3D 프리뷰 뷰포트(베이크 대상 메시 + 향후 SDF 오버레이).
		+ SSplitter::Slot()
		.Value(0.6f)
		[
			SNew(SOverlay)

			+ SOverlay::Slot()
			[
				SAssignNew(PreviewViewport, SRopeSDFPreviewViewport)
			]

			// 프리뷰할 메시가 없을 때만 보이는 안내.
			+ SOverlay::Slot()
			.HAlign(HAlign_Center)
			.VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Visibility(this, &SRopeSDFAuthoringPanel::GetPreviewHintVisibility)
				.AutoWrapText(true)
				.Justification(ETextJustify::Center)
				.Text(LOCTEXT("PreviewHint",
					"Pick a Rope SDF Data asset whose SourceMesh is set to preview it here."))
			]
		]
	];

	// 패널 생성 시점에 타깃이 이미 있을 수 있으니 한 번 반영.
	RefreshPreviewMesh();
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeFloatRow(const FText& Label,
	float FRopeSDFBakeSettings::* Member, float MinVal, float MaxVal, const FText& Tip)
{
	// 툴팁은 행 컨테이너에 단다 — Slate가 hover 위젯에서 부모로 올라가며 툴팁을 찾으므로
	// 라벨/입력칸 어디에 마우스를 올려도 동일하게 보인다. Tip이 비면 표시되지 않는다.
	return SNew(SHorizontalBox)
		.ToolTipText(Tip)
		+ SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
		[
			SNew(STextBlock).Text(Label)
		]
		+ SHorizontalBox::Slot().FillWidth(0.45f)
		[
			SNew(SNumericEntryBox<float>)
			.AllowSpin(true)
			.MinValue(MinVal).MaxValue(MaxVal)
			.MinSliderValue(MinVal).MaxSliderValue(MaxVal)
			.Value_Lambda([this, Member]() { return TOptional<float>(Settings.*Member); })
			.OnValueChanged_Lambda([this, Member](float NewVal) { Settings.*Member = NewVal; })
		];
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeIntRow(const FText& Label,
	int32 FRopeSDFBakeSettings::* Member, int32 MinVal, int32 MaxVal, const FText& Tip)
{
	// 툴팁은 행 컨테이너에 단다(MakeFloatRow와 동일 규약). Tip이 비면 표시되지 않는다.
	return SNew(SHorizontalBox)
		.ToolTipText(Tip)
		+ SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
		[
			SNew(STextBlock).Text(Label)
		]
		+ SHorizontalBox::Slot().FillWidth(0.45f)
		[
			SNew(SNumericEntryBox<int32>)
			.AllowSpin(true)
			.MinValue(MinVal).MaxValue(MaxVal)
			.MinSliderValue(MinVal).MaxSliderValue(MaxVal)
			.Value_Lambda([this, Member]() { return TOptional<int32>(Settings.*Member); })
			.OnValueChanged_Lambda([this, Member](int32 NewVal) { Settings.*Member = NewVal; })
		];
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeQuantizationRow()
{
	// 두 옵션(8/16-bit)을 라디오처럼: 켜진 것만 checked, 다른 걸 켜면 Settings.Quantization이 바뀌며
	// 이전 것이 자동으로 unchecked된다(이미 켜진 걸 다시 눌러 끄는 건 무시 → 항상 하나는 선택).
	auto MakeOpt = [this](ERopeSDFQuantBits Bits, const FText& Label)
	{
		return SNew(SCheckBox)
			.Style(&FAppStyle::Get().GetWidgetStyle<FCheckBoxStyle>("RadioButton"))
			.IsChecked_Lambda([this, Bits]()
			{
				return Settings.Quantization == Bits ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
			})
			.OnCheckStateChanged_Lambda([this, Bits](ECheckBoxState NewState)
			{
				if (NewState == ECheckBoxState::Checked)
				{
					Settings.Quantization = Bits;
				}
			})
			[
				SNew(STextBlock).Text(Label)
			];
	};

	return SNew(SHorizontalBox)
		.ToolTipText(LOCTEXT("QuantTip", "How precisely each SDF distance is stored. Standard (16-bit) is 256x finer than Low (8-bit) but doubles asset/RAM size; Low is the smallest. Default Standard."))
		+ SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
		[
			SNew(STextBlock).Text(LOCTEXT("Quant", "Distance Precision"))
		]
		+ SHorizontalBox::Slot().FillWidth(0.225f).VAlign(VAlign_Center)
		[
			MakeOpt(ERopeSDFQuantBits::UInt8, LOCTEXT("Quant8", "Low (8-bit)"))
		]
		+ SHorizontalBox::Slot().FillWidth(0.225f).VAlign(VAlign_Center)
		[
			MakeOpt(ERopeSDFQuantBits::UInt16, LOCTEXT("Quant16", "Standard (16-bit)"))
		];
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeOverlayToggleRow(const FText& Label,
	bool FRopeSDFPreviewDrawOptions::* Member)
{
	return SNew(SCheckBox)
		.IsChecked_Lambda([this, Member]()
		{
			const bool bOn = PreviewViewport.IsValid() && PreviewViewport->AccessDrawOptions().*Member;
			return bOn ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		})
		.OnCheckStateChanged_Lambda([this, Member](ECheckBoxState State)
		{
			if (PreviewViewport.IsValid())
			{
				PreviewViewport->AccessDrawOptions().*Member = (State == ECheckBoxState::Checked);
				PreviewViewport->InvalidatePreview();
			}
		})
		[
			SNew(STextBlock).Text(Label)
		];
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakePreviewFloatRow(const FText& Label,
	float FRopeSDFPreviewDrawOptions::* Member, float MinVal, float MaxVal)
{
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
		[
			SNew(STextBlock).Text(Label)
		]
		+ SHorizontalBox::Slot().FillWidth(0.45f)
		[
			SNew(SNumericEntryBox<float>)
			.AllowSpin(true)
			.MinValue(MinVal).MaxValue(MaxVal)
			.MinSliderValue(MinVal).MaxSliderValue(MaxVal)
			.Value_Lambda([this, Member]()
			{
				return TOptional<float>(PreviewViewport.IsValid() ? PreviewViewport->AccessDrawOptions().*Member : 0.0f);
			})
			.OnValueChanged_Lambda([this, Member](float NewVal)
			{
				if (PreviewViewport.IsValid())
				{
					PreviewViewport->AccessDrawOptions().*Member = NewVal;
					PreviewViewport->InvalidatePreview();
				}
			})
		];
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakePreviewIntRow(const FText& Label,
	int32 FRopeSDFPreviewDrawOptions::* Member, int32 MinVal, int32 MaxVal)
{
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
		[
			SNew(STextBlock).Text(Label)
		]
		+ SHorizontalBox::Slot().FillWidth(0.45f)
		[
			SNew(SNumericEntryBox<int32>)
			.AllowSpin(true)
			.MinValue(MinVal).MaxValue(MaxVal)
			.MinSliderValue(MinVal).MaxSliderValue(MaxVal)
			.Value_Lambda([this, Member]()
			{
				return TOptional<int32>(PreviewViewport.IsValid() ? PreviewViewport->AccessDrawOptions().*Member : 0);
			})
			.OnValueChanged_Lambda([this, Member](int32 NewVal)
			{
				if (PreviewViewport.IsValid())
				{
					PreviewViewport->AccessDrawOptions().*Member = NewVal;
					PreviewViewport->InvalidatePreview();
				}
			})
		];
}

FReply SRopeSDFAuthoringPanel::OnCycleSliceAxis()
{
	if (PreviewViewport.IsValid())
	{
		FRopeSDFPreviewDrawOptions& Options = PreviewViewport->AccessDrawOptions();
		const uint8 Next = (static_cast<uint8>(Options.SliceAxis) + 1) % 3;
		Options.SliceAxis = static_cast<ERopeSDFSliceAxis>(Next);
		PreviewViewport->InvalidatePreview();
	}
	return FReply::Handled();
}

FText SRopeSDFAuthoringPanel::GetSliceAxisLabel() const
{
	ERopeSDFSliceAxis Axis = ERopeSDFSliceAxis::Z;
	if (PreviewViewport.IsValid())
	{
		Axis = PreviewViewport->AccessDrawOptions().SliceAxis;
	}
	switch (Axis)
	{
	case ERopeSDFSliceAxis::X: return LOCTEXT("AxisX", "X");
	case ERopeSDFSliceAxis::Y: return LOCTEXT("AxisY", "Y");
	default:                   return LOCTEXT("AxisZ", "Z");
	}
}

bool SRopeSDFAuthoringPanel::CanEditOverlay() const
{
	// 베이크 여부가 아니라 "지금 뷰포트가 그릴 볼륨 스냅샷이 있는가"가 정확한 기준이다. 베이크 직후라도
	// Refresh로 스냅샷을 갱신하기 전엔 그릴 게 없으므로 컨트롤을 비활성으로 둔다.
	return PreviewViewport.IsValid() && PreviewViewport->HasPreviewVolumes();
}

EVisibility SRopeSDFAuthoringPanel::GetOverlayDisabledHintVisibility() const
{
	return CanEditOverlay() ? EVisibility::Collapsed : EVisibility::Visible;
}

EVisibility SRopeSDFAuthoringPanel::GetToggleGroupVisibility(bool FRopeSDFPreviewDrawOptions::* Member) const
{
	const bool bOn = PreviewViewport.IsValid() && PreviewViewport->AccessDrawOptions().*Member;
	return bOn ? EVisibility::Visible : EVisibility::Collapsed;
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeOverlayDescription(const FText& Text)
{
	return SNew(STextBlock)
		.Text(Text)
		.AutoWrapText(true)
		.ColorAndOpacity(FSlateColor::UseSubduedForeground());
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeLegendRow(const FLinearColor& Color, const FText& Label)
{
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0.0f, 0.0f, 6.0f, 0.0f)
		[
			SNew(SBox).WidthOverride(12.0f).HeightOverride(12.0f)
			[
				SNew(SColorBlock).Color(Color)
			]
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(STextBlock).Text(Label).ColorAndOpacity(FSlateColor::UseSubduedForeground())
		];
}

float SRopeSDFAuthoringPanel::GetBandThresholdMax() const
{
	// 베이크 당시 NarrowBand가 유효 표시 범위의 상한. 그 밖은 ±NarrowBand로 포화돼 의미가 없다.
	if (const URopeSDFData* Data = Target.Get())
	{
		return FMath::Max(Data->LastBakeSettings.NarrowBand, KINDA_SMALL_NUMBER);
	}
	return 50.0f; // 타깃 없음(편집 불가 상태) 폴백.
}

TOptional<float> SRopeSDFAuthoringPanel::GetBandThresholdMaxOpt() const
{
	return TOptional<float>(GetBandThresholdMax());
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeBandThresholdRow()
{
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
		[
			SNew(STextBlock).Text(LOCTEXT("Band", "Band Threshold (cm)"))
		]
		+ SHorizontalBox::Slot().FillWidth(0.45f)
		[
			SNew(SNumericEntryBox<float>)
			.AllowSpin(true)
			.MinValue(0.0f)
			.MaxValue(this, &SRopeSDFAuthoringPanel::GetBandThresholdMaxOpt)
			.MinSliderValue(0.0f)
			.MaxSliderValue(this, &SRopeSDFAuthoringPanel::GetBandThresholdMaxOpt)
			.Value_Lambda([this]()
			{
				return TOptional<float>(PreviewViewport.IsValid() ? PreviewViewport->AccessDrawOptions().BandThreshold : 0.0f);
			})
			.OnValueChanged_Lambda([this](float NewVal)
			{
				if (PreviewViewport.IsValid())
				{
					// 상한(NarrowBand)을 넘겨 입력돼도 잘라 저장한다.
					PreviewViewport->AccessDrawOptions().BandThreshold = FMath::Clamp(NewVal, 0.0f, GetBandThresholdMax());
					PreviewViewport->InvalidatePreview();
				}
			})
		];
}

FString SRopeSDFAuthoringPanel::GetTargetPath() const
{
	return Target.IsValid() ? Target->GetPathName() : FString();
}

void SRopeSDFAuthoringPanel::OnTargetChanged(const FAssetData& InAssetData)
{
	Target = Cast<URopeSDFData>(InAssetData.GetAsset());

	// 이미 베이크된 에셋이면 그 당시 설정을 패널로 복원해, 디자이너가 현재 결과와 비교하며 값을
	// 조정할 수 있게 한다. 미베이크 에셋이면 기본값(신규 베이크 출발점)을 유지한다.
	if (URopeSDFData* Data = Target.Get(); Data && Data->HasAnyBakedVolume())
	{
		Settings = Data->LastBakeSettings;
	}
	else
	{
		Settings = FRopeSDFBakeSettings();
	}

	// 타깃이 바뀌면 NarrowBand(=Band Threshold 상한)도 바뀌므로, 이전 타깃에서 남은 값이 새 상한을
	// 넘지 않도록 잘라준다(슬라이더 상한은 입력만 막을 뿐 기존 저장값은 안 줄이므로).
	if (PreviewViewport.IsValid())
	{
		float& Band = PreviewViewport->AccessDrawOptions().BandThreshold;
		Band = FMath::Clamp(Band, 0.0f, GetBandThresholdMax());
	}

	RefreshPreviewMesh();
}

void SRopeSDFAuthoringPanel::RefreshPreviewMesh()
{
	if (!PreviewViewport.IsValid())
	{
		return;
	}

	USkeletalMesh* Mesh = nullptr;
	if (URopeSDFData* Data = Target.Get())
	{
		// soft 참조이므로 오써링 시점에 동기 로드(없거나 로드 실패면 nullptr → 빈 뷰).
		Mesh = Data->SourceMesh.LoadSynchronous();
	}
	PreviewViewport->SetPreviewMesh(Mesh);
	PreviewViewport->SetPreviewData(Target.Get());
}

EVisibility SRopeSDFAuthoringPanel::GetPreviewHintVisibility() const
{
	const bool bHasMesh = Target.IsValid() && !Target->SourceMesh.IsNull();
	return bHasMesh ? EVisibility::Collapsed : EVisibility::HitTestInvisible;
}

bool SRopeSDFAuthoringPanel::CanBake() const
{
	return Target.IsValid() && !Target->SourceMesh.IsNull();
}

FReply SRopeSDFAuthoringPanel::OnBakeClicked()
{
	URopeSDFData* Data = Target.Get();
	if (!Data)
	{
		return FReply::Handled();
	}

	USkeletalMesh* Mesh = Data->SourceMesh.LoadSynchronous();
	if (!Mesh)
	{
		UE_LOG(LogRopeSDFBake, Warning, TEXT("Bake clicked on %s: SourceMesh not set or failed to load."), *Data->GetName());
		FNotificationInfo Info(LOCTEXT("NoMesh", "Bake failed: SourceMesh is not set or failed to load."));
		Info.ExpireDuration = 4.0f;
		FSlateNotificationManager::Get().AddNotification(Info);
		return FReply::Handled();
	}

	// 진행률 총량을 1.0으로 두고, 베이커가 본 하나를 시작할 때마다 1/Total씩 진행시킨다.
	// 취소 버튼을 띄우고, 콜백에서 ShouldCancel()을 보고 false를 반환하면 베이커가 중단한다.
	FScopedSlowTask Slow(1.0f, LOCTEXT("Baking", "Baking per-bone SDF..."));
	Slow.MakeDialog(true /*bShowCancelButton*/);

	// 베이크 중에는 슬로우 태스크가 주기적으로 Slate를 펌프하므로(취소 버튼 처리), 그 틈에 사용자가
	// 설정/본 필터를 바꿔도 베이크 도중 입력값이 흔들리지 않도록 호출 시점 값으로 스냅샷해 넘긴다.
	const FRopeSDFBakeSettings SettingsSnapshot = Settings;
	const TArray<FName> BoneFilterSnapshot = BoneFilter;

	TArray<FRopeBoneSDFVolume> Volumes;
	FRopeSDFBakeStats Stats;
	const ERopeSDFBakeResult Result = FRopeSDFBaker::BakeMesh(Mesh, BoneFilterSnapshot, SettingsSnapshot, Volumes,
		// 본 단위: 진행률 한 칸 전진(+ UI 펌프) 후 취소 여부를 읽는다.
		[&Slow](int32 Done, int32 Total, const FName& Bone) -> bool
		{
			const float Frac = (Total > 0) ? (1.0f / static_cast<float>(Total)) : 1.0f;
			Slow.EnterProgressFrame(Frac, FText::Format(
				LOCTEXT("BakingBone", "Baking SDF: {0} ({1}/{2})"),
				FText::FromName(Bone), FText::AsNumber(Done + 1), FText::AsNumber(Total)));
			return !Slow.ShouldCancel();
		},
		// 본 내부 voxel 배치 사이: UI를 펌프(ShouldCancel 내부)하고 취소 클릭을 처리한다.
		[&Slow]() -> bool { return Slow.ShouldCancel(); },
		&Stats);

	if (Result == ERopeSDFBakeResult::NoGeometry)
	{
		FNotificationInfo Info(LOCTEXT("NoGeom", "Bake failed: mesh has no CPU geometry (cooked/stripped)."));
		Info.ExpireDuration = 4.0f;
		FSlateNotificationManager::Get().AddNotification(Info);
		return FReply::Handled();
	}

	if (Result == ERopeSDFBakeResult::Cancelled)
	{
		// 자산은 건드리지 않는다(부분 결과 버림) — 기존 베이크 결과 유지.
		FNotificationInfo Info(LOCTEXT("BakeCancelled", "Bake cancelled — asset unchanged."));
		Info.ExpireDuration = 4.0f;
		FSlateNotificationManager::Get().AddNotification(Info);
		return FReply::Handled();
	}

	// 베이크 결과는 자산 메모리에만 반영하고 패키지를 dirty로 표시한다. 디스크 저장은 Save 버튼이,
	// 뷰포트 반영은 Refresh 버튼이 담당한다(자동 저장 제거).
	Data->Modify();
	Data->BoneVolumes = MoveTemp(Volumes);
	// 베이크에 실제로 사용된 설정을 에셋에 기록 — 다음에 이 에셋을 열면 패널이 이 값을 복원해
	// 현재 결과와 비교하며 재조정할 수 있다.
	Data->LastBakeSettings = SettingsSnapshot;
	Data->MarkPackageDirty();

	UE_LOG(LogRopeSDFBake, Log, TEXT("Baked %s: %d bone volume(s) (unsaved — press Save)."),
		*Data->GetName(), Data->BoneVolumes.Num());

	// 베이크 결과를 Message Log로 보고한다. Max Resolution 상한 때문에 사용자가 요청한 Voxel Size보다
	// 굵게 구워진(coarsen된) 본이 있으면, 그 본 이름과 요청→실제 크기를 경고로 나열하고 로그 창을 띄운다.
	// (조용한 덮어쓰기를 가시화 — 상세는 토스트에 넣지 않고 Message Log에만 둔다.)
	{
		FMessageLog Log(RopeSDFMessageLogName);
		Log.NewPage(FText::Format(LOCTEXT("BakePage", "SDF bake: {0}"), FText::FromString(Data->GetName())));
		if (Stats.CoarsenedBones.Num() > 0)
		{
			Log.Warning(FText::Format(
				LOCTEXT("CoarsenSummary",
					"{0} of {1} bone(s) were coarsened beyond your requested Voxel Size ({2} cm) "
					"to fit Max Resolution ({3}). Increase Max Resolution (Advanced) or Voxel Size to keep your target."),
				FText::AsNumber(Stats.CoarsenedBones.Num()), FText::AsNumber(Stats.BonesBaked),
				FText::AsNumber(SettingsSnapshot.VoxelSize), FText::AsNumber(SettingsSnapshot.MaxResolution)));
			for (const FRopeSDFCoarsenedBone& C : Stats.CoarsenedBones)
			{
				Log.Warning(FText::Format(
					LOCTEXT("CoarsenBone", "  {0}:  {1} cm -> {2} cm   (res {3}x{4}x{5})"),
					FText::FromName(C.Bone),
					FText::AsNumber(C.RequestedVoxelSize), FText::AsNumber(C.ActualVoxelSize),
					FText::AsNumber(C.Resolution.X), FText::AsNumber(C.Resolution.Y), FText::AsNumber(C.Resolution.Z)));
			}
			Log.Open(EMessageSeverity::Warning); // coarsening이 있으면 로그 창을 앞으로 꺼내 알린다.
		}
		else
		{
			Log.Info(FText::Format(
				LOCTEXT("BakeClean", "Baked {0} bone volume(s) at {1} cm — no coarsening."),
				FText::AsNumber(Stats.BonesBaked), FText::AsNumber(SettingsSnapshot.VoxelSize)));
		}

		// MinBoneGirth로 제외(drop)된 가는 본들을 한 줄로 보고(쉼표 구분). drop이 0개면 보고 안 함.
		if (Stats.DroppedThinBones.Num() > 0)
		{
			TArray<FString> DroppedNames;
			DroppedNames.Reserve(Stats.DroppedThinBones.Num());
			for (const FName& B : Stats.DroppedThinBones)
			{
				DroppedNames.Add(B.ToString());
			}
			Log.Info(FText::Format(
				LOCTEXT("DropSummary", "{0} thin bone(s) dropped (girth < {1} cm): {2}"),
				FText::AsNumber(Stats.DroppedThinBones.Num()),
				FText::AsNumber(SettingsSnapshot.MinBoneGirth),
				FText::FromString(FString::Join(DroppedNames, TEXT(", ")))));
		}
	}

	FNotificationInfo Info(FText::Format(
		LOCTEXT("Baked", "Baked {0} bone volume(s). Press Save to write to disk, Refresh to preview."),
		FText::AsNumber(Data->BoneVolumes.Num())));
	Info.ExpireDuration = 5.0f;
	FSlateNotificationManager::Get().AddNotification(Info);
	return FReply::Handled();
}

bool SRopeSDFAuthoringPanel::CanSave() const
{
	const URopeSDFData* Data = Target.Get();
	const UPackage* Package = Data ? Data->GetPackage() : nullptr;
	return Package && Package->IsDirty();
}

FText SRopeSDFAuthoringPanel::GetSaveButtonText() const
{
	// 저장이 필요하면(=dirty) "Save *"로 미저장 상태를 알린다.
	return CanSave() ? LOCTEXT("SaveDirty", "Save *") : LOCTEXT("Save", "Save");
}

FReply SRopeSDFAuthoringPanel::OnSaveClicked()
{
	URopeSDFData* Data = Target.Get();
	if (!Data)
	{
		return FReply::Handled();
	}

	// (read-only Perforce 파일이면 쓰기 실패 — 소스컨트롤 체크아웃 연동은 추후 개선.)
	bool bSaved = false;
	if (UPackage* Package = Data->GetPackage())
	{
		const FString FileName = FPackageName::LongPackageNameToFilename(
			Package->GetName(), FPackageName::GetAssetPackageExtension());
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.SaveFlags = SAVE_NoError;
		bSaved = UPackage::SavePackage(Package, Data, *FileName, SaveArgs);
	}

	if (bSaved)
	{
		UE_LOG(LogRopeSDFBake, Log, TEXT("Saved %s: %d bone volume(s)."), *Data->GetName(), Data->BoneVolumes.Num());
	}
	else
	{
		UE_LOG(LogRopeSDFBake, Warning, TEXT("Save failed for %s (read-only? check out the file)."), *Data->GetName());
	}

	FNotificationInfo Info(bSaved
		? FText::Format(LOCTEXT("Saved", "Saved {0}."), FText::FromString(Data->GetName()))
		: LOCTEXT("SaveFail", "Save failed — file may be read-only (check out, then Save again)."));
	Info.ExpireDuration = 4.0f;
	FSlateNotificationManager::Get().AddNotification(Info);
	return FReply::Handled();
}

bool SRopeSDFAuthoringPanel::CanRefresh() const
{
	return Target.IsValid() && !Target->SourceMesh.IsNull();
}

FReply SRopeSDFAuthoringPanel::OnRefreshClicked()
{
	// 메시는 그대로 두어 카메라를 유지하고, 베이크된 데이터 출처만 다시 지정해 오버레이를 다시 그린다.
	if (PreviewViewport.IsValid())
	{
		PreviewViewport->SetPreviewData(Target.Get());
		PreviewViewport->InvalidatePreview();
	}
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
