// Copyright Epic Games, Inc. All Rights Reserved.

#include "SRopeSDFAuthoringPanel.h"
#include "SRopeSDFPreviewViewport.h"
#include "RopeSDFBaker.h"
#include "DynamicRopeEditorLog.h"
#include "Collision/SDF/RopeSDFData.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SSplitter.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
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
					"Pick a Rope SDF Data asset (its SourceMesh must be set), then Bake. v1 bakes every "
					"skinned bone into per-bone volumes and auto-saves the asset."))
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
				.Text(LOCTEXT("SettingsHeader", "Bake Settings"))
			]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("VoxelSize", "Voxel Size (cm)"), &FRopeSDFBakeSettings::VoxelSize, 0.25f, 10.0f) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeIntRow(LOCTEXT("MaxRes", "Max Resolution"), &FRopeSDFBakeSettings::MaxResolution, 8, 256) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("NarrowBand", "Narrow Band (cm)"), &FRopeSDFBakeSettings::NarrowBand, 1.0f, 50.0f) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("WeightThresh", "Weight Threshold"), &FRopeSDFBakeSettings::WeightThreshold, 0.0f, 1.0f) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("BoundsPad", "Bounds Padding (cm)"), &FRopeSDFBakeSettings::BoundsPadding, 0.0f, 20.0f) ]

			// 프리뷰 오버레이(자산 비변경 · 패널 로컬). RopeSDFDraw 헬퍼로 그린다.
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 14.0f, 0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("OverlayHeader", "Preview Overlay"))
			]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeOverlayToggleRow(LOCTEXT("DrawBounds", "Bounds"), &FRopeSDFPreviewDrawOptions::bDrawBounds) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeOverlayToggleRow(LOCTEXT("DrawVoxels", "Voxels (narrow band)"), &FRopeSDFPreviewDrawOptions::bDrawVoxels) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeOverlayToggleRow(LOCTEXT("DrawSlice", "Slice heatmap"), &FRopeSDFPreviewDrawOptions::bDrawSlice) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeOverlayToggleRow(LOCTEXT("DrawGradient", "Gradients"), &FRopeSDFPreviewDrawOptions::bDrawGradient) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakePreviewFloatRow(LOCTEXT("Band", "Band Threshold (cm)"), &FRopeSDFPreviewDrawOptions::BandThreshold, 0.0f, 50.0f) ]

			// Slice 파라미터: 축(순환 버튼) / 위치 / 해상도 / 색 스케일.
			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
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

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakePreviewFloatRow(LOCTEXT("GradLen", "Gradient Length (cm)"), &FRopeSDFPreviewDrawOptions::GradientLength, 0.5f, 20.0f) ]
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
	float FRopeSDFBakeSettings::* Member, float MinVal, float MaxVal)
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
			.Value_Lambda([this, Member]() { return TOptional<float>(Settings.*Member); })
			.OnValueChanged_Lambda([this, Member](float NewVal) { Settings.*Member = NewVal; })
		];
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeIntRow(const FText& Label,
	int32 FRopeSDFBakeSettings::* Member, int32 MinVal, int32 MaxVal)
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
			.Value_Lambda([this, Member]() { return TOptional<int32>(Settings.*Member); })
			.OnValueChanged_Lambda([this, Member](int32 NewVal) { Settings.*Member = NewVal; })
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

FString SRopeSDFAuthoringPanel::GetTargetPath() const
{
	return Target.IsValid() ? Target->GetPathName() : FString();
}

void SRopeSDFAuthoringPanel::OnTargetChanged(const FAssetData& InAssetData)
{
	Target = Cast<URopeSDFData>(InAssetData.GetAsset());
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
		[&Slow]() -> bool { return Slow.ShouldCancel(); });

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
	Data->MarkPackageDirty();

	UE_LOG(LogRopeSDFBake, Log, TEXT("Baked %s: %d bone volume(s) (unsaved — press Save)."),
		*Data->GetName(), Data->BoneVolumes.Num());

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
