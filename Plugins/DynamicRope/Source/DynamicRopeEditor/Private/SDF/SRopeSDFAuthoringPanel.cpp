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
				SNew(SButton)
				.Text(LOCTEXT("Bake", "Bake"))
				.IsEnabled(this, &SRopeSDFAuthoringPanel::CanBake)
				.OnClicked(this, &SRopeSDFAuthoringPanel::OnBakeClicked)
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

	FScopedSlowTask Slow(1.0f, LOCTEXT("Baking", "Baking per-bone SDF..."));
	Slow.MakeDialog();

	TArray<FRopeBoneSDFVolume> Volumes;
	const bool bOk = FRopeSDFBaker::BakeMesh(Mesh, BoneFilter, Settings, Volumes);
	if (!bOk)
	{
		FNotificationInfo Info(LOCTEXT("NoGeom", "Bake failed: mesh has no CPU geometry (cooked/stripped)."));
		Info.ExpireDuration = 4.0f;
		FSlateNotificationManager::Get().AddNotification(Info);
		return FReply::Handled();
	}

	Data->Modify();
	Data->BoneVolumes = MoveTemp(Volumes);
	Data->MarkPackageDirty();

	// 임시: 베이크 직후 패키지를 자동저장한다. (read-only Perforce 파일이면 여기서 쓰기 실패 —
	// 소스컨트롤 체크아웃 연동은 추후 개선.)
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
		UE_LOG(LogRopeSDFBake, Log, TEXT("Baked %s: %d bone volume(s), auto-saved."),
			*Data->GetName(), Data->BoneVolumes.Num());
	}
	else
	{
		UE_LOG(LogRopeSDFBake, Warning, TEXT("Baked %s: %d bone volume(s), but auto-save failed (read-only? save manually)."),
			*Data->GetName(), Data->BoneVolumes.Num());
	}

	FNotificationInfo Info(FText::Format(
		LOCTEXT("Baked", "Baked {0} bone volume(s).{1}"),
		FText::AsNumber(Data->BoneVolumes.Num()),
		bSaved ? FText::GetEmpty() : LOCTEXT("SaveFailed", " (auto-save failed — save manually)")));
	Info.ExpireDuration = 5.0f;
	FSlateNotificationManager::Get().AddNotification(Info);
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
