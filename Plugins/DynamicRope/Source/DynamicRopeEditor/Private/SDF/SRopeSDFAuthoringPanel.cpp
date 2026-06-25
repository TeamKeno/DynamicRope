// Copyright Epic Games, Inc. All Rights Reserved.

#include "SRopeSDFAuthoringPanel.h"
#include "RopeSDFBaker.h"
#include "Collision/SDF/RopeSDFData.h"

#include "Widgets/SBoxPanel.h"
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
	];
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

	FNotificationInfo Info(FText::Format(
		LOCTEXT("Baked", "Baked {0} bone volume(s).{1}"),
		FText::AsNumber(Data->BoneVolumes.Num()),
		bSaved ? FText::GetEmpty() : LOCTEXT("SaveFailed", " (auto-save failed — save manually)")));
	Info.ExpireDuration = 5.0f;
	FSlateNotificationManager::Get().AddNotification(Info);
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
