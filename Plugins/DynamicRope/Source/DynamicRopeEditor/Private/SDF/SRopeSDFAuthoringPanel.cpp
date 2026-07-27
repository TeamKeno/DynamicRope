// Copyright Epic Games, Inc. All Rights Reserved.

#include "SRopeSDFAuthoringPanel.h"
#include "SRopeSDFPreviewViewport.h"
#include "RopeSDFBaker.h"
#include "DynamicRopeEditorLog.h"
#include "Collision/SDF/RopeSDFData.h"

// Reporting the bake result, including coarsening.
#include "Logging/MessageLog.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SSplitter.h"
#include "Widgets/Layout/SBorder.h"
// The collapsible section holding the advanced bake settings.
#include "Widgets/Layout/SExpandableArea.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
// The legend's colour swatches.
#include "Widgets/Colors/SColorBlock.h"
// Fixing the swatch size.
#include "Widgets/Layout/SBox.h"
#include "Styling/AppStyle.h"
// SObjectPropertyEntryBox
#include "PropertyCustomizationHelpers.h"
// Creating the embedded details view.
#include "PropertyEditorModule.h"
#include "IDetailsView.h"
#include "Modules/ModuleManager.h"
#include "Engine/SkeletalMesh.h"
#include "Misc/ScopedSlowTask.h"
#include "Framework/Notifications/NotificationManager.h"
#include "Widgets/Notifications/SNotificationList.h"

// A temporary automatic save after baking.
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "Misc/PackageName.h"

#define LOCTEXT_NAMESPACE "RopeSDFAuthoring"

void SRopeSDFAuthoringPanel::Construct(const FArguments& InArgs)
{
	// The embedded details view showing the target asset's own properties. Double-clicking opens this tab
	// instead of the generic property editor, so the source mesh and bone volumes are inspected and edited
	// here. It is created before being placed in the child slot.
	{
		FPropertyEditorModule& PropertyModule =
			FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
		FDetailsViewArgs DetailsArgs;
		DetailsArgs.bHideSelectionTip = true;
		// The target's name is already shown by the picker above.
		DetailsArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;
		DetailsView = PropertyModule.CreateDetailView(DetailsArgs);
		// Covers the path where the source mesh changes through the details view rather than the picker,
		// which needs its own hook.
		DetailsView->OnFinishedChangingProperties().AddSP(this, &SRopeSDFAuthoringPanel::OnAssetPropertyChanged);
	}

	ChildSlot
	[
		SNew(SSplitter)
		.Orientation(Orient_Horizontal)

		// Left: the controls, meaning the target picker, the bake button and the settings.
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

			// The target asset picker.
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

				// Bake produces the per-bone SDFs with the current settings, writing them into the asset in
				// memory; committing to disk is what Save does.
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

				// Save writes the bake result to disk. It is enabled only when there are unsaved changes and
				// marks itself accordingly.
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

				// Refresh redraws the preview viewport from the currently baked data.
				// It is applied automatically right after a bake, so this is only for a manual refresh.
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

			// The bake settings, editable before baking.
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 14.0f, 0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("SettingsHeader", "Bake Settings (last bake)"))
			]

			// The main knobs: the values an everyday user works with, namely the quality target, the
			// collision band, and dropping thin bones.
			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("VoxelSize", "Voxel Size (cm)"), &FRopeSDFBakeSettings::VoxelSize, 0.25f, 10.0f,
				LOCTEXT("VoxelSizeTip", "Sample spacing in cm (cube voxel). Smaller sharpens the surface but increases memory and bake time.")) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("NarrowBand", "Narrow Band - outward (cm)"), &FRopeSDFBakeSettings::NarrowBand, 1.0f, 50.0f,
				LOCTEXT("NarrowBandTip", "Outward (free-space) detection band in cm: how far outside the surface the rope starts reacting to the body. Contact happens at CollisionRadius, so ~2-3x that is stable. The inward (inside-body) band is auto-sized per bone to the deepest interior distance at bake, so the whole interior is covered - no setting needed.")) ]

			// The thin-bone drop threshold. A bone whose cross-sectional girth falls below it is excluded
			// from baking, which removes redundant volumes such as fingers.
			+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f)
			[ MakeFloatRow(LOCTEXT("MinGirth", "Min Bone Girth (cm)"), &FRopeSDFBakeSettings::MinBoneGirth, 0.0f, 20.0f,
				LOCTEXT("MinGirthTip", "Bones whose cross-section girth is thinner than this (cm) are dropped from baking (not merged into the parent). A rope cannot catch features finer than its radius, so set this near (or above) the CollisionRadius of the thinnest rope that will use this SDF. 0 bakes every bone.")) ]

			// Advanced settings: the maximum resolution, which bounds memory and time and can override the
			// voxel size, along with the tuning values that are rarely touched, are collapsed by default.
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

					// The quantization bit depth, which trades output size against precision. The default is
					// sufficient, so it is hidden under the advanced section.
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

			// The preview overlays, which change no asset and are panel-local. They are drawn by the
			// RopeSDFDraw helpers.
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 14.0f, 0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("OverlayHeader", "Preview Overlay"))
			]

			// With no baked data, meaning no preview volume snapshot, the controls remain visible but
			// disabled, with a hint explaining why.
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Visibility(this, &SRopeSDFAuthoringPanel::GetOverlayDisabledHintVisibility)
				.AutoWrapText(true)
				.ColorAndOpacity(FSlateColor::UseSubduedForeground())
				.Text(LOCTEXT("OverlayDisabledHint",
					"No baked data to preview. Bake the asset to enable these overlays."))
			]

			// Every overlay control is wrapped in one container, so that with no preview volume the disabled
			// state propagates to all of its children and the whole section greys out; what debugging is
			// available stays visible while being unusable.
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

					// The band threshold is shared by the voxel and gradient groups. It appears in both, but
					// both bind to the same member, so changing one updates the other automatically.
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

					// The slice parameters: the axis, through a cycling button, plus the position, the
					// resolution and the colour scale.
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

					// The band threshold shared with the voxel group, bound to the same member and therefore
					// synchronized automatically.
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 4.0f, 0.0f, 0.0f)
					[ MakeBandThresholdRow() ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 2.0f, 0.0f, 0.0f)
					[ MakePreviewFloatRow(LOCTEXT("GradLen", "Gradient Length (cm)"), &FRopeSDFPreviewDrawOptions::GradientLength, 0.5f, 20.0f) ]
				]
			]

			// The target asset's own properties, namely the source mesh and the bone volumes. It views the
			// live asset directly, so a bake result appears immediately. The customization that puts the
			// bone name in each array element's header applies here as well.
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 14.0f, 0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("DetailsHeader", "Asset Details"))
			]

			// It takes the remaining vertical space and scrolls internally; the left column itself does not
			// scroll.
			+ SVerticalBox::Slot()
			.FillHeight(1.0f)
			[
				DetailsView.ToSharedRef()
			]
		]
		]

		// Right: the 3D preview viewport, showing the mesh being baked and the SDF overlays.
		+ SSplitter::Slot()
		.Value(0.6f)
		[
			SNew(SOverlay)

			+ SOverlay::Slot()
			[
				SAssignNew(PreviewViewport, SRopeSDFPreviewViewport)
			]

			// The hint shown only when there is no mesh to preview.
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

	// A target may already be assigned when the panel is created, so it is applied once here.
	RefreshPreviewMesh();
}

TSharedRef<SWidget> SRopeSDFAuthoringPanel::MakeFloatRow(const FText& Label,
	float FRopeSDFBakeSettings::* Member, float MinVal, float MaxVal, const FText& Tip)
{
	// The tooltip is attached to the row container: Slate walks up from the hovered widget looking for one,
	// so hovering either the label or the entry box shows the same tooltip. An empty tip shows nothing.
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
	// The tooltip is attached to the row container, on the same convention as the float row. An empty tip
	// shows nothing.
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
	// The two options behave like radio buttons: only the active one is checked, and selecting the other
	// changes the setting so the previous one unchecks automatically. Clicking the already-active one to
	// uncheck it is ignored, so one is always selected.
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
	// The correct criterion is not whether a bake has happened but whether the viewport has a volume
	// snapshot to draw. After a bake the automatic refresh fills that in, but before it, as when an asset
	// has merely been selected, there is nothing to draw and the controls stay disabled.
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
	// The narrow band used at bake time is the upper limit of the useful display range; beyond it everything
	// saturates at the band limits and means nothing.
	if (const URopeSDFData* Data = Target.Get())
	{
		return FMath::Max(Data->LastBakeSettings.NarrowBand, KINDA_SMALL_NUMBER);
	}
	// The fallback when there is no target, which is the disabled state.
	return 50.0f;
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
					// A value entered above the limit is clamped before it is stored.
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
	SetTargetAsset(Cast<URopeSDFData>(InAssetData.GetAsset()));
}

void SRopeSDFAuthoringPanel::SetTargetAsset(URopeSDFData* InData)
{
	Target = InData;

	// For an already-baked asset the settings used at the time are restored into the panel, so a designer
	// can adjust them while comparing against the current result. An unbaked asset keeps the defaults,
	// which are the starting point for a new bake.
	if (URopeSDFData* Data = Target.Get(); Data && Data->HasAnyBakedVolume())
	{
		Settings = Data->LastBakeSettings;
	}
	else
	{
		Settings = FRopeSDFBakeSettings();
	}

	// Changing the target also changes the narrow band, and therefore the band threshold's upper limit, so
	// a value left over from the previous target is clamped to the new limit; the slider's limit only
	// constrains new input and does not reduce a value already stored.
	if (PreviewViewport.IsValid())
	{
		float& Band = PreviewViewport->AccessDrawOptions().BandThreshold;
		Band = FMath::Clamp(Band, 0.0f, GetBandThresholdMax());
	}

	// Point the embedded details view at the new target as well; a null target gives an empty details view.
	if (DetailsView.IsValid())
	{
		DetailsView->SetObject(InData);
	}

	RefreshPreviewMesh();
}

void SRopeSDFAuthoringPanel::OnAssetPropertyChanged(const FPropertyChangedEvent& Event)
{
	// Changing the source mesh through the details view does not go through the picker's handler, so the
	// preview is refreshed here.
	if (Event.GetPropertyName() == GET_MEMBER_NAME_CHECKED(URopeSDFData, SourceMesh))
	{
		RefreshPreviewMesh();
	}
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
	// It is a soft reference, so it is loaded synchronously while authoring; a missing or failed load gives
	// an empty view.
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

	// The total progress is treated as one, and the baker advances it by one over the bone count as each
	// bone begins.
	// A cancel button is shown, and the callback returning false, after seeing that cancellation was
	// requested, aborts the bake.
	FScopedSlowTask Slow(1.0f, LOCTEXT("Baking", "Baking per-bone SDF..."));
	Slow.MakeDialog(true /*bShowCancelButton*/);

	// During a bake the slow task pumps Slate periodically to process the cancel button, and the user could
	// change the settings or the bone filter in that window, so both are snapshotted at the moment of the
	// call and passed in, which keeps the inputs stable for the duration.
	const FRopeSDFBakeSettings SettingsSnapshot = Settings;
	const TArray<FName> BoneFilterSnapshot = BoneFilter;

	TArray<FRopeBoneSDFVolume> Volumes;
	FRopeSDFBakeStats Stats;
	const ERopeSDFBakeResult Result = FRopeSDFBaker::BakeMesh(Mesh, BoneFilterSnapshot, SettingsSnapshot, Volumes,
	// Per bone: advance the progress by one step, which also pumps the UI, then read whether it was
	// cancelled.
		[&Slow](int32 Done, int32 Total, const FName& Bone) -> bool
		{
			const float Frac = (Total > 0) ? (1.0f / static_cast<float>(Total)) : 1.0f;
			Slow.EnterProgressFrame(Frac, FText::Format(
				LOCTEXT("BakingBone", "Baking SDF: {0} ({1}/{2})"),
				FText::FromName(Bone), FText::AsNumber(Done + 1), FText::AsNumber(Total)));
			return !Slow.ShouldCancel();
		},
	// Between voxel batches within a bone: pump the UI, inside the cancellation check, and process a click
	// on cancel.
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
		// The asset is left untouched, discarding the partial result, so the previous bake survives.
		FNotificationInfo Info(LOCTEXT("BakeCancelled", "Bake cancelled — asset unchanged."));
		Info.ExpireDuration = 4.0f;
		FSlateNotificationManager::Get().AddNotification(Info);
		return FReply::Handled();
	}

	// The bake result is applied to the asset in memory and the package is marked dirty. Writing to disk is
	// what the Save button does. The viewport is updated directly below.
	Data->Modify();
	Data->BoneVolumes = MoveTemp(Volumes);
	// Record the settings actually used for the bake on the asset, so that reopening it restores them into
	// the panel and they can be adjusted against the current result.
	Data->LastBakeSettings = SettingsSnapshot;
	Data->MarkPackageDirty();

	// Apply the bake result to the preview viewport immediately, with no need to press Refresh.
	RefreshPreviewOverlay();

	UE_LOG(LogRopeSDFBake, Log, TEXT("Baked %s: %d bone volume(s) (unsaved — press Save)."),
		*Data->GetName(), Data->BoneVolumes.Num());

	// Report the bake result to the message log. Where the maximum resolution limit forced a bone to be
	// baked more coarsely than the requested voxel size, the bone's name and the requested against actual
	// sizes are listed as a warning and the log window is brought forward.
	// That makes the silent override visible; the detail stays in the message log rather than a toast.
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
	// Bring the log window forward when anything was coarsened.
			Log.Open(EMessageSeverity::Warning);
		}
		else
		{
			Log.Info(FText::Format(
				LOCTEXT("BakeClean", "Baked {0} bone volume(s) at {1} cm — no coarsening."),
				FText::AsNumber(Stats.BonesBaked), FText::AsNumber(SettingsSnapshot.VoxelSize)));
		}

		// Report the thin bones dropped by the minimum girth setting on one comma-separated line. Nothing is
		// reported when none were dropped.
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
		LOCTEXT("Baked", "Baked {0} bone volume(s). Press Save to write to disk."),
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
	// Mark the button when saving is needed, which signals the unsaved state.
	return CanSave() ? LOCTEXT("SaveDirty", "Save *") : LOCTEXT("Save", "Save");
}

FReply SRopeSDFAuthoringPanel::OnSaveClicked()
{
	URopeSDFData* Data = Target.Get();
	if (!Data)
	{
		return FReply::Handled();
	}

	// A read-only Perforce file fails to write; integrating with source control checkout is future work.
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
	RefreshPreviewOverlay();
	return FReply::Handled();
}

void SRopeSDFAuthoringPanel::RefreshPreviewOverlay()
{
	// The mesh is left alone so the camera is preserved, and only the source of the baked data is
	// reassigned to redraw the overlays.
	if (PreviewViewport.IsValid())
	{
		PreviewViewport->SetPreviewData(Target.Get());
		PreviewViewport->InvalidatePreview();
	}
}

#undef LOCTEXT_NAMESPACE
