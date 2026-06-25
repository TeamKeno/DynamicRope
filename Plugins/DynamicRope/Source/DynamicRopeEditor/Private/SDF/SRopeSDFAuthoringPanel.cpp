// Copyright Epic Games, Inc. All Rights Reserved.

#include "SRopeSDFAuthoringPanel.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Styling/AppStyle.h"

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
				.Text(LOCTEXT("Placeholder",
					"Per-bone SDF authoring/baking panel (scaffold). Target URopeSDFData selection, bone list, and bake controls will live here."))
			]

			+ SVerticalBox::Slot()
			.AutoHeight()
			[
				SNew(SButton)
				.Text(LOCTEXT("Bake", "Bake (TODO)"))
				.IsEnabled(false)
				.OnClicked(this, &SRopeSDFAuthoringPanel::OnBakeClicked)
			]
		]
	];
}

FReply SRopeSDFAuthoringPanel::OnBakeClicked()
{
	// TODO(B3): run the per-bone SDF bake for the selected URopeSDFData.
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
