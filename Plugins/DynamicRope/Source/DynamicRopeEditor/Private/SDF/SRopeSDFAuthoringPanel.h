// Copyright Epic Games, Inc. All Rights Reserved.
//
// Content panel (scaffold) for the SDF authoring dock tab. Will host a mesh picker, bone list,
// bake controls, and a details view for the target URopeSDFData. For now it only lays out the
// layout and entry points.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

class SRopeSDFAuthoringPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SRopeSDFAuthoringPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	/** Bake trigger (stub). */
	FReply OnBakeClicked();
};
