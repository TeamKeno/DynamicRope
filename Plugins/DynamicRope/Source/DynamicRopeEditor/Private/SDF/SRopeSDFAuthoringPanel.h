// Copyright Epic Games, Inc. All Rights Reserved.
//
// Content panel (scaffold) for the SDF authoring dock tab. Will host a mesh picker, bone list,
// bake controls, and a details view for the target URopeSDFData. For now it only lays out the
// layout and entry points.

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
	/** Runs the per-bone bake for the selected target, writes it back, and (temporarily) saves. */
	FReply OnBakeClicked();

	/** Target URopeSDFData picked in the asset entry box. */
	TWeakObjectPtr<URopeSDFData> Target;

	/** Bake knobs (defaults for now; UI to follow). */
	FRopeSDFBakeSettings Settings;

	/** Bones to bake. Empty = every skinned bone (v1). A picker UI comes later. */
	TArray<FName> BoneFilter;

	FString GetTargetPath() const;
	void OnTargetChanged(const FAssetData& InAssetData);
	bool CanBake() const;

	/** Builds a labeled numeric row bound to a FRopeSDFBakeSettings member (kept DRY across fields). */
	TSharedRef<class SWidget> MakeFloatRow(const FText& Label, float FRopeSDFBakeSettings::* Member, float MinVal, float MaxVal);
	TSharedRef<class SWidget> MakeIntRow(const FText& Label, int32 FRopeSDFBakeSettings::* Member, int32 MinVal, int32 MaxVal);
};
