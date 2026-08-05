// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The content panel of the SDF authoring dock tab. It selects a target URopeSDFData, bakes the per-bone
// SDFs, and hosts the controls for adjusting the pre-bake settings such as the voxel size.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
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

	/** Assigns the target asset from outside, which is the entry point used when double-clicking an asset
	    opens the tab. Passing nullptr clears the target.
	    It takes the same path as choosing one through the picker, including restoring the bake settings and
	    refreshing the preview. */
	void SetTargetAsset(URopeSDFData* InData);

private:
	/** Runs the per-bone bake for the selected target and writes the result into the asset in memory;
	 *  committing it to disk is what the Save button does. */
	FReply OnBakeClicked();

	/** The target URopeSDFData chosen in the asset picker. */
	TWeakObjectPtr<URopeSDFData> Target;

	/** The bake settings. */
	FRopeSDFBakeSettings Settings;

	/** The bones to bake. Empty means every skinned bone. A bone selection UI is future work. */
	TArray<FName> BoneFilter;

	FString GetTargetPath() const;
	void OnTargetChanged(const FAssetData& InAssetData);
	bool CanBake() const;

	//~ The Save and Refresh buttons.
	/** Whether there are unsaved changes, meaning the target's package is dirty. It drives both enabling
	 *  the Save button and showing its modified marker. */
	bool CanSave() const;
	/** The Save button's label, which marks it when saving is needed. */
	FText GetSaveButtonText() const;
	/** Saves the target package to disk, committing the bake result. On success the package is no longer
	 *  dirty. */
	FReply OnSaveClicked();
	/** Whether the preview viewport can be redrawn, which requires a target and a source mesh. */
	bool CanRefresh() const;
	/** Redraws the preview viewport overlays from the currently baked data, preserving the camera. */
	FReply OnRefreshClicked();
	/** The body of the overlay refresh, shared by the automatic update after a bake and the Refresh
	 *  button. */
	void RefreshPreviewOverlay();

	/** Synchronously loads the current target's source mesh and shows it in the preview viewport; without
	 *  one it shows an empty view and a hint. */
	void RefreshPreviewMesh();

	/** Called when an asset property changes in the embedded details view; a change of source mesh
	 *  refreshes the preview mesh. */
	void OnAssetPropertyChanged(const FPropertyChangedEvent& Event);

	/** The embedded details view showing the target asset's own properties, namely the source mesh and the
	    bone volumes.
	    Double-clicking opens this tab instead of the generic property editor, so inspection and editing
	    happen here. */
	TSharedPtr<IDetailsView> DetailsView;

	/** The visibility of the hint overlay, shown only when there is no mesh to preview. */
	EVisibility GetPreviewHintVisibility() const;

	/** The 3D preview viewport on the right, showing the mesh being baked and the SDF overlays. */
	TSharedPtr<SRopeSDFPreviewViewport> PreviewViewport;

	/** Builds a labelled numeric entry row bound to a member of the bake settings, which removes the
	    duplication across fields.
	    Supplying a tip attaches a hover tooltip to the whole row; leaving it empty gives none. The
	    authoring tab is custom Slate and does not read the property tooltip metadata, so tooltips are
	    attached here directly. */
	TSharedRef<class SWidget> MakeFloatRow(const FText& Label, float FRopeSDFBakeSettings::* Member, float MinVal, float MaxVal, const FText& Tip = FText::GetEmpty());
	TSharedRef<class SWidget> MakeIntRow(const FText& Label, int32 FRopeSDFBakeSettings::* Member, int32 MinVal, int32 MaxVal, const FText& Tip = FText::GetEmpty());

	/** The quantization selection row: a label plus two mutually exclusive options that behave like radio
	 *  buttons. */
	TSharedRef<class SWidget> MakeQuantizationRow();

	//~ Controls bound to the preview overlay state, which is panel-local and lives on the viewport's draw
	//~ options.
	/** An overlay toggle checkbox row, for the bounds, voxels, slice and gradient overlays. */
	TSharedRef<class SWidget> MakeOverlayToggleRow(const FText& Label, bool FRopeSDFPreviewDrawOptions::* Member);
	/** A numeric overlay parameter row, for the band, slice and gradient settings. */
	TSharedRef<class SWidget> MakePreviewFloatRow(const FText& Label, float FRopeSDFPreviewDrawOptions::* Member, float MinVal, float MaxVal);
	TSharedRef<class SWidget> MakePreviewIntRow(const FText& Label, int32 FRopeSDFPreviewDrawOptions::* Member, int32 MinVal, int32 MaxVal);
	/** The button that cycles the slice axis through X, Y and Z, plus a label showing the current one. */
	FReply OnCycleSliceAxis();
	FText GetSliceAxisLabel() const;

	//~ Overlay enablement and visibility, which depend on baked data existing and on the toggles.
	/** Whether the overlay controls can be edited, which requires a preview volume snapshot. Without one
	 *  the whole section is greyed out. */
	bool CanEditOverlay() const;
	/** The visibility of the hint shown only while editing is disabled, telling the user to bake and
	 *  refresh. */
	EVisibility GetOverlayDisabledHintVisibility() const;
	/** The visibility of a toggle-dependent group, covering its description, legend and values: visible
	 *  while that toggle is on and collapsed otherwise. */
	EVisibility GetToggleGroupVisibility(bool FRopeSDFPreviewDrawOptions::* Member) const;

	/** A one-line overlay description, in muted text. It sits inside a toggle group and says what is being
	 *  drawn. */
	TSharedRef<class SWidget> MakeOverlayDescription(const FText& Text);
	/** One line of the colour legend: a swatch plus a label, saying what each debug colour means. */
	TSharedRef<class SWidget> MakeLegendRow(const FLinearColor& Color, const FText& Label);

	/**
	 * The band threshold row. Its maximum is limited to the narrow band used at bake time, because
	 * everything beyond that saturates at the band limits and carries no direction or distance
	 * information, so raising it further would achieve nothing. The voxel and gradient groups share the
	 * same member.
	 */
	TSharedRef<class SWidget> MakeBandThresholdRow();
	/** The band threshold's maximum, taken from the target's last bake settings, with a fallback when
	 *  there is no target. */
	float GetBandThresholdMax() const;
	TOptional<float> GetBandThresholdMaxOpt() const;
};
