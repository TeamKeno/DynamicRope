// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The 3D preview viewport embedded in the SDF authoring panel. It shows the skeletal mesh being baked,
// URopeSDFData::SourceMesh, in its reference pose over an FAdvancedPreviewScene providing lighting and a floor, for
// inspection with an orbit camera.
// The SDF overlay, meaning the bounds, voxels, slice and gradient, is drawn from the viewport client's Draw, which
// renders each bone's volume through the RopeSDFDraw helpers.
// The toggles and parameters live in this widget's FRopeSDFPreviewDrawOptions, which is panel-local state, rather
// than in the asset.

#pragma once

#include "CoreMinimal.h"
#include "SEditorViewport.h"
#include "UObject/GCObject.h"
// ERopeSDFSliceAxis, for the overlay option defaults.
#include "Collision/SDF/RopeSDFProvider.h"
// FRopeBoneSDFVolume, held as the overlay snapshot.
#include "Collision/SDF/RopeSDFData.h"

class FAdvancedPreviewScene;
class FRopeSDFPreviewViewportClient;
class FPrimitiveDrawInterface;
class UDebugSkelMeshComponent;
class URopeSDFData;
class USkeletalMesh;

/**
 * Display toggles and parameters for the preview SDF overlay. This is panel-local state, separate from the runtime
 * URopeSDFProvider's editor-only toggles, and never dirties the asset, having no dependency on the provider. The
 * defaults match the level visualizer.
 */
struct FRopeSDFPreviewDrawOptions
{
	bool bDrawBounds = true;
	bool bDrawVoxels = false;
	bool bDrawSlice = false;
	bool bDrawGradient = false;

	/** The band thickness for voxel and gradient display, in centimetres. Only samples whose absolute distance is within it are drawn. */
	float BandThreshold = 3.0f;

	/** The axis the slice plane passes through. */
	ERopeSDFSliceAxis SliceAxis = ERopeSDFSliceAxis::Z;
	/** The slice position along that axis, from zero to one. */
	float SlicePosition = 0.5f;
	/** The number of slice samples along one side of the grid. */
	int32 SliceResolution = 24;
	/** The slice colour mapping scale, in centimetres, at which the absolute distance saturates. */
	float SliceColorScale = 2.5f;

	/** The gradient arrow length, in centimetres. */
	float GradientLength = 4.0f;

	bool AnyEnabled() const { return bDrawBounds || bDrawVoxels || bDrawSlice || bDrawGradient; }
};

/**
 * The preview viewport widget placed on the right of the authoring panel. It owns the preview scene and the mesh
 * component, and protects the mesh component from collection through FGCObject.
 */
class SRopeSDFPreviewViewport : public SEditorViewport, public FGCObject
{
public:
	SLATE_BEGIN_ARGS(SRopeSDFPreviewViewport) {}
	SLATE_END_ARGS()

	SRopeSDFPreviewViewport();
	virtual ~SRopeSDFPreviewViewport() override;

	void Construct(const FArguments& InArgs);

	/** Replaces the mesh being previewed. A null mesh leaves an empty scene, with the mesh removed. */
	void SetPreviewMesh(USkeletalMesh* InMesh);

	/**
	 * Snapshots the per-bone volumes the SDF overlay draws, as a copy, from this asset's current bone volumes. It is
	 * fixed to the data as of the call, so a later bake does not update it until SetPreviewData is called again,
	 * meaning a bake is not reflected in the viewport until a refresh. A null asset clears the overlay.
	 */
	void SetPreviewData(URopeSDFData* InData);

	/** The entry point through which the panel UI reads and writes the toggles and parameters. Calling InvalidatePreview() after a change is recommended. */
	FRopeSDFPreviewDrawOptions& AccessDrawOptions() { return DrawOptions; }

	/**
	 * Whether there is a per-bone volume snapshot for the overlay to draw. It is the basis on which the panel enables
	 * or disables the overlay controls: the precise condition is whether the viewport has data to draw right now
	 * rather than whether a bake has happened, since the snapshot updates only on SetPreviewData or a refresh.
	 */
	bool HasPreviewVolumes() const { return PreviewVolumes.Num() > 0; }

	/** Forces a redraw on the next frame, so a toggle or parameter change takes effect. */
	void InvalidatePreview();

	/** Called from the viewport client's Draw to render the per-bone SDF overlay through the PDI. */
	void DrawSDFOverlay(FPrimitiveDrawInterface* PDI);

	// FGCObject
	virtual void AddReferencedObjects(FReferenceCollector& Collector) override;
	virtual FString GetReferencerName() const override { return TEXT("SRopeSDFPreviewViewport"); }

protected:
	// SEditorViewport
	virtual TSharedRef<FEditorViewportClient> MakeEditorViewportClient() override;
	/** The viewport's top toolbar: the camera menu, including its speed, and the view mode menu, which switches wireframe on. */
	virtual TSharedPtr<SWidget> BuildViewportToolbar() override;

private:
	/** The preview world providing the lighting, floor and environment. */
	TSharedPtr<FAdvancedPreviewScene> PreviewScene;

	/** The mesh component added to the preview scene, in its reference pose. The SDF overlay takes its bone transforms from it. */
	TObjectPtr<UDebugSkelMeshComponent> PreviewMeshComponent;

	/** The viewport client responsible for the camera and rendering. */
	TSharedPtr<FRopeSDFPreviewViewportClient> ViewportClient;

	/** A snapshot copy of the per-bone volumes the overlay draws, updated only when SetPreviewData is called, which
	    decouples it from the live asset. Skipping saturated samples and greying them out is decided directly from
	    each volume's asymmetric inner and outer narrow band, so no separate field is needed. */
	TArray<FRopeBoneSDFVolume> PreviewVolumes;

	/** The overlay's display toggles and parameters, as panel-local state. */
	FRopeSDFPreviewDrawOptions DrawOptions;
};
