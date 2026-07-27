// Copyright Epic Games, Inc. All Rights Reserved.

#include "SDF/SRopeSDFPreviewViewport.h"
#include "SDF/RopeSDFDraw.h"

#include "AdvancedPreviewScene.h"
#include "EditorViewportClient.h"
#include "Animation/DebugSkelMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "Collision/SDF/RopeSDFData.h"
#include "ToolMenus.h"
#include "ViewportToolbar/UnrealEdViewportToolbar.h"
#include "ViewportToolbar/UnrealEdViewportToolbarContext.h"
// UE_VERSION_OLDER_THAN, needed for the version guard because the viewport toolbar submenus below are a 5.6 and later API.
#include "Misc/EngineVersionComparison.h"

#define LOCTEXT_NAMESPACE "RopeSDFPreviewViewport"

//////////////////////////////////////////////////////////////////////////
// FRopeSDFPreviewViewportClient — an orbit camera plus a tick of the preview world.

class FRopeSDFPreviewViewportClient : public FEditorViewportClient
{
public:
	FRopeSDFPreviewViewportClient(FAdvancedPreviewScene& InPreviewScene,
		const TSharedRef<SRopeSDFPreviewViewport>& InViewport)
		: FEditorViewportClient(nullptr, &InPreviewScene, StaticCastSharedRef<SEditorViewport>(InViewport))
		, AdvancedScene(&InPreviewScene)
		, ViewportWidget(InViewport)
	{
		// The preview is always realtime, so animation and bone updates never stop.
		SetRealtime(true);

		// The orbit camera's default placement, looking at the origin from slightly above and behind.
		SetViewLocation(FVector(0.0f, -200.0f, 120.0f));
		SetViewRotation(FRotator(-15.0f, 90.0f, 0.0f));
		SetViewLocationForOrbiting(FVector::ZeroVector);

		// Game mode show flags, so the editor lighting and materials look correct.
		EngineShowFlags.SetSnap(false);
		EngineShowFlags.SetGrid(true);
		EngineShowFlags.SetCompositeEditorPrimitives(true);

		// The preview has nothing to edit, so the transform gizmo is turned off, removing the move widget floating at the origin.
		ShowWidget(false);

		bSetListenerPosition = false;
	}

	virtual void Tick(float DeltaSeconds) override
	{
		FEditorViewportClient::Tick(DeltaSeconds);

		// Ticks the preview world directly so the bone transforms update, which the SDF overlay depends on.
		if (AdvancedScene && !GIntraFrameDebuggingGameThread)
		{
			AdvancedScene->GetWorld()->Tick(LEVELTICK_All, DeltaSeconds);
		}
	}

	virtual void Draw(const FSceneView* View, FPrimitiveDrawInterface* PDI) override
	{
		FEditorViewportClient::Draw(View, PDI);

		// The per-bone SDF overlay is drawn by the widget, which owns the data and the mesh component, using the same helpers as the level visualizer.
		if (TSharedPtr<SRopeSDFPreviewViewport> PreviewWidget = ViewportWidget.Pin())
		{
			PreviewWidget->DrawSDFOverlay(PDI);
		}
	}

private:
	FAdvancedPreviewScene* AdvancedScene = nullptr;
	TWeakPtr<SRopeSDFPreviewViewport> ViewportWidget;
};

//////////////////////////////////////////////////////////////////////////
// SRopeSDFPreviewViewport

SRopeSDFPreviewViewport::SRopeSDFPreviewViewport() = default;
SRopeSDFPreviewViewport::~SRopeSDFPreviewViewport() = default;

void SRopeSDFPreviewViewport::Construct(const FArguments& InArgs)
{
	FAdvancedPreviewScene::ConstructionValues CVS;
	CVS.bShouldSimulatePhysics = false;
	CVS.bCreatePhysicsScene = false;
	PreviewScene = MakeShared<FAdvancedPreviewScene>(CVS);

	// The mesh component is created ahead of time, with the mesh itself assigned in SetPreviewMesh, and registered with the scene.
	PreviewMeshComponent = NewObject<UDebugSkelMeshComponent>(GetTransientPackage());
	PreviewScene->AddComponent(PreviewMeshComponent, FTransform::Identity);

	SEditorViewport::Construct(SEditorViewport::FArguments());
}

void SRopeSDFPreviewViewport::SetPreviewMesh(USkeletalMesh* InMesh)
{
	if (!PreviewMeshComponent)
	{
		return;
	}

	PreviewMeshComponent->SetSkeletalMesh(InMesh);

	if (InMesh)
	{
		// Pinned to the reference pose with no animation, since the SDF is baked in bone-local space and this is the most honest inspection view.
		PreviewMeshComponent->SetAnimationMode(EAnimationMode::AnimationSingleNode);
		PreviewMeshComponent->SetAnimation(nullptr);
		PreviewMeshComponent->InitAnim(false);
		PreviewMeshComponent->RefreshBoneTransforms();
	}

	// Refits the camera to the new mesh's bounds.
	if (ViewportClient.IsValid())
	{
		if (InMesh)
		{
			const FBoxSphereBounds Bounds = PreviewMeshComponent->Bounds;
			ViewportClient->FocusViewportOnBox(Bounds.GetBox());
		}
		ViewportClient->Invalidate();
	}
}

void SRopeSDFPreviewViewport::SetPreviewData(URopeSDFData* InData)
{
	// Fixes the bake result as of the moment of the call, as a copy, rather than reading the live asset every frame.
	// That is what keeps the viewport from updating the instant a bake changes the asset, so it takes effect on a refresh, meaning another call, alone.
	if (InData)
	{
		PreviewVolumes = InData->BoneVolumes;
	}
	else
	{
		PreviewVolumes.Reset();
	}
	InvalidatePreview();
}

void SRopeSDFPreviewViewport::InvalidatePreview()
{
	if (ViewportClient.IsValid())
	{
		ViewportClient->Invalidate();
	}
}

void SRopeSDFPreviewViewport::DrawSDFOverlay(FPrimitiveDrawInterface* PDI)
{
	if (!PDI || !PreviewMeshComponent || !DrawOptions.AnyEnabled())
	{
		return;
	}

	// Draws the snapshot fixed by SetPreviewData rather than the live asset, so a bake is not reflected until a refresh.
	for (const FRopeBoneSDFVolume& Vol : PreviewVolumes)
	{
		if (Vol.Bone.IsNone())
		{
			continue;
		}

		// The preview mesh's world bone transforms. The SDF is baked in bone-local space, so they are used for the
		// world placement directly, reconstructing the world bone transform through GetSocketTransform, which agrees
		// with the bone-local frame the bake used.
		const FTransform Xform = PreviewMeshComponent->GetSocketTransform(Vol.Bone);

		if (DrawOptions.bDrawBounds)
		{
			RopeSDFDraw::DrawBounds(PDI, Vol.LocalBounds, Xform, FLinearColor(1.0f, 0.6f, 0.0f));
		}
		if (DrawOptions.bDrawVoxels && Vol.IsBaked())
		{
			RopeSDFDraw::DrawVoxels(PDI, Vol, Xform, DrawOptions.BandThreshold);
		}
		if (DrawOptions.bDrawSlice && Vol.IsBaked())
		{
			RopeSDFDraw::DrawSlice(PDI, Vol, Xform, DrawOptions.SliceAxis, DrawOptions.SlicePosition,
				DrawOptions.SliceResolution, DrawOptions.SliceColorScale);
		}
		if (DrawOptions.bDrawGradient && Vol.IsBaked())
		{
			RopeSDFDraw::DrawGradients(PDI, Vol, Xform, DrawOptions.BandThreshold, DrawOptions.GradientLength);
		}
	}
}

TSharedRef<FEditorViewportClient> SRopeSDFPreviewViewport::MakeEditorViewportClient()
{
	ViewportClient = MakeShared<FRopeSDFPreviewViewportClient>(*PreviewScene, SharedThis(this));
	return ViewportClient.ToSharedRef();
}

TSharedPtr<SWidget> SRopeSDFPreviewViewport::BuildViewportToolbar()
{
	// The toolbar menus go into the global UToolMenus registry, so they are registered exactly once, in case the tab is reopened.
	const FName ToolbarName = "DynamicRope.SDFPreviewViewportToolbar";
	if (!UToolMenus::Get()->IsMenuRegistered(ToolbarName))
	{
		UToolMenu* Menu = UToolMenus::Get()->RegisterMenu(ToolbarName, NAME_None, EMultiBoxType::SlimHorizontalToolBar);
		Menu->StyleName = "ViewportToolbar";

	// Placed right-aligned, as in the main level viewport.
		FToolMenuSection& RightSection = Menu->AddSection("Right");
		RightSection.Alignment = EToolMenuSectionAlign::Last;

		// The camera and view mode submenus are part of the viewport toolbar API introduced in 5.6,
		// UE::UnrealEd::Create*Submenu. They do not exist in 5.5 and would break the compile, so they are guarded;
		// in 5.5 the toolbar appears without those two submenus.
#if !UE_VERSION_OLDER_THAN(5, 6, 0)
		// The camera submenu, including the movement speed options. The speed menu inside it is automatically
		// promoted to the top level of the toolbar by SetShowInToolbarTopLevel(true), matching the main level
		// viewport, so adding a separate speed entry would duplicate it.
		RightSection.AddEntry(UE::UnrealEd::CreateCameraSubmenu(
			UE::UnrealEd::FViewportCameraMenuOptions().ShowCameraMovement()));

		// The view modes, lit, unlit, wireframe and so on, so the overlay can be inspected in wireframe when it is
		// inside the mesh. The switching commands are already bound by SEditorViewport::BindCommands and are merely
		// exposed as a menu here.
		RightSection.AddEntry(UE::UnrealEd::CreateViewModesSubmenu());
#endif
	}

	// The context in which the menu widget is built: this viewport's command list plus a reference to the viewport, which the camera speed and similar entries use.
	FToolMenuContext Context;
	Context.AppendCommandList(GetCommandList());
	UUnrealEdViewportToolbarContext* ContextObject = NewObject<UUnrealEdViewportToolbarContext>();
	ContextObject->Viewport = SharedThis(this);
	Context.AddObject(ContextObject);

	return UToolMenus::Get()->GenerateWidget(ToolbarName, Context);
}

void SRopeSDFPreviewViewport::AddReferencedObjects(FReferenceCollector& Collector)
{
	Collector.AddReferencedObject(PreviewMeshComponent);
}

#undef LOCTEXT_NAMESPACE
