// Copyright Epic Games, Inc. All Rights Reserved.

#include "SDF/SRopeSDFPreviewViewport.h"
#include "SDF/RopeSDFDraw.h"

#include "AdvancedPreviewScene.h"
#include "EditorViewportClient.h"
#include "Animation/DebugSkelMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "Collision/SDF/RopeSDFData.h"

#define LOCTEXT_NAMESPACE "RopeSDFPreviewViewport"

//////////////////////////////////////////////////////////////////////////
// FRopeSDFPreviewViewportClient — orbit 카메라 + 프리뷰 월드 틱.

class FRopeSDFPreviewViewportClient : public FEditorViewportClient
{
public:
	FRopeSDFPreviewViewportClient(FAdvancedPreviewScene& InPreviewScene,
		const TSharedRef<SRopeSDFPreviewViewport>& InViewport)
		: FEditorViewportClient(nullptr, &InPreviewScene, StaticCastSharedRef<SEditorViewport>(InViewport))
		, AdvancedScene(&InPreviewScene)
		, ViewportWidget(InViewport)
	{
		// 프리뷰는 항상 실시간(애니/본 갱신이 멈추지 않도록).
		SetRealtime(true);

		// orbit 카메라 기본 배치: 살짝 위/뒤에서 원점을 바라본다.
		SetViewLocation(FVector(0.0f, -200.0f, 120.0f));
		SetViewRotation(FRotator(-15.0f, 90.0f, 0.0f));
		SetViewLocationForOrbiting(FVector::ZeroVector);

		// 에디터 라이팅/머티리얼이 정상으로 보이도록 게임 모드 show flags 기준.
		EngineShowFlags.SetSnap(false);
		EngineShowFlags.SetGrid(true);
		EngineShowFlags.SetCompositeEditorPrimitives(true);

		bSetListenerPosition = false;
	}

	virtual void Tick(float DeltaSeconds) override
	{
		FEditorViewportClient::Tick(DeltaSeconds);

		// 프리뷰 월드를 직접 틱해 본 트랜스폼이 갱신되게 한다(SDF 오버레이가 의존).
		if (AdvancedScene && !GIntraFrameDebuggingGameThread)
		{
			AdvancedScene->GetWorld()->Tick(LEVELTICK_All, DeltaSeconds);
		}
	}

	virtual void Draw(const FSceneView* View, FPrimitiveDrawInterface* PDI) override
	{
		FEditorViewportClient::Draw(View, PDI);

		// 본별 SDF 오버레이는 위젯이 그린다(데이터/메시 컴포넌트 소유자). 레벨 비주얼라이저와 동일 헬퍼.
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

	// 메시 컴포넌트를 미리 만들어 두고(메시는 SetPreviewMesh에서 지정) 씬에 등록한다.
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
		// 애니메이션 없이 ref 포즈로 고정 — SDF는 본 로컬에 구워져 있으므로 가장 정직한 검수 뷰.
		PreviewMeshComponent->SetAnimationMode(EAnimationMode::AnimationSingleNode);
		PreviewMeshComponent->SetAnimation(nullptr);
		PreviewMeshComponent->InitAnim(false);
		PreviewMeshComponent->RefreshBoneTransforms();
	}

	// 새 메시 경계에 맞춰 카메라를 다시 맞춘다.
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
	// 호출 시점의 베이크 결과를 사본으로 고정한다(라이브 자산을 매 프레임 읽지 않음). 이렇게 해야
	// Bake가 자산을 바꿔도 뷰포트가 즉시 갱신되지 않고, Refresh(=재호출) 시에만 반영된다.
	if (InData)
	{
		PreviewVolumes = InData->BoneVolumes;
		PreviewNarrowBand = InData->LastBakeSettings.NarrowBand; // 포화 스킵 기준(그라디언트)
	}
	else
	{
		PreviewVolumes.Reset();
		PreviewNarrowBand = 0.0f;
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

	// 라이브 자산이 아니라 SetPreviewData로 고정된 스냅샷을 그린다(Bake는 Refresh 전까지 반영 안 됨).
	for (const FRopeBoneSDFVolume& Vol : PreviewVolumes)
	{
		if (Vol.Bone.IsNone())
		{
			continue;
		}

		// 프리뷰 메시의 본 월드 트랜스폼. SDF는 본 로컬에 구워져 있으므로 그대로 월드 배치에 쓴다
		// (GetSocketTransform으로 본 월드 트랜스폼 재구성 — 베이크가 구운 본 로컬 프레임과 정합).
		const FTransform Xform = PreviewMeshComponent->GetSocketTransform(Vol.Bone);

		if (DrawOptions.bDrawBounds)
		{
			RopeSDFDraw::DrawBounds(PDI, Vol.LocalBounds, Xform, FLinearColor(1.0f, 0.6f, 0.0f));
		}
		if (DrawOptions.bDrawVoxels && Vol.IsBaked())
		{
			RopeSDFDraw::DrawVoxels(PDI, Vol, Xform, DrawOptions.BandThreshold, PreviewNarrowBand);
		}
		if (DrawOptions.bDrawSlice && Vol.IsBaked())
		{
			RopeSDFDraw::DrawSlice(PDI, Vol, Xform, DrawOptions.SliceAxis, DrawOptions.SlicePosition,
				DrawOptions.SliceResolution, DrawOptions.SliceColorScale);
		}
		if (DrawOptions.bDrawGradient && Vol.IsBaked())
		{
			RopeSDFDraw::DrawGradients(PDI, Vol, Xform, DrawOptions.BandThreshold, DrawOptions.GradientLength, PreviewNarrowBand);
		}
	}
}

TSharedRef<FEditorViewportClient> SRopeSDFPreviewViewport::MakeEditorViewportClient()
{
	ViewportClient = MakeShared<FRopeSDFPreviewViewportClient>(*PreviewScene, SharedThis(this));
	return ViewportClient.ToSharedRef();
}

void SRopeSDFPreviewViewport::AddReferencedObjects(FReferenceCollector& Collector)
{
	Collector.AddReferencedObject(PreviewMeshComponent);
}

#undef LOCTEXT_NAMESPACE
