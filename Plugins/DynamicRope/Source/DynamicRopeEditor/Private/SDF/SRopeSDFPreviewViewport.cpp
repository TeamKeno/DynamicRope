// Copyright Epic Games, Inc. All Rights Reserved.

#include "SDF/SRopeSDFPreviewViewport.h"

#include "AdvancedPreviewScene.h"
#include "EditorViewportClient.h"
#include "Animation/DebugSkelMeshComponent.h"
#include "Engine/SkeletalMesh.h"

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

		// 프리뷰 월드를 직접 틱해 본 트랜스폼이 갱신되게 한다(3b 오버레이가 의존).
		if (AdvancedScene && !GIntraFrameDebuggingGameThread)
		{
			AdvancedScene->GetWorld()->Tick(LEVELTICK_All, DeltaSeconds);
		}
	}

private:
	FAdvancedPreviewScene* AdvancedScene = nullptr;
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
