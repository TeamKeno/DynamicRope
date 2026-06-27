// Copyright Epic Games, Inc. All Rights Reserved.
//
// SDF 오써링 패널에 박히는 3D 프리뷰 뷰포트. FAdvancedPreviewScene(조명/바닥) 위에 베이크 대상
// 스켈레탈 메시(URopeSDFData::SourceMesh)를 ref 포즈로 띄워 orbit 카메라로 검수한다.
// 3a 단계에서는 메시 표시까지만 — SDF 오버레이 드로잉(RopeSDFDraw 호출)은 3b에서 클라이언트의
// Draw()에 붙인다. 레벨 비주얼라이저(FRopeSDFVisualizer)와 동일한 드로잉 헬퍼를 공유할 예정.

#pragma once

#include "CoreMinimal.h"
#include "SEditorViewport.h"
#include "UObject/GCObject.h"

class FAdvancedPreviewScene;
class FRopeSDFPreviewViewportClient;
class UDebugSkelMeshComponent;
class USkeletalMesh;

/**
 * 오써링 패널 우측에 배치되는 프리뷰 뷰포트 위젯. 프리뷰 씬과 메시 컴포넌트의 소유자이며,
 * 메시 컴포넌트를 GC로부터 보호한다(FGCObject).
 */
class SRopeSDFPreviewViewport : public SEditorViewport, public FGCObject
{
public:
	SLATE_BEGIN_ARGS(SRopeSDFPreviewViewport) {}
	SLATE_END_ARGS()

	SRopeSDFPreviewViewport();
	virtual ~SRopeSDFPreviewViewport() override;

	void Construct(const FArguments& InArgs);

	/** 프리뷰할 메시를 교체한다. nullptr이면 빈 씬(메시 제거)으로 둔다. */
	void SetPreviewMesh(USkeletalMesh* InMesh);

	// FGCObject
	virtual void AddReferencedObjects(FReferenceCollector& Collector) override;
	virtual FString GetReferencerName() const override { return TEXT("SRopeSDFPreviewViewport"); }

protected:
	// SEditorViewport
	virtual TSharedRef<FEditorViewportClient> MakeEditorViewportClient() override;

private:
	/** 조명/바닥/환경을 제공하는 프리뷰 월드. */
	TSharedPtr<FAdvancedPreviewScene> PreviewScene;

	/** 프리뷰 씬에 추가된 메시 컴포넌트(ref 포즈). 본 트랜스폼은 3b의 SDF 오버레이가 사용. */
	TObjectPtr<UDebugSkelMeshComponent> PreviewMeshComponent;

	/** 카메라/렌더링을 담당하는 뷰포트 클라이언트. */
	TSharedPtr<FRopeSDFPreviewViewportClient> ViewportClient;
};
