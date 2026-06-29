// Copyright Epic Games, Inc. All Rights Reserved.
//
// SDF 오써링 패널에 박히는 3D 프리뷰 뷰포트. FAdvancedPreviewScene(조명/바닥) 위에 베이크 대상
// 스켈레탈 메시(URopeSDFData::SourceMesh)를 ref 포즈로 띄워 orbit 카메라로 검수한다.
// SDF 오버레이(bounds/voxels/slice/gradient)는 뷰포트 클라이언트의 Draw(View, PDI)에서 본별
// 볼륨을 RopeSDFDraw 헬퍼로 그린다 — 레벨 비주얼라이저(FRopeSDFVisualizer)와 동일 코드 공유.
// 토글/파라미터는 자산이 아니라 이 위젯의 FRopeSDFPreviewDrawOptions(패널 로컬 상태)에 둔다.

#pragma once

#include "CoreMinimal.h"
#include "SEditorViewport.h"
#include "UObject/GCObject.h"
#include "Collision/SDF/RopeSDFProvider.h" // ERopeSDFSliceAxis (오버레이 옵션 기본값)
#include "Collision/SDF/RopeSDFData.h"     // FRopeBoneSDFVolume (오버레이 스냅샷 보관)

class FAdvancedPreviewScene;
class FRopeSDFPreviewViewportClient;
class FPrimitiveDrawInterface;
class UDebugSkelMeshComponent;
class URopeSDFData;
class USkeletalMesh;

/**
 * 프리뷰 SDF 오버레이의 표시 토글/파라미터. 런타임 URopeSDFProvider의 WITH_EDITORONLY_DATA 토글과는
 * 별개의 "패널 로컬 상태"이며, 자산을 더럽히지 않는다(provider 의존 X). 기본값은 레벨 비주얼라이저와 일치.
 */
struct FRopeSDFPreviewDrawOptions
{
	bool bDrawBounds = true;
	bool bDrawVoxels = false;
	bool bDrawSlice = false;
	bool bDrawGradient = false;

	/** voxel/gradient 표시 밴드 두께(cm). |distance| <= 이 값만 그린다. */
	float BandThreshold = 3.0f;

	/** slice 평면이 통과하는 축. */
	ERopeSDFSliceAxis SliceAxis = ERopeSDFSliceAxis::Z;
	/** 축을 따른 slice 위치(0~1). */
	float SlicePosition = 0.5f;
	/** slice 샘플 격자 한 변 개수. */
	int32 SliceResolution = 24;
	/** slice 색 매핑 스케일(cm): |distance| = 이 값에서 포화. */
	float SliceColorScale = 10.0f;

	/** gradient 화살표 길이(cm). */
	float GradientLength = 4.0f;

	bool AnyEnabled() const { return bDrawBounds || bDrawVoxels || bDrawSlice || bDrawGradient; }
};

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

	/**
	 * SDF 오버레이가 그릴 본별 볼륨을, 이 자산의 현재 BoneVolumes로 스냅샷한다(사본 보관). 호출 시점의
	 * 데이터로 고정되므로, 이후 자산이 베이크돼 바뀌어도 SetPreviewData를 다시 부를 때까지 갱신되지 않는다
	 * (Bake는 뷰포트에 즉시 반영 X, Refresh 시 반영). nullptr이면 오버레이를 비운다.
	 */
	void SetPreviewData(URopeSDFData* InData);

	/** 패널 UI가 토글/파라미터를 읽고 쓰는 진입점. 변경 후 InvalidatePreview() 호출 권장. */
	FRopeSDFPreviewDrawOptions& AccessDrawOptions() { return DrawOptions; }

	/** 다음 프레임 리드로우를 강제한다(토글/파라미터 변경 반영). */
	void InvalidatePreview();

	/** 뷰포트 클라이언트의 Draw(View, PDI)에서 호출 — 본별 SDF 오버레이를 PDI로 그린다. */
	void DrawSDFOverlay(FPrimitiveDrawInterface* PDI);

	// FGCObject
	virtual void AddReferencedObjects(FReferenceCollector& Collector) override;
	virtual FString GetReferencerName() const override { return TEXT("SRopeSDFPreviewViewport"); }

protected:
	// SEditorViewport
	virtual TSharedRef<FEditorViewportClient> MakeEditorViewportClient() override;

private:
	/** 조명/바닥/환경을 제공하는 프리뷰 월드. */
	TSharedPtr<FAdvancedPreviewScene> PreviewScene;

	/** 프리뷰 씬에 추가된 메시 컴포넌트(ref 포즈). SDF 오버레이가 본 트랜스폼을 가져온다. */
	TObjectPtr<UDebugSkelMeshComponent> PreviewMeshComponent;

	/** 카메라/렌더링을 담당하는 뷰포트 클라이언트. */
	TSharedPtr<FRopeSDFPreviewViewportClient> ViewportClient;

	/** 오버레이가 그릴 본별 볼륨의 스냅샷 사본. SetPreviewData 호출 때만 갱신된다(라이브 자산과 디커플링). */
	TArray<FRopeBoneSDFVolume> PreviewVolumes;

	/** 오버레이 표시 토글/파라미터(패널 로컬 상태). */
	FRopeSDFPreviewDrawOptions DrawOptions;
};
