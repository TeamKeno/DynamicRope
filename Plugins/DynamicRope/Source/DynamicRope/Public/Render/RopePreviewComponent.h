// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "Core/RopeTypes.h"
#include "RopePreviewComponent.generated.h"

class UMaterialInterface;

/** 미리보기 호 전용 material slot. Rope 본체 material과 분리한다. */
UENUM(BlueprintType)
enum class ERopePreviewMaterialSlot : uint8
{
	WrapPreview UMETA(DisplayName = "Wrap Preview")
};

/**
 * 던지기 전 호 미리보기만 그리는 별도 primitive.
 * RopeSceneProxy의 static/dynamic relevance 계약을 건드리지 않기 위해 로프 본체 렌더링과 분리한다.
 */
UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopePreviewComponent : public UMeshComponent
{
	GENERATED_BODY()

public:
	URopePreviewComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> WrapPreviewMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Arc Search", meta = (ClampMin = "0.0", DisplayName = "Arc Reach Scale"))
	float PreviewReachScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Arc Search", meta = (ClampMin = "1", ClampMax = "128", DisplayName = "Arc Segment Count"))
	int32 PreviewSegmentCount = 32;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Arc Search", meta = (ClampMin = "1.0", Units = "cm", DisplayName = "Arc Sample Step"))
	float PreviewSampleStep = 80.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Arc Search", meta = (ClampMin = "0.0", Units = "cm", DisplayName = "Arc Query Radius"))
	float PreviewQueryRadius = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Shape", meta = (ClampMin = "0.1", Units = "cm"))
	float WrapPreviewRadius = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Shape", meta = (ClampMin = "3", ClampMax = "32"))
	int32 WrapPreviewSides = 8;

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void SetArcPreviewWorld(const FRopeArcPreviewData& InPreview);

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void SetWrapPreviewWorld(const FRopeWrapPreviewData& InPreview);

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void ClearPreview();

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview", meta = (DeprecatedFunction, DeprecationMessage = "Use ClearPreview."))
	void ClearArcPreview() { ClearPreview(); }

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	bool IsPreviewVisible() const { return bPreviewVisible; }

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview", meta = (DeprecatedFunction, DeprecationMessage = "Use IsPreviewVisible."))
	bool IsArcPreviewVisible() const { return IsPreviewVisible(); }

	//~ UPrimitiveComponent
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual void SendRenderDynamicData_Concurrent() override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

	//~ UMeshComponent
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;

private:
	void RebuildLocalBounds();

	FRopeArcPreviewData PreviewLocal;
	FRopeWrapPreviewData WrapPreviewLocal;
	bool bPreviewVisible = false;
	FBoxSphereBounds LocalPreviewBounds;
};

UCLASS(ClassGroup = (DynamicRope), meta = (DeprecatedNode, DeprecationMessage = "Use RopePreviewComponent."))
class DYNAMICROPE_API URopeArcPreviewComponent : public URopePreviewComponent
{
	GENERATED_BODY()

public:
	URopeArcPreviewComponent();
};
