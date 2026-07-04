// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "Core/RopeTypes.h"
#include "RopeArcPreviewComponent.generated.h"

class UMaterialInterface;

/** 미리보기 호 전용 material slot. Rope 본체 material과 분리한다. */
UENUM(BlueprintType)
enum class ERopeArcPreviewMaterialSlot : uint8
{
	ArcFill UMETA(DisplayName = "Arc Fill"),
	ArcRim UMETA(DisplayName = "Arc Rim"),
	BlockedArcFill UMETA(DisplayName = "Blocked Arc Fill"),
	BlockedArcRim UMETA(DisplayName = "Blocked Arc Rim"),
	HitPoint UMETA(DisplayName = "Hit Point")
};

/**
 * 던지기 전 호 미리보기만 그리는 별도 primitive.
 * RopeSceneProxy의 static/dynamic relevance 계약을 건드리지 않기 위해 로프 본체 렌더링과 분리한다.
 */
UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeArcPreviewComponent : public UMeshComponent
{
	GENERATED_BODY()

public:
	URopeArcPreviewComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> ArcFillMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> ArcRimMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> BlockedArcFillMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> BlockedArcRimMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> HitPointMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Shape", meta = (ClampMin = "0.1", Units = "cm"))
	float RimThickness = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Shape", meta = (ClampMin = "0.1", Units = "cm"))
	float HitPointRadius = 6.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Shape", meta = (ClampMin = "0.0", Units = "cm"))
	float RimPlaneOffset = 0.25f;

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void SetArcPreviewWorld(const FRopeArcPreviewData& InPreview);

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void ClearArcPreview();

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	bool IsArcPreviewVisible() const { return bPreviewVisible; }

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
	bool bPreviewVisible = false;
	FBoxSphereBounds LocalPreviewBounds;
};
