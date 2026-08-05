// Copyright 2026 TeamKeno. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "Components/MeshComponent.h"
#include "Core/RopeThrowTypes.h"
#include "RopePreviewComponent.generated.h"

class UMaterialInterface;

/** The material slot used by the preview arc alone, kept separate from the rope's own material. */
UENUM(BlueprintType)
enum class ERopePreviewMaterialSlot : uint8
{
	WrapPreview UMETA(DisplayName = "Wrap Preview")
};

/**
 * A display-only component that stores and renders the preview centreline shown before a throw.
 * Computing the path, and the actual rope simulation and wrap state, belong to RopeComponent; this
 * component only draws the centreline it is given as a tube, through SetWrapPreviewWorld and
 * ClearPreview.
 */
UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopePreviewComponent : public UMeshComponent
{
	GENERATED_BODY()

public:
	URopePreviewComponent();

	/**
	 * The only thing that decides the preview tube's colour. Left empty it uses the engine's default
	 * material, which is opaque grey. The tube leaves its vertex colour at the default of white, so
	 * there is no hardcoded colour anywhere; a translucent or coloured preview is the material's job.
	 *
	 * Replace it at runtime through SetMaterial(0, M) only. The scene proxy captures the material when
	 * it is created, and later dynamic data updates the centreline alone, so writing this property
	 * directly leaves the proxy on the old material. That is why it is not BlueprintReadWrite: a direct
	 * Blueprint set cannot be hooked. Editing it in the editor's details panel is safe, because that
	 * goes through a component re-registration.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> WrapPreviewMaterial = nullptr;

	// The arc search tuning, namely the reach scale, segment count, sample step and query radius,
	// lives on URopeComponent: this component is display only, and those values are gameplay inputs
	// that the wielder path and a direct Blueprint call to Throw() have to share.

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Shape", meta = (ClampMin = "0.1", Units = "cm"))
	float WrapPreviewRadius = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Shape", meta = (ClampMin = "3", ClampMax = "32"))
	int32 WrapPreviewSides = 8;

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void SetWrapPreviewWorld(const FRopeWrapPreviewData& InPreview);

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void ClearPreview();

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	bool IsPreviewVisible() const { return bPreviewVisible; }

	bool TryClaimPreviewOwner(UObject* InOwner);
	void ReleasePreviewOwner(UObject* InOwner);
	bool IsPreviewOwner(const UObject* InOwner) const;

	//~ UPrimitiveComponent
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual void SendRenderDynamicData_Concurrent() override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

	//~ UMeshComponent
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;

private:
	// Applies an already-local preview to the render state and the bounds.
	void SetWrapPreviewLocal(const FRopeWrapPreviewData& InPreview);
	// Converts an external world-space preview into this component's local space.
	FRopeWrapPreviewData ConvertWrapPreviewToLocal(const FRopeWrapPreviewData& InPreview) const;
	void RebuildLocalBounds();

	FRopeWrapPreviewData WrapPreviewLocal;
	bool bPreviewVisible = false;
	FBoxSphereBounds LocalPreviewBounds;

	UPROPERTY(Transient)
	TWeakObjectPtr<UObject> PreviewOwner;
};
