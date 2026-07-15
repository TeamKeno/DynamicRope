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
 * 던지기 전 preview centerline의 보관과 렌더링을 담당하는 **표시 전용** 컴포넌트다.
 * 경로 계산과 실제 로프 시뮬레이션/감김 상태는 RopeComponent가 소유하고, 이 컴포넌트는 주어진
 * centerline을 튜브로 그리는 일만 한다(SetWrapPreviewWorld / ClearPreview).
 */
UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopePreviewComponent : public UMeshComponent
{
	GENERATED_BODY()

public:
	URopePreviewComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> WrapPreviewMaterial = nullptr;

	// NOTE: 아크 탐색 튜닝(Reach Scale/Segment Count/Sample Step/Query Radius)은 URopeComponent로 이사했다 —
	// 이 컴포넌트는 표시 전용이고, 그 값들은 Wielder 경로와 BP 직행 Throw()가 공유해야 하는 게임플레이 입력이다.

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

	//~ UPrimitiveComponent
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual void SendRenderDynamicData_Concurrent() override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

	//~ UMeshComponent
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;

private:
	// 이미 local인 preview를 렌더 상태와 bounds에 반영한다.
	void SetWrapPreviewLocal(const FRopeWrapPreviewData& InPreview);
	// 외부 월드 preview를 이 컴포넌트 기준 local 좌표로 변환한다.
	FRopeWrapPreviewData ConvertWrapPreviewToLocal(const FRopeWrapPreviewData& InPreview) const;
	void RebuildLocalBounds();

	FRopeWrapPreviewData WrapPreviewLocal;
	bool bPreviewVisible = false;
	FBoxSphereBounds LocalPreviewBounds;
};
