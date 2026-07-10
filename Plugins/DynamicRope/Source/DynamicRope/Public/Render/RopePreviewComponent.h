// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "Core/RopeTypes.h"
#include "RopeComponent.h"
#include "RopePreviewComponent.generated.h"

class UMaterialInterface;

UENUM(BlueprintType)
enum class ERopePreviewMode : uint8
{
	/** 최종 접촉과 감김 경로를 한 번에 표시한다. */
	WrappedPath UMETA(DisplayName = "Wrapped Path"),
	/** 실제 Flight와 같은 whip guide를 시간 순서대로 재생한다. */
	WhipGuideAnimation UMETA(DisplayName = "Whip Guide Animation")
};

/** 미리보기 호 전용 material slot. Rope 본체 material과 분리한다. */
UENUM(BlueprintType)
enum class ERopePreviewMaterialSlot : uint8
{
	WrapPreview UMETA(DisplayName = "Wrap Preview")
};

/**
 * 던지기 전 preview의 생성, prepared 결과 보관, 애니메이션 재생과 렌더링을 담당한다.
 * 실제 로프 시뮬레이션/감김 상태는 RopeComponent가 소유하고, 이 컴포넌트는 preview 수명만 관리한다.
 */
UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopePreviewComponent : public UMeshComponent
{
	GENERATED_BODY()

public:
	URopePreviewComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Material")
	TObjectPtr<UMaterialInterface> WrapPreviewMaterial = nullptr;

	/** 정적인 감김 결과와 시간에 따라 움직이는 whip guide 표시 중 하나를 선택한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	ERopePreviewMode PreviewMode = ERopePreviewMode::WrappedPath;

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

	/** whip preview 프레임 사이의 sweep 각도 간격이다. 작을수록 프레임 수가 늘어난다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Whip Animation", meta = (ClampMin = "1.0", ClampMax = "45.0", Units = "deg"))
	float WhipPreviewAngleStepDegrees = 10.0f;

	/** 생성된 whip guide 프레임 하나를 화면에 유지하는 시간이다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview|Whip Animation", meta = (ClampMin = "0.01", Units = "s"))
	float WhipPreviewSecondsPerStep = 0.05f;

	/** 선택한 PreviewMode로 표시 데이터를 만들고, 요청 시 실제 throw용 prepared 결과도 함께 보관한다. */
	bool UpdatePreviewFromRope(const URopeComponent& Rope, const FRopeThrowContext& ThrowContext,
		bool bBuildPreparedPreview, FString* OutFailureReason = nullptr);
	bool HasPreparedPreview() const { return LastPreparedPreview.IsValid(); }
	const FRopePreparedThrowPreview& GetPreparedPreview() const { return LastPreparedPreview; }
	void ResetPreparedPreview() { LastPreparedPreview.Reset(); }

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
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual void SendRenderDynamicData_Concurrent() override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

	//~ UMeshComponent
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;

private:
	// 최종 contact/anchor까지 계산한 정적 감김 경로를 만든다.
	bool UpdateWrappedPathPreviewFromRope(const URopeComponent& Rope, const FRopeThrowContext& ThrowContext,
		bool bBuildPreparedPreview, FString* OutFailureReason);
	// 실제 whip guide 생성식으로 애니메이션 프레임 배열을 만든다.
	bool UpdateWhipGuidePreviewFromRope(const URopeComponent& Rope, const FRopeThrowContext& ThrowContext,
		bool bBuildPreparedPreview, FString* OutFailureReason);
	// throw sweep를 각도 간격으로 샘플링해 component-local 프레임으로 캐시한다.
	void RebuildWhipPreviewFrames(const struct FRopePreviewBuildContext& Source);
	// 재생 타이머를 전진시키며, 생성 시 충돌 전까지만 캐시된 프레임을 반복한다.
	void AdvanceWhipPreview(float DeltaTime);
	// 프레임 생성 호출 안에서만 전달된 collider/SDF에 질의한다. collider 포인터를 멤버에 보관하지 않는다.
	bool DoesWhipFrameHit(const FRopeWrapPreviewData& Frame, const struct FRopePreviewBuildContext& Source) const;
	// 이미 local인 preview를 렌더 상태와 bounds에 반영한다.
	void SetWrapPreviewLocal(const FRopeWrapPreviewData& InPreview);
	// 외부 월드 preview를 이 컴포넌트 기준 local 좌표로 변환한다.
	FRopeWrapPreviewData ConvertWrapPreviewToLocal(const FRopeWrapPreviewData& InPreview) const;
	void RebuildLocalBounds();

	FRopeArcPreviewData PreviewLocal;
	FRopeWrapPreviewData WrapPreviewLocal;
	// WhipGuideAnimation 재생용 component-local 프레임. 충돌 프레임 이후는 생성 단계에서 제외한다.
	TArray<FRopeWrapPreviewData> WhipPreviewFramesLocal;
	FRopePreparedThrowPreview LastPreparedPreview;
	int32 WhipPreviewFrameIndex = 0;
	float WhipPreviewPlaybackTimer = 0.0f;
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
