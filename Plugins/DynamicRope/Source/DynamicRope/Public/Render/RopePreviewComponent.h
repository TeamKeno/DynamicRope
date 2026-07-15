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

	// NOTE: 아크 탐색 튜닝(Reach Scale/Segment Count/Sample Step/Query Radius)은 URopeComponent로 이사했다 —
	// 이 컴포넌트는 표시 전용이고, 그 값들은 Wielder 경로와 BP 직행 Throw()가 공유해야 하는 게임플레이 입력이다.

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

	/**
	 * WhipGuideAnimation 표시: 실제 whip guide 생성식으로 애니메이션 프레임을 만들어 재생한다. **표시 전용** —
	 * 게임플레이 데이터(prepared contact/anchor)는 만들지 않는다. 실패해도 던지기에는 영향이 없다.
	 * WrappedPath 표시는 SetWrapPreviewWorld로 centerline을 그대로 넘기면 된다(계산은 호출자 몫).
	 */
	bool ShowWhipGuideAnimation(const URopeComponent& Rope, const FRopeThrowContext& ThrowContext,
		FString* OutFailureReason = nullptr);

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void SetWrapPreviewWorld(const FRopeWrapPreviewData& InPreview);

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void ClearPreview();

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	bool IsPreviewVisible() const { return bPreviewVisible; }

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

	FRopeWrapPreviewData WrapPreviewLocal;
	// WhipGuideAnimation 재생용 component-local 프레임. 충돌 프레임 이후는 생성 단계에서 제외한다.
	TArray<FRopeWrapPreviewData> WhipPreviewFramesLocal;
	int32 WhipPreviewFrameIndex = 0;
	float WhipPreviewPlaybackTimer = 0.0f;
	bool bPreviewVisible = false;
	FBoxSphereBounds LocalPreviewBounds;
};
