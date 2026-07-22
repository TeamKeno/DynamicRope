// Copyright Epic Games, Inc. All Rights Reserved.
//
// Aim-ray 조준의 데모 조준 HUD 위젯. 표시 조건은 URopeWielderComponent::IsAimActive() —
// 조준 모드(UsesAimRay: ResolveMode가 ①FullSimulation이 아님)이면서 **지금 던질 수 있는 phase**일 때만이다
// (③GuaranteedWrap은 Loaded/장전 전용 → Free 등에서는 십자선까지 통째로 숨는다. ①②는 phase 게이트 없음).
// 평상시에는 화면 중앙 십자선을, aim ray가 감을 수 있는 본에 걸리는 동안에는 그 본 주위에
// 스크린 투영된 강조 링(획득 팝 + 펄스)을 그린다.
//
// RopePluginInfoWidget과 같은 "C++ 베이스 + WBP 리스타일" 구성이다:
//  - 에셋 없이 C++ NativePaint가 십자선/링을 직접 그리므로 이 클래스만으로 즉시 동작한다.
//  - WBP 서브클래스로 리스타일하려면: 스타일 프로퍼티(색/두께/펄스)를 디폴트에서 덮어쓰거나,
//    bDrawBuiltInVisuals를 꺼서 내장 페인트를 끄고 GetAimSample()/GetTargetScreenPosition()으로
//    자체 비주얼(이미지/애니메이션)을 배치한다. OnAimTargetChanged/OnAimTargetLost 이벤트로
//    사운드/추가 연출을 붙인다.
//  - 어떤 위젯 클래스를 쓸지는 Project Settings > Dynamic Rope > AimHudWidgetClass가 정하고,
//    생성/수명은 URopeWielderComponent가 관리한다(로컬 플레이어 전용, bShowAimHudWidget).
//
// 데이터 소스는 wielder가 틱마다 캐시하는 FRopeAimHudSample 하나다 — 위젯은 조준 로직을 다시
// 돌리지 않고 읽기만 한다(스크린 투영/펄스 타이밍만 위젯 소유).

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Gameplay/RopeWielderComponent.h"
#include "RopeAimWidget.generated.h"

class USceneComponent;

UCLASS(Blueprintable)
class DYNAMICROPE_API URopeAimWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	URopeAimWidget(const FObjectInitializer& ObjectInitializer);

	//~ 스타일(WBP 디폴트에서 덮어쓰기 가능) ---------------------------------

	/** 내장 페인트(십자선+링)를 그릴지. WBP가 자체 비주얼을 쓰면 끈다(데이터 게터/이벤트는 계속 동작). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD")
	bool bDrawBuiltInVisuals = true;

	/** 십자선 색. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair")
	FLinearColor CrosshairColor = FLinearColor(1.0f, 1.0f, 1.0f, 0.9f);

	/** 십자선 한 팔 길이(px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair", meta = (ClampMin = "1.0"))
	float CrosshairArmLength = 8.0f;

	/** 십자선 중앙 공백 반경(px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair", meta = (ClampMin = "0.0"))
	float CrosshairGap = 5.0f;

	/** 십자선 선 두께(px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair", meta = (ClampMin = "0.5"))
	float CrosshairThickness = 2.0f;

	/** 대상이 잡혀 있는 동안 십자선에 섞을 색(획득 피드백 — 링과 같은 톤 권장). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair")
	FLinearColor CrosshairTargetColor = FLinearColor(0.2f, 1.0f, 0.4f, 1.0f);

	/** ray는 걸렸지만 wrap 불가일 때(월드 정적/게이트 거부/본 없음) 십자선·링에 쓸 색. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD")
	FLinearColor BlockedColor = FLinearColor(1.0f, 0.2f, 0.15f, 0.9f);

	/** 강조 링 색. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring")
	FLinearColor RingColor = FLinearColor(0.2f, 1.0f, 0.4f, 0.9f);

	/** 링 선 두께(px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.5"))
	float RingThickness = 2.0f;

	/** 링 세그먼트 수(원 근사 정밀도). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "8", ClampMax = "64"))
	int32 RingSegments = 32;

	/** 대상 월드 반경 → 링 반경 배율(1보다 살짝 크게 잡아 본을 여유 있게 감싼다). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.5"))
	float RingRadiusScale = 1.15f;

	/** 링 최소 화면 반경(px) — 먼 대상에서 링이 점으로 뭉개지는 것 방지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "1.0"))
	float RingMinScreenRadius = 18.0f;

	/** 획득 순간 팝 지속(초): 링이 이 시간 동안 크게 시작해 제 크기로 수축한다. 0 = 팝 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.0", Units = "s"))
	float AcquirePopDuration = 0.15f;

	/** 유지 중 펄스 주기(초). 0 = 펄스 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.0", Units = "s"))
	float PulsePeriod = 1.2f;

	/** 펄스 반경 진폭(비율 — 0.06이면 ±6%). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.0", ClampMax = "0.5"))
	float PulseAmplitude = 0.06f;

	//~ WBP/게임 소비용 데이터 ------------------------------------------------

	/** wielder가 틱마다 캐시한 조준 샘플(대상 유무/본/월드 위치/반경). wielder가 없으면 빈 샘플. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	FRopeAimHudSample GetAimSample() const;

	/** 이 위젯이 붙어 있는 로컬 폰의 wielder(NativeConstruct에서 해석). */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	URopeWielderComponent* GetWielder() const { return Wielder.Get(); }

	/**
	 * 이번 프레임 대상의 스크린(뷰포트 위젯 공간) 위치/반경. 자체 비주얼을 쓰는 WBP용.
	 * @return 대상이 있고 화면 안에 투영됐으면 true.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	bool GetTargetScreenPosition(FVector2D& OutPosition, float& OutRadius) const;

	/** AimRayOriginMode를 반영한 실제 ray의 hit(또는 끝점)를 투영한 조준원 위치. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	bool GetAimScreenPosition(FVector2D& OutPosition) const;

	//~ BP 연출 훅(사운드/추가 이펙트) — wielder 델리게이트를 위젯 이벤트로 중계 ----
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Aim HUD")
	void OnAimTargetChanged(USceneComponent* Mesh, FName Bone);

	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Aim HUD")
	void OnAimTargetLost();

protected:
	//~ UUserWidget
	virtual bool Initialize() override;
	virtual void NativeConstruct() override;
	virtual void NativeDestruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;
	virtual int32 NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
		bool bParentEnabled) const override;

	/** wielder 델리게이트 → BP 이벤트 중계 핸들러(획득 팝 타이머도 여기서 리셋). */
	UFUNCTION()
	void HandleAimTargetChanged(USceneComponent* Mesh, FName Bone);

	UFUNCTION()
	void HandleAimTargetLost();

private:
	/** 소유 폰에서 wielder를 찾는다(폰 교체/지연 빙의 대비 — 틱에서 무효 시 재시도). */
	void ResolveWielder();

	/** 조준 HUD가 그려져야 하는 상태인가(wielder 유효 + IsAimActive — 조준 모드이면서 던질 수 있는 phase). */
	bool IsAimHudActive() const;

	TWeakObjectPtr<URopeWielderComponent> Wielder;

	//~ NativeTick이 캐시하고 NativePaint(const)가 읽는 프레임 상태 -------------
	bool bHasScreenAim = false;
	FVector2D AimScreenPos = FVector2D::ZeroVector;
	bool bHasScreenTarget = false;
	// 이번 프레임 화면 대상이 wrap 불가(빨강)인가. bHasScreenTarget일 때만 의미.
	bool bScreenTargetBlocked = false;
	FVector2D TargetScreenPos = FVector2D::ZeroVector;
	float TargetScreenRadius = 0.0f;
	float TimeSinceAcquire = 0.0f;
	float PulseTime = 0.0f;
};
