// Copyright Epic Games, Inc. All Rights Reserved.
//
// Pull 장전/발동 상태를 보여주는 게이지 위젯. 이 UI가 답해야 하는 질문은 "토글이 켜졌나"가 아니라
// **"눌렀는데 왜 아직 안 당겨지나"**다. 그래서 On/Off 램프가 아니라 발동 임계까지의 진행도를 링으로
// 그린다(URopeWielderComponent::GetPullEngageProgress) — 링이 안 차 있으면 "줄이 아직 안 팽팽하다"가
// 그림만으로 읽힌다.
//
// 상태는 셋이고 각각 다르게 보인다:
//   ① 해제        — 아무것도 안 그린다.
//   ② 장전(대기)  — 배경 링 + 진행도만큼 채워진 호. 진행도가 오르면 색이 대기색→발동색으로 섞인다.
//   ③ 발동        — 꽉 찬 링 + 발동 순간 한 번의 팝(EngagePopTime 동안 확대/페이드).
//
// RopeAimWidget과 같은 "C++ 베이스 + WBP 리스타일" 구성이다:
//  - 에셋 없이 NativePaint가 직접 그리므로 이 클래스만 화면에 올려도 즉시 동작한다.
//  - WBP 서브클래스로 리스타일하려면 스타일 프로퍼티를 덮어쓰거나, bDrawBuiltInVisuals를 꺼서 내장
//    페인트를 끄고 IsPullArmed()/IsPullEngaged()/GetProgress()로 자체 비주얼을 구성한다.
//    상태 전환 연출(사운드/애니메이션)은 OnPullArmedStateChanged/OnPullEngagedStateChanged 이벤트로 붙인다.
//
// wielder는 소유 폰에서 스스로 찾는다(RopeAimWidget과 동일 규약) — HUD에 얹기만 하면 배선이 끝난다.
// 명시 배선이 필요하면 SetWielder()로 지정한다.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "RopePullGaugeWidget.generated.h"

class URopeWielderComponent;

UCLASS(Blueprintable)
class DYNAMICROPE_API URopePullGaugeWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	URopePullGaugeWidget(const FObjectInitializer& ObjectInitializer);

	//~ 데이터 --------------------------------------------------------------

	/** 이 위젯이 읽는 wielder를 명시 지정(비우면 소유 폰에서 자동 탐색). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Pull HUD")
	void SetWielder(URopeWielderComponent* InWielder);

	/** Pull 장전 상태(② 이상). */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull HUD")
	bool IsPullArmed() const;

	/** Pull 발동 상태(③). */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull HUD")
	bool IsPullEngaged() const;

	/** 발동 임계까지의 진행도 0..1. */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull HUD")
	float GetProgress() const;

	//~ 이벤트(WBP에서 사운드/애니메이션을 붙이는 지점) -----------------------

	/** 장전 토글이 바뀐 순간. */
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Pull HUD")
	void OnPullArmedStateChanged(bool bArmed);

	/** 발동 래치가 바뀐 순간(false = wrap 해제로 재무장). */
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Pull HUD")
	void OnPullEngagedStateChanged(bool bEngaged, float Tension);

	//~ 스타일 --------------------------------------------------------------

	/** 내장 페인트를 그릴지. WBP가 자체 비주얼을 쓰면 끈다(게터/이벤트는 계속 동작). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	bool bDrawBuiltInVisuals = true;

	/** 게이지 중심(위젯 로컬 좌표 비율 0..1). 기본은 화면 중앙 살짝 아래 — 십자선과 겹치지 않게. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	FVector2D CenterAnchor = FVector2D(0.5f, 0.62f);

	/** 링 반경(px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "4.0"))
	float Radius = 26.0f;

	/** 링 선 두께(px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "0.5"))
	float Thickness = 3.0f;

	/** 링 세그먼트 수(원 근사 정밀도). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "8", ClampMax = "128"))
	int32 Segments = 48;

	/** 채워지지 않은 배경 링 색(장전 중에만 보인다). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	FLinearColor TrackColor = FLinearColor(1.0f, 1.0f, 1.0f, 0.18f);

	/** 진행도 0에서의 색(장전했지만 아직 느슨함). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	FLinearColor ArmedColor = FLinearColor(1.0f, 0.72f, 0.15f, 0.95f);

	/** 진행도 1/발동에서의 색. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	FLinearColor EngagedColor = FLinearColor(0.2f, 1.0f, 0.45f, 1.0f);

	/** 발동 순간 팝 연출 시간(초). 0이면 팝 없음. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "0.0", Units = "s"))
	float EngagePopTime = 0.25f;

	/** 팝이 최대일 때 반경 배율. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "1.0"))
	float EngagePopScale = 1.35f;

	/** 게이지 표시 진행도의 보간 속도(1/s). 장력이 떨리는 프레임에 링이 파르르 떠는 걸 막는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "0.0"))
	float ProgressInterpSpeed = 12.0f;

protected:
	//~ UUserWidget
	virtual void NativeConstruct() override;
	virtual void NativeDestruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;
	virtual int32 NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;

private:
	/** 무효일 때만 소유 폰에서 wielder를 찾는다(평시 무비용). */
	void ResolveWielder();

	/** wielder 이벤트 구독/해제 — 폰 교체로 wielder가 바뀌면 다시 건다. */
	void BindWielder(URopeWielderComponent* NewWielder);

	UFUNCTION()
	void HandlePullArmedChanged(bool bArmed);

	UFUNCTION()
	void HandlePullEngagedChanged(bool bEngaged, float Tension);

	/** 링(또는 그 일부 호)을 선분으로 근사해 그린다. Alpha01 = 그릴 비율(1이면 완전한 원). */
	void DrawArc(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
		const FVector2D& Center, float InRadius, float Alpha01, const FLinearColor& Color) const;

	TWeakObjectPtr<URopeWielderComponent> Wielder;

	/** 표시용 보간 진행도(실제 값은 wielder가 소유 — 이건 순수 연출 상태). */
	float DisplayProgress = 0.0f;

	/** 발동 팝 경과(EngagePopTime을 넘으면 팝 종료). */
	float TimeSinceEngage = BIG_NUMBER;
};
