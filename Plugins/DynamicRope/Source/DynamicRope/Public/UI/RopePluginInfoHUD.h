// Copyright Epic Games, Inc. All Rights Reserved.
//
// 플러그인 설명 HUD. URopePluginInfoWidget(WBP)을 생성해 뷰포트에 올리고, 입력 키로 각 패널
// (키 안내 / 필요 컴포넌트 / 지원·한계)을 개별 토글한다. GameMode의 HUDClass에 이 클래스(또는 BP 자식)를
// 지정하면 끝 — 별도 셋업 없이 Play만 하면 뜬다.
//
// 입력: AActor::EnableInput + InputComponent->BindKey로 토글 키를 직접 바인딩한다(Input Action 에셋 불필요).
//       기본 키는 F1~F4라 게임플레이 IMC와 잘 겹치지 않는다. 필요하면 EditDefaultsOnly로 바꾼다.
//
// 위젯 폴백: InfoWidgetClass를 비워 두면(WBP 미제작) DrawHUD가 Canvas로 같은 콘텐츠를 텍스트로 그린다.
//            아트 없이도 즉시 확인 가능. WBP를 지정하면 Canvas 폴백은 자동으로 꺼진다.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "InputCoreTypes.h"
#include "UI/RopePluginInfoWidget.h"
#include "RopePluginInfoHUD.generated.h"

class URopePluginInfoWidget;

UCLASS()
class DYNAMICROPE_API ARopePluginInfoHUD : public AHUD
{
	GENERATED_BODY()

public:
	ARopePluginInfoHUD();

	//~ AActor / AHUD
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void DrawHUD() override;

	/** 뷰포트에 올릴 위젯 클래스(보통 이 부모로 만든 WBP). 비우면 Canvas 텍스트 폴백을 쓴다. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info")
	TSubclassOf<URopePluginInfoWidget> InfoWidgetClass;

	/** InfoWidgetClass가 비었을 때 DrawHUD가 Canvas로 콘텐츠를 그릴지. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info")
	bool bDrawCanvasFallbackWhenNoWidget = true;

	//~ 토글 키(개별) — 게임플레이와 잘 안 겹치는 F1~F4가 기본 -----------------------------
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey KeyGuideToggleKey;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey ComponentsToggleKey;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey CapabilitiesToggleKey;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey LimitationsToggleKey;

	/** 전체 HUD를 한 번에 표시/숨김. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey MasterToggleKey;

	//~ API ----------------------------------------------------------------
	/** 지정 패널을 토글한다(위젯이 있으면 위젯에, 없으면 Canvas 폴백 상태에 반영). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void TogglePanel(ERopeInfoPanel Panel);

	/** 열린 패널을 모두 숨긴다 / 다시 누르면 마지막 패널을 복원한다. 상시 힌트 줄은 영향받지 않는다. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void ToggleAll();

	/** 현재 토글 키 라벨로 상시 힌트 줄 텍스트를 만든다(위젯/ Canvas 폴백 공용). */
	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText BuildHintText() const;

	/** 생성된 위젯(없으면 null — Canvas 폴백 사용 중). */
	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	URopePluginInfoWidget* GetInfoWidget() const { return InfoWidget; }

protected:
	/** EnableInput 후 토글 키들을 InputComponent에 바인딩한다. */
	void SetupInputBindings();

	// 키 핸들러(BindKey 대상).
	void OnKeyGuideKey()     { TogglePanel(ERopeInfoPanel::KeyGuide); }
	void OnComponentsKey()   { TogglePanel(ERopeInfoPanel::Components); }
	void OnCapabilitiesKey() { TogglePanel(ERopeInfoPanel::Capabilities); }
	void OnLimitationsKey()  { TogglePanel(ERopeInfoPanel::Limitations); }

	// Canvas 폴백: 제목 + 본문 블록을 그리고 다음 블록의 Y를 돌려준다.
	float DrawFallbackBlock(const FString& Title, const FText& Body, float X, float Y);

	/** 생성된 위젯 인스턴스. Canvas 폴백일 땐 null. */
	UPROPERTY(Transient)
	TObjectPtr<URopePluginInfoWidget> InfoWidget = nullptr;

	// Canvas 폴백에서만 쓰는 패널 표시 상태(위젯이 있으면 위젯이 진실을 소유한다). 힌트 줄은 상시.
	bool bFallbackPanelVisible[4] = { true, false, false, false };

	// ToggleAll이 "숨김 → 복원"할 때 되살릴 마지막으로 연 패널.
	ERopeInfoPanel LastShownPanel = ERopeInfoPanel::KeyGuide;
};
