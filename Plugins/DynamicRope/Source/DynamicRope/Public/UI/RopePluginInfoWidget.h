// Copyright Epic Games, Inc. All Rights Reserved.
//
// 플러그인 "설명 HUD"의 프레젠테이션 위젯. C++는 콘텐츠 데이터(키 안내 / 필요 컴포넌트 / 지원·한계)와
// 패널 토글 로직만 소유하고, 실제 레이아웃은 이 클래스를 부모로 하는 WBP(에디터 제작)가 담당한다
// (요청한 "C++ 베이스 + WBP" 구성). ARopePluginInfoHUD가 이 위젯을 생성해 뷰포트에 올리고 입력 키로
// 각 패널을 토글한다.
//
// WBP에서 하는 일 — 명명된 "빈 컨테이너" 3개만 두면 끝(그래프 로직 불필요):
//  - 네 패널의 루트를 UPanelWidget(VerticalBox 권장 — 슬롯 패딩이 적용됨)으로 만들고 이름을
//    KeyGuidePanel / ComponentsPanel / CapabilitiesPanel / LimitationsPanel로 맞춘다(BindWidgetOptional).
//    LimitationsPanel이 없는 기존 WBP는 그대로 동작한다 — 한계 항목이 Capabilities 패널 뒤에 이어 붙는다.
//  - 항목 문구는 "제목 + 한 줄"로 유지한다. 이 HUD는 읽는 문서가 아니라 훑는 참조표이고, 설명이 길어지면
//    화면 밖으로 밀린다(스크롤은 마우스 휠이 이미 리엘에 묶여 있어 쓸 수 없다).
//  - C++가 NativeConstruct에서 이 컨테이너를 비우고 데이터(아래 배열)로 행 위젯(TextBlock)을 직접 채운다.
//    WBP는 배치·스타일(컨테이너 위치/크기/배경)만 담당하고, 표시/숨김 토글도 C++가 제어한다.
//  - 이름을 안 맞추면(포인터 null) 그 패널은 그냥 비어 있고 컴파일/실행은 정상.
// 콘텐츠 배열은 EditAnywhere라 WBP 디폴트에서 프로젝트에 맞게 덮어쓸 수 있다(기본값은 C++가 채운다).
// 더 커스터마이즈하려면 OnRefreshContent(자동 채움 뒤 호출)에서 추가로 손대면 된다.
// GetXxxText()는 단일 TextBlock에 바로 바인딩하고 싶을 때를 위한 대체 경로로 남겨 둔다.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "RopePluginInfoWidget.generated.h"

class UPanelWidget;
class UTextBlock;

/** 개별 토글 대상 패널. HUD 입력 키 ↔ 위젯 패널을 잇는 식별자. */
UENUM(BlueprintType)
enum class ERopeInfoPanel : uint8
{
	/** 입력 키 안내(던지기/당기기/되감기/절단 …). */
	KeyGuide,
	/** 로프를 쓰려면 어떤 컴포넌트를 붙여야 하는지. */
	Components,
	/** 무엇을 지원하는지. */
	Capabilities,
	/** 어떤 한계가 있는지. Capabilities와 한 패널에 있었으나 화면을 넘겨 분리했다 —
	 *  WBP에 LimitationsPanel 컨테이너가 없으면 예전처럼 Capabilities 패널 아래에 이어 붙는다. */
	Limitations
};

/** 키 안내 한 줄: 키 라벨 + 그 키가 하는 일. 실제 키는 프로젝트의 Input Mapping Context가 정하므로 라벨은 표시용. */
USTRUCT(BlueprintType)
struct FRopePluginKeyBinding
{
	GENERATED_BODY()

	/** 화면에 보일 키 라벨(예: "LMB", "R", "Mouse Wheel Up"). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Info")
	FText Key;

	/** 그 키가 하는 동작 설명. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Info")
	FText Action;
};

/** 제목 + 설명으로 된 정보 항목(필요 컴포넌트 / 지원 기능 / 한계 공용). */
USTRUCT(BlueprintType)
struct FRopePluginInfoEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Info")
	FText Title;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Info", meta = (MultiLine = true))
	FText Description;
};

UCLASS(Abstract, Blueprintable)
class DYNAMICROPE_API URopePluginInfoWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	URopePluginInfoWidget(const FObjectInitializer& ObjectInitializer);

	//~ 콘텐츠 데이터 — 기본값은 생성자가 채우고, WBP 디폴트에서 덮어쓸 수 있다 -------------
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Key Guide")
	TArray<FRopePluginKeyBinding> KeyBindings;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Components")
	TArray<FRopePluginInfoEntry> RequiredComponents;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Capabilities")
	TArray<FRopePluginInfoEntry> Capabilities;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Capabilities")
	TArray<FRopePluginInfoEntry> Limitations;

	//~ 패널 토글 API — HUD 입력 키가 호출(BlueprintCallable로도 노출) --------------------
	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void TogglePanel(ERopeInfoPanel Panel);

	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void SetPanelVisible(ERopeInfoPanel Panel, bool bVisible);

	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void HideAllPanels();

	/** 상시 힌트 줄 텍스트를 설정한다(HUD가 실제 토글 키 라벨로 채운다). HintText 슬롯이 있어야 표시됨. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void SetHintText(const FText& InText);

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	bool IsPanelVisible(ERopeInfoPanel Panel) const;

	//~ 간단 경로: TextBlock 하나에 바인딩할 멀티라인 텍스트 -----------------------------
	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetKeyGuideText() const;

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetComponentsText() const;

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetCapabilitiesText() const;

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetLimitationsText() const;

	//~ 풍부 경로 훅: 배열을 순회해 행 위젯을 만들고 싶을 때 WBP에서 구현 ------------------
	/** NativeConstruct 후 1회 호출. 배열을 읽어 행 위젯을 채우는 곳(구현은 선택). */
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Info")
	void OnRefreshContent();

	//~ 기본 콘텐츠 빌더 — 위젯 생성자와 HUD Canvas 폴백이 공유(static) --------------------
	static TArray<FRopePluginKeyBinding> GetDefaultKeyBindings();
	static TArray<FRopePluginInfoEntry> GetDefaultRequiredComponents();
	static TArray<FRopePluginInfoEntry> GetDefaultCapabilities();
	static TArray<FRopePluginInfoEntry> GetDefaultLimitations();

	/** 배열을 멀티라인 FText로 포매팅(위젯 게터와 HUD Canvas 폴백 공용). */
	static FText FormatKeyBindings(const TArray<FRopePluginKeyBinding>& Bindings);
	static FText FormatEntries(const TArray<FRopePluginInfoEntry>& Entries);

protected:
	//~ UUserWidget
	virtual void NativeConstruct() override;

	/** 패널 식별자 → BindWidgetOptional 루트 컨테이너(없으면 null). */
	UPanelWidget* GetPanelWidget(ERopeInfoPanel Panel) const;

	/** 각 컨테이너를 비우고 콘텐츠 배열로 행 위젯(TextBlock)을 채운다(NativeConstruct에서 호출). */
	void PopulatePanels();

	//~ WBP가 이름을 맞춰 두면 C++가 채우고 표시/숨김을 제어한다(선택). --------------------
	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> KeyGuidePanel;

	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> ComponentsPanel;

	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> CapabilitiesPanel;

	/** 없으면 한계 항목이 CapabilitiesPanel 뒤에 이어 붙는다(구 WBP 호환). */
	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> LimitationsPanel;

	/** 상시 표시되는 힌트 줄(토글 패널과 별개 — 어느 패널을 켜/꺼도 계속 보인다). WBP에서 이름을 맞춰 둔다. */
	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UTextBlock> HintText;
};
