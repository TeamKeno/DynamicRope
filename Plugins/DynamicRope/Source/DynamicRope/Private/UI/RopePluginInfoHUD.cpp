// Copyright Epic Games, Inc. All Rights Reserved.

#include "UI/RopePluginInfoHUD.h"

#include "Blueprint/UserWidget.h"
#include "Components/InputComponent.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "GameFramework/PlayerController.h"

ARopePluginInfoHUD::ARopePluginInfoHUD()
{
	// 숫자키 1/2/3으로 패널 선택, H로 전체 표시/숨김.
	// (F1~F8은 에디터 뷰포트 ViewMode 단축키와 겹쳐서 피한다.)
	KeyGuideToggleKey = EKeys::One;
	ComponentsToggleKey = EKeys::Two;
	CapabilitiesToggleKey = EKeys::Three;
	LimitationsToggleKey = EKeys::Four;
	ToolsToggleKey = EKeys::Five;
	MasterToggleKey = EKeys::H;
}

void ARopePluginInfoHUD::BeginPlay()
{
	Super::BeginPlay();

	if (InfoWidgetClass)
	{
		if (APlayerController* PC = GetOwningPlayerController())
		{
			InfoWidget = CreateWidget<URopePluginInfoWidget>(PC, InfoWidgetClass);
			if (InfoWidget)
			{
				InfoWidget->AddToViewport();
				// 상시 힌트 줄을 실제 토글 키 라벨로 채운다(위젯의 기본 문구를 덮어씀).
				InfoWidget->SetHintText(BuildHintText());
			}
		}
	}

	SetupInputBindings();
}

void ARopePluginInfoHUD::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (InfoWidget)
	{
		InfoWidget->RemoveFromParent();
		InfoWidget = nullptr;
	}
	Super::EndPlay(EndPlayReason);
}

void ARopePluginInfoHUD::SetupInputBindings()
{
	APlayerController* PC = GetOwningPlayerController();
	if (!PC)
	{
		return;
	}

	// AActor::EnableInput이 InputComponent를 만들어 플레이어 입력 스택에 올려 준다(Input Action 에셋 불필요).
	EnableInput(PC);
	if (!InputComponent)
	{
		return;
	}

	InputComponent->BindKey(KeyGuideToggleKey, IE_Pressed, this, &ARopePluginInfoHUD::OnKeyGuideKey);
	InputComponent->BindKey(ComponentsToggleKey, IE_Pressed, this, &ARopePluginInfoHUD::OnComponentsKey);
	InputComponent->BindKey(CapabilitiesToggleKey, IE_Pressed, this, &ARopePluginInfoHUD::OnCapabilitiesKey);
	InputComponent->BindKey(LimitationsToggleKey, IE_Pressed, this, &ARopePluginInfoHUD::OnLimitationsKey);
	InputComponent->BindKey(ToolsToggleKey, IE_Pressed, this, &ARopePluginInfoHUD::OnToolsKey);
	InputComponent->BindKey(MasterToggleKey, IE_Pressed, this, &ARopePluginInfoHUD::ToggleAll);
}

void ARopePluginInfoHUD::TogglePanel(ERopeInfoPanel Panel)
{
	LastShownPanel = Panel;

	if (InfoWidget)
	{
		InfoWidget->TogglePanel(Panel);
		return;
	}

	// Canvas 폴백 상태(배타 토글 — 하나 켜면 나머지는 꺼짐).
	const int32 Idx = static_cast<int32>(Panel);
	if (Idx < 0 || Idx >= UE_ARRAY_COUNT(bFallbackPanelVisible))
	{
		return;
	}
	const bool bWasVisible = bFallbackPanelVisible[Idx];
	for (bool& bVisible : bFallbackPanelVisible)
	{
		bVisible = false;
	}
	bFallbackPanelVisible[Idx] = !bWasVisible;
}

void ARopePluginInfoHUD::ToggleAll()
{
	// 상시 힌트 줄은 그대로 두고 열린 패널만 숨긴다 / 없으면 마지막 패널을 복원한다.
	if (InfoWidget)
	{
		const bool bAnyVisible =
			InfoWidget->IsPanelVisible(ERopeInfoPanel::KeyGuide) ||
			InfoWidget->IsPanelVisible(ERopeInfoPanel::Components) ||
			InfoWidget->IsPanelVisible(ERopeInfoPanel::Capabilities) ||
			InfoWidget->IsPanelVisible(ERopeInfoPanel::Limitations) ||
			InfoWidget->IsPanelVisible(ERopeInfoPanel::Tools);

		if (bAnyVisible)
		{
			InfoWidget->HideAllPanels();
		}
		else
		{
			InfoWidget->SetPanelVisible(LastShownPanel, true);
		}
		return;
	}

	// Canvas 폴백.
	bool bAnyVisible = false;
	for (const bool bVisible : bFallbackPanelVisible)
	{
		bAnyVisible |= bVisible;
	}
	for (bool& bVisible : bFallbackPanelVisible)
	{
		bVisible = false;
	}
	if (!bAnyVisible)
	{
		bFallbackPanelVisible[static_cast<int32>(LastShownPanel)] = true;
	}
}

FText ARopePluginInfoHUD::BuildHintText() const
{
	return FText::FromString(FString::Printf(
		TEXT("Dynamic Rope   [%s] Keys    [%s] Components    [%s] Capabilities    [%s] Limits    [%s] Tools    [%s] Hide"),
		*KeyGuideToggleKey.GetDisplayName().ToString(),
		*ComponentsToggleKey.GetDisplayName().ToString(),
		*CapabilitiesToggleKey.GetDisplayName().ToString(),
		*LimitationsToggleKey.GetDisplayName().ToString(),
		*ToolsToggleKey.GetDisplayName().ToString(),
		*MasterToggleKey.GetDisplayName().ToString()));
}

float ARopePluginInfoHUD::DrawFallbackBlock(const FString& Title, const FText& Body, float X, float Y)
{
	UFont* Font = GEngine ? GEngine->GetMediumFont() : nullptr;
	const float LineH = (Font ? Font->GetMaxCharHeight() : 14.0f) + 2.0f;

	if (!Title.IsEmpty())
	{
		DrawText(Title, FLinearColor(1.0f, 0.85f, 0.2f), X, Y, Font);
		Y += LineH;
	}

	// 멀티라인 본문을 줄 단위로 그려 Y를 정확히 쌓는다("\n\n" 빈 줄은 보존해 문단 간격이 된다).
	TArray<FString> Lines;
	Body.ToString().ParseIntoArray(Lines, TEXT("\n"), false);
	for (const FString& Line : Lines)
	{
		DrawText(Line, FLinearColor::White, X + 8.0f, Y, Font);
		Y += LineH;
	}

	return Y + LineH * 0.5f;
}

void ARopePluginInfoHUD::DrawHUD()
{
	Super::DrawHUD();

	// 위젯이 살아 있으면 위젯이 그린다. 폴백은 WidgetClass 미설정일 때만.
	if (InfoWidget || !bDrawCanvasFallbackWhenNoWidget || !Canvas)
	{
		return;
	}

	UFont* Font = GEngine ? GEngine->GetMediumFont() : nullptr;
	float X = 40.0f;
	float Y = 40.0f;

	// 상시 힌트 줄(패널을 다 숨겨도 계속 보인다).
	DrawText(BuildHintText().ToString(), FLinearColor(0.4f, 1.0f, 0.5f), X, Y, Font);
	Y += (Font ? Font->GetMaxCharHeight() : 14.0f) + 12.0f;

	if (bFallbackPanelVisible[static_cast<int32>(ERopeInfoPanel::KeyGuide)])
	{
		Y = DrawFallbackBlock(TEXT("KEY GUIDE"),
			URopePluginInfoWidget::FormatKeyBindings(URopePluginInfoWidget::GetDefaultKeyBindings()), X, Y);
	}
	if (bFallbackPanelVisible[static_cast<int32>(ERopeInfoPanel::Components)])
	{
		Y = DrawFallbackBlock(TEXT("REQUIRED COMPONENTS"),
			URopePluginInfoWidget::FormatEntries(URopePluginInfoWidget::GetDefaultRequiredComponents()), X, Y);
	}
	if (bFallbackPanelVisible[static_cast<int32>(ERopeInfoPanel::Capabilities)])
	{
		Y = DrawFallbackBlock(TEXT("CAPABILITIES"),
			URopePluginInfoWidget::FormatEntries(URopePluginInfoWidget::GetDefaultCapabilities()), X, Y);
	}
	if (bFallbackPanelVisible[static_cast<int32>(ERopeInfoPanel::Limitations)])
	{
		Y = DrawFallbackBlock(TEXT("LIMITATIONS"),
			URopePluginInfoWidget::FormatEntries(URopePluginInfoWidget::GetDefaultLimitations()), X, Y);
	}
	if (bFallbackPanelVisible[static_cast<int32>(ERopeInfoPanel::Tools)])
	{
		Y = DrawFallbackBlock(TEXT("TOOLS & DIAGNOSTICS"),
			URopePluginInfoWidget::FormatEntries(URopePluginInfoWidget::GetDefaultTools()), X, Y);
	}
}
