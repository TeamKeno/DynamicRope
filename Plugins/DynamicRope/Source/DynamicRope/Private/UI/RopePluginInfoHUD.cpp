// Copyright 2026 TeamKeno. All Rights Reserved.

#include "UI/RopePluginInfoHUD.h"

#include "Blueprint/UserWidget.h"
#include "Components/InputComponent.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "GameFramework/PlayerController.h"

ARopePluginInfoHUD::ARopePluginInfoHUD()
{
	// The number keys 1, 2 and 3 select a panel and H shows or hides everything.
	// F1 to F8 are avoided because they collide with the editor viewport's view mode shortcuts.
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
				// Fills the always-visible hint line with the real toggle key labels, overwriting the widget's default wording.
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

	// AActor::EnableInput creates the input component and puts it on the player's input stack, so no input action asset is needed.
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

	// The canvas fallback state, as an exclusive toggle: turning one on turns the rest off.
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
	// Hides the open panel alone, leaving the always-visible hint line, or restores the last panel if none is open.
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

	// The canvas fallback.
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

	// Draws multi-line body text line by line so the Y advances exactly. A blank line from "\n\n" is preserved and becomes the paragraph spacing.
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

	// While the widget is alive the widget draws. The fallback applies only when no widget class is set.
	if (InfoWidget || !bDrawCanvasFallbackWhenNoWidget || !Canvas)
	{
		return;
	}

	UFont* Font = GEngine ? GEngine->GetMediumFont() : nullptr;
	float X = 40.0f;
	float Y = 40.0f;

	// The always-visible hint line, which stays on screen even when every panel is hidden.
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
