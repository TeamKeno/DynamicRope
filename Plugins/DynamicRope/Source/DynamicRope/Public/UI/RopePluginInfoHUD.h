// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The plugin's information HUD. It creates a URopePluginInfoWidget, adds it to the viewport, and
// toggles each panel individually from an input key: the key guide, the required components, and the
// capabilities and limitations. Set this class, or a Blueprint child of it, as the game mode's HUD
// class and nothing else is needed; it appears on Play.
//
// Input: it binds the toggle keys directly through AActor::EnableInput and InputComponent->BindKey,
// so no input action assets are required. The defaults are F1 to F4, which rarely clash with a
// gameplay mapping context; change them on the class defaults if needed.
//
// Widget fallback: leaving InfoWidgetClass empty, as when no Blueprint has been authored, makes
// DrawHUD render the same content as canvas text, so it can be checked immediately without any art.
// Assigning a Blueprint disables the canvas fallback automatically.

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

	//~ AActor and AHUD
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void DrawHUD() override;

	/** The widget class added to the viewport, normally a Blueprint deriving from it. Leave it empty to
	 *  use the canvas text fallback. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info")
	TSubclassOf<URopePluginInfoWidget> InfoWidgetClass;

	/** Whether DrawHUD renders the content to the canvas while InfoWidgetClass is empty. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info")
	bool bDrawCanvasFallbackWhenNoWidget = true;

	//~ The individual toggle keys, defaulting to F1 to F4 to avoid clashing with gameplay.
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey KeyGuideToggleKey;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey ComponentsToggleKey;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey CapabilitiesToggleKey;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey LimitationsToggleKey;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey ToolsToggleKey;

	/** Shows or hides the entire HUD at once. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Rope|Info|Input")
	FKey MasterToggleKey;

	//~ API ----------------------------------------------------------------
	/** Toggles the given panel, applying it to the widget when there is one and to the canvas fallback
	 *  state otherwise. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void TogglePanel(ERopeInfoPanel Panel);

	/** Hides every open panel, and restores the last one when pressed again. The always-visible hint
	 *  line is unaffected. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void ToggleAll();

	/** Builds the text of the always-visible hint line from the current toggle key labels, shared by
	 *  the widget and the canvas fallback. */
	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText BuildHintText() const;

	/** The created widget, or null while the canvas fallback is in use. */
	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	URopePluginInfoWidget* GetInfoWidget() const { return InfoWidget; }

protected:
	/** Binds the toggle keys to the input component after EnableInput. */
	void SetupInputBindings();

	// The key handlers bound through BindKey.
	void OnKeyGuideKey()     { TogglePanel(ERopeInfoPanel::KeyGuide); }
	void OnComponentsKey()   { TogglePanel(ERopeInfoPanel::Components); }
	void OnCapabilitiesKey() { TogglePanel(ERopeInfoPanel::Capabilities); }
	void OnLimitationsKey()  { TogglePanel(ERopeInfoPanel::Limitations); }
	void OnToolsKey()        { TogglePanel(ERopeInfoPanel::Tools); }

	// Canvas fallback: draws a title and body block and returns the Y position of the next block.
	float DrawFallbackBlock(const FString& Title, const FText& Body, float X, float Y);

	/** The created widget instance, null while using the canvas fallback. */
	UPROPERTY(Transient)
	TObjectPtr<URopePluginInfoWidget> InfoWidget = nullptr;

	// Panel visibility used by the canvas fallback only; when a widget exists it owns the truth. The
	// hint line is always visible.
	bool bFallbackPanelVisible[5] = { true, false, false, false, false };

	// The panel most recently opened, restored when ToggleAll goes from hidden back to shown.
	ERopeInfoPanel LastShownPanel = ERopeInfoPanel::KeyGuide;
};
