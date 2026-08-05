// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The presentation widget for the plugin's information HUD. The C++ side owns only the content data,
// namely the key guide, the required components, and the capabilities and limitations, together with
// the panel toggle logic; the layout itself belongs to a Blueprint widget authored in the editor that
// derives from this class. ARopePluginInfoHUD creates the widget, adds it to the viewport, and
// toggles each panel from an input key.
//
// What the Blueprint has to do: provide named empty containers, and nothing in the graph.
//  - Make the root of each panel a UPanelWidget, preferably a vertical box so slot padding applies,
//    and name them KeyGuidePanel, ComponentsPanel, CapabilitiesPanel, LimitationsPanel and
//    ToolsPanel, which are bound optionally.
//    A Blueprint without a LimitationsPanel still works: the limitation entries are appended after
//    the capabilities panel instead.
//  - Keep each entry to a title plus one line. This HUD is a reference table to be scanned rather
//    than a document to be read, and longer descriptions push content off screen; scrolling is not
//    available because the mouse wheel is already bound to reeling.
//  - The C++ side empties these containers during NativeConstruct and fills them with text block rows
//    built from the content arrays below. The Blueprint is responsible only for layout and style,
//    meaning where the containers sit, how large they are and what is behind them, while showing and
//    hiding is controlled from C++.
//  - Getting a name wrong simply leaves that pointer null and the panel empty; it compiles and runs
//    normally.
// The content arrays are editable, so they can be overridden per project from the Blueprint's
// defaults, with the C++ constructor supplying the defaults.
// For further customization, do the extra work in OnRefreshContent, which is called after the
// automatic fill.
// The GetXxxText() functions remain as an alternative path for binding a single text block directly.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "RopePluginInfoWidget.generated.h"

class UPanelWidget;
class UTextBlock;

/** An individually toggleable panel. The identifier that links a HUD input key to a widget panel. */
UENUM(BlueprintType)
enum class ERopeInfoPanel : uint8
{
	/** The input key guide, covering throwing, pulling, reeling, cutting and so on. */
	KeyGuide,
	/** Which components have to be added to use a rope. */
	Components,
	/** What the plugin supports. */
	Capabilities,
	/** What its limitations are. It used to share a panel with the capabilities and was separated
	 *  because the two together overflowed the screen. A Blueprint with no LimitationsPanel container
	 *  appends these entries under the capabilities panel as before. */
	Limitations,
	/** Where to look: console commands, the profiler, the debugger and editor tooling. */
	Tools
};

/** One line of the key guide: a key label and what that key does. The actual key is decided by the
 *  project's input mapping context, so the label is for display only. */
USTRUCT(BlueprintType)
struct FRopePluginKeyBinding
{
	GENERATED_BODY()

	/** The key label shown on screen, such as "LMB", "R" or "Mouse Wheel Up". */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Info")
	FText Key;

	/** A description of what that key does. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Info")
	FText Action;
};

/** An information entry of a title plus a description, shared by the required components, the
 *  capabilities and the limitations. */
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

	//~ Content data. The constructor supplies the defaults and a Blueprint's defaults can override
	//~ them.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Key Guide")
	TArray<FRopePluginKeyBinding> KeyBindings;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Components")
	TArray<FRopePluginInfoEntry> RequiredComponents;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Capabilities")
	TArray<FRopePluginInfoEntry> Capabilities;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Capabilities")
	TArray<FRopePluginInfoEntry> Limitations;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Info|Tools")
	TArray<FRopePluginInfoEntry> Tools;

	//~ The panel toggle API, called by the HUD's input keys and also exposed to Blueprint.
	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void TogglePanel(ERopeInfoPanel Panel);

	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void SetPanelVisible(ERopeInfoPanel Panel, bool bVisible);

	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void HideAllPanels();

	/** Sets the text of the always-visible hint line, which the HUD fills in with the actual toggle key
	 *  labels. It is shown only when a HintText slot exists. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Info")
	void SetHintText(const FText& InText);

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	bool IsPanelVisible(ERopeInfoPanel Panel) const;

	//~ The simple path: multi-line text to bind to a single text block.
	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetKeyGuideText() const;

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetComponentsText() const;

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetCapabilitiesText() const;

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetLimitationsText() const;

	UFUNCTION(BlueprintPure, Category = "Rope|Info")
	FText GetToolsText() const;

	//~ The rich path: implement this in Blueprint to walk the arrays and build row widgets.
	/** Called once after NativeConstruct. The place to read the arrays and populate row widgets;
	 *  implementing it is optional. */
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Info")
	void OnRefreshContent();

	//~ The default content builders, shared by the widget constructor and the HUD's canvas fallback.
	static TArray<FRopePluginKeyBinding> GetDefaultKeyBindings();
	static TArray<FRopePluginInfoEntry> GetDefaultRequiredComponents();
	static TArray<FRopePluginInfoEntry> GetDefaultCapabilities();
	static TArray<FRopePluginInfoEntry> GetDefaultLimitations();
	static TArray<FRopePluginInfoEntry> GetDefaultTools();

	/** Formats an array as multi-line text, shared by the widget getters and the HUD's canvas
	 *  fallback. */
	static FText FormatKeyBindings(const TArray<FRopePluginKeyBinding>& Bindings);
	static FText FormatEntries(const TArray<FRopePluginInfoEntry>& Entries);

protected:
	//~ UUserWidget
	virtual void NativeConstruct() override;

	/** Maps a panel identifier to its optionally bound root container, or null. */
	UPanelWidget* GetPanelWidget(ERopeInfoPanel Panel) const;

	/** Empties each container and fills it with text block rows from the content arrays. Called from
	 *  NativeConstruct. */
	void PopulatePanels();

	//~ Optional: when the Blueprint uses these names, the C++ side fills them in and controls their
	//~ visibility.
	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> KeyGuidePanel;

	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> ComponentsPanel;

	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> CapabilitiesPanel;

	/** Without this, the limitation entries are appended after CapabilitiesPanel, which keeps older
	 *  Blueprints working. */
	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> LimitationsPanel;

	/** Without this the tools panel is simply empty, since there is no natural panel to append it
	 *  to. */
	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UPanelWidget> ToolsPanel;

	/** The always-visible hint line, which is separate from the toggleable panels and stays on screen
	 *  whichever of them are shown. Name it to match in the Blueprint. */
	UPROPERTY(meta = (BindWidgetOptional))
	TObjectPtr<UTextBlock> HintText;
};
