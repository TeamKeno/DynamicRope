// Copyright 2026 TeamKeno. All Rights Reserved.

#include "UI/RopePluginInfoWidget.h"

#include "Blueprint/WidgetTree.h"
#include "Components/HorizontalBox.h"
#include "Components/HorizontalBoxSlot.h"
#include "Components/PanelWidget.h"
#include "Components/SizeBox.h"
#include "Components/TextBlock.h"
#include "Components/VerticalBoxSlot.h"
#include "Styling/CoreStyle.h"

#define LOCTEXT_NAMESPACE "RopePluginInfo"

namespace
{
	// The colour palette, whose values assume the container background is left dark in the widget Blueprint.
	const FLinearColor RopeInfoTitleColor(1.0f, 0.85f, 0.2f);   // Entry titles and section headers, in yellow.
	const FLinearColor RopeInfoBodyColor(0.92f, 0.92f, 0.92f);  // Body text, in light grey.
	const FLinearColor RopeInfoKeyColor(0.4f, 1.0f, 0.6f);      // Key labels, in green.

	// Builds a text block in the shared style.
	UTextBlock* MakeText(UWidgetTree* Tree, const FText& Text, int32 FontSize, const FLinearColor& Color, bool bWrap)
	{
		UTextBlock* TB = Tree->ConstructWidget<UTextBlock>();
		TB->SetText(Text);
		TB->SetColorAndOpacity(FSlateColor(Color));
		TB->SetFont(FCoreStyle::GetDefaultFontStyle("Regular", FontSize));
		TB->SetAutoWrapText(bWrap);
		return TB;
	}

	// Applies padding on a vertical box slot, and is silently ignored on any other panel type.
	void SetSlotPadding(UPanelSlot* Slot, const FMargin& Padding)
	{
		if (UVerticalBoxSlot* VBSlot = Cast<UVerticalBoxSlot>(Slot))
		{
			VBSlot->SetPadding(Padding);
		}
	}

	// Adds one entry, a title plus an optional description, to the panel.
	void AddEntry(UWidgetTree* Tree, UPanelWidget* Panel, const FRopePluginInfoEntry& Entry)
	{
		SetSlotPadding(Panel->AddChild(MakeText(Tree, Entry.Title, 13, RopeInfoTitleColor, false)), FMargin(0, 6, 0, 1));
		if (!Entry.Description.IsEmpty())
		{
			SetSlotPadding(Panel->AddChild(MakeText(Tree, Entry.Description, 10, RopeInfoBodyColor, true)), FMargin(12, 0, 0, 2));
		}
	}

	// A single section header line, for when support and limitations share one panel.
	void AddHeader(UWidgetTree* Tree, UPanelWidget* Panel, const FText& Text)
	{
		SetSlotPadding(Panel->AddChild(MakeText(Tree, Text, 14, RopeInfoTitleColor, false)), FMargin(0, 10, 0, 4));
	}

	// A single key guide line: a fixed-width key column followed by the description of what it does. A size box fixes the column width so it stays aligned even in a proportional font.
	void AddKeyRow(UWidgetTree* Tree, UPanelWidget* Panel, const FRopePluginKeyBinding& Binding)
	{
		UHorizontalBox* Row = Tree->ConstructWidget<UHorizontalBox>();

		USizeBox* KeyBox = Tree->ConstructWidget<USizeBox>();
		KeyBox->SetWidthOverride(160.0f);
		KeyBox->AddChild(MakeText(Tree, Binding.Key, 12, RopeInfoKeyColor, false));
		Row->AddChild(KeyBox);

		if (UHorizontalBoxSlot* ActionSlot = Cast<UHorizontalBoxSlot>(Row->AddChild(MakeText(Tree, Binding.Action, 12, RopeInfoBodyColor, true))))
		{
			ActionSlot->SetSize(FSlateChildSize(ESlateSizeRule::Fill));
		}

		SetSlotPadding(Panel->AddChild(Row), FMargin(0, 2, 0, 2));
	}
}

URopePluginInfoWidget::URopePluginInfoWidget(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// The default content is filled in ahead of time and can be overwritten in the widget Blueprint's defaults to suit the project.
	KeyBindings = GetDefaultKeyBindings();
	RequiredComponents = GetDefaultRequiredComponents();
	Capabilities = GetDefaultCapabilities();
	Limitations = GetDefaultLimitations();
	Tools = GetDefaultTools();
}

void URopePluginInfoWidget::NativeConstruct()
{
	Super::NativeConstruct();

	// C++ fills the named containers with data; the widget Blueprint supplies the empty containers alone.
	PopulatePanels();

	// The default wording of the always-visible hint. The HUD overwrites it with the real key labels on BeginPlay, so this is the fallback until then.
	SetHintText(LOCTEXT("HintDefault", "[1] Keys   [2] Components   [3] Capabilities   [4] Limits   [5] Tools   [H] Hide"));

	// The initial state leaves the key guide on and hides the rest.
	SetPanelVisible(ERopeInfoPanel::KeyGuide, true);
	SetPanelVisible(ERopeInfoPanel::Components, false);
	SetPanelVisible(ERopeInfoPanel::Capabilities, false);
	SetPanelVisible(ERopeInfoPanel::Limitations, false);
	SetPanelVisible(ERopeInfoPanel::Tools, false);

	// Further customization after the automatic fill goes here; implementing it is optional.
	OnRefreshContent();
}

void URopePluginInfoWidget::PopulatePanels()
{
	if (!WidgetTree)
	{
		return;
	}

	// The key guide panel: a key label plus what it does.
	if (KeyGuidePanel)
	{
		KeyGuidePanel->ClearChildren();
		for (const FRopePluginKeyBinding& Binding : KeyBindings)
		{
			AddKeyRow(WidgetTree, KeyGuidePanel, Binding);
		}
	}

	// The required components panel: a title plus a description.
	if (ComponentsPanel)
	{
		ComponentsPanel->ClearChildren();
		for (const FRopePluginInfoEntry& Entry : RequiredComponents)
		{
			AddEntry(WidgetTree, ComponentsPanel, Entry);
		}
	}

	// The support panel.
	if (CapabilitiesPanel)
	{
		CapabilitiesPanel->ClearChildren();
		AddHeader(WidgetTree, CapabilitiesPanel, LOCTEXT("CapabilitiesHeader", "CAPABILITIES"));
		for (const FRopePluginInfoEntry& Entry : Capabilities)
		{
			AddEntry(WidgetTree, CapabilitiesPanel, Entry);
		}
	}

	// The tools and diagnostics panel.
	if (ToolsPanel)
	{
		ToolsPanel->ClearChildren();
		AddHeader(WidgetTree, ToolsPanel, LOCTEXT("ToolsHeader", "TOOLS & DIAGNOSTICS"));
		for (const FRopePluginInfoEntry& Entry : Tools)
		{
			AddEntry(WidgetTree, ToolsPanel, Entry);
		}
	}

	// The limitations panel. In a widget Blueprint with no dedicated container it is appended after the support panel as before, for compatibility with older widget Blueprints.
	if (UPanelWidget* Target = LimitationsPanel ? ToRawPtr(LimitationsPanel) : ToRawPtr(CapabilitiesPanel))
	{
		if (LimitationsPanel)
		{
			LimitationsPanel->ClearChildren();
		}
		AddHeader(WidgetTree, Target, LOCTEXT("LimitationsHeader", "LIMITATIONS"));
		for (const FRopePluginInfoEntry& Entry : Limitations)
		{
			AddEntry(WidgetTree, Target, Entry);
		}
	}
}

UPanelWidget* URopePluginInfoWidget::GetPanelWidget(ERopeInfoPanel Panel) const
{
	switch (Panel)
	{
	case ERopeInfoPanel::KeyGuide:     return KeyGuidePanel;
	case ERopeInfoPanel::Components:   return ComponentsPanel;
	case ERopeInfoPanel::Capabilities: return CapabilitiesPanel;
	// With no dedicated container it is part of the support panel and follows that panel's visibility.
	case ERopeInfoPanel::Limitations:  return LimitationsPanel ? LimitationsPanel : CapabilitiesPanel;
	case ERopeInfoPanel::Tools:        return ToolsPanel;
	default:                           return nullptr;
	}
}

void URopePluginInfoWidget::TogglePanel(ERopeInfoPanel Panel)
{
	// An exclusive toggle: pressing the panel that is already on turns everything off, and otherwise only that panel is on and the rest are off.
	const bool bWasVisible = IsPanelVisible(Panel);
	HideAllPanels();
	if (!bWasVisible)
	{
		SetPanelVisible(Panel, true);
	}
}

void URopePluginInfoWidget::SetPanelVisible(ERopeInfoPanel Panel, bool bVisible)
{
	if (UWidget* PanelWidget = GetPanelWidget(Panel))
	{
		PanelWidget->SetVisibility(bVisible ? ESlateVisibility::SelfHitTestInvisible : ESlateVisibility::Collapsed);
	}
}

void URopePluginInfoWidget::HideAllPanels()
{
	SetPanelVisible(ERopeInfoPanel::KeyGuide, false);
	SetPanelVisible(ERopeInfoPanel::Components, false);
	SetPanelVisible(ERopeInfoPanel::Capabilities, false);
	SetPanelVisible(ERopeInfoPanel::Limitations, false);
	SetPanelVisible(ERopeInfoPanel::Tools, false);
}

void URopePluginInfoWidget::SetHintText(const FText& InText)
{
	if (HintText)
	{
		HintText->SetText(InText);
	}
}

bool URopePluginInfoWidget::IsPanelVisible(ERopeInfoPanel Panel) const
{
	const UWidget* PanelWidget = GetPanelWidget(Panel);
	return PanelWidget && PanelWidget->GetVisibility() != ESlateVisibility::Collapsed
		&& PanelWidget->GetVisibility() != ESlateVisibility::Hidden;
}

FText URopePluginInfoWidget::GetKeyGuideText() const
{
	return FormatKeyBindings(KeyBindings);
}

FText URopePluginInfoWidget::GetComponentsText() const
{
	return FormatEntries(RequiredComponents);
}

FText URopePluginInfoWidget::GetCapabilitiesText() const
{
	return FormatEntries(Capabilities);
}

FText URopePluginInfoWidget::GetLimitationsText() const
{
	return FormatEntries(Limitations);
}

FText URopePluginInfoWidget::GetToolsText() const
{
	return FormatEntries(Tools);
}

FText URopePluginInfoWidget::FormatKeyBindings(const TArray<FRopePluginKeyBinding>& Bindings)
{
	FString Result;
	for (const FRopePluginKeyBinding& Binding : Bindings)
	{
		if (!Result.IsEmpty())
		{
			Result += TEXT("\n");
		}
		// "  LMB   Throw / release rope" — the key label is padded to the left-aligned column width.
		Result += FString::Printf(TEXT("%-16s %s"), *Binding.Key.ToString(), *Binding.Action.ToString());
	}
	return FText::FromString(Result);
}

FText URopePluginInfoWidget::FormatEntries(const TArray<FRopePluginInfoEntry>& Entries)
{
	FString Result;
	for (const FRopePluginInfoEntry& Entry : Entries)
	{
		if (!Result.IsEmpty())
		{
			Result += TEXT("\n\n");
		}
		Result += FString::Printf(TEXT("• %s"), *Entry.Title.ToString());
		const FString Desc = Entry.Description.ToString();
		if (!Desc.IsEmpty())
		{
			Result += FString::Printf(TEXT("\n    %s"), *Desc);
		}
	}
	return FText::FromString(Result);
}

TArray<FRopePluginKeyBinding> URopePluginInfoWidget::GetDefaultKeyBindings()
{
	// The real keys are decided by the project's input mapping context and the URopeWielderComponent actions.
	// The labels below match this project's current bindings, so a rebinding is edited here, or in the widget Blueprint's defaults.
	auto Make = [](const TCHAR* Key, const TCHAR* Action)
	{
		FRopePluginKeyBinding B;
		B.Key = FText::FromString(Key);
		B.Action = FText::FromString(Action);
		return B;
	};

	return {
		Make(TEXT("LMB"),        TEXT("Throw")),
		Make(TEXT("RMB"),        TEXT("Release")),
		Make(TEXT("Wheel Up"),   TEXT("Reel in")),
		Make(TEXT("Wheel Down"), TEXT("Reel out")),
		Make(TEXT("T"),          TEXT("Arm the pull (toggle) - fires itself when the rope goes taut")),
		Make(TEXT("R"),          TEXT("Reload - back to the ready state")),
	};
}

TArray<FRopePluginInfoEntry> URopePluginInfoWidget::GetDefaultRequiredComponents()
{
	auto Make = [](const TCHAR* Title, const TCHAR* Desc)
	{
		FRopePluginInfoEntry E;
		E.Title = FText::FromString(Title);
		E.Description = FText::FromString(Desc);
		return E;
	};

	return {
		Make(TEXT("URopeComponent"),
			TEXT("The rope itself. Attach, then Throw().")),
		Make(TEXT("URopeWielderComponent  (optional)"),
			TEXT("Makes a character use it - hand socket, input, aiming.")),
		Make(TEXT("Collider provider  (on wrap targets)"),
			TEXT("Bone capsules or baked SDF on characters; URopeWrapTargetComponent on props.")),
		Make(TEXT("URopeSimSubsystem  (automatic)"),
			TEXT("Ticks every rope and gathers colliders. No setup.")),
		Make(TEXT("URopePreviewComponent  (optional)"),
			TEXT("Throw-arc preview before you throw.")),
		Make(TEXT("URopeRagdollResponseComponent  (optional)"),
			TEXT("Target goes limp when wrapped, stands up when released.")),
		Make(TEXT("URopeReelGaugeComponent  (optional)"),
			TEXT("On-character gauge that lights while the rope length changes.")),
		Make(TEXT("Rope anim notifies  (optional)"),
			TEXT("Fire the throw and the pull window from a montage.")),
		Make(TEXT("URopePreset  (optional asset)"),
			TEXT("A whole tuning in one call. Six ship with the plugin.")),
	};
}

TArray<FRopePluginInfoEntry> URopePluginInfoWidget::GetDefaultCapabilities()
{
	auto Make = [](const TCHAR* Title, const TCHAR* Desc)
	{
		FRopePluginInfoEntry E;
		E.Title = FText::FromString(Title);
		E.Description = FText::FromString(Desc);
		return E;
	};

	return {
		Make(TEXT("Throw, flight & collision"),
			TEXT("Flies as a simulated chain, collides with bones.")),
		Make(TEXT("Wrap around bones & props"),
			TEXT("Coils on a bone or a prop's collision and follows it.")),
		Make(TEXT("Hold / pull / release"),
			TEXT("Tension tether plus an armed pull that engages when taut.")),
		Make(TEXT("Reel in / out"),
			TEXT("Change rope length at runtime.")),
		Make(TEXT("Cross-actor wrap"),
			TEXT("A rope on one actor can wrap and follow another.")),
		Make(TEXT("GPU solver & tube"),
			TEXT("Solve, contact detection and tube build on the GPU.")),
		Make(TEXT("SDF collision"),
			TEXT("Baked per-bone distance fields for thin limbs.")),
		Make(TEXT("Cut"),
			TEXT("Sever the rope on a gameplay event.")),
		Make(TEXT("Three throw modes"),
			TEXT("Full sim / assisted / guaranteed. Per rope.")),
		Make(TEXT("Pierce"),
			TEXT("Spear tip plants a single anchor instead of coiling.")),
		Make(TEXT("Hang & swing"),
			TEXT("Hang from the taut rope and steer the swing; anim hooks included.")),
		Make(TEXT("Mass decides who moves"),
			TEXT("Heavy target pulls you in instead - same rule, not a mode.")),
		Make(TEXT("Moving surfaces"),
			TEXT("A moving body drags and sweeps the rope aside.")),
		Make(TEXT("Swept contact"),
			TEXT("Fast nodes and fast colliders do not tunnel.")),
		Make(TEXT("World collision"),
			TEXT("Analytic static meshes, global distance field, opt-in physics bodies.")),
		Make(TEXT("Runtime presets"),
			TEXT("Swap the whole tuning while free or reeled.")),
	};
}

TArray<FRopePluginInfoEntry> URopePluginInfoWidget::GetDefaultLimitations()
{
	auto Make = [](const TCHAR* Title, const TCHAR* Desc)
	{
		FRopePluginInfoEntry E;
		E.Title = FText::FromString(Title);
		E.Description = FText::FromString(Desc);
		return E;
	};

	return {
		Make(TEXT("No networking"),
			TEXT("Local simulation only. Replicate events, simulate per machine.")),
		Make(TEXT("No save / load"),
			TEXT("Sim state is transient - a rope in flight will not survive a save.")),
		Make(TEXT("One dominant wrap target per rope"),
			TEXT("Secondary seeds latch on, but only the dominant target gets the spiral.")),
		Make(TEXT("Ragdolls hang from a physics constraint"),
			TEXT("Like dragging a body by a handle, not a full-body force model.")),
		Make(TEXT("GPU tube ring limit"),
			TEXT("Over 512 rings falls back to the CPU tube builder.")),
		Make(TEXT("CPU fallback contexts"),
			TEXT("Cook, dedicated server and -nullrhi have no renderable RHI.")),
		Make(TEXT("Some debug overlays are local only"),
			TEXT("Foreground DrawDebug overlays do not reach a remote client.")),
		Make(TEXT("Wrap tuning is sensitive"),
			TEXT("Defaults latch on first sustained contact - tune per target.")),
	};
}

TArray<FRopePluginInfoEntry> URopePluginInfoWidget::GetDefaultTools()
{
	auto Make = [](const TCHAR* Title, const TCHAR* Desc)
	{
		FRopePluginInfoEntry E;
		E.Title = FText::FromString(Title);
		E.Description = FText::FromString(Desc);
		return E;
	};

	// The console commands and the debug overlay exist in non-shipping builds alone.
	return {
		Make(TEXT("stat DynamicRope"),
			TEXT("Frame cost per stage, active vs sleeping ropes, GPU timings.")),
		Make(TEXT("Gameplay Debugger"),
			TEXT("Apostrophe key - 'Rope' inspects one rope, 'RopePerf' the whole world.")),
		Make(TEXT("Rope.Preset.List / .Apply / .Cycle"),
			TEXT("Swap tuning presets on the world's ropes at runtime.")),
		Make(TEXT("Rope.Ragdoll  /  .Recover  /  .Destroy"),
			TEXT("Toggle ragdoll on wrap targets, or destroy one mid-wrap.")),
		Make(TEXT("Rope category views: P/U/I/O/J/K"),
			TEXT("In the 'Rope' category, toggle nodes/flight/wrap/colliders/aim/advanced.")),
		Make(TEXT("Tools > Rope SDF Authoring"),
			TEXT("Editor tab that bakes per-bone distance fields for a skeletal mesh.")),
		Make(TEXT("Project Settings > Plugins > Dynamic Rope"),
			TEXT("Project-wide defaults, HUD widget classes, demo preset list.")),
		Make(TEXT("Session Frontend > Automation"),
			TEXT("Filter on 'DynamicRope.' to run the solver, wrap and GPU parity tests.")),
	};
}

#undef LOCTEXT_NAMESPACE
