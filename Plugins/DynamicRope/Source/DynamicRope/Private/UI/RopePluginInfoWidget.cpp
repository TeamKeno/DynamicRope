// Copyright Epic Games, Inc. All Rights Reserved.

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
	// 색상 팔레트(WBP에서 컨테이너 배경을 어둡게 두는 것을 전제로 한 값).
	const FLinearColor RopeInfoTitleColor(1.0f, 0.85f, 0.2f);   // 항목 제목 / 섹션 헤더(노랑)
	const FLinearColor RopeInfoBodyColor(0.92f, 0.92f, 0.92f);  // 설명 본문(밝은 회색)
	const FLinearColor RopeInfoKeyColor(0.4f, 1.0f, 0.6f);      // 키 라벨(초록)

	// 공통 스타일의 TextBlock을 만든다.
	UTextBlock* MakeText(UWidgetTree* Tree, const FText& Text, int32 FontSize, const FLinearColor& Color, bool bWrap)
	{
		UTextBlock* TB = Tree->ConstructWidget<UTextBlock>();
		TB->SetText(Text);
		TB->SetColorAndOpacity(FSlateColor(Color));
		TB->SetFont(FCoreStyle::GetDefaultFontStyle("Regular", FontSize));
		TB->SetAutoWrapText(bWrap);
		return TB;
	}

	// VerticalBox 슬롯이면 패딩을 준다(다른 패널 타입이면 조용히 무시).
	void SetSlotPadding(UPanelSlot* Slot, const FMargin& Padding)
	{
		if (UVerticalBoxSlot* VBSlot = Cast<UVerticalBoxSlot>(Slot))
		{
			VBSlot->SetPadding(Padding);
		}
	}

	// 제목 + (선택)설명 한 항목을 패널에 추가한다.
	void AddEntry(UWidgetTree* Tree, UPanelWidget* Panel, const FRopePluginInfoEntry& Entry)
	{
		SetSlotPadding(Panel->AddChild(MakeText(Tree, Entry.Title, 13, RopeInfoTitleColor, false)), FMargin(0, 6, 0, 1));
		if (!Entry.Description.IsEmpty())
		{
			SetSlotPadding(Panel->AddChild(MakeText(Tree, Entry.Description, 10, RopeInfoBodyColor, true)), FMargin(12, 0, 0, 2));
		}
	}

	// 섹션 헤더 한 줄(지원/한계를 한 패널에 나눠 담을 때).
	void AddHeader(UWidgetTree* Tree, UPanelWidget* Panel, const FText& Text)
	{
		SetSlotPadding(Panel->AddChild(MakeText(Tree, Text, 14, RopeInfoTitleColor, false)), FMargin(0, 10, 0, 4));
	}

	// 키 안내 한 줄: [고정폭 키 컬럼] [동작 설명]. 비고정폭 폰트라도 컬럼이 정렬되도록 SizeBox로 폭 고정.
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
	// 기본 콘텐츠를 미리 채워 둔다. WBP 디폴트에서 프로젝트에 맞게 덮어쓸 수 있다.
	KeyBindings = GetDefaultKeyBindings();
	RequiredComponents = GetDefaultRequiredComponents();
	Capabilities = GetDefaultCapabilities();
	Limitations = GetDefaultLimitations();
}

void URopePluginInfoWidget::NativeConstruct()
{
	Super::NativeConstruct();

	// C++가 명명된 컨테이너를 데이터로 채운다(WBP는 빈 컨테이너만 제공).
	PopulatePanels();

	// 상시 힌트의 기본 문구(HUD가 BeginPlay에서 실제 키 라벨로 덮어쓴다 — 이건 그 전 폴백).
	SetHintText(LOCTEXT("HintDefault", "[1] Keys   [2] Components   [3] Capabilities & Limits   [H] Hide"));

	// 시작 상태: 키 안내만 켜 두고 나머지는 숨긴다.
	SetPanelVisible(ERopeInfoPanel::KeyGuide, true);
	SetPanelVisible(ERopeInfoPanel::Components, false);
	SetPanelVisible(ERopeInfoPanel::Capabilities, false);

	// 자동 채움 뒤 추가 커스터마이즈가 필요하면 여기서(구현은 선택).
	OnRefreshContent();
}

void URopePluginInfoWidget::PopulatePanels()
{
	if (!WidgetTree)
	{
		return;
	}

	// 키 안내 패널: 키 라벨 + 동작.
	if (KeyGuidePanel)
	{
		KeyGuidePanel->ClearChildren();
		for (const FRopePluginKeyBinding& Binding : KeyBindings)
		{
			AddKeyRow(WidgetTree, KeyGuidePanel, Binding);
		}
	}

	// 필요 컴포넌트 패널: 제목 + 설명.
	if (ComponentsPanel)
	{
		ComponentsPanel->ClearChildren();
		for (const FRopePluginInfoEntry& Entry : RequiredComponents)
		{
			AddEntry(WidgetTree, ComponentsPanel, Entry);
		}
	}

	// 지원/한계 패널: 두 섹션을 헤더로 나눠 한 컨테이너에 담는다.
	if (CapabilitiesPanel)
	{
		CapabilitiesPanel->ClearChildren();
		AddHeader(WidgetTree, CapabilitiesPanel, LOCTEXT("CapabilitiesHeader", "CAPABILITIES"));
		for (const FRopePluginInfoEntry& Entry : Capabilities)
		{
			AddEntry(WidgetTree, CapabilitiesPanel, Entry);
		}
		AddHeader(WidgetTree, CapabilitiesPanel, LOCTEXT("LimitationsHeader", "LIMITATIONS"));
		for (const FRopePluginInfoEntry& Entry : Limitations)
		{
			AddEntry(WidgetTree, CapabilitiesPanel, Entry);
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
	default:                           return nullptr;
	}
}

void URopePluginInfoWidget::TogglePanel(ERopeInfoPanel Panel)
{
	// 배타 토글: 켜져 있던 패널을 다시 누르면 전부 끄고, 아니면 그 패널만 켠다(나머지는 꺼짐).
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

FText URopePluginInfoWidget::FormatKeyBindings(const TArray<FRopePluginKeyBinding>& Bindings)
{
	FString Result;
	for (const FRopePluginKeyBinding& Binding : Bindings)
	{
		if (!Result.IsEmpty())
		{
			Result += TEXT("\n");
		}
		// "  LMB   Throw / release rope" — 키 라벨을 좌측 정렬 폭에 맞춘다.
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
	// 실제 키는 프로젝트의 Input Mapping Context / URopeWielderComponent 액션이 정한다.
	// 아래 라벨은 이 프로젝트의 현재 바인딩에 맞춘 값 — 바꾸면 여기(또는 WBP 디폴트)에서 수정한다.
	auto Make = [](const TCHAR* Key, const TCHAR* Action)
	{
		FRopePluginKeyBinding B;
		B.Key = FText::FromString(Key);
		B.Action = FText::FromString(Action);
		return B;
	};

	return {
		Make(TEXT("LMB"),        TEXT("Throw the rope")),
		Make(TEXT("RMB"),        TEXT("Release the rope")),
		Make(TEXT("Wheel Up"),   TEXT("Reel in (shorten the rope)")),
		Make(TEXT("Wheel Down"), TEXT("Reel out (lengthen the rope)")),
		Make(TEXT("R"),          TEXT("Pull the wrapped target")),
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
			TEXT("Core facade. Attach to the actor that owns the rope, then call Throw(). Owns the sim state, solver and phase state machine.")),
		Make(TEXT("URopeWielderComponent  (optional)"),
			TEXT("Gameplay wielder. Attaches the rope to a hand socket, wires Enhanced Input (throw / release / pull / reel) and handles aiming. One component to make a character throw a rope.")),
		Make(TEXT("A collider provider  (on wrap targets)"),
			TEXT("URopeBoneCapsuleProvider (per-bone capsules) or URopeSDFProvider (baked per-bone SDF). Put it on every actor the rope may wrap; it registers with the sim subsystem each frame.")),
		Make(TEXT("URopeSimSubsystem  (automatic)"),
			TEXT("World subsystem, created for you. Centrally ticks every rope and gathers colliders once per frame. No manual setup.")),
		Make(TEXT("URopePreviewComponent  (optional)"),
			TEXT("Draws a throw-arc / landing preview before you throw.")),
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
			TEXT("The rope flies as a physically-simulated chain and collides with skeletal bones.")),
		Make(TEXT("Wrap around bones"),
			TEXT("Wraps a character's bone and then follows the animation while held.")),
		Make(TEXT("Hold / Pull / Release"),
			TEXT("A convergent tether plus a constant active pull that transfers force to the wrapped character's movement.")),
		Make(TEXT("Reel in / out"),
			TEXT("Shorten or lengthen the rope at runtime — combine with the tether for a grapple pull-up.")),
		Make(TEXT("Cross-actor wrap"),
			TEXT("A rope owned by actor A can wrap and follow a bone on a different actor B (safely released if B is destroyed).")),
		Make(TEXT("GPU solver & tube"),
			TEXT("XPBD solve + contact detection + tube build run on the GPU when a renderable RHI exists; CPU is the fallback.")),
		Make(TEXT("SDF collision"),
			TEXT("Optional baked per-bone signed-distance-field colliders, authored in the editor, for thin limbs.")),
		Make(TEXT("Cut"),
			TEXT("Sever the rope on a gameplay event via CutRope().")),
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
		Make(TEXT("No networking / replication"),
			TEXT("The simulation is local only. For multiplayer, replicate the high-level events and simulate the rope locally on each machine.")),
		Make(TEXT("GPU tube ring limit"),
			TEXT("The GPU tube path requires NumRings <= 256; oversized ropes fall back to the CPU tube builder.")),
		Make(TEXT("CPU fallback contexts"),
			TEXT("Cook, dedicated server and -nullrhi have no renderable RHI, so they run the CPU solver / tube instead of the GPU path.")),
		Make(TEXT("Spiral wraps only the dominant target"),
			TEXT("Secondary seeds (two-leg / suspension) hold and commit on their own bones, but the wrap spiral is ")
			TEXT("built for the dominant target only. Raise MaxWrapSeeds above its default of 1 to enable them.")),
		Make(TEXT("Wrap tuning is sensitive"),
			TEXT("Default WrapDecisionTime (~1 frame) and MinLatchNodes (1) latch on first sustained contact — tune them for your targets.")),
	};
}

#undef LOCTEXT_NAMESPACE
