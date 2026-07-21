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
	SetHintText(LOCTEXT("HintDefault", "[1] Keys   [2] Components   [3] Capabilities   [4] Limits   [H] Hide"));

	// 시작 상태: 키 안내만 켜 두고 나머지는 숨긴다.
	SetPanelVisible(ERopeInfoPanel::KeyGuide, true);
	SetPanelVisible(ERopeInfoPanel::Components, false);
	SetPanelVisible(ERopeInfoPanel::Capabilities, false);
	SetPanelVisible(ERopeInfoPanel::Limitations, false);

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

	// 지원 패널.
	if (CapabilitiesPanel)
	{
		CapabilitiesPanel->ClearChildren();
		AddHeader(WidgetTree, CapabilitiesPanel, LOCTEXT("CapabilitiesHeader", "CAPABILITIES"));
		for (const FRopePluginInfoEntry& Entry : Capabilities)
		{
			AddEntry(WidgetTree, CapabilitiesPanel, Entry);
		}
	}

	// 한계 패널. 전용 컨테이너가 없는 WBP에서는 예전처럼 지원 패널 뒤에 이어 붙인다(구 WBP 호환).
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
	// 전용 컨테이너가 없으면 지원 패널과 한 몸이라 그쪽 가시성을 따른다.
	case ERopeInfoPanel::Limitations:  return LimitationsPanel ? LimitationsPanel : CapabilitiesPanel;
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
	SetPanelVisible(ERopeInfoPanel::Limitations, false);
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
			TEXT("Bone capsules or baked SDF. Required on anything the rope may wrap.")),
		Make(TEXT("URopeSimSubsystem  (automatic)"),
			TEXT("Ticks every rope and gathers colliders. No setup.")),
		Make(TEXT("URopePreviewComponent  (optional)"),
			TEXT("Throw-arc preview before you throw.")),
		Make(TEXT("URopeRagdollResponseComponent  (optional)"),
			TEXT("Target goes limp when wrapped, stands up when released.")),
		Make(TEXT("Rope anim notifies  (optional)"),
			TEXT("Fire the throw and the pull window from a montage.")),
		Make(TEXT("URopePreset  (optional asset)"),
			TEXT("A whole tuning in one call. Three ship with the plugin.")),
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
		Make(TEXT("Wrap around bones"),
			TEXT("Coils on a bone and follows the animation.")),
		Make(TEXT("Hold / pull / release"),
			TEXT("Tension-driven tether plus an active pull.")),
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
		Make(TEXT("Armed pull"),
			TEXT("Engages only once the rope is genuinely taut.")),
		Make(TEXT("Mass decides who moves"),
			TEXT("Heavy target pulls you in instead - same rule, not a mode.")),
		Make(TEXT("Moving surfaces"),
			TEXT("A moving body drags and sweeps the rope aside.")),
		Make(TEXT("Swept contact"),
			TEXT("Fast nodes and fast colliders do not tunnel.")),
		Make(TEXT("World collision"),
			TEXT("Analytic static meshes plus optional global distance field.")),
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
		Make(TEXT("Cinch is not implemented"),
			TEXT("Falls back to a bare wrap. Use pierce or bare wrap.")),
		Make(TEXT("One wrap target per rope"),
			TEXT("A rope commits to a single dominant target.")),
		Make(TEXT("Ragdolls hang from a physics constraint"),
			TEXT("Like dragging a body by a handle, not a full-body force model.")),
		Make(TEXT("Spiral follows the dominant target"),
			TEXT("Secondary seeds hold and commit, but do not get their own spiral.")),
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

#undef LOCTEXT_NAMESPACE
