// Copyright 2026 TeamKeno. All Rights Reserved.

#include "UI/RopeAimWidget.h"

#include "Blueprint/WidgetLayoutLibrary.h"
#include "Blueprint/WidgetTree.h"
#include "Components/CanvasPanel.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Rendering/DrawElements.h"

URopeAimWidget::URopeAimWidget(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool URopeAimWidget::Initialize()
{
	const bool bResult = Super::Initialize();

	// For use from C++ alone, meaning this class is constructed directly with no widget Blueprint: with no root widget
	// the Slate tree is never built and NativePaint is never called, so an empty canvas is created as the root to
	// establish full-screen geometry.
	// A widget Blueprint subclass has its own root and does not take this branch.
	if (bResult && WidgetTree && !WidgetTree->RootWidget)
	{
		WidgetTree->RootWidget = WidgetTree->ConstructWidget<UCanvasPanel>(UCanvasPanel::StaticClass(), TEXT("AimRootCanvas"));
	}
	return bResult;
}

void URopeAimWidget::NativeConstruct()
{
	Super::NativeConstruct();

	// The aim HUD is for display alone and never intercepts game input or the cursor.
	SetVisibility(ESlateVisibility::HitTestInvisible);

	ResolveWielder();
	if (URopeWielderComponent* W = Wielder.Get())
	{
		W->OnAimTargetChanged.AddDynamic(this, &URopeAimWidget::HandleAimTargetChanged);
		W->OnAimTargetLost.AddDynamic(this, &URopeAimWidget::HandleAimTargetLost);
	}
}

void URopeAimWidget::NativeDestruct()
{
	if (URopeWielderComponent* W = Wielder.Get())
	{
		W->OnAimTargetChanged.RemoveDynamic(this, &URopeAimWidget::HandleAimTargetChanged);
		W->OnAimTargetLost.RemoveDynamic(this, &URopeAimWidget::HandleAimTargetLost);
	}
	Super::NativeDestruct();
}

void URopeAimWidget::ResolveWielder()
{
	if (Wielder.IsValid())
	{
		return;
	}
	if (const APawn* Pawn = GetOwningPlayerPawn())
	{
		Wielder = Pawn->FindComponentByClass<URopeWielderComponent>();
	}
}

bool URopeAimWidget::IsAimHudActive() const
{
	// It looks at the per-frame IsAimActive rather than the mode's UsesAimRay, because GuaranteedWrap aims from Loaded
	// alone. The crosshair is drawn from this gate alone, in NativePaint, without looking at the sample, so without
	// this check an empty sample's zero-vector aim position would project to the world origin and leave the crosshair
	// stuck at the origin, meaning the centre of the screen.
	const URopeWielderComponent* W = Wielder.Get();
	return W && W->IsAimActive();
}

FRopeAimHudSample URopeAimWidget::GetAimSample() const
{
	const URopeWielderComponent* W = Wielder.Get();
	return W ? W->GetAimHudSample() : FRopeAimHudSample();
}

bool URopeAimWidget::GetTargetScreenPosition(FVector2D& OutPosition, float& OutRadius) const
{
	OutPosition = TargetScreenPos;
	OutRadius = TargetScreenRadius;
	return bHasScreenTarget;
}

bool URopeAimWidget::GetAimScreenPosition(FVector2D& OutPosition) const
{
	OutPosition = AimScreenPos;
	return bHasScreenAim;
}

void URopeAimWidget::HandleAimTargetChanged(USceneComponent* Mesh, FName Bone)
{
	// The moment a target is acquired or changed, which replays the acquisition pop from the start.
	TimeSinceAcquire = 0.0f;
	OnAimTargetChanged(Mesh, Bone);
}

void URopeAimWidget::HandleAimTargetLost()
{
	OnAimTargetLost();
}

void URopeAimWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);

	// Re-resolved in case the pawn was replaced or possession was deferred, and only when invalid, so it costs nothing in the normal case.
	ResolveWielder();

	bHasScreenAim = false;
	bHasScreenTarget = false;
	bScreenTargetBlocked = false;
	if (!IsAimHudActive())
	{
		return;
	}

	TimeSinceAcquire += InDeltaTime;
	PulseTime += InDeltaTime;

	const FRopeAimHudSample Sample = GetAimSample();
	APlayerController* PC = GetOwningPlayer();
	if (!PC)
	{
		return;
	}

	// The real aim ray's hit, or its end point where it hit nothing, is used as the position of the aim ring. It is
	// not fixed to the centre of the screen, so the UI and the ray path agree under any AimRayOriginMode, whether the
	// attached mesh, the socket, the owner or the view.
	if (UWidgetLayoutLibrary::ProjectWorldLocationToWidgetPosition(
		PC, Sample.AimWorldPos, AimScreenPos, /*bPlayerViewportRelative*/ false))
	{
		bHasScreenAim = true;
	}

	// A wrappable target, in green, or a wrap target that was refused, in red; both draw a ring. A ray merely stopped
	// by level geometry is not a target at all: it draws no ring and keeps the neutral crosshair, since colouring every
	// floor and wall red reads as the player's aim being broken.
	const bool bShowBlocked = Sample.bBlocked && Sample.bBlockedByTarget;
	if (!Sample.bHasTarget && !bShowBlocked)
	{
		return;
	}

	// The ring is a fixed-size marker centred on the crosshair: it signals that the aim is on a wrap target without
	// tracking the bone's projected position or size. It shares the crosshair's screen-centre fallback so the two
	// never separate.
	bHasScreenTarget = true;
	bScreenTargetBlocked = bShowBlocked && !Sample.bHasTarget;
	TargetScreenPos = bHasScreenAim ? AimScreenPos : MyGeometry.GetLocalSize() * 0.5;
	TargetScreenRadius = RingScreenRadius;
}

int32 URopeAimWidget::NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
	FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	LayerId = Super::NativePaint(Args, AllottedGeometry, MyCullingRect, OutDrawElements, LayerId, InWidgetStyle, bParentEnabled);
	if (!bDrawBuiltInVisuals || !IsAimHudActive())
	{
		return LayerId;
	}

	const FPaintGeometry PaintGeometry = AllottedGeometry.ToPaintGeometry();

	// The crosshair, at the hit or end point of the real ray as decided by AimRayOriginMode, falling back to the centre of the screen only when the projection fails.
	{
		const FVector2D Center = bHasScreenAim ? AimScreenPos : AllottedGeometry.GetLocalSize() * 0.5f;
	// No target, which includes a ray stopped by level geometry, uses the default colour; a wrappable target the
	// acquisition colour; and a wrap target that cannot be wrapped red.
		FLinearColor Color = CrosshairColor;
		if (bHasScreenTarget)
		{
			Color = bScreenTargetBlocked ? BlockedColor : CrosshairTargetColor;
		}
		const float In = CrosshairGap;
		const float Out = CrosshairGap + CrosshairArmLength;

		const FVector2D Dirs[4] = { FVector2D(1, 0), FVector2D(-1, 0), FVector2D(0, 1), FVector2D(0, -1) };
		for (const FVector2D& Dir : Dirs)
		{
			TArray<FVector2f> Arm;
			Arm.Add(FVector2f(Center + Dir * In));
			Arm.Add(FVector2f(Center + Dir * Out));
			FSlateDrawElement::MakeLines(OutDrawElements, LayerId + 1, PaintGeometry, Arm,
				ESlateDrawEffect::None, Color, /*bAntialias*/ true, CrosshairThickness);
		}
	}

	// The highlight ring around the target bone, with an acquisition pop, from large down to its own size, plus a sustained pulse.
	if (bHasScreenTarget)
	{
		float Radius = TargetScreenRadius;
	// The acquisition pop is measured from the moment of acquisition, in HandleAimTargetChanged, and so applies to wrappable targets alone; a blocked one gets the pulse alone.
		if (!bScreenTargetBlocked && AcquirePopDuration > KINDA_SMALL_NUMBER && TimeSinceAcquire < AcquirePopDuration)
		{
			// Contracts from 1.6 times its size down to its own size, easing out.
			const float T = TimeSinceAcquire / AcquirePopDuration;
			Radius *= FMath::Lerp(1.6f, 1.0f, 1.0f - FMath::Square(1.0f - T));
		}
		else if (PulsePeriod > KINDA_SMALL_NUMBER)
		{
			Radius *= 1.0f + PulseAmplitude * FMath::Sin(PulseTime * (2.0f * PI / PulsePeriod));
		}

		const int32 NumSegments = FMath::Clamp(RingSegments, 8, 64);
		TArray<FVector2f> Circle;
		Circle.Reserve(NumSegments + 1);
		for (int32 i = 0; i <= NumSegments; ++i)
		{
			const float Angle = (2.0f * PI) * static_cast<float>(i) / static_cast<float>(NumSegments);
			Circle.Add(FVector2f(TargetScreenPos + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius));
		}
		const FLinearColor UseRingColor = bScreenTargetBlocked ? BlockedColor : RingColor;
		FSlateDrawElement::MakeLines(OutDrawElements, LayerId + 1, PaintGeometry, Circle,
			ESlateDrawEffect::None, UseRingColor, /*bAntialias*/ true, RingThickness);
	}

	return LayerId + 1;
}
