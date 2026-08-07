// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Gameplay/RopeReelGaugeComponent.h"
#include "Gameplay/RopeWielderComponent.h"
#include "RopeComponent.h"
#include "UI/RopeReelGaugeWidget.h"

#include "Camera/PlayerCameraManager.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"

URopeReelGaugeComponent::URopeReelGaugeComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// World space so any camera — view-target blends, cinematics — can film it; a screen-space widget
	// only exists for the local player's viewport.
	SetWidgetSpace(EWidgetSpace::World);
	SetDrawSize(FVector2D(96.0f, 256.0f));
	// DrawSize is in slate units and world space maps them 1:1 to centimetres, so the scale brings the
	// arc to roughly 19 x 51 cm beside the character. The default offset sits it off the right
	// shoulder (+Y), chest height, where the outward bow of the arc frames the character.
	SetRelativeScale3D(FVector(0.2f));
	SetRelativeLocation(FVector(0.0f, 60.0f, 30.0f));
	SetTwoSided(false);

	// A pure indicator: no collision, no shadow, no input.
	SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetGenerateOverlapEvents(false);
	CastShadow = false;
	SetReceivesDecals(false);
	bWindowFocusable = false;

	WidgetClass = URopeReelGaugeWidget::StaticClass();
}

URopeComponent* URopeReelGaugeComponent::ResolveRope()
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}
	if (!CachedWielder.IsValid())
	{
		CachedWielder = Owner->FindComponentByClass<URopeWielderComponent>();
	}
	if (URopeWielderComponent* Wielder = CachedWielder.Get())
	{
		return Wielder->GetRope();
	}
	return Owner->FindComponentByClass<URopeComponent>();
}

void URopeReelGaugeComponent::BindRope(URopeComponent* NewRope)
{
	if (BoundRope.Get() == NewRope)
	{
		return;
	}
	if (URopeComponent* Old = BoundRope.Get())
	{
		Old->OnPresetApplied.RemoveDynamic(this, &URopeReelGaugeComponent::HandlePresetApplied);
	}
	BoundRope = NewRope;
	if (NewRope)
	{
		NewRope->OnPresetApplied.AddDynamic(this, &URopeReelGaugeComponent::HandlePresetApplied);
	}
}

void URopeReelGaugeComponent::HandlePresetApplied(const URopePreset* Preset)
{
	// A preset rewrites the length and its domain in one go — a settings change, not a reel — so the
	// jump must not light the gauge. The baseline reseeds (the next tick only records it) and the
	// widget snaps its display to the new domain instead of animating across it.
	bHasPrevLength = false;
	bPendingDisplaySnap = true;
}

void URopeReelGaugeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Ensures no dead handler is left on the rope's delegate if the gauge disappears first.
	BindRope(nullptr);
	Super::EndPlay(EndPlayReason);
}

void URopeReelGaugeComponent::FaceCamera()
{
	// The camera manager's location is the rendered camera through view-target blends, so this is the
	// same camera a recording sees.
	const UWorld* World = GetWorld();
	const APlayerController* PC = World ? World->GetFirstPlayerController() : nullptr;
	if (!PC || !PC->PlayerCameraManager)
	{
		return;
	}
	const FVector ToCamera = PC->PlayerCameraManager->GetCameraLocation() - GetComponentLocation();
	if (!ToCamera.IsNearlyZero())
	{
		// A widget component's face is its +X plane, so pointing +X at the camera fronts the bar.
		SetWorldRotation(ToCamera.Rotation());
	}
}

void URopeReelGaugeComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	URopeComponent* Rope = ResolveRope();
	BindRope(Rope);

	// The widget is rewired every tick so a rope or pawn swapped at runtime is picked up; SetRope is a
	// no-op when unchanged.
	if (URopeReelGaugeWidget* Gauge = Cast<URopeReelGaugeWidget>(GetWidget()))
	{
		Gauge->SetRope(Rope);
		if (bPendingDisplaySnap)
		{
			Gauge->SnapDisplay();
		}
	}
	bPendingDisplaySnap = false;

	// Activity is the observed length change, not the reel input — see the class comment. The seeding
	// frame only records the baseline.
	bool bActive = false;
	if (Rope)
	{
		const float CurrentLength = Rope->GetCurrentRopeLength();
		if (bHasPrevLength)
		{
			bActive = FMath::Abs(CurrentLength - PrevLength) > ActivityEpsilon;
		}
		PrevLength = CurrentLength;
		bHasPrevLength = true;
	}
	else
	{
		bHasPrevLength = false;
	}

	DisplayAlpha = Fade.Update(bActive, DeltaTime, FadeOutTime);

	// Fully faded, the component is hidden outright so it costs no rendering; the widget object stays
	// alive for the next reel.
	const bool bShow = DisplayAlpha > KINDA_SMALL_NUMBER;
	SetVisibility(bShow);
	if (UUserWidget* GaugeWidget = GetWidget())
	{
		GaugeWidget->SetRenderOpacity(DisplayAlpha);
	}
	if (bShow && bFaceCamera)
	{
		FaceCamera();
	}
}
