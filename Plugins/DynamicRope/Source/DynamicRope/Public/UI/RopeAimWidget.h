// Copyright Epic Games, Inc. All Rights Reserved.
//
// The demo aiming HUD widget for aim ray targeting. It is shown exactly while
// URopeWielderComponent::IsAimActive(), meaning the rope is in an aim ray mode, that is any resolve
// mode other than FullSimulation, and is in a phase it can currently be thrown from. GuaranteedWrap
// is restricted to the Loaded phase, so in Free and elsewhere even the crosshair is hidden; the
// other modes have no phase gate.
// Normally it draws a crosshair in the centre of the screen, and while the aim ray is on a wrappable
// bone it draws a screen-projected highlight ring around that bone, with an acquisition pop and a
// pulse.
//
// It follows the same "C++ base plus Blueprint restyle" arrangement as RopePluginInfoWidget:
//  - The C++ NativePaint draws the crosshair and ring directly with no assets, so this class works
//    on its own.
//  - To restyle it with a Blueprint subclass, either override the style properties from their
//    defaults, or clear bDrawBuiltInVisuals to disable the built-in painting and place your own
//    visuals, such as images and animations, using GetAimSample() and GetTargetScreenPosition().
//    Sounds and additional effects hang off the OnAimTargetChanged and OnAimTargetLost events.
//  - Which widget class is used is decided by
//    Project Settings > Dynamic Rope > AimHudWidgetClass, and its creation and lifetime are managed
//    by URopeWielderComponent for the local player only, subject to bShowAimHudWidget.
//
// The single data source is the FRopeAimHudSample the wielder caches each tick; the widget only
// reads it and never re-runs the aiming logic. It owns only the screen projection and the pulse
// timing.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "Blueprint/UserWidget.h"
#include "Gameplay/RopeWielderComponent.h"
#include "RopeAimWidget.generated.h"

class USceneComponent;

UCLASS(Blueprintable)
class DYNAMICROPE_API URopeAimWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	URopeAimWidget(const FObjectInitializer& ObjectInitializer);

	//~ Style, overridable from a Blueprint subclass's defaults ------------------

	/** Whether to draw the built-in crosshair and ring. Turn it off when a Blueprint provides its own
	 *  visuals; the data getters and events keep working. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD")
	bool bDrawBuiltInVisuals = true;

	/** Crosshair colour. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair")
	FLinearColor CrosshairColor = FLinearColor(1.0f, 1.0f, 1.0f, 0.9f);

	/** Length of one crosshair arm (px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair", meta = (ClampMin = "1.0"))
	float CrosshairArmLength = 8.0f;

	/** Radius of the gap at the centre of the crosshair (px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair", meta = (ClampMin = "0.0"))
	float CrosshairGap = 5.0f;

	/** Crosshair line thickness (px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair", meta = (ClampMin = "0.5"))
	float CrosshairThickness = 2.0f;

	/** Colour blended into the crosshair while a target is held, as acquisition feedback. Matching the
	 *  ring's tone is recommended. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Crosshair")
	FLinearColor CrosshairTargetColor = FLinearColor(0.2f, 1.0f, 0.4f, 1.0f);

	/** Colour used for the crosshair and ring when the ray hit a wrap target that cannot be wrapped,
	 *  whether refused by the gate or carrying no bone. A ray stopped by plain level geometry, a floor
	 *  or a wall, is not shown as blocked and keeps the ordinary crosshair. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD")
	FLinearColor BlockedColor = FLinearColor(1.0f, 0.2f, 0.15f, 0.9f);

	/** Highlight ring colour. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring")
	FLinearColor RingColor = FLinearColor(0.2f, 1.0f, 0.4f, 0.9f);

	/** Ring line thickness (px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.5"))
	float RingThickness = 2.0f;

	/** Number of ring segments, which is how closely it approximates a circle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "8", ClampMax = "64"))
	int32 RingSegments = 32;

	/** Multiplier from the target's world radius to the ring radius. Slightly above 1 leaves a little
	 *  room around the bone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.5"))
	float RingRadiusScale = 1.15f;

	/** Minimum ring radius on screen (px), which stops the ring collapsing into a dot on a distant
	 *  target. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "1.0"))
	float RingMinScreenRadius = 18.0f;

	/** Duration of the acquisition pop (s), over which the ring starts oversized and contracts to its
	 *  proper size. 0 disables the pop. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.0", Units = "s"))
	float AcquirePopDuration = 0.15f;

	/** Pulse period while a target is held (s). 0 disables the pulse. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.0", Units = "s"))
	float PulsePeriod = 1.2f;

	/** Pulse amplitude as a fraction of the radius, so 0.06 means plus or minus 6 percent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim HUD|Ring", meta = (ClampMin = "0.0", ClampMax = "0.5"))
	float PulseAmplitude = 0.06f;

	//~ Data for Blueprint and game code ----------------------------------------

	/** The aiming sample the wielder cached this tick, covering whether a target exists and its bone,
	 *  world position and radius. An empty sample when there is no wielder. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	FRopeAimHudSample GetAimSample() const;

	/** The wielder on the local pawn this widget is attached to, resolved during NativeConstruct. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	URopeWielderComponent* GetWielder() const { return Wielder.Get(); }

	/**
	 * This frame's target position and radius in screen space, meaning viewport widget space, for a
	 * Blueprint providing its own visuals.
	 * @return true when a target exists and projects on screen.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	bool GetTargetScreenPosition(FVector2D& OutPosition, float& OutRadius) const;

	/** The projected reticle position, taken from the actual ray's hit, or the end of the ray, with
	 *  AimRayOriginMode applied. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	bool GetAimScreenPosition(FVector2D& OutPosition) const;

	//~ Blueprint effect hooks for sounds and extra effects, relaying the wielder's delegates as widget
	//~ events.
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Aim HUD")
	void OnAimTargetChanged(USceneComponent* Mesh, FName Bone);

	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Aim HUD")
	void OnAimTargetLost();

protected:
	//~ UUserWidget
	virtual bool Initialize() override;
	virtual void NativeConstruct() override;
	virtual void NativeDestruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;
	virtual int32 NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
		bool bParentEnabled) const override;

	/** Relays the wielder's delegates to the Blueprint events, and resets the acquisition pop timer. */
	UFUNCTION()
	void HandleAimTargetChanged(USceneComponent* Mesh, FName Bone);

	UFUNCTION()
	void HandleAimTargetLost();

private:
	/** Finds the wielder on the owning pawn, retried from the tick while invalid so that a replaced
	 *  pawn or late possession is handled. */
	void ResolveWielder();

	/** Whether the aiming HUD should be drawn: a valid wielder and IsAimActive, meaning an aim ray mode
	 *  in a phase that can be thrown from. */
	bool IsAimHudActive() const;

	TWeakObjectPtr<URopeWielderComponent> Wielder;

	//~ Frame state cached by NativeTick and read by the const NativePaint ---------
	bool bHasScreenAim = false;
	FVector2D AimScreenPos = FVector2D::ZeroVector;
	bool bHasScreenTarget = false;
	// Whether this frame's on-screen target is a wrap target that cannot be wrapped, which is drawn in
	// the blocked colour. Only meaningful while bHasScreenTarget is set.
	bool bScreenTargetBlocked = false;
	FVector2D TargetScreenPos = FVector2D::ZeroVector;
	float TargetScreenRadius = 0.0f;
	float TimeSinceAcquire = 0.0f;
	float PulseTime = 0.0f;
};
