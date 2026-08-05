// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The gauge widget showing the pull arming and engagement state. The question this UI has to answer
// is not "is the toggle on" but "I pressed it, so why is nothing pulling yet". It therefore draws
// progress towards the engage threshold as a ring, from
// URopeWielderComponent::GetPullEngageProgress, rather than an on/off lamp: an unfilled ring reads at
// a glance as "the rope is not taut yet".
//
// There are three states and each looks different:
//   Disarmed  draws nothing.
//   Armed     draws a background ring plus an arc filled to the current progress. As progress rises,
//             the colour blends from the armed colour towards the engaged colour.
//   Engaged   draws a full ring plus a single pop at the moment of engagement, expanding and fading
//             over EngagePopTime.
//
// It follows the same "C++ base plus Blueprint restyle" arrangement as RopeAimWidget:
//  - NativePaint draws it directly with no assets, so putting this class on screen is enough.
//  - To restyle it with a Blueprint subclass, either override the style properties, or clear
//    bDrawBuiltInVisuals to disable the built-in painting and build your own visuals from
//    IsPullArmed(), IsPullEngaged() and GetProgress(). Effects for state transitions, such as sounds
//    and animations, hang off the OnPullArmedStateChanged and OnPullEngagedStateChanged events.
//
// The widget finds the wielder on its owning pawn by itself, following the same convention as
// RopeAimWidget, so adding it to the HUD is all the wiring needed. Use SetWielder() when it must be
// wired explicitly.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "Blueprint/UserWidget.h"
#include "RopePullGaugeWidget.generated.h"

class URopeWielderComponent;

UCLASS(Blueprintable)
class DYNAMICROPE_API URopePullGaugeWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	URopePullGaugeWidget(const FObjectInitializer& ObjectInitializer);

	//~ Data ---------------------------------------------------------------------

	/** Sets the wielder this widget reads explicitly. Leave it unset to find one on the owning pawn. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Pull HUD")
	void SetWielder(URopeWielderComponent* InWielder);

	/** Whether pull is armed, that is armed or engaged. */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull HUD")
	bool IsPullArmed() const;

	/** Whether pull has engaged. */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull HUD")
	bool IsPullEngaged() const;

	/** Progress towards the engage threshold, from 0 to 1. */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull HUD")
	float GetProgress() const;

	//~ Events, where Blueprint attaches sounds and animations --------------------

	/** Fired when the arming toggle changes. */
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Pull HUD")
	void OnPullArmedStateChanged(bool bArmed);

	/** Fired when the engage latch changes; false means the wrap released and pull rearmed. */
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Pull HUD")
	void OnPullEngagedStateChanged(bool bEngaged, float Tension);

	//~ Style --------------------------------------------------------------------

	/** Whether to draw the built-in visuals. Turn it off when a Blueprint provides its own; the getters
	 *  and events keep working. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	bool bDrawBuiltInVisuals = true;

	/** The gauge centre, as a fraction of the widget's local space from 0 to 1. The default sits just
	 *  below the centre of the screen so it does not overlap the crosshair. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	FVector2D CenterAnchor = FVector2D(0.5f, 0.62f);

	/** Ring radius (px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "4.0"))
	float Radius = 26.0f;

	/** Ring line thickness (px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "0.5"))
	float Thickness = 3.0f;

	/** Number of ring segments, which is how closely it approximates a circle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "8", ClampMax = "128"))
	int32 Segments = 48;

	/** Colour of the unfilled background ring, visible only while armed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	FLinearColor TrackColor = FLinearColor(1.0f, 1.0f, 1.0f, 0.18f);

	/** Colour at zero progress, meaning armed but still slack. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	FLinearColor ArmedColor = FLinearColor(1.0f, 0.72f, 0.15f, 0.95f);

	/** Colour at full progress and once engaged. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD")
	FLinearColor EngagedColor = FLinearColor(0.2f, 1.0f, 0.45f, 1.0f);

	/** Duration of the pop at the moment of engagement (s). 0 disables the pop. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "0.0", Units = "s"))
	float EngagePopTime = 0.25f;

	/** Radius multiplier at the peak of the pop. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "1.0"))
	float EngagePopScale = 1.35f;

	/** Interpolation speed of the displayed progress (1/s), which stops the ring flickering on frames
	 *  where the tension wobbles. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull HUD", meta = (ClampMin = "0.0"))
	float ProgressInterpSpeed = 12.0f;

protected:
	//~ UUserWidget
	virtual void NativeConstruct() override;
	virtual void NativeDestruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;
	virtual int32 NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;

private:
	/** Finds the wielder on the owning pawn, but only while the cached one is invalid, so it costs
	 *  nothing in the normal case. */
	void ResolveWielder();

	/** Subscribes to and unsubscribes from the wielder's events, rebinding when a replaced pawn changes
	 *  which wielder is used. */
	void BindWielder(URopeWielderComponent* NewWielder);

	UFUNCTION()
	void HandlePullArmedChanged(bool bArmed);

	UFUNCTION()
	void HandlePullEngagedChanged(bool bEngaged, float Tension);

	/** Draws a ring, or an arc of one, approximated by line segments. Alpha01 is the fraction to draw,
	 *  where 1 is a complete circle. */
	void DrawArc(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
		const FVector2D& Center, float InRadius, float Alpha01, const FLinearColor& Color) const;

	TWeakObjectPtr<URopeWielderComponent> Wielder;

	/** The interpolated progress used for display. The real value belongs to the wielder; this is
	 *  purely presentation state. */
	float DisplayProgress = 0.0f;

	/** Time since engagement, where exceeding EngagePopTime ends the pop. */
	float TimeSinceEngage = BIG_NUMBER;
};
