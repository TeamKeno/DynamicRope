// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The arc gauge showing the rope's length while it is being reeled. The question this UI answers is
// "why is the rope changing length with no visible input": on camera, a reel-in or reel-out reads as a
// bug unless something on screen says the length is being driven. So the gauge shows the current
// length as a stroke filling a vertical arc — bowed outward like a parenthesis, sitting beside the
// character — between MinRopeLength (empty, bottom) and RopeLength (full, top). Reeling in visibly
// drains it downward, reeling out refills it upward, and a chevron train marches along the fill edge
// in the direction it is moving.
//
// Readability against arbitrary backgrounds is built in rather than left to styling: the arc sits on
// a dark backing stroke, the fill runs a colour gradient from the empty colour to the full colour so
// it never reads as a plain white line, and every chevron is drawn twice — a dark outline pass under
// a bright core — so the direction stays legible over sky, foliage or snow.
//
// The widget itself is always "on"; appearing and disappearing belong to its owner. It is designed to
// live inside a URopeReelGaugeComponent, which resolves the rope, drives the fade (SetRenderOpacity)
// and faces the camera. The widget only turns a rope into gauge geometry.
//
// It follows the same "C++ base plus Blueprint restyle" arrangement as RopePullGaugeWidget:
//  - NativePaint draws it directly with no assets, so the default class is enough.
//  - To restyle it with a Blueprint subclass, either override the style properties, or clear
//    bDrawBuiltInVisuals and build your own visuals from GetLengthFraction() and GetReelDirection();
//    OnReelDirectionChanged fires on in/out/idle transitions for sounds and animations.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "Blueprint/UserWidget.h"
#include "RopeReelGaugeWidget.generated.h"

class URopeComponent;

UCLASS(Blueprintable)
class DYNAMICROPE_API URopeReelGaugeWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	URopeReelGaugeWidget(const FObjectInitializer& ObjectInitializer);

	//~ Data ---------------------------------------------------------------------

	/** Sets the rope this widget reads. The owning URopeReelGaugeComponent wires this every tick, so a
	 *  rope swapped at runtime is picked up without any extra wiring. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Reel Gauge")
	void SetRope(URopeComponent* InRope);

	/** Snaps the displayed fill to the next measurement instead of animating to it. The owning
	 *  component calls this when a preset rewrites the length domain in place. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Reel Gauge")
	void SnapDisplay() { bDisplaySeeded = false; }

	/** Where the current length sits between MinRopeLength (0) and RopeLength (1). */
	UFUNCTION(BlueprintPure, Category = "Rope|Reel Gauge")
	float GetLengthFraction() const;

	/** Which way the reel is being driven: 1 reeling in (gauge draining), -1 reeling out (gauge
	 *  filling), 0 idle. The sign of the rope's reel rate; a direct SetRopeLength shows as 0 while the
	 *  gauge still moves. */
	UFUNCTION(BlueprintPure, Category = "Rope|Reel Gauge")
	int32 GetReelDirection() const;

	/** The pure fill mapping, exposed for unit tests and Blueprint math: (Current - Min) / (Max - Min),
	 *  clamped to [0, 1]. A degenerate range (Max <= Min) reads as full. */
	UFUNCTION(BlueprintPure, Category = "Rope|Reel Gauge")
	static float ComputeLengthFraction(float CurrentLength, float MinLength, float MaxLength);

	//~ Events, where Blueprint attaches sounds and animations --------------------

	/** Fired when the reel direction changes between in (1), out (-1) and idle (0). */
	UFUNCTION(BlueprintImplementableEvent, Category = "Rope|Reel Gauge")
	void OnReelDirectionChanged(int32 NewDirection);

	//~ Style: arc ---------------------------------------------------------------

	/** Whether to draw the built-in visuals. Turn it off when a Blueprint provides its own; the getters
	 *  and events keep working. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	bool bDrawBuiltInVisuals = true;

	/** Total angular sweep of the arc (deg). Larger values curve the gauge more strongly; the radius is
	 *  fitted so the arc always fills the widget's height. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "20.0", ClampMax = "180.0", Units = "deg"))
	float ArcSweepDeg = 100.0f;

	/** Stroke thickness of the gauge arc (px). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "1.0"))
	float ArcThickness = 7.0f;

	/** Inset from the widget's edges to the arc's bounds (px), which is also the room the chevrons use
	 *  to overhang the stroke. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "0.0"))
	float ArcPadding = 12.0f;

	/** Number of segments approximating the full arc. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "8", ClampMax = "128"))
	int32 Segments = 48;

	//~ Style: colour ------------------------------------------------------------

	/** The dark stroke under everything, slightly wider than the arc, which keeps the gauge readable
	 *  over bright backgrounds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	FLinearColor BackingColor = FLinearColor(0.0f, 0.0f, 0.0f, 0.45f);

	/** Colour of the empty track behind the fill. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	FLinearColor TrackColor = FLinearColor(1.0f, 1.0f, 1.0f, 0.15f);

	/** Fill colour at the empty (bottom) end of the arc. The fill is a gradient from here to
	 *  FillFullColor along the arc, so the gauge reads as a gauge and not a plain line. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	FLinearColor FillEmptyColor = FLinearColor(1.0f, 0.55f, 0.1f, 0.9f);

	/** Fill colour at the full (top) end of the arc. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	FLinearColor FillFullColor = FLinearColor(0.2f, 0.85f, 1.0f, 0.95f);

	/** Chevron colour while reeling in. The default matches the pull gauge's armed amber, so the two
	 *  rope UIs read as one family. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	FLinearColor ReelInColor = FLinearColor(1.0f, 0.72f, 0.15f, 1.0f);

	/** Chevron colour while reeling out. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	FLinearColor ReelOutColor = FLinearColor(0.3f, 0.9f, 1.0f, 1.0f);

	/** Colour of the outline pass under each chevron. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	FLinearColor ChevronOutlineColor = FLinearColor(0.0f, 0.0f, 0.0f, 0.6f);

	//~ Style: chevrons ----------------------------------------------------------

	/** Number of chevrons in the marching train. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "1", ClampMax = "5"))
	int32 ChevronCount = 3;

	/** Chevron size (px), tip to base span. Sized well past the stroke so the direction reads at a
	 *  glance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "4.0"))
	float ChevronSize = 13.0f;

	/** Chevron line thickness (px), for the bright core; the outline pass draws wider by itself. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "0.5"))
	float ChevronThickness = 2.5f;

	/** How fast the chevron train marches along the arc toward the fill edge, in cycles per second.
	 *  The motion is what makes the direction visible in peripheral vision; 0 freezes the train. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "0.0"))
	float ChevronMarchSpeed = 1.2f;

	/** Interpolation speed of the displayed fill (1/s), which absorbs the frame-to-frame wobble of a
	 *  GPU-mirrored length. 0 disables the smoothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "0.0"))
	float FillInterpSpeed = 12.0f;

protected:
	//~ UUserWidget
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;
	virtual int32 NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;

private:
	/** The fitted arc: a circle centre and radius whose [-HalfSweep, +HalfSweep] slice fills the
	 *  widget's padded bounds, bowing to the right. Angles are measured from the +X axis, so with
	 *  Slate's Y running down, +HalfSweep is the bottom (empty) end and -HalfSweep the top (full)
	 *  end. */
	struct FArcGeometry
	{
		FVector2D Center = FVector2D::ZeroVector;
		float Radius = 0.0f;
		float HalfSweepRad = 0.0f;
		bool IsValid() const { return Radius > 0.0f && HalfSweepRad > 0.0f; }
	};

	/** Fits the arc into the widget's local size; invalid when the widget is too small to hold it. */
	FArcGeometry BuildArcGeometry(const FVector2D& LocalSize) const;

	/** The fill-edge angle for a fraction: +HalfSweep at empty, -HalfSweep at full. */
	static float FractionToAngle(const FArcGeometry& Arc, float Fraction);

	/** Draws one stroke of the arc between two angles as a polyline, single colour. */
	void DrawArcStroke(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
		const FArcGeometry& Arc, float StartAngleRad, float EndAngleRad, const FLinearColor& Color, float Thickness) const;

	/** Draws the fill between two angles segment by segment, each coloured by its own position along
	 *  the full arc, giving the empty-to-full gradient. */
	void DrawGradientFill(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
		const FArcGeometry& Arc, float StartAngleRad, float EndAngleRad) const;

	/** Draws one chevron on the arc at AngleRad, pointing along the arc in the direction the fill edge
	 *  is moving (bTowardEmpty follows the reel), as an outline pass and a core pass. */
	void DrawChevron(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
		const FArcGeometry& Arc, float AngleRad, bool bTowardEmpty, const FLinearColor& Color, float AlphaScale) const;

	TWeakObjectPtr<URopeComponent> Rope;

	/** The interpolated fill used for display. The real value belongs to the rope; this is purely
	 *  presentation state. */
	float DisplayFraction = 0.0f;

	/** Whether DisplayFraction has been seeded from a live rope, so the first frame snaps instead of
	 *  sweeping in from zero. */
	bool bDisplaySeeded = false;

	/** The last direction reported through OnReelDirectionChanged. */
	int32 LastDirection = 0;

	/** Phase of the chevron march, in [0, 1); advanced only while reeling. */
	float ChevronPhase = 0.0f;
};
