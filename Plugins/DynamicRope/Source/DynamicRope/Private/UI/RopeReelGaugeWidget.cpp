// Copyright 2026 TeamKeno. All Rights Reserved.

#include "UI/RopeReelGaugeWidget.h"
#include "RopeComponent.h"

#include "Rendering/DrawElements.h"

URopeReelGaugeWidget::URopeReelGaugeWidget(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A pure display widget that never consumes input.
	SetVisibility(ESlateVisibility::HitTestInvisible);
}

void URopeReelGaugeWidget::SetRope(URopeComponent* InRope)
{
	if (Rope.Get() != InRope)
	{
		Rope = InRope;
		// A different rope means a different length domain, so the display snaps to it rather than
		// sweeping across the gauge.
		bDisplaySeeded = false;
	}
}

float URopeReelGaugeWidget::GetLengthFraction() const
{
	const URopeComponent* R = Rope.Get();
	return R ? ComputeLengthFraction(R->GetCurrentRopeLength(), R->MinRopeLength, R->RopeLength) : 0.0f;
}

int32 URopeReelGaugeWidget::GetReelDirection() const
{
	const URopeComponent* R = Rope.Get();
	const float Rate = R ? R->GetReelRate() : 0.0f;
	if (Rate > KINDA_SMALL_NUMBER)
	{
		return 1;
	}
	return (Rate < -KINDA_SMALL_NUMBER) ? -1 : 0;
}

float URopeReelGaugeWidget::ComputeLengthFraction(float CurrentLength, float MinLength, float MaxLength)
{
	// A degenerate range means the rope cannot reel at all, and a full gauge is the honest reading of
	// "at its one possible length".
	if (MaxLength - MinLength <= KINDA_SMALL_NUMBER)
	{
		return 1.0f;
	}
	return FMath::Clamp((CurrentLength - MinLength) / (MaxLength - MinLength), 0.0f, 1.0f);
}

void URopeReelGaugeWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);

	const float Target = GetLengthFraction();
	if (!bDisplaySeeded)
	{
		// The first live frame, and a rope swap, seed the display from the measurement so the gauge
		// does not animate in from an arbitrary zero.
		DisplayFraction = Target;
		bDisplaySeeded = Rope.IsValid();
	}
	else
	{
		DisplayFraction = (FillInterpSpeed > 0.0f)
			? FMath::FInterpTo(DisplayFraction, Target, InDeltaTime, FillInterpSpeed)
			: Target;
	}

	const int32 Direction = GetReelDirection();
	if (Direction != LastDirection)
	{
		LastDirection = Direction;
		OnReelDirectionChanged(Direction);
	}

	// The march only runs while reeling; the phase freezing while idle keeps the train from having
	// visibly "jumped" between two reels.
	if (Direction != 0 && ChevronMarchSpeed > 0.0f)
	{
		ChevronPhase = FMath::Frac(ChevronPhase + InDeltaTime * ChevronMarchSpeed);
	}
}

URopeReelGaugeWidget::FArcGeometry URopeReelGaugeWidget::BuildArcGeometry(const FVector2D& LocalSize) const
{
	// The arc bows to the right like a parenthesis: its circle centre sits to the left of the widget,
	// and the [-HalfSweep, +HalfSweep] slice around the +X axis is what shows. The radius is fitted to
	// the padded height, then the centre is placed so the arc's rightmost point touches the right
	// bound.
	FArcGeometry Arc;
	const float PaddedWidth = static_cast<float>(LocalSize.X) - 2.0f * ArcPadding;
	const float PaddedHeight = static_cast<float>(LocalSize.Y) - 2.0f * ArcPadding;
	if (PaddedWidth <= 0.0f || PaddedHeight <= 0.0f)
	{
		return Arc;
	}

	Arc.HalfSweepRad = FMath::DegreesToRadians(FMath::Clamp(ArcSweepDeg, 20.0f, 180.0f) * 0.5f);
	const float SinHalf = FMath::Sin(Arc.HalfSweepRad);
	if (SinHalf <= KINDA_SMALL_NUMBER)
	{
		return Arc;
	}
	Arc.Radius = PaddedHeight * 0.5f / SinHalf;
	Arc.Center = FVector2D(
		(static_cast<float>(LocalSize.X) - ArcPadding) - Arc.Radius,
		static_cast<float>(LocalSize.Y) * 0.5f);
	return Arc;
}

float URopeReelGaugeWidget::FractionToAngle(const FArcGeometry& Arc, float Fraction)
{
	// With Slate's Y running down, +HalfSweep is the bottom of the arc. Empty sits at the bottom and
	// the fill climbs, which is how a fuel-style gauge is conventionally read.
	return Arc.HalfSweepRad * (1.0f - 2.0f * FMath::Clamp(Fraction, 0.0f, 1.0f));
}

void URopeReelGaugeWidget::DrawArcStroke(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
	const FArcGeometry& Arc, float StartAngleRad, float EndAngleRad, const FLinearColor& Color, float Thickness) const
{
	const float Span = EndAngleRad - StartAngleRad;
	if (FMath::Abs(Span) <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const int32 SegmentCount = FMath::Max(1,
		FMath::CeilToInt(Segments * FMath::Abs(Span) / (2.0f * Arc.HalfSweepRad)));

	TArray<FVector2D> Points;
	Points.Reserve(SegmentCount + 1);
	for (int32 i = 0; i <= SegmentCount; ++i)
	{
		const float Angle = StartAngleRad + Span * (static_cast<float>(i) / static_cast<float>(SegmentCount));
		Points.Add(Arc.Center + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Arc.Radius);
	}
	FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geometry.ToPaintGeometry(), Points,
		ESlateDrawEffect::None, Color, /*bAntialias=*/true, Thickness);
}

void URopeReelGaugeWidget::DrawGradientFill(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
	const FArcGeometry& Arc, float StartAngleRad, float EndAngleRad) const
{
	// MakeLines is single-colour, so the gradient is one short polyline per segment, each coloured by
	// its own position along the full arc. The colour is anchored to position rather than to the fill
	// amount, so draining the gauge peels colours off the top instead of recolouring the whole stroke.
	const float Span = EndAngleRad - StartAngleRad;
	if (FMath::Abs(Span) <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const int32 SegmentCount = FMath::Max(1,
		FMath::CeilToInt(Segments * FMath::Abs(Span) / (2.0f * Arc.HalfSweepRad)));

	for (int32 i = 0; i < SegmentCount; ++i)
	{
		const float A0 = StartAngleRad + Span * (static_cast<float>(i) / static_cast<float>(SegmentCount));
		const float A1 = StartAngleRad + Span * (static_cast<float>(i + 1) / static_cast<float>(SegmentCount));

		TArray<FVector2D> Points;
		Points.Reserve(2);
		Points.Add(Arc.Center + FVector2D(FMath::Cos(A0), FMath::Sin(A0)) * Arc.Radius);
		Points.Add(Arc.Center + FVector2D(FMath::Cos(A1), FMath::Sin(A1)) * Arc.Radius);

		// The segment midpoint's position along the arc, 0 at the empty bottom to 1 at the full top.
		const float MidAngle = (A0 + A1) * 0.5f;
		const float PositionT = FMath::Clamp(
			(Arc.HalfSweepRad - MidAngle) / (2.0f * Arc.HalfSweepRad), 0.0f, 1.0f);
		const FLinearColor SegmentColor = FMath::Lerp(FillEmptyColor, FillFullColor, PositionT);

		FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geometry.ToPaintGeometry(), Points,
			ESlateDrawEffect::None, SegmentColor, /*bAntialias=*/true, ArcThickness);
	}
}

void URopeReelGaugeWidget::DrawChevron(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
	const FArcGeometry& Arc, float AngleRad, bool bTowardEmpty, const FLinearColor& Color, float AlphaScale) const
{
	const FVector2D Radial(FMath::Cos(AngleRad), FMath::Sin(AngleRad));
	const FVector2D Point = Arc.Center + Radial * Arc.Radius;
	// The tangent of increasing angle runs toward the bottom (empty) end; the chevron points the way
	// the fill edge is moving, so reeling in points it down the arc and reeling out up.
	const FVector2D TangentDown(-Radial.Y, Radial.X);
	const FVector2D Along = bTowardEmpty ? TangentDown : -TangentDown;

	TArray<FVector2D> Points;
	Points.Reserve(3);
	Points.Add(Point - Along * (ChevronSize * 0.35f) - Radial * (ChevronSize * 0.5f));
	Points.Add(Point + Along * (ChevronSize * 0.5f));
	Points.Add(Point - Along * (ChevronSize * 0.35f) + Radial * (ChevronSize * 0.5f));

	// Two passes: a wider dark outline under a bright core, so the chevron stays legible over any
	// background the camera puts behind it.
	FLinearColor Outline = ChevronOutlineColor;
	Outline.A *= AlphaScale;
	FLinearColor Core = Color;
	Core.A *= AlphaScale;
	FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geometry.ToPaintGeometry(), Points,
		ESlateDrawEffect::None, Outline, /*bAntialias=*/true, ChevronThickness + 2.5f);
	FSlateDrawElement::MakeLines(OutDrawElements, LayerId + 1, Geometry.ToPaintGeometry(), Points,
		ESlateDrawEffect::None, Core, /*bAntialias=*/true, ChevronThickness);
}

int32 URopeReelGaugeWidget::NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
	FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const int32 Result = Super::NativePaint(Args, AllottedGeometry, MyCullingRect, OutDrawElements, LayerId, InWidgetStyle, bParentEnabled);

	if (!bDrawBuiltInVisuals)
	{
		return Result;
	}

	// Appearing and disappearing are the owning component's job (render opacity), so the gauge itself
	// always draws.
	const FArcGeometry Arc = BuildArcGeometry(AllottedGeometry.GetLocalSize());
	if (!Arc.IsValid())
	{
		return Result;
	}

	const float TopAngle = -Arc.HalfSweepRad;
	const float BottomAngle = +Arc.HalfSweepRad;
	const float Shown = FMath::Clamp(DisplayFraction, 0.0f, 1.0f);
	const float EdgeAngle = FractionToAngle(Arc, Shown);

	// The dark backing, wider than the stroke, then the track, then the gradient fill from the bottom
	// up to the displayed fraction.
	DrawArcStroke(AllottedGeometry, OutDrawElements, LayerId + 1, Arc, TopAngle, BottomAngle,
		BackingColor, ArcThickness + 4.0f);
	DrawArcStroke(AllottedGeometry, OutDrawElements, LayerId + 2, Arc, TopAngle, BottomAngle,
		TrackColor, ArcThickness);
	DrawGradientFill(AllottedGeometry, OutDrawElements, LayerId + 3, Arc, BottomAngle, EdgeAngle);

	// The chevron train marches along the arc into the fill edge, pointing the way the edge is moving.
	// A direct SetRopeLength has no rate, so the gauge moves with no chevrons, which is the honest
	// reading of "length set, not being reeled".
	const int32 Direction = LastDirection;
	if (Direction != 0 && ChevronCount > 0)
	{
		const bool bTowardEmpty = Direction > 0;
		const FLinearColor ChevronColor = bTowardEmpty ? ReelInColor : ReelOutColor;
		const float SpacingRad = (ChevronSize * 1.2f) / Arc.Radius;
		const float TrailRad = SpacingRad * static_cast<float>(ChevronCount);

		for (int32 i = 0; i < ChevronCount; ++i)
		{
			// Each chevron runs its own loop from the tail of the trail into the edge; the sine ramp
			// fades it in at the tail and out at the edge so the march has no popping.
			const float U = FMath::Frac(ChevronPhase + static_cast<float>(i) / static_cast<float>(ChevronCount));
			const float Behind = (1.0f - U) * TrailRad;
			// "Behind the edge" is against the direction the edge is moving.
			const float Angle = EdgeAngle + (bTowardEmpty ? -Behind : +Behind);
			if (Angle < TopAngle || Angle > BottomAngle)
			{
				continue;
			}
			const float AlphaScale = FMath::Sin(U * PI);
			DrawChevron(AllottedGeometry, OutDrawElements, LayerId + 4, Arc, Angle, bTowardEmpty,
				ChevronColor, AlphaScale);
		}
	}

	return FMath::Max(Result, LayerId + 5);
}
