// Copyright 2026 TeamKeno. All Rights Reserved.

#include "UI/RopePullGaugeWidget.h"
#include "Gameplay/RopeWielderComponent.h"

#include "GameFramework/Pawn.h"
#include "Rendering/DrawElements.h"

URopePullGaugeWidget::URopePullGaugeWidget(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A pure display widget that never consumes input.
	SetVisibility(ESlateVisibility::HitTestInvisible);
}

void URopePullGaugeWidget::NativeConstruct()
{
	Super::NativeConstruct();
	ResolveWielder();
}

void URopePullGaugeWidget::NativeDestruct()
{
	// Ensures no dead handler is left on the wielder's delegates if the widget disappears first.
	BindWielder(nullptr);
	Super::NativeDestruct();
}

void URopePullGaugeWidget::SetWielder(URopeWielderComponent* InWielder)
{
	BindWielder(InWielder);
}

void URopePullGaugeWidget::ResolveWielder()
{
	if (Wielder.IsValid())
	{
		return;
	}

	if (const APawn* Pawn = GetOwningPlayerPawn())
	{
		BindWielder(Pawn->FindComponentByClass<URopeWielderComponent>());
	}
}

void URopePullGaugeWidget::BindWielder(URopeWielderComponent* NewWielder)
{
	if (URopeWielderComponent* Old = Wielder.Get())
	{
		if (Old == NewWielder)
		{
			return;
		}
		Old->OnPullArmedChanged.RemoveDynamic(this, &URopePullGaugeWidget::HandlePullArmedChanged);
		Old->OnPullEngagedChanged.RemoveDynamic(this, &URopePullGaugeWidget::HandlePullEngagedChanged);
	}

	Wielder = NewWielder;

	if (NewWielder)
	{
		NewWielder->OnPullArmedChanged.AddDynamic(this, &URopePullGaugeWidget::HandlePullArmedChanged);
		NewWielder->OnPullEngagedChanged.AddDynamic(this, &URopePullGaugeWidget::HandlePullEngagedChanged);
	}
}

void URopePullGaugeWidget::HandlePullArmedChanged(bool bArmed)
{
	if (!bArmed)
	{
		// On release the display state is reset so the next arming fills again from zero.
		DisplayProgress = 0.0f;
		TimeSinceEngage = BIG_NUMBER;
	}
	OnPullArmedStateChanged(bArmed);
}

void URopePullGaugeWidget::HandlePullEngagedChanged(bool bEngaged, float Tension)
{
	if (bEngaged)
	{
		TimeSinceEngage = 0.0f;
	}
	else
	{
		// Rearming, meaning the wrap was released. Without turning the pop off, the engaged indicator would stay lit.
		TimeSinceEngage = BIG_NUMBER;
	}
	OnPullEngagedStateChanged(bEngaged, Tension);
}

bool URopePullGaugeWidget::IsPullArmed() const
{
	const URopeWielderComponent* W = Wielder.Get();
	return W && W->IsPullArmed();
}

bool URopePullGaugeWidget::IsPullEngaged() const
{
	const URopeWielderComponent* W = Wielder.Get();
	return W && W->IsPullEngaged();
}

float URopePullGaugeWidget::GetProgress() const
{
	const URopeWielderComponent* W = Wielder.Get();
	return W ? W->GetPullEngageProgress() : 0.0f;
}

void URopePullGaugeWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);

	// Re-resolved in case the pawn was replaced or possession was deferred, and only when invalid.
	ResolveWielder();

	TimeSinceEngage += InDeltaTime;

	// The tension jitters from frame to frame, so the displayed value alone follows it smoothly; the value used for decisions is untouched.
	const float Target = GetProgress();
	DisplayProgress = (ProgressInterpSpeed > 0.0f)
		? FMath::FInterpTo(DisplayProgress, Target, InDeltaTime, ProgressInterpSpeed)
		: Target;
}

void URopePullGaugeWidget::DrawArc(const FGeometry& Geometry, FSlateWindowElementList& OutDrawElements, int32 LayerId,
	const FVector2D& Center, float InRadius, float Alpha01, const FLinearColor& Color) const
{
	const float Fraction = FMath::Clamp(Alpha01, 0.0f, 1.0f);
	if (Fraction <= 0.0f || InRadius <= 0.0f)
	{
		return;
	}

	// Fills clockwise from twelve o'clock, as a gauge is conventionally read.
	const int32 SegmentCount = FMath::Max(1, FMath::CeilToInt(Segments * Fraction));
	const float TotalAngle = 2.0f * PI * Fraction;

	TArray<FVector2D> Points;
	Points.Reserve(SegmentCount + 1);
	for (int32 i = 0; i <= SegmentCount; ++i)
	{
		const float T = static_cast<float>(i) / static_cast<float>(SegmentCount);
		const float Angle = -PI * 0.5f + TotalAngle * T;
		Points.Add(Center + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * InRadius);
	}

	FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geometry.ToPaintGeometry(), Points,
		ESlateDrawEffect::None, Color, /*bAntialias=*/true, Thickness);
}

int32 URopePullGaugeWidget::NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
	FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const int32 Result = Super::NativePaint(Args, AllottedGeometry, MyCullingRect, OutDrawElements, LayerId, InWidgetStyle, bParentEnabled);

	// Nothing is drawn while disarmed, which leaves the screen clear in normal play.
	if (!bDrawBuiltInVisuals || !IsPullArmed())
	{
		return Result;
	}

	const FVector2D LocalSize = AllottedGeometry.GetLocalSize();
	const FVector2D Center(LocalSize.X * CenterAnchor.X, LocalSize.Y * CenterAnchor.Y);

	// The engagement pop briefly grows the radius and returns it, as a single beat announcing the event.
	float PopScale = 1.0f;
	if (EngagePopTime > 0.0f && TimeSinceEngage < EngagePopTime)
	{
		const float T = TimeSinceEngage / EngagePopTime;
		// From zero to the maximum, converging on one at the end.
		PopScale = FMath::Lerp(EngagePopScale, 1.0f, FMath::Sin(T * PI * 0.5f));
	}
	const float DrawRadius = Radius * PopScale;

	// The background ring, being the remainder, is always laid down as a full circle so it is clear how much further there is to pull.
	DrawArc(AllottedGeometry, OutDrawElements, LayerId + 1, Center, DrawRadius, 1.0f, TrackColor);

	// The progress arc. Its colour interpolates from the waiting colour to the engaged colour, so that approaching the threshold is readable from the colour before it fills.
	const float Shown = IsPullEngaged() ? 1.0f : FMath::Clamp(DisplayProgress, 0.0f, 1.0f);
	const FLinearColor ArcColor = FMath::Lerp(ArmedColor, EngagedColor, Shown);
	DrawArc(AllottedGeometry, OutDrawElements, LayerId + 2, Center, DrawRadius, Shown, ArcColor);

	return FMath::Max(Result, LayerId + 2);
}
