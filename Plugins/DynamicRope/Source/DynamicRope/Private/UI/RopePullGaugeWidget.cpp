// Copyright Epic Games, Inc. All Rights Reserved.

#include "UI/RopePullGaugeWidget.h"
#include "Gameplay/RopeWielderComponent.h"

#include "GameFramework/Pawn.h"
#include "Rendering/DrawElements.h"

URopePullGaugeWidget::URopePullGaugeWidget(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// 입력을 먹지 않는 순수 표시 위젯.
	SetVisibility(ESlateVisibility::HitTestInvisible);
}

void URopePullGaugeWidget::NativeConstruct()
{
	Super::NativeConstruct();
	ResolveWielder();
}

void URopePullGaugeWidget::NativeDestruct()
{
	// 위젯이 먼저 사라져도 wielder 델리게이트에 죽은 핸들러가 남지 않게 한다.
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
		// 해제되면 다음 장전이 0에서 다시 차오르도록 표시 상태를 되돌린다.
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
		// 재무장(wrap 해제) — 팝을 끄지 않으면 발동 표시가 켜진 채 남는다.
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

	// 폰 교체/지연 빙의 대비 재해석(무효일 때만).
	ResolveWielder();

	TimeSinceEngage += InDeltaTime;

	// 장력은 프레임마다 떨리므로 표시값만 부드럽게 따라가게 한다(판정값은 건드리지 않는다).
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

	// 12시에서 시계 방향으로 채운다(게이지의 통념).
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

	// ① 해제 상태에서는 아무것도 그리지 않는다 — 평상시 화면을 비워 둔다.
	if (!bDrawBuiltInVisuals || !IsPullArmed())
	{
		return Result;
	}

	const FVector2D LocalSize = AllottedGeometry.GetLocalSize();
	const FVector2D Center(LocalSize.X * CenterAnchor.X, LocalSize.Y * CenterAnchor.Y);

	// 발동 팝: 짧은 시간 동안 반경을 키웠다 되돌린다(사건임을 알리는 한 방).
	float PopScale = 1.0f;
	if (EngagePopTime > 0.0f && TimeSinceEngage < EngagePopTime)
	{
		const float T = TimeSinceEngage / EngagePopTime;
		// 0에서 최대, 끝에서 1로 수렴.
		PopScale = FMath::Lerp(EngagePopScale, 1.0f, FMath::Sin(T * PI * 0.5f));
	}
	const float DrawRadius = Radius * PopScale;

	// 배경 링(남은 몫) — 얼마나 더 당겨야 하는지가 보이도록 항상 완전한 원으로 깐다.
	DrawArc(AllottedGeometry, OutDrawElements, LayerId + 1, Center, DrawRadius, 1.0f, TrackColor);

	// 진행 호. 색은 대기색 → 발동색 보간이라, 다 차기 전에도 "가까워지고 있다"가 색으로 읽힌다.
	const float Shown = IsPullEngaged() ? 1.0f : FMath::Clamp(DisplayProgress, 0.0f, 1.0f);
	const FLinearColor ArcColor = FMath::Lerp(ArmedColor, EngagedColor, Shown);
	DrawArc(AllottedGeometry, OutDrawElements, LayerId + 2, Center, DrawRadius, Shown, ArcColor);

	return FMath::Max(Result, LayerId + 2);
}
