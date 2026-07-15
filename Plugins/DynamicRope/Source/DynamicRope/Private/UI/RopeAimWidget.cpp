// Copyright Epic Games, Inc. All Rights Reserved.

#include "UI/RopeAimWidget.h"

#include "Blueprint/WidgetLayoutLibrary.h"
#include "Blueprint/WidgetTree.h"
#include "Components/CanvasPanel.h"
#include "Camera/PlayerCameraManager.h"
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

	// C++ 단독 사용(WBP 없이 이 클래스를 그대로 생성) 대비: 루트 위젯이 없으면 Slate 트리가 만들어지지
	// 않아 NativePaint가 불리지 않는다 — 빈 캔버스를 루트로 만들어 전체 화면 지오메트리를 확보한다.
	// WBP 서브클래스는 자기 루트를 가지므로 이 분기를 타지 않는다.
	if (bResult && WidgetTree && !WidgetTree->RootWidget)
	{
		WidgetTree->RootWidget = WidgetTree->ConstructWidget<UCanvasPanel>(UCanvasPanel::StaticClass(), TEXT("AimRootCanvas"));
	}
	return bResult;
}

void URopeAimWidget::NativeConstruct()
{
	Super::NativeConstruct();

	// 조준 HUD는 표시 전용 — 게임 입력/커서를 가로채지 않는다.
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
	// 모드(UsesAimRay)가 아니라 프레임 단위 IsAimActive를 본다 — ③는 Reel에서만 조준이 성립하므로.
	// 십자선은 샘플을 보지 않고 이 게이트로만 그려지므로(NativePaint), 여기서 막지 않으면 빈 샘플의
	// AimWorldPos(=ZeroVector)가 월드 원점으로 투영돼 십자선이 원점/화면중앙에 남는다.
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
	// 대상 진입/전환 순간 — 획득 팝을 처음부터 다시 재생한다.
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

	// 폰 교체/지연 빙의 대비 재해석(무효일 때만 — 평시 무비용).
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

	// 실제 aim ray의 hit(또는 미충돌 시 끝점)를 조준원 위치로 쓴다. 화면 중앙 고정이 아니므로
	// AttachMesh/Socket/Owner/View 중 어떤 AimRayOriginMode를 써도 UI와 레이 경로가 일치한다.
	if (UWidgetLayoutLibrary::ProjectWorldLocationToWidgetPosition(
		PC, Sample.AimWorldPos, AimScreenPos, /*bPlayerViewportRelative*/ false))
	{
		bHasScreenAim = true;
	}

	// 감길 대상(초록) 또는 wrap 불가 hit(빨강) — 둘 다 화면에 투영해 링을 그린다.
	if (!Sample.bHasTarget && !Sample.bBlocked)
	{
		return;
	}

	// 월드 → 뷰포트 위젯 공간(DPI 보정 포함). 화면 뒤/투영 실패면 링을 숨긴다.
	FVector2D CenterPos = FVector2D::ZeroVector;
	if (!UWidgetLayoutLibrary::ProjectWorldLocationToWidgetPosition(PC, Sample.TargetWorldPos, CenterPos, /*bPlayerViewportRelative*/ false))
	{
		return;
	}

	// 화면 반경: 대상 월드 반경만큼 카메라 오른쪽으로 이동한 점을 함께 투영해 픽셀 거리로 환산한다.
	float ScreenRadius = RingMinScreenRadius;
	if (const APlayerCameraManager* Camera = PC->PlayerCameraManager)
	{
		const FVector CamRight = Camera->GetCameraRotation().RotateVector(FVector::RightVector);
		const FVector EdgeWorld = Sample.TargetWorldPos + CamRight * FMath::Max(Sample.TargetRadius, 1.0f) * RingRadiusScale;
		FVector2D EdgePos = FVector2D::ZeroVector;
		if (UWidgetLayoutLibrary::ProjectWorldLocationToWidgetPosition(PC, EdgeWorld, EdgePos, /*bPlayerViewportRelative*/ false))
		{
			ScreenRadius = FMath::Max(RingMinScreenRadius, static_cast<float>(FVector2D::Distance(CenterPos, EdgePos)));
		}
	}

	bHasScreenTarget = true;
	bScreenTargetBlocked = Sample.bBlocked && !Sample.bHasTarget;
	TargetScreenPos = CenterPos;
	TargetScreenRadius = ScreenRadius;
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

	// --- 십자선: AimRayOriginMode가 정한 실제 ray의 hit/끝점, 투영 실패 시에만 화면 중앙 폴백.
	{
		const FVector2D Center = bHasScreenAim ? AimScreenPos : AllottedGeometry.GetLocalSize() * 0.5f;
		// 대상 없음=기본, 감길 대상=획득색, wrap 불가=빨강.
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

	// --- 강조 링: 대상 본 주위. 획득 팝(크게 → 제 크기) + 유지 펄스.
	if (bHasScreenTarget)
	{
		float Radius = TargetScreenRadius;
		// 획득 팝은 대상 획득 순간(HandleAimTargetChanged) 기준이라 감길 대상에만 적용 — blocked는 펄스만.
		if (!bScreenTargetBlocked && AcquirePopDuration > KINDA_SMALL_NUMBER && TimeSinceAcquire < AcquirePopDuration)
		{
			// 1.6배에서 제 크기로 수축(ease-out).
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
