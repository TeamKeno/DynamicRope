// Copyright Epic Games, Inc. All Rights Reserved.

#include "Visualizers/RopeComponentVisualizer.h"
#include "RopeComponent.h"
#include "SceneManagement.h"

// 레벨 에디터에서 로프 액터를 선택했을 때만 호출되는 배치-보조 비주얼라이저(편집 중 sim 없음, PIE 아님).
// 실제 로프 형상은 scene proxy 튜브가 이미 렌더하므로, 여기서는 튜브가 못 보여주는 "배치 정보"만 그린다:
//  앵커+조준 화살표 / 도달 범위 / 던지기 예상 아크.
// (wrap 타깃 링크는 제거됨 — 런타임 wrap 대상은 접촉에서 확정되므로 지정할 값 자체가 없다.)
namespace
{
	// 조준 벡터로부터 Up/Side 프레임 구성(StartFreshThrow와 동일 규약).
	void MakeAimFrame(const FVector& Aim, FVector& OutUp, FVector& OutSide)
	{
		FVector Up = FVector::UpVector;
		if (FMath::Abs(FVector::DotProduct(Aim, Up)) > 0.96f)
		{
			Up = FVector::RightVector;
		}
		OutSide = FVector::CrossProduct(Up, Aim).GetSafeNormal();
		OutUp   = FVector::CrossProduct(Aim, OutSide).GetSafeNormal();
	}
}

void FRopeComponentVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View,
	FPrimitiveDrawInterface* PDI)
{
	const URopeComponent* Rope = Cast<URopeComponent>(Component);
	if (!Rope || !PDI || !Rope->bShowPlacementGuides)
	{
		return;
	}

	const FVector Anchor   = Rope->GetComponentLocation();
	const FVector Aim      = Rope->GetForwardVector().GetSafeNormal();
	const float   RopeLen  = FMath::Max(Rope->RopeLength, 1.0f);

	FVector Up, Side;
	MakeAimFrame(Aim, Up, Side);

	// --- 앵커(소켓/고정점) 마커 + 조준(throw) 방향 화살표.
	PDI->DrawPoint(Anchor, FLinearColor::Green, 12.0f, SDPG_Foreground);
	{
		const float ArrowLen = FMath::Clamp(RopeLen * 0.5f, 30.0f, 250.0f);
		const FVector Tip = Anchor + Aim * ArrowLen;
		const FLinearColor AimColor(1.0f, 0.85f, 0.1f); // 노랑
		PDI->DrawLine(Anchor, Tip, AimColor, SDPG_Foreground, 2.0f);
		// 화살촉: tip에서 뒤로 벌어진 두 선.
		const float Head = ArrowLen * 0.18f;
		PDI->DrawLine(Tip, Tip - Aim * Head + Side * (Head * 0.5f), AimColor, SDPG_Foreground, 2.0f);
		PDI->DrawLine(Tip, Tip - Aim * Head - Side * (Head * 0.5f), AimColor, SDPG_Foreground, 2.0f);
	}

	// --- 도달 범위: 앵커 기준 RopeLength 반경 와이어 구(3개 great circle). 로프가 닿는 최대 거리.
	{
		const FLinearColor ReachColor(0.1f, 0.8f, 1.0f, 1.0f); // 시안
		const int32 Sides = 48;
		DrawCircle(PDI, Anchor, FVector::XAxisVector, FVector::YAxisVector, ReachColor, RopeLen, Sides, SDPG_World, 0.5f);
		DrawCircle(PDI, Anchor, FVector::YAxisVector, FVector::ZAxisVector, ReachColor, RopeLen, Sides, SDPG_World, 0.5f);
		DrawCircle(PDI, Anchor, FVector::XAxisVector, FVector::ZAxisVector, ReachColor, RopeLen, Sides, SDPG_World, 0.5f);
	}

	// --- 던지기 예상 아크: whip 파라미터로 던질 때의 대략 궤적(정확한 sim 아님, 방향/높이 미리보기).
	{
		const float ArcHeight = Rope->WhipConfig.ArcHeight;
		const float SideOff   = Rope->WhipConfig.SideOffset;
		const FLinearColor ArcColor(1.0f, 0.5f, 0.0f); // 주황
		const int32 Seg = 24;
		FVector Prev = Anchor;
		for (int32 i = 1; i <= Seg; ++i)
		{
			const float S = static_cast<float>(i) / static_cast<float>(Seg);
			const float ArcF = FMath::Sin(S * PI); // 중간에서 최대로 솟음
			const FVector P = Anchor + Aim * (S * RopeLen) + Up * (ArcF * ArcHeight) + Side * (ArcF * SideOff);
			PDI->DrawLine(Prev, P, ArcColor, SDPG_World, 1.0f);
			Prev = P;
		}
	}
}
