// Copyright Epic Games, Inc. All Rights Reserved.

#include "Visualizers/RopeComponentVisualizer.h"
#include "RopeComponent.h"
#include "SceneManagement.h"

void FRopeComponentVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View,
	FPrimitiveDrawInterface* PDI)
{
	const URopeComponent* Rope = Cast<URopeComponent>(Component);
	if (!Rope || !PDI)
	{
		return;
	}

	const TArray<FVector>& Points = Rope->GetCenterlinePositions();
	if (Points.Num() >= 2)
	{
		// Simulated centerline (PIE, or after a tick has populated the sim state).
		for (int32 i = 0; i + 1 < Points.Num(); ++i)
		{
			PDI->DrawLine(Points[i], Points[i + 1], FLinearColor(0.0f, 1.0f, 1.0f), SDPG_Foreground, 1.0f);
		}
	}
	else
	{
		// Not simulated (placing in-editor): draw the predicted straight rest line to aid placement.
		const FVector Start = Rope->GetComponentLocation();
		const FVector End = Start + Rope->GetForwardVector() * Rope->RopeLength;
		PDI->DrawLine(Start, End, FLinearColor(0.5f, 0.5f, 0.5f), SDPG_Foreground, 1.0f);
		PDI->DrawPoint(Start, FLinearColor::Green, 8.0f, SDPG_Foreground);
	}
}
