// Copyright Epic Games, Inc. All Rights Reserved.
//
// Editor-only visualizer for URopeComponent. Draws the simulated centerline when available
// (PIE / after a tick), or a predicted straight rest line while placing the component in-editor.

#pragma once

#include "CoreMinimal.h"
#include "ComponentVisualizer.h"

class FRopeComponentVisualizer : public FComponentVisualizer
{
public:
	virtual void DrawVisualization(const UActorComponent* Component, const FSceneView* View,
		FPrimitiveDrawInterface* PDI) override;
};
