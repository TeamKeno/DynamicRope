// Copyright Epic Games, Inc. All Rights Reserved.
//
// Editor visualizer for URopeSDFProvider. When the component is selected, draws each baked (or
// synthetic-preview) per-bone SDF volume: bounds box, coarse grid, and narrow-band voxels colored
// by sign. Reads the SDF Debug toggles on the provider. Uses the shared RopeSDFSampler.

#pragma once

#include "CoreMinimal.h"
#include "ComponentVisualizer.h"

class FRopeSDFVisualizer : public FComponentVisualizer
{
public:
	virtual void DrawVisualization(const UActorComponent* Component, const FSceneView* View,
		FPrimitiveDrawInterface* PDI) override;
};
