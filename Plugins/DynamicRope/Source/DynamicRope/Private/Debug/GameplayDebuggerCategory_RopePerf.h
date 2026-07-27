// Copyright Epic Games, Inc. All Rights Reserved.
//
// The 'RopePerf' gameplay debugger category, a world-wide overview of rope performance and throttling. Where the
// 'Rope' category draws one debug actor's ropes in depth, this one walks every active rope in the world and
// aggregates the frame load at a glance, which is what catches a performance problem on a rope nobody is looking at.
// It puts the same values as the 'stat DynamicRope' group onto the HUD as per-rope lines plus a summary at the top.
//
// The rope list comes from URopeSimSubsystem::GetRegisteredRopes() alone: the definition of the "ropes actually
// ticked this frame" that this screen aggregates is exactly that registration list, and walking the world separately
// would mix in ropes the subsystem never drives, which would disagree with the frame load.
// Once the list is obtained, every value is read live from URopeComponent's public getters: GetPhase, GetNodeCount,
// IsSleeping, GetSolverLODScale, IsGpuSteppedThisFrame, WasSolvedThisFrame, bUseWorldGDF and GetCenterlinePositions.
// Nothing depends on a snapshot. The camera distance is derived directly from the owning player controller's camera.
// The whole file is excluded from the compile in builds where WITH_GAMEPLAY_DEBUGGER is off, such as shipping.

#pragma once

#include "CoreMinimal.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "GameplayDebuggerCategory.h"

class APlayerController;
class AActor;

class FGameplayDebuggerCategory_RopePerf : public FGameplayDebuggerCategory
{
public:
	FGameplayDebuggerCategory_RopePerf();

	virtual void CollectData(APlayerController* OwnerPC, AActor* DebugActor) override;
	virtual void DrawData(APlayerController* OwnerPC, FGameplayDebuggerCanvasContext& CanvasContext) override;

	static TSharedRef<FGameplayDebuggerCategory> MakeInstance();

private:
	// MakePoint is a wire sphere of sixteen divisions and therefore produces 512 batched lines per marker. Only the
	// position, colour, pixel size and number are replicated, and the viewing client draws them from its draw data
	// with DrawDebugPoint and DrawDebugString, which reduces the render load.
	struct FRepData
	{
		struct FMarker
		{
			FVector Location = FVector::ZeroVector;
			FColor Color = FColor::White;
			float PixelSize = 6.0f;
			int32 DisplayIndex = INDEX_NONE;
		};

		TArray<FMarker> Markers;

		void Serialize(FArchive& Ar);
	};

	FRepData DataPack;
};

#endif // WITH_GAMEPLAY_DEBUGGER
