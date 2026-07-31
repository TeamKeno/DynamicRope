// Copyright Epic Games, Inc. All Rights Reserved.
//
// The throwaway view target behind the wrap camera cut. URopeWrapCameraDirectorComponent spawns one of
// these, binds it to a URopeWrapCameraComponent on the wrap target, and hands it to the player controller
// as the view target for the length of the shot.
//
// It exists because a view target is an actor, not a component, and pointing the controller at the target
// actor itself would be ambiguous: AActor::CalcCamera picks the first *active* camera component, so a target
// that is a player pawn would put its own gameplay camera up against the marker. This rig owns no camera
// component at all and instead overrides CalcCamera to delegate to the bound marker, which
// APlayerCameraManager::UpdateViewTargetInternal calls on any view target. The result is the exact view the
// marker authored, FOV and post process included, with nothing on the target actor touched and camera
// modifiers and shakes still applied normally.
//
// Spawned and destroyed by the director; game code has no reason to place one.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeWrapCameraRig.generated.h"

class URopeWrapCameraComponent;

UCLASS(NotBlueprintable, NotPlaceable, Transient)
class DYNAMICROPE_API ARopeWrapCameraRig : public AActor
{
	GENERATED_BODY()

public:
	ARopeWrapCameraRig();

	/** Points this rig at the marker whose view it should reproduce, and attaches to it so the actor
	 *  transform tracks the shot as well, which costs nothing and keeps the view target's location
	 *  meaningful to anything that queries it. */
	void Bind(URopeWrapCameraComponent* Marker);

	/** The bound marker, or null once it or its actor has been destroyed. The director watches this to end
	 *  the shot when the target disappears mid-cut. */
	URopeWrapCameraComponent* GetBoundCamera() const { return BoundCamera.Get(); }

	//~ AActor. Delegates the view to the bound marker; falls back to the actor's own transform once the
	//~ marker is gone, which freezes the shot rather than snapping it somewhere unexpected.
	virtual void CalcCamera(float DeltaTime, FMinimalViewInfo& OutResult) override;

private:
	// Weak, since the marker belongs to the wrap target and that actor can be destroyed while the shot is
	// still blending.
	TWeakObjectPtr<URopeWrapCameraComponent> BoundCamera;
};
