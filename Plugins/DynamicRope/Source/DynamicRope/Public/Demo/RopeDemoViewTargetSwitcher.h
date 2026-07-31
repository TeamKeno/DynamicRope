// Copyright Epic Games, Inc. All Rights Reserved.
//
// Shared view-target switch for the demo actors. Blends the captured player's view to a camera
// mounted on the demo actor for the duration of the capture, then hands the view back.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"

class AActor;
class APlayerController;

/** Blends a player's ViewTarget to a demo actor's on-board camera and back.
 *
 *  Owned by value by a demo actor. Activate at the moment the ropes fire and Deactivate on every
 *  recall path; both are safe to call redundantly. Only a player-controlled pawn is switched — an
 *  AI or prop target is a quiet no-op — and a view already taken by something else, such as a
 *  cinematic, is respected in both directions. */
struct DYNAMICROPE_API FRopeDemoViewTargetSwitcher
{
	/** Blends the target's player view to DemoActor, whose first camera component frames the shot.
	 *  A no-op without a player-controlled pawn target, without a camera on DemoActor, or when the
	 *  player is already viewing something other than their pawn. */
	void Activate(AActor* DemoActor, AActor* Target, float BlendTime);

	/** Blends back to the view target captured by Activate, or to the controller's current pawn when
	 *  that actor is gone. A no-op when not active, when the demo camera no longer owns the view, or
	 *  during world teardown. Always clears the switch state. */
	void Deactivate(float BlendTime);

	/** Whether Activate has switched a view that Deactivate has not yet handed back. */
	bool IsActive() const { return Controller.IsValid(); }

private:
	/** The controller whose view was switched. Doubles as the active flag. */
	TWeakObjectPtr<APlayerController> Controller;

	/** Where the view returns on Deactivate, normally the captured pawn. */
	TWeakObjectPtr<AActor> ReturnTarget;

	/** The demo actor serving as the view target, compared against the current view on Deactivate. */
	TWeakObjectPtr<AActor> DemoViewTarget;
};
