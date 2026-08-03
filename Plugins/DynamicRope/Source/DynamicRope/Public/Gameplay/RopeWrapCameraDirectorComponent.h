// Copyright Epic Games, Inc. All Rights Reserved.
//
// Opt-in camera cut for the moment a rope wraps. Add it to the wielder actor, the one holding the rope, and
// whenever that rope establishes a wrap on a target carrying a URopeWrapCameraComponent it blends the
// player's view to that marker's shot, holds it for the marker's HoldTime, and blends back. A release, a cut
// or losing the target ends the shot early. Trigger picks the moment of the cut: the wrap commit
// (default), or the flight capture, which puts the wrapping motion itself inside the shot.
//
// It lives on the wielder rather than on the target because the player's view belongs to the wielder: this
// component is where the PlayerController is, one wielder can only be watching one shot at a time whatever
// the target does, and a game can leave it off for a wielder that should never lose control of its camera.
// The target side only authors *where* the shot is, through its marker.
//
// The framing itself is nothing this component owns; it hands a spawned ARopeWrapCameraRig to the controller
// and the rig delegates to the marker. See RopeWrapCameraRig.h for why the target actor is not made the view
// target directly.
//
// A wrap on a target with no marker does nothing at all, and neither does a wrap arriving while another
// system, a cinematic or the ragdoll follow camera, already holds the view target.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
// FRopeWrappedEventInfo and ERopeReleaseReason, the payloads of the rope events bound below. Dynamic
// delegates need the full types in the handler signatures.
#include "Core/RopeLifecycleTypes.h"
// FViewTargetTransitionParams, held by value so the blend back survives the marker being destroyed mid-cut.
#include "Camera/PlayerCameraManager.h"
#include "Engine/TimerHandle.h"
#include "RopeWrapCameraDirectorComponent.generated.h"

class APlayerController;
class ARopeWrapCameraRig;
class URopeComponent;
class URopeWrapCameraComponent;

/** The rope moment URopeWrapCameraDirectorComponent cuts on. */
UENUM(BlueprintType)
enum class ERopeWrapCameraTrigger : uint8
{
	/** Cut once the wrap has committed (OnRopeWrapped): the shot opens on the established catch. */
	WrapCommitted,

	/** Cut the moment the flight capture latches onto the target (OnRopeCaptured), before the wrap
	 *  commits, so the wrapping motion itself plays inside the shot. Author the marker's HoldTime long
	 *  enough to cover the wrap; a capture that aborts ends the shot through the release event. */
	Captured,
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeWrapCameraOnBegin, URopeWrapCameraComponent*, Camera);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FRopeWrapCameraOnEnd);

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeWrapCameraDirectorComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeWrapCameraDirectorComponent();

	//~ UActorComponent. Subscribes to and unsubscribes from the rope's wrap and release events. The tick is
	//~ only enabled while a shot is playing.
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** Master switch. Turning it off mid-shot does not interrupt the shot in progress; call StopCamera for
	 *  that. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera")
	bool bEnabled = true;

	/** Which rope moment starts the shot. WrapCommitted opens on the established catch; Captured cuts at
	 *  the touch, so the wrapping motion itself is on screen. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera")
	ERopeWrapCameraTrigger Trigger = ERopeWrapCameraTrigger::WrapCommitted;

	/** The rope to watch. Left unset, the owner's first URopeComponent is used, which is the normal setup
	 *  where the wielder owns its rope. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera")
	TObjectPtr<URopeComponent> Rope;

	/** Skip the cut when the rope wrapped the wielder itself, which is a self-tether rather than a catch and
	 *  has nothing to show. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera")
	bool bSkipSelfWrap = true;

	/** Fires once the view target has been handed to the shot, carrying the marker that was chosen. Useful
	 *  for the game's own accompaniment, a sound or a slow motion effect. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Wrap Camera")
	FRopeWrapCameraOnBegin OnWrapCameraBegin;

	/** Fires when the blend back to the wielder starts, whether the shot ran its course or was cut short. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Wrap Camera")
	FRopeWrapCameraOnEnd OnWrapCameraEnd;

	UFUNCTION(BlueprintPure, Category = "Rope|Wrap Camera")
	bool IsPlaying() const { return ActiveRig.IsValid(); }

	/** The marker currently being framed, or null when no shot is playing. */
	UFUNCTION(BlueprintPure, Category = "Rope|Wrap Camera")
	URopeWrapCameraComponent* GetActiveCamera() const;

	/** Ends the shot now and starts the blend back to the wielder. Harmless when nothing is playing. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Wrap Camera")
	void StopCamera();

protected:
	/** The rope wrapped something: pick the target's marker and start the shot. Not bound as a state change
	 *  on the rope, so it respects the OnRopeWrapped contract of not mutating rope state. */
	UFUNCTION()
	void HandleRopeWrapped(const FRopeWrappedEventInfo& Info);

	/** The rope captured a target mid-flight: with Trigger = Captured this is the cut point. The event
	 *  carries only the bone, so the target is read from the rope's capture tracker. */
	UFUNCTION()
	void HandleRopeCaptured(FName Bone);

	/** Any release, including a cut and a pre-commit abort, ends the shot early. */
	UFUNCTION()
	void HandleRopeReleased(FName Bone, ERopeReleaseReason Reason);

private:
	/** Fills Rope from the owner when it was left unset, and returns it. */
	URopeComponent* ResolveRope();

	/** The player controller watching through the wielder, or null when the owner is not a
	 *  player-controlled pawn, in which case there is no camera to direct. */
	APlayerController* ResolveController() const;

	/** Spawns the rig, hands it to the controller and arms the hold timer. */
	void BeginCamera(URopeWrapCameraComponent* Marker);

	/** Blends back to the wielder and retires the rig. Safe to call repeatedly and while nothing is
	 *  playing. */
	void FinishCamera();

	// The rig currently acting as the view target. Weak: it is a world actor, so a level teardown or an
	// outside Destroy must not be able to leave a dangling pointer here.
	TWeakObjectPtr<ARopeWrapCameraRig> ActiveRig;
	// The controller the shot was handed to, held weakly for the same reason.
	TWeakObjectPtr<APlayerController> ActiveController;
	// Copied from the marker at the start of the shot rather than read back at the end, because the target
	// actor, and with it the marker, can be destroyed while the shot is still running and the blend back
	// still has to happen with the values the designer authored.
	FViewTargetTransitionParams PendingBlendOut;

	FTimerHandle HoldTimer;
};
