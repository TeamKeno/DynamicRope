// Copyright Epic Games, Inc. All Rights Reserved.
//
// Demo rescue helicopter. Hovers overhead until a passenger enters the grab zone below, drops its
// cable to wrap the passenger's hand bone (GuaranteedWrap resolve mode), reels them up, flies the
// configured waypoints and releases them at the destination. It then returns home for the next run.
//
// State flow:
//   Idle (hover + rotor) -> [grab zone entered / Grab()] Grabbing (guaranteed throw, retried)
//        -> Carrying (reel in, then fly the waypoints)
//        -> [last waypoint reached] ReleaseCarried() -> Returning (fly home) -> Idle
//
// Content requirements:
//   - Body and rotor meshes are assigned by the level or Blueprint (BodyMesh / RotorMesh); the
//     plugin never references /Game content itself.
//   - A passenger needs a skeletal mesh plus a rope collider provider (capsule or SDF) to be
//     wrappable, and must own the bone named by GrabBone (hand_r by default).
//   - If the passenger has a URopeRagdollResponseComponent with bRagdollOnWrapped, it goes limp the
//     moment the wrap lands. Leave it on for a dangling-cargo look; leave it off to keep the
//     passenger controllable.
//   - A player-controlled passenger's view blends to ViewCamera from the moment the cable fires
//     until it is recalled (bSwitchPlayerViewTarget). An AI passenger keeps its own camera.
//
// Lifting a walking character clear of the ground is the combined effect of the reel-in tether and
// CarryPullForce; raise LiftReelSpeed or CarryPullForce if the passenger stays grounded.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Demo/RopeDemoViewTargetSwitcher.h"
#include "RopeDemoHelicopter.generated.h"

class URopeComponent;
class UStaticMeshComponent;
class USphereComponent;
class USkeletalMeshComponent;
class UCameraComponent;

/** Fired when a passenger is picked up or set down (wrap established / released). */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoHelicopterCarrySignature,
	ARopeDemoHelicopter*, Helicopter, bool, bCarrying);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Helicopter"))
class DYNAMICROPE_API ARopeDemoHelicopter : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoHelicopter();

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Drops the cable towards the given passenger. Only valid while idle; returns true on success.
	 *  Use this to pick a passenger explicitly instead of, or alongside, automatic grabbing. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	bool Grab(AActor* Passenger);

	/** Aborts the grab attempt in progress, cancelling any queued throw and recalling the cable.
	 *  Use ReleaseCarried instead once the passenger is being carried. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void CancelGrab();

	/** Sets the carried passenger down: releases the wrap and restores the cable length. The
	 *  helicopter then returns home or hovers in place depending on bReturnHomeAfterRelease. Also
	 *  exposed as a details panel button. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void ReleaseCarried();

	/** True while a passenger is wrapped, including during the reel-in. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsCarrying() const;

	/** The passenger currently carried or being grabbed, or nullptr. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	AActor* GetCarriedActor() const { return CarryTarget.Get(); }

	/** Broadcast when a passenger is picked up or set down. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoHelicopterCarrySignature OnCarryStateChanged;

	//~ Route -------------------------------------------------------------------

	/** Waypoints flown in order once a passenger is aboard (any actor, for example a TargetPoint);
	 *  the last one is the destination. Leave empty to hold position like a hovering crane. */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo")
	TArray<TObjectPtr<AActor>> Waypoints;

	//~ Grabbing ----------------------------------------------------------------

	/** Automatically drop the cable when an actor with a skeletal mesh enters the grab zone. Turn
	 *  off to pick passengers through Grab() only. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab")
	bool bAutoGrab = true;

	/** Passenger bone to wrap. Grabbing does not start if the passenger's skeleton lacks it, and a
	 *  warning is logged. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab")
	FName GrabBone = TEXT("hand_r");

	/** Distance below the body to the centre of the grab zone (cm). Must be within cable range. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab", meta = (ClampMin = "0.0", Units = "cm"))
	float GrabZoneDrop = 600.0f;

	/** Radius of the grab zone (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab", meta = (ClampMin = "10.0", Units = "cm"))
	float GrabZoneRadius = 250.0f;

	/** Delay between grab retries (s), which also settles the throw arming edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab", meta = (ClampMin = "0.1", Units = "s"))
	float GrabRetryPeriod = 1.0f;

	/** Give up grabbing if no wrap lands within this time (s). 0 disables the timeout. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab", meta = (ClampMin = "0.0", Units = "s"))
	float GrabTimeout = 8.0f;

	//~ Carrying ----------------------------------------------------------------

	/** Cable length while carrying (cm). The passenger is reeled in to this length; shorter values
	 *  hang them closer to the body. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry", meta = (ClampMin = "50.0", Units = "cm"))
	float CarryRopeLength = 350.0f;

	/** Reel-in speed used to lift the passenger (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry", meta = (ClampMin = "1.0", Units = "cm/s"))
	float LiftReelSpeed = 200.0f;

	/** Active pull force applied while carrying. 0 relies on the reel-in tether alone; add force
	 *  when reeling by itself does not lift the passenger clear of the ground. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry", meta = (ClampMin = "0.0"))
	float CarryPullForce = 0.0f;

	/** Tolerance on the carry length (cm) below which the lift counts as finished and flight starts. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry", meta = (ClampMin = "0.1", Units = "cm"))
	float LiftTolerance = 15.0f;

	/** Release the passenger automatically on reaching the last waypoint. Turn off to keep hovering
	 *  with the passenger until ReleaseCarried is called. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry")
	bool bReleaseAtLastWaypoint = true;

	/** Fly back to the start position after releasing. Turn off to hover in place and wait for the
	 *  next passenger. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry")
	bool bReturnHomeAfterRelease = true;

	//~ View camera -------------------------------------------------------------

	/** Blend the player's view to ViewCamera from the moment the cable fires until it is recalled.
	 *  Only a player-controlled passenger switches; an AI passenger leaves the view alone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Camera")
	bool bSwitchPlayerViewTarget = true;

	/** Blend time into ViewCamera (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Camera", meta = (ClampMin = "0.0", Units = "s"))
	float ViewBlendInTime = 0.75f;

	/** Blend time back to the passenger's own view (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Camera", meta = (ClampMin = "0.0", Units = "s"))
	float ViewBlendOutTime = 0.75f;

	//~ Flight ------------------------------------------------------------------

	/** Cruise speed (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "1.0", Units = "cm/s"))
	float FlySpeed = 600.0f;

	/** Radius within which a waypoint counts as reached (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "1.0", Units = "cm"))
	float WaypointTolerance = 100.0f;

	/** Turn the nose towards the direction of travel (yaw only, no banking). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight")
	bool bFaceTravelDirection = true;

	/** Yaw turn rate (degrees per second). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "1.0"))
	float TurnRateDeg = 90.0f;

	/** Vertical hover bob amplitude (cm). 0 disables the bob. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "0.0", Units = "cm"))
	float HoverBobAmplitude = 15.0f;

	/** Hover bob period (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "0.1", Units = "s"))
	float HoverBobPeriod = 3.0f;

	/** Rotor spin rate (RPM). 0 stops the rotor. Only applies to RotorMesh. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "0.0"))
	float RotorRPM = 300.0f;

protected:
	/** Root and flight reference point. The helicopter is kinematic and drives the actor location
	 *  directly. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USceneComponent> Base = nullptr;

	/** Body mesh, assigned by the level or Blueprint. Collision follows the asset settings: blocking
	 *  PhysicsBody lets other ropes wrap or collide with it. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> BodyMesh = nullptr;

	/** Optional rotor mesh, assigned when the rotor is a separate asset from the body. Spun about
	 *  the yaw axis at RotorRPM. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> RotorMesh = nullptr;

	/** Point under the body the cable hangs from; adjust its location in the details panel. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USceneComponent> RopeAttach = nullptr;

	/** The rescue cable, configured for GuaranteedWrap. Wraps the passenger's hand bone. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<URopeComponent> Rope = nullptr;

	/** Grab zone sphere below the body. Query-only, so rope collider gathering ignores it. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USphereComponent> GrabVolume = nullptr;

	/** Ride camera the passenger's view blends to while grabbed and carried. Reframe it in the
	 *  Blueprint or level; the default looks down at the hanging cable from behind the body. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UCameraComponent> ViewCamera = nullptr;

private:
	/** Demo state. Transitions are owned by Tick and the public entry points. */
	enum class EState : uint8 { Idle, Grabbing, Carrying, Returning };

	/** Fires a guaranteed throw at the passenger, including the arming edge. True once queued. */
	bool FireRopeAtTarget();

	/** The passenger's first skeletal mesh component. */
	USkeletalMeshComponent* ResolveTargetMesh() const;

	/** Shared cable recall: cancels a queued throw, releases the wrap in any phase and restores the
	 *  cable length and traction settings. */
	void RecallRope();

	/** Moves towards Dest at cruise speed, optionally turning the nose. Returns the distance left. */
	float MoveTowards(const FVector& Dest, float DeltaSeconds);

	/** Writes the current navigation position and heading back onto the actor. */
	void ApplyPose();

	/** Grab zone entry handler for automatic grabbing. */
	UFUNCTION()
	void HandleGrabZoneBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	EState State = EState::Idle;

	/** Passenger being grabbed or carried. Weak so a destroyed passenger is safe. */
	TWeakObjectPtr<AActor> CarryTarget;

	/** Hover reference point, excluding the bob. Doubles as the Idle and Returning destination. */
	FVector IdleAnchor = FVector::ZeroVector;

	/** Current navigation position, excluding the bob (the bob is display only). */
	FVector NavPos = FVector::ZeroVector;

	/** Current heading yaw (degrees). */
	float NavYaw = 0.0f;

	/** Hover bob clock (s). */
	float BobTime = 0.0f;

	/** Grab retry cooldown and elapsed grab time (s). */
	float GrabRetryRemaining = 0.0f;
	float GrabElapsed = 0.0f;

	/** Index of the waypoint currently being flown to. */
	int32 WaypointIndex = 0;

	/** Whether the pick-up has already been broadcast (guards against duplicates). */
	bool bCarryBroadcast = false;

	/** Blends the passenger's view to ViewCamera and back. Active between grab fire and recall. */
	FRopeDemoViewTargetSwitcher ViewSwitcher;
};
