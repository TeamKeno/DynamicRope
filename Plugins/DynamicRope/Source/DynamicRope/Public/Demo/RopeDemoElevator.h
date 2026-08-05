// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Demo rope elevator: a rideable platform that grapples the ceiling and reels itself up. It is the
// physical climb-in demo. A GuaranteedWrap rope attached to the platform, which is a physics body,
// wraps a ceiling anchor, and reeling in plus active pull hauls the platform up towards it. Reeling
// out pays the rope back out and the platform descends under gravity.
//
// State flow:
//   Idle -> fire a guaranteed grapple at the anchor -> Establishing -> wrap confirmed -> Docked
//        -> RequestAscend -> Ascending -> minimum length reached -> stopped at the top
//        -> RequestDescend -> Descending -> maximum length reached -> stopped at the bottom
// The grapple is established once only; from then on the elevator travels by reeling alone and stays
// attached, like a real elevator cable.
//
// Content requirements:
//   - The platform and shapes are engine basic cubes, since the plugin never references /Game
//     content.
//   - The ceiling anchor is the one exception: the rope can only wrap something wrappable, meaning it
//     has a skeletal bone collider or an SDF provider, so AnchorTarget must be assigned such a
//     component from the level, for example a single-bone skeletal ring. Without one it warns and
//     does nothing.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoElevator.generated.h"

class URopeComponent;
class UStaticMeshComponent;
class ARopeDemoPressurePlate;

/** Fired when the elevator reaches the target floor, at the stop transition rather than when the
 *  motion finishes. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoElevatorArrivedSignature,
	ARopeDemoElevator*, Elevator, bool, bAtTop);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Elevator"))
class DYNAMICROPE_API ARopeDemoElevator : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoElevator();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Sends the elevator to the upper floor; it only actually moves once the grapple is established.
	 *  Also exposed as a details panel button. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void RequestAscend() { SetTargetTop(true); }

	/** Sends the elevator to the lower floor. Also exposed as a details panel button. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void RequestDescend() { SetTargetTop(false); }

	/** Flips the target floor, for a single input key or a details panel button. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void ToggleTarget() { SetTargetTop(!bTargetTop); }

	/** Sets the target floor directly: true for the top, false for the bottom. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void SetTargetTop(bool bNewTargetTop);

	/** Whether the grapple is established on the ceiling and the elevator can travel. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsGrappleReady() const { return bGrappleReady; }

	/** Whether the current target is the upper floor. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsTargetTop() const { return bTargetTop; }

	/** Broadcast on arriving at the target floor. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoElevatorArrivedSignature OnElevatorArrived;

	//~ Anchor (the ceiling grapple target) -------------------------------------

	/** The ceiling anchor the ropes wrap. It must be wrappable, meaning it carries a skeletal bone
	 *  collider or an SDF provider, and is assigned in the level. Leaving it empty means the grapple
	 *  never establishes and the elevator never moves, which is logged as a warning. */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<AActor> AnchorTarget = nullptr;

	//~ Trigger (the call button) -----------------------------------------------

	/** Travels up while this pressure plate is pressed and down when it clears, acting as the call
	 *  button. Leave it empty to drive the elevator from Blueprint or code instead. */
	UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<ARopeDemoPressurePlate> CallPlate = nullptr;

	//~ Travel tuning -----------------------------------------------------------

	/** Reel-in speed while ascending (cm/s). The rope shortens and the tether hauls the platform up
	 *  towards the anchor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "1.0", Units = "cm/s"))
	float AscendReelSpeed = 150.0f;

	/** Reel-out speed while descending (cm/s). The rope lengthens and the platform falls under
	 *  gravity. Slower than free fall leaves it hanging from the tether and descending gently; faster
	 *  approaches free fall. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "1.0", Units = "cm/s"))
	float DescendReelSpeed = 120.0f;

	/** Active pull force per cable while ascending, so with four ropes the total is four times this.
	 *  It supplements reeling in when that alone cannot lift the platform's weight. 0 ascends on the
	 *  reel-in and tether alone, which requires the tether's MaxTetherTension to exceed the weight. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "0.0"))
	float ClimbForce = 60000.0f;

	/** Tolerance on reaching the target length (cm). The elevator counts as arrived once the current
	 *  rope length is within this of the minimum or maximum. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "0.1", Units = "cm"))
	float ArrivalTolerance = 5.0f;

	//~ Stability (keeping the platform from tipping) ----------------------------

	/** Locks the platform's pitch and roll so it always stays level, on by default, which stops it
	 *  tilting when a character stands on one side. It leaves the physical travel along the vertical
	 *  free and removes only the degrees of freedom that let it tip, the standard solution for a
	 *  rideable platform. Turn it off for free physics, where it can tilt and overturn. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Stability")
	bool bLockPlatformTilt = true;

	/** Also locks yaw, that is rotation about the vertical, so the platform does not rotate at all and
	 *  keeps a fixed heading. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Stability")
	bool bLockPlatformYaw = false;

	/** Angular damping on the platform, which settles rotation and sway. It slows tipping even with
	 *  the locks disabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Stability", meta = (ClampMin = "0.0"))
	float PlatformAngularDamping = 10.0f;

	/** Platform mass override (kg). 0 uses the mesh default. A heavier platform is more stable because
	 *  a character's load produces relatively less torque, at the cost of needing a correspondingly
	 *  larger ClimbForce and tether tension to lift. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Stability", meta = (ClampMin = "0.0", Units = "kg"))
	float PlatformMass = 0.0f;

protected:
	/** The rideable platform, a physics body and the actor root. It is the receiver of the climb-in
	 *  traction. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Platform = nullptr;

	/** Four GuaranteedWrap ropes that wrap the ceiling anchor, one at each corner of the platform.
	 *  They wrap and reel together, which spreads the load across the corners and tilts far less than a
	 *  single central rope would. All four wrap the same AnchorTarget. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<TObjectPtr<URopeComponent>> Ropes;

private:
	/** The number of cables, one per platform corner. */
	static constexpr int32 NumRopes = 4;

	/** Fires a guaranteed throw at the anchor from every rope not yet wrapped, attempting to establish
	 *  the grapple. */
	void FireGrapples();

	/** Fires a guaranteed throw at the anchor from one rope. True once queued. */
	bool FireGrappleFor(URopeComponent* InRope);

	/** Whether all four ropes are wrapped, which completes the grapple. */
	bool AreAllRopesWrapped() const;

	/** The world position to aim at on the anchor, falling back to the anchor actor's location. */
	FVector ResolveAnchorAimWorld() const;

	/** Applies the stability settings, that is the rotation locks, angular damping and mass, to the
	 *  platform's physics body during BeginPlay. */
	void ApplyPlatformStability();

	/** Pressure plate state change handler, matching the delegate signature. Pressed travels up and
	 *  cleared travels down. */
	UFUNCTION()
	void HandleCallPlateChanged(ARopeDemoPressurePlate* Plate, bool bPressed);

	/** Cooldown before firing again while the grapple is not yet established (s). It retries
	 *  periodically until every rope is wrapped. */
	float EstablishRetryRemaining = 0.0f;

	/** Whether the grapple is wrapped on the ceiling and travel is possible, set once the phase is
	 *  confirmed as Wrapped. */
	bool bGrappleReady = false;

	/** The current target floor: true for the top, false for the bottom. */
	bool bTargetTop = false;

	/** Whether arrival at this target has already been broadcast, which prevents duplicates. */
	bool bArrivedBroadcast = false;
};
