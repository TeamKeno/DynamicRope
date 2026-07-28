// Copyright Epic Games, Inc. All Rights Reserved.
//
// Demo rope lever: a wrappable handle that is thrown over with the rope and then physically pulled
// to toggle On and Off, rotating a linked target actor by a configured offset. It is the "pull to
// actuate" unit of the demo, the counterpart of the pressure plate's "place to actuate".
//
// The handle carries a URopeWrapTargetComponent, so any rope can wrap it through the normal
// contact-to-wrap path. Which rope will wrap it cannot be known in advance, so instead of polling
// every rope the lever subscribes to the subsystem's central signals, OnAnyRopeWrapped and
// OnAnyRopeReleased, and keeps the set of ropes currently wrapping its own handle, the same pattern
// URopeRagdollResponseComponent uses.
//
// The handle only moves under a plausible pull, which is the point of the gimmick: wrapping alone
// does nothing. A pull is *ignited* per rope by three gates:
//  1) the rope is taut (IsPullTaut), so a slack rope lying over the handle has no effect;
//  2) the authoritative constraint tension is at or above PullTensionThreshold;
//  3) the pull direction, from the handle tip towards the rope's hand end, runs along the tip's
//     swing arc — the alignment is the dot product against the arc tangent; its magnitude must reach
//     MinPullAlignment and its sign picks the swing direction, so the handle can be hauled towards
//     either end of its travel depending on which side the rope pulls from.
// Ignition and sustain are split because the swing itself creates slack: as the tip yields towards
// the hand, the chain momentarily slackens, tension collapses, and gates re-checked every frame
// would oscillate — the lever stalling against its own motion, never completing the travel. So once
// ignited, the pull is sustained for PullGraceTime with only the alignment gate re-checked; every
// frame that passes the full gates refreshes the grace, and actually letting go stops the swing
// within the grace window.
// The swing rate is scaled by the alignment, which reads as the handle following the player's pull
// rather than playing a canned animation. Releasing the pull lets the handle spring back to rest.
//
// TargetActor mirrors the handle in real time: its rotation is the initial rotation slerped towards
// initial plus TargetRotationOffset by the handle's travel, so the prop rises exactly as far as the
// lever has been pulled and settles back as the handle springs home — the lever is a crank, not a
// button. Holding the prop up therefore means holding the pull, unless bHoldAtEnd holds the handle
// where the pull left it, which turns a completed pull into a persistent state: a reverse pull from
// the far side hauls the handle back down its travel — reaching rest broadcasts the Off transition —
// and SetOn(false) drops the hold from script. Independently of that, reaching the end of the travel
// still toggles the logical On/Off
// state and broadcasts OnLeverStateChanged for reactions that do want a discrete trigger; the latch
// disarms until the handle has returned near rest, so holding it pinned at the end cannot
// machine-gun the toggle.
//
// The components carry no meshes and no transforms: the frame and handle geometry, including the
// pivot placement, is authored on the Blueprint subclass or the placed instance, since the project
// supplies its own assets. The handle is assumed to extend along the pivot's local +Z;
// HandleLength tells the pull gate where the tip is. The swing itself is pitch about the pivot's
// local Y axis, so the tip travels in the actor's local X-Z plane: the rest pose leans it towards
// +X and pulling swings it towards -X, which is the side the player pulls from.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
// FRopeWrappedEventInfo and ERopeReleaseReason, the central signal payloads.
#include "Core/RopeLifecycleTypes.h"
#include "RopeDemoLever.generated.h"

class URopeComponent;
class URopeWrapTargetComponent;
class USceneComponent;
class UStaticMeshComponent;

/** Fired when the lever toggles, at the state transition. bOn is the state after the toggle. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoLeverStateSignature,
	ARopeDemoLever*, Lever, bool, bOn);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Lever"))
class DYNAMICROPE_API ARopeDemoLever : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoLever();

	//~ AActor. Subscribes to and unsubscribes from the central wrap and release signals.
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Whether the lever is On, as a logical state, independent of where the handle currently is. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsOn() const { return bOn; }

	/** Whether at least one rope is currently wrapping the handle. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsWrapped() const { return WrappingRopes.Num() > 0; }

	/** The handle's travel as 0 at rest to 1 fully pulled, for HUD, audio and debug display. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	float GetPullProgress() const;

	/** Forces the logical lever state, for cheats and Blueprint scripting. Only the toggle event
	 *  depends on it; the target mirrors the handle, which is left to spring back on its own. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void SetOn(bool bNewOn);

	/** Broadcast when the lever toggles, for reactions that want a discrete trigger on top of the
	 *  continuous target rotation. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoLeverStateSignature OnLeverStateChanged;

	//~ Target -----------------------------------------------------------------

	/** The actor the lever drives. Its rotation mirrors the handle in real time: at rest it sits at
	 *  its BeginPlay rotation, fully pulled it is offset by TargetRotationOffset, and in between it
	 *  follows the handle's travel. Leave it empty to react through OnLeverStateChanged instead. */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo|Target")
	TObjectPtr<AActor> TargetActor = nullptr;

	/** How far the target is rotated at the fully pulled end of the travel, relative to the rotation
	 *  it had at BeginPlay. The default raises a flat-lying prop upright by 90 degrees of pitch. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Target")
	FRotator TargetRotationOffset = FRotator(90.0f, 0.0f, 0.0f);

	/** Whether the offset is applied about the target's own axes, the default, or about the world
	 *  axes. About its own axes, a yawed prop still tips over its own edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Target")
	bool bOffsetInLocalSpace = true;

	/** Ease exponent of the handle-travel to target-rotation mapping. 1 follows the handle linearly;
	 *  higher values move the prop least near the ends of the travel, which reads as weight while
	 *  staying a direct, real-time coupling. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Target", meta = (ClampMin = "1.0", ClampMax = "5.0"))
	float TargetEaseExponent = 2.0f;

	//~ Pull tuning -------------------------------------------------------------

	/** Only count a pull while the rope reports itself taut (IsPullTaut). On by default, which stops
	 *  a slack rope draped over the handle from moving it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Pull")
	bool bRequireTautPull = true;

	/** Minimum authoritative constraint tension (kg*cm/s^2, GetConstraintTension) for a pull to
	 *  count. 0, the default, accepts any taut pull; raise it to demand a hard yank. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Pull", meta = (ClampMin = "0.0"))
	float PullTensionThreshold = 0.0f;

	/** How long an ignited pull keeps driving after the taut and tension gates drop out (s). The
	 *  swing itself slackens the rope — the tip yields towards the hand — so without a grace the
	 *  lever stalls against its own motion; while it runs only the alignment gate is re-checked,
	 *  and every frame that passes the full gates refreshes it. 0 demands the full gates every
	 *  frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Pull", meta = (ClampMin = "0.0", Units = "s"))
	float PullGraceTime = 0.5f;

	/** Minimum magnitude of the alignment between the pull direction and the handle tip's swing
	 *  arc, as a dot product in -1 to 1. The sign picks the swing direction — positive hauls
	 *  towards the pulled end, negative back towards rest — and below the magnitude the pull is
	 *  treated as sideways noise and ignored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Pull", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinPullAlignment = 0.15f;

	/** Swing speed towards the pulled end at perfect alignment (deg/s); a grazing pull scales it
	 *  down by the alignment. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Pull", meta = (ClampMin = "1.0"))
	float PullAngularSpeed = 90.0f;

	/** Spring-back speed towards rest while not being pulled (deg/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Pull", meta = (ClampMin = "1.0"))
	float ReturnAngularSpeed = 120.0f;

	/** Hold the handle wherever the pull leaves it once a completed pull has turned the lever On,
	 *  instead of springing back, so the target stays rotated with no need to keep holding the
	 *  pull. Releasing the hold takes a deliberate act: a reverse pull from the far side hauls the
	 *  handle back down its travel — reaching rest broadcasts the Off transition — or SetOn(false)
	 *  drops the hold from Blueprint or the details panel button. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Pull")
	bool bHoldAtEnd = false;

	/** Draw the pull gate every frame: the handle tip, the swing arc tangent in yellow, and each
	 *  wrapping rope's pull direction — green when the full gates pass, cyan when only the grace
	 *  sustains it, red when rejected — with the per-gate values printed at the tip. The fastest
	 *  way to see why a pull is not moving the
	 *  handle; a wrong tip position also points at HandleLength or the pivot placement being off.
	 *  Does nothing in shipping builds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Pull")
	bool bDebugDrawPull = false;

	//~ Travel geometry ---------------------------------------------------------

	/** Handle pitch at rest (deg). Negative leans it away from the pull side, which both reads as a
	 *  resting lever and leaves the tip presented for a throw. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "-80.0", ClampMax = "80.0"))
	float RestAngleDeg = -30.0f;

	/** Handle pitch at the fully pulled end of the travel (deg). Reaching it fires the toggle. Must
	 *  be greater than RestAngleDeg. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "-80.0", ClampMax = "80.0"))
	float PulledAngleDeg = 45.0f;

	/** Distance from the pivot to the handle tip along the pivot's local +Z (cm). The pull gate uses
	 *  it to place the tip, so it should match the authored handle mesh. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "1.0", Units = "cm"))
	float HandleLength = 130.0f;

protected:
	/** The plain root; the fixed and moving parts hang side by side below it. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USceneComponent> Root = nullptr;

	/** The fixed frame the handle pivots on. Mesh and transform are authored on the subclass or the
	 *  instance. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Frame = nullptr;

	/** The hinge. Only its pitch changes; the handle rides on it. Place it where the handle should
	 *  rotate about. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USceneComponent> Pivot = nullptr;

	/** The handle bar, the wrap target the rope is thrown over. Authored to extend along the pivot's
	 *  local +Z. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Handle = nullptr;

	/** Serves the handle as wrappable colliders, which is what lets a rope wrap and hold it. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<URopeWrapTargetComponent> WrapTarget = nullptr;

private:
	//~ Handlers for the central signals on URopeSimSubsystem.
	/** Any rope in the world established a wrap. If it wrapped this lever's handle, the rope joins
	 *  the engagement set that drives the pull. */
	void HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info);
	/** Any rope in the world released. If it was one of the ropes wrapping this handle it leaves the
	 *  engagement set. */
	void HandleAnyRopeReleased(const URopeComponent* Rope, const USceneComponent* WrappedMesh, FName Bone,
		ERopeReleaseReason Reason);

	/** The strongest signed swing rate any wrapping rope produces this frame (deg/s, positive
	 *  towards the pulled end), 0 when no rope ignites or sustains a pull. bOutIgnited reports
	 *  whether at least one rope passed the full taut, tension and alignment gates this frame,
	 *  which is what refreshes the sustain grace. */
	float ComputePullRate(bool& bOutIgnited) const;

	/** The handle tip's world position with the pivot at the given pitch. Evaluating it at two nearby
	 *  angles also yields the swing arc tangent without any axis sign bookkeeping. */
	FVector ComputeTipWorld(float AngleDeg) const;

	/** Flips bOn, logs and broadcasts. Reaching the pulled end of the travel calls this. */
	void ToggleFromPull();

	/** Mirrors the handle's travel onto the target actor's rotation, applied whenever the travel
	 *  changed this frame. */
	void UpdateTargetRotation();

	/** The ropes currently wrapping the handle, maintained from the central signals. Weak, since a
	 *  rope can be destroyed while still wrapped; expired entries are pruned each tick. */
	TSet<TWeakObjectPtr<URopeComponent>> WrappingRopes;

	/** The handle's current pitch (deg), from RestAngleDeg to PulledAngleDeg. */
	float CurrentAngleDeg = 0.0f;

	/** Time left on the ignited pull's sustain window (s). While positive, only the alignment gate
	 *  is re-checked; refreshed to PullGraceTime whenever a rope passes the full gates. */
	float PullGraceRemaining = 0.0f;

	/** Whether reaching the pulled end may fire the toggle. Disarmed on firing and re-armed once the
	 *  handle has returned near rest, so one pull is one toggle. */
	bool bArmed = true;

	bool bOn = false;

	/** The target's rotation at BeginPlay, the rest end of its motion; the fully pulled end is this
	 *  composed with TargetRotationOffset. */
	FQuat TargetInitialQuat = FQuat::Identity;

	/** The handle travel last applied to the target, so an idle lever does not dirty the target's
	 *  transform every frame. -1 forces the first application. */
	float LastAppliedTargetProgress = -1.0f;

	// Signal subscription handles, released in EndPlay.
	FDelegateHandle WrappedHandle;
	FDelegateHandle ReleasedHandle;
};
