// Copyright Epic Games, Inc. All Rights Reserved.
//
// Demo spread-eagle snare. Several ropes each wrap a different limb of the target and pull outwards,
// holding it spread and immobilized.
//
// State flow:
//   Idle -> TriggerSnare() -> Binding (a guaranteed aimed throw per limb, retried until every rope
//        is wrapped)
//        -> Snared (reel the cables in to SnareLength, spreading the limbs towards their anchors)
//        -> ReleaseSnare() -> Idle (release every wrap and restore the cable lengths)
//
// There are four anchor slots, two arms and two legs, and only as many are used as Bindings holds.
// The default fills all four for a full spread; deleting an entry or clearing its bone stops that
// slot from firing, so leaving two bound restrains the arms alone.
//
// Target requirements:
//   - TargetActor needs a skeletal mesh plus a rope collider provider, capsule or SDF, to be
//     wrappable.
//   - Leaving TargetActor empty and assigning only TriggerPlate enables trap mode: the moment the
//     plate is pressed, an occupying actor with a skeletal mesh is picked up as the target
//     automatically, and releasing the plate clears the target along with the snare.
//   - Limbs can only be dragged by physics while limp. A target with a
//     URopeRagdollResponseComponent goes limp from the wrap event by itself, but the limbs only
//     spread freely if it is already limp before the wrap lands, so bForceRagdollOnSnare, on by
//     default, makes it limp at the moment the ropes fire.
//   - A player-controlled target's view blends to ViewCamera from the moment the ropes fire until
//     the release (bSwitchPlayerViewTarget). An AI target keeps its own camera.
//
// GuaranteedWrap locks onto the first wrappable bone the aim ray hits, so aiming at hand_l can catch
// lowerarm_l if that is in the way; either reads fine as a spread-eagle. What was actually caught is
// recorded in the completion log.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "GameFramework/Actor.h"
#include "Demo/RopeDemoViewTargetSwitcher.h"
#include "RopeDemoSnare.generated.h"

class URopeComponent;
class UStaticMeshComponent;
class USkeletalMeshComponent;
class ARopeDemoPressurePlate;
class UCameraComponent;

/** One snare slot: pull this bone towards that anchor. */
USTRUCT(BlueprintType)
struct FRopeDemoSnareBinding
{
	GENERATED_BODY()

	/** The target bone to wrap, such as hand_l, hand_r, foot_l or foot_r. The slot is unused when
	 *  empty. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	FName Bone = NAME_None;

	/** Where to pull the bone to, in snare actor local space (cm). The anchor marker and the start of
	 *  the rope are placed here. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	FVector AnchorOffset = FVector::ZeroVector;
};

/** Fired when the snare is established, with every slot wrapped, or released. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoSnareStateSignature,
	ARopeDemoSnare*, Snare, bool, bSnared);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Snare"))
class DYNAMICROPE_API ARopeDemoSnare : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoSnare();

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Starts the snare: fires an aimed throw per slot and retries until every rope is wrapped. Also
	 *  exposed as a details panel button. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void TriggerSnare();

	/** Releases the snare, releasing every wrap and restoring the cable lengths. Also exposed as a
	 *  details panel button. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void ReleaseSnare();

	/** Toggles the snare state, for a single input key or a details panel button. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void ToggleSnare();

	/** Whether every slot is wrapped and the snare is established. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsSnared() const { return bSnared; }

	/** How many cables are actually wrapped right now, for progress display. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetBoundRopeCount() const;

	/** How many slots are in use, that is how many Bindings have a bone assigned, up to four. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetActiveBindingCount() const;

	/** The actual target of this snare: an assigned TargetActor always wins, otherwise the target
	 *  picked up from the plate in trap mode. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	AActor* GetEffectiveTargetActor() const;

	/** Broadcast when the snare is established or released. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoSnareStateSignature OnSnareStateChanged;

	//~ Target and slots --------------------------------------------------------

	/** The actor to snare, which must have a skeletal mesh. Assigned in the level and always takes
	 *  priority. Leaving it empty still works as a trap when TriggerPlate is set, picking up whichever
	 *  actor stands on the plate. With neither set, triggering warns and does nothing. */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<AActor> TargetActor = nullptr;

	/** The snare slots, up to four. The default fills all four limbs for a full spread; reduce it by
	 *  deleting entries or clearing their bone. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<FRopeDemoSnareBinding> Bindings;

	/** Snares when this pressure plate is pressed and releases when it clears, acting as the trap
	 *  trigger. Leave it empty to drive the snare from Blueprint or code instead. */
	UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<ARopeDemoPressurePlate> TriggerPlate = nullptr;

	/** Snares immediately during BeginPlay, for showing or measuring an already-sprung trap with no
	 *  trigger. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bSnareOnBeginPlay = false;

	//~ Snare tuning ------------------------------------------------------------

	/** Reel-in speed used to spread the limbs once wrapped (cm/s). The rope shortens and drags the
	 *  bone towards its anchor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "1.0", Units = "cm/s"))
	float SnareReelSpeed = 120.0f;

	/** Cable length once the snare is established (cm). 0 reels each rope to its own MinRopeLength.
	 *  Shorter values spread the target more forcefully. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "0.0", Units = "cm"))
	float SnareLength = 0.0f;

	/** Tolerance on reaching the target length (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "0.1", Units = "cm"))
	float ArrivalTolerance = 5.0f;

	/** Active pull force per slot while snared. 0 relies on the reel-in and tether alone; add force
	 *  when reeling by itself does not spread the limbs far enough. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "0.0"))
	float LimbPullForce = 0.0f;

	/** Makes the target limp at the moment the ropes fire, when it has a
	 *  URopeRagdollResponseComponent. Turn it off to leave it to the automatic transition on the wrap
	 *  event, which can leave the limbs held by animation and spreading less. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare")
	bool bForceRagdollOnSnare = true;

	/** Releases automatically this long after the snare is established with every slot wrapped (s).
	 *  0 disables it.
	 *  After releasing, a target still standing on the plate is not snared again: rearming requires
	 *  the plate's next pressing edge, that is stepping off and back on, or a manual TriggerSnare. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "0.0", Units = "s"))
	float AutoReleaseDelay = 0.0f;

	//~ View camera -------------------------------------------------------------

	/** Blend the player's view to ViewCamera from the moment the ropes fire until the release. Only
	 *  a player-controlled target switches; an AI target leaves the view alone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Camera")
	bool bSwitchPlayerViewTarget = true;

	/** Blend time into ViewCamera (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Camera", meta = (ClampMin = "0.0", Units = "s"))
	float ViewBlendInTime = 0.75f;

	/** Blend time back to the target's own view (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Camera", meta = (ClampMin = "0.0", Units = "s"))
	float ViewBlendOutTime = 0.75f;

protected:
	/** The fixed root the anchors are measured from. Slot markers and ropes attach here. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USceneComponent> Base = nullptr;

	/** Four anchor markers that visualize the slot positions. Unused slots are hidden. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<TObjectPtr<UStaticMeshComponent>> AnchorMarkers;

	/** Four GuaranteedWrap ropes, one per slot. Only as many fire and reel as Bindings fills. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<TObjectPtr<URopeComponent>> Ropes;

	/** Capture camera the target's view blends to while the snare holds it. Reframe it in the
	 *  Blueprint or level; the default looks back at the origin, where the bindings assume the
	 *  target stands. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UCameraComponent> ViewCamera = nullptr;

private:
	/** The maximum number of slots, two arms plus two legs. */
	static constexpr int32 MaxBindings = 4;

	/** Whether slot i is in use: within the Bindings range, with a bone assigned and a valid rope. */
	bool IsSlotActive(int32 SlotIndex) const;

	/** Places the anchor markers and ropes at the AnchorOffsets from Bindings and hides unused
	 *  slots. */
	void ApplyBindingLayout();

	/** Fires a guaranteed throw at the target bone from every slot that is not yet wrapped. */
	void FireSnareRopes();

	/** Fires a guaranteed throw at the target bone from one slot. True once queued. */
	bool FireSnareRopeFor(int32 SlotIndex);

	/** Whether every active slot is wrapped, which establishes the snare. False when no slot is
	 *  active. */
	bool AreAllBoundRopesWrapped() const;

	/** The target's first skeletal mesh, or nullptr. The target is GetEffectiveTargetActor(). */
	USkeletalMeshComponent* ResolveTargetMesh() const;

	/** Picks up an occupying actor with a skeletal mesh from TriggerPlate as the target when
	 *  TargetActor is empty. */
	void ResolveAutoTargetFromPlate();

	/** Makes the target limp immediately when it has a ragdoll response component, subject to
	 *  bForceRagdollOnSnare. */
	void ForceTargetRagdoll();

	/** Applies the established or released state and broadcasts on a change. */
	void SetSnared(bool bNewSnared);

	/** Pressure plate state change handler, matching the delegate signature. Pressed snares and
	 *  cleared releases. */
	UFUNCTION()
	void HandleTriggerPlateChanged(ARopeDemoPressurePlate* Plate, bool bPressed);

	/** The target picked up from the plate in trap mode, cleared on release. Ignored while an assigned
	 *  TargetActor exists. */
	TWeakObjectPtr<AActor> AutoTargetActor;

	/** Automatic release countdown (s), seeded with AutoReleaseDelay when the snare is established and
	 *  calling ReleaseSnare on reaching 0. */
	float AutoReleaseRemaining = 0.0f;

	/** Whether a snare attempt is in progress, between TriggerSnare and ReleaseSnare. */
	bool bTriggered = false;

	/** Whether every active slot is wrapped and the snare is established. */
	bool bSnared = false;

	/** Cooldown before firing again while the snare is not yet established (s). */
	float FireRetryRemaining = 0.0f;

	/** Blends the target's view to ViewCamera and back. Active between trigger and release. */
	FRopeDemoViewTargetSwitcher ViewSwitcher;
};
