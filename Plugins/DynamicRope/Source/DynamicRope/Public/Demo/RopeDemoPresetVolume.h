// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Demo preset volume. Applies a URopePreset to the rope held by any actor that enters, so each room
// can give the rope a different character, whether free simulation, capture, or a grappling hook.
// The point is to show that tuning alone turns it into a completely different rope. Assign an
// ExitPreset to revert on the way out.
//
// ApplyPreset only succeeds in the Free and Loaded phases, because swapping values mid-flight or
// mid-wrap makes the simulation jump. A rope that is flying or wrapping when the volume is entered
// therefore fails to apply, and failing silently would look like walking through the door and
// nothing changing. With bApplyWhenRopeSettles, on by default, such a rope is queued and
// OnRopePhaseChanged is subscribed to, so the preset lands the first moment the rope returns to Free
// or Loaded while still inside the volume.
//
// Reverting is not automatic because URopeComponent applies a preset by stamping its values and does
// not keep a pointer to it, so there is no way to know what the rope was before. Restoring is
// therefore only done by naming an ExitPreset explicitly.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "Core/RopeLifecycleTypes.h"
#include "GameFramework/Actor.h"
#include "RopeDemoPresetVolume.generated.h"

class UBoxComponent;
class URopeComponent;
class URopePreset;

/** Fired whenever a preset is actually applied, including after waiting for the rope to settle. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FRopeDemoPresetAppliedSignature,
	ARopeDemoPresetVolume*, Volume, URopeComponent*, Rope, const URopePreset*, Preset);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Preset Volume"))
class DYNAMICROPE_API ARopeDemoPresetVolume : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoPresetVolume();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** Broadcast each time a preset is applied. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoPresetAppliedSignature OnPresetVolumeApplied;

	/** Preset applied on entry. Nothing happens when it is empty. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<URopePreset> Preset = nullptr;

	/** Optional preset applied on exit. Leave it empty to leave the rope as it is. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<URopePreset> ExitPreset = nullptr;

	/** Whether to affect only ropes held by a pawn. Turn it off to apply to the rope of any actor
	 *  entering the volume. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bPawnsOnly = true;

	/**
	 * When the rope is flying or wrapping on entry and the preset is refused, wait while the actor
	 * remains inside the volume and apply it the moment the rope returns to Free or Loaded. Turn it
	 * off to let that attempt simply fail.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bApplyWhenRopeSettles = true;

protected:
	/** The transition volume. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UBoxComponent> Trigger = nullptr;

private:
	UFUNCTION()
	void HandleBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	UFUNCTION()
	void HandleEndOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);

	/** Checks whether a waiting rope has returned to Free or Loaded and retries the application. */
	UFUNCTION()
	void HandleRopePhaseChanged(ERopePhase OldPhase, ERopePhase NewPhase);

	/** Tries to apply the preset to the actor's rope, queueing it when the attempt fails and
	 *  bWaitIfBusy is set. */
	void ApplyToActor(AActor* Actor, const URopePreset* InPreset, bool bWaitIfBusy);

	/** Applies the preset to one rope. Returns true on success. */
	bool ApplyToRope(URopeComponent* Rope, const URopePreset* InPreset);

	/** Removes a rope from the queue and unsubscribes from it. */
	void StopWaitingFor(URopeComponent* Rope);

	/** Overlap count per actor, so multi-body actors such as ragdolls are counted once; the same
	 *  reason ARopeDemoPressurePlate does it. */
	TMap<TWeakObjectPtr<AActor>, int32> OverlapCounts;

	/** Ropes waiting to return to Free or Loaded. */
	TSet<TWeakObjectPtr<URopeComponent>> PendingRopes;
};
