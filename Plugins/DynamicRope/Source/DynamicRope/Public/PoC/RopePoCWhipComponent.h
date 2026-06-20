// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. Everything under PoC/ is disposable.
// Attaches a rope to the character's hand and whips it with a montage on left-click.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RopePoCWhipComponent.generated.h"

class ARopePoCActor;
class UAnimMontage;
class USkeletalMeshComponent;

/**
 * Drop this on a character to test the "whip" feel: the rope's pinned end is attached
 * to a hand bone, so the free end trails like a whip when a swing montage plays.
 */
UCLASS(ClassGroup = (DynamicRopePoC), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopePoCWhipComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopePoCWhipComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Rope whose start follows the hand. If null and bSpawnRopeIfMissing, one is spawned at the hand. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Whip")
	TObjectPtr<ARopePoCActor> Rope = nullptr;

	/** Spawn a default rope if Rope is not assigned. */
	UPROPERTY(EditAnywhere, Category = "Whip")
	bool bSpawnRopeIfMissing = true;

	/** Hand bone/socket the rope's pinned end follows. */
	UPROPERTY(EditAnywhere, Category = "Whip")
	FName HandSocketName = TEXT("hand_r");

	/** Montage played by Swing(). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Whip")
	TObjectPtr<UAnimMontage> SwingMontage = nullptr;

	/** Playback speed of the swing montage. 1 = normal, 2 = twice as fast, 0.5 = half speed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Whip", meta = (ClampMin = "0.01", UIMin = "0.1", UIMax = "3.0"))
	float MontagePlayRate = 1.0f;

	/** Try to bind Left Mouse Button → Swing() automatically (no input asset needed). */
	UPROPERTY(EditAnywhere, Category = "Whip")
	bool bAutoBindLeftMouse = true;

	/** Attach the rope to the hand and pin its start there. */
	UFUNCTION(BlueprintCallable, Category = "Whip")
	void AttachRopeToHand();

	/** Play the swing montage. Bind this to your own left-click input if auto-bind is off. */
	UFUNCTION(BlueprintCallable, Category = "Whip")
	void Swing();

private:
	bool bLeftMouseBound = false;

	USkeletalMeshComponent* GetOwnerMesh() const;
	bool TryBindLeftMouse();
};
