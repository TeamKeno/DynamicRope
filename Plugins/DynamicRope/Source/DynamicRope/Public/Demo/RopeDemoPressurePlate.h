// Copyright Epic Games, Inc. All Rights Reserved.
//
// Demo pressure plate. It presses down while something is resting on it and returns when the plate
// clears, which is the smallest unit of a puzzle about moving objects onto plates with the rope.
//
// By default only physically simulating bodies count (bRequireSimulatingPhysics), so a player
// walking onto the plate does not press it and a simulating static mesh or a limp character must be
// placed on it instead. That default exists to make "move it with the rope" the actual requirement
// of the puzzle. Turn it off to let walking onto the plate count as well.
//
// A ragdoll has one body per bone, so a single actor raises many overlap events. Occupancy is
// therefore counted per actor rather than per component, in OverlapCounts. A ragdoll that recovers
// to animation while on the plate also stops simulating without raising any overlap event, so the
// eligibility of every tracked actor is re-evaluated each tick.
//
// There are no content dependencies, only engine basic shapes and a point light. The plugin's demo
// map must not reference project assets if it is to open as-is in a distributed build.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "GameFramework/Actor.h"
#include "RopeDemoPressurePlate.generated.h"

class UBoxComponent;
class UPointLightComponent;
class UStaticMeshComponent;

/** Fired when the plate's pressed state changes. bPressed is whether it is pressed now. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoPlatePressedSignature,
	ARopeDemoPressurePlate*, Plate, bool, bPressed);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Pressure Plate"))
class DYNAMICROPE_API ARopeDemoPressurePlate : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoPressurePlate();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Whether the plate is pressed, that is whether the eligible occupant count has reached
	 *  RequiredOccupants. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsPressed() const { return bPressed; }

	/** How many eligible actors are occupying the plate, for HUD and debug display. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetOccupantCount() const { return OccupantCount; }

	/** The eligible occupying actors, counted per actor and excluding destroyed ones, on the same
	 *  basis as the pressed test. A snare uses this to ask which target stepped on the plate. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	TArray<AActor*> GetQualifyingOccupants() const;

	/** Broadcast when the pressed state changes. ARopeDemoDoor subscribes to this. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoPlatePressedSignature OnPlatePressedChanged;

	/** How many occupying actors are needed to press the plate. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo", meta = (ClampMin = "1"))
	int32 RequiredOccupants = 1;

	/** Count only physically simulating bodies as occupants. Turn it off to accept a pawn that walked
	 *  on as well. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bRequireSimulatingPhysics = false;

	/** When set, only actors carrying this tag count as occupants; leave it empty to ignore tags. Use
	 *  it to build a plate that accepts one specific object. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	FName RequiredActorTag = NAME_None;

	/** How far the pad sinks when pressed (cm). Presentation only, with no bearing on the test. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (ClampMin = "0.0", Units = "cm"))
	float PressDepth = 8.0f;

	/** How fast the pad sinks and rises (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (ClampMin = "1.0"))
	float PressSpeed = 40.0f;

	/** Whether to use the indicator light that signals the pressed state by colour. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation")
	bool bUseIndicatorLight = true;

	/** Light colour while not pressed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (EditCondition = "bUseIndicatorLight"))
	FLinearColor IdleColor = FLinearColor(1.0f, 0.25f, 0.1f);

	/** Light colour while pressed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (EditCondition = "bUseIndicatorLight"))
	FLinearColor PressedColor = FLinearColor(0.15f, 1.0f, 0.3f);

protected:
	/** The fixed surround, which visually frames where the pad sinks to. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Frame = nullptr;

	/** The pad that actually sinks. The presentation only moves this component's relative Z. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Pad = nullptr;

	/** Occupancy detection volume, covering the space above the pad. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UBoxComponent> Trigger = nullptr;

	/** The pressed-state indicator. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UPointLightComponent> IndicatorLight = nullptr;

private:
	UFUNCTION()
	void HandleBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	UFUNCTION()
	void HandleEndOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);

	/** Whether an occupant satisfies the tag and physics simulation conditions. */
	bool IsQualifyingOccupant(const AActor* OtherActor) const;

	/** Re-evaluates the tracked actors, updates OccupantCount and bPressed, and broadcasts on a
	 *  change. */
	void RefreshPressedState();

	/** Applies the indicator colour for the current state. */
	void ApplyIndicatorColor();

	/** Overlap count per actor, so a ragdoll's many bone bodies collapse into a single actor. */
	TMap<TWeakObjectPtr<AActor>, int32> OverlapCounts;

	int32 OccupantCount = 0;
	bool  bPressed = false;

	/** The pad's current relative Z while interpolating. 0 is at rest and -PressDepth is fully
	 *  pressed. */
	float CurrentPadOffset = 0.0f;
};
