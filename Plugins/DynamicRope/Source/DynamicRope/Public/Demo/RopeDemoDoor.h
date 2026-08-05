// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Demo door. Opens once the pressure plates it is linked to are pressed, either all of them or as
// many as RequiredPressedCount asks for. It is the backbone of demo progression: clearing a stage
// opens the next area.
//
// Plates can be linked in two ways:
//  1) Assign the level's plate actors directly in the Plates array, which makes it explicit and
//     visible which plates drive this door.
//  2) Set PlateTag and every plate carrying that tag is collected from the world during BeginPlay,
//     so placing them in the level is the only wiring needed.
// Both can be used together, and duplicates are merged.
//
// With bStayOpen, on by default, a door that has opened stays open even when the plates clear.
// Solving a puzzle usually does not need undoing, and it removes the need to keep an object sitting
// on a plate while walking through. Turn it off to close again the moment the plates clear.
//
// There are no content dependencies, only engine basic shapes. The leaf slides along a local axis by
// OpenOffset; sliding rather than swinging looks acceptable without frame or hinge assets.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoDoor.generated.h"

class ARopeDemoPressurePlate;
class UStaticMeshComponent;

/** Fired when the door opens or closes, at the state transition rather than when the motion
 *  finishes. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoDoorStateSignature,
	ARopeDemoDoor*, Door, bool, bOpen);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Door"))
class DYNAMICROPE_API ARopeDemoDoor : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoDoor();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Whether the door is open, as a logical state, even while the motion is still playing. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsOpen() const { return bOpen; }

	/** How many plates are currently pressed. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetPressedPlateCount() const;

	/** How many plates the open condition needs, which is RequiredPressedCount, or all of them when
	 *  that is 0. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetRequiredPlateCount() const;

	/** Forces the door open or closed regardless of the plates, for cheats, cutscenes and Blueprint
	 *  scripting. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void SetOpen(bool bNewOpen);

	/** Broadcast on a door state transition. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoDoorStateSignature OnDoorStateChanged;

	/** The pressure plates that open this door, assigned in the level. */
	UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<TObjectPtr<ARopeDemoPressurePlate>> Plates;

	/** When set, plates carrying this tag are additionally collected from the world during BeginPlay.
	 *  Leave it empty to collect nothing. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	FName PlateTag = NAME_None;

	/** How many pressed plates are required to open. 0 requires every linked plate, which is the
	 *  default for a two-plate puzzle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo", meta = (ClampMin = "0"))
	int32 RequiredPressedCount = 0;

	/** Whether to stay open once opened. Turn it off to close the moment the condition breaks. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bStayOpen = true;

	/** Local offset the leaf moves by when opening; the default raises it by 220 cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation")
	FVector OpenOffset = FVector(0.0f, 0.0f, 220.0f);

	/** Leaf movement speed (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (ClampMin = "1.0"))
	float OpenSpeed = 120.0f;

protected:
	/** The fixed door frame. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Frame = nullptr;

	/** The leaf that actually moves. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Leaf = nullptr;

private:
	/** Called when a plate is pressed or released; matches the delegate signature. */
	UFUNCTION()
	void HandlePlatePressedChanged(ARopeDemoPressurePlate* Plate, bool bPressed);

	/** Re-evaluates the condition, updates bOpen and broadcasts on a change. */
	void EvaluateOpenCondition();

	/** Collects the world's plates by PlateTag and merges them into Plates, removing duplicates. */
	void GatherTaggedPlates();

	/** The leaf's closed position, which is its relative location at BeginPlay. */
	FVector ClosedLeafLocation = FVector::ZeroVector;

	bool bOpen = false;
};
