// Copyright Epic Games, Inc. All Rights Reserved.
//
// Test and debug utility that drives its owning actor back and forth on a sine, optionally rotating
// it, so the way a rope reacts to a moving body can be seen directly. The static body provider
// currently captures WorldStatic objects only, as a fresh static snapshot each frame with no surface
// velocity and no relative continuous collision, so this component moves a movable WorldStatic mesh
// to observe that behaviour and its limits: moved slowly the rope rides along, and moved quickly the
// missing surface velocity shows up as no drag and visible tunnelling.
//
// Usage: add it to a movable static mesh actor whose collision object type is WorldStatic and which
// has simple collision. Leave it as WorldStatic, since the provider does not yet collect dynamic
// bodies and would not pick up a WorldDynamic actor at all.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RopeTestMoverComponent.generated.h"

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeTestMoverComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeTestMoverComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Travel amplitude. The owning actor oscillates on a sine between its start location plus and
	 *  minus this offset. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Test")
	FVector MoveOffset = FVector(200.0f, 0.0f, 0.0f);

	/** The period of one full oscillation (s). Smaller is faster, and moving fast is what makes the
	 *  missing surface velocity visible as tunnelling and absent drag. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Test", meta = (ClampMin = "0.05", Units = "s"))
	float Period = 3.0f;

	/** Rotation per second (degrees). 0 disables rotation. For observing rope behaviour on a rotating
	 *  surface. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Test")
	FRotator RotationRate = FRotator::ZeroRotator;

	/** Whether to apply the travel along axes local to the starting rotation. False uses world axes. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Test")
	bool bMoveInLocalSpace = true;

private:
	FVector  StartLocation = FVector::ZeroVector;
	FRotator StartRotation = FRotator::ZeroRotator;
	float    ElapsedTime = 0.0f;
};
