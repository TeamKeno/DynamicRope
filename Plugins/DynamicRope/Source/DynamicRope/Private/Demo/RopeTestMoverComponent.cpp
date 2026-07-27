// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeTestMoverComponent.h"
#include "GameFramework/Actor.h"

URopeTestMoverComponent::URopeTestMoverComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	// The actor is moved first, in the default tick group of PrePhysics, and the rope simulation in TG_PostPhysics then reads the updated position.
}

void URopeTestMoverComponent::BeginPlay()
{
	Super::BeginPlay();
	if (const AActor* Owner = GetOwner())
	{
		StartLocation = Owner->GetActorLocation();
		StartRotation = Owner->GetActorRotation();
	}
}

void URopeTestMoverComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	ElapsedTime += DeltaTime;

	// A sinusoidal oscillation with the move offset as its amplitude. In local space the offset is rotated by the starting rotation.
	const float   Alpha = FMath::Sin(2.0f * PI * ElapsedTime / FMath::Max(Period, 0.05f));
	const FVector Offset = bMoveInLocalSpace ? StartRotation.RotateVector(MoveOffset * Alpha) : (MoveOffset * Alpha);
	const FRotator Rot = StartRotation + RotationRate * ElapsedTime;

	// Moved as a teleport, with no sweep, for observation in tests. The mesh has to be movable for it actually to move.
	Owner->SetActorLocationAndRotation(StartLocation + Offset, Rot);
}
