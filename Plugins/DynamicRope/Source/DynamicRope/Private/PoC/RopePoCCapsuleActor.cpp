// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. See Docs/PoC/01_PostWrapModel.md.

#include "PoC/RopePoCCapsuleActor.h"

#include "Components/CapsuleComponent.h"

ARopePoCCapsuleActor::ARopePoCCapsuleActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	Capsule = CreateDefaultSubobject<UCapsuleComponent>(TEXT("Capsule"));
	SetRootComponent(Capsule);

	// Visualization only — the rope solver reads the geometry directly, so this
	// component itself does no physics collision.
	Capsule->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Capsule->SetCapsuleSize(Radius, HalfHeight);
	Capsule->ShapeColor = FColor::Green;
	Capsule->bDrawOnlyIfSelected = false; // always visible in editor
	Capsule->SetHiddenInGame(false);
}

void ARopePoCCapsuleActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	if (Capsule)
	{
		Capsule->SetCapsuleSize(Radius, HalfHeight);
	}
}

void ARopePoCCapsuleActor::BeginPlay()
{
	Super::BeginPlay();
	RestTransform = GetActorTransform();
	SwingElapsed = 0.0f;
}

#if WITH_EDITOR
void ARopePoCCapsuleActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// Re-anchor the swing to wherever the capsule currently sits.
	RestTransform = GetActorTransform();
	SwingElapsed = 0.0f;
}
#endif

void ARopePoCCapsuleActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!bAutoSwing)
	{
		// Not swinging: keep the rest pose tracking the (possibly hand-placed) transform.
		RestTransform = GetActorTransform();
		SwingElapsed = 0.0f;
		return;
	}

	SwingElapsed += DeltaSeconds;

	const float Period = FMath::Max(SwingPeriod, 0.05f);
	const float Phase = FMath::Sin(2.0f * PI * SwingElapsed / Period);
	const float AngleRad = FMath::DegreesToRadians(SwingAngleDeg * Phase);

	const FVector AxisWorld = RestTransform.TransformVectorNoScale(SwingAxis).GetSafeNormal(1e-4f, FVector::RightVector);
	const FQuat SwingQuat(AxisWorld, AngleRad);

	const FVector PivotWorld = RestTransform.TransformPosition(SwingPivotOffset);
	const FVector RestLoc = RestTransform.GetLocation();

	const FVector NewLoc = PivotWorld + SwingQuat.RotateVector(RestLoc - PivotWorld);
	const FQuat NewRot = SwingQuat * RestTransform.GetRotation();

	SetActorLocationAndRotation(NewLoc, NewRot);
}

void ARopePoCCapsuleActor::GetCapsuleSegment(FVector& OutA, FVector& OutB, float& OutRadius) const
{
	OutRadius = Radius;

	const FVector Center = GetActorLocation();
	const FVector Up = GetActorQuat().GetAxisZ(); // capsule axis is local Z

	// Inner segment half-length: the cylindrical part between the two hemispherical caps.
	const float SegmentHalf = FMath::Max(0.0f, HalfHeight - Radius);
	OutA = Center - Up * SegmentHalf;
	OutB = Center + Up * SegmentHalf;
}
