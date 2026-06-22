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

	// 1) Base pose (un-dragged): either the hand-placed rest transform, or the swing of it.
	FVector BaseLoc;
	FQuat BaseRot;

	if (!bAutoSwing)
	{
		// Recover the rest pose by removing the current drag offset, so dragging never
		// drifts the rest anchor and the spring can always pull back to where it was placed.
		BaseLoc = GetActorLocation() - DragOffset;
		BaseRot = GetActorQuat();
		RestTransform.SetLocation(BaseLoc);
		RestTransform.SetRotation(BaseRot);
		SwingElapsed = 0.0f;
	}
	else
	{
		SwingElapsed += DeltaSeconds;

		const float Period = FMath::Max(SwingPeriod, 0.05f);
		const float Phase = FMath::Sin(2.0f * PI * SwingElapsed / Period);
		const float AngleRad = FMath::DegreesToRadians(SwingAngleDeg * Phase);

		const FVector AxisWorld = RestTransform.TransformVectorNoScale(SwingAxis).GetSafeNormal(1e-4f, FVector::RightVector);
		const FQuat SwingQuat(AxisWorld, AngleRad);

		const FVector PivotWorld = RestTransform.TransformPosition(SwingPivotOffset);
		const FVector RestLoc = RestTransform.GetLocation();

		BaseLoc = PivotWorld + SwingQuat.RotateVector(RestLoc - PivotWorld);
		BaseRot = SwingQuat * RestTransform.GetRotation();
	}

	// 2) S4: integrate the rope's pull as a soft body — impulse → velocity, spring back to
	//    rest, damping. Lets the limb get dragged but recover when the pull eases.
	if (bDraggable)
	{
		const float Dt = FMath::Min(DeltaSeconds, 1.0f / 30.0f);
		DragVelocity += PendingImpulse / FMath::Max(Mass, 0.1f);
		DragVelocity += -ReturnStiffness * DragOffset * Dt;   // spring toward rest pose
		DragVelocity *= FMath::Exp(-DragDamping * Dt);        // velocity damping
		DragOffset += DragVelocity * Dt;
	}
	else
	{
		DragOffset = FVector::ZeroVector;
		DragVelocity = FVector::ZeroVector;
	}
	PendingImpulse = FVector::ZeroVector;

	// 3) Final pose = base pose + accumulated drag.
	SetActorLocationAndRotation(BaseLoc + DragOffset, BaseRot);
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

void ARopePoCCapsuleActor::GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const
{
	FRopeCapsule Cap;
	GetCapsuleSegment(Cap.A, Cap.B, Cap.Radius);
	OutCapsules.Add(Cap);
}

void ARopePoCCapsuleActor::ApplyRopeReaction(const FVector& WorldImpulse, const FVector& /*WorldLocation*/)
{
	if (!bDraggable)
	{
		return;
	}
	// Accumulate; Tick integrates it (tick order between rope and capsule is undefined).
	PendingImpulse += WorldImpulse;
}
