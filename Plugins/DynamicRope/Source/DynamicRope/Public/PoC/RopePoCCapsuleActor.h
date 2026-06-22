// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. Everything under PoC/ is disposable.
// S1: a single capsule collider the rope can rest on. Reused as the "moving arm" in S2.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PoC/RopeCapsuleProvider.h"
#include "RopePoCCapsuleActor.generated.h"

class UCapsuleComponent;

/**
 * Proof-of-concept capsule collider.
 * Exposes its world-space swept-sphere segment (two centers + radius) so the rope
 * solver can push particles out of it. Stand-in for a character limb's physics-asset capsule.
 */
UCLASS()
class DYNAMICROPE_API ARopePoCCapsuleActor : public AActor, public IRopeCapsuleProvider
{
	GENERATED_BODY()

public:
	ARopePoCCapsuleActor();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual bool ShouldTickIfViewportsOnly() const override { return true; } // swing in editor viewport
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	/** Capsule radius (cm). */
	UPROPERTY(EditAnywhere, Category = "Capsule", meta = (ClampMin = "1.0", UIMin = "1.0", Units = "cm"))
	float Radius = 30.0f;

	/** Capsule half-height including the hemispherical caps (cm). */
	UPROPERTY(EditAnywhere, Category = "Capsule", meta = (ClampMin = "1.0", UIMin = "1.0", Units = "cm"))
	float HalfHeight = 60.0f;

	//~ Auto-swing (S2) — animate the capsule like a moving limb -----------

	/** Oscillate the capsule about a pivot so the rope must keep up with a moving target. */
	UPROPERTY(EditAnywhere, Category = "Swing")
	bool bAutoSwing = false;

	/** Peak swing angle to each side (deg). */
	UPROPERTY(EditAnywhere, Category = "Swing", meta = (EditCondition = "bAutoSwing", ClampMin = "0.0", Units = "deg"))
	float SwingAngleDeg = 60.0f;

	/** Time for one full back-and-forth swing (s). Lower = faster = harder on the solver. */
	UPROPERTY(EditAnywhere, Category = "Swing", meta = (EditCondition = "bAutoSwing", ClampMin = "0.05", Units = "s"))
	float SwingPeriod = 2.0f;

	/** Local axis the capsule rotates around (default Y → swings in the X-Z plane). */
	UPROPERTY(EditAnywhere, Category = "Swing", meta = (EditCondition = "bAutoSwing"))
	FVector SwingAxis = FVector(0.0f, 1.0f, 0.0f);

	/** Pivot point in local space. Default (0,0,HalfHeight) ≈ top cap = "shoulder", so it sweeps like an arm. */
	UPROPERTY(EditAnywhere, Category = "Swing", meta = (EditCondition = "bAutoSwing"))
	FVector SwingPivotOffset = FVector(0.0f, 0.0f, 60.0f);

	//~ Pull / draggable (S4) — respond to the rope's contact reaction ------

	/** Let the rope drag this capsule (the "limb gets pulled"). Reaction is integrated as a soft body. */
	UPROPERTY(EditAnywhere, Category = "Pull")
	bool bDraggable = true;

	/** Heavier = harder to drag. Reaction impulse is divided by this. Low = the limb gets reeled in. */
	UPROPERTY(EditAnywhere, Category = "Pull", meta = (EditCondition = "bDraggable", ClampMin = "0.1"))
	float Mass = 10.0f;

	/** Velocity damping of the drag offset (per second). Higher = settles faster / less drift. */
	UPROPERTY(EditAnywhere, Category = "Pull", meta = (EditCondition = "bDraggable", ClampMin = "0.0"))
	float DragDamping = 2.0f;

	/**
	 * Spring pulling the capsule back to its rest pose (per second^2).
	 * 0 = the limb stays where it was dragged (capture feel — recommended). Raise it for an elastic snap-back.
	 */
	UPROPERTY(EditAnywhere, Category = "Pull", meta = (EditCondition = "bDraggable", ClampMin = "0.0"))
	float ReturnStiffness = 0.0f;

	/**
	 * World-space inner segment of the capsule (the two sphere centers) and its radius.
	 * A point is inside the capsule when its distance to segment [OutA, OutB] is < OutRadius.
	 */
	void GetCapsuleSegment(FVector& OutA, FVector& OutB, float& OutRadius) const;

	//~ IRopeCapsuleProvider
	virtual void GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const override;
	virtual void ApplyRopeReaction(const FVector& WorldImpulse, const FVector& WorldLocation) override;

private:
	UPROPERTY(VisibleAnywhere, Category = "Capsule")
	TObjectPtr<UCapsuleComponent> Capsule = nullptr;

	/** Captured rest pose the swing oscillates around. */
	FTransform RestTransform = FTransform::Identity;
	float SwingElapsed = 0.0f;

	// --- S4 drag state (world space) ---
	/** Impulse received from the rope since the last Tick. */
	FVector PendingImpulse = FVector::ZeroVector;
	/** Current drag velocity and accumulated offset from the rest/swing pose. */
	FVector DragVelocity = FVector::ZeroVector;
	FVector DragOffset = FVector::ZeroVector;
};
