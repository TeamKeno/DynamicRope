// Copyright Epic Games, Inc. All Rights Reserved.
//
// Gameplay-authoritative rope length constraint.  This is deliberately independent from
// XPBD particles/tension so movement can enforce an inextensible cable before PostPhysics.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/WeakObjectPtrTemplates.h"

class USceneComponent;

/**
 * Current CPU-side constraint for the wielder's hand-side free rope span.
 *
 * PivotWorld is resolved directly from the live wrap binding (component/bone local anchor),
 * never from the delayed GPU particle mirror.  MaxDistance is material rope length, not the
 * solver strain cap or the soft tether activation slack.
 */
struct DYNAMICROPE_API FRopeWielderMovementConstraint
{
	FVector PivotWorld = FVector::ZeroVector;
	FVector PivotVelocity = FVector::ZeroVector;
	FVector AnchorWorld = FVector::ZeroVector;
	float MaxDistance = 0.0f;
	int32 AnchorNode = INDEX_NONE;
	TWeakObjectPtr<const USceneComponent> TargetComponent;
	FName TargetBone = NAME_None;

	bool IsValid() const
	{
		return AnchorNode >= 0 && MaxDistance >= 0.0f && FMath::IsFinite(MaxDistance)
			&& TargetComponent.IsValid();
	}
};

namespace RopeMovementConstraint
{
	/** Result of projecting a proposed hand point into a unilateral maximum-distance sphere. */
	struct DYNAMICROPE_API FProjectionResult
	{
		FVector Position = FVector::ZeroVector;
		FVector OutwardNormal = FVector::ZeroVector;
		float Violation = 0.0f;
		bool bAtLimit = false;
		bool bConstrained = false;
	};

	/**
	 * Projects Point into |Point - Pivot| <= MaxDistance.
	 * Inward/slack points are unchanged. Tangential movement is retained with only the
	 * second-order inward correction required to remain on the sphere.
	 */
	DYNAMICROPE_API FProjectionResult ProjectPoint(
		const FVector& Point, const FVector& Pivot, float MaxDistance, float BoundaryTolerance = 0.1f);

	/** Removes only separating velocity at an active boundary; inward/tangent velocity survives. */
	DYNAMICROPE_API FVector RemoveOutwardVelocity(
		const FVector& Velocity, const FVector& PivotVelocity, const FVector& OutwardNormal);

	/**
	 * Measures the outward motion rejected by the hard boundary. Velocity rejection is
	 * authoritative when available; positional projection covers teleport/custom-mover
	 * motion that did not update Velocity. The result feeds the reaction-tension solve.
	 */
	DYNAMICROPE_API float ComputeRejectedSeparatingSpeed(
		const FVector& AttemptedVelocity,
		const FVector& ConstrainedVelocity,
		const FVector& OutwardNormal,
		float PositionViolation,
		float DeltaTime);

	/**
	 * Selects the material-constraint speed term without counting a hard projection twice.
	 *
	 * A hard-projected attempt already measures the rejected boundary change. The live
	 * endpoint/rest-rate sample may describe the same reel or a newer target motion, so the
	 * hard path takes their maximum instead of summing them. The ordinary analytic path has
	 * no recorded projection and uses only the live material-relative rate.
	 */
	DYNAMICROPE_API float ComputeConstraintSeparatingSpeed(
		bool bHardProjectedAttempt,
		float RejectedSeparatingSpeed,
		float EndpointSeparatingSpeed,
		float MaterialLengthRate);
}
