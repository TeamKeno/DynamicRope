// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Core/RopeMovementConstraint.h"

namespace RopeMovementConstraint
{
	FProjectionResult ProjectPoint(
		const FVector& Point, const FVector& Pivot, float MaxDistance, float BoundaryTolerance)
	{
		FProjectionResult Out;
		Out.Position = Point;

		const float Radius = FMath::Max(MaxDistance, 0.0f);
		const FVector Offset = Point - Pivot;
		const float Distance = static_cast<float>(Offset.Size());
		const float Tolerance = FMath::Max(BoundaryTolerance, 0.0f);

		if (Distance > KINDA_SMALL_NUMBER)
		{
			Out.OutwardNormal = Offset / Distance;
		}
		else
		{
			// Direction is undefined at the pivot. It is slack unless the permitted radius is zero.
			Out.bAtLimit = Radius <= Tolerance;
			return Out;
		}

		Out.bAtLimit = Distance >= FMath::Max(0.0f, Radius - Tolerance);
		Out.Violation = FMath::Max(0.0f, Distance - Radius);
		Out.bConstrained = Out.Violation > KINDA_SMALL_NUMBER;
		if (Out.bConstrained)
		{
			Out.Position = Pivot + Out.OutwardNormal * Radius;
			Out.bAtLimit = true;
		}
		return Out;
	}

	FVector RemoveOutwardVelocity(
		const FVector& Velocity, const FVector& PivotVelocity, const FVector& OutwardNormal)
	{
		if (OutwardNormal.IsNearlyZero())
		{
			return Velocity;
		}

		const FVector RelativeVelocity = Velocity - PivotVelocity;
		const float OutwardSpeed = static_cast<float>(FVector::DotProduct(RelativeVelocity, OutwardNormal));
		return OutwardSpeed > 0.0f ? Velocity - OutwardNormal * OutwardSpeed : Velocity;
	}

	float ComputeRejectedSeparatingSpeed(
		const FVector& AttemptedVelocity,
		const FVector& ConstrainedVelocity,
		const FVector& OutwardNormal,
		float PositionViolation,
		float DeltaTime)
	{
		const float RejectedVelocity = OutwardNormal.IsNearlyZero()
			? 0.0f
			: static_cast<float>(FVector::DotProduct(
				AttemptedVelocity - ConstrainedVelocity, OutwardNormal));
		const float RejectedPositionRate = DeltaTime > KINDA_SMALL_NUMBER
			? FMath::Max(PositionViolation, 0.0f) / DeltaTime
			: 0.0f;
		return FMath::Max(0.0f, FMath::Max(RejectedVelocity, RejectedPositionRate));
	}

	float ComputeConstraintSeparatingSpeed(
		bool bHardProjectedAttempt,
		float RejectedSeparatingSpeed,
		float EndpointSeparatingSpeed,
		float MaterialLengthRate)
	{
		const float LiveSeparatingSpeed =
			EndpointSeparatingSpeed - MaterialLengthRate;
		return bHardProjectedAttempt
			? FMath::Max(
				FMath::Max(RejectedSeparatingSpeed, 0.0f),
				LiveSeparatingSpeed)
			: LiveSeparatingSpeed;
	}
}
