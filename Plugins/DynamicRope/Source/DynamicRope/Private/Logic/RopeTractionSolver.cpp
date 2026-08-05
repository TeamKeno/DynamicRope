// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeTractionSolver.h"
#include "Logic/RopeLengthConstraintSolver.h"

namespace RopeTraction
{
	float ComputeAxisDeltaV(float CurAlong, const FRopeAxisServo& Servo)
	{
		// The baseline for cancellation: movement outwards is treated as starting from zero, and that cancelled amount is included in the velocity delta below and applied with it.
		const float Base = Servo.bCancelOutward ? FMath::Max(CurAlong, 0.0f) : CurAlong;
		// In accelerate-only mode, already being at or past the target keeps the baseline, meaning no braking.
		const bool bApproach = Servo.bBidirectional || (Servo.TargetSpeed > Base);
		const float Final = bApproach ? (Base + (Servo.TargetSpeed - Base) * Servo.Alpha) : Base;
		return Final - CurAlong;
	}

	float ClampAxisImpulse(float DeltaV, float Mass, float MaxImpulse)
	{
		const float J = Mass * DeltaV;
		return (MaxImpulse > 0.0f) ? FMath::Clamp(J, -MaxImpulse, MaxImpulse) : J;
	}

	FVector ClampInjectedVelocity(const FVector& NewVel, const FVector& OldVel, float SpeedCap)
	{
		if (SpeedCap <= 0.0f)
		{
			return NewVel;
		}
		return NewVel.GetClampedToMaxSize(FMath::Max(SpeedCap, static_cast<float>(OldVel.Size())));
	}

	float InvMassFromMass(float Mass)
	{
		return (Mass > KINDA_SMALL_NUMBER) ? (1.0f / Mass) : 0.0f;
	}

	FVector ComputePointVelocityDelta(
		const FRopePointMassProperties& Body,
		const FVector& PointWorld,
		const FVector& ImpulseWorld)
	{
		const float InvMass = InvMassFromMass(Body.Mass);
		if (InvMass <= 0.0f)
		{
			return FVector::ZeroVector;
		}

		const FVector Arm =
			PointWorld - Body.MassSpaceToWorld.GetLocation();
		const FVector AngularImpulseWorld =
			FVector::CrossProduct(Arm, ImpulseWorld);
		const FQuat MassRotation =
			Body.MassSpaceToWorld.GetRotation().GetNormalized();
		const FVector AngularImpulseLocal =
			MassRotation.UnrotateVector(AngularImpulseWorld);
		const FVector InvInertiaLocal(
			Body.InertiaTensor.X > KINDA_SMALL_NUMBER
				? 1.0f / Body.InertiaTensor.X
				: 0.0f,
			Body.InertiaTensor.Y > KINDA_SMALL_NUMBER
				? 1.0f / Body.InertiaTensor.Y
				: 0.0f,
			Body.InertiaTensor.Z > KINDA_SMALL_NUMBER
				? 1.0f / Body.InertiaTensor.Z
				: 0.0f);
		const FVector AngularVelocityDeltaWorld =
			MassRotation.RotateVector(
				AngularImpulseLocal * InvInertiaLocal);
		return ImpulseWorld * InvMass +
			FVector::CrossProduct(AngularVelocityDeltaWorld, Arm);
	}

	float ComputePointInverseMass(
		const FRopePointMassProperties& Body,
		const FVector& PointWorld,
		const FVector& DirectionWorld)
	{
		const FVector Direction = DirectionWorld.GetSafeNormal();
		if (Direction.IsNearlyZero())
		{
			return 0.0f;
		}
		return FMath::Max(
			0.0f,
			static_cast<float>(FVector::DotProduct(
				Direction,
				ComputePointVelocityDelta(
					Body, PointWorld, Direction))));
	}

	float ExpSmoothAlpha(float Tau, float DeltaTime)
	{
		return (Tau > KINDA_SMALL_NUMBER) ? (1.0f - FMath::Exp(-DeltaTime / Tau)) : 1.0f;
	}

	FVector SmoothDirection(const FVector& Current, const FVector& Target, float Alpha)
	{
		if (Current.IsNearlyZero())
		{
			return Target; // Unseeded, so it is seeded from the measurement, with no lag.
		}
		const FVector Smoothed = FMath::Lerp(Current, Target, Alpha).GetSafeNormal();
		// The degenerate case where exactly opposite directions cancel, at the instant of a 180 degree reversal: when the interpolation reaches zero it is reseeded from the raw value.
		return Smoothed.IsNearlyZero() ? Target : Smoothed;
	}

	FVector SampleFractionalAim(const TArray<FVector>& Positions, float AimF, int32 AnchorNode)
	{
		const int32 A0 = FMath::FloorToInt(AimF);
		const int32 A1 = FMath::Min(A0 + 1, AnchorNode);
		if (!Positions.IsValidIndex(A0) || !Positions.IsValidIndex(A1))
		{
			return FVector::ZeroVector;
		}
		return FMath::Lerp(Positions[A0], Positions[A1], AimF - static_cast<float>(A0));
	}

	bool EvaluateTautGate(float Tension, float Threshold, float ReleaseRatio, bool bWasTaut)
	{
		// Separate engage and sustain thresholds, giving hysteresis. At a threshold of zero or below, both converge on a tension above about zero, which is the previous gate.
		const float EnterAbove = FMath::Max(Threshold, KINDA_SMALL_NUMBER);
		const float StayAbove = FMath::Max(Threshold * FMath::Clamp(ReleaseRatio, 0.0f, 1.0f), KINDA_SMALL_NUMBER);
		return Tension > (bWasTaut ? StayAbove : EnterAbove);
	}

	bool EvaluateChainTautGate(float ChordLen, float RestLen, float SlackRatio, float ReleaseScale, bool bWasTaut)
	{
		if (RestLen <= KINDA_SMALL_NUMBER)
		{
			// Undecidable, as when there is no free span because the anchor is the hand, so it is not taut.
			return false;
		}
		// Separate engage and sustain thresholds, giving hysteresis: sustaining relaxes the permitted slack by the
		// release scale. A product of one or more converges on permitting all the slack, meaning a latched gate is
		// sustained regardless of the chord, so it is capped at one to prevent a negative threshold.
		const float Ratio = FMath::Clamp(SlackRatio, 0.0f, 1.0f);
		const float EffRatio = bWasTaut ? FMath::Min(Ratio * FMath::Max(ReleaseScale, 1.0f), 1.0f) : Ratio;
		return ChordLen >= RestLen * (1.0f - EffRatio);
	}

	float SolveTetherLambda(const FRopeTetherConstraint& In, float DeltaTime)
	{
		const float WSum = FMath::Max(In.InvMassTarget, 0.0f) + FMath::Max(In.InvMassWielder, 0.0f);
		RopeLengthConstraint::FInput LengthIn;
		LengthIn.Violation = In.C;
		LengthIn.SeparatingSpeed = In.SepSpeed;
		LengthIn.EffectiveInverseMass = WSum;
		LengthIn.SettleAlpha = In.SettleAlpha;
		LengthIn.MaxBiasSpeed = In.MaxBiasSpeed;
		LengthIn.Compliance = In.Compliance;
		LengthIn.MaxTension = In.MaxTension;
		return RopeLengthConstraint::Solve(LengthIn, DeltaTime).Lambda;
	}
}
