// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Runtime state for the gameplay-authoritative material-length constraint.  This is
// intentionally separate from FRopePullDriveState: pull input/direction smoothing and
// the passive unilateral length reaction are different domains.

#pragma once

#include "CoreMinimal.h"

enum class ERopeLengthConstraintBackend : uint8
{
	None,
	Chaos,
	HardReaction,
	Analytic
};

struct FRopeLengthConstraintState
{
	/** The one backend that owned this frame. Backends are never summed. */
	ERopeLengthConstraintBackend Backend = ERopeLengthConstraintBackend::None;

	/** Final material-boundary violation observed this frame (cm, clamped to >= 0). */
	float LastViolation = 0.0f;

	/** Authoritative unilateral reaction impulse and the step that produced it. */
	float LastLambda = 0.0f;
	float LastLambdaDt = 0.0f;

	/**
	 * The movement adapter runs after Pawn movement and before physics.  It records the
	 * motion that was rejected by the hard material boundary so the later rope update
	 * does not mistake the successfully projected C=0 state for "no load".
	 */
	uint64 WielderAttemptFrame = MAX_uint64;
	int32 WielderAttemptAnchorNode = INDEX_NONE;
	float WielderAttemptViolation = 0.0f;
	float WielderAttemptSeparatingSpeed = 0.0f;
	FVector WielderAttemptOutwardNormal = FVector::ZeroVector;
	bool bWielderAttemptAtLimit = false;

	/** Previous live material length / anchor sample used for reel and moving-anchor rates. */
	float PrevMaterialLength = 0.0f;
	FVector PrevAnchorWorldPoint = FVector::ZeroVector;
	FVector SmoothedAnchorPointVelocity = FVector::ZeroVector;
	int32 PrevAnchorNode = INDEX_NONE;
	bool bPrevGeometryValid = false;

	/**
	 * Previous hand-side (wielder-end) point sample. An Anchor-kind wielder endpoint (a
	 * kinematic carrier such as a helicopter) has no physics velocity, so the constraint
	 * measures the hand point by finite difference and smooths it here — the wielder-side
	 * mirror of SmoothedAnchorPointVelocity. Guarded by the same bPrevGeometryValid.
	 */
	FVector PrevWielderWorldPoint = FVector::ZeroVector;
	FVector SmoothedWielderPointVelocity = FVector::ZeroVector;

	void BeginFrame(float DeltaTime)
	{
		Backend = ERopeLengthConstraintBackend::None;
		LastViolation = 0.0f;
		LastLambda = 0.0f;
		LastLambdaDt = DeltaTime;
	}

	void RecordWielderAttempt(
		uint64 Frame,
		int32 AnchorNode,
		float Violation,
		float SeparatingSpeed,
		const FVector& OutwardNormal,
		bool bAtLimit)
	{
		WielderAttemptFrame = Frame;
		WielderAttemptAnchorNode = AnchorNode;
		WielderAttemptViolation = FMath::Max(Violation, 0.0f);
		WielderAttemptSeparatingSpeed = FMath::Max(SeparatingSpeed, 0.0f);
		WielderAttemptOutwardNormal = OutwardNormal.GetSafeNormal();
		bWielderAttemptAtLimit = bAtLimit;
	}

	bool HasWielderAttempt(uint64 Frame, int32 AnchorNode) const
	{
		return WielderAttemptFrame == Frame
			&& WielderAttemptAnchorNode == AnchorNode
			&& bWielderAttemptAtLimit;
	}

	float GetTension() const
	{
		return LastLambdaDt > 1e-4f ? LastLambda / LastLambdaDt : 0.0f;
	}

	void ResetTransient()
	{
		Backend = ERopeLengthConstraintBackend::None;
		LastViolation = 0.0f;
		LastLambda = 0.0f;
		LastLambdaDt = 0.0f;
		WielderAttemptFrame = MAX_uint64;
		WielderAttemptAnchorNode = INDEX_NONE;
		WielderAttemptViolation = 0.0f;
		WielderAttemptSeparatingSpeed = 0.0f;
		WielderAttemptOutwardNormal = FVector::ZeroVector;
		bWielderAttemptAtLimit = false;
		PrevMaterialLength = 0.0f;
		PrevAnchorWorldPoint = FVector::ZeroVector;
		SmoothedAnchorPointVelocity = FVector::ZeroVector;
		PrevAnchorNode = INDEX_NONE;
		bPrevGeometryValid = false;
		PrevWielderWorldPoint = FVector::ZeroVector;
		SmoothedWielderPointVelocity = FVector::ZeroVector;
	}
};
