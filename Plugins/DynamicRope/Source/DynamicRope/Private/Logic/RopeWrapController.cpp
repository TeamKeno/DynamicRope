// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrapController.h"

void FRopeWrapController::BeginWrap(FRopeSimState& /*Sim*/, const FRopeWrapState& Seed)
{
	// TODO(M2): convert each contact node's world position into bone-local and store in Latched.
	State = Seed;
}

void FRopeWrapController::Hold(FRopeSimState& /*Sim*/, const USkeletalMeshComponent* /*Mesh*/)
{
	// TODO(M2/M3): for each latched node, world = BoneTransform * BoneLocalPos; pin Sim node there.
}

void FRopeWrapController::Pull(FRopeSimState& /*Sim*/, const FVector& /*PullTarget*/)
{
	// TODO(M3): procedural drag of the captured limb toward PullTarget, keep wrap taut.
}

void FRopeWrapController::Release(ERopeReleaseReason /*Reason*/)
{
	State.Reset();
}
