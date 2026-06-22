// Copyright Epic Games, Inc. All Rights Reserved.
//
// The binding-semantics layer: everything *after* the wrap is decided. This is LOGIC, not
// physics — latch contact nodes to bone-local, hold them via skinning, pull, and release.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class USkeletalMeshComponent;

class DYNAMICROPE_API FRopeWrapController
{
public:
	FRopeWrapState State;

	/** Freeze the seeded contact nodes into bone-local space (physics → logic handoff). */
	void BeginWrap(FRopeSimState& Sim, const FRopeWrapState& Seed);

	/** Re-place latched nodes on the (skinned) bone each frame. No collision, no solver. */
	void Hold(FRopeSimState& Sim, const USkeletalMeshComponent* Mesh);

	/** Drag the captured limb toward a target procedurally (IK/tension). */
	void Pull(FRopeSimState& Sim, const FVector& PullTarget);

	/** Unlatch and hand control back to the solver. */
	void Release(ERopeReleaseReason Reason);

	bool IsActive() const { return State.IsWrapped(); }
};
