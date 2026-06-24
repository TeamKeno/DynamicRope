// Copyright Epic Games, Inc. All Rights Reserved.
//
// The binding-semantics layer: everything *after* the wrap is decided. This is LOGIC, not
// physics — latch contact nodes to bone-local, hold them via skinning, pull, and release.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class USkeletalMeshComponent;
class IRopeCollider;

class DYNAMICROPE_API FRopeWrapController
{
public:
	FRopeWrapState State;

	/**
	 * Contact decision (physics → logic gate). Queries each node against the colliders, finds the
	 * dominant contacted bone, and requires MinLatchNodes nodes in sustained contact for
	 * WrapDecisionTime before committing. Returns true once and fills OutSeed (node indices + bone)
	 * for BeginWrap. Tracks the candidate internally across frames; call every Contacting tick.
	 */
	bool DecideWrap(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
		const FRopeWrapConfig& Config, float Dt, FRopeWrapState& OutSeed);

	/** Freeze the seeded contact nodes into bone-local space (physics → logic handoff). */
	void BeginWrap(FRopeSimState& Sim, const FRopeWrapState& Seed, const USkeletalMeshComponent* Mesh);

	/** Re-place latched nodes on the (skinned) bone each frame so the wrap follows animation. */
	void Hold(FRopeSimState& Sim, const USkeletalMeshComponent* Mesh, float Dt);

	/** Drag the captured limb toward a target procedurally (IK/tension). */
	void Pull(FRopeSimState& Sim, const FVector& PullTarget);

	/** Unlatch and hand control back to the solver. */
	void Release(ERopeReleaseReason Reason);

	bool IsActive() const { return State.IsWrapped(); }

private:
	// Sustained-contact accumulation for the decision (transient; not part of the wrap state).
	FName         CandidateBone = NAME_None;
	float         CandidateTime = 0.0f;
	TArray<int32> CandidateNodes;
};
