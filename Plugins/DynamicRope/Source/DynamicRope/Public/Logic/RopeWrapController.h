// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The binding semantics layer: everything after a wrap has been decided. This is logic rather than
// physics. It latches the contacting nodes into bone-local space, holds them against the skinning,
// pulls, and releases.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeSimTypes.h"
#include "Core/RopeTractionTypes.h"
#include "Core/RopeWrappingTypes.h"
// Wrap target abstraction: FRopeBindingFrame and ResolveBindingWorld for the hold binding, plus the
// structural queries in RopeWrapTargets::
#include "Core/RopeWrapTarget.h"

class USkeletalMeshComponent;
class IRopeCollider;

class DYNAMICROPE_API FRopeWrapController
{
public:
	FRopeWrapState State;

	/**
	 * Freezes the seeded contact nodes into bone-local space, which is the physics-to-logic handoff.
	 * The mesh to wrap must already be established as Seed.Mesh, propagated from the contact, which is
	 * what makes cross-actor wraps work. Without one, nothing is latched and the state is reset.
	 * Position and mass writes go into OutFrame, the per-node override output, rather than straight
	 * into the simulation state, so the caller uses the same frame for both the CPU application and
	 * the GPU override packing.
	 */
	void BeginWrap(const FRopeSimState& Sim, const FRopeWrapState& Seed, FRopeNodeOverrideFrame& OutFrame);

	/**
	 * Writes, every frame, the targets of the latched nodes on their skinned bones into OutFrame, so
	 * the wrap follows the animation. Each latched node gets Pos and Prev at the anchor on the bone and
	 * an inverse mass of 0; applying them is the caller's path.
	 * @return true while the wrap can be maintained. False once the wrapped mesh has gone, as when a
	 *         cross-actor target actor is destroyed, in which case the caller must return the nodes to
	 *         the solver and release.
	 */
	bool Hold(const FRopeSimState& Sim, float Dt, FRopeNodeOverrideFrame& OutFrame);

	/**
	 * Produces the pull: fills in, as data, the pull the first anchor on the hand side receives from
	 * the rope, as a direction and a tension.
	 * The direction is the unit vector towards the end node of the first straight leg, found by walking
	 * along the rope from the anchor towards the hand. The walk stops at a corner, meaning where the
	 * next segment bends more than BendThresholdDeg away from the leg direction accumulated so far. A
	 * straight rope therefore walks all the way to the hand at node 0 and gives exactly the chord from
	 * the anchor to the hand, while a rope caught on a wall or an edge stops just before it and pulls
	 * along the rope's actual path, that is its first leg, where a straight chord would pass through
	 * the obstacle. Comparing against the accumulated leg direction, rather than the previous segment,
	 * means sag or jitter on a single node does not terminate the walk early; that is a spatial
	 * average, and the caller's exponential moving average takes care of the residual jitter between
	 * frames.
	 * The tension is the SegmentTension of the adjacent segment on the hand side, which is 0 when
	 * slack, giving zero force.
	 * Applying the force, whether to a character or a physics bone, and smoothing it over time, are
	 * UObject and state work and belong to the caller; this produces pure data and can be unit tested.
	 * @return true when a valid anchor and segment existed and Out was filled in, including when the
	 *         tension is 0.
	 */
	bool ComputePull(const FRopeSimState& Sim, float BendThresholdDeg, FRopePullSample& Out) const;

	/**
	 * The anchor-independent body of ComputePull: the same leg walk and taut observations, but for a
	 * caller-chosen hand-side anchor instead of this controller's wrap state. The Wrapping phase uses
	 * it to observe the pull before the wrap commits — its anchors still live in FRopeWrappingState,
	 * so the caller selects the hand-side (minimum-node) anchor there and passes it in. Pure data,
	 * unit-testable, no controller state involved.
	 */
	static bool ComputePullFromAnchor(
		const FRopeSimState& Sim, int32 AnchorNode, FName AnchorBone,
		float BendThresholdDeg, FRopePullSample& Out);

	/** Unlatches and returns control to the solver. */
	void Release(ERopeReleaseReason Reason);

	bool IsActive() const { return State.IsWrapped(); }
};
