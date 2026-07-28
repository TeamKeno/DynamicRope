// Copyright Epic Games, Inc. All Rights Reserved.
//
// Position-based (XPBD) rope solver. It works only on FRopeSimState and has no UObject dependency, which is
// what keeps it unit-testable and portable to a compute shader. The GPU port (FRopeGPUSolver, RopeXPBD.usf)
// is the normal runtime path; this CPU implementation is the fallback (cook, -nullrhi, server, or a rope over
// the node cap) and the parity and unit-test reference. It runs the whole chain in Free and Flight; in
// Wrapping and Wrapped only the free span survives the mass mask, and Contacting and Releasing do not solve.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeConfigTypes.h"
#include "Core/RopeSimTypes.h"

class IRopeCollider;

/** One frame's fixed-timestep substep schedule, shared by the CPU and GPU solvers. */
struct FRopeSubstepSchedule
{
	/** Substeps to run this frame. 0 means skip the solve entirely. */
	int32 NumSub = 0;

	/** Fixed dt per substep (s). */
	float FixedDt = 0.0f;
};

/**
 * Accumulate DeltaSeconds into State.TimeAccumulator and hand back how much of it this frame consumes in
 * fixed-size substeps, capped against the spiral of death. State is non-const because the accumulator is
 * drawn down. Kept in one place so the CPU path (FRopeXPBDSolver::Step) and the GPU path (FRopeGPUSolver)
 * cannot schedule differently.
 */
DYNAMICROPE_API FRopeSubstepSchedule RopeSolverSubsteps(FRopeSimState& State, const FRopeSolverConfig& Config, float DeltaSeconds);

/**
 * One node's contact constraint state. DetectContacts — swept once per substep, so it is continuous — sets
 * whether the contact is active along with its normal and surface velocity. SolveContacts then re-queries the
 * candidate colliders *fresh* every iteration for the surface distance and normal at the node's current
 * position and reprojects, so a curved or concave surface never suffers a stale cached plane and collision
 * competes on equal terms with the distance and bending constraints.
 * Lambda accumulates the normal impulse, which is the contact normal force, and friction uses it as the
 * Coulomb limit μ·Lambda·w (in XPBD, λ is the constraint force). Normal and SurfaceVel refresh each substep.
 */
struct FRopeContactState
{
	bool    bActive = false;

	FVector Normal = FVector::ZeroVector;
	FVector SurfaceVel = FVector::ZeroVector;
	float   Lambda = 0.0f;
};

/**
 * Collider candidates per node and per segment, chosen in a single detect pass.
 *
 * DetectContacts already does every (node × collider) broad-phase box test and then threw the results away,
 * leaving SolveContacts and SolveSegmentContacts to rescan the colliders before every iteration — the
 * dominant term in the CPU fallback's frame cost, segment collision above all, since it has no active gate.
 * Doing that test once at detect time and reusing it turns O(N·C) per iteration into O(N · candidate count),
 * and a segment that overlaps no collider skips its inner sample loop altogether.
 *
 * Candidates come from the node's sweep AABB and the segment's span AABB, each widened by a margin of one
 * segment rest length, so a collider stays in the set even when the distance, bending or segment corrections
 * pull the node within that margin during the iterations.
 * If one item overlaps more colliders than MaxPerItem, that item alone falls back to the full loop, so no
 * detection result is ever lost — this is a pure cost saving, not a behaviour change.
 */
struct DYNAMICROPE_API FRopeColliderCandidates
{
	/** Candidate cap per item (node or segment). An item over the cap falls back to the full loop. */
	static constexpr int32 MaxPerItem = 12;

	/** False when candidates could not be built (no broad-phase bounds), and the caller loops over everything. */
	bool bValid = false;

	/** Per-node sweep AABB (Prev → Pos), built once at the top of detect rather than per collider. */
	TArray<FBox>  NodeBounds;

	/** Per-node candidate collider indices, stride MaxPerItem. */
	TArray<int32> NodeIndices;
	TArray<int32> NodeNum;
	TArray<bool>  bNodeOverflow;

	/** Per-segment candidates (segment k spans node k to k+1), stride MaxPerItem. */
	TArray<int32> SegIndices;
	TArray<int32> SegNum;
	TArray<bool>  bSegOverflow;

	/** Clear for this detect pass. Buffer allocations are kept, so nothing reallocates per substep. */
	void Reset(int32 NumNodes);

	void AddNode(int32 NodeIndex, int32 ColliderIndex);
	void AddSegment(int32 SegIndex, int32 ColliderIndex);

	/**
	 * How many entries to walk when solving one node or segment. With bOutAll true the candidates are
	 * unusable, so the count is the total collider count and the slot index *is* the collider index — the
	 * fallback. With it false, walk through NodeAt / SegAt.
	 */
	int32 NodeCount(int32 NodeIndex, int32 NumColliders, bool& bOutAll) const;
	int32 SegCount(int32 SegIndex, int32 NumColliders, bool& bOutAll) const;

	FORCEINLINE int32 NodeAt(int32 NodeIndex, int32 Slot) const { return NodeIndices[NodeIndex * MaxPerItem + Slot]; }
	FORCEINLINE int32 SegAt(int32 SegIndex, int32 Slot) const { return SegIndices[SegIndex * MaxPerItem + Slot]; }
};

class DYNAMICROPE_API FRopeXPBDSolver
{
public:
	/** Advance one frame: integrate per substep, then solve distance, bending and collision constraints. */
	void Step(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, float DeltaSeconds) const;

private:
	void Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const;

	/**
	 * XPBD distance: hold the segment length with StretchCompliance. Lambda accumulates across the substep's
	 * iterations, one entry per segment constraint, which is what makes the stiffness independent of the
	 * substep and iteration counts.
	 */
	void SolveDistance(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	/** XPBD bending: a "support stick" from i to i+2 (rest = 2 × SegmentLength) under BendCompliance. */
	void SolveBending(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	/**
	 * Contact detection, once per substep or per collision pass. A swept (continuous) query finds each node's
	 * first contact, pushes it out of the surface immediately, and caches the contact surface as a plane —
	 * RestPoint and Normal — into OutContacts. ColliderBounds is the broad-phase AABB expanded by Radius, and
	 * SubAlpha0/1 give the substep's slice of a moving collider's pose.
	 * SolveContacts then enforces that cached plane cheaply on every iteration, and ApplyContactFriction
	 * applies Coulomb friction at the end of the substep.
	 * The broad-phase test happens here anyway, so its result is kept in OutCandidates — the per-node and
	 * per-segment collider shortlist — which is what stops the later iterations repeating it.
	 */
	void DetectContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		float SubAlpha0, float SubAlpha1, TArray<FRopeContactState>& Contacts,
		FRopeColliderCandidates& Candidates) const;

	/**
	 * Re-query *every* collider near an active node afresh each iteration, as a point query, and reproject the
	 * node out of the surface. It accumulates the normal impulse in Lambda (≥ 0, one-sided contact) and is
	 * rigid (compliance 0).
	 * Defending against all overlapping bones rather than a single cached one is what fixed the traversal bug
	 * where one cache let a node slip past a second bone, and it matches the per-node collider loop in the GPU
	 * .usf. Competing inside the Gauss-Seidel sweep alongside distance and bending is what keeps collision
	 * from being overruled under tension.
	 * The set walked is whatever Candidates shortlisted at detect time — only a shortlist over the cap falls
	 * back to every collider — and ColliderBounds still culls by node point within it.
	 */
	void SolveContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		const FRopeColliderCandidates& Candidates, TArray<FRopeContactState>& Contacts) const;

	/**
	 * Segment (edge) collision. Point collision at the nodes cannot stop chording, where the straight line
	 * between two nodes cuts through a thin surface such as an arm or a leg with both end nodes outside it.
	 * This samples along each segment (spacing from its length and SweepStep), pushes any penetrating sample
	 * out of the surface, and distributes the correction barycentrically to the two end nodes as (1-t) : t.
	 * A segment whose ends are both pinned (invMass 0), as in the wrapped span, cannot move and is skipped.
	 * It runs every iteration so it competes in the sweep with distance and bending.
	 * A segment with no candidates is skipped before its inner sample loop even begins, which covers most
	 * segments — without that gate, segment collision was the single biggest cost in the CPU fallback.
	 */
	void SolveSegmentContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		const FRopeColliderCandidates& Candidates, bool bReverse) const;

	/**
	 * Apply Coulomb friction once at the end of the substep: cap the tangential correction at μ (tapered) ×
	 * Lambda × w, where Lambda is the accumulated normal force. Below the cap all relative motion is removed
	 * (static friction); above it the rope slips. SubDt converts the surface velocity (cm/s) into a substep
	 * displacement.
	 */
	void ApplyContactFriction(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<FRopeContactState>& Contacts, float SubDt) const;

	/**
	 * Strain limiting — the maximum stretch clamp. With few XPBD iterations, a long chain hanging off a pinned
	 * node (a pin or anchor, InvMass 0) cannot propagate the Gauss-Seidel correction to its far end, and the
	 * segments beside the pin blow out. A sequential sweep, forward then backward, hard-projects every segment
	 * to at most MaxStretchRatio × SegmentLength and so propagates the correction along the whole chain at
	 * once. The move is velocity-neutral because Prev moves with it, so the clamp neither injects nor removes
	 * Verlet velocity. A MaxStretchRatio below 1 makes it a no-op. Mirrored by the strain-limit stage in the
	 * GPU RopeXPBD.usf.
	 */
	void SolveStrainLimit(FRopeSimState& State, const FRopeSolverConfig& Config) const;
};
