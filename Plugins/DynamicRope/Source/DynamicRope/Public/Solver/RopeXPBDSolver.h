// Copyright Epic Games, Inc. All Rights Reserved.
//
// Position-based (XPBD) rope solver. Since it operates only on FRopeSimState without UObject dependency,
// Unit testing is possible. The runtime regular path is the GPU port (FRopeGPUSolver, RopeXPBD.usf), and this CPU
// implementation is fallback (cook/-nullrhi/server/node number exceeded) + parity/unit test reference point. All on Free/Flight,
// In Wrapping/Wrapped, only Free spans that are not mass masked run (Contacting/Releasing does not solve).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeConfigTypes.h"
#include "Core/RopeSimTypes.h"

class IRopeCollider;

/** One frame pinned timestep substep schedule. Shared by CPU solver and GPU solver.*/
struct FRopeSubstepSchedule
{
	/** Number of substeps to run in this frame (if 0, skip solving this frame).*/
	int32 NumSub = 0;

	/** pinned dt (seconds) per substep.*/
	float FixedDt = 0.0f;
};

/**
 * Accumulate DeltaSeconds in State.TimeAccumulator and consume them in pinned size substep units for this frame.
 * Returns the schedule (including spiral-of-death cap). Since the accumulator is updated (deducted), the State is non-const.
 * Extracted to one place so that both CPU (FRopeXPBDSolver::Step) and GPU (FRopeGPUSolver) use the same schedule.
 */
DYNAMICROPE_API FRopeSubstepSchedule RopeSolverSubsteps(FRopeSimState& State, const FRopeSolverConfig& Config, float DeltaSeconds);

/**
 * Contact constraint status of one node. Whether DetectContacts (swept once per substep, CCD) is active and contact normal/surface
 * Set the velocity, and SolveContacts *fresh* the candidate colliders every iteration to determine the actual position of the current position.
 * Obtain surface distance/normal and reproject → Even in curved/concave Chris, collision occurs without cache plane staleness.
 * Competes equally with distance/bending. Lambda is friction with accumulated normal impulse (= contact normal force)
 * Used in Coulomb limit μ·Lambda·w (XPBD: λ is the constraint force). Normal/SurfaceVel is updated for every material.
 */
struct FRopeContactState
{
	bool    bActive = false;

	FVector Normal = FVector::ZeroVector;
	FVector SurfaceVel = FVector::ZeroVector;
	float   Lambda = 0.0f;
};

/**
 * List of collider candidates for each node/segment selected from one detect pass.
 *
 * DetectContacts does all the (node × collider) broad-phase box checks anyway, but discards the results.
 * SolveContacts/SolveSegmentContacts was re-scanning the collider before each iteration (CPU fallback frame cost
 * dominant term — especially segment collision where there is no active gate). Do that check only once at the time of detect and then
 * iterations reuse: O(N·C) per iteration is reduced to O(N·candidate number), and does not overlap with any collider.
 * segment skips entering the inner sample loop itself.
 *
 * Candidates are selected by widening node sweep AABB / segment section AABB by Margin (segment rest length) — iteration
 * Even if the distance/bending/segment correction pulls the node, the collider is not missed within that margin.
 * If the collider overlapping with one item exceeds MaxPerItem, only that item falls back to a loop, so in any case,
 * No reduction in detection results (pure cost savings, not behavior change).
 */
struct DYNAMICROPE_API FRopeColliderCandidates
{
	/** candidate cap per item (node/segment). If it exceeds, only that item falls back into the loop.*/
	static constexpr int32 MaxPerItem = 12;

	/** If false, the candidate cannot be created (absence of broad-phase bounds) → The caller loops entirely.*/
	bool bValid = false;

	/** per-node sweep AABB(Prev→Pos). Fill it once at the front of the detect to avoid creating it again for each collider.*/
	TArray<FBox>  NodeBounds;

	/** per-node candidate collider index. stride = MaxPerItem.*/
	TArray<int32> NodeIndices;
	TArray<int32> NodeNum;
	TArray<bool>  bNodeOverflow;

	/** candidate for each segment k (= node k~k+1). stride = MaxPerItem.*/
	TArray<int32> SegIndices;
	TArray<int32> SegNum;
	TArray<bool>  bSegOverflow;

	/** Empty this detect pass. Since buffer allocation is maintained, it is not reallocated for each substep.*/
	void Reset(int32 NumNodes);

	void AddNode(int32 NodeIndex, int32 ColliderIndex);
	void AddSegment(int32 SegIndex, int32 ColliderIndex);

	/**
	 * Number of nodes/segment to be traversed when solving one. If bOutAll is true, the candidate cannot be used, so the return value is all
	 * is the collider number and the slot number is the collider index (fallback). If false, it must be solved with NodeAt/SegAt.
	 */
	int32 NodeCount(int32 NodeIndex, int32 NumColliders, bool& bOutAll) const;
	int32 SegCount(int32 SegIndex, int32 NumColliders, bool& bOutAll) const;

	FORCEINLINE int32 NodeAt(int32 NodeIndex, int32 Slot) const { return NodeIndices[NodeIndex * MaxPerItem + Slot]; }
	FORCEINLINE int32 SegAt(int32 SegIndex, int32 Slot) const { return SegIndices[SegIndex * MaxPerItem + Slot]; }
};

class DYNAMICROPE_API FRopeXPBDSolver
{
public:
	/** One frame progress: substep unit integration + distance/bending/collision constraint.*/
	void Step(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, float DeltaSeconds) const;

private:
	void Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const;

	/**
	 * XPBD distance: Enforce segment length with StretchCompliance. Lambda is used throughout the iteration of substeps.
	 * is cumulative (one item per segment constraint), which makes the stiffness independent of the step/iter number.
	 */
	void SolveDistance(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	/** XPBD bending: i<->i+2 "support stick" (rest = 2*SegmentLength) with BendCompliance applied.*/
	void SolveBending(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
		TArray<float>& Lambda) const;

	/**
	 * contact detection (once per substep or CollisionPasses): Find the first contact of each node using swept query (CCD) and bring it out of the surface.
	 * Immediately push-out and cache the contact surface as a plane (RestPoint/Normal) (Out Contacts). ColliderBounds is broad-phase
	 * AABB(+Radius), SubAlpha0/1 is the substep sub-pose section of the moving collider. Afterwards, SolveContacts runs every iteration.
	 * Forces this cache plane cheaply, and ApplyContactFriction applies Coulomb friction at the end of the substep.
	 * Here, the broad-phase check, which is running anyway, is left as Out Candidates (colider candidate by node/segment),
	 * Prevents subsequent iterations from repeating the same check.
	 */
	void DetectContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		float SubAlpha0, float SubAlpha1, TArray<FRopeContactState>& Contacts,
		FRopeColliderCandidates& Candidates) const;

	/**
	 * *All* colliders close to the active node are re-projected out of the surface by querying (point querying) each iteration fresh.
	 * Accumulates normal impulse Lambda (>=0, one-sided contact). rigid(compliance 0). Not just one cache, but all overlapping bones
	 * defends (fixes a single-cache traversal bug — consistent with the pre-colider loop per node in GPU .usf), such as distance/bending.
	 * Competition in the Gauss-Seidel sweep → does not fall behind in tension. The traversal target is the node that Candidates selected when detecting.
	 * is the only candidate (full fallback only when cap is exceeded), and ColliderBounds remain as node-point broad-phase curls within it.
	 */
	void SolveContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		const FRopeColliderCandidates& Candidates, TArray<FRopeContactState>& Contacts) const;

	/**
	 * segment (edge) collision: Node point collision is a chording where a straight line between two nodes crosses a thin surface (arm, leg, etc.)
	 * cannot be blocked (both end nodes are outside the surface, only the straight line between them penetrates). Each segment as an internal sample point (based on length/SweepStep)
	 * , push the penetrated sample out of the surface and distribute the correction to both end nodes barycentrically ((1-t):t). both ends
	 * The wrap section with pin(invMass 0) cannot be moved, so skip it. Called each iteration to compete in sweeps such as distance/bending.
	 * Segments with no candidates in Candidates are skipped entirely before entering the inner sample loop (most segments are
	 * applies here — without this gate, segment collisions were the biggest source of CPU fallback cost).
	 */
	void SolveSegmentContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
		const FRopeColliderCandidates& Candidates, bool bReverse) const;

	/**
	 * Apply Coulomb friction once at the end of the substep: cap the tangent correction amount as μ (taper)·Lambda·w (Lambda=accumulated normal force). small
	 * All relative motion is eliminated (stationary friction), excess grip is slip. Convert surface velocity (cm/s) to substep displacement with SubDt.
	 */
	void ApplyContactFriction(FRopeSimState& State, const FRopeSolverConfig& Config,
		const TArray<FRopeContactState>& Contacts, float SubDt) const;

	/**
	 * Strain limiting (maximum elongation clamp): If the XPBD iteration is small, a long chain will be connected to a pinned node (pin/anchor, InvMass 0).
	 * When hanging, the Gauss-Seidel correction cannot be propagated to the end, causing a burst of elongation in the segment adjacent to the pinned node. sequential
	 * Hard project each segment with sweep (forward + backward) with ≤ MaxStretchRatio × SegmentLength to the entire chain at once.
	 * Propagate correction. Position movement is velocity neutral (prev also moves) — the clamp does not inject/remove Verlet velocity.
	 * If MaxStretchRatio < 1, no-op. Strain-limit stage and mirror in GPU RopeXPBD.usf.
	 */
	void SolveStrainLimit(FRopeSimState& State, const FRopeSolverConfig& Config) const;
};
