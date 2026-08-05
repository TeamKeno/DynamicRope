// Copyright 2026 TeamKeno. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class USceneComponent;

/**
 * Narrow-phase contact: one rope node against one collider, returned by IRopeCollider::Query.
 *
 * CONTRACT — FROZEN. Every IRopeCollider (capsule, per-bone SDF, world GDF) must obey it exactly.
 * Read this block before changing the struct.
 * It describes a single (node, collider) pair; aggregating them is the caller's job — the solver sums the
 * push-outs, and DecideWrap picks the bone with the deepest penetration per node.
 *
 *   bHit           The node's sphere (centre = the queried WorldPos, radius = the queried Radius) overlaps
 *                  the collider. False means every other field is undefined and must be ignored.
 *   Normal         UNIT length, pointing outward from the collider toward the node — the push-out direction.
 *                  Invariant: NodePos += Normal * Penetration puts the node on the surface.
 *                  *** The sign is load-bearing: an inward normal sucks the rope into the body. ***
 *                  Degenerate case (the node sits on the medial axis): any fixed unit vector; the capsule
 *                  reports +Z.
 *   Penetration    Overlap depth along Normal when bHit, always positive. Measured against the QUERY radius:
 *                  (ColliderRadius + QueryRadius) - Distance. Callers pass QueryRadius 0 for a solver
 *                  push-out, and WrapConfig.ContactQueryRadius for the wrap-decision skin.
 *   SurfacePoint   The point on the collider surface closest to the node. Diagnostic only — the solver does
 *                  not need it, so fill it when it is cheap to obtain.
 *   Bone           A skeletal collider must report a non-None bone: that is how DecideWrap attributes the
 *                  wrap. A multi-bone SDF reports the bone owning the closest surface. World colliders
 *                  report None.
 *   SourceMesh     The mesh component owning Bone. It propagates into FRopeWrapState::Mesh, which is what
 *                  lets a rope follow a bone on a different actor. Null for a non-skeletal collider.
 *   SurfaceVelocity  World velocity (cm/s) of the collider surface at the contact point. The solver uses it
 *                  for *relative* tangential friction, so a moving body drags and sweeps the rope aside.
 *                  Leave it 0 for a static surface; the v1 capsule does, and the SDF collider derives it
 *                  from the bone's per-frame motion.
 */
struct FRopeContact
{
	bool    bHit = false;
	FVector Normal = FVector::UpVector;
	float   Penetration = 0.0f;
	FVector SurfacePoint = FVector::ZeroVector;
	FName   Bone = NAME_None;
	const USceneComponent* SourceMesh = nullptr;
	FVector SurfaceVelocity = FVector::ZeroVector;
};

/** The rope centerline: a chain of particles. The single source of truth for solver, logic and render. */
struct FRopeSimState
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;
	float           SegmentLength = 0.0f;
	float           RopeLength = 0.0f;

	/**
	 * Pinned start point (hand or socket). The solver sweeps it from Prev to Target across the substeps, so a
	 * fast anchor jump absorbs energy instead of injecting it and blowing the chain up.
	 */
	bool            bStartPinned = false;
	FVector         StartPinPrev = FVector::ZeroVector;
	FVector         StartPinTarget = FVector::ZeroVector;

	/** Fixed timestep accumulator. The solver consumes real frame time in fixed-size substeps. */
	float           TimeAccumulator = 0.0f;

	/**
	 * Per-segment tension as a force, stretch only and never negative, derived from the converged λ of the
	 * XPBD distance constraint: F = max(0, -λ)/h². The units are relative to a unit-mass node, so thresholds
	 * against it are tuned by measurement. The CPU solver fills it at the end of a step; a GPU-resident rope
	 * fills it from the λ readback, one to two frames late. A frame without a solve keeps the previous value.
	 * Size is Num()-1, and it may be empty if the rope has never solved.
	 */
	TArray<float>   SegmentTension;

	int32 Num() const { return Positions.Num(); }
	void  Reset() { Positions.Reset(); PrevPositions.Reset(); InvMass.Reset(); SegmentTension.Reset(); TimeAccumulator = 0.0f; }

	//~ Verlet vocabulary — pure inline helpers, no context and no policy. Naming the idioms is what keeps
	//  sign and dimension mistakes out. The solver's integration loop (RopeXPBDSolver) and the throw velocity
	//  injection loop deliberately keep the raw form instead: the first is compared 1:1 against the .usf
	//  kernel, and the second accumulates in displacement-unit impulses, so its shape differs.

	/** One frame's displacement (Pos - Prev) for node i. In Verlet, velocity is proportional to it, before dividing by dt. */
	FVector Displacement(int32 i) const { return Positions[i] - PrevPositions[i]; }

	/** How far node i moved this frame (cm/frame). What counts as a "fast" node is the caller's policy. */
	float NodeSpeed(int32 i) const { return Displacement(i).Size(); }

	/** Zero node i's velocity (Prev = Pos). Seed and reseed paths only — a logic phase writes position and
	 *  velocity through FRopeNodeOverrideFrame instead, so the GPU-resident rope stays in sync. */
	void SetStill(int32 i) { PrevPositions[i] = Positions[i]; }
};

/**
 * Per-node bits in FRopeNodeOverrideFrame::Flags. They must stay 1:1 with ERopeGPUOverride (RopeGPUSolver.h),
 * which the subsystem asserts — Core does not depend on the Shaders module, so the constants are mirrored.
 */
namespace RopeNodeOverride
{
	/** Pos[i] = Positions[i] */
	constexpr uint8 Position         = 1 << 0;

	/** Prev[i] = PrevPositions[i] (Verlet velocity injection) */
	constexpr uint8 Prev             = 1 << 1;

	/** Prev[i] = Pos[i], zeroing velocity. Applied *after* Position. */
	constexpr uint8 PrevFromPosition = 1 << 2;

	/** InvMass[i] = InvMass[i] */
	constexpr uint8 InvMass          = 1 << 3;
}

/**
 * One frame of logic-phase output: targets are computed on the game thread, and applied through one path.
 * Wrapping, Wrapped, Releasing and the rest scatter the positions, velocities and masses they want here
 * instead of writing Sim directly. It is applied to the CPU Sim once at the end of PrepareSimFrame
 * (ApplyToSim), and the same data goes to a GPU-resident rope as an override pass (FRopeGPUResidentStep),
 * so the kernel applies it without a reseed.
 * Writing the same node twice lets the last write win, matching a sequential write to Sim.
 * Caution: do not mix explicit Prev with PrevFromPosition in one frame. The kernel applies PrevFromPosition
 * last, so it always wins regardless of the order they were filled in — which is why the logic phases use
 * PrevFromPosition only.
 */
struct FRopeNodeOverrideFrame
{
	/** Per-node OR of RopeNodeOverride bits. Empty means this frame produced no output. */
	TArray<uint8>   Flags;
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;

	bool HasAny() const { return Flags.Num() > 0; }

	void Reset()
	{
		Flags.Reset();
		Positions.Reset();
		PrevPositions.Reset();
		InvMass.Reset();
	}

	/** Size to the node count on the first scatter, zeroed. Calling it again within the frame is a no-op. */
	void EnsureSize(int32 NumNodes)
	{
		if (Flags.Num() != NumNodes)
		{
			Flags.SetNumZeroed(NumNodes);
			Positions.SetNumZeroed(NumNodes);
			PrevPositions.SetNumZeroed(NumNodes);
			InvMass.SetNumZeroed(NumNodes);
		}
	}

	/** Pin a position: Pos = World, and with bZeroVelocity also Prev = Pos — the standard write for wrapping and hold. */
	void SetPosition(int32 NodeIndex, const FVector& World, bool bZeroVelocity)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::Position | (bZeroVelocity ? RopeNodeOverride::PrevFromPosition : 0);
			Positions[NodeIndex] = World;
		}
	}

	/** Overwrite the mass, for masking and restoring. */
	void SetInvMass(int32 NodeIndex, float Value)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::InvMass;
			InvMass[NodeIndex] = Value;
		}
	}

	/** Remove velocity only (Prev = current Pos, position unchanged). Keeps a release from flinging the nodes. */
	void SetPrevFromPosition(int32 NodeIndex)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::PrevFromPosition;
		}
	}

	/** CPU application, in the same order as the GPU kernel's override stage: Pos → Prev → Prev=Pos → InvMass. */
	void ApplyToSim(FRopeSimState& Sim) const
	{
		const int32 N = FMath::Min(Flags.Num(), Sim.Num());
		for (int32 i = 0; i < N; ++i)
		{
			const uint8 F = Flags[i];
			if (F == 0)
			{
				continue;
			}
			if (F & RopeNodeOverride::Position)         { Sim.Positions[i] = Positions[i]; }
			if (F & RopeNodeOverride::Prev)             { Sim.PrevPositions[i] = PrevPositions[i]; }
			if (F & RopeNodeOverride::PrevFromPosition) { Sim.PrevPositions[i] = Sim.Positions[i]; }
			if (F & RopeNodeOverride::InvMass)          { Sim.InvMass[i] = InvMass[i]; }
		}
	}
};
