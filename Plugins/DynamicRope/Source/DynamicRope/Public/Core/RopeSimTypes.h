// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class USceneComponent;

/**
 * narrow-phase contact: one rope node vs one collider, returned by IRopeCollider::Query.
 *
 * CONTRACT — FROZEN 2026-06-24 (2026-06-27 SurfaceVelocity added: backwards compatible as it is an additive field with default 0).
 * All IRopeColliders (capsule, bone-SDF, world-GDF) must comply with this.
 * Describes a single (node, collider) pair. Aggregation is up to the caller (solver aggregates push-outs,
 * DecideWrap selects the bone with the deepest penetration on a per-node basis).
 *
 *   bHit node The sphere (center = query WorldPos, radius = query Radius) overlaps the collider.
 *                false => All remaining fields are undefined. The caller should ignore this.
 *   Normal UNIT, points outward from the collider toward the node (push-out direction).
 *                Invariant: NodePos += Normal*Penetration places a node on the surface.
 *                *** The sign is load-bearing: the inward-facing normal pulls the rope into the body. ***
 *                degenerate (node is on medial axis) => arbitrary non-static unit vector (capsule: +Z).
 *   Overlap depth according to Penetration Normal, when bHit is > 0. Measured based on QUERY radius:
 *                (ColliderRadius + QueryRadius) - Distance. The caller sets QueryRadius 0 to solver push-out,
 *                Pass WrapConfig.ContactQueryRadius to wrap-decision skin.
 *   Point on the collider surface closest to the SurfacePoint node (auxiliary/debug). It is not required for the solver,
 *                Fill when you can get it cheaply.
 *   In Bone skeletal collider, DecideWrap must be non-None — bone attribution (attribution).
 *                Wrap. Multi-bone SDFs must report the bone that owns the closest surface. world => None.
 *   SourceMesh A skeletal mesh that owns Bone. Passes follow between actors (-> FRopeWrapState::Mesh).
 *                null for non-skeletal colliders.
 *   SurfaceVelocity World velocity (cm/s) of the collider surface at the point of contact. The solver uses the relative tangential velocity friction.
 *                Used to drag a rope (the moving body sweeps the stationary rope left and right).
 *                static/Not supported Collider is set to 0 (= static surface) — Same as existing behavior.
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

/** rope centerline: The chain of particles. Single source of truth for solver / logic / render.*/
struct FRopeSimState
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;
	float           SegmentLength = 0.0f;
	float           RopeLength = 0.0f;

	/**
	 * Pinned starting point (hand/socket). The solver sweeps from Prev->Target across substeps.
	 * Fast anchor jumps absorb energy instead of injecting it (which would explode the chain).
	 */
	bool            bStartPinned = false;
	FVector         StartPinPrev = FVector::ZeroVector;
	FVector         StartPinTarget = FVector::ZeroVector;

	/** Fixed timestep accumulator. The solver consumes real frame time in fixed-size substeps. */
	float           TimeAccumulator = 0.0f;

	/**
	 * Tension (force, stretch = positive numbers only) for each segment. Derived from the convergence λ of the XPBD distance constraint: F = max(0, -λ)/h².
	 * The unit is mass·cm/s² (relative value) based on mass 1 node — the threshold value is tuned to actual measurements. CPU solver at the end of step
	 * is filled, and the GPU-resident rope is filled by λ readback (1-2 frame delay). Frames without solve maintain the previous value.
	 * Size = Num()-1 (may be empty — never solved yet).
	 */
	TArray<float>   SegmentTension;

	int32 Num() const { return Positions.Num(); }
	void  Reset() { Positions.Reset(); PrevPositions.Reset(); InvMass.Reset(); SegmentTension.Reset(); TimeAccumulator = 0.0f; }

	//~ Verlet vocabulary (pure inline — no context/policy). Give names to iteration idioms to prevent sign and dimension mistakes.
	//  The solver integration loop (RopeXPBDSolver) and throwing velocity injection loop intentionally keep the raw representation —
	//  The former is a 1:1 parity comparison with the .usf kernel, and the latter is a cumulative type (displacement unit impulse), so the form is different.

	/** One frame displacement (Pos - Prev) of node i. In Verlet, velocity ∝ displacement (before dividing by dt).*/
	FVector Displacement(int32 i) const { return Positions[i] - PrevPositions[i]; }

	/** Movement distance for one frame of node i (cm/frame). Threshold checks such as “fast nodes” are consumer policies.*/
	float NodeSpeed(int32 i) const { return Displacement(i).Size(); }

	/** velocity of node i 0 (Prev = Pos). Seed/reseed path only — Write the position and velocity of the logic phase
	 *  FRopeNodeOverrideFrame single pass (G2, GPU-resident synchronization).*/
	void SetStill(int32 i) { PrevPositions[i] = Positions[i]; }
};

/**
 * per-node bit in FRopeNodeOverrideFrame::Flags. The number should be 1:1 with ERopeGPUOverride (RopeGPUSolver.h)
 * (verified by subsystem) — Core does not depend on the Shaders module, so it mirrors the constants.
 */
namespace RopeNodeOverride
{
	/** Pos[i] = Positions[i] */
	constexpr uint8 Position         = 1 << 0;

	/** Prev[i] = PrevPositions[i] (Verlet velocity injection)*/
	constexpr uint8 Prev             = 1 << 1;

	/** Prev[i] = Pos[i] (velocity 0; value *after* application of Position)*/
	constexpr uint8 PrevFromPosition = 1 << 2;

	/** InvMass[i] = InvMass[i] */
	constexpr uint8 InvMass          = 1 << 3;
}

/**
 * Output of one frame of logic phase (G2): "Target calculation is GT, application is one path".
 * If logic such as Wrapping/Wrapped/Releasing scatters the position/velocity/mass you want to use in the Sim,
 * Applied to CPU Sim once at the end of PrepareSimFrame (ApplyToSim — same result as existing direct write),
 * The same data is Loaded into the GPU-resident rope as an override pass (FRopeGPUResidentStep) without reseeding.
 * Applies to kernel. If you fill the same node multiple times, the last one wins (same as sequential SIM writing).
 * Caution: Do not mix Prev (explicit) and PrevFromPosition in one frame — due to kernel application order
 * PrevFromPosition always wins, making the filling order irrelevant (only PrevFromPosition is used as the logic phase).
 */
struct FRopeNodeOverrideFrame
{
	/** per-node RopeNodeOverride bit OR (if empty, no output this frame).*/
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

	/** At the first scatter, the number of nodes is secured as 0 (recall within the frame is no-op).*/
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

	/** Position pinned: Pos=World, bZeroVelocity then Prev=Pos (velocity 0 — standard writing for Wrapping/hold).*/
	void SetPosition(int32 NodeIndex, const FVector& World, bool bZeroVelocity)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::Position | (bZeroVelocity ? RopeNodeOverride::PrevFromPosition : 0);
			Positions[NodeIndex] = World;
		}
	}

	/** mass Overwrite (mask/restore).*/
	void SetInvMass(int32 NodeIndex, float Value)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::InvMass;
			InvMass[NodeIndex] = Value;
		}
	}

	/** Only remove velocity (Prev=current Pos — position remains the same). Release series splash prevention.*/
	void SetPrevFromPosition(int32 NodeIndex)
	{
		if (Flags.IsValidIndex(NodeIndex))
		{
			Flags[NodeIndex] |= RopeNodeOverride::PrevFromPosition;
		}
	}

	/** CPU application — Same order as the override stage of the GPU kernel (Pos → Prev → Prev=Pos → InvMass).*/
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
