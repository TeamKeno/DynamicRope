// Copyright Epic Games, Inc. All Rights Reserved.
//
// The frame contract between URopeComponent and URopeSimSubsystem. It gathers into one type the
// per-frame inputs and outputs the subsystem reads or writes through friend access, which makes the
// boundary explicit: unlike the rest of the component's private state, the values in here are filled
// in or consumed by the subsystem each frame.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeContactTrackingTypes.h"
#include "Core/RopeSimTypes.h"

class IRopeCollider;
class USceneComponent;

/**
 * One rope's simulation inputs and outputs for one frame, as the subsystem's frame contract.
 * Lifetime summary; the full flow is described in the three-stage contract comment on
 * URopeComponent::PrepareSimFrame:
 *  - Frame scoped, reset or rewritten every frame: FrameColliders, AimFrameColliders, OverrideFrame,
 *    bSolveThisFrame, bSolveCollisionsThisFrame, bForceNonStretchThisFrame, bGpuSteppedThisFrame, the
 *    GPU attribution arrays, GpuFlightCandidates and bGpuContactsThisFrame.
 *  - Persisting across frames: SimGeneration, which only increases on a genuine reseed;
 *    AimRayColliderQueryBounds, kept for as long as aiming lasts; and
 *    LockedTargetColliderQueryBounds, kept through the Flight, Contacting and Wrapping phases of an
 *    aim lock.
 */
struct FRopeSimFrameIO
{
	/**
	 * One frame's collider snapshot, filled in by RopeSimSubsystem during Tick from the central gather,
	 * which runs the provider registry and then filters per rope. Read during Prepare, Solve and
	 * Finalize. The colliders belong to the providers, so these are raw pointers valid for that frame
	 * only.
	 */
	TArray<IRopeCollider*> FrameColliders;

	/**
	 * The aiming-only collider snapshot, gathered from the union of the rope's AABB and the aim ray
	 * region. It is deliberately separate from FrameColliders above, which physics, contact detection
	 * and debugging use: aiming at a distant target must not load every bone collider of that target
	 * into solver packing, contact detection and node proximity debug queries.
	 * Consumers are the aim ray hit test and the GuaranteedWrap preview build. Gathering follows the
	 * same rules as FrameColliders, namely the owner exclusion, the static budget and the cross-actor
	 * rule, and the same collider can appear in both lists.
	 * It can still be refilled from an active lock's cached target bounds even when
	 * AimRayColliderQueryBounds is invalid. Pointer lifetime matches FrameColliders.
	 */
	TArray<IRopeCollider*> AimFrameColliders;

	/** The AABB the aim ray tests against, which is the gather region for AimFrameColliders above and
	 *  is unrelated to the physics gather region. */
	FBox AimRayColliderQueryBounds = FBox(ForceInit);

	/**
	 * The union of the target collider bounds from the first frame an aimed throw was committed. It is
	 * a throw-lifetime cache that keeps re-gathering a target into AimFrameColliders even after the
	 * wielder clears the ray bounds on the following frame, since the target can move ahead of the
	 * delayed CPU mirror of the GPU state. FilterFrameCollidersForAimWrapTarget refreshes and clears
	 * it against the permitted target only, and colliders gathered through these bounds are promoted
	 * into FrameColliders only after passing that target filter.
	 */
	FBox LockedTargetColliderQueryBounds = FBox(ForceInit);

	/**
	 * Whether to run Solver.Step this frame. True in Free, Flight, Wrapping and Wrapped, where
	 * Wrapping and Wrapped rely on position overrides and an inverse mass of 0 on anchored nodes.
	 * False in Contacting and Releasing, which are driven by logic.
	 */
	bool bSolveThisFrame = false;

	/** Limits the maximum stretch to 1.0 for this frame alone, on frames where dynamic node ownership
	 *  changes, such as the Wrapping to Wrapped commit. */
	bool bForceNonStretchThisFrame = false;

	/** An aim-hit flight can solve distance, bending and damping only and disable SDF and collider
	 *  push-out. Independent of the contact detection list. */
	bool bSolveCollisionsThisFrame = true;

	/**
	 * One frame's output from the logic phases. While the logic for Wrapping, Wrapped, Releasing and
	 * so on runs during Prepare, it scatters positions and masses into here; at the end of Prepare they
	 * are applied to the CPU simulation state once, and for a GPU-resident rope the subsystem carries
	 * the same data as an override pass so the kernel applies it without a reseed. Reset at the start
	 * of every Prepare, making it frame scoped.
	 */
	FRopeNodeOverrideFrame OverrideFrame;

	/**
	 * The seed generation for the GPU-resident solver. It increases only on a genuine reseed, meaning
	 * initialization, a throw, or a change in node count, which is how the subsystem detects that the
	 * persistent GPU buffers must be reseeded. Position and mass writes from the logic phases and the
	 * whip are injected through the override pass instead and do not reseed, so residency is preserved
	 * across every phase.
	 */
	uint32 SimGeneration = 0;

	/**
	 * Whether this rope was actually stepped on the GPU this frame, set by the subsystem every frame.
	 * It decides whether the GPU tube render reads the resident position buffer directly, when true, or
	 * draws from the CPU mirror, when false, as on the CPU fallback or with the solver off.
	 * Whip frames are on the GPU too, through override injection, so this is true for them as well and
	 * the position buffer reflects the guide targets within the same frame.
	 */
	bool bGpuSteppedThisFrame = false;

	/**
	 * The attribution table for GPU contact detection, which turns a collider index emitted by the GPU
	 * back into a (bone, mesh) pair. The subsystem fills it in on every GPU step frame in the same
	 * order as Step.Capsules and Step.SDFColliders. The mesh is weak because it can be destroyed
	 * during the delay.
	 * A delayed GPU contact, one to two frames old, carries a collider index relative to the set as it
	 * was at dispatch time, whereas this table is rebuilt for the current frame, so a set that changed
	 * during the delay would map an index onto a different bone. The signature below is sent with the
	 * dispatch and returned with the result, so the two can be compared exactly and mismatches dropped,
	 * which prevents misattribution.
	 */
	struct FGpuColliderAttribution
	{
		FName Bone = NAME_None;
		TWeakObjectPtr<const USceneComponent> Mesh;
	};

	/** Parallel to the GPU capsules. */
	TArray<FGpuColliderAttribution> GpuCapsuleAttribution;

	/** Parallel to the GPU SDF colliders. */
	TArray<FGpuColliderAttribution> GpuSdfAttribution;

	/** Parallel to the GPU boxes, for attributing detections against wrappable boxes. */
	TArray<FGpuColliderAttribution> GpuBoxAttribution;

	/** Parallel to the GPU convexes, for attributing detections against wrappable convexes. */
	TArray<FGpuColliderAttribution> GpuConvexAttribution;

	/**
	 * An ordered signature over the (bone, mesh) pairs of the attribution sets above. The subsystem
	 * recomputes it on every GPU step frame that detects, carries it on the dispatch as
	 * FRopeGPUResidentStep::AttribSig, and the detection result carries the same value back as
	 * FRopeResidentContacts::AttribSig. BuildGpuFlightCandidates consumes the delayed contacts only
	 * when the result's signature matches the current one, and otherwise drops them exactly as it would
	 * a generation mismatch, leaving no GPU candidates for that frame.
	 *
	 * Comparing the dispatch-time value directly is exact regardless of how long the delay is. An
	 * approximation such as "stable if the last few frames all match" would let a delay longer than
	 * that window through after the colliders had been reordered, which could attribute an arm contact
	 * to a leg; conversely, a set that changed and came back to the same arrangement is consumed
	 * correctly here.
	 * 0 means unset, which happens while warming up just after entering Flight and is dropped safely.
	 */
	uint32 GpuAttribSig = 0;

	/**
	 * The frame output of GPU detection: the subsystem attributes GetLatestContacts and fills this in
	 * before Finalize. While valid, FinalizeSimFrame uses these candidates as its Flight contact source
	 * instead of a CPU sweep.
	 */
	TArray<FRopeContactCandidate> GpuFlightCandidates;
	bool bGpuContactsThisFrame = false;
};
