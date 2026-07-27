// Copyright Epic Games, Inc. All Rights Reserved.
//
// The whip swing presentation at the start of a throw. It rotates a guide curve over time, sweeping
// from the side opposite the aim round to the aim direction. An ordinary throw takes hold of the
// leading stretch of the guide, while an aim-hit throw holds the middle firmly and blends smoothly
// into the solver's own state towards the hand and the free end. It is active only during Flight.
//
// Computing the targets, meaning the sweep angle, the guide curve and the resampling to node
// spacing, stays on the game thread, while application is done either by ApplyToSim on the CPU or by
// the GPU resident override pass, both consuming the same frame output. CurrentTargets, PrevTargets
// and GuidedNodeMask are the data contract shared by the two paths.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeSimTypes.h"
#include "Core/RopeThrowTypes.h"

class DYNAMICROPE_API FRopeWhipGuide
{
public:
	/**
	 * A snapshot of the designer settings. The properties themselves stay on URopeComponent, under the
	 * Rope|Whip category, to keep their serialization path, and are copied here on every call.
	 */
	struct FConfig
	{
		/** Total swing duration (s). */
		float Duration = 0.35f;

		/** The fraction of the rope length the guide controls, from 0 to 1. */
		float GuidedLength = 0.65f;

		/** The angle swept from the starting angle, opposite the aim, round to the aim direction. */
		float SweepAngleDegrees = 180.0f;

		/** The throw speed at which Duration is used as given. */
		float ReferenceThrowSpeed = 1500.0f;

		/** Used when sizing the guide: the larger of Sim.RopeLength and this value. */
		float ComponentRopeLength = 0.0f;

		/** For an aim hit: the stretches at the hand and free ends where the guide relaxes, and the
		 *  exponent that brings the hit direction blend forward. */
		float AimHitRootSolverFraction = 0.20f;
		float AimHitTipSolverFraction = 0.25f;
		float AimHitDirectionBias = 2.0f;
	};

	struct FSwingBasis
	{
		FVector AimDir = FVector::ForwardVector;
		FVector GuideUp = FVector::UpVector;
		FVector GuideRight = FVector::RightVector;
	};

	/** Normalizes a vector, substituting the fallback when it is degenerate. Shared by the throw frame
	 *  and swing basis resolution. */
	static FVector SafeNormalOr(const FVector& Value, const FVector& Fallback);

	/** Resolves a throw context and a swing plane setting into the aim, up and right axes the whip
	 *  guide actually uses. */
	static FSwingBasis ResolveSwingBasis(const FRopeThrowContext& ThrowContext,
		ERopeSwingPlane SwingPlane, const FVector& CustomPlaneNormal);

	/**
	 * Called on a throw: builds the guide frame, that is the forward and up axes, about the aim
	 * direction and activates the swing.
	 * The fallback vectors are the component axes used in degenerate cases, such as a zero aim or one
	 * close to vertical.
	 */
	void Begin(const FVector& InAimDir, const FVector& InOrigin,
		const FVector& FallbackAim, const FVector& FallbackUp, const FVector& FallbackSide,
		float InThrowSpeed = 0.0f, const FVector& InInheritedVelocity = FVector::ZeroVector,
		bool bInHasAimTarget = false, const FVector& InAimTarget = FVector::ZeroVector,
		float InAimSteerStartAlpha = 0.25f, float InAimLockAlpha = 0.50f);

	/**
	 * Snaps the initial pose immediately after a throw, at time zero: an ordinary guide places its
	 * stretch on the targets, while an aim hit places only the middle firmly and blends the envelopes at
	 * both ends with the existing solver positions. Called once from StartFreshThrow.
	 */
	void SnapToInitialPose(FRopeSimState& Sim, const FConfig& Config);

	/**
	 * Called every frame during Flight on the game thread: advances the elapsed time and computes the
	 * guide targets and mask only, leaving the simulation state untouched.
	 * The two application paths consume the same output: the CPU solve path calls ApplyToSim, and the
	 * GPU resident path uses the override pass, with the position and previous-position flags, which
	 * the subsystem carries on the step.
	 * The guide deactivates itself once the swing ends, that is when the elapsed time reaches the
	 * duration.
	 */
	void Advance(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config);

	/**
	 * The application half of the CPU path: writes the targets and mask computed by Advance into the
	 * simulation state, for guided nodes only, setting the position to the current target and the
	 * previous position to the last one so their difference becomes the Verlet velocity. It is not
	 * called for GPU ropes.
	 */
	void ApplyToSim(FRopeSimState& Sim) const;

	/** For predictive contact: previews the guide targets as of the next frame, that is at the elapsed
	 *  time plus the delta, without changing any state. */
	void PreviewNextTargets(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config,
		TArray<FVector>& OutTargets) const;

	/** Clears this frame's output alone, so no stale data is left on an inactive frame. */
	void ResetFrameOutputs();

	bool IsActive() const { return bActive; }
	float GetElapsed() const { return Elapsed; }

	/** The normalized aim direction, after any fallback has been applied. It is also used when
	 *  injecting the throw impulse. */
	const FVector& GetAimDir() const { return AimDir; }

	//~ The frame output, which is the data contract. Filled in by SnapToInitialPose and Advance and
	//~ valid until the next update.
	const TArray<FVector>& GetCurrentTargets() const { return CurrentTargetsThisFrame; }
	const TArray<FVector>& GetPrevTargets() const { return PrevTargetsThisFrame; }
	const TArray<uint8>& GetGuidedNodeMask() const { return GuidedNodesThisFrame; }
	bool IsGuidedNodeThisFrame(int32 NodeIndex) const
	{
		return GuidedNodesThisFrame.IsValidIndex(NodeIndex) && GuidedNodesThisFrame[NodeIndex] != 0;
	}

	/** Counts the guided nodes in the actual frame output. Call it only when statistics or observation
	 *  need it. */
	int32 GetGuidedNodeCountThisFrame() const;

	/** Copies the actual frame output into debug snapshot arrays, leaving the simulation state
	 *  unchanged. */
	void CopyGuidedTargetsForDebug(TArray<int32>& OutNodeIndices, TArray<FVector>& OutTargets) const;

private:
	/** Builds the guide curve at the given normalized time, from 0 to 1, and resamples it to the node
	 *  spacing to fill in the targets. */
	void BuildGuideTargets(float NormalizedTime, int32 LastGuidedNode,
		const FRopeSimState& Sim, const FConfig& Config, TArray<FVector>& OutTargets) const;

	/** Resamples raw curve points at even intervals matching the rope's node spacing, that is its
	 *  segment length. */
	void ResampleGuideByNodeSpacing(const TArray<FVector>& SourcePoints, float NodeSpacing,
		int32 DesiredPointCount, TArray<FVector>& OutPoints) const;

	bool bActive = false;
	float Elapsed = 0.0f;

	FVector AimDir = FVector::ForwardVector;
	FVector Origin = FVector::ZeroVector;
	FVector GuideForward = FVector::ForwardVector;
	FVector GuideUp = FVector::UpVector;
	FVector GuideInheritedVelocity = FVector::ZeroVector;
	float GuideThrowSpeed = 0.0f;

	/** An aim target is kept only as a final direction plus the spatial blend parameters, never as a
	 *  fixed node position. */
	bool bHasAimTarget = false;
	FVector AimTarget = FVector::ZeroVector;
	float AimSteerStartAlpha = 0.25f;
	float AimLockAlpha = 0.50f;

	/** The previous frame's guide targets, which inject the Verlet velocity of the guided nodes by
	 *  becoming their previous positions. */
	TArray<FVector> PreviousTargets;

	//~ Frame output
	TArray<FVector> PrevTargetsThisFrame;
	TArray<FVector> CurrentTargetsThisFrame;
	TArray<uint8> GuidedNodesThisFrame;
};
