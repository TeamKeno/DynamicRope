// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The whip swing presentation at the start of a throw. It rotates one coherent guide from the
// preserved reference frame's backward direction through the hemisphere selected by SwingPlane. For
// an aim-hit throw, that continuous spherical path ends at the live hand-to-target direction without
// replacing the initial direction with the opposite of the target.
// Full Simulation begins on a shallow C-shaped guide whose free end lies opposite the reference
// forward and becomes exactly straight halfway through the whip timer. Both crossfade after
// GuidedLength. Active only during Flight.
//
// Computing the targets, meaning the sweep angle, the guide curve and the resampling to node
// spacing, stays on the game thread. The CPU solver and GPU resident override sweep the same CurrentTargets,
// PrevTargets and GuidedNodeMask across their substeps as a kinematic path. Aim-hit retains its endpoint
// solver-state blend while using that same temporal application.

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

		/** The fraction of the rope length the guide fully controls, from 0 to 1. */
		float GuidedLength = 0.65f;

		/** The untargeted angle swept from the starting direction round to the reference forward. A
		 *  targeted throw always starts at reference backward and uses SwingPlane as its hemisphere. */
		float SweepAngleDegrees = 180.0f;

		/** The throw speed at which Duration is used as given. */
		float ReferenceThrowSpeed = 1500.0f;

		/** Used when sizing the guide: the larger of Sim.RopeLength and this value. */
		float ComponentRopeLength = 0.0f;

		/** Full Simulation's initial C-shape amplitude as a fraction of guide length, and the normalized
		 *  whip time at which that curvature reaches zero. */
		float FullSimInitialCurveFraction = 0.12f;
		float FullSimStraightenTimeFraction = 0.50f;

		/** The shared crossfade length after GuidedLength. AimHitDirectionBias is retained for serialized
		 *  compatibility; the current continuous targeted swing does not use directional lerping. */
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
	 * Called on a throw: stores the final impulse/target direction separately from the pre-hit reference
	 * axes that fix the backward start and choose the swing hemisphere.
	 */
	void Begin(const FVector& InAimDir, const FVector& InOrigin,
		const FVector& InGuideForward, const FVector& FallbackUp, const FVector& FallbackSide,
		float InThrowSpeed = 0.0f, const FVector& InInheritedVelocity = FVector::ZeroVector,
		bool bInHasAimTarget = false, const FVector& InAimTarget = FVector::ZeroVector,
		float InAimSteerStartAlpha = 0.25f, float InAimLockAlpha = 0.50f);

	/**
	 * Initializes the pose immediately after a throw, at time zero: every physical flight uses the same
	 * initial C-shaped guide. Advance removes the curvature by FullSimStraightenTimeFraction, moves the
	 * fully guided boundary towards GuidedLength, and crossfades into the solver-owned tail. An aim hit
	 * changes only the final forward direction.
	 * Called once from StartFreshThrow.
	 */
	void SnapToInitialPose(FRopeSimState& Sim, const FConfig& Config);

	/**
	 * Called every frame during Flight on the game thread: advances the elapsed time and computes the
	 * guide targets and mask only, leaving the simulation state untouched.
	 * The two runtime application paths consume the same output: the CPU solve path receives a
	 * FRopeKinematicTargetFrame, and the GPU resident path uses a KinematicPath override carried on the step.
	 * The guide deactivates itself once the swing ends, that is when the elapsed time reaches the
	 * duration.
	 */
	void Advance(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config);

	/**
	 * Direct target application helper used by focused guide tests and as the initial CPU mirror update. The
	 * runtime solve additionally supplies the same targets as a substep kinematic path.
	 */
	void ApplyToSim(FRopeSimState& Sim) const;

	/** For predictive contact: previews the guide targets as of the next frame, that is at the elapsed
	 *  time plus the delta, without changing any state. */
	void PreviewNextTargets(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config,
		TArray<FVector>& OutTargets) const;

	/** Clears this frame's output alone, so no stale data is left on an inactive frame. */
	void ResetFrameOutputs();

	bool IsActive() const { return bActive; }
	bool HasAimTarget() const { return bHasAimTarget; }
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

	/** An aim target is kept only as a final direction plus the temporal blend parameters, never as a
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
