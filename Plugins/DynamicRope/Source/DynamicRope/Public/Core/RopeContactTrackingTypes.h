// Copyright 2026 TeamKeno. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class USceneComponent;
struct FRopeSimState;

/**
 * Where a Flight contact candidate came from. The values combine as bits, so a node and bone caught
 * through several paths has them ORed together in SourceMask.
 * Actual is a real swept contact along this frame's travel path, from Prev to Pos. PredictiveFree is
 * a predicted contact from extrapolating a free node's inertia, and PredictiveGuided is a predicted
 * contact from extrapolating the whip guide targets.
 */
enum class ERopeContactCandidateSource : uint8
{
	Actual = 1,
	PredictiveFree = 2,
	PredictiveGuided = 4
};

/** One contact candidate, per node and bone, consumed by the Flight and Contacting phases. It is an
 *  FRopeContact plus the output of the relative motion evaluation. */
struct FRopeContactCandidate
{
	bool bValid = false;
	int32 NodeIndex = INDEX_NONE;
	FName Bone = NAME_None;
	const USceneComponent* Mesh = nullptr;
	ERopeContactCandidateSource Source = ERopeContactCandidateSource::Actual;
	uint8 SourceMask = static_cast<uint8>(ERopeContactCandidateSource::Actual);

	FVector WorldPoint = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	FVector SurfaceVelocity = FVector::ZeroVector;

	float Penetration = 0.0f;
	float RelativeTangentialSpeed = 0.0f;

	/** Positive when the motion is in the wrapping direction and negative when it opposes it. */
	float WrapDirectionScore = 0.0f;
};

/**
 * A snapshot of the rope's travel frame at the moment of capture, that is the Flight to Contacting
 * transition. There is no solve from Contacting onwards, so the nodes come to rest and the questions
 * "which way was the rope flying" and "how did it come to lie" can only be answered at this instant.
 * BuildContactingState fills it in and ResetTransientPhaseState discards it.
 * Its consumer is the CaptureTravelPlane axis, where it provides the fallback plane normal for a
 * throw with no guide plane and the region centre used as the axis origin. On a GPU-resident rope the
 * CPU mirror can be one to two frames stale, which is still accurate enough for the direction
 * components.
 */
struct DYNAMICROPE_API FRopeCaptureTravelFrame
{
	bool bValid = false;

	/** The mean of the candidates' surface points, which is the centre of the contact region in world
	 *  space. */
	FVector RegionCenter = FVector::ZeroVector;

	/** The mean Verlet velocity of the contacting nodes (cm/s). Zero when the delta time is at or
	 *  below 0. */
	FVector AverageVelocity = FVector::ZeroVector;

	/** The unit direction from head to tail across the contact span, that is the direction the rope
	 *  lies in. With only one contacting node, the span is widened to its neighbours to measure it. */
	FVector SpanDirection = FVector::ZeroVector;

	/** Whether deriving a normal from AverageVelocity crossed with SpanDirection succeeded. It is
	 *  false when the velocity is zero or the two are parallel, which signals a natural fallback. */
	bool bHasPlaneNormal = false;

	/** The derived travel plane normal, as a unit vector. Its sign is interpreted by the consumer,
	 *  meaning the winding decision and OrientWrappingAxisByTail. */
	FVector PlaneNormal = FVector::ZeroVector;

	void Reset()
	{
		*this = FRopeCaptureTravelFrame();
	}

	/** Computes the snapshot from the simulation state and candidates at the moment of capture. Free of
	 *  UObject dependencies; implemented in RopeTypes.cpp. */
	static FRopeCaptureTravelFrame Compute(const FRopeSimState& Sim,
		const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);
};

/** The per-target contact aggregate, keyed by (mesh, bone), that the tracker maintains alongside the
 *  dominant target. It is the input to seeding several wraps at once. */
struct FRopeTrackedContactTarget
{
	FName Bone = NAME_None;
	const USceneComponent* Mesh = nullptr;

	/** The nodes touching this target this frame, replaced with the latest candidates on every
	 *  update. */
	TArray<int32> Nodes;

	/** How long contact with this target has been sustained. On frames with no contact it decays by
	 *  the same amount, and the target is dropped from the list once it reaches 0. */
	float DwellTime = 0.0f;
};

/**
 * A plain-data tracker that follows the dominant target, as a (mesh, bone) pair, across the contact
 * candidates. Keying on the bone name alone would merge and misattribute candidates when two actors
 * sharing a skeleton are touched at the same time, so the mesh is part of the key.
 * It is shared by the Flight capture decision and the dwell tracking in Contacting.
 * Ties are broken first by whichever target has a node closer to the head, that is the hand, and then
 * by score, which combines penetration and wrap direction, which keeps it stable between frames.
 * When the target changes, whether the bone or the mesh, DwellTime restarts from 0, which guards
 * against false positives on the transition frame.
 * Alongside the dominant target it keeps every (mesh, bone) currently in contact in Targets, together
 * with their dwell times, which is what lets several wrap seeds be chosen when MaxWrapSeeds is above
 * 1. The rules for selecting and resetting the dominant target are unaffected by Targets.
 */
struct DYNAMICROPE_API FRopeContactTracker
{
	FName CandidateBone = NAME_None;
	const USceneComponent* CandidateMesh = nullptr;

	TArray<int32> CandidateNodes;
	float DwellTime = 0.0f;

	/** The aggregate for every target in contact, keyed by (mesh, bone). The dominant target is
	 *  included and can be looked up by the same key. */
	TArray<FRopeTrackedContactTarget> Targets;

	void Reset()
	{
		CandidateBone = NAME_None;
		CandidateMesh = nullptr;
		CandidateNodes.Reset();
		DwellTime = 0.0f;
		Targets.Reset();
	}

	void Decay(float DeltaTime)
	{
		DwellTime = FMath::Max(0.0f, DwellTime - DeltaTime);

		// On a frame with no contact at all, drain every target's dwell at the same rate, which keeps
		// the same tolerance for brief flicker. The node lists are cleared because they do not describe
		// this frame's contact and must not be consumed stale.
		for (int32 Index = Targets.Num() - 1; Index >= 0; --Index)
		{
			Targets[Index].DwellTime -= DeltaTime;
			Targets[Index].Nodes.Reset();
			if (Targets[Index].DwellTime <= 0.0f)
			{
				Targets.RemoveAt(Index);
			}
		}

		if (DwellTime <= 0.0f)
		{
			Reset();
		}
	}

	// Assisted multi-bone contract: Targets must keep every permitted bone while CandidateBone alone
	// is pinned to the aimed bone. That is why the preferred target input, which overrides the normal
	// ranking, lives on this shared tracker.
	/** Aggregates the candidates by (mesh, bone) pair and updates the dominant target, its nodes and
	 *  its dwell time.
	 *  A preferred target present among the current candidates takes priority over the normal ranking.
	 *  With bRequirePreferred set and no such candidate, the dominant target is cleared while the
	 *  secondary aggregates in Targets are kept. Implemented in RopeTypes.cpp. */
	void Update(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime,
		const USceneComponent* PreferredMesh = nullptr, FName PreferredBone = NAME_None,
		bool bRequirePreferred = false);
};
