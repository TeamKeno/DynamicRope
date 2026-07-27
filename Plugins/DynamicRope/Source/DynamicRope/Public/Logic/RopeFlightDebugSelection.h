// Copyright Epic Games, Inc. All Rights Reserved.
//
// Candidate box selection for the Flight debugger. It is a pure function, so it can be unit tested
// without a world or any UObject, and is shared by the debugger drawing, under
// WITH_GAMEPLAY_DEBUGGER, and by the automated tests. It consumes only the arrays the snapshot has
// already collected and gathers nothing further.
// Meshes are identified by FObjectKey rather than by raw pointer, because a snapshot outlives its
// frame by several frames and the pointer may be dead by then; both comparison and resolution go
// through the key.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectKey.h"
#include "Core/RopeContactTrackingTypes.h"

namespace RopeFlightDebug
{
	// The result of selecting candidate boxes. Every index refers to the input Candidates array.
	struct FCandidateSelection
	{
		// The index of the capture target candidate, meaning the one whose mesh and bone match the
		// tracker, or INDEX_NONE when there is none.
		int32 CaptureTargetIndex = INDEX_NONE;
		// The indices of the boxes to draw. When a capture target exists it is at index 0, followed by
		// the general candidates with the greatest penetration.
		TArray<int32> BoxIndices;
		// The total number of valid candidates, reported as the candidate count in the summary.
		int32 TotalValid = 0;
		// The number of boxes actually drawn, that is BoxIndices.Num(), and how many valid candidates
		// were left out.
		int32 Shown = 0;
		int32 Hidden = 0;
	};

	/**
	 * Chooses which candidate boxes to draw. The capture target is always included even when it falls
	 * outside the top N by penetration: what the rope is about to catch on is a predictive candidate,
	 * so the moment its penetration is low is exactly the moment worth seeing.
	 *  - MeshKeys is one-to-one with Candidates, sharing indices. Where it is shorter, as with an older
	 *    snapshot, those indices are matched against the tracker by bone alone.
	 *  - MaxBoxes caps how many boxes are drawn.
	 *  - bFillWithGeneral fills the remaining slots with the general candidates that have the greatest
	 *    penetration; false draws the capture target alone, which is the default.
	 * The ranking is deterministic: penetration descending, then node index ascending, then the
	 * original index ascending.
	 */
	DYNAMICROPE_API FCandidateSelection SelectCandidateBoxes(
		const TArray<FRopeContactCandidate>& Candidates,
		const TArray<FObjectKey>& MeshKeys,
		FName TrackerBone,
		FObjectKey TrackerMeshKey,
		int32 MaxBoxes,
		bool bFillWithGeneral);
}
