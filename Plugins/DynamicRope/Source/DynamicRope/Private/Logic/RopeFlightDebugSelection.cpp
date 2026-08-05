// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeFlightDebugSelection.h"

namespace RopeFlightDebug
{
	namespace
	{
	// Ranks two candidates by their original indices: greater penetration first, then the lower node index on a tie,
	// and then the lower original index. That last tie-break makes the ordering fully deterministic.
		bool RanksBefore(const TArray<FRopeContactCandidate>& C, int32 A, int32 B)
		{
			if (C[A].Penetration != C[B].Penetration)
			{
				return C[A].Penetration > C[B].Penetration;
			}
			if (C[A].NodeIndex != C[B].NodeIndex)
			{
				return C[A].NodeIndex < C[B].NodeIndex;
			}
			return A < B;
		}

	// Whether this candidate is the tracked target, by mesh and bone. The bone is always compared and the mesh is
	// compared when a key is present; with no key, as in an older snapshot, the bone alone decides. A tracker bone of
	// none means there is no tracked target at all.
		bool MatchesTracker(const FRopeContactCandidate& Cand, const TArray<FObjectKey>& MeshKeys, int32 Index,
			FName TrackerBone, FObjectKey TrackerMeshKey)
		{
			if (TrackerBone.IsNone() || Cand.Bone != TrackerBone)
			{
				return false;
			}
			return !MeshKeys.IsValidIndex(Index) || MeshKeys[Index] == TrackerMeshKey;
		}
	}

	FCandidateSelection SelectCandidateBoxes(
		const TArray<FRopeContactCandidate>& Candidates,
		const TArray<FObjectKey>& MeshKeys,
		FName TrackerBone,
		FObjectKey TrackerMeshKey,
		int32 MaxBoxes,
		bool bFillWithGeneral)
	{
		FCandidateSelection Out;

	// Collects the original indices of the valid candidates alone, since both the summary and the boxes are based on valid candidates.
		TArray<int32> ValidIdx;
		ValidIdx.Reserve(Candidates.Num());
		for (int32 i = 0; i < Candidates.Num(); ++i)
		{
			if (Candidates[i].bValid)
			{
				ValidIdx.Add(i);
			}
		}
		Out.TotalValid = ValidIdx.Num();

	// The representative, being the tracked target: the highest-ranked valid candidate whose mesh and bone match the
	// tracker. The walk is in ascending order, so on a complete tie the lower original index survives, since the rank
	// comparison is strict and the first seen is kept, which is deterministic.
		for (int32 i : ValidIdx)
		{
			if (!MatchesTracker(Candidates[i], MeshKeys, i, TrackerBone, TrackerMeshKey))
			{
				continue;
			}
			if (Out.CaptureTargetIndex == INDEX_NONE || RanksBefore(Candidates, i, Out.CaptureTargetIndex))
			{
				Out.CaptureTargetIndex = i;
			}
		}

	// The box list, with the representative first if there is one. The default view ends here, with one representative or none.
		if (Out.CaptureTargetIndex != INDEX_NONE && MaxBoxes > 0)
		{
			Out.BoxIndices.Add(Out.CaptureTargetIndex);
		}

	// The remaining slots are filled with the ordinary candidates by penetration, in the advanced view alone. The representative is excluded to avoid duplication.
		if (bFillWithGeneral && Out.BoxIndices.Num() < MaxBoxes)
		{
			TArray<int32> General = ValidIdx;
			General.RemoveAll([&](int32 i) { return i == Out.CaptureTargetIndex; });
			General.Sort([&](int32 A, int32 B) { return RanksBefore(Candidates, A, B); });
			for (int32 i : General)
			{
				if (Out.BoxIndices.Num() >= MaxBoxes)
				{
					break;
				}
				Out.BoxIndices.Add(i);
			}
		}

		Out.Shown = Out.BoxIndices.Num();
		Out.Hidden = FMath::Max(0, Out.TotalValid - Out.Shown);
		return Out;
	}
}
