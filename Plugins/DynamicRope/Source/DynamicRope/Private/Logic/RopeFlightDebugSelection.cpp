// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeFlightDebugSelection.h"

namespace RopeFlightDebug
{
	namespace
	{
		// 두 후보의 원본 인덱스를 순위 비교: penetration 큰 쪽 → 동률이면 NodeIndex 작은 쪽 → 그래도
		// 동률이면 원본 인덱스 작은 쪽이 앞선다. 마지막 인덱스 tie-break가 정렬을 완전 결정적으로 만든다.
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

		// 이 후보가 tracker 대상((Mesh,Bone))인가. Bone은 항상 대조, Mesh는 키가 있으면 대조한다(키가
		// 없으면 — 구 스냅샷 — 본만으로 판단). TrackerBone이 None이면 tracker 자체가 없다.
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

		// 유효 후보 원본 인덱스만 모은다 — 요약/박스 모두 유효 후보 기준이다.
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

		// 대표(포착 대상): tracker와 (Mesh,Bone)이 일치하는 유효 후보 중 순위 1위. 순회는 오름차순이라
		// 완전 동률이면 원본 인덱스가 작은 쪽이 남는다(RanksBefore가 strict라 first-seen 유지 = 결정적).
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

		// 박스 목록: 대표 먼저(있으면). 기본 [U]은 여기서 끝(대표 1개 또는 0개).
		if (Out.CaptureTargetIndex != INDEX_NONE && MaxBoxes > 0)
		{
			Out.BoxIndices.Add(Out.CaptureTargetIndex);
		}

		// 남는 칸을 penetration 상위 일반 후보로 채운다(Advanced만). 대표는 제외해 중복을 막는다.
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
