// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/RopeDebugDraw.h"
#include "Stats/Stats.h"

#if !UE_BUILD_SHIPPING

DECLARE_STATS_GROUP(TEXT("RopeFlight"), STATGROUP_RopeFlight, STATCAT_Advanced);
DECLARE_DWORD_COUNTER_STAT(TEXT("Components"), STAT_RopeFlightComponents, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Frame Colliders"), STAT_RopeFlightFrameColliders, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Num Particles"), STAT_RopeFlightNumParticles, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Solve This Frame"), STAT_RopeFlightSolveThisFrame, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Candidates"), STAT_RopeFlightCandidates, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Actual Candidates"), STAT_RopeFlightActualCandidates, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Predictive Free Candidates"), STAT_RopeFlightPredictiveFreeCandidates, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Predictive Guided Candidates"), STAT_RopeFlightPredictiveGuidedCandidates, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Candidate Nodes"), STAT_RopeFlightCandidateNodes, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Min Latch Nodes"), STAT_RopeFlightMinLatchNodes, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Should Capture"), STAT_RopeFlightShouldCapture, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Whip Guided Nodes"), STAT_RopeFlightWhipGuidedNodes, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Whip Guide Points"), STAT_RopeFlightWhipGuidePoints, STATGROUP_RopeFlight);
DECLARE_DWORD_COUNTER_STAT(TEXT("Whip Solver Only Nodes"), STAT_RopeFlightWhipSolverOnlyNodes, STATGROUP_RopeFlight);

DECLARE_STATS_GROUP(TEXT("RopeWrapped"), STATGROUP_RopeWrapped, STATCAT_Advanced);
DECLARE_DWORD_COUNTER_STAT(TEXT("Components"), STAT_RopeWrappedComponents, STATGROUP_RopeWrapped);
DECLARE_DWORD_COUNTER_STAT(TEXT("Num Particles"), STAT_RopeWrappedNumParticles, STATGROUP_RopeWrapped);
DECLARE_DWORD_COUNTER_STAT(TEXT("Latched Nodes"), STAT_RopeWrappedLatchedNodes, STATGROUP_RopeWrapped);

namespace
{
	bool IsStatEnabled(TStatId StatId)
	{
		return FThreadStats::IsCollectingData(StatId);
	}
}

bool RopeDebug::IsFlightStatEnabled()
{
	return IsStatEnabled(GET_STATID(STAT_RopeFlightComponents));
}

bool RopeDebug::IsWrappedStatEnabled()
{
	return IsStatEnabled(GET_STATID(STAT_RopeWrappedComponents));
}

void RopeDebug::RecordFlightStats(const FRopeSimState& Sim, bool bSolveThisFrame, int32 FrameColliderCount,
	const TArray<FRopeContactCandidate>& Candidates, const FRopeContactTracker& ContactTracker,
	const FRopeDetectConfig& DetectConfig, bool bShouldCapture)
{
	if (!IsFlightStatEnabled())
	{
		return;
	}

	int32 ActualCandidateCount = 0;
	int32 PredictiveFreeCandidateCount = 0;
	int32 PredictiveGuidedCandidateCount = 0;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if ((Candidate.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::Actual)) != 0)
		{
			++ActualCandidateCount;
		}
		if ((Candidate.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)) != 0)
		{
			++PredictiveFreeCandidateCount;
		}
		if ((Candidate.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided)) != 0)
		{
			++PredictiveGuidedCandidateCount;
		}
	}

	INC_DWORD_STAT(STAT_RopeFlightComponents);
	INC_DWORD_STAT_BY(STAT_RopeFlightFrameColliders, FrameColliderCount);
	INC_DWORD_STAT_BY(STAT_RopeFlightNumParticles, Sim.Num());
	INC_DWORD_STAT_BY(STAT_RopeFlightSolveThisFrame, bSolveThisFrame ? 1 : 0);
	INC_DWORD_STAT_BY(STAT_RopeFlightCandidates, Candidates.Num());
	INC_DWORD_STAT_BY(STAT_RopeFlightActualCandidates, ActualCandidateCount);
	INC_DWORD_STAT_BY(STAT_RopeFlightPredictiveFreeCandidates, PredictiveFreeCandidateCount);
	INC_DWORD_STAT_BY(STAT_RopeFlightPredictiveGuidedCandidates, PredictiveGuidedCandidateCount);
	INC_DWORD_STAT_BY(STAT_RopeFlightCandidateNodes, ContactTracker.CandidateNodes.Num());
	INC_DWORD_STAT_BY(STAT_RopeFlightMinLatchNodes, DetectConfig.MinLatchNodes);
	INC_DWORD_STAT_BY(STAT_RopeFlightShouldCapture, bShouldCapture ? 1 : 0);
}

void RopeDebug::RecordWhipStats(const FRopeSimState& Sim, const TArray<int32>& GuideNodeIndices,
	const TArray<FVector>& GuideTargets, float GuidedEnd, bool bWhipActive)
{
	if (!bWhipActive || !IsFlightStatEnabled())
	{
		return;
	}

	const int32 LastNode = Sim.Num() - 1;
	if (LastNode < 1)
	{
		return;
	}

	const int32 GuidedNodeCount = GuideNodeIndices.Num();
	const int32 LastGuidedNode = FMath::Clamp(FMath::FloorToInt(static_cast<float>(LastNode) * GuidedEnd), 0, LastNode);
	const int32 SolverOnlyCount = FMath::Max(0, LastNode - LastGuidedNode);

	INC_DWORD_STAT_BY(STAT_RopeFlightWhipGuidedNodes, GuidedNodeCount);
	INC_DWORD_STAT_BY(STAT_RopeFlightWhipGuidePoints, GuideTargets.Num());
	INC_DWORD_STAT_BY(STAT_RopeFlightWhipSolverOnlyNodes, SolverOnlyCount);
}

void RopeDebug::RecordWrappedStats(const FRopeSimState& Sim, const FRopeWrapState& Wrap)
{
	if (!Wrap.IsWrapped() || !IsWrappedStatEnabled())
	{
		return;
	}

	INC_DWORD_STAT(STAT_RopeWrappedComponents);
	INC_DWORD_STAT_BY(STAT_RopeWrappedNumParticles, Sim.Num());
	INC_DWORD_STAT_BY(STAT_RopeWrappedLatchedNodes, Wrap.Latched.Num());
}

#else // UE_BUILD_SHIPPING — 모두 no-op

bool RopeDebug::IsFlightStatEnabled() { return false; }
bool RopeDebug::IsWrappedStatEnabled() { return false; }
void RopeDebug::RecordFlightStats(const FRopeSimState&, bool, int32, const TArray<FRopeContactCandidate>&,
	const FRopeContactTracker&, const FRopeDetectConfig&, bool) {}
void RopeDebug::RecordWhipStats(const FRopeSimState&, const TArray<int32>&, const TArray<FVector>&, float, bool) {}
void RopeDebug::RecordWrappedStats(const FRopeSimState&, const FRopeWrapState&) {}

#endif
