// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/RopeDebugDraw.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/Engine.h"
#include "DrawDebugHelpers.h"
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
	TAutoConsoleVariable<int32> CVarRopeDebug(
		TEXT("r.DynamicRope.Debug"), 0,
		TEXT("DynamicRope 디버그 마스터 토글. per-instance bDrawDebug과 OR된다. 0=off, 1=on."),
		ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugCenterline(
		TEXT("r.DynamicRope.Debug.Centerline"), 1,
		TEXT("중심선/노드/latch 노드 표시."), ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugColliders(
		TEXT("r.DynamicRope.Debug.Colliders"), 1,
		TEXT("provider collider(capsule / SDF 볼륨 bounds) 표시. provider별 bDrawDebug과 OR된다."), ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugFlight(
		TEXT("r.DynamicRope.Debug.Flight"), 1,
		TEXT("Flight contact-path debug lines/points. Requires r.DynamicRope.Debug=1. 0=off, 1=on."),
		ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugWrapped(
		TEXT("r.DynamicRope.Debug.Wrapped"), 1,
		TEXT("Wrapped-state debug points/table. Requires r.DynamicRope.Debug=1. 0=off, 1=on."),
		ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugLabels(
		TEXT("r.DynamicRope.Debug.Labels"), 1,
		TEXT("World-space debug text labels for DynamicRope visual debug. 0=off, 1=on."),
		ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugScreenText(
		TEXT("r.DynamicRope.Debug.ScreenText"), 1,
		TEXT("On-screen DynamicRope debug messages. 0=off, 1=on."),
		ECVF_Cheat);

	const TCHAR* DebugPhaseName(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Free:       return TEXT("Free");
		case ERopePhase::Flight:     return TEXT("Flight");
		case ERopePhase::Contacting: return TEXT("Contacting");
		case ERopePhase::Wrapping: return TEXT("Wrapping");
		case ERopePhase::Wrapped:    return TEXT("Wrapped");
		case ERopePhase::Releasing:  return TEXT("Releasing");
		default:                     return TEXT("?");
		}
	}

	FString NodeListString(const TArray<int32>& Nodes)
	{
		FString Result;
		for (int32 i = 0; i < Nodes.Num(); ++i)
		{
			if (i > 0)
			{
				Result += TEXT(",");
			}
			Result += LexToString(Nodes[i]);
		}
		return Result;
	}

	const TCHAR* CandidateSourceName(ERopeContactCandidateSource Source)
	{
		switch (Source)
		{
		case ERopeContactCandidateSource::Actual: return TEXT("actual");
		case ERopeContactCandidateSource::PredictiveFree: return TEXT("predFree");
		case ERopeContactCandidateSource::PredictiveGuided: return TEXT("predGuided");
		default: return TEXT("?");
		}
	}

	FString CandidateSourceMaskName(uint8 SourceMask)
	{
		TArray<const TCHAR*> Parts;
		if ((SourceMask & static_cast<uint8>(ERopeContactCandidateSource::Actual)) != 0)
		{
			Parts.Add(TEXT("actual"));
		}
		if ((SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)) != 0)
		{
			Parts.Add(TEXT("predFree"));
		}
		if ((SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided)) != 0)
		{
			Parts.Add(TEXT("predGuided"));
		}

		FString Result;
		for (int32 i = 0; i < Parts.Num(); ++i)
		{
			if (i > 0)
			{
				Result += TEXT("+");
			}
			Result += Parts[i];
		}
		return Result.IsEmpty() ? FString(TEXT("?")) : Result;
	}

	FColor CandidateSourceColor(ERopeContactCandidateSource Source)
	{
		switch (Source)
		{
		case ERopeContactCandidateSource::Actual: return FColor::Cyan;
		case ERopeContactCandidateSource::PredictiveFree: return FColor::Green;
		case ERopeContactCandidateSource::PredictiveGuided: return FColor(255, 80, 255);
		default: return FColor::White;
		}
	}

	bool IsStatEnabled(TStatId StatId)
	{
		return FThreadStats::IsCollectingData(StatId);
	}

	FColor PhaseColor(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Flight:     return FColor::Cyan;
		case ERopePhase::Contacting: return FColor::Yellow;
		case ERopePhase::Wrapped:    return FColor::Green;
		case ERopePhase::Wrapping:   return FColor(255, 160, 0);
		case ERopePhase::Releasing:  return FColor::Orange;
		case ERopePhase::Free:
		default:                     return FColor(160, 160, 160);
		}
	}
}

bool RopeDebug::IsEnabled(bool bInstanceForce)
{
	return bInstanceForce || CVarRopeDebug.GetValueOnGameThread() != 0;
}

bool RopeDebug::IsFlightVisualEnabled()
{
	return CVarRopeDebug.GetValueOnGameThread() != 0 && CVarRopeDebugFlight.GetValueOnGameThread() != 0;
}

bool RopeDebug::IsWrappedVisualEnabled()
{
	return CVarRopeDebug.GetValueOnGameThread() != 0 && CVarRopeDebugWrapped.GetValueOnGameThread() != 0;
}

bool RopeDebug::IsFlightStatEnabled()
{
	return IsStatEnabled(GET_STATID(STAT_RopeFlightComponents));
}

bool RopeDebug::IsWrappedStatEnabled()
{
	return IsStatEnabled(GET_STATID(STAT_RopeWrappedComponents));
}

void RopeDebug::DrawCenterline(const UWorld* World, const FRopeSimState& Sim, ERopePhase Phase,
	const FRopeWrapState& Wrap, bool bInstanceForce)
{
	if (!World || !IsEnabled(bInstanceForce) || CVarRopeDebugCenterline.GetValueOnGameThread() == 0)
	{
		return;
	}

	const FColor LineColor = PhaseColor(Phase);
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		DrawDebugPoint(World, Sim.Positions[i], 6.0f, FColor::Yellow, false, -1.0f, SDPG_Foreground);
		if (i + 1 < Sim.Num())
		{
			DrawDebugLine(World, Sim.Positions[i], Sim.Positions[i + 1], LineColor, false, -1.0f, SDPG_Foreground, 0.5f);
		}
	}

	// latch된 노드는 굵은 빨간 점으로 강조(어느 노드가 bone에 고정됐는지).
	for (const FRopeLatchNode& Latch : Wrap.Latched)
	{
		if (Sim.Positions.IsValidIndex(Latch.NodeIndex))
		{
			DrawDebugPoint(World, Sim.Positions[Latch.NodeIndex], 12.0f, FColor::Red, false, -1.0f, SDPG_Foreground);
		}
	}
}

void RopeDebug::DrawFlight(const UWorld* World, uint64 DebugKey, const FString& RopeName, const FRopeSimState& Sim,
	ERopePhase Phase, bool bSolveThisFrame, int32 FrameColliderCount,
	const TArray<FRopeFlightNodeDebug>& NodeDebug, const TArray<FRopeContactCandidate>& Candidates,
	const FRopeContactTracker& ContactTracker, const FRopeWrapConfig& WrapConfig, bool bShouldCapture)
{
	const bool bStatEnabled = IsFlightStatEnabled();
	const bool bVisualEnabled = IsFlightVisualEnabled();
	if (!World || (!bStatEnabled && !bVisualEnabled))
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

	if (bStatEnabled)
	{
		INC_DWORD_STAT(STAT_RopeFlightComponents);
		INC_DWORD_STAT_BY(STAT_RopeFlightFrameColliders, FrameColliderCount);
		INC_DWORD_STAT_BY(STAT_RopeFlightNumParticles, Sim.Num());
		INC_DWORD_STAT_BY(STAT_RopeFlightSolveThisFrame, bSolveThisFrame ? 1 : 0);
		INC_DWORD_STAT_BY(STAT_RopeFlightCandidates, Candidates.Num());
		INC_DWORD_STAT_BY(STAT_RopeFlightActualCandidates, ActualCandidateCount);
		INC_DWORD_STAT_BY(STAT_RopeFlightPredictiveFreeCandidates, PredictiveFreeCandidateCount);
		INC_DWORD_STAT_BY(STAT_RopeFlightPredictiveGuidedCandidates, PredictiveGuidedCandidateCount);
		INC_DWORD_STAT_BY(STAT_RopeFlightCandidateNodes, ContactTracker.CandidateNodes.Num());
		INC_DWORD_STAT_BY(STAT_RopeFlightMinLatchNodes, WrapConfig.MinLatchNodes);
		INC_DWORD_STAT_BY(STAT_RopeFlightShouldCapture, bShouldCapture ? 1 : 0);
	}

	if (!bVisualEnabled)
	{
		return;
	}

	TArray<FRopeContactCandidate> SortedCandidates = Candidates;
	SortedCandidates.Sort([](const FRopeContactCandidate& A, const FRopeContactCandidate& B)
		{
			return A.Penetration > B.Penetration;
		});

	TSet<int32> ValidCandidateNodes;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (Candidate.bValid)
		{
			ValidCandidateNodes.Add(Candidate.NodeIndex);
		}
	}

	for (const FRopeFlightNodeDebug& Node : NodeDebug)
	{
		DrawDebugLine(World, Node.PrevPosition, Node.Position, FColor::White, false, -1.0f, SDPG_Foreground, 0.75f);

		if (Node.bNearBody)
		{
			DrawDebugPoint(World, Node.Position, 8.0f, FColor::Yellow, false, -1.0f, SDPG_Foreground);
		}

		if (Node.Contact.bHit)
		{
			const FColor HitColor = ValidCandidateNodes.Contains(Node.NodeIndex) ? FColor::Green : FColor::Red;
			DrawDebugPoint(World, Node.Contact.SurfacePoint, 10.0f, HitColor, false, -1.0f, SDPG_Foreground);
			DrawDebugDirectionalArrow(World, Node.Contact.SurfacePoint,
				Node.Contact.SurfacePoint + Node.Contact.Normal.GetSafeNormal() * 22.0f,
				6.0f, FColor::Blue, false, -1.0f, SDPG_Foreground, 0.75f);
		}
	}

	const int32 MaxLabels = FMath::Min(5, SortedCandidates.Num());
	for (int32 i = 0; i < MaxLabels; ++i)
	{
		const FRopeContactCandidate& Candidate = SortedCandidates[i];
		const FColor SourceColor = CandidateSourceColor(Candidate.Source);
		const FColor CandidateColor = (Candidate.Bone == ContactTracker.CandidateBone)
			? FColor(
				FMath::Min(255, SourceColor.R + 40),
				FMath::Min(255, SourceColor.G + 20),
				FMath::Min(255, SourceColor.B + 40))
			: SourceColor;
		DrawDebugBox(World, Candidate.WorldPoint, FVector(3.5f), CandidateColor, false, -1.0f, SDPG_Foreground, 0.75f);
		if (CVarRopeDebugLabels.GetValueOnGameThread() != 0)
		{
			const FString SourceLabel = CandidateSourceMaskName(Candidate.SourceMask);
			const FString Label = FString::Printf(TEXT("node=%d src=%s primary=%s bone=%s pen=%.2f relTan=%.1f wrap=%.2f"),
				Candidate.NodeIndex, *SourceLabel, CandidateSourceName(Candidate.Source), *Candidate.Bone.ToString(), Candidate.Penetration,
				Candidate.RelativeTangentialSpeed, Candidate.WrapDirectionScore);
			DrawDebugString(World, Candidate.WorldPoint + FVector(0.0f, 0.0f, 10.0f), Label, nullptr,
				CandidateColor, 0.0f, true);
		}
	}

	if (GEngine && CVarRopeDebugScreenText.GetValueOnGameThread() != 0)
	{
		const FString Text = FString::Printf(
			TEXT("[RopeFlight] %s  phase=%s  colliders=%d  nodes=%d  solve=%d\n")
			TEXT("candidates=%d  actual=%d  predFree=%d  predGuided=%d\n")
			TEXT("trackerBone=%s  trackerNodes=%d/%d [%s]  capture=%s"),
			*RopeName, DebugPhaseName(Phase), FrameColliderCount, Sim.Num(), bSolveThisFrame ? 1 : 0,
			Candidates.Num(), ActualCandidateCount, PredictiveFreeCandidateCount, PredictiveGuidedCandidateCount,
			*ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num(),
			WrapConfig.MinLatchNodes, *NodeListString(ContactTracker.CandidateNodes),
			bShouldCapture ? TEXT("yes") : TEXT("no"));
		GEngine->AddOnScreenDebugMessage(DebugKey, 0.05f, FColor::Cyan, Text, false);
	}
}

void RopeDebug::DrawFlightWhipGuide(const UWorld* World, const FRopeSimState& Sim, const TArray<int32>& GuideNodeIndices,
	const TArray<FVector>& GuideTargets, float GuidedEnd, bool bWhipActive)
{
	const bool bStatEnabled = IsFlightStatEnabled();
	const bool bVisualEnabled = IsFlightVisualEnabled();
	if (!World || (!bStatEnabled && !bVisualEnabled) || !bWhipActive)
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

	if (bStatEnabled)
	{
		INC_DWORD_STAT_BY(STAT_RopeFlightWhipGuidedNodes, GuidedNodeCount);
		INC_DWORD_STAT_BY(STAT_RopeFlightWhipGuidePoints, GuideTargets.Num());
		INC_DWORD_STAT_BY(STAT_RopeFlightWhipSolverOnlyNodes, SolverOnlyCount);
	}

	if (!bVisualEnabled)
	{
		return;
	}

	DrawDebugBox(World, Sim.Positions[0], FVector(4.5f), FColor::White, false, -1.0f, SDPG_Foreground, 0.75f);

	for (int32 i = 1; i <= LastNode; ++i)
	{
		const float S = static_cast<float>(i) / static_cast<float>(LastNode);
		if (S <= GuidedEnd)
		{
			DrawDebugBox(World, Sim.Positions[i], FVector(3.5f), FColor::Cyan, false, -1.0f, SDPG_Foreground, 0.5f);
		}
		else
		{
			DrawDebugPoint(World, Sim.Positions[i], 7.0f, FColor::Green, false, -1.0f, SDPG_Foreground);
		}
	}

	for (int32 i = 0; i < GuideTargets.Num(); ++i)
	{
		DrawDebugPoint(World, GuideTargets[i], 8.0f, FColor::Cyan, false, -1.0f, SDPG_Foreground);
		if (i + 1 < GuideTargets.Num())
		{
			DrawDebugLine(World, GuideTargets[i], GuideTargets[i + 1], FColor::Cyan, false, -1.0f, SDPG_Foreground, 1.0f);
		}

		if (GuideNodeIndices.IsValidIndex(i) && Sim.Positions.IsValidIndex(GuideNodeIndices[i]))
		{
			DrawDebugLine(World, Sim.Positions[GuideNodeIndices[i]], GuideTargets[i],
				FColor(255, 140, 0), false, -1.0f, SDPG_Foreground, 0.75f);
		}
	}
}

void RopeDebug::DrawWrappedTable(const UWorld* World, uint64 DebugKey, const FString& RopeName,
	const FRopeSimState& Sim, const FRopeWrapState& Wrap)
{
	const bool bStatEnabled = IsWrappedStatEnabled();
	const bool bVisualEnabled = IsWrappedVisualEnabled();
	if (!World || !Wrap.IsWrapped() || (!bStatEnabled && !bVisualEnabled))
	{
		return;
	}

	if (bStatEnabled)
	{
		INC_DWORD_STAT(STAT_RopeWrappedComponents);
		INC_DWORD_STAT_BY(STAT_RopeWrappedNumParticles, Sim.Num());
		INC_DWORD_STAT_BY(STAT_RopeWrappedLatchedNodes, Wrap.Latched.Num());
	}

	if (!bVisualEnabled)
	{
		return;
	}

	TSet<int32> LatchedNodes;
	for (const FRopeLatchNode& Latch : Wrap.Latched)
	{
		LatchedNodes.Add(Latch.NodeIndex);
	}

	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (LatchedNodes.Contains(i))
		{
			DrawDebugBox(World, Sim.Positions[i], FVector(4.5f), FColor::Yellow, false, -1.0f, SDPG_Foreground, 0.75f);
		}
		else
		{
			DrawDebugPoint(World, Sim.Positions[i], 7.0f, FColor::Green, false, -1.0f, SDPG_Foreground);
		}
	}

	if (GEngine && CVarRopeDebugScreenText.GetValueOnGameThread() != 0)
	{
		const USkeletalMeshComponent* Mesh = Wrap.Mesh.Get();
		FString Text = FString::Printf(TEXT("[RopeWrapped] %s  bone=%s  mesh=%s  latched=%d\n"),
			*RopeName, *Wrap.BoneName.ToString(), Mesh ? *Mesh->GetName() : TEXT("None"), Wrap.Latched.Num());
		Text += TEXT("Node | Bone | Local Position | World Position\n");
		Text += TEXT("-----+------+----------------+---------------");

		const int32 MaxRows = FMath::Min(12, Wrap.Latched.Num());
		for (int32 i = 0; i < MaxRows; ++i)
		{
			const FRopeLatchNode& Latch = Wrap.Latched[i];
			const FVector WorldPos = Sim.Positions.IsValidIndex(Latch.NodeIndex)
				? Sim.Positions[Latch.NodeIndex]
				: FVector::ZeroVector;
			Text += FString::Printf(TEXT("\n%4d | %s | %s | %s"),
				Latch.NodeIndex, *Latch.Bone.ToString(),
				*Latch.BoneLocalPos.ToCompactString(), *WorldPos.ToCompactString());
		}
		if (Wrap.Latched.Num() > MaxRows)
		{
			Text += FString::Printf(TEXT("\n... %d more"), Wrap.Latched.Num() - MaxRows);
		}

		GEngine->AddOnScreenDebugMessage(DebugKey, 0.05f, FColor::Yellow, Text, false);
	}
}

void RopeDebug::DrawCapsule(const UWorld* World, const FVector& A, const FVector& B, float Radius, bool bInstanceForce)
{
	if (!World || !IsEnabled(bInstanceForce) || CVarRopeDebugColliders.GetValueOnGameThread() == 0)
	{
		return;
	}
	const FVector Center = (A + B) * 0.5f;
	const float   HalfHeight = static_cast<float>((B - A).Size()) * 0.5f + Radius;
	const FQuat   Rot = FRotationMatrix::MakeFromZ(B - A).ToQuat();
	DrawDebugCapsule(World, Center, HalfHeight, Radius, Rot, FColor::Green, false, -1.0f, 0, 0.5f);
}

void RopeDebug::DrawColliderBounds(const UWorld* World, const FBox& WorldBounds, bool bInstanceForce)
{
	if (!World || !WorldBounds.IsValid || !IsEnabled(bInstanceForce) || CVarRopeDebugColliders.GetValueOnGameThread() == 0)
	{
		return;
	}
	DrawDebugBox(World, WorldBounds.GetCenter(), WorldBounds.GetExtent(), FColor::Green, false, -1.0f, 0, 0.5f);
}

#else // UE_BUILD_SHIPPING — 모두 no-op

bool RopeDebug::IsEnabled(bool) { return false; }
bool RopeDebug::IsFlightVisualEnabled() { return false; }
bool RopeDebug::IsWrappedVisualEnabled() { return false; }
bool RopeDebug::IsFlightStatEnabled() { return false; }
bool RopeDebug::IsWrappedStatEnabled() { return false; }
void RopeDebug::DrawCenterline(const UWorld*, const FRopeSimState&, ERopePhase, const FRopeWrapState&, bool) {}
void RopeDebug::DrawFlight(const UWorld*, uint64, const FString&, const FRopeSimState&, ERopePhase, bool, int32,
	const TArray<FRopeFlightNodeDebug>&, const TArray<FRopeContactCandidate>&, const FRopeContactTracker&,
	const FRopeWrapConfig&, bool) {
}
void RopeDebug::DrawFlightWhipGuide(const UWorld*, const FRopeSimState&, const TArray<int32>&,
	const TArray<FVector>&, float, bool) {
}
void RopeDebug::DrawWrappedTable(const UWorld*, uint64, const FString&, const FRopeSimState&, const FRopeWrapState&) {}
void RopeDebug::DrawCapsule(const UWorld*, const FVector&, const FVector&, float, bool) {}
void RopeDebug::DrawColliderBounds(const UWorld*, const FBox&, bool) {}

#endif
