// Copyright Epic Games, Inc. All Rights Reserved.
//
// The entry points for the rope debug stat counters. Visualization, covering the centreline, flight, wrapped state,
// colliders, labels and screen text, is consolidated into FGameplayDebuggerCategory_Rope, which is the sole debug
// entry point. What remains here is the 'stat RopeFlight' and 'stat RopeWrapped' profiling path alone: INC_DWORD_STAT
// has to be in the same translation unit as the declaration, so the counter writes live in this module's .cpp. They
// are a no-op when the stat system is not collecting.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeContactTrackingTypes.h"
#include "Core/RopeSimTypes.h"
#include "Core/RopeWrappingTypes.h"

namespace RopeDebug
{
	/** Whether the 'stat RopeFlight' or 'stat RopeWrapped' group is currently collecting. */
	bool IsFlightStatEnabled();
	bool IsWrappedStatEnabled();

	/** Records the flight frame counters, only while stats are collecting. Independent of the debug visual capture. */
	void RecordFlightStats(const FRopeSimState& Sim, bool bSolveThisFrame, int32 FrameColliderCount,
		const TArray<FRopeContactCandidate>& Candidates, const FRopeContactTracker& ContactTracker,
		bool bShouldCapture);

	/** Records the whip guide counters, only while the whip is active and stats are collecting. */
	void RecordWhipStats(const FRopeSimState& Sim, int32 GuidedNodeCount, float GuidedEnd);

	/** Records the wrapped counters, only while wrapped and stats are collecting. */
	void RecordWrappedStats(const FRopeSimState& Sim, const FRopeWrapState& Wrap);
}
