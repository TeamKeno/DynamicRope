// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그 stat 카운터 진입점. 시각화(센터라인/flight/wrapped/collider/라벨/스크린텍스트)는
// FGameplayDebuggerCategory_Rope로 일원화됐다 — 디버그 진입점은 그 카테고리 하나뿐이다. 여기 남은
// 것은 'stat RopeFlight' / 'stat RopeWrapped' 프로파일링 경로뿐: INC_DWORD_STAT은 DECLARE와 같은
// 번역 단위에 있어야 하므로 카운터 기록을 이 모듈(.cpp)에 둔다. stat 시스템이 수집 중이 아니면 no-op.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

namespace RopeDebug
{
	/** 'stat RopeFlight' / 'stat RopeWrapped' 그룹이 현재 수집 중인지. */
	bool IsFlightStatEnabled();
	bool IsWrappedStatEnabled();

	/** flight 프레임 카운터 기록(stat 수집 중일 때만). 디버그 비주얼 캡처와 독립. */
	void RecordFlightStats(const FRopeSimState& Sim, bool bSolveThisFrame, int32 FrameColliderCount,
		const TArray<FRopeContactCandidate>& Candidates, const FRopeContactTracker& ContactTracker,
		const FRopeWrapConfig& WrapConfig, bool bShouldCapture);

	/** whip 가이드 카운터 기록(whip 활성 + stat 수집 중일 때만). */
	void RecordWhipStats(const FRopeSimState& Sim, const TArray<int32>& GuideNodeIndices,
		const TArray<FVector>& GuideTargets, float GuidedEnd, bool bWhipActive);

	/** wrapped 카운터 기록(wrapped 상태 + stat 수집 중일 때만). */
	void RecordWrappedStats(const FRopeSimState& Sim, const FRopeWrapState& Wrap);
}
