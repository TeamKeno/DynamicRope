// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그 시각화의 단일 진입점. 흩어져 있던 draw를 cvar(r.DynamicRope.Debug.*)로 토글되는
// 한 곳으로 모은다. POD 상태를 const-ref로 받아 UObject 결합이 없다(솔버/로직 철학과 동일).
// 각 함수는 per-instance 강제 enable(bInstanceForce) 또는 해당 cvar이 켜졌을 때만 그린다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class UWorld;

namespace RopeDebug
{
	struct FRopeFlightNodeDebug
	{
		int32 NodeIndex = INDEX_NONE;
		FVector PrevPosition = FVector::ZeroVector;
		FVector Position = FVector::ZeroVector;
		float NodeSpeed = 0.0f;
		bool bFast = false;
		bool bNearBody = false;
		FRopeContact Contact;
	};

	/** 마스터 토글(per-instance 강제 또는 r.DynamicRope.Debug). */
	bool IsEnabled(bool bInstanceForce);
	bool IsFlightStatEnabled();
	bool IsWrappedStatEnabled();

	/** 중심선 + 노드 + latch 노드 강조. 색은 phase를 따른다. */
	void DrawCenterline(const UWorld* World, const FRopeSimState& Sim, ERopePhase Phase,
		const FRopeWrapState& Wrap, bool bInstanceForce);

	void DrawFlight(const UWorld* World, uint64 DebugKey, const FString& RopeName, const FRopeSimState& Sim,
		ERopePhase Phase, bool bSolveThisFrame, int32 FrameColliderCount,
		const TArray<FRopeFlightNodeDebug>& NodeDebug, const TArray<FRopeContactCandidate>& Candidates,
		const FRopeContactTracker& ContactTracker, const FRopeWrapConfig& WrapConfig, bool bShouldCapture);

	void DrawFlightWhipGuide(const UWorld* World, const FRopeSimState& Sim, const TArray<int32>& GuideNodeIndices,
		const TArray<FVector>& GuideTargets, float GuidedEnd, bool bWhipActive);

	void DrawWrappedTable(const UWorld* World, uint64 DebugKey, const FString& RopeName,
		const FRopeSimState& Sim, const FRopeWrapState& Wrap);

	/** provider collider 시각화(중앙화): 본 capsule(A-B 세그먼트 + 반지름). r.DynamicRope.Debug.Colliders로 게이트. */
	void DrawCapsule(const UWorld* World, const FVector& A, const FVector& B, float Radius, bool bInstanceForce);

	/** provider collider 시각화: collider 월드 bounds 박스(예: SDF 볼륨). */
	void DrawColliderBounds(const UWorld* World, const FBox& WorldBounds, bool bInstanceForce);
}
