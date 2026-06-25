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
	/** 마스터 토글(per-instance 강제 또는 r.DynamicRope.Debug). */
	bool IsEnabled(bool bInstanceForce);

	/** 중심선 + 노드 + latch 노드 강조. 색은 phase를 따른다. */
	void DrawCenterline(const UWorld* World, const FRopeSimState& Sim, ERopePhase Phase,
		const FRopeWrapState& Wrap, bool bInstanceForce);

	/** 브로드페이즈 bounds 박스. */
	void DrawBounds(const UWorld* World, const FBox& Bounds, bool bInstanceForce);

	/** 온스크린 통계 텍스트(phase / provider·collider 수 / wrap bone). Key는 메시지 슬롯 식별자. */
	void DrawStats(const UWorld* World, uint64 Key, ERopePhase Phase,
		int32 ProviderCount, int32 ColliderCount, FName WrapBone, bool bInstanceForce);
}
