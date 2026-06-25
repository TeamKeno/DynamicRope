// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/RopeDebugDraw.h"
#include "Engine/Engine.h"
#include "DrawDebugHelpers.h"

#if !UE_BUILD_SHIPPING

namespace
{
	TAutoConsoleVariable<int32> CVarRopeDebug(
		TEXT("r.DynamicRope.Debug"), 0,
		TEXT("DynamicRope 디버그 마스터 토글. per-instance bDrawDebug과 OR된다. 0=off, 1=on."),
		ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugCenterline(
		TEXT("r.DynamicRope.Debug.Centerline"), 1,
		TEXT("중심선/노드/latch 노드 표시."), ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugBounds(
		TEXT("r.DynamicRope.Debug.Bounds"), 0,
		TEXT("브로드페이즈 bounds 박스 표시."), ECVF_Cheat);

	TAutoConsoleVariable<int32> CVarRopeDebugStats(
		TEXT("r.DynamicRope.Debug.Stats"), 1,
		TEXT("온스크린 phase/collider/wrap 통계 표시."), ECVF_Cheat);

	FColor PhaseColor(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Flight:     return FColor::Cyan;
		case ERopePhase::Contacting: return FColor::Yellow;
		case ERopePhase::Wrapped:    return FColor::Green;
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

void RopeDebug::DrawBounds(const UWorld* World, const FBox& Bounds, bool bInstanceForce)
{
	if (!World || !Bounds.IsValid || !IsEnabled(bInstanceForce) || CVarRopeDebugBounds.GetValueOnGameThread() == 0)
	{
		return;
	}
	DrawDebugBox(World, Bounds.GetCenter(), Bounds.GetExtent(), FColor::Orange, false, -1.0f, 0, 0.5f);
}

void RopeDebug::DrawStats(const UWorld* World, uint64 Key, ERopePhase Phase,
	int32 ProviderCount, int32 ColliderCount, FName WrapBone, bool bInstanceForce)
{
	if (!World || !GEngine || !IsEnabled(bInstanceForce) || CVarRopeDebugStats.GetValueOnGameThread() == 0)
	{
		return;
	}
	GEngine->AddOnScreenDebugMessage(static_cast<uint64>(Key), 0.0f, PhaseColor(Phase),
		FString::Printf(TEXT("[Rope] phase=%d providers=%d colliders=%d wrapBone=%s"),
			static_cast<int32>(Phase), ProviderCount, ColliderCount, *WrapBone.ToString()));
}

#else // UE_BUILD_SHIPPING — 모두 no-op

bool RopeDebug::IsEnabled(bool) { return false; }
void RopeDebug::DrawCenterline(const UWorld*, const FRopeSimState&, ERopePhase, const FRopeWrapState&, bool) {}
void RopeDebug::DrawBounds(const UWorld*, const FBox&, bool) {}
void RopeDebug::DrawStats(const UWorld*, uint64, ERopePhase, int32, int32, FName, bool) {}

#endif
