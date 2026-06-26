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

	TAutoConsoleVariable<int32> CVarRopeDebugColliders(
		TEXT("r.DynamicRope.Debug.Colliders"), 1,
		TEXT("provider collider(capsule / SDF 볼륨 bounds) 표시. provider별 bDrawDebug과 OR된다."), ECVF_Cheat);

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
void RopeDebug::DrawCenterline(const UWorld*, const FRopeSimState&, ERopePhase, const FRopeWrapState&, bool) {}
void RopeDebug::DrawCapsule(const UWorld*, const FVector&, const FVector&, float, bool) {}
void RopeDebug::DrawColliderBounds(const UWorld*, const FBox&, bool) {}

#endif
