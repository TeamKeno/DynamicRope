// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "RopeComponent.h"
#include "Core/RopeWrapTarget.h"

namespace RopeComponentPrivate
{
	void LogWrappingFailureState(const FString& OwnerName, const TCHAR* FailureSite,
		const FRopeWrappingState& State, const FRopeSimState& Sim);

	// The cooldown, in seconds, from entering Releasing until the return to Free. Shared by aborts, a failed hold and a manual release.
	inline constexpr float ReleaseCooldownSeconds = 0.08f;

	// A short name for the phase transition log, safe on the hot path since it needs no UEnum reflection.
	inline const TCHAR* PhaseName(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Free:        return TEXT("Free");
		case ERopePhase::Flight:      return TEXT("Flight");
		case ERopePhase::Contacting:  return TEXT("Contacting");
		case ERopePhase::Wrapping:    return TEXT("Wrapping");
		case ERopePhase::Wrapped:     return TEXT("Wrapped");
		case ERopePhase::GuidedThrow: return TEXT("GuidedThrow");
		case ERopePhase::Releasing:   return TEXT("Releasing");
		case ERopePhase::Loaded:        return TEXT("Loaded");
		default:                      return TEXT("?");
		}
	}

	// Restores the aim hit to world space through the target bone's current transform. Where a bone-local position was
	// stored it follows the target's movement and animation, and otherwise the world value from the moment of aiming
	// is used as a fallback. This is symmetric with restoring a bone-local anchor.
	inline FVector ResolveAimGuideHitWorld(const FRopeThrowContext& Ctx)
	{
		if (Ctx.bHasAimGuideLocalHit && !Ctx.AimGuideBone.IsNone())
		{
			if (const USceneComponent* Mesh = Ctx.AimGuideMesh.Get())
			{
				return ResolveBindingWorld(Mesh, Ctx.AimGuideBone).TransformPosition(Ctx.AimGuideLocalHitPos);
			}
		}
		return Ctx.AimGuideHitWorldPos;
	}
}
