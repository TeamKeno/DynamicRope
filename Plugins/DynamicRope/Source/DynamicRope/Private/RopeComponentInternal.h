// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "RopeComponent.h"
#include "Core/RopeWrapTarget.h"

namespace RopeComponentPrivate
{
#if !UE_BUILD_SHIPPING
	void DrawWrapIslandDebug(const UWorld* World, const FRopeWrappingState& State,
		const FRopeSimState& Sim, const FRopeWrapConfig& Config);
#endif

	void LogWrappingFailureState(const FString& OwnerName, const TCHAR* FailureSite,
		const FRopeWrappingState& State, const FRopeSimState& Sim);

	// Releasing 진입 시 Free 복귀까지의 쿨다운(초). Abort/Hold 실패/수동 해제 공통.
	inline constexpr float ReleaseCooldownSeconds = 0.08f;

	// phase 전이 로그용 짧은 이름(UEnum 리플렉션 없이 hot-path에서도 안전).
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
		case ERopePhase::Reel:        return TEXT("Reel");
		default:                      return TEXT("?");
		}
	}

	// 조준 hit을 현재 대상 본 트랜스폼 기준 월드로 복원한다. 본-로컬을 저장한 경우 대상의 이동/애니메이션을
	// 추종하고, 아니면 조준 순간의 월드 값(폴백)을 그대로 쓴다. bone-local 앵커 복원과 대칭이다.
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
