// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class IRopeCollider;
class USceneComponent;

/**
 * Stateless throw-preview path builder. It owns the expensive preview search/path-build
 * policy so URopeComponent can stay focused on authoritative sim state and phase changes.
 */
class DYNAMICROPE_API FRopeThrowPreviewBuilder
{
public:
	struct FInput
	{
		const FRopeSimState* Sim = nullptr;
		const TArray<IRopeCollider*>* Colliders = nullptr;

		/** wrap 대상 게이트(URopeComponent::CanWrapTarget 주입 — 빌더는 UObject-free라 virtual을 직접 못 부른다).
		 *  arc 탐색이 aim 경로(FindAimRayBoneHit)와 **같은 기준**으로 후보를 거르게 하는 유일한 통로다:
		 *  이게 없으면 aim이 거부한 대상을 preview가 주워 둘의 판정이 갈린다. 미설정이면 전부 허용
		 *  (CanWrapTarget의 기본 구현과 같은 의미) — 월드 없는 단위 테스트는 설정하지 않아도 된다. */
		TFunction<bool(const USceneComponent*, FName)> CanWrapTarget;

		FRopeThrowContext ThrowContext;
		FRopeWrapConfig WrapConfig;
		/** 도달 모드 스냅샷(컴포넌트 ResolveMode). GuaranteedWrap Cinch의 감김 경로 빌드가 감김 축/경로 선택에 쓴다. */
		ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

		/** 결착 모델. Pierce면 감김 나선 경로 대신 aim-hit 접점에 단일 앵커로 꽂는다(GuaranteedWrap 전용). */
		ERopeTipEngagement TipEngagement = ERopeTipEngagement::BareWrap;

		float RopeRadius = 0.0f;
		int32 RopeNumSides = 8;
		float RopeLength = 0.0f;
		float SweepAngleDegrees = 180.0f;
		FVector FallbackForward = FVector::ForwardVector;
		FString OwnerName;

		float ReachScale = 1.0f;
		int32 SegmentCount = 32;
		float SampleStep = 80.0f;
		float QueryRadius = 0.0f;
	};

	// GuaranteedWrap 모드 전용: Reel에서 조준한 대상의 확정 throw path(contact/anchor 포함)를 만든다.
	static bool BuildFreePreparedPreview(const FInput& Input, FRopePreparedThrowPreview& OutPrepared,
		FString* OutFailureReason = nullptr);
};
