// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class IRopeCollider;

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

		FRopeThrowContext ThrowContext;
		FRopeWrapConfig WrapConfig;
		ERopeWrappingPathMode PathMode = ERopeWrappingPathMode::SurfaceVectorField;

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
