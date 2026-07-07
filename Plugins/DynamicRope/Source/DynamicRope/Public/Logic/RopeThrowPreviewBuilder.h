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

	static bool BuildFreeWrappingPreview(const FInput& Input, FRopeWrapPreviewData& OutPreview,
		FString* OutFailureReason = nullptr);

	static bool BuildFreePreparedPreview(const FInput& Input, FRopePreparedThrowPreview& OutPrepared,
		FString* OutFailureReason = nullptr);

	static bool BuildFlightWrappingPreview(const FInput& Input, FRopeWrapPreviewData& OutPreview,
		FString* OutFailureReason = nullptr);

	static bool BuildWrappingPreviewFromCandidate(const FInput& Input, const FRopeContactCandidate& Candidate,
		const FRopeSimState& SourceSim, FRopeWrapPreviewData& OutPreview,
		FString* OutFailureReason = nullptr);
};
