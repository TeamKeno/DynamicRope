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

		/** The wrap target gate, injected from URopeComponent::CanWrapTarget because the builder is
		 *  free of UObject dependencies and cannot call the virtual itself.
		 *  It is the only way to make the arc search filter candidates on the same basis as the aiming
		 *  path in FindAimRayBoneHit: without it the preview would pick up a target aiming had refused,
		 *  and the two would disagree. Leaving it unset permits everything, which matches the default
		 *  implementation of CanWrapTarget, so unit tests without a world need not set it. */
		TFunction<bool(const USceneComponent*, FName)> CanWrapTarget;

		FRopeThrowContext ThrowContext;
		FRopeWrapConfig WrapConfig;
		/** A snapshot of the component's resolve mode. Under GuaranteedWrap the rope embeds with a
		 *  single anchor at the aim hit point instead of building a wrapping helix, and the choice of
		 *  wrap axis and path reads this value too. */
		ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

		float RopeRadius = 0.0f;
		int32 RopeNumSides = 8;
		FString OwnerName;
	};

	// GuaranteedWrap only: builds the committed throw path, including the contact and anchor, for the
	// target aimed at while Loaded.
	static bool BuildFreePreparedPreview(const FInput& Input, FRopePreparedThrowPreview& OutPrepared,
		FString* OutFailureReason = nullptr);
};
