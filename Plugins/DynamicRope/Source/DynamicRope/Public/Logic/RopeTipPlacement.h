// Copyright 2026 TeamKeno. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Pure maths for placing the tip mesh by its sockets. It references no UObject or world state, so
 * the pierce embed and the in-flight socket follow contracts can both be unit tested.
 */
class DYNAMICROPE_API FRopeTipPlacement
{
public:
	/** Places the head socket at HitPoint and aligns the tail-to-head axis with PierceDir. */
	static void SolvePierceEmbed(const FVector& HitPoint, const FVector& PierceDir,
		const FTransform& HeadSocketLocal, bool bHasTailSocket, const FTransform& TailSocketLocal,
		FTransform& OutComponentWorld, FVector& OutTailWorld);

	/** Places the tail socket at RopeAttachWorld and, where possible, aligns the tail-to-head axis with
	 *  ForwardDir. */
	static void SolveSocketFollow(const FVector& RopeAttachWorld, const FVector& ForwardDir,
		const FTransform& TailSocketLocal, bool bHasHeadSocket, const FTransform& HeadSocketLocal,
		FTransform& OutComponentWorld);

	/** Keeps the vertical inclination of SourceDir and locks only its horizontal yaw to AimDir. */
	static FVector MakeAimYawLockedDirection(const FVector& SourceDir, const FVector& AimDir,
		const FVector& UpHint);
};
