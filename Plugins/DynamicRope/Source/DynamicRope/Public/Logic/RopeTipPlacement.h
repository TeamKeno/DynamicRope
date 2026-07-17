// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * 팁 메쉬 소켓 배치에 쓰는 순수 수학 모음. UObject/월드 상태를 참조하지 않아
 * Pierce 임베드와 비행 중 소켓 추종 계약을 단위 테스트할 수 있다.
 */
class DYNAMICROPE_API FRopeTipPlacement
{
public:
	/** Head 소켓을 HitPoint에 놓고 Tail->Head 축을 PierceDir에 맞춘다. */
	static void SolvePierceEmbed(const FVector& HitPoint, const FVector& PierceDir,
		const FTransform& HeadSocketLocal, bool bHasTailSocket, const FTransform& TailSocketLocal,
		FTransform& OutComponentWorld, FVector& OutTailWorld);

	/** Tail 소켓을 RopeAttachWorld에 놓고, 가능한 경우 Tail->Head 축을 ForwardDir에 맞춘다. */
	static void SolveSocketFollow(const FVector& RopeAttachWorld, const FVector& ForwardDir,
		const FTransform& TailSocketLocal, bool bHasHeadSocket, const FTransform& HeadSocketLocal,
		FTransform& OutComponentWorld);

	/** SourceDir의 상하 기울기는 유지하고 수평 yaw만 AimDir에 고정한다. */
	static FVector MakeAimYawLockedDirection(const FVector& SourceDir, const FVector& AimDir,
		const FVector& UpHint);
};
