// Fill out your copyright notice in the Description page of Project Settings.

#include "RopeWanderAIController.h"
#include "NavigationSystem.h"
#include "Engine/World.h"
#include "TimerManager.h"

ARopeWanderAIController::ARopeWanderAIController()
{
	// 데모용 최소 설정.
}

void ARopeWanderAIController::OnPossess(APawn* InPawn)
{
	Super::OnPossess(InPawn);
	PickNewDestination();
}

void ARopeWanderAIController::SetMovementPaused(bool bInPaused)
{
	bPaused = bInPaused;
	if (bPaused)
	{
		// 소프트 제한: 진행 중인 이동을 멈추고 다음 목적지 선택을 막는다.
		GetWorldTimerManager().ClearTimer(RepickTimer);
		StopMovement();
	}
	else
	{
		PickNewDestination();
	}
}

void ARopeWanderAIController::OnMoveCompleted(FAIRequestID RequestID, const FPathFollowingResult& Result)
{
	Super::OnMoveCompleted(RequestID, Result);
	if (bPaused)
	{
		return;
	}
	// 즉시 재선택하면 도착/실패가 반복될 때 타이트 루프가 되므로 짧게 지연.
	GetWorldTimerManager().SetTimer(RepickTimer, this, &ARopeWanderAIController::PickNewDestination, RepickDelay, false);
}

void ARopeWanderAIController::PickNewDestination()
{
	if (bPaused)
	{
		return;
	}
	APawn* P = GetPawn();
	UNavigationSystemV1* Nav = UNavigationSystemV1::GetCurrent(GetWorld());
	if (!P || !Nav)
	{
		return; // NavMesh가 없으면 제자리. (레벨에 NavMeshBoundsVolume 필요)
	}

	FNavLocation Dest;
	if (Nav->GetRandomReachablePointInRadius(P->GetActorLocation(), WanderRadius, Dest))
	{
		MoveToLocation(Dest.Location);
	}
}
