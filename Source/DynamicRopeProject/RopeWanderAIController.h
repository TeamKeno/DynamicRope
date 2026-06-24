// Fill out your copyright notice in the Description page of Project Settings.
//
// NavMesh 위를 랜덤하게 배회하는 최소 AI 컨트롤러 (Tier 1 동적 타깃용, BehaviorTree 없이 C++만).
// 로프에 테더되면 이동을 멈춘다.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "RopeWanderAIController.generated.h"

UCLASS()
class DYNAMICROPEPROJECT_API ARopeWanderAIController : public AAIController
{
	GENERATED_BODY()

public:
	ARopeWanderAIController();

	/** 이동 일시정지/재개 — 로프 wrap/release 시 타깃이 호출(행동 제한). */
	void SetMovementPaused(bool bInPaused);

	/** 배회 목적지를 고를 반경(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	float WanderRadius = 1500.0f;

	/** 목적지 도착 후 다음 목적지를 고르기까지의 대기(초). 타이트 루프 방지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	float RepickDelay = 0.5f;

protected:
	virtual void OnPossess(APawn* InPawn) override;
	virtual void OnMoveCompleted(FAIRequestID RequestID, const FPathFollowingResult& Result) override;

private:
	bool bPaused = false;
	FTimerHandle RepickTimer;

	/** NavMesh에서 도달 가능한 랜덤 지점으로 이동 시작. */
	void PickNewDestination();
};
