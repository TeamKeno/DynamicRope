// Fill out your copyright notice in the Description page of Project Settings.

#include "RopeTetherTarget.h"
#include "RopeWanderAIController.h"
#include "Collision/RopeBoneCapsuleProvider.h"
#include "GameFramework/CharacterMovementComponent.h"

ARopeTetherTarget::ARopeTetherTarget()
{
	// 스폰/배치 시 자동으로 배회 AI가 빙의하도록.
	AIControllerClass = ARopeWanderAIController::StaticClass();
	AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;

	// 본 캡슐 provider. SkeletalMesh는 비워두면 owner(이 캐릭터)의 메시로 자동 해석된다.
	RopeColliders = CreateDefaultSubobject<URopeBoneCapsuleProvider>(TEXT("RopeColliders"));
	RopeColliders->bDrawDebug = true; // 데모: 생성된 캡슐을 화면에 표시.

	// UE5 Manny/Quinn 스켈레톤 기준 기본 본. 다른 스켈레톤이면 에디터에서 교체.
	RopeColliders->Bones = {
		TEXT("spine_03"), TEXT("pelvis"),
		TEXT("thigh_l"),  TEXT("thigh_r"),
		TEXT("calf_l"),   TEXT("calf_r"),
		TEXT("upperarm_l"), TEXT("upperarm_r")
	};
}

void ARopeTetherTarget::SetTethered(bool bInTethered)
{
	if (bTethered == bInTethered)
	{
		return;
	}
	bTethered = bInTethered;

	// 하드 제한: 그 자리에 묶이도록 이동 속도를 0으로(해제 시 복원).
	if (UCharacterMovementComponent* Move = GetCharacterMovement())
	{
		if (bTethered)
		{
			CachedMaxWalkSpeed = Move->MaxWalkSpeed;
			Move->StopMovementImmediately();
			Move->MaxWalkSpeed = 0.0f;
		}
		else if (CachedMaxWalkSpeed > 0.0f)
		{
			Move->MaxWalkSpeed = CachedMaxWalkSpeed;
		}
	}

	// 소프트 제한: 배회 AI의 이동 선택을 멈추거나 재개.
	if (ARopeWanderAIController* AI = Cast<ARopeWanderAIController>(GetController()))
	{
		AI->SetMovementPaused(bTethered);
	}
}
