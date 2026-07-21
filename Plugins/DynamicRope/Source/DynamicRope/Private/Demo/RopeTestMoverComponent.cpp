// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeTestMoverComponent.h"
#include "GameFramework/Actor.h"

URopeTestMoverComponent::URopeTestMoverComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	// 기본 틱 그룹(PrePhysics)에서 먼저 액터를 옮긴 뒤, TG_PostPhysics의 로프 시뮬이 갱신된 위치를 읽는다.
}

void URopeTestMoverComponent::BeginPlay()
{
	Super::BeginPlay();
	if (const AActor* Owner = GetOwner())
	{
		StartLocation = Owner->GetActorLocation();
		StartRotation = Owner->GetActorRotation();
	}
}

void URopeTestMoverComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	ElapsedTime += DeltaTime;

	// sin 왕복(진폭 MoveOffset). 로컬 공간이면 시작 회전 기준으로 오프셋을 회전시킨다.
	const float   Alpha = FMath::Sin(2.0f * PI * ElapsedTime / FMath::Max(Period, 0.05f));
	const FVector Offset = bMoveInLocalSpace ? StartRotation.RotateVector(MoveOffset * Alpha) : (MoveOffset * Alpha);
	const FRotator Rot = StartRotation + RotationRate * ElapsedTime;

	// 텔레포트 이동(sweep 없음) — 테스트 관찰용. Movable 메시여야 실제로 움직인다.
	Owner->SetActorLocationAndRotation(StartLocation + Offset, Rot);
}
