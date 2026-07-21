// Copyright Epic Games, Inc. All Rights Reserved.
//
// 테스트/디버그 유틸: 소유 액터를 sin 왕복 이동(+ 선택적 회전)시켜, "움직이는 바디"에 로프가 어떻게
// 반응하는지 눈으로 보게 한다. 정적 바디 프로바이더는 현재 WorldStatic만 매 프레임 정적 스냅샷으로
// 잡으므로(표면 속도 0, 상대 CCD 없음), 이 컴포넌트로 Movable WorldStatic 메시를 움직여 그 거동/한계를
// 관찰한다: 느리면 로프가 얹혀 따라가고, 빠르면 표면 속도 미반영으로 드래그가 없고 터널링이 보인다.
//
// 사용: Movable 스태틱 메시 액터(콜리전 오브젝트 타입 WorldStatic, 심플 콜리전 있음)에 이 컴포넌트를
// 붙이면 된다. WorldDynamic으로 두면 프로바이더가 아직 안 잡으니(동적 바디 미지원) WorldStatic 유지.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RopeTestMoverComponent.generated.h"

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeTestMoverComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeTestMoverComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** 왕복 이동 진폭. 소유 액터가 시작 위치 ± 이 오프셋 사이를 sin 왕복한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Test")
	FVector MoveOffset = FVector(200.0f, 0.0f, 0.0f);

	/** 한 왕복 주기(초). 작을수록 빠르다(빠르게 하면 표면 속도 미반영으로 터널링/드래그 부재가 보인다). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Test", meta = (ClampMin = "0.05", Units = "s"))
	float Period = 3.0f;

	/** 초당 회전(도). 0이면 회전 없음. 회전하는 표면 위 로프 거동 관찰용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Test")
	FRotator RotationRate = FRotator::ZeroRotator;

	/** 이동을 시작 회전 기준 로컬 축으로 적용할지. false면 월드 축. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Test")
	bool bMoveInLocalSpace = true;

private:
	FVector  StartLocation = FVector::ZeroVector;
	FRotator StartRotation = FRotator::ZeroRotator;
	float    ElapsedTime = 0.0f;
};
