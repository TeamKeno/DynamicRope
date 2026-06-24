// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — 실험용이며 출시 대상 아님. PoC/ 아래의 모든 것은 폐기 가능한 코드다.
// 캐릭터의 손에 rope를 붙이고, 좌클릭 시 montage로 whip을 휘두른다.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RopePoCWhipComponent.generated.h"

class ARopePoCActor;
class UAnimMontage;
class USkeletalMeshComponent;

/**
 * "whip" 느낌을 테스트하려면 캐릭터에 붙인다: rope의 pinned end가 hand bone에 붙어,
 * swing montage가 재생될 때 free end가 whip처럼 따라 휘날린다.
 */
UCLASS(ClassGroup = (DynamicRopePoC), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopePoCWhipComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopePoCWhipComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** start가 손을 따라가는 rope. null이고 bSpawnRopeIfMissing이면 손 위치에 하나 spawn한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Whip")
	TObjectPtr<ARopePoCActor> Rope = nullptr;

	/** Rope가 지정되지 않았으면 기본 rope를 spawn한다. */
	UPROPERTY(EditAnywhere, Category = "Whip")
	bool bSpawnRopeIfMissing = true;

	/** rope의 pinned end가 따라가는 hand bone/socket. */
	UPROPERTY(EditAnywhere, Category = "Whip")
	FName HandSocketName = TEXT("hand_r");

	/** Swing()이 재생하는 montage. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Whip")
	TObjectPtr<UAnimMontage> SwingMontage = nullptr;

	/** swing montage의 재생 속도. 1 = 보통, 2 = 두 배 빠름, 0.5 = 절반 속도. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Whip", meta = (ClampMin = "0.01", UIMin = "0.1", UIMax = "3.0"))
	float MontagePlayRate = 1.0f;

	/** Left Mouse Button → Swing()을 자동으로 bind 시도한다(input asset 불필요). */
	UPROPERTY(EditAnywhere, Category = "Whip")
	bool bAutoBindLeftMouse = true;

	/** rope를 손에 attach하고 start를 그곳에 pin한다. */
	UFUNCTION(BlueprintCallable, Category = "Whip")
	void AttachRopeToHand();

	/** swing montage를 재생한다. auto-bind가 꺼져 있으면 직접 좌클릭 입력에 bind하라. */
	UFUNCTION(BlueprintCallable, Category = "Whip")
	void Swing();

private:
	bool bLeftMouseBound = false;

	USkeletalMeshComponent* GetOwnerMesh() const;
	bool TryBindLeftMouse();
};
