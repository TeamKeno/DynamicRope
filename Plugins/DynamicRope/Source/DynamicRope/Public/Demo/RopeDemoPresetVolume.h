// Copyright Epic Games, Inc. All Rights Reserved.
//
// 데모 씬용 프리셋 전환 볼륨. 들어온 액터가 들고 있는 로프에 URopePreset을 적용한다 — 방마다 로프의
// 성격을 바꿔(자유 시뮬 / 포획용 / 그래플링 훅) "튜닝 하나로 완전히 다른 로프가 된다"를 보여주는 게
// 목적이다. 나가면서 ExitPreset을 지정해 두면 되돌리는 것도 된다.
//
// ApplyPreset은 **Free/Reel에서만** 성립한다(날아가거나 감고 있는 중에 값을 갈아끼우면 시뮬이
// 튄다). 그래서 볼륨에 들어온 순간 로프가 비행/감김 중이면 적용이 실패하는데, 그때 조용히 넘어가면
// "문을 통과했는데 로프가 안 바뀌는" 상황이 된다. bApplyWhenRopeSettles를 켜면(기본) 실패한 로프를
// 대기열에 넣고 OnRopePhaseChanged를 구독해, **볼륨 안에 있는 동안** 로프가 Free/Reel로 돌아오는
// 첫 순간에 적용한다.
//
// 되돌리기가 자동이 아닌 이유: URopeComponent는 프리셋을 "스탬프"(값 복사)로 적용하고 프리셋 포인터를
// 보관하지 않는다 — 들어오기 전 상태를 알 방법이 없다. 그래서 복원은 ExitPreset 명시로만 한다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"
#include "GameFramework/Actor.h"
#include "RopeDemoPresetVolume.generated.h"

class UBoxComponent;
class URopeComponent;
class URopePreset;

/** 프리셋이 실제로 적용된 순간(대기 후 적용도 포함). */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FRopeDemoPresetAppliedSignature,
	ARopeDemoPresetVolume*, Volume, URopeComponent*, Rope, const URopePreset*, Preset);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Preset Volume"))
class DYNAMICROPE_API ARopeDemoPresetVolume : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoPresetVolume();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** 프리셋이 적용될 때마다. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoPresetAppliedSignature OnPresetVolumeApplied;

	/** 들어올 때 적용할 프리셋. 비면 아무것도 하지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<URopePreset> Preset = nullptr;

	/** 나갈 때 적용할 프리셋(선택). 비면 나가도 그대로 둔다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<URopePreset> ExitPreset = nullptr;

	/** 폰이 들고 있는 로프만 대상으로 할지. 끄면 볼륨에 들어온 아무 액터의 로프에나 적용한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bPawnsOnly = true;

	/**
	 * 들어온 시점에 로프가 비행/감김 중이라 적용이 거부되면, 볼륨 안에 있는 동안 기다렸다가
	 * Free/Reel로 돌아오는 순간 적용한다. 끄면 그 순간 실패로 끝난다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bApplyWhenRopeSettles = true;

protected:
	/** 전환 영역. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UBoxComponent> Trigger = nullptr;

private:
	UFUNCTION()
	void HandleBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	UFUNCTION()
	void HandleEndOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);

	/** 대기 중인 로프가 Free/Reel로 돌아왔는지 확인해 적용을 재시도한다. */
	UFUNCTION()
	void HandleRopePhaseChanged(ERopePhase OldPhase, ERopePhase NewPhase);

	/** 액터가 가진 로프에 프리셋 적용을 시도. 실패하고 bWaitIfBusy면 대기열에 넣는다. */
	void ApplyToActor(AActor* Actor, const URopePreset* InPreset, bool bWaitIfBusy);

	/** 로프 하나에 적용. 성공하면 true. */
	bool ApplyToRope(URopeComponent* Rope, const URopePreset* InPreset);

	/** 대기열에서 빼고 구독도 해제한다. */
	void StopWaitingFor(URopeComponent* Rope);

	/** 액터 단위 오버랩 카운트(랙돌 등 다중 바디 대응 — RopeDemoPressurePlate와 같은 이유). */
	TMap<TWeakObjectPtr<AActor>, int32> OverlapCounts;

	/** Free/Reel 복귀를 기다리는 로프. */
	TSet<TWeakObjectPtr<URopeComponent>> PendingRopes;
};
