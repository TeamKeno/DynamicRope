// Copyright Epic Games, Inc. All Rights Reserved.
//
// 데모 씬용 압력판(버튼). 위에 "무언가가 올라와 있으면" 판이 눌리고, 비면 되돌아온다 —
// 로프로 물건을 옮겨 판을 채우는 퍼즐의 최소 단위다.
//
// 기본 판정은 **물리 시뮬 중인 바디만 인정**한다(bRequireSimulatingPhysics). 즉 플레이어가 제 발로
// 올라서는 것으로는 안 눌리고, 물리 스태틱 메시나 랙돌 상태의 캐릭터를 얹어야 한다 — "로프로 옮겨라"가
// 퍼즐의 요구가 되도록 하는 게 이 기본값의 목적이다. 걸어 올라서는 것도 허용하려면 꺼라.
//
// 랙돌은 본마다 바디가 있어 한 액터가 오버랩 이벤트를 여러 번 낸다. 그래서 점유 수는 컴포넌트가 아니라
// **액터 단위**로 센다(OverlapCounts). 또 랙돌이 판 위에서 애니메이션으로 복귀하면(=시뮬 해제) 오버랩
// 이벤트 없이 자격만 사라지므로, 추적 중인 액터의 자격은 매 틱 재평가한다.
//
// 콘텐츠 의존이 없다(엔진 기본 셰이프 + 포인트 라이트). 플러그인 데모 맵이 프로젝트 에셋을 참조하지
// 않아야 배포본에서 그대로 열린다.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoPressurePlate.generated.h"

class UBoxComponent;
class UPointLightComponent;
class UStaticMeshComponent;

/** 압력판의 눌림 상태가 바뀔 때. bPressed = 지금 눌린 상태인지. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoPlatePressedSignature,
	ARopeDemoPressurePlate*, Plate, bool, bPressed);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Pressure Plate"))
class DYNAMICROPE_API ARopeDemoPressurePlate : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoPressurePlate();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** 지금 눌려 있는가(= 자격 있는 점유 수 >= RequiredOccupants). */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsPressed() const { return bPressed; }

	/** 자격 있는 점유 액터 수. HUD/디버그 표시용. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetOccupantCount() const { return OccupantCount; }

	/** 눌림 상태 변화 브로드캐스트. 문(ARopeDemoDoor)이 여기에 구독한다. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoPlatePressedSignature OnPlatePressedChanged;

	/** 판이 눌리는 데 필요한 점유 액터 수. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo", meta = (ClampMin = "1"))
	int32 RequiredOccupants = 1;

	/** 물리 시뮬 중인 바디만 점유로 인정. 끄면 걸어 올라선 폰도 인정한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bRequireSimulatingPhysics = true;

	/** 지정하면 이 태그를 가진 액터만 점유로 인정(비면 태그 무관). 특정 물체 전용 판을 만들 때. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	FName RequiredActorTag = NAME_None;

	/** 눌렸을 때 판이 내려가는 깊이(cm). 연출 전용 — 판정과 무관. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (ClampMin = "0.0", Units = "cm"))
	float PressDepth = 8.0f;

	/** 판이 내려가고 올라오는 속도(cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (ClampMin = "1.0"))
	float PressSpeed = 40.0f;

	/** 눌림 여부를 색으로 알리는 인디케이터 라이트를 쓸지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation")
	bool bUseIndicatorLight = true;

	/** 안 눌린 상태의 라이트 색. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (EditCondition = "bUseIndicatorLight"))
	FLinearColor IdleColor = FLinearColor(1.0f, 0.25f, 0.1f);

	/** 눌린 상태의 라이트 색. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (EditCondition = "bUseIndicatorLight"))
	FLinearColor PressedColor = FLinearColor(0.15f, 1.0f, 0.3f);

protected:
	/** 고정 프레임(테두리). 판이 내려갈 자리를 시각적으로 잡아준다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Frame = nullptr;

	/** 실제로 내려가는 판. 연출은 이 컴포넌트의 상대 Z만 움직인다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Pad = nullptr;

	/** 점유 감지 볼륨. 판 위 공간을 덮는다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UBoxComponent> Trigger = nullptr;

	/** 눌림 인디케이터. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UPointLightComponent> IndicatorLight = nullptr;

private:
	UFUNCTION()
	void HandleBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	UFUNCTION()
	void HandleEndOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);

	/** 태그/물리 시뮬 조건을 만족하는 점유인지. */
	bool IsQualifyingOccupant(const AActor* OtherActor) const;

	/** 추적 중인 액터를 재평가해 OccupantCount/bPressed를 갱신하고, 변화 시 브로드캐스트한다. */
	void RefreshPressedState();

	/** 인디케이터 색을 현재 상태에 맞춘다. */
	void ApplyIndicatorColor();

	/** 액터 단위 오버랩 카운트(랙돌 = 본 바디 다수 → 액터 하나로 합산). */
	TMap<TWeakObjectPtr<AActor>, int32> OverlapCounts;

	int32 OccupantCount = 0;
	bool  bPressed = false;

	/** 현재 판의 상대 Z(연출 보간 상태). 0 = 원위치, -PressDepth = 완전히 눌림. */
	float CurrentPadOffset = 0.0f;
};
