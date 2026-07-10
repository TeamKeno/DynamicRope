// Fill out your copyright notice in the Description page of Project Settings.
//
// 드래곤 비행 데모 컴포넌트(7.09 마일스톤 "드래곤 — 날리기/당기기" — 정식 기능 아님, 데모 폴더).
// 스켈레탈 메시를 가진 드래곤 액터에 붙이면 시작 지점 주변을 선회 비행한다.
//
// 핵심 설계 — 절대 경로가 아니라 *스티어링*이다:
//  매 틱 "궤도 접선 + 반경 복원 + 고도 복원"으로 원하는 방향을 만들고, 현재 헤딩을 TurnRate로
//  그쪽에 회전시켜 전진(AddActorWorldOffset)한다. 위치를 경로식으로 덮어쓰지 않으므로
//  로프 테더가 밀어낸 변위(UpdateTether의 비캐릭터 오프셋 폴백)가 그대로 누적된다 —
//  즉 로프로 나는 드래곤을 실제로 끌어당길 수 있고, 드래곤은 끌려간 자리에서 궤도로 복귀하려 한다.
//
// 로프 연동 시나리오(로프 쪽 설정으로 갈린다):
//  - 당기기(끌어내리기): 로프 TetherTargetShare=1(기본) + 되감기 → 초과분만큼 드래곤이 끌려온다.
//    WrappedSpeedScale로 감긴 동안 비행이 둔해지는 반응을 함께 준다.
//  - 타고 끌려가기: TetherTargetShare=0 → wielder가 드래곤에 견인된다(지상 이탈/스윙/되감기 등반은
//    Wielder 쪽 기존 기능). TensionReleaseForce/DistanceReleaseSlack은 0(기본) 유지해야 안 풀린다.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RopeDragonFlightDemoComponent.generated.h"

class URopeComponent;
class USkeletalMeshComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPEPROJECT_API URopeDragonFlightDemoComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeDragonFlightDemoComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** 순항 속도(cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo", meta = (ClampMin = "0.0", Units = "cm/s"))
	float FlightSpeed = 600.0f;

	/** 선회 반경(cm). 궤도 중심은 BeginPlay 때 시작 위치 오른쪽으로 이 거리만큼 떨어진 곳. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo", meta = (ClampMin = "100.0", Units = "cm"))
	float OrbitRadius = 1500.0f;

	/** 헤딩 회전 속도(도/초). 낮을수록 크게 도는 둔한 비행. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo", meta = (ClampMin = "1.0"))
	float TurnRateDeg = 45.0f;

	/** 고도 복원 최대 상승/하강 성분(순항 속도 대비 비율 0~1). 테더에 끌어내려져도 이 비율로만 되오른다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MaxClimbRatio = 0.35f;

	/** 선회 뱅크(롤) 최대각(도). 시각용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo", meta = (ClampMin = "0.0", ClampMax = "60.0"))
	float BankAngleMax = 20.0f;

	/** 로프에 감긴(Wrapped) 동안의 속도 배율. 1 = 반응 없음, 0.5 = 절반으로 둔해짐. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WrappedSpeedScale = 0.5f;

	/** 지금 어떤 로프가 이 액터의 메시를 감고 있는가(반응 판정과 동일한 질의 — BP 연출용). */
	UFUNCTION(BlueprintPure, Category = "Rope|DragonDemo")
	bool IsWrappedByRope() const { return FindRopeWrappingUs() != nullptr; }

private:
	URopeComponent* FindRopeWrappingUs() const;
	USkeletalMeshComponent* ResolveMesh() const;

	FVector OrbitCenter = FVector::ZeroVector;
	float PreferredAltitude = 0.0f;
	FVector Heading = FVector::ForwardVector;
	float CurrentBank = 0.0f;
};
