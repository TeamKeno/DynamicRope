// Copyright Epic Games, Inc. All Rights Reserved.
//
// 데모 씬용 문. 지정한 압력판(ARopeDemoPressurePlate)이 모두(또는 RequiredPressedCount만큼) 눌리면
// 열린다 — "단계를 클리어하면 다음 구역이 열린다"는 데모 진행의 뼈대다.
//
// 판 연결은 두 가지다:
//  1) Plates 배열에 레벨의 판 액터를 직접 지정(명시적. 어떤 판이 이 문에 걸리는지 눈에 보인다).
//  2) PlateTag를 주면 BeginPlay에 그 태그를 가진 판을 월드에서 수집(레벨 배치만으로 배선 끝).
// 둘을 함께 써도 되고, 중복 지정은 합쳐진다.
//
// bStayOpen(기본 켬)이면 한 번 열린 문은 판이 비어도 닫히지 않는다 — 퍼즐 클리어는 되돌아가지 않는
// 게 보통이고, 물체를 판에 얹은 채로 통과할 필요도 없어진다. 끄면 판이 비는 즉시 다시 닫힌다.
//
// 콘텐츠 의존이 없다(엔진 기본 셰이프). 문짝은 로컬 축으로 OpenOffset만큼 미끄러진다 — 회전 대신
// 슬라이드를 쓰는 이유는 문틀·힌지 에셋 없이도 어색하지 않기 때문이다.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoDoor.generated.h"

class ARopeDemoPressurePlate;
class UStaticMeshComponent;

/** 문이 열리거나 닫힐 때(연출 완료가 아니라 상태 전이 시점). */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoDoorStateSignature,
	ARopeDemoDoor*, Door, bool, bOpen);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Door"))
class DYNAMICROPE_API ARopeDemoDoor : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoDoor();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** 문이 열린 상태인가(연출 진행 중이어도 논리 상태 기준). */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsOpen() const { return bOpen; }

	/** 지금 눌려 있는 판 수. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetPressedPlateCount() const;

	/** 열림 조건을 채우는 데 필요한 판 수(= RequiredPressedCount, 0이면 전체). */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetRequiredPlateCount() const;

	/** 판 상태와 무관하게 강제로 열고/닫는다(치트·컷신·BP 스크립팅용). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void SetOpen(bool bNewOpen);

	/** 문 상태 전이 브로드캐스트. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoDoorStateSignature OnDoorStateChanged;

	/** 이 문을 여는 압력판들. 레벨에서 직접 지정한다. */
	UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<TObjectPtr<ARopeDemoPressurePlate>> Plates;

	/** 지정하면 BeginPlay에 이 태그를 가진 판을 월드에서 추가로 수집한다(비면 수집 안 함). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	FName PlateTag = NAME_None;

	/** 열리는 데 필요한 눌린 판 수. 0이면 연결된 판 **전부**가 눌려야 한다(기본 = 2판 퍼즐). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo", meta = (ClampMin = "0"))
	int32 RequiredPressedCount = 0;

	/** 한 번 열리면 계속 열린 채로 둘지. 끄면 조건이 깨지는 즉시 닫힌다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bStayOpen = true;

	/** 열릴 때 문짝이 이동하는 로컬 오프셋(기본 = 위로 220cm 상승). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation")
	FVector OpenOffset = FVector(0.0f, 0.0f, 220.0f);

	/** 문짝 이동 속도(cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (ClampMin = "1.0"))
	float OpenSpeed = 120.0f;

protected:
	/** 문틀(고정). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Frame = nullptr;

	/** 실제로 움직이는 문짝. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Leaf = nullptr;

private:
	/** 판이 눌리거나 풀릴 때 호출(델리게이트 시그니처). */
	UFUNCTION()
	void HandlePlatePressedChanged(ARopeDemoPressurePlate* Plate, bool bPressed);

	/** 조건을 재평가해 bOpen을 갱신하고, 변화 시 브로드캐스트한다. */
	void EvaluateOpenCondition();

	/** PlateTag로 월드의 판을 수집해 Plates에 합친다(중복 제거). */
	void GatherTaggedPlates();

	/** 문짝의 닫힘 위치(BeginPlay 시점의 상대 위치). */
	FVector ClosedLeafLocation = FVector::ZeroVector;

	bool bOpen = false;
};
