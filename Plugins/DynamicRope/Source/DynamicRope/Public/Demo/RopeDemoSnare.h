// Copyright Epic Games, Inc. All Rights Reserved.
//
// 데모 씬용 "대자 결박" 함정 — 여러 로프가 각각 대상의 다른 팔다리를 감아 사방으로 당겨,
// 대상을 대(大)자로 벌린 채 고정한다.
//
// 진행 개요:
//   Idle → TriggerSnare() → Binding(팔다리별 ③ Guaranteed 조준 발사, 전부 Wrapped 될 때까지 재시도)
//        → Snared(케이블을 SnareLength까지 릴-인 → 사지가 앵커 쪽으로 벌어짐)
//        → ReleaseSnare() → Idle(감김 해제 + 길이 복원)
//
// 앵커 슬롯은 4개(팔 2 + 다리 2)지만 **Bindings에 넣은 만큼만** 쓴다 — 2개면 양팔 결박
// (2로프 스파이크), 4개면 완전한 대자다. 코드 변경 없이 레벨에서 늘린다.
//
// 대상 조건:
//   - TargetActor에 스켈레탈 메시 + 로프 콜라이더 프로바이더(캡슐/SDF)가 있어야 감긴다.
//   - 팔다리가 물리로 끌려가려면 랙돌이어야 한다. 대상에 URopeRagdollResponseComponent가 있으면
//     감김 이벤트로 자동 전환되지만, 결박은 **감기 전에** 랙돌이어야 사지가 순순히 벌어지므로
//     bForceRagdollOnSnare(기본 켬)가 발사 시점에 먼저 랙돌로 만든다.
//
// ⚠ Guaranteed는 조준 ray가 **처음 맞은** wrappable 본을 잠근다 — hand_l을 조준해도 앞을 가로막은
//   lowerarm_l이 잡힐 수 있다(대자 연출로는 둘 다 무방). 실제로 무엇을 잡았는지는 결박 완료 로그가 찍는다.
//
// ⚠ 물리 튜닝은 PIE 실측이 정본이다: 랙돌 한 구를 여러 테더가 동시에 당기는 건 이 데모가 첫 실측이라
//   릴 속도/최종 길이/추가 견인력을 아래 노브로 맞춘다.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoSnare.generated.h"

class URopeComponent;
class UStaticMeshComponent;
class USkeletalMeshComponent;
class ARopeDemoPressurePlate;

/** 결박 슬롯 하나 — "이 본을, 저 앵커 위치로 당긴다". */
USTRUCT(BlueprintType)
struct FRopeDemoSnareBinding
{
	GENERATED_BODY()

	/** 감을 대상 본(예: hand_l / hand_r / foot_l / foot_r). 비면 이 슬롯은 쓰지 않는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	FName Bone = NAME_None;

	/** 이 본을 당길 앵커 위치(스네어 액터 로컬, cm). 앵커 표식과 로프 시작점이 여기에 놓인다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	FVector AnchorOffset = FVector::ZeroVector;
};

/** 결박이 성립(모든 슬롯 Wrapped)하거나 풀린 순간. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoSnareStateSignature,
	ARopeDemoSnare*, Snare, bool, bSnared);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Snare"))
class DYNAMICROPE_API ARopeDemoSnare : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoSnare();

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** 결박을 시작한다(슬롯별 조준 발사 → 전부 감길 때까지 재시도). 디테일 패널 버튼으로도 호출. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void TriggerSnare();

	/** 결박을 푼다(모든 감김 해제 + 케이블 길이 복원). 디테일 패널 버튼으로도 호출. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void ReleaseSnare();

	/** 결박 상태를 뒤집는다(입력 한 키/디테일 패널 버튼용). */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void ToggleSnare();

	/** 모든 슬롯이 감겨 결박이 성립했는가. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsSnared() const { return bSnared; }

	/** 지금 실제로 감겨 있는 케이블 수(진행 표시용). */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetBoundRopeCount() const;

	/** 쓰이는 슬롯 수(= Bone이 지정된 Bindings 수, 최대 4). */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	int32 GetActiveBindingCount() const;

	/** 결박 성립/해제 브로드캐스트. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoSnareStateSignature OnSnareStateChanged;

	//~ 대상/슬롯 -------------------------------------------------------------

	/** 결박할 대상 액터(스켈레탈 메시 보유). 레벨에서 지정한다 — 비면 경고 후 no-op. */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<AActor> TargetActor = nullptr;

	/** 결박 슬롯(최대 4). 기본값은 양팔 2슬롯 — 다리 2슬롯을 추가하면 완전한 대자가 된다. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<FRopeDemoSnareBinding> Bindings;

	/** 이 압력판이 눌리면 결박, 풀리면 해제한다(함정 트리거). 비워도 BP/코드로 직접 제어 가능. */
	UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<ARopeDemoPressurePlate> TriggerPlate = nullptr;

	/** BeginPlay에 곧바로 결박을 시작한다(트리거 없이 "이미 걸려 있는" 연출/실측용). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bSnareOnBeginPlay = false;

	//~ 결박 튜닝(PIE 실측) ---------------------------------------------------

	/** 감긴 뒤 사지를 벌리는 릴-인 속도(cm/s). 로프가 짧아지며 본이 앵커 쪽으로 끌려간다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "1.0", Units = "cm/s"))
	float SnareReelSpeed = 120.0f;

	/** 결박 완료 시 케이블 길이(cm, 0=각 로프의 MinRopeLength까지). 짧을수록 강하게 벌어진다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "0.0", Units = "cm"))
	float SnareLength = 0.0f;

	/** 길이 도달 판정 여유(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "0.1", Units = "cm"))
	float ArrivalTolerance = 5.0f;

	/** 결박 중 **슬롯당** 능동 Pull 견인력(0=릴-인 + 테더만). 릴만으로 사지가 덜 벌어질 때 보탠다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare", meta = (ClampMin = "0.0"))
	float LimbPullForce = 0.0f;

	/** 발사 시점에 대상을 먼저 랙돌로 만든다(대상에 URopeRagdollResponseComponent가 있을 때).
	 *  끄면 감김 이벤트가 만드는 자동 전환에 맡긴다 — 사지가 애니메이션에 붙들려 덜 벌어질 수 있다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Snare")
	bool bForceRagdollOnSnare = true;

protected:
	/** 앵커 기준점(고정 루트). 슬롯 표식/로프가 여기에 붙는다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USceneComponent> Base = nullptr;

	/** 앵커 표식 4개(슬롯 위치 시각화). 쓰이지 않는 슬롯은 숨긴다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<TObjectPtr<UStaticMeshComponent>> AnchorMarkers;

	/** 슬롯별 ③ GuaranteedWrap 로프 4개. Bindings에 채운 만큼만 발사/릴한다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<TObjectPtr<URopeComponent>> Ropes;

private:
	/** 슬롯 수 상한(팔 2 + 다리 2). */
	static constexpr int32 MaxBindings = 4;

	/** 슬롯 i가 쓰이는가(Bindings 범위 안 + Bone 지정 + 로프 유효). */
	bool IsSlotActive(int32 SlotIndex) const;

	/** 앵커 표식/로프를 Bindings의 AnchorOffset에 맞춰 배치하고, 미사용 슬롯을 숨긴다. */
	void ApplyBindingLayout();

	/** 아직 감기지 않은 모든 슬롯을 대상 본으로 향해 Guaranteed 발사한다. */
	void FireSnareRopes();

	/** 슬롯 하나를 대상 본으로 향해 Guaranteed 발사한다. 큐잉 성공 시 true. */
	bool FireSnareRopeFor(int32 SlotIndex);

	/** 활성 슬롯이 모두 Wrapped인가(= 결박 성립). 활성 슬롯이 없으면 false. */
	bool AreAllBoundRopesWrapped() const;

	/** 대상의 스켈레탈 메시(첫 번째). 없으면 nullptr. */
	USkeletalMeshComponent* ResolveTargetMesh() const;

	/** 대상에 랙돌 응답 컴포넌트가 있으면 즉시 랙돌로 만든다(bForceRagdollOnSnare). */
	void ForceTargetRagdoll();

	/** 결박 성립/해제를 상태에 반영하고 변화 시 브로드캐스트한다. */
	void SetSnared(bool bNewSnared);

	/** 압력판 상태 변화(델리게이트 시그니처) — 눌림=결박, 풀림=해제. */
	UFUNCTION()
	void HandleTriggerPlateChanged(ARopeDemoPressurePlate* Plate, bool bPressed);

	/** 결박 시도 중인가(TriggerSnare ~ ReleaseSnare). */
	bool bTriggered = false;

	/** 활성 슬롯이 모두 감겨 결박이 성립했는가. */
	bool bSnared = false;

	/** 미성립 상태에서 재발사 쿨다운(초). */
	float FireRetryRemaining = 0.0f;
};
