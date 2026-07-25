// Copyright Epic Games, Inc. All Rights Reserved.
//
// 데모 씬용 로프 구조 헬기 — 상공을 호버링하다가 승객이 아래 잡기 구역에 들어오면 로프를 내려꽂아
// (③ GuaranteedWrap으로 손 본을 감아) 릴-인으로 끌어올린 뒤, 경유지(Waypoints)를 따라 비행해
// 목적지에서 내려준다(release). 놓은 뒤에는 시작 지점으로 복귀해 다음 승객을 기다린다.
//
// 진행 개요:
//   Idle(호버+로터) → [잡기 구역 진입/Grab()] Grabbing(손 본으로 Guaranteed 발사, 재시도)
//        → Carrying(릴-인으로 끌어올림 → Waypoints 순회 비행)
//        → [마지막 경유지 도착] ReleaseCarried() → Returning(홈 복귀) → Idle
//
// 콘텐츠 의존:
//   - 기체/로터 메시는 레벨/BP에서 지정한다(BodyMesh/RotorMesh — 플러그인 → /Game 참조 금지 규칙).
//   - 승객은 스켈레탈 메시 + 로프 콜라이더 프로바이더(캡슐/SDF)가 있어야 감긴다(GrabBone 기본 hand_r).
//   - 승객에 URopeRagdollResponseComponent(bRagdollOnWrapped)가 있으면 감기는 순간 랙돌이 된다 —
//     축 늘어진 "매달린 화물" 연출이면 그대로 두고, 조작을 유지할 플레이어면 붙이지 말 것.
//
// ⚠ 물리 튜닝은 PIE 실측이 정본이다: 걷는 캐릭터를 지면에서 들어올리는 힘은 릴-인 테더와
//   CarryPullForce의 합이 결정한다 — 안 들리면 LiftReelSpeed/CarryPullForce를 올린다.
//   (스네어와 동일한 계약: 해제 시 발사 큐 취소 + 전 페이즈 ReleaseWrap — CL 768의 교훈 반영.)

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoHelicopter.generated.h"

class URopeComponent;
class UStaticMeshComponent;
class USphereComponent;
class USkeletalMeshComponent;

/** 승객을 실었/내렸을 때(감김 성립/해제 시점). */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoHelicopterCarrySignature,
	ARopeDemoHelicopter*, Helicopter, bool, bCarrying);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Helicopter"))
class DYNAMICROPE_API ARopeDemoHelicopter : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoHelicopter();

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** 지정 승객을 향해 로프를 내려꽂는다(잡기 시작). Idle에서만 유효 — 성공 시 true. 자동 잡기
	 *  (bAutoGrab) 대신/외에 BP·코드로 직접 태울 때 쓴다. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	bool Grab(AActor* Passenger);

	/** 진행 중인 잡기 시도를 중단한다(발사 큐/비행 중 로프 회수 포함). Carrying에는 ReleaseCarried를 쓸 것. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void CancelGrab();

	/** 실은 승객을 내려놓는다(감김 해제 + 로프 길이 복원). 이후 bReturnHomeAfterRelease에 따라 홈으로
	 *  복귀하거나 그 자리에서 호버한다. 디테일 패널 버튼으로도 호출. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Rope|Demo")
	void ReleaseCarried();

	/** 승객을 감아 실은 상태인가(끌어올리는 중 포함). */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsCarrying() const;

	/** 현재 실은(또는 잡는 중인) 승객. 없으면 nullptr. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	AActor* GetCarriedActor() const { return CarryTarget.Get(); }

	/** 감김 성립/해제 브로드캐스트. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoHelicopterCarrySignature OnCarryStateChanged;

	//~ 경로 -------------------------------------------------------------------

	/** 승객을 실은 뒤 순서대로 지나는 경유지(TargetPoint 등 아무 액터). 마지막이 목적지다.
	 *  비면 제자리에서 들고만 있는다(호버 크레인). */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo")
	TArray<TObjectPtr<AActor>> Waypoints;

	//~ 잡기(승객 획득) ---------------------------------------------------------

	/** 잡기 구역에 스켈레탈 메시 보유 액터가 들어오면 자동으로 로프를 내려꽂는다. 끄면 Grab() 호출 전용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab")
	bool bAutoGrab = true;

	/** 감을 승객 본(잡는 손). 승객 스켈레톤에 없으면 잡기가 시작되지 않는다(경고 로그). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab")
	FName GrabBone = TEXT("hand_r");

	/** 잡기 구역(구) 중심의 기체 아래 거리(cm). 로프가 내려꽂힐 사거리 안이어야 한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab", meta = (ClampMin = "0.0", Units = "cm"))
	float GrabZoneDrop = 600.0f;

	/** 잡기 구역 반지름(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab", meta = (ClampMin = "10.0", Units = "cm"))
	float GrabZoneRadius = 250.0f;

	/** 잡기 재시도 간격(초) — 스네어와 동일한 장전 에지 안정화 여유. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab", meta = (ClampMin = "0.1", Units = "s"))
	float GrabRetryPeriod = 1.0f;

	/** 이 시간 안에 감기지 못하면 잡기를 포기한다(초, 0 = 무제한). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Grab", meta = (ClampMin = "0.0", Units = "s"))
	float GrabTimeout = 8.0f;

	//~ 수송(끌어올림/비행) -----------------------------------------------------

	/** 수송 중 케이블 길이(cm) — 릴-인으로 이 길이까지 감아 승객을 끌어올린다(짧을수록 높이 매달림). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry", meta = (ClampMin = "50.0", Units = "cm"))
	float CarryRopeLength = 350.0f;

	/** 끌어올리는 릴-인 속도(cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry", meta = (ClampMin = "1.0", Units = "cm/s"))
	float LiftReelSpeed = 200.0f;

	/** 수송 중 능동 Pull 견인력(0 = 릴-인 테더만). 릴만으로 승객이 지면에서 안 뜨면 보탠다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry", meta = (ClampMin = "0.0"))
	float CarryPullForce = 0.0f;

	/** 케이블 길이 도달 판정 여유(cm) — 이 안이면 "다 끌어올렸다"로 보고 비행을 시작한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry", meta = (ClampMin = "0.1", Units = "cm"))
	float LiftTolerance = 15.0f;

	/** 마지막 경유지에 도착하면 자동으로 내려놓는다. 끄면 ReleaseCarried 호출 전까지 들고 호버한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry")
	bool bReleaseAtLastWaypoint = true;

	/** 내려놓은 뒤 시작 지점으로 복귀한다. 끄면 그 자리에서 호버하며 다음 승객을 기다린다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Carry")
	bool bReturnHomeAfterRelease = true;

	//~ 비행 연출 --------------------------------------------------------------

	/** 순항 속도(cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "1.0", Units = "cm/s"))
	float FlySpeed = 600.0f;

	/** 경유지 도착 판정 반경(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "1.0", Units = "cm"))
	float WaypointTolerance = 100.0f;

	/** 진행 방향으로 기수를 돌린다(요만 — 기울임 없음). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight")
	bool bFaceTravelDirection = true;

	/** 기수 회전 속도(도/초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "1.0"))
	float TurnRateDeg = 90.0f;

	/** 호버 승강 흔들림 진폭(cm, 0 = 끔). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "0.0", Units = "cm"))
	float HoverBobAmplitude = 15.0f;

	/** 호버 흔들림 주기(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "0.1", Units = "s"))
	float HoverBobPeriod = 3.0f;

	/** 로터 회전 속도(RPM, 0 = 정지). RotorMesh에만 적용된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Flight", meta = (ClampMin = "0.0"))
	float RotorRPM = 300.0f;

protected:
	/** 루트(비행 기준점). 기체는 키네마틱으로 이 액터 위치를 직접 움직인다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USceneComponent> Base = nullptr;

	/** 기체 메시(레벨/BP에서 지정). 충돌 설정은 에셋을 따른다 — PhysicsBody를 Block하면 다른 로프가 감/부딪을 수 있다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> BodyMesh = nullptr;

	/** 로터 메시(선택 — 기체와 분리된 로터 에셋이 있을 때 지정). RotorRPM으로 요 축 회전한다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> RotorMesh = nullptr;

	/** 로프가 매달리는 기체 하단 지점(디테일에서 위치 조정). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USceneComponent> RopeAttach = nullptr;

	/** 구조 케이블(③ GuaranteedWrap). 승객 손 본을 감는다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<URopeComponent> Rope = nullptr;

	/** 잡기 구역(기체 아래 구 — QueryOnly라 로프 콜라이더로는 수집되지 않는다). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<USphereComponent> GrabVolume = nullptr;

private:
	/** 데모 상태(전이는 Tick/각 API가 소유). */
	enum class EState : uint8 { Idle, Grabbing, Carrying, Returning };

	/** 승객을 향해 케이블을 Guaranteed 발사한다(③ 장전 에지 포함). 큐잉 성공 시 true. */
	bool FireRopeAtTarget();

	/** 승객의 스켈레탈 메시(첫 번째). */
	USkeletalMeshComponent* ResolveTargetMesh() const;

	/** 케이블 회수 공통부: 발사 큐 취소 + 전 페이즈 release + 길이/견인 복원(CL 768과 동일 위생). */
	void RecallRope();

	/** 목표점으로 등속 이동(+선택 기수 회전). 남은 거리를 돌려준다. */
	float MoveTowards(const FVector& Dest, float DeltaSeconds);

	/** 현 위치(흔들림 제외 기준점)와 기수를 액터에 반영한다. */
	void ApplyPose();

	/** 잡기 구역 진입(자동 잡기). */
	UFUNCTION()
	void HandleGrabZoneBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	EState State = EState::Idle;

	/** 잡는 중/실은 승객(파괴 안전 weak). */
	TWeakObjectPtr<AActor> CarryTarget;

	/** 호버 기준점(흔들림 제외). Idle/Returning의 목적지이기도 하다. */
	FVector IdleAnchor = FVector::ZeroVector;

	/** 이동 기준 현재 위치(흔들림 제외 — 흔들림은 표시에만 얹는다). */
	FVector NavPos = FVector::ZeroVector;

	/** 현재 기수 요(도). */
	float NavYaw = 0.0f;

	/** 호버 흔들림 시계(초). */
	float BobTime = 0.0f;

	/** 잡기 재시도 쿨다운/경과(초). */
	float GrabRetryRemaining = 0.0f;
	float GrabElapsed = 0.0f;

	/** 현재 향하는 경유지 인덱스. */
	int32 WaypointIndex = 0;

	/** 감김 성립을 브로드캐스트했는가(중복 방지). */
	bool bCarryBroadcast = false;
};
