// Copyright Epic Games, Inc. All Rights Reserved.
//
// 데모 씬용 로프 엘리베이터 — "천장을 그래플로 감아 스스로를 릴로 감아올리는 탑승 플랫폼".
// 물리 climb-in 데모다: 플랫폼(물리 바디)에 붙은 ③ GuaranteedWrap 로프가 천장 앵커를 감으면(Wrapped),
// 릴-인 + 능동 Pull(climb-in)이 플랫폼을 앵커 쪽으로 끌어올린다. 릴-아웃하면 로프가 풀려 중력으로 내려온다.
//
// 진행 개요:
//   Idle → (앵커로 Guaranteed 그래플 발사) → Establishing → (Wrapped 확인) → Docked
//        → RequestAscend → Ascending → (최소 길이 도달) → 정지(위)
//        → RequestDescend → Descending → (최대 길이 도달) → 정지(아래)
// 그래플은 **한 번만** 확립하고 이후엔 릴로만 오르내린다(실제 엘리베이터 케이블처럼 계속 붙어 있다).
//
// 콘텐츠 의존:
//   - 플랫폼/셰이프는 엔진 기본 큐브(플러그인 → /Game 참조 금지 규칙 준수).
//   - **천장 앵커만 예외**: 로프가 감으려면 wrappable 대상(스켈레탈 본 콜라이더 / SDF)이어야 하므로,
//     AnchorTarget에 그런 컴포넌트를 레벨에서 지정한다(1-본 스켈레탈 "고리" 등). 없으면 경고 후 no-op.
//
// ⚠ 물리 튜닝은 PIE 실측이 정본이다: 테더/climb-in은 캐릭터 wielder로 튜닝됐어서, 물리 플랫폼을 수직으로
//   견인하는 건 첫 실측이다. 탑승 안정성(흔들림/캐릭터 충돌), 상승력(ClimbForce vs 플랫폼 질량),
//   하강 속도(DescendReelSpeed vs 중력 낙하)는 아래 노브로 조정한다.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoElevator.generated.h"

class URopeComponent;
class UStaticMeshComponent;
class ARopeDemoPressurePlate;

/** 엘리베이터가 목표 층(위/아래)에 도착한 순간(연출 완료가 아니라 정지 전이 시점). */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoElevatorArrivedSignature,
	ARopeDemoElevator*, Elevator, bool, bAtTop);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Elevator"))
class DYNAMICROPE_API ARopeDemoElevator : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoElevator();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** 위층으로 올라가도록 지시한다(그래플이 확립돼 있어야 실제로 움직인다). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void RequestAscend() { SetTargetTop(true); }

	/** 아래층으로 내려가도록 지시한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void RequestDescend() { SetTargetTop(false); }

	/** 목표 층을 뒤집는다(입력 한 키 토글용). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void ToggleTarget() { SetTargetTop(!bTargetTop); }

	/** 목표 층을 직접 지정한다(true=위, false=아래). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void SetTargetTop(bool bNewTargetTop);

	/** 그래플이 확립돼(천장에 감겨) 승강이 가능한 상태인가. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsGrappleReady() const { return bGrappleReady; }

	/** 현재 목표가 위층인가. */
	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsTargetTop() const { return bTargetTop; }

	/** 목표 층 도착 브로드캐스트. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoElevatorArrivedSignature OnElevatorArrived;

	//~ 앵커(천장 그래플 대상) -------------------------------------------------

	/** 로프가 감을 천장 앵커. **wrappable해야 한다**(스켈레탈 본 콜라이더 / SDF 프로바이더 보유).
	 *  레벨에서 지정한다 — 비면 그래플이 확립되지 않아 엘리베이터가 움직이지 않는다(경고 로그). */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<AActor> AnchorTarget = nullptr;

	//~ 트리거(호출 버튼) -----------------------------------------------------

	/** 이 압력판이 눌리면 위층, 풀리면 아래층으로 향한다(호출 버튼). 비워도 BP/코드로 직접 제어 가능. */
	UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<ARopeDemoPressurePlate> CallPlate = nullptr;

	//~ 승강 튜닝(PIE 실측) ---------------------------------------------------

	/** 상승 시 릴-인 속도(cm/s). 로프가 짧아지며 테더가 플랫폼을 앵커 쪽으로 끌어올린다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "1.0", Units = "cm/s"))
	float AscendReelSpeed = 150.0f;

	/** 하강 시 릴-아웃 속도(cm/s). 로프가 길어지며 플랫폼이 중력으로 내려온다. 중력 낙하보다 느리면
	 *  테더에 매달려 천천히 내려오고, 빠르면 자유낙하에 가깝다 — PIE로 맞춘다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "1.0", Units = "cm/s"))
	float DescendReelSpeed = 120.0f;

	/** 상승 중 능동 Pull(climb-in) 견인력. 릴-인만으로 플랫폼 무게를 못 들면 이 힘이 보태 끌어올린다.
	 *  0이면 릴-인 + 테더만으로 상승(테더 MaxTetherTension이 플랫폼 무게를 넘어야 한다). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "0.0"))
	float ClimbForce = 200000.0f;

	/** 목표 길이 도달 판정 여유(cm). 현재 로프 길이가 최소/최대에서 이 값 이내면 도착으로 본다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Travel", meta = (ClampMin = "0.1", Units = "cm"))
	float ArrivalTolerance = 5.0f;

protected:
	/** 탑승 플랫폼(물리 바디, 루트). climb-in의 견인 수신자다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UStaticMeshComponent> Platform = nullptr;

	/** 천장 앵커를 감는 ③ GuaranteedWrap 로프. 플랫폼 위에 부착돼 있어 시작점이 플랫폼을 따라간다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<URopeComponent> Rope = nullptr;

private:
	/** 앵커를 향해 Guaranteed 그래플을 발사한다(Wrapped 확립 시도). 성공 큐잉 시 true. */
	bool FireGrapple();

	/** 앵커의 조준 목표 월드 위치(지정 소켓/본이 있으면 그 위치, 없으면 앵커 액터 위치). */
	FVector ResolveAnchorAimWorld() const;

	/** 압력판 상태 변화(델리게이트 시그니처) — 눌림=위, 풀림=아래. */
	UFUNCTION()
	void HandleCallPlateChanged(ARopeDemoPressurePlate* Plate, bool bPressed);

	/** 그래플 미확립 상태에서 재발사 쿨다운(초). Wrapped 될 때까지 주기적으로 재시도한다. */
	float EstablishRetryRemaining = 0.0f;

	/** 그래플이 천장에 감겨 승강 가능한가(Phase==Wrapped 확인 시 true). */
	bool bGrappleReady = false;

	/** 현재 목표 층(true=위, false=아래). */
	bool bTargetTop = false;

	/** 이번 목표에 도착해 이미 브로드캐스트했는가(중복 발화 방지). */
	bool bArrivedBroadcast = false;
};
