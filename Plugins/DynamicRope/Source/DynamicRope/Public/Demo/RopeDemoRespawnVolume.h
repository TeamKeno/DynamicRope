// Copyright Epic Games, Inc. All Rights Reserved.
//
// 데모 씬용 낙사 복귀 볼륨(KillZ 대체). 레벨 아래에 깔아 두고, 여기 닿은 것을 파괴하는 대신 되돌린다:
//  - 플레이어 폰 → PlayerStart(GameMode의 선택 규칙을 그대로 사용)
//  - 물리 소품     → BeginPlay 시점에 기록해 둔 제 자리
//
// 엔진에도 낙사 처리는 있다(World Settings의 Kill Z + bEnableWorldBoundsChecks). 하지만 기본 동작인
// AActor::FellOutOfWorld는 **액터를 Destroy** 한다 — 폰은 사라진 채 리스폰이 없고(GameModeBase는
// 자동 재시작을 하지 않는다), 퍼즐용 소품이 떨어지면 영영 사라져 그 방을 클리어할 수 없게 된다.
// 데모에서 원하는 건 "파괴"가 아니라 "복귀"라 별도 볼륨을 둔다.
//
// KillZ는 이 볼륨보다 **더 아래**에 두고 최후의 안전망으로만 쓰는 걸 권한다(볼륨을 빠져나간 무언가가
// 무한히 낙하하는 것 방지).
//
// 복귀시키기 전에 관련된 로프를 먼저 푼다 — 안 그러면 감긴 로프가 맵을 가로질러 늘어난 채로 남는다.
// 대상이 랙돌이면 랙돌을 먼저 해제한다: 랙돌 중에는 본 바디가 월드 공간에 있어 액터만 옮겨도 메시가
// 따라오지 않기 때문이다(URopeRagdollResponseComponent가 붙어 있을 때만 가능 — 없으면 액터만 옮긴다).

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoRespawnVolume.generated.h"

class UBoxComponent;

/** 무언가가 복귀 처리될 때(HUD 표시·카운터 등에 쓰라고 노출). */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoRespawnSignature,
	ARopeDemoRespawnVolume*, Volume, AActor*, RespawnedActor);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Respawn Volume"))
class DYNAMICROPE_API ARopeDemoRespawnVolume : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoRespawnVolume();

	//~ AActor
	virtual void BeginPlay() override;

	/** 이 액터를 즉시 복귀시킨다(볼륨에 닿지 않아도 — 콘솔·BP·리셋 버튼용). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	bool RespawnActor(AActor* Target);

	/** 추적 중인 소품 전부를 제자리로(방 리셋). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void RespawnAllProps();

	/** 복귀 발생 브로드캐스트. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoRespawnSignature OnActorRespawned;

	/** 폰을 PlayerStart로 되돌릴지. 끄면 소품만 처리한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bRespawnPawns = true;

	/**
	 * 물리 시뮬 중인 소품을 BeginPlay에 자동 등록할지. 끄면 PropTag를 가진 액터만 등록한다.
	 * 등록되지 않은 액터가 볼륨에 닿으면 무시한다(파괴하지 않는다).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bAutoTrackPhysicsProps = true;

	/** 이 태그를 가진 액터는 물리 시뮬 여부와 무관하게 등록한다(비면 태그 등록 안 함). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	FName PropTag = TEXT("RopeDemoProp");

	/** 지정하면 폰을 PlayerStart 대신 이 액터의 트랜스폼으로 되돌린다(방별 체크포인트). */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<AActor> PawnRespawnPointOverride = nullptr;

	/** 복귀 직전에 대상과 얽힌 로프를 푼다. 끄면 맵을 가로질러 늘어난 로프를 보게 된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bReleaseRopesOnRespawn = true;

protected:
	/** 낙사 감지 볼륨. 레벨 아래를 넓게 덮도록 배치한다. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UBoxComponent> Trigger = nullptr;

private:
	UFUNCTION()
	void HandleBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	/** 등록 대상(태그 또는 물리 시뮬)의 시작 트랜스폼을 기록한다. */
	void TrackProps();

	/** 폰을 PlayerStart(또는 오버라이드)로. */
	bool RespawnPawn(APawn* Pawn);

	/** 등록된 소품을 기록해 둔 자리로. */
	bool RespawnProp(AActor* Prop, const FTransform& StartTransform);

	/** 이 액터가 던졌거나(wielder) 이 액터를 감고 있는 로프를 모두 푼다. */
	void ReleaseRopesInvolving(AActor* Actor);

	/** 물리 속도를 0으로(복귀 직후 남은 속도로 다시 튀어나가지 않게). */
	static void ZeroPhysicsVelocities(AActor* Actor);

	/** 소품 시작 트랜스폼. 액터가 파괴돼도 안전하도록 약참조 키. */
	TMap<TWeakObjectPtr<AActor>, FTransform> TrackedProps;
};
