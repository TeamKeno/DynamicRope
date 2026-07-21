// Fill out your copyright notice in the Description page of Project Settings.
//
// 드래곤 데모 컴포넌트(정식 기능 아님, 데모 폴더) — **10초 안쪽 데모 영상 한 컷**을 위한 하드코딩 연출.
//
// 흐름(고정):
//   Idle(가만히 앉아 있음)
//     └ 로프가 이 액터를 감으면(중앙 OnAnyRopeWrapped) 또는 Rope.Demo.Dragon / StartSequence()로 수동 발동
//   Howl(하울링 1회)
//   Thrash(지면과 수평인 8자 궤적으로 몸부림)
//   Ascend(위로 솟구침)
//   Descend(다시 내려와 정지) → Idle 애니메이션으로 복귀하고 끝
//
// 설계 메모:
//  - 이전 버전의 "스티어링 선회 비행"(로프 테더에 실제로 끌려가던 물리 친화 경로)은 통째로 버렸다.
//    이 컴포넌트는 연출 전용이라 매 틱 액터 트랜스폼을 **경로식으로 덮어쓴다** — 로프 테더가 밀어낸
//    변위는 남지 않는다. 영상용 결정론적 타이밍이 목적이고, 물리 상호작용 데모는 별도다.
//  - 애니메이션은 AnimBP 없이 단일 AnimSequence 3종(Idle/Howl/Fly)을 SkeletalMeshComponent의
//    싱글 노드 모드(PlayAnimation)로 직접 갈아 끼운다. 드래곤 BP에 AnimBP를 붙이지 말 것.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RopeDragonFlightDemoComponent.generated.h"

class UAnimSequenceBase;
class URopeComponent;
class USkeletalMeshComponent;
struct FRopeWrappedEventInfo;

/** 데모 연출 단계. 순서는 고정이며 되돌아가지 않는다(Finished 뒤엔 Idle 애님만 유지). */
UENUM(BlueprintType)
enum class ERopeDragonDemoState : uint8
{
	Idle		UMETA(DisplayName = "Idle"),
	Howl		UMETA(DisplayName = "Howl"),
	Thrash		UMETA(DisplayName = "Thrash (8자)"),
	Ascend		UMETA(DisplayName = "Ascend"),
	Descend		UMETA(DisplayName = "Descend"),
	Finished	UMETA(DisplayName = "Finished")
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPEPROJECT_API URopeDragonFlightDemoComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeDragonFlightDemoComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	//==================================================================================
	// 애니메이션 — AnimBP 없이 단일 시퀀스를 직접 재생한다.
	//==================================================================================

	/** 대기/종료 포즈(루프). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Anim")
	TObjectPtr<UAnimSequenceBase> IdleAnim;

	/** 발동 직후 1회 재생하는 하울링. 이 시퀀스 길이가 Howl 단계의 길이가 된다(HowlDuration=0일 때). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Anim")
	TObjectPtr<UAnimSequenceBase> HowlAnim;

	/** 몸부림/상승/하강 내내 도는 비행 루프. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Anim")
	TObjectPtr<UAnimSequenceBase> FlyAnim;

	//==================================================================================
	// 타이밍 — 전부 합쳐 10초 안쪽이 되도록 잡은 기본값.
	//==================================================================================

	/** 하울링 길이(초). 0 = HowlAnim 시퀀스 길이를 그대로 쓴다(없으면 1.5초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Timing", meta = (ClampMin = "0.0", Units = "s"))
	float HowlDuration = 0.0f;

	/** 8자 몸부림 길이(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Timing", meta = (ClampMin = "0.1", Units = "s"))
	float ThrashDuration = 3.5f;

	/** 상승 길이(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Timing", meta = (ClampMin = "0.1", Units = "s"))
	float AscendDuration = 1.8f;

	/** 하강 길이(초). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Timing", meta = (ClampMin = "0.1", Units = "s"))
	float DescendDuration = 1.6f;

	//==================================================================================
	// 8자 궤적 — 발동 시점의 액터 트랜스폼을 원점/기저로 삼는다(지면과 수평).
	//==================================================================================

	/** 8자 긴 축(발동 시점 전방) 반폭(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "0.0", Units = "cm"))
	float ThrashLength = 700.0f;

	/** 8자 짧은 축(발동 시점 오른쪽) 반폭(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "0.0", Units = "cm"))
	float ThrashWidth = 450.0f;

	/** 몸부림 동안 8자를 도는 횟수. 정수여야 시작/끝이 원점에서 매끄럽게 만난다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "1"))
	int32 ThrashLoops = 2;

	/** 8자를 도는 동안의 상하 흔들림 진폭(cm). 완전 평면이면 밋밋해서 살짝만 준다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "0.0", Units = "cm"))
	float ThrashBobHeight = 120.0f;

	/** 8자 교차 방향에 맞춘 뱅크(롤) 최대각(도). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "0.0", ClampMax = "80.0"))
	float ThrashBankDeg = 35.0f;

	//==================================================================================
	// 상승/하강.
	//==================================================================================

	/** 상승 높이(cm, 발동 지점 기준). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Climb", meta = (ClampMin = "0.0", Units = "cm"))
	float AscendHeight = 1400.0f;

	/** 상승 중 앞으로 밀려 나가는 거리(cm) — 수직으로만 뜨면 부자연스러우니 전방 성분을 섞는다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Climb", meta = (ClampMin = "0.0", Units = "cm"))
	float AscendForward = 500.0f;

	/** 상승/하강 시 몸을 세우는 피치 최대각(도). 상승=위로, 하강=아래로. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Climb", meta = (ClampMin = "0.0", ClampMax = "80.0"))
	float ClimbPitchDeg = 30.0f;

	/** 하강이 끝났을 때 발동 지점으로 되돌아올지(false면 상승 때 밀려난 전방 위치에 착지). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Climb")
	bool bReturnToStartOnLand = false;

	//==================================================================================
	// 발동.
	//==================================================================================

	/** 로프가 이 액터의 메시를 감으면 자동 발동. false면 StartSequence()/콘솔로만 발동. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Trigger")
	bool bStartOnWrapped = true;

	/** 감긴 뒤 하울링까지의 지연(초) — 로프가 걸리는 순간을 한 박자 보여주고 반응한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Trigger", meta = (ClampMin = "0.0", Units = "s"))
	float TriggerDelay = 0.2f;

	/** 연출 시작(이미 진행 중이거나 끝났으면 무시). */
	UFUNCTION(BlueprintCallable, Category = "Rope|DragonDemo")
	void StartSequence();

	/** Idle 상태로 되돌린다(발동 지점 트랜스폼 복원 — 반복 촬영용). */
	UFUNCTION(BlueprintCallable, Category = "Rope|DragonDemo")
	void ResetSequence();

	/** 현재 단계. */
	UFUNCTION(BlueprintPure, Category = "Rope|DragonDemo")
	ERopeDragonDemoState GetDemoState() const { return State; }

private:
	void HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info);

	USkeletalMeshComponent* ResolveMesh() const;
	void PlayAnim(UAnimSequenceBase* Anim, bool bLoop);
	void EnterState(ERopeDragonDemoState NewState);

	/** 발동 지점 기준 로컬 오프셋/회전을 월드에 적용한다(모든 단계 공용 출구). */
	void ApplyPose(const FVector& LocalOffset, float YawDeg, float PitchDeg, float RollDeg);

	float ResolveHowlDuration() const;

	ERopeDragonDemoState State = ERopeDragonDemoState::Idle;
	float TimeInState = 0.0f;
	float PendingTriggerTime = -1.0f;

	/** 발동 시점의 액터 위치/기저(Yaw만 사용 — 8자는 지면과 수평이어야 한다). */
	FVector AnchorLocation = FVector::ZeroVector;
	float AnchorYawDeg = 0.0f;

	/** 상승 끝에서의 로컬 오프셋 — 하강이 여기서 출발한다. */
	FVector AscendEndOffset = FVector::ZeroVector;

	FDelegateHandle WrappedHandle;
};
