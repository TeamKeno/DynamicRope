// Copyright Epic Games, Inc. All Rights Reserved.
//
// 캐릭터가 로프를 "들고 던지게" 해 주는 게임플레이 컴포넌트. URopeComponent를 손 소켓에 붙이고,
// 던지기 입력/조준을 한 곳에 모은다. 캐릭터에 이 컴포넌트 하나만 붙이면(+ 로프 컴포넌트) 셋업 끝.
//
// 수동으로 하던 것: 로프를 Hand_r에 reparent → BP에서 F키→Throw 배선. 이 컴포넌트가 둘 다 대신한다.
//  - 소켓 자동 부착: BeginPlay에 owner의 SkeletalMesh를 찾아 로프를 HandSocketName에 attach.
//  - Throw()/Release()/ToggleThrow() BlueprintCallable. 조준 방향은 AimSource(컨트롤 회전/카메라/액터)에서 계산.
//  - 선택적 Enhanced Input 자동 바인딩: ThrowAction/ReleaseAction(+ MappingContext)을 꽂으면 BeginPlay에 바인딩.
//    안 꽂으면 Throw()를 직접 호출하면 된다.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Core/RopeTypes.h"
#include "RopeWielderComponent.generated.h"

class URopeComponent;
class USkeletalMeshComponent;
class UInputAction;
class UInputMappingContext;
class UAnimMontage;

/** 던질 때 조준 방향을 어디서 가져올지. */
UENUM(BlueprintType)
enum class ERopeAimSource : uint8
{
	/** 컨트롤러 회전(보통 카메라/조준 방향). 기본. */
	ControlRotation,
	/** owner의 UCameraComponent forward(있으면). 없으면 ControlRotation 폴백. */
	CameraForward,
	/** owner 액터 forward. */
	ActorForward
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeWielderComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeWielderComponent();

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	//~ Setup --------------------------------------------------------------
	/** 들 로프. 비우면 BeginPlay에 owner의 URopeComponent를 자동 탐색한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	TObjectPtr<URopeComponent> Rope = nullptr;

	/** 로프를 붙일 스켈레탈 메시. 비우면 owner의 첫 USkeletalMeshComponent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	TObjectPtr<USkeletalMeshComponent> AttachMesh = nullptr;

	/** 로프를 붙일 소켓/본 이름. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	FName HandSocketName = TEXT("hand_r");

	/** BeginPlay에 로프를 소켓에 자동 부착할지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	bool bAttachOnBeginPlay = true;

	//~ Aim ----------------------------------------------------------------
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim")
	ERopeAimSource AimSource = ERopeAimSource::ControlRotation;

	//~ Throw --------------------------------------------------------------
	/** 던질 때 Up/Right 기준축을 어디서 가져올지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeThrowFrameMode ThrowFrameMode = ERopeThrowFrameMode::Owner;

	/** AimDir과 기준축을 조합해 스윙 호가 놓일 평면을 고른다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeSwingPlane SwingPlane = ERopeSwingPlane::AimAndFrameUp;

	/** Wielder가 책임지는 던지기 속도. ThrowContext를 통해 RopeComponent로 전달된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float ThrowSpeed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "ThrowFrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameForward = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "ThrowFrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameUp = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "ThrowFrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameRight = FVector::RightVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (EditCondition = "SwingPlane == ERopeSwingPlane::CustomNormal"))
	FVector CustomSwingPlaneNormal = FVector::RightVector;

	//~ Input(선택) — 비우면 무시, Throw()를 직접 호출하면 된다 ------------
	/** Action/MappingContext가 설정돼 있으면 BeginPlay에 자동 바인딩할지. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	bool bAutoBindInput = true;

	/** 플레이어에 추가할 Input Mapping Context(있으면). Action이 트리거되려면 활성 IMC에 들어 있어야 한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputMappingContext> MappingContext = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input", meta = (EditCondition = "MappingContext != nullptr"))
	int32 MappingPriority = 0;

	/** 던지기 액션. Started에 Throw(). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ThrowAction = nullptr;

	/** 해제 액션. Started에 Release(). 비우면 ThrowAction을 ToggleThrow로 쓸 수도 있다(bThrowActionToggles). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReleaseAction = nullptr;

	/** ReleaseAction이 비었을 때, ThrowAction을 던지기/해제 토글로 사용. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	bool bThrowActionToggles = true;

	//~ Animation(선택) ----------------------------------------------------
	/**
	 * 설정하면 Throw()가 즉시 던지지 않고 이 몽타주를 재생한다. 실제 로프 던지기는 몽타주 안에 배치한
	 * UAnimNotify_RopeThrow가 ThrowNow()를 호출해 일어난다(던지는 모션의 손 떼는 순간에 맞춤).
	 * 비우면 Throw()가 즉시 던진다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation")
	TObjectPtr<UAnimMontage> ThrowMontage = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation", meta = (ClampMin = "0.1"))
	float ThrowMontagePlayRate = 1.0f;

	//~ API ----------------------------------------------------------------
	/**
	 * 던지기 시작. ThrowMontage가 설정돼 있으면 몽타주를 재생(실제 던지기는 몽타주의 UAnimNotify_RopeThrow가
	 * ThrowNow() 호출), 없으면 즉시 ThrowNow(). 입력/게임플레이가 호출하는 진입점.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw();

	/** 실제 로프 던지기를 *지금* 실행한다. 방향은 ThrowFrameMode의 Forward를 사용한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowNow();

	/** 현재 Wielder/Rope 설정으로 throw 순간의 origin/frame/속도 context를 만든다. AimDir은 legacy 호환용이며 내부에서는 무시한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	FRopeThrowContext BuildThrowContext(const FVector& AimDir) const;

	/** Legacy API. AimDir은 더 이상 주 방향이 아니며, 실제 방향은 ThrowFrameMode의 Forward를 사용한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowInDirection(const FVector& AimDir);

	/** ThrowMontage를 owner 메시의 AnimInstance에서 재생한다(설정돼 있을 때). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void PlayThrowMontage();

	/** 현재 wrap을 해제한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Release();

	/** wrap/contact 중이면 Release, 아니면 Throw. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ToggleThrow();

	/** 들고 있는 로프(없으면 null). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	URopeComponent* GetRope() const { return Rope; }

	/** 현재 AimSource 기준 조준 방향(정규화). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	FVector GetAimDirection() const;

	/** 입력을 수동으로 바인딩한다. 자동 바인딩이 타이밍상 실패하면(InputComponent 미준비) Pawn의
	 *  SetupPlayerInputComponent에서 호출하라. 이미 바인딩됐으면 무시. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Input")
	void BindInput();

private:
	void ResolveRefs();        // Rope/AttachMesh 해석(미설정 시 owner에서 탐색).
	void AttachRopeToSocket(); // Rope를 AttachMesh의 HandSocketName에 부착.
	void AddMappingContext();  // MappingContext를 로컬 플레이어 Enhanced Input 서브시스템에 추가.

	void OnThrowInput();
	void OnReleaseInput();

	bool bInputBound = false;
};
