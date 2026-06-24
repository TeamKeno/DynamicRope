// Fill out your copyright notice in the Description page of Project Settings.
//
// 정지한 로프 소유 액터 (Tier 1 repro). 동적 AI 타깃에게 로프를 던져 감고, wrap 이벤트로 타깃의
// 행동을 제한한다. 키 입력으로 Throw / DebugForceWrap / Release 제어.
//   T = 타깃 향해 던지기 · F = 강제 wrap(게이트 우회) · R = release

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Core/RopeTypes.h"
#include "RopeDemoAnchor.generated.h"

class URopeComponent;
class ARopeTetherTarget;

UCLASS()
class DYNAMICROPEPROJECT_API ARopeDemoAnchor : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoAnchor();

	/** 감을 동적 AI 타깃(레벨에서 지정). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<ARopeTetherTarget> Target;

	/** 로프 컴포넌트(센터라인 시뮬 + 튜브 렌더). 이 액터의 루트. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<URopeComponent> Rope;

protected:
	virtual void BeginPlay() override;

private:
	UFUNCTION() void HandleWrapped(FName Bone);
	UFUNCTION() void HandleReleased(FName Bone, ERopeReleaseReason Reason);

	void DoThrow();
	void DoForceWrap();
	void DoRelease();
};
