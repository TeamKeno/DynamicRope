// Fill out your copyright notice in the Description page of Project Settings.
//
// 로프가 감겨 행동이 제한되는 동적 AI 타깃 (Tier 1 repro). 본 캡슐 provider를 들고 있어 로프의
// 충돌/감김 소스가 되고, 테더되면 이동을 멈춘다. cross-actor wrap의 "움직이는 타깃" 역할.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "RopeTetherTarget.generated.h"

class URopeBoneCapsuleProvider;

UCLASS()
class DYNAMICROPEPROJECT_API ARopeTetherTarget : public ACharacter
{
	GENERATED_BODY()

public:
	ARopeTetherTarget();

	/** 로프 wrap/release 시 호출 — AI 이동을 멈추거나 재개(행동 제한). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void SetTethered(bool bInTethered);

	UFUNCTION(BlueprintPure, Category = "Rope|Demo")
	bool IsTethered() const { return bTethered; }

	/** 이 캐릭터 메시에서 본 캡슐을 만들어 로프에 공급하는 provider. SkeletalMesh는 owner에서 자동 해석. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<URopeBoneCapsuleProvider> RopeColliders;

private:
	bool  bTethered = false;
	float CachedMaxWalkSpeed = 0.0f;
};
