// Copyright Epic Games, Inc. All Rights Reserved.
//
// 월드당 1개 스폰되는 로프 매니저 액터. 정적 월드 충돌용 URopeStaticBodyProvider를 품고 있어,
// URopeSimSubsystem이 게임/PIE 월드 시작 시 이 액터(또는 프로젝트 세팅이 지정한 서브클래스)를
// 자동 스폰한다 — 레벨마다 프로바이더 컴포넌트를 수동 배치할 필요 없이 "월드당 정확히 1개"를 보장한다.
// 프로젝트는 이 클래스를 서브클래스해 프로바이더의 IgnoredComponents를 조정할 수 있다
// (콜라이더 예산/평면 상한은 Project Settings > Dynamic Rope에서 전역 관리).

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeController.generated.h"

class URopeStaticBodyProvider;

UCLASS(ClassGroup = (DynamicRope))
class DYNAMICROPE_API ARopeController : public AActor
{
	GENERATED_BODY()

public:
	ARopeController();

	/** 정적 월드 심플 콜리전 → 로프 콜라이더 프로바이더. 디테일 패널/서브클래스에서 튜닝 가능. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Collision")
	TObjectPtr<URopeStaticBodyProvider> StaticBodyProvider;
};
