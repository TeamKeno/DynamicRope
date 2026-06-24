// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "DynamicRopeSettings.generated.h"

/**
 * Dynamic Rope 플러그인의 프로젝트 전역 설정.
 * Project Settings > Plugins > Dynamic Rope에서 편집 가능하며 DefaultGame.ini에 저장된다.
 */
UCLASS(config = Game, defaultconfig, meta = (DisplayName = "Dynamic Rope"))
class DYNAMICROPE_API UDynamicRopeSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UDynamicRopeSettings();

	/** 활성 (CDO) 설정 객체에 접근하기 위한 편의 접근자. */
	static const UDynamicRopeSettings* Get();

	//~ Rope ---------------------------------------------------------------

	/** 새로 스폰된 직선 로프의 기본 휴지 길이(rest length). */
	UPROPERTY(EditAnywhere, config, Category = "Rope", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float DefaultRopeLength = 200.0f;

	/** 로프를 따라 적용되는 기본 장력. 값이 클수록 감겨 있는 동안 로프가 더 팽팽하게 유지된다. */
	UPROPERTY(EditAnywhere, config, Category = "Rope", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float DefaultTension = 1.0f;

	//~ Wrapping -----------------------------------------------------------

	/** 추가 감기가 차단되기 전에 로프가 단일 신체 부위를 감을 수 있는 최대 회전 수. */
	UPROPERTY(EditAnywhere, config, Category = "Wrapping", meta = (ClampMin = "1", UIMin = "1"))
	int32 MaxWrapTurnsPerBodyPart = 3;

	/** 로프가 감을 수 있도록 허용된 스켈레탈 메시 본(팔, 다리, 몸통, ...). */
	UPROPERTY(EditAnywhere, config, Category = "Wrapping")
	TArray<FName> WrappableBones;

	/** 캐릭터가 멀어질 때 감긴 로프가 자동으로 풀리는 거리. */
	UPROPERTY(EditAnywhere, config, Category = "Wrapping", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float AutoUnwrapDistance = 500.0f;

	//~ Debug --------------------------------------------------------------

	/** 게임 내에서 접촉점과 감김 상태의 디버그 시각화를 그린다. */
	UPROPERTY(EditAnywhere, config, Category = "Debug")
	bool bEnableDebugDraw = false;
};
