// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 런타임 인트로스펙션을 위한 Gameplay Debugger 카테고리. 인게임에서 디버거를 켜고 'Rope'
// 카테고리를 토글하면 디버그 액터의 URopeComponent들에 대해 phase/node/wrap bone과 중심선을 표시한다.
// WITH_GAMEPLAY_DEBUGGER 가 꺼진 빌드(shipping 등)에서는 전체가 컴파일에서 제외된다.

#pragma once

#include "CoreMinimal.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "GameplayDebuggerCategory.h"

class APlayerController;
class AActor;

class FGameplayDebuggerCategory_Rope : public FGameplayDebuggerCategory
{
public:
	FGameplayDebuggerCategory_Rope();

	virtual void CollectData(APlayerController* OwnerPC, AActor* DebugActor) override;

	static TSharedRef<FGameplayDebuggerCategory> MakeInstance();
};

#endif // WITH_GAMEPLAY_DEBUGGER
