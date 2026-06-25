// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/GameplayDebuggerCategory_Rope.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "RopeComponent.h"
#include "GameFramework/Actor.h"

FGameplayDebuggerCategory_Rope::FGameplayDebuggerCategory_Rope()
{
	bShowOnlyWithDebugActor = false;
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_Rope::MakeInstance()
{
	return MakeShareable(new FGameplayDebuggerCategory_Rope());
}

void FGameplayDebuggerCategory_Rope::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
	if (!DebugActor)
	{
		AddTextLine(TEXT("{grey}no debug actor"));
		return;
	}

	int32 Count = 0;
	for (UActorComponent* Comp : DebugActor->GetComponents())
	{
		const URopeComponent* Rope = Cast<URopeComponent>(Comp);
		if (!Rope)
		{
			continue;
		}
		++Count;

		const FName Bone = Rope->GetWrappedBoneName();
		const TArray<FVector>& Points = Rope->GetCenterlinePositions();
		AddTextLine(FString::Printf(TEXT("{yellow}Rope #%d{white} phase=%d nodes=%d wrapBone=%s"),
			Count, static_cast<int32>(Rope->GetPhase()), Points.Num(),
			Bone.IsNone() ? TEXT("-") : *Bone.ToString()));

		// 중심선을 월드 공간 세그먼트로 그린다.
		for (int32 i = 0; i + 1 < Points.Num(); ++i)
		{
			AddShape(FGameplayDebuggerShape::MakeSegment(Points[i], Points[i + 1], 1.0f, FColor::Cyan));
		}
	}

	if (Count == 0)
	{
		AddTextLine(TEXT("{grey}no URopeComponent on debug actor"));
	}
}

#endif // WITH_GAMEPLAY_DEBUGGER
