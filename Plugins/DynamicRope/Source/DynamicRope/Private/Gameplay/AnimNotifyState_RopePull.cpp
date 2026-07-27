// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/AnimNotifyState_RopePull.h"
#include "Gameplay/RopeWielderComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"

namespace
{
	// Resolved the same way as AnimNotify_RopeThrow: the wielder is found on the owner of the mesh that received the notify.
	URopeWielderComponent* ResolveWielder(USkeletalMeshComponent* MeshComp)
	{
		if (!MeshComp)
		{
			return nullptr;
		}
		AActor* Owner = MeshComp->GetOwner();
		return Owner ? Owner->FindComponentByClass<URopeWielderComponent>() : nullptr;
	}
}

void UAnimNotifyState_RopePull::NotifyBegin(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
	float TotalDuration, const FAnimNotifyEventReference& EventReference)
{
	Super::NotifyBegin(MeshComp, Animation, TotalDuration, EventReference);

	if (URopeWielderComponent* Wielder = ResolveWielder(MeshComp))
	{
		// Stops a NotifyBegin from a previous montage, arriving late after StopPull, from switching the pull back on.
		if (Wielder->IsPullArmed() && Wielder->IsPullEngaged())
		{
			Wielder->StartPullNow(bIgnoreTautGate);
		}
	}
}

void UAnimNotifyState_RopePull::NotifyEnd(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
	const FAnimNotifyEventReference& EventReference)
{
	// The engine calls the active state's NotifyEnd even when the montage is interrupted or blends out, so releasing
	// the window happens in this one place.
	// (On the path where the input is released, StopPull stops the montage, and that montage stopping comes back through here and turns the force off.)
	if (URopeWielderComponent* Wielder = ResolveWielder(MeshComp))
	{
	// A late NotifyEnd from a cancelled window does not touch the state StopPull has already finished with.
		if (Wielder->IsPullEngaged())
		{
			Wielder->StopPullNow();
		}
	}

	Super::NotifyEnd(MeshComp, Animation, EventReference);
}

FString UAnimNotifyState_RopePull::GetNotifyName_Implementation() const
{
	return bIgnoreTautGate ? TEXT("Rope Pull Window (Ignore Taut)") : TEXT("Rope Pull Window");
}
