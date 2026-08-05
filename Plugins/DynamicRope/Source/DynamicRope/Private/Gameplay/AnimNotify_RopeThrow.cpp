// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Gameplay/AnimNotify_RopeThrow.h"
#include "Gameplay/RopeWielderComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"

void UAnimNotify_RopeThrow::Notify(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
	const FAnimNotifyEventReference& EventReference)
{
	Super::Notify(MeshComp, Animation, EventReference);

	if (!MeshComp)
	{
		return;
	}
	if (AActor* Owner = MeshComp->GetOwner())
	{
		if (URopeWielderComponent* Wielder = Owner->FindComponentByClass<URopeWielderComponent>())
		{
			Wielder->ThrowNow();
		}
	}
}

FString UAnimNotify_RopeThrow::GetNotifyName_Implementation() const
{
	return TEXT("Rope Throw");
}
