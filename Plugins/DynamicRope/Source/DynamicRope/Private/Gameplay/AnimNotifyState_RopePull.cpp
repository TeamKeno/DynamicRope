// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/AnimNotifyState_RopePull.h"
#include "Gameplay/RopeWielderComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"

namespace
{
	// AnimNotify_RopeThrow와 같은 해석: notify를 받은 메시의 owner에서 wielder를 찾는다.
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
		Wielder->StartPullNow(bIgnoreTautGate);
	}
}

void UAnimNotifyState_RopePull::NotifyEnd(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
	const FAnimNotifyEventReference& EventReference)
{
	// 몽타주 인터럽트/블렌드아웃에도 엔진이 활성 state의 NotifyEnd를 호출한다 — 창의 해제는 이 한 곳이다.
	// (입력을 뗀 경로는 StopPull이 몽타주를 멈추고, 그 몽타주 정지가 다시 여기로 들어와 힘을 끈다.)
	if (URopeWielderComponent* Wielder = ResolveWielder(MeshComp))
	{
		Wielder->StopPullNow();
	}

	Super::NotifyEnd(MeshComp, Animation, EventReference);
}

FString UAnimNotifyState_RopePull::GetNotifyName_Implementation() const
{
	return bIgnoreTautGate ? TEXT("Rope Pull Window (Ignore Taut)") : TEXT("Rope Pull Window");
}
