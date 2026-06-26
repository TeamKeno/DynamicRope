// Copyright Epic Games, Inc. All Rights Reserved.
//
// 던지는 모션 몽타주에 배치하는 AnimNotify. 손에서 로프가 떠나는 프레임에 두면, 그 순간 메시 owner의
// URopeWielderComponent::ThrowNow()를 호출해 실제 로프 던지기를 발사한다(모션과 타이밍 동기).

#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimNotifies/AnimNotify.h"
#include "AnimNotify_RopeThrow.generated.h"

UCLASS(meta = (DisplayName = "Rope Throw"))
class DYNAMICROPE_API UAnimNotify_RopeThrow : public UAnimNotify
{
	GENERATED_BODY()

public:
	virtual void Notify(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
		const FAnimNotifyEventReference& EventReference) override;

	virtual FString GetNotifyName_Implementation() const override;
};
