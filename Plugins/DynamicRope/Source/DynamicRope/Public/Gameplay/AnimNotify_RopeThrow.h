// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The anim notify placed in a throwing montage. Put it on the frame the rope leaves the hand and it
// calls URopeWielderComponent::ThrowNow() on the mesh's owner at that instant, which fires the actual
// throw in sync with the motion.

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
