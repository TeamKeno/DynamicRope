// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The pull window notify state placed in a pull montage. Opening the window arms active pull through
// StartPullNow and closing it releases it through StopPullNow, so this notify defines the time window
// and nothing more. Whether force is actually applied is decided by the rope's own per-frame gate,
// HoldConfig.bActivePullRequiresTaut and ActivePullTautTension, queried through IsPullTaut(), which is
// evaluated every frame inside the window; there is no need to duplicate that check here. Within the
// window a slack rope receives no force, and force engages the moment it goes taut.
// Instances belong to the animation asset and are shared across meshes, so no runtime state is kept
// in members.

#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimNotifies/AnimNotifyState.h"
#include "AnimNotifyState_RopePull.generated.h"

UCLASS(meta = (DisplayName = "Rope Pull Window"))
class DYNAMICROPE_API UAnimNotifyState_RopePull : public UAnimNotifyState
{
	GENERATED_BODY()

public:
	/**
	 * When true this window ignores tautness and always applies the pull, bypassing the rope's gate for
	 * this call through SetActivePull(Force, bIgnoreTautGate). Use it for scripted sequences where the
	 * target must be dragged regardless.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	bool bIgnoreTautGate = false;

	virtual void NotifyBegin(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
		float TotalDuration, const FAnimNotifyEventReference& EventReference) override;
	virtual void NotifyEnd(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
		const FAnimNotifyEventReference& EventReference) override;

	virtual FString GetNotifyName_Implementation() const override;
};
