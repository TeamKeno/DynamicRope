// Copyright Epic Games, Inc. All Rights Reserved.
//
// Pull 몽타주에 배치하는 "pull window" NotifyState. 구간이 열리면 능동 Pull을 장전(StartPullNow)하고
// 닫히면 해제(StopPullNow)한다 — 이 notify는 **시간 창만** 정의한다. "팽팽할 때만 실제 인가"는 로프의
// 프레임 게이트(HoldConfig.bActivePullRequiresTaut/ActivePullTautTension, 조회는 IsPullTaut())가 창 안에서
// 매 프레임 판정하므로 여기서 tick 검사를 중복하지 않는다: 창 안이라도 로프가 늘어져 있으면 힘이 안 걸리고,
// 팽팽해지는 순간 걸린다. 인스턴스는 애니 에셋 소유(여러 메시가 공유)라 런타임 상태를 멤버에 두지 않는다.

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
	 * true면 이 창에서는 팽팽함을 무시하고 항상 pull을 인가한다(로프 게이트 per-call 우회 —
	 * SetActivePull(Force, bIgnoreTautGate)). 확정 연출(무조건 끌려와야 하는 컷) 구간용.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	bool bIgnoreTautGate = false;

	virtual void NotifyBegin(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
		float TotalDuration, const FAnimNotifyEventReference& EventReference) override;
	virtual void NotifyEnd(USkeletalMeshComponent* MeshComp, UAnimSequenceBase* Animation,
		const FAnimNotifyEventReference& EventReference) override;

	virtual FString GetNotifyName_Implementation() const override;
};
