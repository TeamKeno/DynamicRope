// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeDebugSubsystem.h"
#include "RopeComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"

URopeDebugSubsystem* URopeDebugSubsystem::Get(const UWorld* World)
{
	return World ? World->GetSubsystem<URopeDebugSubsystem>() : nullptr;
}

bool URopeDebugSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	// sim 서브시스템과 동일하게 게임/PIE에서만(에디터 프리뷰/인스펙터 월드 제외).
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

#if WITH_GAMEPLAY_DEBUGGER

void URopeDebugSubsystem::SetTarget(AActor* InActor)
{
	TargetActor = InActor;
	LastActiveFrame = GFrameCounter;
}

bool URopeDebugSubsystem::ShouldCapture(const URopeComponent* Rope) const
{
	if (!Rope)
	{
		return false;
	}

	// 카테고리가 최근(ActiveFrameWindow 내) 그렸는가 = 디버거/카테고리 활성.
	if (GFrameCounter - LastActiveFrame > ActiveFrameWindow)
	{
		return false;
	}

	const AActor* Target = TargetActor.Get();
	return Target != nullptr && Rope->GetOwner() == Target;
}

void URopeDebugSubsystem::SubmitSnapshot(const URopeComponent* Rope, FRopeDebugSnapshot&& Snapshot)
{
	if (!Rope)
	{
		return;
	}
	Snapshot.FrameStamp = GFrameCounter;
	Snapshots.Add(Rope, MoveTemp(Snapshot));

	// 죽었거나 오래된 항목 정리(서브시스템이 tick하지 않으므로 제출 시 기회적으로 청소).
	for (auto It = Snapshots.CreateIterator(); It; ++It)
	{
		if (!It.Key().IsValid() || GFrameCounter - It.Value().FrameStamp > ActiveFrameWindow)
		{
			It.RemoveCurrent();
		}
	}
}

const FRopeDebugSnapshot* URopeDebugSubsystem::GetSnapshot(const URopeComponent* Rope) const
{
	const FRopeDebugSnapshot* Found = Snapshots.Find(Rope);
	if (!Found || GFrameCounter - Found->FrameStamp > ActiveFrameWindow)
	{
		return nullptr;
	}
	return Found;
}

#else // !WITH_GAMEPLAY_DEBUGGER — 디버그 비활성 빌드: 모두 no-op.

void URopeDebugSubsystem::SetTarget(AActor*) {}
bool URopeDebugSubsystem::ShouldCapture(const URopeComponent*) const { return false; }
void URopeDebugSubsystem::SubmitSnapshot(const URopeComponent*, FRopeDebugSnapshot&&) {}
const FRopeDebugSnapshot* URopeDebugSubsystem::GetSnapshot(const URopeComponent*) const { return nullptr; }

#endif // WITH_GAMEPLAY_DEBUGGER
