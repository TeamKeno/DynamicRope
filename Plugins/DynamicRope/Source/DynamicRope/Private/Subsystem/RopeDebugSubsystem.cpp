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

void URopeDebugSubsystem::Tick(float DeltaTime)
{
#if WITH_GAMEPLAY_DEBUGGER
	if (Snapshots.Num() == 0)
	{
		return;
	}

	// 카테고리가 최근(ActiveFrameWindow 내) 그리지 않았으면 비활성 — 제출이 멈춰도 마지막 배열이 월드
	// 종료까지 남지 않도록 보관분을 통째로 비운다.
	if (GFrameCounter - LastActiveFrame > ActiveFrameWindow)
	{
		Snapshots.Reset();
		return;
	}

	// 활성 중 스테일/무효 정리는 여기 한 곳에서 프레임당 한 번만 — SubmitSnapshot마다 전체 맵을 훑던
	// O(제출수×로프수)를 없앤다.
	for (auto It = Snapshots.CreateIterator(); It; ++It)
	{
		if (!It.Key().IsValid() || GFrameCounter - It.Value().FrameStamp > ActiveFrameWindow)
		{
			It.RemoveCurrent();
		}
	}
#endif
}

TStatId URopeDebugSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URopeDebugSubsystem, STATGROUP_Tickables);
}

bool URopeDebugSubsystem::IsTickable() const
{
	// 디버그 전용 수명 관리 — WITH_GAMEPLAY_DEBUGGER가 꺼진 빌드에선 틱할 게 없다.
#if WITH_GAMEPLAY_DEBUGGER
	return true;
#else
	return false;
#endif
}

#if WITH_GAMEPLAY_DEBUGGER

void URopeDebugSubsystem::SetTarget(AActor* InActor, ERopeDebugCapture InCaptureMask)
{
	TargetActor = InActor;
	CaptureMask = InCaptureMask;
	LastActiveFrame = GFrameCounter;
}

ERopeDebugCapture URopeDebugSubsystem::GetCaptureMask() const
{
	return CaptureMask;
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
	// 스테일/무효 정리는 Tick이 프레임당 한 번 돈다 — 여기서 매 제출마다 전체 맵을 훑지 않는다.
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

void URopeDebugSubsystem::SetTarget(AActor*, ERopeDebugCapture) {}
bool URopeDebugSubsystem::ShouldCapture(const URopeComponent*) const { return false; }
ERopeDebugCapture URopeDebugSubsystem::GetCaptureMask() const { return ERopeDebugCapture::None; }
void URopeDebugSubsystem::SubmitSnapshot(const URopeComponent*, FRopeDebugSnapshot&&) {}
const FRopeDebugSnapshot* URopeDebugSubsystem::GetSnapshot(const URopeComponent*) const { return nullptr; }

#endif // WITH_GAMEPLAY_DEBUGGER
