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
	if (Snapshots.Num() == 0 && HeldFlight.Num() == 0)
	{
		return;
	}

	// 카테고리가 최근(ActiveFrameWindow 내) 그리지 않았으면 비활성 — 제출이 멈춰도 마지막 배열이 월드
	// 종료까지 남지 않도록 보관분을 통째로 비운다(hold 보관소 포함).
	if (GFrameCounter - LastActiveFrame > ActiveFrameWindow)
	{
		Snapshots.Reset();
		HeldFlight.Reset();
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

	// hold 보관소는 실시간 창(FlightHoldSeconds) 기준으로 만료 — 무효 키도 함께 제거.
	if (HeldFlight.Num() > 0)
	{
		const UWorld* World = GetWorld();
		const double Now = World ? World->GetRealTimeSeconds() : 0.0;
		for (auto It = HeldFlight.CreateIterator(); It; ++It)
		{
			if (!It.Key().IsValid() || Now - It.Value().RealTimeSeconds > FlightHoldSeconds)
			{
				It.RemoveCurrent();
			}
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
	// flight 오버레이는 다음(Wrapping) 프레임 스냅샷에 덮여 사라지므로, bHasFlight면 별도로 복사 보관해
	// 실시간 FlightHoldSeconds 동안 잔류시킨다("무엇을 잡기로 했나"를 결정 직후에도 보게). 아래 MoveTemp가
	// 원본을 소비하므로 그 전에 복사한다.
	if (Snapshot.bHasFlight)
	{
		FHeldFlightSnapshot& Held = HeldFlight.FindOrAdd(Rope);
		Held.Snapshot = Snapshot;
		const UWorld* World = GetWorld();
		Held.RealTimeSeconds = World ? World->GetRealTimeSeconds() : 0.0;
	}
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

const FRopeDebugSnapshot* URopeDebugSubsystem::GetHeldFlightSnapshot(const URopeComponent* Rope, float& OutAgeSeconds) const
{
	OutAgeSeconds = 0.0f;
	const FHeldFlightSnapshot* Found = HeldFlight.Find(Rope);
	if (!Found)
	{
		return nullptr;
	}
	const UWorld* World = GetWorld();
	const double Now = World ? World->GetRealTimeSeconds() : 0.0;
	const double Age = Now - Found->RealTimeSeconds;
	if (Age < 0.0 || Age > FlightHoldSeconds)
	{
		return nullptr;
	}
	OutAgeSeconds = static_cast<float>(Age);
	return &Found->Snapshot;
}

#else // !WITH_GAMEPLAY_DEBUGGER — 디버그 비활성 빌드: 모두 no-op.

void URopeDebugSubsystem::SetTarget(AActor*, ERopeDebugCapture) {}
bool URopeDebugSubsystem::ShouldCapture(const URopeComponent*) const { return false; }
ERopeDebugCapture URopeDebugSubsystem::GetCaptureMask() const { return ERopeDebugCapture::None; }
void URopeDebugSubsystem::SubmitSnapshot(const URopeComponent*, FRopeDebugSnapshot&&) {}
const FRopeDebugSnapshot* URopeDebugSubsystem::GetSnapshot(const URopeComponent*) const { return nullptr; }
const FRopeDebugSnapshot* URopeDebugSubsystem::GetHeldFlightSnapshot(const URopeComponent*, float& OutAgeSeconds) const { OutAgeSeconds = 0.0f; return nullptr; }

#endif // WITH_GAMEPLAY_DEBUGGER
