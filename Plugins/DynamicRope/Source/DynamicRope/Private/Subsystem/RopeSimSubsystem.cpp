// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeSimSubsystem.h"
#include "RopeComponent.h"
#include "Engine/World.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

void URopeSimSubsystem::RegisterRope(URopeComponent* Rope)
{
	if (Rope)
	{
		Ropes.AddUnique(Rope);
	}
}

void URopeSimSubsystem::UnregisterRope(URopeComponent* Rope)
{
	Ropes.RemoveSingleSwap(Rope);
}

URopeSimSubsystem* URopeSimSubsystem::Get(const UWorld* World)
{
	return World ? World->GetSubsystem<URopeSimSubsystem>() : nullptr;
}

void URopeSimSubsystem::Tick(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SubsystemTick);

	// 단일 오케스트레이션 지점. 현재는 순차 구동.
	// TODO(Tier2): collider gather를 소스 메시별로 1회 디둡 → ParallelFor로 로프별 Solver.Step 병렬화
	//              (Query는 const, collider는 스냅샷이라 스레드 안전. WrapController::Hold의 본 트랜스폼만
	//              GT에서 스냅샷 필요).
	// TODO: LOD/sleep(멀거나 안정된 로프 스킵), 프레임당 총 솔브 비용 상한.
	// TODO: tick 순서 — 충돌은 애니메이션(본 트랜스폼) 이후가 필요. 현재는 FTickableGameObject 타이밍에
	//       의존하므로, 정밀 정렬이 필요하면 TG_PostPhysics tick function으로 전환.
	for (int32 i = Ropes.Num() - 1; i >= 0; --i)
	{
		URopeComponent* Rope = Ropes[i];
		if (!IsValid(Rope))
		{
			Ropes.RemoveAtSwap(i);
			continue;
		}
		Rope->SimulateFrame(DeltaTime);
	}
}

TStatId URopeSimSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URopeSimSubsystem, STATGROUP_Tickables);
}

bool URopeSimSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	// 게임/PIE에서만 시뮬레이션(에디터 프리뷰/인스펙터 월드 제외 → 컴포넌트도 그때만 BeginPlay 등록).
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}
