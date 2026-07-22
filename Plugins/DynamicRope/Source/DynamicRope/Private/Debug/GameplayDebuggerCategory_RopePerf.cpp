// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/GameplayDebuggerCategory_RopePerf.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "RopeComponent.h"
#include "Core/RopeLifecycleTypes.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"
#include "Camera/PlayerCameraManager.h"
// 등록된 로프 목록(GetRegisteredRopes) — 이 화면의 단일 소스
#include "Subsystem/RopeSimSubsystem.h"

namespace
{
	const TCHAR* PerfPhaseName(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Free:        return TEXT("Free");
		case ERopePhase::Flight:      return TEXT("Flight");
		case ERopePhase::Contacting:  return TEXT("Contacting");
		case ERopePhase::Wrapping:    return TEXT("Wrapping");
		case ERopePhase::Wrapped:     return TEXT("Wrapped");
		case ERopePhase::GuidedThrow: return TEXT("GuidedThrow");
		case ERopePhase::Releasing:   return TEXT("Releasing");
		case ERopePhase::Loaded:        return TEXT("Loaded");
		default:                      return TEXT("?");
		}
	}

	// per-rope 한 줄에 필요한 라이브 스냅(정렬용으로 미리 모은다).
	struct FPerfRow
	{
		ERopePhase Phase = ERopePhase::Free;
		int32 Nodes = 0;
		bool bGpuStepped = false;   // 이번 프레임 GPU 상주 step
		bool bCpuSolved = false;    // 솔브했지만 GPU 아님 = CPU 폴백
		bool bSleeping = false;     // Free 정지 슬립
		bool bGdf = false;          // bUseWorldGDF 설정
		float LodScale = 1.0f;
		float Distance = -1.0f;     // 카메라→앵커(cm), 카메라 없으면 -1
		FVector Anchor = FVector::ZeroVector;
		// centerline이 아직 없으면(시드 전/노드 0) Anchor는 의미 없는 0벡터다. 그대로 마커를 그리면
		// 월드 원점에 있지도 않은 로프의 #n 라벨이 찍혀(여럿이면 겹쳐 쌓여) 엉뚱한 곳을 찾게 만든다.
		bool bHasAnchor = false;
	};
}

FGameplayDebuggerCategory_RopePerf::FGameplayDebuggerCategory_RopePerf()
{
	// 디버그 액터 없이도 월드 전역을 그린다.
	bShowOnlyWithDebugActor = false;
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_RopePerf::MakeInstance()
{
	return MakeShareable(new FGameplayDebuggerCategory_RopePerf());
}

void FGameplayDebuggerCategory_RopePerf::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
	const UWorld* World = OwnerPC ? OwnerPC->GetWorld() : (DebugActor ? DebugActor->GetWorld() : nullptr);
	if (!World)
	{
		AddTextLine(TEXT("{grey}no world"));
		return;
	}

	// 카메라 위치(거리 산출용): 카메라 매니저 우선, 없으면 폰. 둘 다 없으면 거리 생략.
	bool bHasCam = false;
	FVector CamLoc = FVector::ZeroVector;
	if (OwnerPC)
	{
		if (const APlayerCameraManager* CM = OwnerPC->PlayerCameraManager)
		{
			CamLoc = CM->GetCameraLocation();
			bHasCam = true;
		}
		else if (const APawn* Pawn = OwnerPC->GetPawn())
		{
			CamLoc = Pawn->GetActorLocation();
			bHasCam = true;
		}
	}

	// 월드의 활성 로프를 훑어 라이브 값 수집(서브시스템 private 목록 대신 월드 순회 — 등록 여부로 필터).
	TArray<FPerfRow> Rows;
	int32 NumGpu = 0, NumCpu = 0, NumSleeping = 0, NumGdf = 0;
	int64 TotalParticles = 0;
	// 서브시스템의 등록 목록을 읽는다 — 이 화면이 세는 것은 "월드에 있는 로프"가 아니라 **이 서브시스템이
	// 실제로 구동하는 로프**다(등록 안 된 로프는 틱되지 않아 비용도 0이라 성능 화면의 분모로 맞지 않다).
	// 전체 UObject 스캔을 쓰면 로프 수와 무관하게 비싸고, 성능을 재려고 켠 화면이 스스로 프레임을
	// 무겁게 만들어 측정 대상을 왜곡한다.
	const URopeSimSubsystem* SimSub = URopeSimSubsystem::Get(World);
	if (!SimSub)
	{
		AddTextLine(TEXT("{grey}no rope sim subsystem"));
		return;
	}
	for (const TObjectPtr<URopeComponent>& RopePtr : SimSub->GetRegisteredRopes())
	{
		// 파괴 후 GC 대기 중인 항목이 섞일 수 있다(무효 정리는 Tick 프레임 경계에서만 돈다).
		const URopeComponent* Rope = RopePtr.Get();
		if (!IsValid(Rope))
		{
			continue;
		}

		FPerfRow Row;
		Row.Phase = Rope->GetPhase();
		Row.Nodes = Rope->GetNodeCount();
		Row.bGpuStepped = Rope->IsGpuSteppedThisFrame();
		Row.bCpuSolved = !Row.bGpuStepped && Rope->WasSolvedThisFrame();
		Row.bSleeping = Rope->IsSleeping();
		Row.bGdf = Rope->bUseWorldGDF;
		Row.LodScale = Rope->GetSolverLODScale();

		const TArray<FVector>& Points = Rope->GetCenterlinePositions();
		if (Points.Num() > 0)
		{
			Row.Anchor = Points[0];
			Row.bHasAnchor = true;
			if (bHasCam)
			{
				Row.Distance = static_cast<float>(FVector::Dist(CamLoc, Row.Anchor));
			}
		}

		TotalParticles += Row.Nodes;
		if (Row.bGpuStepped) { ++NumGpu; }
		else if (Row.bCpuSolved) { ++NumCpu; }
		if (Row.bSleeping) { ++NumSleeping; }
		if (Row.bGdf) { ++NumGdf; }
		Rows.Add(Row);
	}

	const int32 NumRopes = Rows.Num();
	const int32 NumIdle = FMath::Max(0, NumRopes - NumGpu - NumCpu);

	// 상단 집계. registered = 서브시스템에 등록돼 이번 프레임 구동된 로프 수이지 월드에 배치된 수가 아니다
	// — 등록 전(스폰 직후 BeginPlay 전)이거나 등록에 실패한 로프는 틱되지 않으므로 여기 없다.
	// 그 구성(gpu + cpu + idle)은 ‘stat DynamicRope’의 Active와 같은 정의다.
	AddTextLine(FString::Printf(
		TEXT("{white}Rope Perf (world){grey}  registered=%d  {green}gpu=%d {red}cpu=%d {grey}idle=%d  {cyan}sleeping=%d"),
		NumRopes, NumGpu, NumCpu, NumIdle, NumSleeping));
	AddTextLine(FString::Printf(
		TEXT("{grey}particles=%lld  gdf-enabled=%d%s"),
		static_cast<long long>(TotalParticles), NumGdf,
		bHasCam ? TEXT("") : TEXT("  (no camera — distances hidden)")));

	if (NumRopes == 0)
	{
		AddTextLine(TEXT("{grey}no registered ropes in world"));
		return;
	}

	// 정렬: 솔브 중(비용 있는) 로프를 위로, 그 안에서 노드 수 내림차순 — 상한을 넘겨 잘려도 비싼 로프가 남는다.
	Rows.Sort([](const FPerfRow& A, const FPerfRow& B)
	{
		const bool bActiveA = A.bGpuStepped || A.bCpuSolved;
		const bool bActiveB = B.bGpuStepped || B.bCpuSolved;
		if (bActiveA != bActiveB)
		{
			return bActiveA;
		}
		return A.Nodes > B.Nodes;
	});

	// per-rope 한 줄 + 월드에서 로프 위치를 찾도록 앵커에 phase색 점 + '#i' 라벨(리스트↔월드 상관).
	const int32 MaxRows = FMath::Min(40, Rows.Num());
	for (int32 i = 0; i < MaxRows; ++i)
	{
		const FPerfRow& R = Rows[i];

		// 솔브 경로 토큰: 슬립이 우선(잠들면 솔브 자체가 없다), 그 다음 gpu/cpu/idle.
		const TCHAR* Solve = R.bSleeping ? TEXT("{cyan}SLEEP")
			: (R.bGpuStepped ? TEXT("{green}gpu")
			: (R.bCpuSolved ? TEXT("{red}cpu") : TEXT("{grey}idle")));

		const FString Lod = (R.LodScale < 0.999f)
			? FString::Printf(TEXT(" {cyan}lod=x%.2f"), R.LodScale) : FString();
		const FString Dist = (R.Distance >= 0.0f)
			? FString::Printf(TEXT(" {grey}d=%.0f"), R.Distance) : FString();

		AddTextLine(FString::Printf(
			TEXT("{yellow}#%d {white}%s{white} n=%d %s{white}%s %s%s"),
			i + 1, PerfPhaseName(R.Phase), R.Nodes, Solve, *Lod,
			R.bGdf ? TEXT("{green}gdf") : TEXT("{grey}gdf-off"), *Dist));

		// 앵커 마커(월드↔리스트 상관용). 슬립=cyan, 솔브 중=흰, idle=회색.
		// centerline이 없는 로프는 찍을 위치가 없다 — 0벡터로 그리면 월드 원점에 유령 마커가 생긴다.
		if (R.bHasAnchor)
		{
			const FColor MarkerColor = R.bSleeping ? FColor::Cyan
				: ((R.bGpuStepped || R.bCpuSolved) ? FColor::White : FColor(140, 140, 140));
			AddShape(FGameplayDebuggerShape::MakePoint(R.Anchor, 6.0f, MarkerColor,
				FString::Printf(TEXT("#%d"), i + 1)));
		}
	}
	if (Rows.Num() > MaxRows)
	{
		AddTextLine(FString::Printf(TEXT("{grey}... %d more"), Rows.Num() - MaxRows));
	}
}

#endif // WITH_GAMEPLAY_DEBUGGER
