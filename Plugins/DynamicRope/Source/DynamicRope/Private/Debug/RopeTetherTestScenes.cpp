// Copyright Epic Games, Inc. All Rights Reserved.
//
// 테더 재현 씬 리그(개발 전용) — Docs/PoC/05 §8. 장력 3증상(근접 랙돌 wrap 폭주 / 랙돌 끌기 탄성 /
// 벽 탈출 윈치)을 어느 맵에서든 한 커맨드로 같은 배치로 재현한다. 고정 .umap 대신 코드 스폰인 이유:
// 리뷰/머지 가능한 텍스트고, 플레이어 기준 상대 배치라 각자의 테스트 맵(Lvl_*Test)에서 그대로 돈다.
// 엔진 기본 셰이프만 쓴다(게임 콘텐츠 무의존) — 랙돌 씬은 맵에 이미 배치된 랙돌 캐릭터를 끌어온다.
// (레거시 테더 모드 A/B 전환 커맨드(Rope.Test.TetherMode)는 모드 제거와 함께 삭제 — Docs/PoC/05 §6 F.)

#include "CoreMinimal.h"

#if !UE_BUILD_SHIPPING

#include "Components/StaticMeshComponent.h"
#include "DynamicRopeLog.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Gameplay/RopeRagdollResponseComponent.h"
#include "HAL/IConsoleManager.h"
#include "RopeComponent.h"
#include "UObject/UObjectIterator.h"

namespace RopeTetherTestScenes
{
	static APawn* GetLocalPawn(UWorld* World)
	{
		APlayerController* PC = World ? World->GetFirstPlayerController() : nullptr;
		APawn* Pawn = PC ? PC->GetPawn() : nullptr;
		if (!Pawn)
		{
			UE_LOG(LogDynamicRope, Warning, TEXT("Rope.Test.TetherScene: 로컬 플레이어 폰이 없다 — PIE에서 실행할 것."));
		}
		return Pawn;
	}

	// 플레이어 전방(수평) DistCm + 수직 ZOffsetCm. 씬 배치가 전부 이 기준이라 어느 맵에서든 동일 기하가 된다.
	static FVector AheadOfPawn(const APawn& Pawn, float DistCm, float ZOffsetCm)
	{
		const FVector Fwd = Pawn.GetActorForwardVector().GetSafeNormal2D();
		return Pawn.GetActorLocation() + Fwd * DistCm + FVector(0.0f, 0.0f, ZOffsetCm);
	}

	static AStaticMeshActor* SpawnShape(UWorld* World, const TCHAR* MeshPath,
		const FVector& Location, const FVector& Scale)
	{
		UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, MeshPath);
		if (!Mesh)
		{
			UE_LOG(LogDynamicRope, Error, TEXT("Rope.Test.TetherScene: 엔진 기본 셰이프 로드 실패(%s)."), MeshPath);
			return nullptr;
		}
		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AStaticMeshActor* Actor = World->SpawnActor<AStaticMeshActor>(Location, FRotator::ZeroRotator, Params);
		if (!Actor)
		{
			return nullptr;
		}
		UStaticMeshComponent* Comp = Actor->GetStaticMeshComponent();
		// Movable: 런타임 스폰/SetStaticMesh 허용. 비시뮬 비캐릭터라 테더 수신자 해석은 앵커(무한질량)로
		// Static과 동일하고, 콜라이더 수집도 world-dynamic 포함이 기본이라 잡힌다.
		Comp->SetMobility(EComponentMobility::Movable);
		Comp->SetStaticMesh(Mesh);
		Comp->SetCollisionProfileName(TEXT("BlockAll"));
		Actor->SetActorScale3D(Scale);
		return Actor;
	}

	// ① 벽/기둥 씬(윈치 증상): 전방 4m에 지름 60cm 기둥. 감고 반대로 걸어 나가본다.
	//    레거시(MassShare): 정적 앵커 = wielder 몫 100% × 상시 리엘 400cm/s → 벽 쪽으로 끌려감(윈치).
	//    Constraint: λ가 바깥 성분만 상쇄 + 경계 유지 — 로프 끝에서 멈추되 끌려가지 않아야 한다.
	static void SceneWall(UWorld* World)
	{
		APawn* Pawn = GetLocalPawn(World);
		if (!Pawn)
		{
			return;
		}
		const FVector Base = AheadOfPawn(*Pawn, 400.0f, 0.0f);
		if (SpawnShape(World, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"), Base + FVector(0, 0, 175.0f),
			FVector(0.6f, 0.6f, 3.5f)))
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[TetherScene] wall: 전방 4m 기둥(지름 60cm). 절차 = 기둥에 감기 → 뒤로 걷기/점프 탈출."));
			UE_LOG(LogDynamicRope, Log, TEXT("[TetherScene] 기대: 로프 끝에서 정지(윈치 끌림 없음). 디버거 상세 줄의 tether T 관찰."));
		}
	}

	// ② 드래그 씬(탄성/분배 증상, 컴포넌트 바디 경로): 전방 6m에 물리 큐브(기본 100kg, 인자로 변경).
	//    감고 걸어서/되감기로 끌어본다. Constraint: 질량비 분배가 자동 — 무거운 큐브(500)는 wielder가 양보.
	static void SceneDrag(UWorld* World, float MassKg)
	{
		APawn* Pawn = GetLocalPawn(World);
		if (!Pawn)
		{
			return;
		}
		AStaticMeshActor* Actor = SpawnShape(World, TEXT("/Engine/BasicShapes/Cube.Cube"),
			AheadOfPawn(*Pawn, 600.0f, 60.0f), FVector(1.0f));
		if (Actor)
		{
			UStaticMeshComponent* Comp = Actor->GetStaticMeshComponent();
			Comp->SetSimulatePhysics(true);
			Comp->SetMassOverrideInKg(NAME_None, FMath::Max(MassKg, 1.0f), true);
			UE_LOG(LogDynamicRope, Log, TEXT("[TetherScene] drag: 전방 6m 물리 큐브 %.0fkg. 절차 = 감기 → 뒤로 걷기·되감기(Reel)로 끌기."), MassKg);
			UE_LOG(LogDynamicRope, Log, TEXT("[TetherScene] 기대: 질량비 분배 자동(가벼우면 큐브가, 무거우면 내가 양보) — 스트레치 잔류(탄성 룩) 없이 경계 유지."));
		}
	}

	// ③ 근접 랙돌 씬(폭주 증상): 맵에 배치된 랙돌 반응 캐릭터(플레이어 제외)를 전방 2.5m로 소환한다.
	//    근접 wrap은 자유 다리가 짧아 방향 노이즈가 최대 = 레거시 폭주의 재현 조건.
	static void SceneRagdoll(UWorld* World)
	{
		APawn* Pawn = GetLocalPawn(World);
		if (!Pawn)
		{
			return;
		}
		for (TObjectIterator<URopeRagdollResponseComponent> It; It; ++It)
		{
			URopeRagdollResponseComponent* Comp = *It;
			AActor* Owner = IsValid(Comp) ? Comp->GetOwner() : nullptr;
			if (!IsValid(Owner) || Comp->GetWorld() != World || Owner == Pawn)
			{
				continue; // 플레이어 자신(같은 컴포넌트 보유)은 제외.
			}
			const FVector Dest = AheadOfPawn(*Pawn, 250.0f, 0.0f);
			const FRotator FaceMe = (Pawn->GetActorLocation() - Dest).GetSafeNormal2D().Rotation();
			Owner->TeleportTo(Dest, FaceMe, /*bIsATest*/ false, /*bNoCheck*/ true);
			UE_LOG(LogDynamicRope, Log, TEXT("[TetherScene] ragdoll: '%s'를 전방 2.5m로 소환. 절차 = 근접 wrap(자동 랙돌) → 유지/되감기, 이후 Rope.Ragdoll로 수동 토글도."),
				*GetNameSafe(Owner));
			UE_LOG(LogDynamicRope, Log, TEXT("[TetherScene] 기대: 요요/폭주 없이 물리 제약으로 차분히 끌림(창 레버 정렬 포함)."));
			return;
		}
		UE_LOG(LogDynamicRope, Warning, TEXT("[TetherScene] ragdoll: 랙돌 반응 캐릭터(URopeRagdollResponseComponent, 플레이어 제외)가 맵에 없다 — 데모 캐릭터를 배치할 것."));
	}

	static FAutoConsoleCommandWithWorldAndArgs GSceneCmd(
		TEXT("Rope.Test.TetherScene"),
		TEXT("테더 재현 씬 스폰(Docs/PoC/05 §8). 인자: wall(기둥 탈출) | drag [질량kg=100](물리 큐브) | ragdoll(근접 랙돌 소환)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			const FString Scene = (Args.Num() > 0) ? Args[0].ToLower() : FString();
			if (Scene == TEXT("wall"))
			{
				SceneWall(World);
			}
			else if (Scene == TEXT("drag"))
			{
				SceneDrag(World, (Args.Num() > 1) ? FCString::Atof(*Args[1]) : 100.0f);
			}
			else if (Scene == TEXT("ragdoll"))
			{
				SceneRagdoll(World);
			}
			else
			{
				UE_LOG(LogDynamicRope, Warning, TEXT("Rope.Test.TetherScene <wall|drag [masskg]|ragdoll>"));
			}
		}));

}

#endif // !UE_BUILD_SHIPPING
