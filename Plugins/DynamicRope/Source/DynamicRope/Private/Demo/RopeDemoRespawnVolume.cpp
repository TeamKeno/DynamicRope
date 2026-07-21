// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoRespawnVolume.h"
#include "DynamicRopeLog.h"
#include "RopeComponent.h"
#include "Gameplay/RopeRagdollResponseComponent.h"
#include "Gameplay/RopeWielderComponent.h"
#include "Subsystem/RopeSimSubsystem.h"

#include "Components/BoxComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Controller.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PawnMovementComponent.h"

ARopeDemoRespawnVolume::ARopeDemoRespawnVolume()
{
	PrimaryActorTick.bCanEverTick = false;

	Trigger = CreateDefaultSubobject<UBoxComponent>(TEXT("Trigger"));
	SetRootComponent(Trigger);
	// 기본 크기는 넉넉하게(100m x 100m x 10m). 레벨에 맞춰 스케일을 조정한다.
	Trigger->SetBoxExtent(FVector(5000.0f, 5000.0f, 500.0f));
	Trigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	Trigger->SetCollisionObjectType(ECC_WorldStatic);
	Trigger->SetCollisionResponseToAllChannels(ECR_Ignore);
	Trigger->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
	Trigger->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap);
	Trigger->SetCollisionResponseToChannel(ECC_WorldDynamic, ECR_Overlap);
	Trigger->SetGenerateOverlapEvents(true);
	Trigger->SetHiddenInGame(true);
}

void ARopeDemoRespawnVolume::BeginPlay()
{
	Super::BeginPlay();

	TrackProps();

	if (Trigger)
	{
		Trigger->OnComponentBeginOverlap.AddDynamic(this, &ARopeDemoRespawnVolume::HandleBeginOverlap);
	}
}

void ARopeDemoRespawnVolume::TrackProps()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!IsValid(Actor) || Actor == this || Actor->IsA<APawn>())
		{
			continue;
		}

		bool bTrack = !PropTag.IsNone() && Actor->ActorHasTag(PropTag);
		if (!bTrack && bAutoTrackPhysicsProps)
		{
			// 시작 시점에 물리 시뮬 중인 것 = 굴러다닐 수 있는 소품. 로프로 옮길 대상이 여기 다 걸린다.
			TArray<UPrimitiveComponent*> Primitives;
			Actor->GetComponents<UPrimitiveComponent>(Primitives);
			for (const UPrimitiveComponent* Primitive : Primitives)
			{
				if (Primitive && Primitive->IsSimulatingPhysics())
				{
					bTrack = true;
					break;
				}
			}
		}

		if (bTrack)
		{
			TrackedProps.Add(Actor, Actor->GetActorTransform());
		}
	}

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] respawn volume tracking %d prop(s)."), *GetName(), TrackedProps.Num());
}

void ARopeDemoRespawnVolume::HandleBeginOverlap(UPrimitiveComponent* /*OverlappedComponent*/, AActor* OtherActor,
	UPrimitiveComponent* /*OtherComp*/, int32 /*OtherBodyIndex*/, bool /*bFromSweep*/, const FHitResult& /*SweepResult*/)
{
	RespawnActor(OtherActor);
}

bool ARopeDemoRespawnVolume::RespawnActor(AActor* Target)
{
	if (!IsValid(Target) || Target == this)
	{
		return false;
	}

	// 랙돌 본은 여러 바디가 각각 볼륨에 들어오므로 같은 액터로 이벤트가 여러 번 온다. 이미 복귀해
	// 볼륨 밖으로 나간 뒤의 늦은 이벤트까지 처리하지 않도록, 실제 처리 대상인지 먼저 가른다.
	if (APawn* Pawn = Cast<APawn>(Target))
	{
		return bRespawnPawns && RespawnPawn(Pawn);
	}

	if (const FTransform* StartTransform = TrackedProps.Find(Target))
	{
		return RespawnProp(Target, *StartTransform);
	}

	// 등록되지 않은 액터는 건드리지 않는다(엔진 KillZ와 달리 파괴하지 않는다).
	UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] respawn volume ignored untracked actor %s."), *GetName(), *Target->GetName());
	return false;
}

bool ARopeDemoRespawnVolume::RespawnPawn(APawn* Pawn)
{
	if (!IsValid(Pawn))
	{
		return false;
	}

	FTransform Destination;
	if (IsValid(PawnRespawnPointOverride))
	{
		Destination = PawnRespawnPointOverride->GetActorTransform();
	}
	else if (AGameModeBase* GameMode = GetWorld() ? GetWorld()->GetAuthGameMode() : nullptr)
	{
		// GameMode의 PlayerStart 선택 규칙(태그/점유 판정)을 그대로 쓴다.
		if (const AActor* Start = GameMode->FindPlayerStart(Pawn->GetController()))
		{
			Destination = Start->GetActorTransform();
		}
	}

	if (!Destination.IsValid() || Destination.GetLocation().IsNearlyZero())
	{
		// PlayerStart가 없는 맵(플러그인 데모 맵을 그냥 열었을 때)에서도 최소한 볼륨 위로는 올린다.
		Destination.SetLocation(GetActorLocation() + FVector(0.0f, 0.0f, 1000.0f));
		Destination.SetRotation(FQuat::Identity);
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] no PlayerStart found — respawning %s above the volume instead."), *GetName(), *Pawn->GetName());
	}

	if (bReleaseRopesOnRespawn)
	{
		ReleaseRopesInvolving(Pawn);
	}

	// 랙돌 중이면 본 바디가 월드 공간에 있어 액터만 옮겨도 메시가 따라오지 않는다 — 먼저 복귀시킨다.
	if (URopeRagdollResponseComponent* Ragdoll = Pawn->FindComponentByClass<URopeRagdollResponseComponent>())
	{
		if (Ragdoll->IsRagdolled())
		{
			Ragdoll->RecoverFromRagdoll();
		}
	}

	if (UPawnMovementComponent* Movement = Pawn->GetMovementComponent())
	{
		Movement->StopMovementImmediately();
	}
	ZeroPhysicsVelocities(Pawn);

	Pawn->TeleportTo(Destination.GetLocation(), Destination.Rotator());
	if (AController* Controller = Pawn->GetController())
	{
		Controller->SetControlRotation(Destination.Rotator());
	}

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] respawned pawn %s."), *GetName(), *Pawn->GetName());
	OnActorRespawned.Broadcast(this, Pawn);
	return true;
}

bool ARopeDemoRespawnVolume::RespawnProp(AActor* Prop, const FTransform& StartTransform)
{
	if (!IsValid(Prop))
	{
		return false;
	}

	if (bReleaseRopesOnRespawn)
	{
		ReleaseRopesInvolving(Prop);
	}

	// 물리 바디는 TeleportPhysics로 옮겨야 시뮬 상태가 함께 따라간다(그냥 이동하면 속도가 주입된다).
	Prop->SetActorTransform(StartTransform, /*bSweep=*/false, /*OutSweepHitResult=*/nullptr, ETeleportType::TeleportPhysics);
	ZeroPhysicsVelocities(Prop);

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] respawned prop %s."), *GetName(), *Prop->GetName());
	OnActorRespawned.Broadcast(this, Prop);
	return true;
}

void ARopeDemoRespawnVolume::RespawnAllProps()
{
	for (auto It = TrackedProps.CreateIterator(); It; ++It)
	{
		AActor* Prop = It.Key().Get();
		if (!IsValid(Prop))
		{
			It.RemoveCurrent();
			continue;
		}
		RespawnProp(Prop, It.Value());
	}
}

void ARopeDemoRespawnVolume::ReleaseRopesInvolving(AActor* Actor)
{
	if (!IsValid(Actor))
	{
		return;
	}

	UWorld* World = GetWorld();
	URopeSimSubsystem* Subsystem = World ? World->GetSubsystem<URopeSimSubsystem>() : nullptr;
	if (!Subsystem)
	{
		return;
	}

	for (URopeComponent* Rope : Subsystem->GetRegisteredRopes())
	{
		if (!IsValid(Rope))
		{
			continue;
		}

		// (1) 이 액터가 들고 있는 로프, (2) 이 액터를 감고 있는 로프 — 둘 다 푼다.
		const bool bOwnedByActor = Rope->GetOwner() == Actor;
		const USkeletalMeshComponent* WrappedMesh = Rope->GetWrappedMesh();
		const bool bWrapsActor = WrappedMesh && WrappedMesh->GetOwner() == Actor;

		if (bOwnedByActor || bWrapsActor)
		{
			Rope->ReleaseWrap();
		}
	}
}

void ARopeDemoRespawnVolume::ZeroPhysicsVelocities(AActor* Actor)
{
	if (!IsValid(Actor))
	{
		return;
	}

	TArray<UPrimitiveComponent*> Primitives;
	Actor->GetComponents<UPrimitiveComponent>(Primitives);
	for (UPrimitiveComponent* Primitive : Primitives)
	{
		if (Primitive && Primitive->IsSimulatingPhysics())
		{
			Primitive->SetPhysicsLinearVelocity(FVector::ZeroVector);
			Primitive->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);
		}
	}
}
