// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoRespawnVolume.h"
#include "DynamicRopeLog.h"
#include "RopeComponent.h"
#include "Gameplay/RopeRagdollResponseComponent.h"
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
	// A generous default size of 100 x 100 x 10 m, whose scale is adjusted to suit the level.
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
			// Whatever is simulating physics at startup is a prop that can roll away, which catches everything a rope might move.
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

	// A ragdoll's bodies each enter the volume separately, so the same actor sends the event several times. To avoid
	// handling a late event after it has already respawned and left the volume, whether it is genuinely to be handled is decided first.
	if (APawn* Pawn = Cast<APawn>(Target))
	{
		return bRespawnPawns && RespawnPawn(Pawn);
	}

	if (const FTransform* StartTransform = TrackedProps.Find(Target))
	{
		return RespawnProp(Target, *StartTransform);
	}

	// An unregistered actor is left alone and, unlike the engine's kill Z, is not destroyed.
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
		// Uses the game mode's rules for choosing a player start, including its tags and occupancy test, as they stand.
		if (const AActor* Start = GameMode->FindPlayerStart(Pawn->GetController()))
		{
			Destination = Start->GetActorTransform();
		}
	}

	if (!Destination.IsValid() || Destination.GetLocation().IsNearlyZero())
	{
		// On a map with no player start, as when the plugin's demo map is simply opened, it is at least lifted above the volume.
		Destination.SetLocation(GetActorLocation() + FVector(0.0f, 0.0f, 1000.0f));
		Destination.SetRotation(FQuat::Identity);
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] no PlayerStart found — respawning %s above the volume instead."), *GetName(), *Pawn->GetName());
	}

	if (bReleaseRopesOnRespawn)
	{
		ReleaseRopesInvolving(Pawn);
	}

		// While ragdolling the bone bodies are in world space and moving the actor alone would leave the mesh behind, so it is recovered first.
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

	// A physics body has to be moved with a teleport so its simulation state follows; moving it directly injects velocity.
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

		// Both are released: the ropes this actor is holding, and the ropes wrapped around it.
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
