// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoPresetVolume.h"
#include "DynamicRopeLog.h"
#include "Preset/RopePreset.h"
#include "RopeComponent.h"

#include "Components/BoxComponent.h"
#include "GameFramework/Pawn.h"

ARopeDemoPresetVolume::ARopeDemoPresetVolume()
{
	PrimaryActorTick.bCanEverTick = false;

	Trigger = CreateDefaultSubobject<UBoxComponent>(TEXT("Trigger"));
	SetRootComponent(Trigger);
	// A passage roughly one doorway wide, at 400 x 200 x 300 cm, adjusted to suit the level.
	Trigger->SetBoxExtent(FVector(200.0f, 100.0f, 150.0f));
	Trigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	Trigger->SetCollisionObjectType(ECC_WorldStatic);
	Trigger->SetCollisionResponseToAllChannels(ECR_Ignore);
	Trigger->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
	Trigger->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap);
	Trigger->SetCollisionResponseToChannel(ECC_WorldDynamic, ECR_Overlap);
	Trigger->SetGenerateOverlapEvents(true);
	Trigger->SetHiddenInGame(true);
}

void ARopeDemoPresetVolume::BeginPlay()
{
	Super::BeginPlay();

	if (Trigger)
	{
		Trigger->OnComponentBeginOverlap.AddDynamic(this, &ARopeDemoPresetVolume::HandleBeginOverlap);
		Trigger->OnComponentEndOverlap.AddDynamic(this, &ARopeDemoPresetVolume::HandleEndOverlap);
	}

	if (!Preset && !ExitPreset)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] preset volume has neither Preset nor ExitPreset set — it will do nothing."), *GetName());
	}
}

void ARopeDemoPresetVolume::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Disappearing while a pending subscription remains would leave a dead handler on the rope's delegates.
	for (const TWeakObjectPtr<URopeComponent>& WeakRope : PendingRopes)
	{
		if (URopeComponent* Rope = WeakRope.Get())
		{
			Rope->OnRopePhaseChanged.RemoveDynamic(this, &ARopeDemoPresetVolume::HandleRopePhaseChanged);
		}
	}
	PendingRopes.Empty();

	Super::EndPlay(EndPlayReason);
}

void ARopeDemoPresetVolume::HandleBeginOverlap(UPrimitiveComponent* /*OverlappedComponent*/, AActor* OtherActor,
	UPrimitiveComponent* /*OtherComp*/, int32 /*OtherBodyIndex*/, bool /*bFromSweep*/, const FHitResult& /*SweepResult*/)
{
	if (!IsValid(OtherActor) || OtherActor == this)
	{
		return;
	}

	// A ragdoll or an actor with several collision shapes sends the event more than once, so it is counted per actor and applied on first entry alone.
	int32& Count = OverlapCounts.FindOrAdd(OtherActor);
	if (++Count > 1)
	{
		return;
	}

	if (Preset)
	{
		ApplyToActor(OtherActor, Preset, bApplyWhenRopeSettles);
	}
}

void ARopeDemoPresetVolume::HandleEndOverlap(UPrimitiveComponent* /*OverlappedComponent*/, AActor* OtherActor,
	UPrimitiveComponent* /*OtherComp*/, int32 /*OtherBodyIndex*/)
{
	if (!IsValid(OtherActor))
	{
		return;
	}

	int32* Count = OverlapCounts.Find(OtherActor);
	if (!Count || --(*Count) > 0)
	{
		return;
	}
	OverlapCounts.Remove(OtherActor);

	// It has left entirely, so any rope still waiting is abandoned, since nothing should change late, outside the volume.
	TArray<URopeComponent*> Ropes;
	OtherActor->GetComponents<URopeComponent>(Ropes);
	for (URopeComponent* Rope : Ropes)
	{
		StopWaitingFor(Rope);
	}

	if (ExitPreset)
	{
	// It does not wait on the way out: it is already outside the volume, so the waiting condition, being inside it, no longer holds.
		ApplyToActor(OtherActor, ExitPreset, /*bWaitIfBusy=*/false);
	}
}

void ARopeDemoPresetVolume::ApplyToActor(AActor* Actor, const URopePreset* InPreset, bool bWaitIfBusy)
{
	if (!IsValid(Actor) || !InPreset)
	{
		return;
	}

	if (bPawnsOnly && !Actor->IsA<APawn>())
	{
		return;
	}

	TArray<URopeComponent*> Ropes;
	Actor->GetComponents<URopeComponent>(Ropes);
	for (URopeComponent* Rope : Ropes)
	{
		if (!IsValid(Rope))
		{
			continue;
		}

		if (ApplyToRope(Rope, InPreset))
		{
			StopWaitingFor(Rope);
			continue;
		}

		if (!bWaitIfBusy)
		{
			UE_LOG(LogDynamicRope, Log,
				TEXT("[%s] preset apply refused for %s (phase busy) and waiting is off — skipped."),
				*GetName(), *Rope->GetName());
			continue;
		}

		// It is in flight or wrapping, so it subscribes in order to apply the preset the first moment it returns to Free or Loaded.
		bool bAlreadyPending = false;
		PendingRopes.Add(Rope, &bAlreadyPending);
		if (!bAlreadyPending)
		{
			Rope->OnRopePhaseChanged.AddDynamic(this, &ARopeDemoPresetVolume::HandleRopePhaseChanged);
			UE_LOG(LogDynamicRope, Log,
				TEXT("[%s] rope %s is busy — will apply the preset when it settles."), *GetName(), *Rope->GetName());
		}
	}
}

bool ARopeDemoPresetVolume::ApplyToRope(URopeComponent* Rope, const URopePreset* InPreset)
{
	// ApplyPreset decides the Free and Loaded gate itself, returning false and changing nothing on a refusal, so the
	// phase is not interpreted again here: the single source for the gate is the component.
	if (!IsValid(Rope) || !InPreset || !Rope->ApplyPreset(InPreset))
	{
		return false;
	}

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] applied preset %s to %s."),
		*GetName(), *InPreset->GetName(), *Rope->GetName());

	OnPresetVolumeApplied.Broadcast(this, Rope, InPreset);
	return true;
}

void ARopeDemoPresetVolume::HandleRopePhaseChanged(ERopePhase /*OldPhase*/, ERopePhase /*NewPhase*/)
{
	// The delegate payload does not carry the rope, so the pending list is walked; it is a few ropes long at most.
	// Only those applied successfully leave the list, so a rope that is still not ready is retried on the next transition.
	TArray<TWeakObjectPtr<URopeComponent>> Snapshot = PendingRopes.Array();
	for (const TWeakObjectPtr<URopeComponent>& WeakRope : Snapshot)
	{
		URopeComponent* Rope = WeakRope.Get();
		if (!IsValid(Rope))
		{
			PendingRopes.Remove(WeakRope);
			continue;
		}

		if (ApplyToRope(Rope, Preset))
		{
			StopWaitingFor(Rope);
		}
	}
}

void ARopeDemoPresetVolume::StopWaitingFor(URopeComponent* Rope)
{
	if (!Rope || PendingRopes.Remove(Rope) == 0)
	{
		return;
	}

	Rope->OnRopePhaseChanged.RemoveDynamic(this, &ARopeDemoPresetVolume::HandleRopePhaseChanged);
}
