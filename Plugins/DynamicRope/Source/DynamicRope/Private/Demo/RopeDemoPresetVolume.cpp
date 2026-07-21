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
	// 문 하나 폭 정도의 통과 영역(400x200x300cm). 레벨에 맞춰 조정한다.
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
	// 대기 구독을 남긴 채 사라지면 로프 쪽 델리게이트에 죽은 핸들러가 남는다.
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

	// 랙돌·다중 콜리전 액터는 이벤트가 여러 번 오므로 액터 단위로 센다. 첫 진입에서만 적용한다.
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

	// 완전히 빠져나갔다 — 아직 기다리던 로프가 있으면 포기한다(볼륨 밖에서 뒤늦게 바뀌면 안 된다).
	TArray<URopeComponent*> Ropes;
	OtherActor->GetComponents<URopeComponent>(Ropes);
	for (URopeComponent* Rope : Ropes)
	{
		StopWaitingFor(Rope);
	}

	if (ExitPreset)
	{
		// 나가는 길에는 기다리지 않는다 — 이미 볼륨 밖이라 대기 조건(안에 있는 동안)이 성립하지 않는다.
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

		// 비행/감김 중 — Free/Reel로 돌아오는 첫 순간에 적용하려고 구독해 둔다.
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
	// ApplyPreset이 Free/Reel 게이트를 직접 판정하므로(거부 시 false, 아무것도 안 바꿈) 여기서
	// 페이즈를 다시 해석하지 않는다 — 게이트의 단일 소스는 컴포넌트다.
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
	// 델리게이트 페이로드에 로프가 실려 오지 않으므로 대기열을 훑는다(대기열은 길어야 로프 몇 개다).
	// 적용은 성공한 것만 대기열에서 빠지므로, 아직 안 되는 로프는 다음 전이에서 다시 시도된다.
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
