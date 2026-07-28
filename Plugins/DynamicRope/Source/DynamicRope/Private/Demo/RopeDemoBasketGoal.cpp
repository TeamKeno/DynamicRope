// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoBasketGoal.h"
#include "DynamicRopeLog.h"

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialParameters.h"

ARopeDemoBasketGoal::ARopeDemoBasketGoal()
{
	// Only the pulse decay needs a tick, so the actor starts ticking when a goal is scored and stops itself once the pulse is over.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = false;

	Trigger = CreateDefaultSubobject<UBoxComponent>(TEXT("Trigger"));
	SetRootComponent(Trigger);
	// 80 x 80 x 30 cm, roughly a hoop's mouth. Scale it in the level to match the rim actually placed there.
	Trigger->SetBoxExtent(FVector(40.0f, 40.0f, 15.0f));
	Trigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	Trigger->SetCollisionObjectType(ECC_WorldStatic);
	Trigger->SetCollisionResponseToAllChannels(ECR_Ignore);
	// A simulating ball is PhysicsBody on the PhysicsActor preset and WorldDynamic on the default one, which covers both ways of setting one up.
	Trigger->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap);
	Trigger->SetCollisionResponseToChannel(ECC_WorldDynamic, ECR_Overlap);
	Trigger->SetGenerateOverlapEvents(true);
	Trigger->SetHiddenInGame(true);
}

void ARopeDemoBasketGoal::BeginPlay()
{
	Super::BeginPlay();

	PrepareHoopMaterials();

	if (Trigger)
	{
		Trigger->OnComponentBeginOverlap.AddDynamic(this, &ARopeDemoBasketGoal::HandleBeginOverlap);
	}

	// Idle is the pulse at zero strength, so the emissive parameter starts from a known value.
	ApplyFlash(0.0f);
}

void ARopeDemoBasketGoal::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	FlashRemaining = FMath::Max(0.0f, FlashRemaining - DeltaSeconds);

	// Squared, so the pulse lands as a snap and trails off rather than fading symmetrically in and out.
	const float Linear = FlashDuration > SMALL_NUMBER ? FlashRemaining / FlashDuration : 0.0f;
	ApplyFlash(Linear * Linear);

	if (FlashRemaining <= 0.0f)
	{
		SetActorTickEnabled(false);
	}
}

void ARopeDemoBasketGoal::HandleBeginOverlap(UPrimitiveComponent* /*OverlappedComponent*/, AActor* OtherActor,
	UPrimitiveComponent* OtherComp, int32 /*OtherBodyIndex*/, bool /*bFromSweep*/, const FHitResult& /*SweepResult*/)
{
	const UWorld* World = GetWorld();
	if (!World || !IsValid(OtherActor) || OtherActor == this)
	{
		return;
	}

	if (!IsQualifyingBall(OtherActor))
	{
		return;
	}

	// A ball rattling around the rim crosses the volume repeatedly, so each one is locked out for a while after it scores.
	const float Now = World->GetTimeSeconds();
	if (const float* LastScoreTime = LastScoreTimes.Find(OtherActor))
	{
		if (Now - *LastScoreTime < ScoreCooldown)
		{
			return;
		}
	}

	if (bRequireDownwardEntry && !IsDownwardEntry(OtherActor, OtherComp))
	{
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] %s crossed the goal without a downward pass — not counted."),
			*GetName(), *OtherActor->GetName());
		return;
	}

	// Records the lockout while clearing out balls that have since been destroyed.
	LastScoreTimes.Add(OtherActor, Now);
	for (auto It = LastScoreTimes.CreateIterator(); It; ++It)
	{
		if (!It.Key().IsValid())
		{
			It.RemoveCurrent();
		}
	}

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] goal by %s."), *GetName(), *OtherActor->GetName());

	// The pulse restarts from its peak; the tick that decays it runs only while it is playing.
	FlashRemaining = FlashDuration;
	ApplyFlash(1.0f);
	SetActorTickEnabled(true);

	OnGoalScored.Broadcast(this, OtherActor);
}

bool ARopeDemoBasketGoal::IsQualifyingBall(AActor* Candidate) const
{
	// The player walking through the hoop is not a goal, whatever else matches.
	if (!IsValid(Candidate) || Candidate == this || Candidate->IsA<APawn>())
	{
		return false;
	}

	if (Balls.Contains(Candidate))
	{
		return true;
	}

	if (!BallTag.IsNone() && Candidate->ActorHasTag(BallTag))
	{
		return true;
	}

	// With neither the array nor the tag configured, anything that simulates physics counts. That is the least wiring
	// for a playground where the ball is the only loose prop near the hoop; either of the two settings above narrows it.
	if (BallTag.IsNone() && Balls.Num() == 0)
	{
		TArray<UPrimitiveComponent*> Primitives;
		Candidate->GetComponents<UPrimitiveComponent>(Primitives);
		for (const UPrimitiveComponent* Primitive : Primitives)
		{
			if (Primitive && Primitive->IsSimulatingPhysics())
			{
				return true;
			}
		}
	}

	return false;
}

bool ARopeDemoBasketGoal::IsDownwardEntry(AActor* Ball, UPrimitiveComponent* BallComponent) const
{
	FVector Velocity = FVector::ZeroVector;
	if (BallComponent && BallComponent->IsSimulatingPhysics())
	{
		Velocity = BallComponent->GetPhysicsLinearVelocity();
	}
	else if (IsValid(Ball))
	{
		// A ball carried or attached rather than simulating still reports actor velocity, so a dunk counts too.
		Velocity = Ball->GetVelocity();
	}

	// Measured along the goal's own down axis rather than world -Z, so a hoop tilted in the level works as long as this actor is rotated with it.
	const float DownwardSpeed = FVector::DotProduct(Velocity, -GetActorUpVector());
	return DownwardSpeed >= MinDownwardSpeed;
}

void ARopeDemoBasketGoal::PrepareHoopMaterials()
{
	HoopMaterials.Reset();
	if (!IsValid(HoopFlashActor))
	{
		return;
	}

	TArray<UStaticMeshComponent*> Meshes;
	HoopFlashActor->GetComponents<UStaticMeshComponent>(Meshes);
	for (UStaticMeshComponent* Mesh : Meshes)
	{
		if (!Mesh)
		{
			continue;
		}
		for (int32 SlotIndex = 0; SlotIndex < Mesh->GetNumMaterials(); ++SlotIndex)
		{
			if (UMaterialInstanceDynamic* Dynamic = Mesh->CreateAndSetMaterialInstanceDynamic(SlotIndex))
			{
				HoopMaterials.Add(Dynamic);
			}
		}
	}

	// Writing a parameter a material does not declare is silently ignored, which would leave the pulse looking broken
	// with nothing to go on. Whether the parameter exists is therefore checked once here and reported.
	FLinearColor ExistingValue;
	const bool bParameterFound = HoopMaterials.ContainsByPredicate([this, &ExistingValue](const UMaterialInstanceDynamic* Dynamic)
	{
		return Dynamic && Dynamic->GetVectorParameterValue(FHashedMaterialParameterInfo(HoopEmissiveParam), ExistingValue);
	});

	if (HoopMaterials.Num() == 0)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] HoopFlashActor %s has no static mesh materials — no pulse."),
			*GetName(), *HoopFlashActor->GetName());
	}
	else if (!bParameterFound)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] none of %s's materials expose the vector parameter '%s' — no pulse. Add the parameter, or point HoopEmissiveParam at one it does have."),
			*GetName(), *HoopFlashActor->GetName(), *HoopEmissiveParam.ToString());
	}
}

void ARopeDemoBasketGoal::ApplyFlash(float Alpha)
{
	Alpha = FMath::Clamp(Alpha, 0.0f, 1.0f);

	const FLinearColor Emissive = FlashColor * (HoopFlashBrightness * Alpha);
	for (UMaterialInstanceDynamic* Dynamic : HoopMaterials)
	{
		if (Dynamic)
		{
			Dynamic->SetVectorParameterValue(HoopEmissiveParam, Emissive);
		}
	}
}
