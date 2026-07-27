// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoPressurePlate.h"
#include "DynamicRopeLog.h"

#include "Components/BoxComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Pawn.h"
#include "UObject/ConstructorHelpers.h"

ARopeDemoPressurePlate::ARopeDemoPressurePlate()
{
	// Interpolating the presentation and re-evaluating occupancy eligibility, which detects a ragdoll recovering, both need a tick.
	PrimaryActorTick.bCanEverTick = true;

	Frame = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Frame"));
	SetRootComponent(Frame);
	// The frame is slightly wider than the plate and very thin, at 120 x 120 x 4 cm.
	Frame->SetRelativeScale3D(FVector(1.2f, 1.2f, 0.04f));
	Frame->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Frame->SetCollisionObjectType(ECC_WorldStatic);
	Frame->SetMobility(EComponentMobility::Static);

	Pad = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Pad"));
	Pad->SetupAttachment(Frame);
	// The parent frame is at the pressed scale, so the child's scale is multiplied by its reciprocal to keep the absolute size.
	// The plate is 100 x 100 x 10 cm.
	Pad->SetRelativeScale3D(FVector(1.0f / 1.2f, 1.0f / 1.2f, 0.1f / 0.04f));
	Pad->SetRelativeLocation(FVector(0.0f, 0.0f, 7.0f));
	// It blocks so that an object resting on it cannot fall through, while the plate itself moves for presentation alone and is therefore movable.
	Pad->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Pad->SetCollisionObjectType(ECC_WorldStatic);
	Pad->SetMobility(EComponentMobility::Movable);

	Trigger = CreateDefaultSubobject<UBoxComponent>(TEXT("Trigger"));
	Trigger->SetupAttachment(Frame);
	// The detection volume covering the space above the plate, 100 x 100 x 120 cm in world units, corrected for the parent scale.
	Trigger->SetBoxExtent(FVector(50.0f, 50.0f, 60.0f));
	Trigger->SetRelativeScale3D(FVector(1.0f / 1.2f, 1.0f / 1.2f, 1.0f / 0.04f));
	Trigger->SetRelativeLocation(FVector(0.0f, 0.0f, 60.0f));
	Trigger->SetMobility(EComponentMobility::Movable);
	// Query-only, overlapping the channels of interest alone. Ragdoll bodies are PhysicsBody and a physical static mesh is usually WorldDynamic.
	Trigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	Trigger->SetCollisionObjectType(ECC_WorldStatic);
	Trigger->SetCollisionResponseToAllChannels(ECR_Ignore);
	Trigger->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap);
	Trigger->SetCollisionResponseToChannel(ECC_WorldDynamic, ECR_Overlap);
	Trigger->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
	Trigger->SetGenerateOverlapEvents(true);
	Trigger->SetHiddenInGame(true);

	IndicatorLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("IndicatorLight"));
	IndicatorLight->SetupAttachment(Frame);
	IndicatorLight->SetRelativeLocation(FVector(0.0f, 0.0f, 30.0f / 0.04f));
	IndicatorLight->SetIntensity(3000.0f);
	IndicatorLight->SetAttenuationRadius(400.0f);
	IndicatorLight->SetCastShadows(false);
	IndicatorLight->SetMobility(EComponentMobility::Movable);

	// Engine primitive shapes alone, to avoid creating a content dependency, since a plugin must not reference /Game.
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Frame->SetStaticMesh(CubeMesh.Object);
		Pad->SetStaticMesh(CubeMesh.Object);
	}
}

void ARopeDemoPressurePlate::BeginPlay()
{
	Super::BeginPlay();

	if (Trigger)
	{
		Trigger->OnComponentBeginOverlap.AddDynamic(this, &ARopeDemoPressurePlate::HandleBeginOverlap);
		Trigger->OnComponentEndOverlap.AddDynamic(this, &ARopeDemoPressurePlate::HandleEndOverlap);
	}

	if (IndicatorLight)
	{
		IndicatorLight->SetVisibility(bUseIndicatorLight);
	}
	ApplyIndicatorColor();

	// Actors already overlapping at BeginPlay, meaning objects placed on it in the level, are counted too.
	RefreshPressedState();
}

void ARopeDemoPressurePlate::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// A ragdoll recovering into animation while on the plate loses its eligibility with no overlap event, so it is re-evaluated every tick.
	RefreshPressedState();

	// The pressing presentation: the plate's relative Z is interpolated to its target at a constant rate.
	const float TargetOffset = bPressed ? -PressDepth : 0.0f;
	if (!FMath::IsNearlyEqual(CurrentPadOffset, TargetOffset) && Pad)
	{
		CurrentPadOffset = FMath::FInterpConstantTo(CurrentPadOffset, TargetOffset, DeltaSeconds, PressSpeed);
		// The pad is under the parent's pressed scale, whose Z is 0.04, so world centimetres are converted into relative coordinates before being applied.
		const float ParentZScale = Frame ? FMath::Max(KINDA_SMALL_NUMBER, Frame->GetRelativeScale3D().Z) : 1.0f;
		FVector Local = Pad->GetRelativeLocation();
		Local.Z = 7.0f + CurrentPadOffset / ParentZScale;
		Pad->SetRelativeLocation(Local);
	}
}

void ARopeDemoPressurePlate::HandleBeginOverlap(UPrimitiveComponent* /*OverlappedComponent*/, AActor* OtherActor,
	UPrimitiveComponent* /*OtherComp*/, int32 /*OtherBodyIndex*/, bool /*bFromSweep*/, const FHitResult& /*SweepResult*/)
{
	if (!OtherActor || OtherActor == this)
	{
		return;
	}

	// A ragdoll sends one event per body, so they are summed per actor.
	int32& Count = OverlapCounts.FindOrAdd(OtherActor);
	++Count;

	RefreshPressedState();
}

void ARopeDemoPressurePlate::HandleEndOverlap(UPrimitiveComponent* /*OverlappedComponent*/, AActor* OtherActor,
	UPrimitiveComponent* /*OtherComp*/, int32 /*OtherBodyIndex*/)
{
	if (!OtherActor)
	{
		return;
	}

	if (int32* Count = OverlapCounts.Find(OtherActor))
	{
		if (--(*Count) <= 0)
		{
			OverlapCounts.Remove(OtherActor);
		}
	}

	RefreshPressedState();
}

TArray<AActor*> ARopeDemoPressurePlate::GetQualifyingOccupants() const
{
	// Occupancy is summed per actor in the overlap counts, so those are walked directly. The eligibility decision
	// shares IsQualifyingOccupant with the press computation, which prevents the plate being pressed while the list is empty.
	TArray<AActor*> Occupants;
	Occupants.Reserve(OverlapCounts.Num());
	for (const TPair<TWeakObjectPtr<AActor>, int32>& Pair : OverlapCounts)
	{
		AActor* Occupant = Pair.Key.Get();
		if (IsQualifyingOccupant(Occupant))
		{
			Occupants.Add(Occupant);
		}
	}
	return Occupants;
}

bool ARopeDemoPressurePlate::IsQualifyingOccupant(const AActor* OtherActor) const
{
	if (!IsValid(OtherActor))
	{
		return false;
	}

	if (!RequiredActorTag.IsNone() && !OtherActor->ActorHasTag(RequiredActorTag))
	{
		return false;
	}

	if (!bRequireSimulatingPhysics)
	{
		return true;
	}

	// A single simulating body is enough to qualify, which catches both a physical static mesh and a ragdoll,
	// including a partial one. A character who walked onto it is filtered out here, since its capsule does not simulate.
	TArray<UPrimitiveComponent*> Primitives;
	OtherActor->GetComponents<UPrimitiveComponent>(Primitives);
	for (const UPrimitiveComponent* Primitive : Primitives)
	{
		if (Primitive && Primitive->IsSimulatingPhysics())
		{
			return true;
		}
	}
	return false;
}

void ARopeDemoPressurePlate::RefreshPressedState()
{
	int32 NewCount = 0;
	// Recounts eligibility while cleaning out destroyed actors.
	for (auto It = OverlapCounts.CreateIterator(); It; ++It)
	{
		const AActor* Occupant = It.Key().Get();
		if (!IsValid(Occupant))
		{
			It.RemoveCurrent();
			continue;
		}
		if (IsQualifyingOccupant(Occupant))
		{
			++NewCount;
		}
	}

	OccupantCount = NewCount;

	const bool bNewPressed = OccupantCount >= FMath::Max(1, RequiredOccupants);
	if (bNewPressed == bPressed)
	{
		return;
	}

	bPressed = bNewPressed;
	ApplyIndicatorColor();

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] pressure plate %s (%d/%d occupants)"),
		*GetName(), bPressed ? TEXT("pressed") : TEXT("released"), OccupantCount, FMath::Max(1, RequiredOccupants));

	OnPlatePressedChanged.Broadcast(this, bPressed);
}

void ARopeDemoPressurePlate::ApplyIndicatorColor()
{
	if (IndicatorLight)
	{
		IndicatorLight->SetLightColor(bPressed ? PressedColor : IdleColor);
	}
}
