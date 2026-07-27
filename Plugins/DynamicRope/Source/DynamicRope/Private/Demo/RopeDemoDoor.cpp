// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoDoor.h"
#include "Demo/RopeDemoPressurePlate.h"
#include "DynamicRopeLog.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "UObject/ConstructorHelpers.h"

ARopeDemoDoor::ARopeDemoDoor()
{
	// Only interpolating the door panel's movement needs a tick.
	PrimaryActorTick.bCanEverTick = true;

	Frame = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Frame"));
	SetRootComponent(Frame);
	// The frame is a thin plate slightly larger than the door, at 220 x 20 x 260 cm, scaled from the engine's 100 cm cube.
	Frame->SetRelativeScale3D(FVector(2.2f, 0.2f, 2.6f));
	Frame->SetRelativeLocation(FVector(0.0f, 0.0f, 130.0f));
	Frame->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Frame->SetVisibility(false);
	Frame->SetMobility(EComponentMobility::Static);

	Leaf = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Leaf"));
	Leaf->SetupAttachment(Frame);
	// Multiplied by the reciprocal of the parent's scale to establish the absolute size, giving a 200 x 30 x 250 cm door panel.
	Leaf->SetRelativeScale3D(FVector(2.0f / 2.2f, 0.3f / 0.2f, 2.5f / 2.6f));
	Leaf->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Leaf->SetCollisionObjectType(ECC_WorldStatic);
	// It actually moves when it opens, so it has to be movable; a static one would not reflect the movement.
	Leaf->SetMobility(EComponentMobility::Movable);

	// Engine primitive shapes alone, to avoid creating a content dependency, since a plugin must not reference /Game.
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Frame->SetStaticMesh(CubeMesh.Object);
		Leaf->SetStaticMesh(CubeMesh.Object);
	}
}

void ARopeDemoDoor::BeginPlay()
{
	Super::BeginPlay();

	if (Leaf)
	{
		ClosedLeafLocation = Leaf->GetRelativeLocation();
	}

	GatherTaggedPlates();

	for (ARopeDemoPressurePlate* Plate : Plates)
	{
		if (IsValid(Plate))
		{
			Plate->OnPlatePressedChanged.AddDynamic(this, &ARopeDemoDoor::HandlePlatePressedChanged);
		}
	}

	if (Plates.Num() == 0)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] demo door has no plates linked — set Plates or PlateTag, or it will never open."), *GetName());
	}

	// Evaluated once at startup so that objects placed on the plates in the level are supported too.
	EvaluateOpenCondition();
}

void ARopeDemoDoor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	for (ARopeDemoPressurePlate* Plate : Plates)
	{
		if (IsValid(Plate))
		{
			Plate->OnPlatePressedChanged.RemoveDynamic(this, &ARopeDemoDoor::HandlePlatePressedChanged);
		}
	}

	Super::EndPlay(EndPlayReason);
}

void ARopeDemoDoor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!Leaf)
	{
		return;
	}

	// The parent's scale applies, so the world offset in centimetres is converted into relative coordinates.
	const FVector ParentScale = Frame ? Frame->GetRelativeScale3D() : FVector::OneVector;
	const FVector SafeScale(
		FMath::Max(KINDA_SMALL_NUMBER, FMath::Abs(ParentScale.X)),
		FMath::Max(KINDA_SMALL_NUMBER, FMath::Abs(ParentScale.Y)),
		FMath::Max(KINDA_SMALL_NUMBER, FMath::Abs(ParentScale.Z)));
	const FVector LocalOpenOffset(OpenOffset.X / SafeScale.X, OpenOffset.Y / SafeScale.Y, OpenOffset.Z / SafeScale.Z);

	const FVector Target = bOpen ? (ClosedLeafLocation + LocalOpenOffset) : ClosedLeafLocation;
	const FVector Current = Leaf->GetRelativeLocation();
	if (Current.Equals(Target))
	{
		return;
	}

	// The speed is in world centimetres per second, so it is converted into an interpolation rate in relative coordinates, taken from the dominant axis where the scale differs per axis.
	const float MaxAxisScale = FMath::Max3(SafeScale.X, SafeScale.Y, SafeScale.Z);
	const float LocalSpeed = OpenSpeed / MaxAxisScale;
	Leaf->SetRelativeLocation(FMath::VInterpConstantTo(Current, Target, DeltaSeconds, LocalSpeed));
}

int32 ARopeDemoDoor::GetPressedPlateCount() const
{
	int32 Pressed = 0;
	for (const ARopeDemoPressurePlate* Plate : Plates)
	{
		if (IsValid(Plate) && Plate->IsPressed())
		{
			++Pressed;
		}
	}
	return Pressed;
}

int32 ARopeDemoDoor::GetRequiredPlateCount() const
{
	if (RequiredPressedCount > 0)
	{
		return RequiredPressedCount;
	}

	// Zero means every connected plate. Only the valid ones are counted.
	int32 Valid = 0;
	for (const ARopeDemoPressurePlate* Plate : Plates)
	{
		if (IsValid(Plate))
		{
			++Valid;
		}
	}
	return Valid;
}

void ARopeDemoDoor::SetOpen(bool bNewOpen)
{
	if (bOpen == bNewOpen)
	{
		return;
	}

	bOpen = bNewOpen;

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] demo door %s (%d/%d plates)"),
		*GetName(), bOpen ? TEXT("opening") : TEXT("closing"), GetPressedPlateCount(), GetRequiredPlateCount());

	OnDoorStateChanged.Broadcast(this, bOpen);
}

void ARopeDemoDoor::HandlePlatePressedChanged(ARopeDemoPressurePlate* /*Plate*/, bool /*bPressed*/)
{
	EvaluateOpenCondition();
}

void ARopeDemoDoor::EvaluateOpenCondition()
{
	// Once it has opened, bStayOpen keeps it open even when the condition breaks, so clearing it does not revert.
	if (bOpen && bStayOpen)
	{
		return;
	}

	const int32 Required = GetRequiredPlateCount();
	// With no connected plates it does not open, which prevents zero being at least zero and opening it.
	const bool bShouldOpen = Required > 0 && GetPressedPlateCount() >= Required;

	SetOpen(bShouldOpen);
}

void ARopeDemoDoor::GatherTaggedPlates()
{
	if (PlateTag.IsNone())
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	for (TActorIterator<ARopeDemoPressurePlate> It(World); It; ++It)
	{
		ARopeDemoPressurePlate* Plate = *It;
		if (IsValid(Plate) && Plate->ActorHasTag(PlateTag))
		{
			// AddUnique, so that a plate also named directly in the list is not subscribed twice.
			Plates.AddUnique(Plate);
		}
	}
}
