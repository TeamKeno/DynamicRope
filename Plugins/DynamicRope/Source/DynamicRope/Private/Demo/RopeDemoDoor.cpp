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
	// 문짝 이동 보간에만 틱이 필요하다.
	PrimaryActorTick.bCanEverTick = true;

	Frame = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Frame"));
	SetRootComponent(Frame);
	// 문틀은 문짝보다 약간 크고 얇은 판(220x20x260cm). 엔진 큐브(100cm) 기준 배율.
	Frame->SetRelativeScale3D(FVector(2.2f, 0.2f, 2.6f));
	Frame->SetRelativeLocation(FVector(0.0f, 0.0f, 130.0f));
	Frame->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Frame->SetVisibility(false);
	Frame->SetMobility(EComponentMobility::Static);

	Leaf = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Leaf"));
	Leaf->SetupAttachment(Frame);
	// 부모 스케일의 역수를 곱해 절대 크기를 잡는다 — 문짝 200x30x250cm.
	Leaf->SetRelativeScale3D(FVector(2.0f / 2.2f, 0.3f / 0.2f, 2.5f / 2.6f));
	Leaf->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Leaf->SetCollisionObjectType(ECC_WorldStatic);
	// 열릴 때 실제로 움직이므로 Movable이어야 한다(Static이면 이동이 반영되지 않는다).
	Leaf->SetMobility(EComponentMobility::Movable);

	// 콘텐츠 의존을 만들지 않으려고 엔진 기본 셰이프만 쓴다(플러그인 → /Game 참조 금지).
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

	// 레벨에 물체를 미리 얹어 둔 배치도 지원하려면 시작 시 한 번 평가해야 한다.
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

	// 부모 스케일이 걸려 있으므로 월드 cm 오프셋을 상대 좌표로 환산한다.
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

	// 속도는 월드 기준 cm/s이므로 상대 좌표 보간 속도로 환산한다(스케일 축이 다르면 지배 축 기준).
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

	// 0 = 연결된 판 전부. 유효한 것만 센다.
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
	// 한 번 열렸고 bStayOpen이면 조건이 깨져도 유지한다(클리어는 되돌아가지 않는다).
	if (bOpen && bStayOpen)
	{
		return;
	}

	const int32 Required = GetRequiredPlateCount();
	// 연결된 판이 없으면 열리지 않는다 — 0 >= 0으로 열려 버리는 걸 막는다.
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
			// AddUnique — Plates에 직접 지정한 것과 겹쳐도 이중 구독이 되지 않게 한다.
			Plates.AddUnique(Plate);
		}
	}
}
