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
	// 연출 보간과 점유 자격 재평가(랙돌 복귀 감지)에 틱이 필요하다.
	PrimaryActorTick.bCanEverTick = true;

	Frame = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Frame"));
	SetRootComponent(Frame);
	// 프레임은 판보다 약간 넓고 아주 얇다 — 120x120x4cm.
	Frame->SetRelativeScale3D(FVector(1.2f, 1.2f, 0.04f));
	Frame->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Frame->SetCollisionObjectType(ECC_WorldStatic);
	Frame->SetMobility(EComponentMobility::Static);

	Pad = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Pad"));
	Pad->SetupAttachment(Frame);
	// 부모(Frame)가 눌린 스케일이라 자식 스케일은 그 역수를 곱해 절대 크기를 맞춘다.
	// 판 = 100x100x10cm.
	Pad->SetRelativeScale3D(FVector(1.0f / 1.2f, 1.0f / 1.2f, 0.1f / 0.04f));
	Pad->SetRelativeLocation(FVector(0.0f, 0.0f, 7.0f));
	// 얹힌 물체가 판을 뚫지 않도록 막되, 판 자체는 연출로만 움직이므로 Movable.
	Pad->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Pad->SetCollisionObjectType(ECC_WorldStatic);
	Pad->SetMobility(EComponentMobility::Movable);

	Trigger = CreateDefaultSubobject<UBoxComponent>(TEXT("Trigger"));
	Trigger->SetupAttachment(Frame);
	// 판 위 공간을 덮는 감지 볼륨(월드 기준 100x100x120cm). 부모 스케일 보정 포함.
	Trigger->SetBoxExtent(FVector(50.0f, 50.0f, 60.0f));
	Trigger->SetRelativeScale3D(FVector(1.0f / 1.2f, 1.0f / 1.2f, 1.0f / 0.04f));
	Trigger->SetRelativeLocation(FVector(0.0f, 0.0f, 60.0f));
	Trigger->SetMobility(EComponentMobility::Movable);
	// 질의 전용 + 관심 채널만 오버랩. 랙돌 바디는 PhysicsBody, 물리 스태틱 메시는 보통 WorldDynamic이다.
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

	// 콘텐츠 의존을 만들지 않으려고 엔진 기본 셰이프만 쓴다(플러그인 → /Game 참조 금지).
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

	// BeginPlay 시점에 이미 겹쳐 있던 액터(레벨에 미리 얹어 둔 물체)도 세어 준다.
	RefreshPressedState();
}

void ARopeDemoPressurePlate::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 랙돌이 판 위에서 애니메이션으로 복귀하면 오버랩 이벤트 없이 자격만 사라진다 — 매 틱 재평가.
	RefreshPressedState();

	// 눌림 연출: 판의 상대 Z를 목표까지 등속 보간.
	const float TargetOffset = bPressed ? -PressDepth : 0.0f;
	if (!FMath::IsNearlyEqual(CurrentPadOffset, TargetOffset) && Pad)
	{
		CurrentPadOffset = FMath::FInterpConstantTo(CurrentPadOffset, TargetOffset, DeltaSeconds, PressSpeed);
		// Pad는 눌린 부모 스케일(Frame Z=0.04) 아래에 있으므로, 월드 cm를 상대 좌표로 환산해 적용한다.
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

	// 랙돌은 본 바디 수만큼 이벤트가 오므로 액터 단위로 합산한다.
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

	// 물리 시뮬 중인 바디가 하나라도 있으면 인정 — 물리 스태틱 메시, 랙돌(부분 랙돌 포함) 둘 다 걸린다.
	// 걸어 올라선 캐릭터는 캡슐이 시뮬을 안 하므로 여기서 걸러진다.
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
	// 파괴된 액터를 정리하면서 자격을 다시 센다.
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
