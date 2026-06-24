// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — 실험용이며 출시 대상 아님. Docs/PoC/01_PostWrapModel.md 참고.

#include "PoC/RopePoCCapsuleActor.h"

#include "Components/CapsuleComponent.h"

ARopePoCCapsuleActor::ARopePoCCapsuleActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	Capsule = CreateDefaultSubobject<UCapsuleComponent>(TEXT("Capsule"));
	SetRootComponent(Capsule);

	// 시각화 전용 — rope solver가 지오메트리를 직접 읽으므로, 이 컴포넌트
	// 자체는 물리 collision을 하지 않는다.
	Capsule->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Capsule->SetCapsuleSize(Radius, HalfHeight);
	Capsule->ShapeColor = FColor::Green;
	Capsule->bDrawOnlyIfSelected = false; // 에디터에서 항상 보임
	Capsule->SetHiddenInGame(false);
}

void ARopePoCCapsuleActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	if (Capsule)
	{
		Capsule->SetCapsuleSize(Radius, HalfHeight);
	}
}

void ARopePoCCapsuleActor::BeginPlay()
{
	Super::BeginPlay();
	RestTransform = GetActorTransform();
	SwingElapsed = 0.0f;
}

#if WITH_EDITOR
void ARopePoCCapsuleActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// swing을 capsule이 현재 위치한 곳에 다시 anchor한다.
	RestTransform = GetActorTransform();
	SwingElapsed = 0.0f;
}
#endif

void ARopePoCCapsuleActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 1) Base 포즈(드래그 안 된 상태): 손으로 배치한 rest transform이거나, 그것의 swing.
	FVector BaseLoc;
	FQuat BaseRot;

	if (!bAutoSwing)
	{
		// 현재 drag offset을 제거해 rest 포즈를 복원한다. 그래야 드래그가 rest anchor를
		// 떠밀지 않고, spring이 항상 배치된 자리로 되돌릴 수 있다.
		BaseLoc = GetActorLocation() - DragOffset;
		BaseRot = GetActorQuat();
		RestTransform.SetLocation(BaseLoc);
		RestTransform.SetRotation(BaseRot);
		SwingElapsed = 0.0f;
	}
	else
	{
		SwingElapsed += DeltaSeconds;

		const float Period = FMath::Max(SwingPeriod, 0.05f);
		const float Phase = FMath::Sin(2.0f * PI * SwingElapsed / Period);
		const float AngleRad = FMath::DegreesToRadians(SwingAngleDeg * Phase);

		const FVector AxisWorld = RestTransform.TransformVectorNoScale(SwingAxis).GetSafeNormal(1e-4f, FVector::RightVector);
		const FQuat SwingQuat(AxisWorld, AngleRad);

		const FVector PivotWorld = RestTransform.TransformPosition(SwingPivotOffset);
		const FVector RestLoc = RestTransform.GetLocation();

		BaseLoc = PivotWorld + SwingQuat.RotateVector(RestLoc - PivotWorld);
		BaseRot = SwingQuat * RestTransform.GetRotation();
	}

	// 2) S4: rope의 pull을 soft body로 integration한다 — impulse → 속도, rest로 spring
	//    복귀, damping. limb가 끌려가되 pull이 약해지면 회복하게 한다.
	if (bDraggable)
	{
		const float Dt = FMath::Min(DeltaSeconds, 1.0f / 30.0f);
		DragVelocity += PendingImpulse / FMath::Max(Mass, 0.1f);
		DragVelocity += -ReturnStiffness * DragOffset * Dt;   // rest 포즈 쪽으로 spring
		DragVelocity *= FMath::Exp(-DragDamping * Dt);        // 속도 damping
		DragOffset += DragVelocity * Dt;
	}
	else
	{
		DragOffset = FVector::ZeroVector;
		DragVelocity = FVector::ZeroVector;
	}
	PendingImpulse = FVector::ZeroVector;

	// 3) 최종 포즈 = base 포즈 + 누적된 drag.
	SetActorLocationAndRotation(BaseLoc + DragOffset, BaseRot);
}

void ARopePoCCapsuleActor::GetCapsuleSegment(FVector& OutA, FVector& OutB, float& OutRadius) const
{
	OutRadius = Radius;

	const FVector Center = GetActorLocation();
	const FVector Up = GetActorQuat().GetAxisZ(); // capsule axis는 로컬 Z

	// 내부 segment half-length: 두 반구형 cap 사이의 cylinder 부분.
	const float SegmentHalf = FMath::Max(0.0f, HalfHeight - Radius);
	OutA = Center - Up * SegmentHalf;
	OutB = Center + Up * SegmentHalf;
}

void ARopePoCCapsuleActor::GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const
{
	FRopeCapsule Cap;
	GetCapsuleSegment(Cap.A, Cap.B, Cap.Radius);
	OutCapsules.Add(Cap);
}

void ARopePoCCapsuleActor::ApplyRopeReaction(const FVector& WorldImpulse, const FVector& /*WorldLocation*/)
{
	if (!bDraggable)
	{
		return;
	}
	// 누적만 한다; Tick이 integration한다(rope와 capsule 사이의 tick 순서는 정의되지 않음).
	PendingImpulse += WorldImpulse;
}
