// Fill out your copyright notice in the Description page of Project Settings.

#include "RopeDragonFlightDemoComponent.h"

#include "RopeComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "UObject/UObjectIterator.h"

URopeDragonFlightDemoComponent::URopeDragonFlightDemoComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void URopeDragonFlightDemoComponent::BeginPlay()
{
	Super::BeginPlay();

	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	// 궤도 중심 = 시작 위치의 오른쪽 OrbitRadius 지점 → 시작 시 액터가 정확히 궤도 위에 있고
	// 초기 접선이 액터 전방과 일치한다(스폰 직후 방향 홱 돌아가는 것 방지).
	const FVector Start = Owner->GetActorLocation();
	OrbitCenter = Start + Owner->GetActorRightVector() * OrbitRadius;
	PreferredAltitude = static_cast<float>(Start.Z);
	Heading = Owner->GetActorForwardVector().GetSafeNormal();
}

void URopeDragonFlightDemoComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner || DeltaTime <= 0.0f)
	{
		return;
	}

	const FVector Location = Owner->GetActorLocation();

	// 조향 벡터 = 궤도 접선 + 반경 오차 복원 + 고도 복원. 절대 경로가 아니므로 테더/충돌로 밀린
	// 변위는 남고, 여기서부터 다시 궤도를 향해 돌아간다.
	FVector ToCenter = OrbitCenter - Location;
	ToCenter.Z = 0.0;
	const float CenterDistance = static_cast<float>(ToCenter.Size());
	const FVector ToCenterDir = (CenterDistance > 1.0f) ? ToCenter / CenterDistance : FVector::ForwardVector;
	// 중심이 오른쪽에 있을 때 전방이 접선이 되는 방향(위 BeginPlay 기하와 세트).
	const FVector Tangent = FVector::CrossProduct(ToCenterDir, FVector::UpVector);

	const float RadialError = FMath::Clamp((CenterDistance - OrbitRadius) / OrbitRadius, -1.0f, 1.0f);
	FVector Desired = Tangent + ToCenterDir * RadialError;

	// 고도 복원: 300cm 오차에서 최대 상승 성분에 도달하는 완만한 P 제어.
	const float AltitudeError = PreferredAltitude - static_cast<float>(Location.Z);
	Desired.Z = FMath::Clamp(AltitudeError / 300.0f, -1.0f, 1.0f) * MaxClimbRatio;
	Desired = Desired.GetSafeNormal();
	if (Desired.IsNearlyZero())
	{
		Desired = Heading;
	}

	const FVector OldHeading = Heading;
	Heading = FMath::VInterpNormalRotationTo(Heading, Desired, DeltaTime, TurnRateDeg);

	const float SpeedScale = IsWrappedByRope() ? WrappedSpeedScale : 1.0f;
	Owner->AddActorWorldOffset(Heading * FlightSpeed * SpeedScale * DeltaTime, /*bSweep*/ false);

	// 시각 뱅크: 수평 선회율(도/초)에 비례해 몸을 기울인다. 오른쪽 선회 = 오른쪽으로 기울기.
	const float TurnSign = FMath::Sign(static_cast<float>(FVector::CrossProduct(OldHeading, Heading).Z));
	const float TurnRateNow = FMath::RadiansToDegrees(
		FMath::Acos(FMath::Clamp(static_cast<float>(FVector::DotProduct(OldHeading, Heading)), -1.0f, 1.0f))) / DeltaTime;
	const float DesiredBank = TurnSign * FMath::Clamp(TurnRateNow / FMath::Max(TurnRateDeg, 1.0f), 0.0f, 1.0f) * BankAngleMax;
	CurrentBank = FMath::FInterpTo(CurrentBank, DesiredBank, DeltaTime, 3.0f);

	FRotator FaceRotation = Heading.Rotation();
	FaceRotation.Roll = CurrentBank;
	Owner->SetActorRotation(FaceRotation);
}

USkeletalMeshComponent* URopeDragonFlightDemoComponent::ResolveMesh() const
{
	const AActor* Owner = GetOwner();
	return Owner ? Owner->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
}

URopeComponent* URopeDragonFlightDemoComponent::FindRopeWrappingUs() const
{
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!MyMesh)
	{
		return nullptr;
	}

	// 데모 규모(월드당 로프 몇 개)라 전수 순회로 충분하다(RopeRagdollDemoComponent와 동일 패턴).
	for (TObjectIterator<URopeComponent> It; It; ++It)
	{
		URopeComponent* Rope = *It;
		if (!IsValid(Rope) || Rope->GetWorld() != GetWorld() || !Rope->IsRegistered())
		{
			continue;
		}
		if (Rope->GetPhase() == ERopePhase::Wrapped && Rope->GetWrappedMesh() == MyMesh)
		{
			return Rope;
		}
	}
	return nullptr;
}
