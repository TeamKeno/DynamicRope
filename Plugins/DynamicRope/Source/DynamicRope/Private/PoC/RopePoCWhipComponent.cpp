// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — 실험용이며 출시 대상 아님. Docs/PoC/01_PostWrapModel.md 참고.

#include "PoC/RopePoCWhipComponent.h"
#include "PoC/RopePoCActor.h"

#include "GameFramework/Character.h"
#include "GameFramework/Pawn.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/InputComponent.h"
#include "Animation/AnimInstance.h"
#include "Animation/AnimMontage.h"

URopePoCWhipComponent::URopePoCWhipComponent()
{
	// Tick은 pawn의 InputComponent가 생길 때까지 input binding을 재시도하는 데만 쓰인다.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

USkeletalMeshComponent* URopePoCWhipComponent::GetOwnerMesh() const
{
	if (const ACharacter* Char = Cast<ACharacter>(GetOwner()))
	{
		return Char->GetMesh();
	}
	if (const AActor* Owner = GetOwner())
	{
		return Owner->FindComponentByClass<USkeletalMeshComponent>();
	}
	return nullptr;
}

void URopePoCWhipComponent::BeginPlay()
{
	Super::BeginPlay();

	AttachRopeToHand();

	if (!bAutoBindLeftMouse || TryBindLeftMouse())
	{
		// 더 재시도할 것이 없음 — input이 bind됨(또는 auto-bind 비활성화).
		PrimaryComponentTick.SetTickFunctionEnable(false);
	}
}

void URopePoCWhipComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// pawn의 InputComponent는 possess되기 전까지 없을 수 있으니, 생길 때까지 재시도한다.
	if (TryBindLeftMouse())
	{
		PrimaryComponentTick.SetTickFunctionEnable(false);
	}
}

bool URopePoCWhipComponent::TryBindLeftMouse()
{
	if (bLeftMouseBound)
	{
		return true;
	}

	APawn* Pawn = Cast<APawn>(GetOwner());
	if (!Pawn || !Pawn->InputComponent)
	{
		return false;
	}

	Pawn->InputComponent->BindKey(EKeys::LeftMouseButton, IE_Pressed, this, &URopePoCWhipComponent::Swing);
	bLeftMouseBound = true;
	return true;
}

void URopePoCWhipComponent::AttachRopeToHand()
{
	USkeletalMeshComponent* Mesh = GetOwnerMesh();
	if (!Mesh)
	{
		return;
	}

	if (!Rope && bSpawnRopeIfMissing)
	{
		if (UWorld* World = GetWorld())
		{
			FActorSpawnParameters Params;
			Params.Owner = GetOwner();
			Rope = World->SpawnActor<ARopePoCActor>(
				ARopePoCActor::StaticClass(), Mesh->GetSocketTransform(HandSocketName), Params);
		}
	}

	if (!Rope)
	{
		return;
	}

	Rope->AttachToComponent(Mesh, FAttachmentTransformRules::SnapToTargetNotIncludingScale, HandSocketName);

	// rope의 start를 손에 pin하고, 먼 쪽 끝은 free로 두어 whip처럼 따라 휘날리게 한다.
	Rope->bPinStart = true;
	Rope->bPinEnd = false;
}

void URopePoCWhipComponent::Swing()
{
	USkeletalMeshComponent* Mesh = GetOwnerMesh();
	if (!Mesh || !SwingMontage)
	{
		return;
	}

	if (UAnimInstance* Anim = Mesh->GetAnimInstance())
	{
		Anim->Montage_Play(SwingMontage, MontagePlayRate);
	}
}
