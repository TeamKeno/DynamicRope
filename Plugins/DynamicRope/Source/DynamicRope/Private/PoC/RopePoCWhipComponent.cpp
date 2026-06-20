// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. See Docs/PoC/01_PostWrapModel.md.

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
	// Tick is used only to retry the input binding until the pawn's InputComponent exists.
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
		// Nothing left to retry — input is bound (or auto-bind disabled).
		PrimaryComponentTick.SetTickFunctionEnable(false);
	}
}

void URopePoCWhipComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// The pawn's InputComponent may not exist until it is possessed; retry until it does.
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

	// Pin the rope's start to the hand; leave the far end free so it trails like a whip.
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
