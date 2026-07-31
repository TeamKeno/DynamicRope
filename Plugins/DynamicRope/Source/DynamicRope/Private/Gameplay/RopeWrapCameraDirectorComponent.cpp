// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeWrapCameraDirectorComponent.h"

#include "DynamicRopeLog.h"
#include "RopeComponent.h"
#include "Gameplay/RopeWrapCameraComponent.h"
#include "Gameplay/RopeWrapCameraRig.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "TimerManager.h"

URopeWrapCameraDirectorComponent::URopeWrapCameraDirectorComponent()
{
	// The shot is driven by the rope events plus a timer. The tick exists only to notice a broken
	// precondition while a shot is playing, and is disabled the rest of the time. PostPhysics puts it after
	// the target's bones reach their final pose, which is what the marker is attached to.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void URopeWrapCameraDirectorComponent::BeginPlay()
{
	Super::BeginPlay();

	if (URopeComponent* Resolved = ResolveRope())
	{
		Resolved->OnRopeWrapped.AddDynamic(this, &URopeWrapCameraDirectorComponent::HandleRopeWrapped);
		Resolved->OnRopeReleased.AddDynamic(this, &URopeWrapCameraDirectorComponent::HandleRopeReleased);
	}
	else
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] RopeWrapCameraDirector: no rope component found - the wrap camera is disabled."),
			*GetNameSafe(GetOwner()));
	}
}

void URopeWrapCameraDirectorComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeComponent* Resolved = Rope.Get())
	{
		Resolved->OnRopeWrapped.RemoveDynamic(this, &URopeWrapCameraDirectorComponent::HandleRopeWrapped);
		Resolved->OnRopeReleased.RemoveDynamic(this, &URopeWrapCameraDirectorComponent::HandleRopeReleased);
	}
	// Hand the view back. This is what stops a removed component, or a level teardown, from leaving the
	// player looking through a rig that is about to disappear; the internal guard makes it a no-op when no
	// shot is playing, and it only cleans up while the world is tearing down.
	FinishCamera();

	Super::EndPlay(EndPlayReason);
}

void URopeWrapCameraDirectorComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	ARopeWrapCameraRig* Rig = ActiveRig.Get();
	APlayerController* PC = ActiveController.Get();
	// Any broken precondition ends the shot early: the target destroyed mid-cut, which takes its marker with
	// it, a lost controller, or the view target taken by something else such as a cinematic. Ending here is
	// what turns those into a blend back to the wielder instead of a frozen camera.
	if (!Rig || !PC || !Rig->GetBoundCamera() || PC->GetViewTarget() != Rig)
	{
		FinishCamera();
	}
}

void URopeWrapCameraDirectorComponent::HandleRopeWrapped(const FRopeWrappedEventInfo& Info)
{
	// A second wrap while a shot is already running is ignored rather than restarting it: recutting mid-blend
	// reads as a glitch, and the running shot is already showing the same catch.
	if (!bEnabled || IsPlaying())
	{
		return;
	}

	USceneComponent* WrappedMesh = Info.Mesh.Get();
	AActor* TargetActor = WrappedMesh ? WrappedMesh->GetOwner() : nullptr;
	if (!TargetActor)
	{
		return;
	}
	if (bSkipSelfWrap && TargetActor == GetOwner())
	{
		UE_LOG(LogDynamicRope, Verbose,
			TEXT("[%s] RopeWrapCameraDirector: self wrap, no camera cut."), *GetNameSafe(GetOwner()));
		return;
	}

	// Bones is empty only where the wrap could not attribute one, as on a static target; the dominant bone
	// alone still lets a filtered marker match.
	TArray<FName> WrappedBones = Info.Bones;
	if (WrappedBones.IsEmpty() && !Info.Bone.IsNone())
	{
		WrappedBones.Add(Info.Bone);
	}

	// GetSocketLocation falls back to the component's own location when the name is not a bone or socket, so
	// this is the wrap position for skeletal and static targets alike.
	const FVector WrapLocation = WrappedMesh->GetSocketLocation(Info.Bone);
	URopeWrapCameraComponent* Marker =
		URopeWrapCameraComponent::SelectForWrap(TargetActor, WrappedBones, WrapLocation);
	if (!Marker)
	{
		UE_LOG(LogDynamicRope, Verbose,
			TEXT("[%s] RopeWrapCameraDirector: '%s' carries no wrap camera accepting bone '%s' - no cut."),
			*GetNameSafe(GetOwner()), *GetNameSafe(TargetActor), *Info.Bone.ToString());
		return;
	}

	BeginCamera(Marker);
}

void URopeWrapCameraDirectorComponent::HandleRopeReleased(FName Bone, ERopeReleaseReason Reason)
{
	// Every release reason ends the shot, a manual release, a cut and a pre-commit abort alike: once the rope
	// is off the target there is nothing left to show.
	FinishCamera();
}

void URopeWrapCameraDirectorComponent::StopCamera()
{
	FinishCamera();
}

URopeWrapCameraComponent* URopeWrapCameraDirectorComponent::GetActiveCamera() const
{
	const ARopeWrapCameraRig* Rig = ActiveRig.Get();
	return Rig ? Rig->GetBoundCamera() : nullptr;
}

URopeComponent* URopeWrapCameraDirectorComponent::ResolveRope()
{
	if (!Rope)
	{
		if (const AActor* Owner = GetOwner())
		{
			Rope = Owner->FindComponentByClass<URopeComponent>();
		}
	}
	return Rope;
}

APlayerController* URopeWrapCameraDirectorComponent::ResolveController() const
{
	const APawn* Pawn = Cast<APawn>(GetOwner());
	return Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
}

void URopeWrapCameraDirectorComponent::BeginCamera(URopeWrapCameraComponent* Marker)
{
	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	APlayerController* PC = ResolveController();
	if (!Marker || !Owner || !World || !PC)
	{
		return;
	}
	// Do not take a view target somebody else is holding, whether a cinematic or the ragdoll follow camera of
	// URopeRagdollResponseComponent. The matching guard on the way out means the two can never stomp each
	// other: whoever took it last keeps it, and neither restores over a foreign view target.
	if (PC->GetViewTarget() != Owner)
	{
		UE_LOG(LogDynamicRope, Verbose,
			TEXT("[%s] RopeWrapCameraDirector: the view target is already held by '%s' - no cut."),
			*GetNameSafe(Owner), *GetNameSafe(PC->GetViewTarget()));
		return;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.Owner = Owner;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	ARopeWrapCameraRig* Rig = World->SpawnActor<ARopeWrapCameraRig>(
		Marker->GetComponentLocation(), Marker->GetComponentRotation(), SpawnParams);
	if (!Rig)
	{
		return;
	}
	Rig->Bind(Marker);
	PC->SetViewTarget(Rig, Marker->BlendIn);

	ActiveRig = Rig;
	ActiveController = PC;
	// Read now, not at the end: the target actor can be destroyed while the shot runs, and the blend back
	// still has to use the values the designer authored on the marker.
	PendingBlendOut = Marker->BlendOut;

	// The hold begins once the blend in has landed, so HoldTime means time actually spent on the shot rather
	// than being partly eaten by the blend. The floor keeps SetTimer valid when both are zero.
	const float Duration =
		FMath::Max(Marker->BlendIn.BlendTime, 0.0f) + FMath::Max(Marker->HoldTime, 0.0f);
	World->GetTimerManager().SetTimer(HoldTimer, this, &URopeWrapCameraDirectorComponent::FinishCamera,
		FMath::Max(Duration, 0.01f), false);
	SetComponentTickEnabled(true);

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeWrapCameraDirector: cutting to '%s' on '%s' for %.2fs."),
		*GetNameSafe(Owner), *GetNameSafe(Marker), *GetNameSafe(Marker->GetOwner()), Duration);

	OnWrapCameraBegin.Broadcast(Marker);
}

void URopeWrapCameraDirectorComponent::FinishCamera()
{
	SetComponentTickEnabled(false);

	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	if (World)
	{
		World->GetTimerManager().ClearTimer(HoldTimer);
	}

	ARopeWrapCameraRig* Rig = ActiveRig.Get();
	APlayerController* PC = ActiveController.Get();
	ActiveRig.Reset();
	ActiveController.Reset();
	if (!Rig)
	{
		return;
	}

	const bool bWorldAlive = World && !World->bIsTearingDown;
	// Blend back only while we are still the view target, respecting anything that took it in the meantime.
	// If the wielder is dying there is no destination, so the rig is left to the camera manager's fallback.
	if (PC && bWorldAlive && PC->GetViewTarget() == Rig
		&& IsValid(Owner) && !Owner->IsActorBeingDestroyed())
	{
		PC->SetViewTarget(Owner, PendingBlendOut);
	}
	// The outgoing view target has to survive until the blend finishes, so the rig is given a lifetime rather
	// than being destroyed immediately. It stays attached to the marker meanwhile, so the shot the blend
	// starts from keeps tracking the target instead of freezing.
	if (bWorldAlive)
	{
		Rig->SetLifeSpan(FMath::Max(PendingBlendOut.BlendTime, 0.0f) + 0.5f);
	}
	else
	{
		Rig->Destroy();
	}

	OnWrapCameraEnd.Broadcast();
}
