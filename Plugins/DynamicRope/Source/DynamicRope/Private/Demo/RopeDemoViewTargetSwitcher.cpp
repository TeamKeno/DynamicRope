// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Demo/RopeDemoViewTargetSwitcher.h"

#include "DynamicRopeLog.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

void FRopeDemoViewTargetSwitcher::Activate(AActor* DemoActor, AActor* Target, float BlendTime)
{
	if (IsActive() || !DemoActor)
	{
		return;
	}
	// Only a player-controlled pawn gets the ride camera; an AI or prop target leaves the player's
	// view alone. The skips are logged because a demo where the camera "does nothing" is otherwise
	// indistinguishable from a bug.
	APawn* Pawn = Cast<APawn>(Target);
	APlayerController* PC = Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
	if (!PC || !PC->PlayerCameraManager)
	{
		UE_LOG(LogDynamicRope, Log,
			TEXT("[%s] view switch skipped: target '%s' is not a player-controlled pawn."),
			*GetNameSafe(DemoActor), *GetNameSafe(Target));
		return;
	}
	// Without a camera component the view target would fall back to the actor origin, which reads as
	// a bug on screen, so the switch requires one.
	if (!DemoActor->FindComponentByClass<UCameraComponent>())
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] view switch skipped: no camera component on the view-target actor."),
			*GetNameSafe(DemoActor));
		return;
	}
	// A view already taken by something else, such as a cinematic, is respected — with one exception:
	// a view target owned by the pawn itself, such as the camera the ragdoll view-target follow
	// spawns, counts as the pawn's own camera rig and is taken over. That follow detects losing the
	// view and cleans itself up, so the two never fight; without the exception a player already
	// ragdolled by the time the ropes fire could never get the demo camera at all.
	AActor* CurrentView = PC->GetViewTarget();
	if (CurrentView != Pawn && (!CurrentView || CurrentView->GetOwner() != Pawn))
	{
		UE_LOG(LogDynamicRope, Log,
			TEXT("[%s] view switch skipped: the player is already viewing '%s', not their pawn."),
			*GetNameSafe(DemoActor), *GetNameSafe(CurrentView));
		return;
	}

	Controller = PC;
	ReturnTarget = Pawn;
	DemoViewTarget = DemoActor;
	PC->SetViewTargetWithBlend(DemoActor, FMath::Max(BlendTime, 0.0f), VTBlend_Cubic);
}

void FRopeDemoViewTargetSwitcher::Deactivate(float BlendTime)
{
	APlayerController* PC = Controller.Get();
	AActor* Demo = DemoViewTarget.Get();
	AActor* Return = ReturnTarget.Get();
	Controller.Reset();
	ReturnTarget.Reset();
	DemoViewTarget.Reset();

	if (!PC || !Demo)
	{
		return;
	}
	UWorld* World = PC->GetWorld();
	if (!World || World->bIsTearingDown)
	{
		return;
	}
	// Hand back only while the demo camera still owns the view, respecting anything that took it in
	// the meantime. With the original pawn gone, the controller's current pawn is the fallback; with
	// neither, the view is left on the demo actor rather than sent nowhere.
	if (PC->GetViewTarget() != Demo)
	{
		return;
	}
	AActor* Destination = Return ? Return : static_cast<AActor*>(PC->GetPawn());
	if (!Destination)
	{
		return;
	}
	PC->SetViewTargetWithBlend(Destination, FMath::Max(BlendTime, 0.0f), VTBlend_Cubic);
}
