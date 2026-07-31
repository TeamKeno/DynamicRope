// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeWrapCameraRig.h"

#include "Gameplay/RopeWrapCameraComponent.h"
#include "Components/SceneComponent.h"

ARopeWrapCameraRig::ARopeWrapCameraRig()
{
	// Nothing here drives itself: the transform comes from the attachment to the marker and the view comes
	// from CalcCamera, so the actor never needs to tick or to collide.
	PrimaryActorTick.bCanEverTick = false;
	SetActorEnableCollision(false);

	// A root is required to attach to the marker.
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));

	// There is no camera component to find, and CalcCamera is overridden anyway, so the engine's per-frame
	// search over the component list is pure waste here.
	bFindCameraComponentWhenViewTarget = false;
}

void ARopeWrapCameraRig::Bind(URopeWrapCameraComponent* Marker)
{
	BoundCamera = Marker;
	if (Marker)
	{
		AttachToComponent(Marker, FAttachmentTransformRules::SnapToTargetIncludingScale);
	}
}

void ARopeWrapCameraRig::CalcCamera(float DeltaTime, FMinimalViewInfo& OutResult)
{
	// APlayerCameraManager::UpdateViewTargetInternal calls this on whatever the view target is, so
	// delegating is what lets an actor with no camera component of its own frame exactly what the marker
	// authored. UCameraComponent::GetCameraView does not test IsActive, which is why the marker can stay
	// permanently deactivated and never compete with the target actor's own camera.
	if (URopeWrapCameraComponent* Marker = BoundCamera.Get())
	{
		Marker->GetCameraView(DeltaTime, OutResult);
		return;
	}
	Super::CalcCamera(DeltaTime, OutResult);
}
