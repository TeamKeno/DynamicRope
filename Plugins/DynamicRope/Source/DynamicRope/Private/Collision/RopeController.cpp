// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeController.h"
#include "Collision/RopeStaticBodyProvider.h"

ARopeController::ARopeController()
{
	// The manager actor. It does not tick, since the subsystem pulls from the providers once per frame, and it is not
	// replicated: the client and the server each spawn their own, which is correct because the server needs static
	// collision even for a CPU fallback simulation.
	PrimaryActorTick.bCanEverTick = false;
	SetReplicates(false);

	// The static body provider is a non-scene UActorComponent and cannot be the root component, so it is attached as a
	// subobject alone. The component registers itself with URopeSimSubsystem on BeginPlay, since it provides world static colliders.
	StaticBodyProvider = CreateDefaultSubobject<URopeStaticBodyProvider>(TEXT("StaticBodyProvider"));
}
