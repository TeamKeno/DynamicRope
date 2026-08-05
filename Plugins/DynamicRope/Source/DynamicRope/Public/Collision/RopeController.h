// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The rope manager actor, of which one is spawned per world. It hosts the URopeStaticBodyProvider
// used for static world collision, and URopeSimSubsystem spawns it, or the subclass named in the
// project settings, automatically when a game or PIE world starts. That guarantees exactly one per
// world without placing a provider component by hand in every level.
// A project can subclass it to adjust the provider's IgnoredComponents; the collider budget and the
// plane limit are managed globally under Project Settings > Dynamic Rope.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeController.generated.h"

class URopeStaticBodyProvider;

UCLASS(ClassGroup = (DynamicRope))
class DYNAMICROPE_API ARopeController : public AActor
{
	GENERATED_BODY()

public:
	ARopeController();

	/** Supplies the world's static simple collision to the rope as colliders. Tunable from the details
	 *  panel or a subclass. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Collision")
	TObjectPtr<URopeStaticBodyProvider> StaticBodyProvider;
};
