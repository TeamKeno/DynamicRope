// Copyright Epic Games, Inc. All Rights Reserved.
//
// Demo fall-recovery volume, used in place of Kill Z. Lay it under the level and anything that
// touches it is returned rather than destroyed:
//  - Player pawns go back to a PlayerStart, using the game mode's own selection rules.
//  - Physics props go back to the transform recorded for them during BeginPlay.
//
// The engine has its own handling for falling out of the world, through Kill Z and world bounds
// checks in the world settings. Its default, AActor::FellOutOfWorld, destroys the actor: a pawn
// simply disappears with no respawn, since GameModeBase does not restart automatically, and a puzzle
// prop that falls is gone for good, making that room impossible to clear. The demo wants recovery
// rather than destruction, hence this separate volume.
//
// Keep Kill Z below this volume and use it purely as a last-resort safety net, so anything that
// escapes the volume does not fall forever.
//
// Ropes involved with the actor are released before it is returned; otherwise a wrapped rope is left
// stretched across the map. A limp target is recovered from its ragdoll first, because while limp the
// bone bodies live in world space and moving the actor alone would leave the mesh behind. That is
// only possible when a URopeRagdollResponseComponent is present; without one, only the actor moves.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoRespawnVolume.generated.h"

class UBoxComponent;

/** Fired when something is returned, exposed for HUD messages and counters. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoRespawnSignature,
	ARopeDemoRespawnVolume*, Volume, AActor*, RespawnedActor);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Respawn Volume"))
class DYNAMICROPE_API ARopeDemoRespawnVolume : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoRespawnVolume();

	//~ AActor
	virtual void BeginPlay() override;

	/** Returns this actor immediately, without it having to touch the volume. For console commands,
	 *  Blueprint and reset buttons. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	bool RespawnActor(AActor* Target);

	/** Returns every tracked prop to its recorded transform, which resets the room. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Demo")
	void RespawnAllProps();

	/** Broadcast when a return happens. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoRespawnSignature OnActorRespawned;

	/** Whether to return pawns to a PlayerStart. Turn it off to handle props only. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bRespawnPawns = true;

	/**
	 * Whether to track physically simulating props automatically during BeginPlay. Turn it off to
	 * track only actors carrying PropTag.
	 * An untracked actor that touches the volume is ignored rather than destroyed.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bAutoTrackPhysicsProps = true;

	/** Actors carrying this tag are tracked whether or not they simulate physics. Leave it empty to
	 *  track nothing by tag. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	FName PropTag = TEXT("RopeDemoProp");

	/** When set, pawns are returned to this actor's transform instead of a PlayerStart, which gives
	 *  each room its own checkpoint. */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo")
	TObjectPtr<AActor> PawnRespawnPointOverride = nullptr;

	/** Release the ropes entangled with the target just before returning it. Turn it off and you will
	 *  see ropes stretched across the map. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bReleaseRopesOnRespawn = true;

protected:
	/** Fall detection volume, placed to cover a wide area under the level. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UBoxComponent> Trigger = nullptr;

private:
	UFUNCTION()
	void HandleBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	/** Records the starting transform of everything tracked, whether by tag or by simulating
	 *  physics. */
	void TrackProps();

	/** Returns a pawn to a PlayerStart, or to the override. */
	bool RespawnPawn(APawn* Pawn);

	/** Returns a tracked prop to its recorded transform. */
	bool RespawnProp(AActor* Prop, const FTransform& StartTransform);

	/** Releases every rope this actor threw, as a wielder, and every rope wrapping it. */
	void ReleaseRopesInvolving(AActor* Actor);

	/** Zeroes physics velocities so nothing flies off again on leftover momentum straight after
	 *  returning. */
	static void ZeroPhysicsVelocities(AActor* Actor);

	/** Starting transforms of the props, keyed weakly so a destroyed actor is safe. */
	TMap<TWeakObjectPtr<AActor>, FTransform> TrackedProps;
};
