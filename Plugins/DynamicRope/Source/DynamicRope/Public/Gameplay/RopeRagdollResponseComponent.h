// Copyright Epic Games, Inc. All Rights Reserved.
//
// Opt-in ragdoll response for a skeletal target wrapped by a rope. Add it to anything wrappable, a
// mannequin, an animal, a dragon, or any actor with a skeletal mesh, and it provides:
//  - bRagdollOnWrapped, on by default: going limp automatically RagdollOnWrappedDelay seconds after
//    a rope wraps this mesh. With bOnlyBelowWrappedBone, only the bones below the wrapped one
//    simulate.
//  - bRecoverRagdollOnRopeRelease, on by default: a ragdoll entered automatically recovers by itself
//    once the rope that wrapped it releases, so wrapping knocks the target down and releasing stands
//    it back up. A ragdoll entered manually is unaffected by rope releases.
//  - EnterRagdoll, EnterPartialRagdoll and RecoverFromRagdoll, the explicit API for Blueprint and
//    code.
//
// Several ropes can wrap one target at once, for example binding both arms, so active engagements
// are counted as a set of ropes and automatic recovery only happens once the last rope releases; see
// WrappingRopes.
//
// A target cannot know in advance which rope will wrap it, since cross-actor throws are common, so
// rather than walking every rope each frame it subscribes to the subsystem's central signals,
// URopeSimSubsystem::OnAnyRopeWrapped and OnAnyRopeReleased, and reacts when its own mesh is wrapped
// or released. Going limp is the game's responsibility rather than the plugin's, so this component
// is an opt-in convenience and reference implementation; a game is free to keep its own ragdoll
// logic instead.
//
// Console commands, in non-shipping builds only, applied to every actor in the world carrying this
// component:
//   Rope.Ragdoll             toggle a full ragdoll, recovering if already limp
//   Rope.Ragdoll spine_01    partial ragdoll below the given bone
//   Rope.Ragdoll.Recover     recover to animation
//   Rope.Ragdoll.Destroy     destroy the target actor, to check the rope releases when its wrap
//                            target is destroyed mid-wrap

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
// FTimerHandle, for the automatic transition delay member.
#include "Engine/TimerHandle.h"
// ERopeReleaseReason and FRopeWrappedEventInfo, the central signal payloads.
#include "Core/RopeLifecycleTypes.h"
// EMovementMode, for the movement mode saved before going limp.
#include "Engine/EngineTypes.h"
#include "RopeRagdollResponseComponent.generated.h"

class USkeletalMeshComponent;
class USceneComponent;
class URopeComponent;
class ACameraActor;
class APlayerController;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeRagdollResponseComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeRagdollResponseComponent();

	//~ UActorComponent. Subscribes to and unsubscribes from the central wrap and release signals. The
	//~ tick is only enabled while the ragdoll camera is following.
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** Go limp automatically when a rope wraps this actor's mesh. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll")
	bool bRagdollOnWrapped = true;

	/** Delay before going limp (s), which controls whether the target drops the instant it is wrapped
	 *  or a moment later. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (ClampMin = "0.0", Units = "s", EditCondition = "bRagdollOnWrapped", DisplayName = "Delay"))
	float RagdollOnWrappedDelay = 0.3f;

	/**
	 * Simulate only the bones below the wrapped one instead of the whole body.
	 *
	 * Known limitation, read before enabling: with this on, the tether cannot drag the target. A
	 * partial ragdoll keeps the capsule and movement alive by definition (see EnterPartialRagdoll), so
	 * only the wrapped bone simulates and its parent stays kinematic. That breaks the rope side in two
	 * separate ways:
	 *  1) Application: the tether servos only the wrapped bone, and the kinematic parent constraint,
	 *     which is effectively infinite mass, absorbs it instead of passing it on to the actor. The
	 *     arm flails to its joint limit while the character does not move. Active pull applies the
	 *     same force to the movement component as well, so it appears as the asymmetry "pulling with
	 *     the key works but the tether does not".
	 *  2) Mass and gating: the tether's endpoint resolution (ResolveTetherEndpoint) reports that
	 *     bone's own body mass, roughly 3 kg for a forearm, as the effective mass, even though being
	 *     bound to a kinematic parent makes the real effective mass infinite. That false value
	 *     corrupts both the share distribution, where a light target is assigned nearly all of it so
	 *     the wielder never yields, and the BinaryPullable drag test, which reports the target as
	 *     pullable and leaves the wielder entirely free. The overshoot then never closes and only the
	 *     rope stretches.
	 * Fixing this needs both layers: the tether must drive the character movement component as well,
	 * and a bone bound to a kinematic parent must report the character's mass. Fixing application
	 * alone would still distribute the shares wrongly. It is harmless while left at the default of
	 * false, since a full ragdoll simulates every body, has no kinematic anchor, and drags the whole
	 * body through its joints correctly.
	 *
	 * It is kept out of the details panel while that limitation stands (Blueprint only). The default
	 * is harmless, but a visible checkbox invites being turned on, and the symptom, "active pull works
	 * but the tether does not", is hard to trace back to its cause. Expose it again once both layers
	 * are fixed.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (EditCondition = "bRagdollOnWrapped"))
	bool bOnlyBelowWrappedBone = false;

	/**
	 * Recover an automatically entered ragdoll once the rope that wrapped it releases, on by default,
	 * so wrapping knocks the target down and releasing stands it back up. Turn it off to stay limp
	 * after a release, until RecoverFromRagdoll is called directly.
	 * It never applies to a ragdoll entered manually, so a rope cannot stand up a target the game put
	 * down for its own reasons.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll", meta = (DisplayName = "Recover On Release"))
	bool bRecoverRagdollOnRopeRelease = true;

	/**
	 * Have the player camera follow the ragdoll's anchor bone during a full ragdoll, on by default.
	 * Only active when the owner is a player-controlled pawn. A full ragdoll disables movement and the
	 * capsule, so the actor, and with it a spring arm camera, stays where it was and the screen would
	 * show an empty spot while the body is carried away. Instead a camera actor is spawned at the
	 * camera's point of view at that moment and made the view target with no blend, which is seamless,
	 * and it then follows the bone with lag interpolation while preserving the bone-to-camera offset.
	 * Unlike teleporting the actor, this touches neither the capsule nor the spring arm, so there is
	 * no feedback loop with the camera lag. Recovery blends back to the pawn camera. Partial ragdolls
	 * keep their capsule and movement and are not affected.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll", meta = (DisplayName = "Follow Camera While Ragdolled"))
	bool bViewTargetFollowRagdoll = true;

	/** Camera follow lag as an interpolation speed (1/s). 0 snaps with no lag; larger values follow
	 *  more tightly. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (ClampMin = "0.0", EditCondition = "bViewTargetFollowRagdoll", DisplayName = "Follow Camera Lag"))
	float FollowCameraLagSpeed = 5.0f;

	/** Blend time back to the pawn camera on recovery (s). 0 cuts immediately. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (ClampMin = "0.0", Units = "s", EditCondition = "bViewTargetFollowRagdoll", DisplayName = "Recover Camera Blend"))
	float RecoverCameraBlendTime = 0.5f;

	/**
	 * Move the capsule, and therefore the actor, horizontally to where the ragdoll came to rest when
	 * recovering from a full ragdoll, on by default. Movement is disabled while limp, so the capsule
	 * stays put while physics drags the mesh away, for example through a pull. Recovering without this
	 * would snap the mesh back to the original capsule, a large visible teleport; moving the capsule
	 * to the mesh, at the RecoverAnchorBoneName bone, makes that snap a visual no-op instead.
	 * Only the horizontal position changes: rotation and height are preserved, and ground snapping is
	 * left to the restored movement mode. Partial ragdolls are unaffected, since their mesh is never
	 * reset and there is no teleport.
	 *
	 * Blueprint only: turning it off simply restores the teleport artefact, so there is no reason to.
	 */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Ragdoll")
	bool bMoveCapsuleToMeshOnRecover = true;

	/**
	 * The reference bone for the capsule realignment above, the one that best represents where the
	 * ragdoll came to rest, usually near the centre of the body. The mannequin default is pelvis;
	 * change it to the main physics body bone for other skeletons such as dragons and animals. If the
	 * skeleton lacks it, realignment is skipped after a warning.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (EditCondition = "bMoveCapsuleToMeshOnRecover", DisplayName = "Recover Anchor Bone"))
	FName RecoverAnchorBoneName = TEXT("pelvis");

	/**
	 * How far below the anchor bone to search for the ground during capsule realignment (cm).
	 * Realigning horizontally alone assumes flat ground and discards Z, which sends a target recovered
	 * after vertical transport, such as a helicopter carry, back to its old height. With a search
	 * distance the resting height is included: a trace runs this far down from the anchor, and the
	 * capsule is stood on the ground if it is found, or recovers in a falling state from the anchor
	 * height if it is not, which covers being dropped in mid-air. 0 restores horizontal-only
	 * behaviour. Applies to ACharacter only; other actors always keep horizontal-only realignment.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (ClampMin = "0.0", Units = "cm", EditCondition = "bMoveCapsuleToMeshOnRecover", DisplayName = "Recover Ground Search"))
	float RecoverGroundSearchDistance = 500.0f;

	/** Collision profile applied to the mesh while limp. The mannequin default, CharacterMesh, has no
	 *  physics collision, so switching profiles is required. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (DisplayName = "Collision Profile"))
	FName RagdollCollisionProfileName = TEXT("Ragdoll");

	/**
	 * Whether to enable overlap events on the mesh while limp. ACharacter disables the mesh's
	 * GenerateOverlapEvents in its constructor, letting the capsule report on its behalf, and a full
	 * ragdoll disables that capsule too. Since the engine only generates an overlap when the flag is
	 * set on both components, a limp character would otherwise be invisible to trigger volumes such as
	 * pressure plates and damage volumes. With this on, the flag is enabled on entry and restored to
	 * its previous value on recovery. Turn it off when the target never interacts with triggers and
	 * has enough bodies that per-body overlap queries every frame are not worth the cost.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (DisplayName = "Generate Overlap Events"))
	bool bGenerateOverlapEventsWhileRagdolled = true;

	/**
	 * Enable continuous collision detection on every body while limp, on by default and restored on
	 * recovery. Thin floors, such as the engine's default Plane with zero collision thickness, are
	 * passed straight through when a ragdoll body crosses the surface within a single step, because no
	 * contact is ever generated; continuous detection sweeps between steps and stops that. It costs
	 * one sweep per body, so turn it off when that is not worth it for large numbers of ragdolls and
	 * solve the problem with floor thickness instead, preferring a cube over a plane.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll|Tuning", meta = (DisplayName = "Use CCD"))
	bool bUseCCDWhileRagdolled = true;

	/** Goes fully limp: the whole mesh simulates, and on an ACharacter the capsule collision and
	 *  movement stop.
	 *  bAutoRecoverOnRelease marks this as rope-driven, which makes it eligible for automatic recovery
	 *  once every rope wrapping it releases, subject to the bRecoverRagdollOnRopeRelease gate; the
	 *  snare's forced ragdoll uses this. The default of false marks it as a manual entry, which is
	 *  unaffected by rope releases. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Ragdoll")
	void EnterRagdoll(bool bAutoRecoverOnRelease = false);

	/** Goes partially limp: only the bodies below BoneName simulate, at blend weight 1, while the rest
	 *  keeps animating. bAutoRecoverOnRelease means the same as in EnterRagdoll. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Ragdoll")
	void EnterPartialRagdoll(FName BoneName, bool bAutoRecoverOnRelease = false);

	/** Recovers to animation. After a full ragdoll the mesh is returned to its original attachment and
	 *  relative transform, which produces an intentional pose pop. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Ragdoll")
	void RecoverFromRagdoll();

	/** Recovers a rope-driven ragdoll, one entered with bAutoRecoverOnRelease, but only while no rope
	 *  is wrapping this mesh. Automatic recovery only fires on a release event, so it never happens on
	 *  paths where the target went limp but the situation ended before any rope wrapped it, such as a
	 *  snare cancelled before it fires; this call closes that gap. It respects the same eligibility
	 *  rules as automatic recovery: a manually entered ragdoll and bRecoverRagdollOnRopeRelease set to
	 *  false are left alone, and while another rope is still attached it defers to the automatic
	 *  recovery on the last release. Returns true when a recovery actually happened. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Ragdoll")
	bool RecoverFromRagdollIfUnheld();

	UFUNCTION(BlueprintPure, Category = "Rope|Ragdoll")
	bool IsRagdolled() const { return bRagdolled; }

private:
	/** The target mesh: GetMesh() on an ACharacter, otherwise the owner's first
	 *  USkeletalMeshComponent. */
	USkeletalMeshComponent* ResolveMesh() const;

	/** Saves the state needed to restore the mesh afterwards: collision profile, attachment and
	 *  relative transform. */
	void SaveRestoreState(USkeletalMeshComponent* Mesh);

	/** Applies bGenerateOverlapEventsWhileRagdolled to the mesh on entry. Recovery restores the saved
	 *  value. */
	void ApplyRagdollOverlapEvents(USkeletalMeshComponent* Mesh);

	//~ Handlers for the central signals on URopeSimSubsystem.
	/** Any rope in the world established a wrap. If Info.Mesh is this component's mesh, the automatic
	 *  ragdoll is scheduled after the configured delay. */
	void HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info);
	/** Any rope in the world released. If the last rope wrapping this mesh has now let go and the
	 *  ragdoll was automatic, recover and cancel any pending schedule. */
	void HandleAnyRopeReleased(const URopeComponent* Rope, const USceneComponent* WrappedMesh, FName Bone,
		ERopeReleaseReason Reason);

	/** Drops destroyed ropes, whose weak pointers have expired, from the engagement set and returns
	 *  how many remain. */
	int32 PruneWrappingRopes();

	/** Performs the transition when RagdollOnWrappedDelay expires, using the bone recorded in
	 *  PendingWrappedBone. */
	void FireAutoRagdoll();

	/** Enables or disables continuous collision detection on every body, subject to the
	 *  bUseCCDWhileRagdolled gate. Enabled on entry and restored on recovery. */
	void ApplyRagdollCCD(USkeletalMeshComponent* Mesh, bool bEnable);

	/** Spawns the camera actor and switches the view target when a full ragdoll begins, for player
	 *  pawns only. Silently does nothing on failure. */
	void BeginRagdollCameraFollow();
	/** Blends the view target back to the pawn and ends the camera actor's lifetime, which must
	 *  outlast the blend. */
	void EndRagdollCameraFollow();

	bool bRagdolled = false;
	bool bPartial = false;
	// Whether the current ragdoll was entered automatically by a wrap, as opposed to manually. This
	// gates eligibility for automatic recovery.
	bool bRagdollWasAutoTriggered = false;
	// The bone that was wrapped when the automatic transition was scheduled, used as the partial
	// ragdoll root. Consumed by FireAutoRagdoll.
	FName PendingWrappedBone = NAME_None;
	FTimerHandle AutoRagdollTimer;

	// The ropes currently wrapping this actor's mesh, maintained from the central wrap and release
	// signals. The automatic recovery gate checks whether this is empty: recovering on a mesh match
	// alone would stand the target up as soon as one of two attached ropes released. A rope can be
	// destroyed while still wrapped, so these are weak and expired entries are cleaned up by
	// PruneWrappingRopes.
	TSet<TWeakObjectPtr<URopeComponent>> WrappingRopes;

	FName SavedCollisionProfile = NAME_None;
	FTransform SavedMeshRelative = FTransform::Identity;
	TWeakObjectPtr<USceneComponent> SavedAttachParent;
	FName SavedAttachSocket = NAME_None;
	TEnumAsByte<ECollisionEnabled::Type> SavedCapsuleCollision = ECollisionEnabled::QueryAndPhysics;
	// The mesh's overlap event flag before entry, which bGenerateOverlapEventsWhileRagdolled may have
	// overwritten, restored on recovery.
	bool bSavedMeshOverlapEvents = false;
	// The movement mode before entry, restored exactly on recovery. Restoring a hardcoded walking mode
	// instead would make a target that went limp while flying, swimming or in a custom mode walk out
	// of the ragdoll. Only an entry made in MOVE_None is rescued to walking.
	TEnumAsByte<EMovementMode> SavedMovementMode = MOVE_Walking;
	uint8 SavedCustomMovementMode = 0;

	// Ragdoll camera follow state. The controller and camera actor are not owned here, so they are
	// held weakly and it is safe whether recovery or destruction happens first.
	TWeakObjectPtr<APlayerController> FollowController;
	TWeakObjectPtr<ACameraActor> FollowCamera;
	// The world offset from the anchor bone to the camera at the moment of entry, preserved while
	// following so the viewing angle and distance are kept.
	FVector FollowCameraOffset = FVector::ZeroVector;

	// Signal subscription handles, released in EndPlay.
	FDelegateHandle WrappedHandle;
	FDelegateHandle ReleasedHandle;
};
