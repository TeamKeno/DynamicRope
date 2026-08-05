// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The dragon demo component. It is not a shipping feature and lives in the demo folder: a hardcoded sequence for a
// single demo video shot of under ten seconds.
//
// The fixed flow:
//   Idle, sitting still
//     └ triggered when a rope wraps this actor, through the central OnAnyRopeWrapped, or manually through
//       Rope.Demo.Dragon or StartSequence()
//   Howl, once
//   Thrash, along a figure-of-eight path parallel to the ground
//   Ascend, surging upwards
//   Descend, coming back down to a stop, then returning to the idle animation and finishing
//
// Design notes:
//  - The previous version's steered turning flight, a physics-friendly path actually dragged along by the rope
//    tether, was dropped entirely. This component is presentation-only and overwrites the actor transform from the
//    path every tick, so displacement pushed in by the rope tether does not persist. The goal is deterministic
//    timing for a video, and demonstrating physical interaction is a separate matter.
//  - The animation uses no animation Blueprint: three single sequences, idle, howl and fly, are swapped directly
//    through the skeletal mesh component's single node mode with PlayAnimation. Do not attach an animation
//    Blueprint to the dragon Blueprint.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RopeDragonFlightDemoComponent.generated.h"

class UAnimSequenceBase;
class URopeComponent;
class USkeletalMeshComponent;
struct FRopeWrappedEventInfo;

/** The demo sequence's stages. The order is fixed and never goes backwards; after finishing, only the idle animation is held. */
UENUM(BlueprintType)
enum class ERopeDragonDemoState : uint8
{
	Idle		UMETA(DisplayName = "Idle"),
	Howl		UMETA(DisplayName = "Howl"),
	Thrash		UMETA(DisplayName = "Thrash (figure of eight)"),
	Ascend		UMETA(DisplayName = "Ascend"),
	Descend		UMETA(DisplayName = "Descend"),
	Finished	UMETA(DisplayName = "Finished")
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPEPROJECT_API URopeDragonFlightDemoComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeDragonFlightDemoComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	//==================================================================================
	// Animation: single sequences played directly, with no animation Blueprint.
	//==================================================================================

	/** The idle and finished pose, looping. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Anim")
	TObjectPtr<UAnimSequenceBase> IdleAnim;

	/** The howl, played once on triggering. This sequence's length becomes the howl stage's length when the duration is zero. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Anim")
	TObjectPtr<UAnimSequenceBase> HowlAnim;

	/** The flight loop running throughout the thrash, ascent and descent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Anim")
	TObjectPtr<UAnimSequenceBase> FlyAnim;

	//==================================================================================
	// Timing. The defaults are chosen to total under ten seconds.
	//==================================================================================

	/** The howl length, in seconds. Zero uses the howl sequence's own length, or 1.5 seconds if there is none. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Timing", meta = (ClampMin = "0.0", Units = "s"))
	float HowlDuration = 0.0f;

	/** The figure-of-eight thrash length, in seconds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Timing", meta = (ClampMin = "0.1", Units = "s"))
	float ThrashDuration = 3.5f;

	/** The ascent length, in seconds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Timing", meta = (ClampMin = "0.1", Units = "s"))
	float AscendDuration = 1.8f;

	/** The descent length, in seconds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Timing", meta = (ClampMin = "0.1", Units = "s"))
	float DescendDuration = 1.6f;

	//==================================================================================
	// The figure-of-eight path, whose origin and basis is the actor transform at the moment of triggering, parallel to the ground.
	//==================================================================================

	/** The half width of the figure of eight's long axis, forward at the moment of triggering, in centimetres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "0.0", Units = "cm"))
	float ThrashLength = 700.0f;

	/** The half width of the figure of eight's short axis, to the right at the moment of triggering, in centimetres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "0.0", Units = "cm"))
	float ThrashWidth = 450.0f;

	/** How many times it goes round the figure of eight during the thrash. It has to be an integer for the start and end to meet smoothly at the origin. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "1"))
	int32 ThrashLoops = 2;

	/** The amplitude, in centimetres, of the vertical bob while going round the figure of eight. A perfectly flat path looks lifeless, so it is given a little. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "0.0", Units = "cm"))
	float ThrashBobHeight = 120.0f;

	/** The maximum bank, meaning roll, in degrees, matched to the direction the figure of eight crosses. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Thrash", meta = (ClampMin = "0.0", ClampMax = "80.0"))
	float ThrashBankDeg = 35.0f;

	//==================================================================================
	// The ascent and descent.
	//==================================================================================

	/** The ascent height, in centimetres, relative to the trigger point. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Climb", meta = (ClampMin = "0.0", Units = "cm"))
	float AscendHeight = 1400.0f;

	/** How far forward it is carried during the ascent, in centimetres. Rising purely vertically looks unnatural, so a forward component is mixed in. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Climb", meta = (ClampMin = "0.0", Units = "cm"))
	float AscendForward = 500.0f;

	/** The maximum pitch, in degrees, as the body rears up: upwards on the ascent and downwards on the descent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Climb", meta = (ClampMin = "0.0", ClampMax = "80.0"))
	float ClimbPitchDeg = 30.0f;

	/** Whether it returns to the trigger point at the end of the descent. When false it lands at the forward position the ascent carried it to. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Climb")
	bool bReturnToStartOnLand = false;

	//==================================================================================
	// Triggering.
	//==================================================================================

	/** Triggers automatically when a rope wraps this actor's mesh. When false it is triggered by StartSequence() or the console alone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Trigger")
	bool bStartOnWrapped = true;

	/** The delay, in seconds, from being wrapped to the howl, which shows the moment the rope catches for a beat before reacting. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|DragonDemo|Trigger", meta = (ClampMin = "0.0", Units = "s"))
	float TriggerDelay = 0.2f;

	/** Starts the sequence. Ignored if it is already running or has finished. */
	UFUNCTION(BlueprintCallable, Category = "Rope|DragonDemo")
	void StartSequence();

	/** Returns it to idle, restoring the transform at the trigger point, for repeated takes. */
	UFUNCTION(BlueprintCallable, Category = "Rope|DragonDemo")
	void ResetSequence();

	/** The current stage. */
	UFUNCTION(BlueprintPure, Category = "Rope|DragonDemo")
	ERopeDragonDemoState GetDemoState() const { return State; }

private:
	void HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info);

	USkeletalMeshComponent* ResolveMesh() const;
	void PlayAnim(UAnimSequenceBase* Anim, bool bLoop);
	void EnterState(ERopeDragonDemoState NewState);

	/** Applies a local offset and rotation, relative to the trigger point, in world space. The shared exit used by every stage. */
	void ApplyPose(const FVector& LocalOffset, float YawDeg, float PitchDeg, float RollDeg);

	float ResolveHowlDuration() const;

	ERopeDragonDemoState State = ERopeDragonDemoState::Idle;
	float TimeInState = 0.0f;
	float PendingTriggerTime = -1.0f;

	/** The actor position and basis at the moment of triggering. Only the yaw is used, since the figure of eight has to stay parallel to the ground. */
	FVector AnchorLocation = FVector::ZeroVector;
	float AnchorYawDeg = 0.0f;

	/** The local offset at the end of the ascent, which is where the descent starts from. */
	FVector AscendEndOffset = FVector::ZeroVector;

	FDelegateHandle WrappedHandle;
};
