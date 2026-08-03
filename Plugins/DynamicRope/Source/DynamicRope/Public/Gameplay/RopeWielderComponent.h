// Copyright Epic Games, Inc. All Rights Reserved.
//
// Gameplay component that lets a character hold and throw a rope. It attaches a URopeComponent to a
// hand socket and gathers throw input and aiming in one place, so a character needs only this
// component plus a rope component to be fully set up.
//
// What it replaces doing by hand: reparenting the rope under hand_r, and wiring a key to Throw() in
// Blueprint.
//  - Automatic socket attachment: BeginPlay finds the owner's skeletal mesh and attaches the rope to
//    HandSocketName.
//  - Throw() / Release() / ToggleThrow() are BlueprintCallable. The aim direction comes from the
//    rope's ThrowParams.FrameMode.
//  - Optional Enhanced Input auto-binding: assign ThrowAction / ReleaseAction (and a mapping
//    context) and they are bound during BeginPlay. Leave them empty and call Throw() directly.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "Components/ActorComponent.h"
#include "Core/RopeTypes.h"
#include "Engine/EngineTypes.h"
#include "RopeWielderComponent.generated.h"

class URopeComponent;
class URopePreset;
class URopePreviewComponent;
class AActor;
struct FRopeAimRayThrowRequest;
class USkeletalMeshComponent;
class UInputAction;
class UInputMappingContext;
class UEnhancedInputLocalPlayerSubsystem;
class UInputComponent;
class UAnimMontage;
class UMaterialInstanceDynamic;
class UMovementComponent;
struct FRopeWielderMovementConstraint;

// Whether aiming uses an aim ray, and whether the throw is locked to a prepared preview, are both
// derived from the rope's URopeComponent::ResolveMode - see UsesAimRay() and UsesLockedPreview().
//   FullSimulation  = free aim, physical outcome.
//   AssistedJudged  = aim ray, physical outcome plus a capture decision.
//   GuaranteedWrap  = aim ray, throw locked to the prepared preview path.

UENUM(BlueprintType)
enum class ERopeAimRayOriginMode : uint8
{
	/** Centre of the held skeletal mesh bounds. Starts near the torso or pelvis without hardcoding a
	 *  bone name. */
	AttachMeshBoundsCenter UMETA(DisplayName = "Attach Mesh Bounds Center"),

	/** A named socket or bone location. Use this when the ray must start from an exact reference such
	 *  as the pelvis. */
	AttachSocketOrBone UMETA(DisplayName = "Attach Socket Or Bone"),

	/** The owning actor's location, which on a Character is usually close to the capsule centre. */
	OwnerActorLocation UMETA(DisplayName = "Owner Actor Location"),

	/** Pawn eye height (GetPawnViewLocation). If the rope's ThrowParams.FrameMode is OwnerCamera the
	 *  camera location is used instead, so the origin and the direction share one reference. Use this
	 *  only when the ray must start from the head or eye height. */
	ViewLocation UMETA(DisplayName = "View Location")
};

/** Why a throw input did not execute. Delivered through OnThrowRejected for UI feedback and gameplay
 *  reactions. */
// The numeric values are pinned: 1 and 2 are permanently retired and must never be reused. Removing a
// reason and letting later values shift down would silently remap already-saved Blueprint switch pins
// onto a different reason, with no compile or load error to catch it.
UENUM(BlueprintType)
enum class ERopeThrowRejectReason : uint8
{
	/** The CanThrow() gate refused, for subclass game rules such as stamina or character state. */
	Gated = 0,
	/** The rope component refused the prepared preview throw, including the CanWrapTarget gate. */
	RopeRejected = 3,
	/** A GuaranteedWrap rope was thrown outside the Loaded phase; EnterLoaded() must come first. */
	NotLoaded = 4
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FRopeWielderOnThrown);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeWielderOnThrowRejected, ERopeThrowRejectReason, Reason);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeWielderOnAimTargetChanged, USceneComponent*, Mesh, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FRopeWielderOnAimTargetLost);
/** Fired when the pull arming toggle changes between disarmed and armed. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeWielderOnPullArmedChanged, bool, bArmed);
/** Fired when the pull engage latch changes between waiting and engaged. Tension is the tension
 *  observed at the moment of engagement, and 0 when disengaging. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeWielderOnPullEngagedChanged, bool, bEngaged, float, Tension);

class URopeAimWidget;
class URopePullGaugeWidget;

/**
 * Per-frame sample for the aiming HUD: which wrappable target the aim ray is currently pointing at.
 * In aim ray modes the wielder registers a request, and the result resolved right after the
 * subsystem's normal collider gather is cached for the next tick. That costs at most one frame of
 * latency and avoids gathering extra providers purely for the HUD.
 * Consumers (URopeAimWidget, Blueprint) are read-only; Mesh is for display and identification only.
 */
USTRUCT(BlueprintType)
struct FRopeAimHudSample
{
	GENERATED_BODY()

	/** Whether the aim ray is on a wrappable bone this frame. When false the remaining fields are
	 *  meaningless unless bBlocked is set. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	bool bHasTarget = false;

	/**
	 * Whether the aim ray hit something that cannot be wrapped: static world geometry, a target with
	 * no bone, or one refused by CanWrapTarget. Mutually exclusive with bHasTarget, which wins when a
	 * wrappable target is present. When true the HUD should read as blocked, and
	 * TargetWorldPos / HitWorldPos / TargetRadius / Distance describe the blocking hit; Bone and Mesh
	 * may be unset.
	 */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	bool bBlocked = false;

	/**
	 * Whether the blocking hit is a wrap target that was refused, meaning a skeletal bone or a wrap
	 * target component, as opposed to plain level geometry such as a floor or a wall. Meaningful only
	 * while bBlocked.
	 * The aiming HUD shows its blocked colour only for the former: a red reticle on every floor and
	 * wall reads as "aiming is broken" rather than "this cannot be wrapped", so bare geometry keeps the
	 * ordinary untargeted crosshair.
	 */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	bool bBlockedByTarget = false;

	/** The bone being aimed at, including virtual bones. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FName Bone = NAME_None;

	/** The component owning the bone: a skeletal mesh or a static wrap target. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	TObjectPtr<USceneComponent> Mesh = nullptr;

	/** Centre of the highlight ring, at the bone binding location (ResolveBindingWorld). */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector TargetWorldPos = FVector::ZeroVector;

	/** The world point the ray actually hit, for effects that need a precise location. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector HitWorldPos = FVector::ZeroVector;

	/** Approximate world radius of the target collider (bounds half-diagonal), used to size the ring. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	float TargetRadius = 0.0f;

	/** Distance from the ray origin to the hit. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	float Distance = 0.0f;

	/** The aim ray origin actually used this frame, matching the location chosen by
	 *  AimRayOriginMode. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector RayOrigin = FVector::ZeroVector;

	/** The aim ray direction actually used, normalized. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector RayDirection = FVector::ForwardVector;

	/** The aim ray length actually used. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	float RayLength = 0.0f;

	/** The query radius the sweep actually used this frame. When AimRayQueryRadius is 0 (the default)
	 *  this holds the rope or contact fallback radius, so it can differ from the configured value.
	 *  This is the real thickness aiming tests against. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	float QueryRadius = 0.0f;

	/** Where to project the aiming reticle: the actual hit when there is one, otherwise the far end of
	 *  the aim ray. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope|Aim HUD")
	FVector AimWorldPos = FVector::ZeroVector;
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeWielderComponent : public UActorComponent
{
	GENERATED_BODY()

#if WITH_DEV_AUTOMATION_TESTS
	friend struct FRopeWielderComponentTestSeam;
#endif

public:
	URopeWielderComponent();

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	//~ Setup --------------------------------------------------------------
	/** The rope to hold. Leave empty to find the owner's URopeComponent during BeginPlay. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	TObjectPtr<URopeComponent> Rope = nullptr;

	/** The skeletal mesh to attach the rope to. Leave empty to use the owner's first
	 *  USkeletalMeshComponent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	TObjectPtr<USkeletalMeshComponent> AttachMesh = nullptr;

	/** Name of the socket or bone the rope is attached to. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder", meta = (DisplayName = "Hand Socket"))
	FName HandSocketName = TEXT("hand_r");

	/** Whether to attach the rope to the socket automatically during BeginPlay. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wielder")
	bool bAttachOnBeginPlay = true;

	//~ Aim ----------------------------------------------------------------
	// The wielder owns only the details of the aim origin. The aim ray direction follows the rope's
	// ThrowParams.FrameMode (Owner, OwnerCamera, Socket and so on), and whether an aim ray is used at
	// all is decided by the rope's ResolveMode - see UsesAimRay().

	/** Chooses the aim ray origin between the mesh bounds centre, the attach component, and a named
	 *  socket or bone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (DisplayName = "Ray Origin"))
	ERopeAimRayOriginMode AimRayOriginMode = ERopeAimRayOriginMode::AttachMeshBoundsCenter;

	/** Socket or bone used as the ray origin in AttachSocketOrBone mode. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (EditCondition = "AimRayOriginMode == ERopeAimRayOriginMode::AttachSocketOrBone", DisplayName = "Origin Socket"))
	FName AimRayOriginSocketName = NAME_None;

	/**
	 * Whether to add the demo aiming HUD (crosshair plus a highlight ring on the wrappable bone) to
	 * the local player viewport automatically. The widget class comes from
	 * Project Settings > Dynamic Rope > AimHudWidgetClass, which defaults to the C++ URopeAimWidget and
	 * can be restyled with a Blueprint subclass. It only means anything in aim ray modes, that is when
	 * the rope's ResolveMode is not FullSimulation.
	 * Declared here so the HUD sub-group sorts before Tuning; the details panel orders sub-groups by
	 * declaration order.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim", meta = (DisplayName = "Show Aim HUD"))
	bool bShowAimHudWidget = true;

	/** Sample spacing of the SDF ray march. Smaller values improve accuracy against thin limbs. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Aim|Tuning", meta = (ClampMin = "0.5", Units = "cm", DisplayName = "Ray Sweep Step"))
	float AimRaySweepStep = 2.0f;

	/** Radius tested around the ray centreline. 0 uses the larger of the rope radius and the contact
	 *  radius.
	 *
	 *  Blueprint only, deliberately not exposed in the details panel: the automatic value tracks the
	 *  rope radius, so it stays consistent as soon as the rope thickness is chosen. Setting an
	 *  explicit value breaks that link, the same convention as FRopeWrapConfig::ContactQueryRadius. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Aim|Tuning", meta = (ClampMin = "0.0", Units = "cm"))
	float AimRayQueryRadius = 0.0f;

	/** Fraction along the rope at which the current swing direction starts blending towards the hit
	 *  direction.
	 *
	 *  Blueprint only: internal guide curve maths paired with LockAlpha below, with no basis for a
	 *  user to pick a value. To change how aiming looks, use GuidedLength or SweepAngleDegrees on the
	 *  whip guide instead, where the intent is visible. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Aim|Tuning", meta = (ClampMin = "0.0", ClampMax = "0.9"))
	float AimRayGuideSteerStartAlpha = 0.25f;

	/** Fraction along the rope at which the spatial blend towards the hit direction reaches its
	 *  maximum. It is not fully locked until the flight-time blend completes.
	 *
	 *  Blueprint only: paired with SteerStartAlpha above. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Aim|Tuning", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float AimRayGuideLockAlpha = 0.50f;

	// Aim ray visualization lives in the Gameplay Debugger's Rope category ([J] aim), which reads
	// GetAimHudSample() below. That category is the single debug entry point.

	/**
	 * Whether to add the pull arming and engagement gauge to the local player viewport automatically.
	 * The widget class comes from Project Settings > Dynamic Rope > PullGaugeWidgetClass, which
	 * defaults to the C++ URopePullGaugeWidget. The gauge only draws while pull is armed, so it does
	 * not obscure the screen the rest of the time.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull|Feedback", meta = (DisplayName = "Show Pull Gauge"))
	bool bShowPullGaugeWidget = true;

	/** Whether aiming uses an aim ray to lock onto a target, derived from the rope's ResolveMode:
	 *  true for AssistedJudged and GuaranteedWrap, false for FullSimulation. This is a per-mode
	 *  answer and ignores the phase; use IsAimActive() to ask whether aiming is live right now. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim")
	bool UsesAimRay() const;

	/** Whether aiming means anything right now: UsesAimRay() and the rope is in a throwable phase
	 *  (CanThrowNow). The per-frame counterpart of UsesAimRay(); GuaranteedWrap is only true while
	 *  Loaded, and the other modes match UsesAimRay(). The aiming HUD, widgets and debugger
	 *  visualization all gate on this one call. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim")
	bool IsAimActive() const;

	/** Whether the throw is locked to the prepared preview, derived from the rope's ResolveMode; true
	 *  for GuaranteedWrap. That mode commits to the aimed path when aiming resolves, and otherwise
	 *  falls back to an arc towards the end of the ray rather than refusing the throw. */
	UFUNCTION(BlueprintPure, Category = "Rope|Throw")
	bool UsesLockedPreview() const;

	//~ Throw --------------------------------------------------------------
	// Throw parameters have a single source: the rope's URopeComponent::ThrowParams
	// (FRopeThrowParams). The wielder only contributes context about where the throw comes from, such
	// as the aim direction and the hand socket origin.

	//~ Preview (GuaranteedWrap only) ---------------------------------------
	// Only GuaranteedWrap uses a preview: it builds the prepared path that turns the target aimed at
	// while Loaded into a committed throw. FullSimulation and AssistedJudged have no preview because
	// their wrap is judged or emergent, so there is no path to commit to before the throw; aiming
	// feedback for AssistedJudged comes from the aim ray HUD. Every field below is GuaranteedWrap
	// display policy.
	/**
	 * Leave empty to use the first unclaimed preview component on the owner, creating one if there is
	 * none. That covers the usual actor with zero or one preview component. Assign it explicitly only
	 * when an actor has several, for example two wielders, or several components styled with
	 * different materials.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Preview", meta = (UseComponentPicker, AllowedClasses = "/Script/DynamicRope.RopePreviewComponent", DisplayName = "Preview Component"))
	FComponentReference PreviewComponentReference;

	UPROPERTY(Transient)
	TObjectPtr<URopePreviewComponent> PreviewComponent = nullptr;

	//~ Input (optional; leave empty and call Throw() directly) --------------
	/** Whether to bind the configured actions and mapping context automatically during BeginPlay. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	bool bAutoBindInput = true;

	/** Input mapping context added to the player, if any. An action only triggers while it belongs to
	 *  an active mapping context. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputMappingContext> MappingContext = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input|Tuning", meta = (EditCondition = "MappingContext != nullptr"))
	int32 MappingPriority = 0;

	/** Throw action. Calls Throw() on Started. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ThrowAction = nullptr;

	/** Release action. Calls Release() on Started. Leave empty to let ThrowAction act as a
	 *  throw/release toggle instead, controlled by bThrowActionToggles. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReleaseAction = nullptr;

	/** When ReleaseAction is empty, use ThrowAction as a throw/release toggle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input|Tuning", meta = (DisplayName = "Throw Toggles Hold"))
	bool bThrowActionToggles = true;

	// Force and speed values live on the rope, since they are physical quantities: the pull force is
	// HoldConfig.PullForce and the reel speed is URopeComponent::ReelSpeed. Only input bindings
	// remain in this section.

	/** Active pull action, used as a toggle: press to arm, press again to disarm. When it actually
	 *  engages is decided by the PullEngageTension threshold. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> PullAction = nullptr;

	/** Reel-in action, held. Shortens the rope at the rope's ReelSpeed while held, and stops on
	 *  release. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReelInAction = nullptr;

	/** Reel-out action, held. Lengthens the rope at the rope's ReelSpeed up to its initial length
	 *  while held, and stops on release. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReelOutAction = nullptr;

	/** Load action. Calls Rope->EnterLoaded() on Started, moving a GuaranteedWrap rope into the
	 *  Loaded phase so it can be thrown. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Input")
	TObjectPtr<UInputAction> ReloadAction = nullptr;

	//~ Movement (character response to tether traction) ---------------------
	// When the tether's wielder share (the inverse-mass ratio in the lambda solve) is above zero, the
	// rope pulls the wielder towards the anchor. This section is the character movement policy for
	// that traction: it is a game response rather than plugin core physics, so it lives on the
	// wielder.

	/**
	 * While the rope pulls upwards and the character is in a walking mode, its feet stay planted and
	 * block the lift. With this enabled, a sufficiently upward traction direction with accumulated
	 * overshoot switches the character to Falling so the body leaves the ground; the engine handles
	 * landing again. Combined with reel-in this produces a grapple-style "reel and get pulled up".
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Movement", meta = (DisplayName = "Leave Ground On Upward Pull"))
	bool bAutoGroundExitOnUpwardPull = true;

	/** Upward threshold: the ground exit only triggers when the Z component of the traction direction
	 *  (hand to anchor, unit length) is at least this value.
	 *
	 *  Blueprint only: bAutoGroundExitOnUpwardPull above turns the behaviour on and off, while this is
	 *  an internal threshold of that test with no basis for a user to pick a value. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Movement|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bAutoGroundExitOnUpwardPull"))
	float GroundExitUpDot = 0.35f;

	/** Minimum tether overshoot required to leave the ground (cm), which stops the movement mode from
	 *  flapping on jitter at the boundary.
	 *
	 *  Blueprint only: a hysteresis constant fixed by measurement. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Movement|Tuning", meta = (ClampMin = "0.0", Units = "cm", EditCondition = "bAutoGroundExitOnUpwardPull"))
	float GroundExitMinOvershoot = 10.0f;

	/**
	 * While swinging (wrapped, airborne, and receiving a wielder tether share) raise air control to
	 * SwingAirControl so the swing can be steered. The CharacterMovement default of 0.05 barely
	 * changes the swing direction. The saved original value is restored when the swing ends, on
	 * landing or release.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Movement")
	bool bBoostAirControlWhileSwinging = true;

	/** Air control applied while swinging, from 0 to 1. Values from 0.35 to 1 work well; 1 gives
	 *  ground-level steering in the air. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Movement|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bBoostAirControlWhileSwinging"))
	float SwingAirControl = 1.0f;

	//~ Animation (optional) -------------------------------------------------
	/**
	 * When set, Throw() plays this montage instead of throwing immediately. The rope is thrown by a
	 * UAnimNotify_RopeThrow placed in the montage, which calls ThrowNow() so the throw lands on the
	 * frame the hand releases. Leave empty and Throw() throws immediately.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation")
	TObjectPtr<UAnimMontage> ThrowMontage = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation|Tuning", meta = (ClampMin = "0.1", DisplayName = "Throw Play Rate"))
	float ThrowMontagePlayRate = 1.0f;

	/**
	 * When set, this montage is played once each time pull engages, and the force is carried solely by
	 * the UAnimNotifyState_RopePull window placed inside it. This mirrors the contract between
	 * ThrowMontage and UAnimNotify_RopeThrow: with no notify placed, no pull happens. There is no
	 * repeat or interruption management; each engagement plays it once and lets it finish naturally.
	 * Leave empty and engaging applies the pull force immediately through StartPullNow().
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation")
	TObjectPtr<UAnimMontage> PullMontage = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Animation|Tuning", meta = (ClampMin = "0.1", DisplayName = "Pull Play Rate"))
	float PullMontagePlayRate = 1.0f;

	//~ Pull (active traction policy) ----------------------------------------
	// The magnitude of the force belongs to the rope (HoldConfig.PullForce); this section only decides
	// when pull engages.
	/**
	 * Tension threshold at which pull engages (GetConstraintTension, in kg*cm/s^2). Do not confuse it
	 * with the XPBD SegmentTension or GetMaxTension. Once the pull input has armed the toggle, pull
	 * engages the first moment the tension crosses this value while wrapped; 0 engages on the taut
	 * test (IsPullTaut) alone. This is the gameplay threshold behind "only start dragging once the
	 * rope has really gone tight". It governs engagement regardless of whether a montage is set, which
	 * is why it lives here rather than under Animation; with no montage, engaging applies the force
	 * immediately. See UpdatePullEngage for the engagement and lifetime rules.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull", meta = (ClampMin = "0.0"))
	float PullEngageTension = 0.0f;

	/**
	 * Whether to push the arming-to-engagement progress into the rope material, on by default. The
	 * rope is always in the player's view, which makes it a harder feedback channel to miss than the
	 * HUD; the PullGlow scalar on M_RopeDefault receives the value. The dynamic material instance is
	 * created the first time pull is armed, so the material setup is untouched until then.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull|Feedback", meta = (DisplayName = "Drive Glow Material"))
	bool bDrivePullGlowMaterial = true;

	/** Name of the scalar parameter that receives the progress. Ignored harmlessly when the material
	 *  has no such parameter. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull|Feedback", meta = (EditCondition = "bDrivePullGlowMaterial", DisplayName = "Glow Parameter"))
	FName PullGlowParameterName = TEXT("PullGlow");

	/** Value written to the parameter while engaged. Above 1 makes engagement clearly brighter than
	 *  waiting. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Pull|Feedback", meta = (ClampMin = "0.0", EditCondition = "bDrivePullGlowMaterial", DisplayName = "Glow Engaged Value"))
	float PullGlowEngagedValue = 1.5f;

	//~ API ----------------------------------------------------------------
	/**
	 * Starts a throw. With ThrowMontage set this plays the montage and the throw itself happens when
	 * its UAnimNotify_RopeThrow calls ThrowNow(); otherwise it calls ThrowNow() immediately. This is
	 * the entry point for input and gameplay code.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw();

	/** Throws the rope right now, using the forward vector of the rope's ThrowParams.FrameMode. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowNow();

	/** Builds the origin, frame and velocity context captured at the moment of a throw, once per
	 *  throw on the game thread. A valid AimDir replaces the configured frame forward. Override to
	 *  change how the context is assembled. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	virtual FRopeThrowContext BuildThrowContext(const FVector& AimDir) const;

	/** Throws in an explicit direction. A valid AimDir replaces the frame forward and the aim ray is
	 *  cast the same way; ZeroVector keeps the forward vector of the rope's ThrowParams.FrameMode,
	 *  which is identical to ThrowNow. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ThrowInDirection(const FVector& AimDir);

	/** Plays ThrowMontage on the owner mesh's anim instance, when one is set. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void PlayThrowMontage();

	/** Plays PullMontage on the owner mesh's anim instance, when one is set. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void PlayPullMontage();

	/** Releases the current wrap. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Release();

	/**
	 * Arms active pull, the entry point for input and gameplay code. Neither the force nor the montage
	 * starts here: pull engages the first moment the tension crosses PullEngageTension while wrapped,
	 * in UpdatePullEngage, which either plays the montage once or applies the force immediately.
	 * Arming before the wrap lands means pull engages by itself as soon as the rope goes tight.
	 * StopPull disarms.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartPull();

	/** Starts active pull right now, applying traction with the rope's HoldConfig.PullForce. Force is
	 *  only applied while wrapped and taut; the taut test and its gate live on the rope's HoldConfig
	 *  (bActivePullRequiresTaut and ActivePullTautTension) and are read through IsPullTaut().
	 *  bIgnoreTautGate makes this one pull ignore tautness, a per-call bypass for the scripted section
	 *  of a pull window. This is the execution point montage notifies call, matching ThrowNow. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartPullNow(bool bIgnoreTautGate = false);

	/** Stops the active pull force only, leaving the montage alone. Called by a pull window's
	 *  NotifyEnd: the window closing must not cut short the rest of the montage, such as a recovery
	 *  motion. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StopPullNow();

	/** Disarms active pull, stops the force and resets the engage latch. The montage is not
	 *  interrupted, since it is played once and left to finish. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StopPull();

	/** Whether pull is armed. Use IsPullEngaged to ask whether force is actually being applied. */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull")
	bool IsPullArmed() const { return bPullArmed; }

	/** Whether pull has engaged and force is being applied. Returns to false when the wrap releases,
	 *  while arming is kept. */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull")
	bool IsPullEngaged() const { return bPullEngaged; }

	/**
	 * Progress towards the engage threshold, from 0 to 1, which is the value that answers "I pressed
	 * it, so why is nothing pulling" on a gauge. It is 1 after engaging, and 0 when not wrapped, since
	 * the threshold does not apply before the wrap lands. When PullEngageTension is 0, engagement is
	 * decided by the taut test alone, so there is no continuous value and this returns 0 or 1 based on
	 * IsPullTaut.
	 */
	UFUNCTION(BlueprintPure, Category = "Rope|Pull")
	float GetPullEngageProgress() const;

	/** Cuts the rope, forcing a release with ERopeReleaseReason::Cut. A pass-through for gameplay cut
	 *  events. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Cut();

	/** Starts reeling in, shortening the rope at its ReelSpeed. For held input and gameplay code. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartReelIn();

	/** Starts reeling out, lengthening the rope at its ReelSpeed up to its initial length. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StartReelOut();

	/** Stops reeling in or out. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void StopReel();

	/** Releases while wrapped or contacting, and throws otherwise. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ToggleThrow();

	/** The rope being held, or null. */
	UFUNCTION(BlueprintPure, Category = "Rope")
	URopeComponent* GetRope() const { return Rope; }

	/** Binds input manually. Call this from the pawn's SetupPlayerInputComponent if automatic binding
	 *  fails on timing, such as when the input component is not ready yet. Ignored when input is
	 *  already bound. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Input")
	void BindInput();

	/**
	 * Whether to display the throw preview, a designer setting. It controls display only: turning it
	 * off leaves the GuaranteedWrap throw calculation (the prepared preview) running, so throw
	 * behaviour is unchanged. While it is on and the mode is GuaranteedWrap, a preview component is
	 * created automatically during BeginPlay.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Preview")
	bool bShowThrowPreview = true;

	UFUNCTION(BlueprintCallable, Category = "Rope|Preview")
	void SetThrowPreviewEnabled(bool bEnabled);

	/** Whether the throw preview is currently displayed. */
	UFUNCTION(BlueprintPure, Category = "Rope|Preview")
	bool IsThrowPreviewEnabled() const { return bShowThrowPreview; }

	/**
	 * Resynchronizes the state derived from the rope's ResolveMode: automatic preview creation and
	 * display, component tick activation, and the aiming HUD. Preview creation and tick activation are
	 * only computed during BeginPlay, so a mode change at runtime recomputes them through this call.
	 * URopeComponent::ApplyPreset triggers it automatically through the OnPresetApplied subscription;
	 * call it by hand when game code changes ResolveMode directly. Game thread, cold path.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void RefreshModeDerivedState();

	//~ Events ---------------------------------------------------------------
	/** Fired right after a throw actually executes, on both the immediate and montage notify paths. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeWielderOnThrown OnThrown;

	/** Fired with a reason when a throw input did not execute, for UI feedback. Throwing a
	 *  GuaranteedWrap rope outside the Loaded phase arrives here too. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeWielderOnThrowRejected OnThrowRejected;

	/** Fired when the aim ray lands on a new (Mesh, Bone) target, whether entering or switching. For
	 *  HUD effects and sound triggers. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Aim HUD")
	FRopeWielderOnAimTargetChanged OnAimTargetChanged;

	/** Fired when the aim ray loses its target. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Aim HUD")
	FRopeWielderOnAimTargetLost OnAimTargetLost;

	/**
	 * Fired when the pull arming toggle changes through StartPull and StopPull, for arming sounds and
	 * UI state. Engagement, when force is actually applied, is the separate OnPullEngagedChanged
	 * below; the two are different events.
	 */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Pull")
	FRopeWielderOnPullArmedChanged OnPullArmedChanged;

	/**
	 * Fired when the pull engage latch changes. True is the moment the tension threshold is crossed
	 * and force is applied, once per wrap; false is the moment the wrap released and pull rearmed, or
	 * pull was disarmed. UI must handle the false case as well: leaving an engagement indicator lit is
	 * the common bug this event exists to prevent.
	 */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Pull")
	FRopeWielderOnPullEngagedChanged OnPullEngagedChanged;

	/** The aiming HUD sample resolved from the most recent normal collider gather, consumed and
	 *  refreshed every tick in aim ray modes. */
	UFUNCTION(BlueprintPure, Category = "Rope|Aim HUD")
	const FRopeAimHudSample& GetAimHudSample() const { return AimHudSample; }

protected:
	//~ Extension hooks for subclasses ---------------------------------------
	// Same principle as URopeComponent: every hook is called on the game thread, once per frame at
	// most, on a cold path. Documenting when and how often a hook is called is part of the contract
	// when adding one. The public BuildThrowContext is an extension hook as well.

	/**
	 * Throw input gate, called once when Throw() is entered. Returning false discards the input and
	 * reports the Gated reason. Override to restrict throwing by game rules such as stamina or
	 * character state. Defaults to true.
	 * ThrowNow() on the montage path, called from the anim notify, is an already-gated committed
	 * throw and is not re-tested.
	 */
	virtual bool CanThrow() const { return true; }

	//~ Native event hooks, called immediately before each delegate broadcast, following the engine's
	//~ Notify convention.
	virtual void NotifyThrown() {}
	virtual void NotifyThrowRejected(ERopeThrowRejectReason Reason) {}

private:
	/** The aiming HUD sample from the most recent normal collider gather, refreshed in Tick by
	 *  UpdateAimHudSample consuming the result. */
	FRopeAimHudSample AimHudSample;

	/** The throw context resolved by the most recent normal gather. The preview reuses it within the
	 *  same wielder tick the HUD consumed it in. */
	FRopeThrowContext AimRayFrameThrowContext;
	uint64 AimRayFrameContextStamp = 0;
	bool bHasAimRayFrameThrowContext = false;

	/** The automatically created aiming HUD widget, for the local player only. Created and destroyed
	 *  as bShowAimHudWidget and the resolve mode change. */
	UPROPERTY(Transient)
	TObjectPtr<URopeAimWidget> AimHudWidget = nullptr;

	/** The automatically created pull gauge widget, for the local player only. Created and destroyed
	 *  as bShowPullGaugeWidget changes. */
	UPROPERTY(Transient)
	TObjectPtr<URopePullGaugeWidget> PullGaugeWidget = nullptr;

	/**
	 * Registers the aim request, refreshes the HUD sample from the most recent normal gather, then
	 * fires OnAimTargetChanged and OnAimTargetLost when the (Mesh, Bone) target changes. Called from
	 * Tick in aim ray modes only, with at most one frame of latency.
	 */
	void UpdateAimHudSample();

	/** Creates and destroys the aiming HUD widget, lazily from Tick once the local player controller
	 *  is ready. */
	void UpdateAimHudWidget();

	/** Resolves Rope and AttachMesh, searching the owner for anything left unset. */
	void ResolveRefs();

	/** Attaches Rope to HandSocketName on AttachMesh. */
	void AttachRopeToSocket();

	/** Adds MappingContext to the local player's Enhanced Input subsystem. */
	void AddMappingContext();

	void ResolvePreviewComponent(bool bAllowAutoCreate);

	/** Whether the component needs to tick, for any of: the hard leash, pull engagement, restoring a
	 *  boosted air control, the ground exit, swinging, or an aim ray mode. The single expression
	 *  shared by BeginPlay, SetThrowPreviewEnabled and RefreshModeDerivedState. */
	bool ComputeDesiredTickEnabled() const;

	/** Enables or disables the pawn hard-leash tick immediately when the rope phase changes. */
	UFUNCTION()
	void HandleRopePhaseChanged(ERopePhase OldPhase, ERopePhase NewPhase);

	/** Finds the owner's actual movement component and registers or unregisters the PrePhysics tick
	 *  prerequisite. */
	void RegisterMovementConstraintHooks();
	void UnregisterMovementConstraintHooks();
	void RefreshTargetMovementConstraintHooks(
		const FRopeWielderMovementConstraint& Constraint);
	void ClearTargetMovementConstraintHooks();

	/**
	 * Enforces the coupled material-length limit between the hand and the target and removes the
	 * relative velocity beyond it. Against a fixed target the hand absorbs all of the correction;
	 * against a simulating body the hand and the Chaos target split the positional correction by their
	 * generalized-mass shares.
	 */
	void EnforceWielderLengthConstraint(float DeltaTime);

	/** Handler for the rope's OnPresetApplied signal, which resynchronizes the mode-derived state. */
	UFUNCTION()
	void HandleRopePresetApplied(const URopePreset* Preset);

	/** Whether the wielder actually receives a tether share: wrapped, with a valid target share below
	 *  1, and not a self-wrap. */
	bool IsWielderTetherActive() const;

	/** Switches a walking character to Falling while the wielder's tether share pulls upwards. Called
	 *  every tick on the game thread; see the Movement section above. */
	void UpdateGroundExit();

	/** Boosts and restores air control according to the swing test. Called every tick on the game
	 *  thread. */
	void UpdateSwingAirControl();

	/**
	 * Decides when an armed pull engages, every tick on the game thread. It engages the first moment
	 * the rope is wrapped and the tension condition holds: with a threshold of 0 that is IsPullTaut,
	 * and above 0 it is GetConstraintTension reaching PullEngageTension. The latch fires once per
	 * wrap. With a montage set up, PullMontage is played once and the force is carried by its window
	 * notify, with no repeat or interruption management; without one, the force is applied
	 * immediately. Leaving the wrapped phase stops the force and rearms, keeping the armed state so
	 * the next wrap can engage again.
	 */
	void UpdatePullEngage();

	void UpdateThrowPreview();

	/** Measures the hand socket's animation-relative velocity from its component-local position delta,
	 *  every tick. It is carried on the throw context as HandAnimationVelocity, which avoids the
	 *  physics-body dependency and sign problems of GetPhysicsLinearVelocity. */
	void UpdateHandAnimVelocity(float DeltaTime);

	/** Converts a component-local socket position delta into a world relative velocity, excluding
	 *  character movement, and clamps its magnitude. Dividing by the real delta time keeps it
	 *  framerate independent, so the same hand swing yields the same velocity. Pure, and the unit test
	 *  seam for the measurement logic. */
	static FVector ComputeHandSwingVelocityWorld(const FVector& PrevSocketCS, const FVector& CurSocketCS,
		float DeltaTime, const FTransform& ComponentXform, float MaxSpeed);

	/** Resets the hand velocity sample so a stale delta cannot fling the rope after reactivation, a
	 *  rig change, a teleport or a hitch. */
	void ResetHandAnimVelocitySample();

	/** Applies ComputeDesiredTickEnabled() to the tick, reseeding the hand velocity sample on an
	 *  off-to-on transition. */
	void RefreshTickEnabled();

	// Hand socket animation velocity measurement state: the previous component-local position, the
	// measured world velocity, whether a first sample exists, and rig change detection, which breaks
	// and reseeds the delta when the mesh or socket changes.
	FVector PreviousHandSocketLocationCS = FVector::ZeroVector;
	FVector MeasuredHandAnimVelocityWorld = FVector::ZeroVector;
	bool bHasHandSocketSample = false;
	TWeakObjectPtr<USkeletalMeshComponent> PreviousHandSampleMesh;
	FName PreviousHandSampleSocket = NAME_None;

	/** Hands the given centreline to the preview component to draw. A no-op when display is off or
	 *  there is no component. */
	void DisplayPreviewCenterline(const FRopeWrapPreviewData& Centerline);

	/** Clears the display only, keeping the prepared throw data. */
	void ClearPreviewDisplay();

	/** Clears the prepared throw state only, leaving the display alone. */
	void ClearPreparedThrow();

	/** Clears both the prepared state and the display. */
	void ClearThrowPreview();

	/** Computes the origin, frame and velocity only, with no collider or SDF side effects. */
	FRopeThrowContext BuildBaseThrowContext(const FVector& AimDir) const;
	// Casts a fresh ray and builds the context to freeze at the moment of the throw.
	FRopeThrowContext BuildThrowContextInternal(const FVector& AimDir) const;
	// Lets the preview reuse the aim context from the most recent gather the HUD consumed in this same
	// wielder tick.
	bool TryGetCachedAimRayThrowContext(const FVector& AimDir, FRopeThrowContext& OutContext) const;
	// Captures the base frame and ray settings at the moment of input as a value-type request.
	FRopeAimRayThrowRequest BuildAimRayThrowRequest(const FVector& AimDir) const;
	// Queues a GuaranteedWrap input ray to be resolved into a prepared throw right after the normal
	// gather.
	bool QueueGuaranteedAimThrow(const FVector& AimDir, bool bExecuteWhenReady);
	// Resolves the selected origin mode into a world location.
	FVector GetAimRayOrigin() const;
	float GetAimReachLength() const;
	// Stores an aim hit's prepared spline in wielder owner-local space so it is decoupled from hand
	// socket animation.
	void StoreAimGuideFrameIfNeeded(FRopePreparedThrowPreview& Prepared) const;
	// Renders a stored owner-local prepared spline against the current owner transform.
	FRopeWrapPreviewData ResolvePreparedPreviewForDisplay(const FRopePreparedThrowPreview& Prepared) const;
	bool ShouldHoldPreparedPreview();
	// Whether a new preview path may be computed in the current rope phase. Returning false avoids
	// entering the expensive build path.
	bool ShouldUpdateThrowPreviewForPhase(ERopePhase Phase) const;
	// Keeps a path GuaranteedWrap has already committed to on screen during GuidedThrow and Wrapped.
	// Returns true when it handled the frame.
	bool UpdateHeldPreparedPreviewForPhase(ERopePhase Phase);

	void OnThrowInput();
	void OnGuaranteedAimPrepared(FRopePreparedThrowPreview& Prepared);
	void OnAimRayThrowResolved();
	void OnAimRayThrowRejected();
	void OnPullInputStarted();
	void OnReloadInput();

	/**
	 * Possession change hook, which rebinds input on late possession, where the pawn spawns without a
	 * controller, and on repossession. Binding only during BeginPlay would leave the mapping context
	 * unattached forever when no controller exists at that moment, and leave the actions unbound
	 * forever when no input component exists, making throw, pull and reel unresponsive. Only
	 * subscribed while bAutoBindInput is set: games that bind manually are already covered, since
	 * SetupPlayerInputComponent is called again on every repossession.
	 */
	UFUNCTION()
	void HandlePawnControllerChanged(APawn* OwnerPawn, AController* OldController, AController* NewController);

	/** The input component is created during PawnClientRestart, so it can still be missing right after
	 *  a controller attaches. Trying again on restart means whichever of the two happens later
	 *  actually completes the binding. */
	UFUNCTION()
	void HandlePawnRestarted(APawn* OwnerPawn);

	/** Reattaches the mapping context and rebinds the actions against the currently possessing pawn.
	 *  The path shared by BeginPlay and the two hooks above. */
	void RefreshInputRegistration();

	/** Removes the mapping context added by AddMappingContext from the cached subsystem. Shared by
	 *  possession changes and EndPlay. */
	void RemoveMappingContext();
	/** Removes only this object's action bindings from the input component they were bound to. */
	void ClearBoundInput();
	/** Tries to play PullMontage and reports whether playback actually started. */
	bool TryPlayPullMontage();

	// The input component the bindings were actually made on. Prevents binding the same component
	// twice, and lets an input component replaced by repossession have its old bindings cleared before
	// rebinding onto the new one.
	TWeakObjectPtr<UInputComponent> BoundInputComponent;
	// The local player Enhanced Input subsystem AddMappingContext added the context to, held weakly.
	// Mapping contexts are registered on the local player rather than the pawn, so removal happens
	// here independently of possession instead of depending on the pawn's current controller. That
	// keeps the context from lingering on the local player when the pawn is unpossessed and destroyed
	// first. Null once the local player is destroyed.
	TWeakObjectPtr<UEnhancedInputLocalPlayerSubsystem> MappedInputSubsystem;
	// The context actually added to the subsystem above. Preserving the exact object until removal
	// means a MappingContext property changed at runtime cannot strand it, and its lifetime does not
	// depend on how the subsystem references it internally.
	UPROPERTY(Transient)
	TObjectPtr<UInputMappingContext> MappedInputContext = nullptr;

	/** The target the hard leash is applied to automatically. The wielder's PrePhysics tick runs after
	 *  this component. */
	TWeakObjectPtr<UMovementComponent> ConstraintMovementComponent;
	TWeakObjectPtr<AActor> ConstraintTargetActor;
	TWeakObjectPtr<USceneComponent> ConstraintTargetComponent;
	TWeakObjectPtr<UMovementComponent> ConstraintTargetMovementComponent;
	bool bLeashCorrectionBlockedLogged = false;
	/** Sets bPullArmed and broadcasts OnPullArmedChanged, only when the value changes. */
	void SetPullArmed(bool bNewArmed);

	/** Sets bPullEngaged and broadcasts OnPullEngagedChanged, only when the value changes. */
	void SetPullEngaged(bool bNewEngaged, float Tension);

	/** Pushes the progress into the rope's dynamic material instance scalar parameter, while
	 *  bDrivePullGlowMaterial is set and only after pull has first been armed. */
	void UpdatePullGlowMaterial();

	/** Manages the pull gauge widget lifetime: created once the local player is ready, destroyed when
	 *  the toggle turns off and on EndPlay. */
	void UpdatePullGaugeWidget();

	// Pull arming state, toggled between StartPull and StopPull. Engagement is decided by
	// UpdatePullEngage.
	bool bPullArmed = false;
	// Pull engage latch, set the first time the tension threshold is crossed while armed, once the
	// montage has been played or the force applied. Rearmed when the wrap releases.
	bool bPullEngaged = false;
	// The dynamic material instance created to carry the progress on the rope material. Applying a
	// preset can replace the rope material, which leaves this out of step with GetMaterial(0), so it
	// is recreated when that happens. Checked once per tick with a single pointer comparison.
	TWeakObjectPtr<UMaterialInstanceDynamic> PullGlowMID;
	// Saved state for restoring the air control boost, stored on entering a swing and restored when it
	// ends or on EndPlay.
	bool bAirControlBoosted = false;
	float SavedAirControl = 0.0f;

	// True while a GuaranteedWrap input ray waits to be resolved into a prepared throw by the normal
	// gather, or waits for the montage notify to execute it.
	bool bGuaranteedAimThrowQueued = false;

	// The committed path kept on screen while GuaranteedWrap executes, including during GuidedThrow.
	FRopeWrapPreviewData HeldPreparedPreview;

#if WITH_DEV_AUTOMATION_TESTS
	// Test-only instrumentation that verifies whether the prepared build runs while display is off,
	// without any external side effects.
	int32 TestPreparedPreviewBuildCount = 0;
#endif
};
