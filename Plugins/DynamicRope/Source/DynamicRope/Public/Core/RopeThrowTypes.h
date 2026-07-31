// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeWrappingTypes.h"
#include "RopeThrowTypes.generated.h"

class USceneComponent;

/** Which coordinate space the reference axes of a throw are taken from. */
UENUM(BlueprintType)
enum class ERopeThrowFrameMode : uint8
{
	World = 0 UMETA(DisplayName = "World"),
	Owner = 1 UMETA(DisplayName = "Owner"),
	OwnerCamera = 3 UMETA(DisplayName = "Owner Camera"),
	Socket = 2 UMETA(DisplayName = "Socket"),
	Custom = 4 UMETA(DisplayName = "Custom")
};

/** Combined with the aim direction, chooses the plane and direction the swing arc lies in. */
UENUM(BlueprintType)
enum class ERopeSwingPlane : uint8
{
	AimAndFrameUp = 0 UMETA(DisplayName = "Aim + Frame Up"),
	AimAndFrameDown = 1 UMETA(DisplayName = "Aim + Frame Down"),
	AimAndFrameRight = 2 UMETA(DisplayName = "Aim + Frame Right"),
	AimAndFrameLeft = 3 UMETA(DisplayName = "Aim + Frame Left"),
	CustomNormal = 4 UMETA(DisplayName = "Custom Plane Normal")
};

// Defined below; MakeDefault takes it as the settings snapshot.
struct FRopeThrowParams;

/** The runtime values the wielder or component computes and passes at the moment of a throw, kept
 *  separate from the settings in FRopeThrowParams. */
USTRUCT(BlueprintType)
struct DYNAMICROPE_API FRopeThrowContext
{
	GENERATED_BODY()

	/**
	 * Assembles a default context from the component transform and the throw settings, once per throw
	 * on the game thread. Used by the default implementation of the URopeComponent::Throw()
	 * convenience entry point, and irrelevant to callers such as the wielder that build the context
	 * themselves. The frame basis convention per FrameMode is:
	 *   World uses the world axes; Owner and Socket use the component basis; OwnerCamera uses the
	 *   owner's first camera, falling back to the component basis when there is none; and Custom uses
	 *   the custom axes from Params as given, leaving normalization and the orthogonality fallback to
	 *   ResolveThrowContext. Implemented in RopeTypes.cpp.
	 */
	static FRopeThrowContext MakeDefault(const USceneComponent& RopeComponent, const FRopeThrowParams& Params);

	// This struct is a runtime snapshot computed at the moment of a throw, so it is not exposed as
	// stored, editable properties: anything edited in the details panel would be overwritten on every
	// throw and mean nothing. Only BlueprintReadWrite is kept, for callers assembling a context with
	// Blueprint make and break nodes.
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector Origin = FVector::ZeroVector;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameForward = FVector::ForwardVector;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameUp = FVector::UpVector;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameRight = FVector::RightVector;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector OwnerVelocity = FVector::ZeroVector;

	/** The hand socket's animation-relative world velocity, excluding character movement, so swinging
	 *  the arm while standing still still carries that swing into the throw. The wielder measures it
	 *  from the component-local socket position delta, which avoids the physics-body dependency and
	 *  sign problems of GetPhysicsLinearVelocity. It is zero on paths without socket tracking, such as
	 *  calling Throw directly from Blueprint. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector HandAnimationVelocity = FVector::ZeroVector;

	/** The throw speed owned by the wielder. At or below 0 the rope component's fallback value is
	 *  used. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float ThrowSpeed = 0.0f;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	ERopeThrowFrameMode FrameMode = ERopeThrowFrameMode::Owner;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	ERopeSwingPlane SwingPlane = ERopeSwingPlane::AimAndFrameUp;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector CustomSwingPlaneNormal = FVector::RightVector;

	/** Whether this context was produced by the aim ray path, which is true regardless of whether the
	 *  ray hit. It is the only thing distinguishing the two situations in which bHasAimGuideHit below
	 *  is false: aimed but missed, when true, versus no aiming at all, when false, as with a direct
	 *  Blueprint or AI call to Throw() with no wielder. The preview builder uses it to decide whether
	 *  to allow the arc to search again for a target: picking one up with the arc after the aim missed
	 *  would embed the rope in a neighbouring target that was never aimed at, breaking the
	 *  GuaranteedWrap contract that the guarantee applies to the target aimed at. */
	bool bAimRayEvaluated = false;

	/** Whether the aim ray secured a valid bone hit. */
	bool bHasAimGuideHit = false;

	/** The primary target bone chosen by the aim ray. In AssistedJudged only the first capture, that
	 *  is the dominant one, is pinned to this bone and other bones on the same mesh remain valid
	 *  multi-bone candidates. In GuaranteedWrap it is kept as the exact target. */
	FName AimGuideBone = NAME_None;

	/** The mesh or component used to resolve the target bone's transform and SDF. */
	TWeakObjectPtr<const USceneComponent> AimGuideMesh = nullptr;

	/** The world position at which the ray centreline first entered the target SDF. */
	FVector AimGuideHitWorldPos = FVector::ZeroVector;

	/** The aim hit expressed in the local space of the target bone (AimGuideBone), obtained by
	 *  transforming the world hit by the inverse of that bone's transform at the moment of aiming.
	 *  Applying the bone transform at the moment of consumption restores the current world position,
	 *  so the tip follows the exact point on the body that was aimed at even as the target moves. A
	 *  world-space position alone cannot track a moving target and would leave the tip hanging in
	 *  mid-air at commit time. */
	FVector AimGuideLocalHitPos = FVector::ZeroVector;

	/** Whether AimGuideLocalHitPos above holds a valid bone-local value. When false, consumers fall
	 *  back to the world-space AimGuideHitWorldPos, which covers having no aim bone or mesh, and
	 *  contexts built before the local hit existed. */
	bool bHasAimGuideLocalHit = false;

	/** The outward normal taken at the surface point. */
	FVector AimGuideNormal = FVector::UpVector;

	/**
	 * The span along the rope over which the blend towards the hit direction starts and completes.
	 * The spatial blend is multiplied by the Flight time blend, so no node is locked to the hit
	 * direction before the throw ends.
	 */
	float AimGuideSteerStartAlpha = 0.25f;
	float AimGuideLockAlpha = 0.50f;
};

/** Runtime centerline data for the pre-wrapped rope preview.
 *  It is a result the builder refills every frame rather than something authored, so it is exposed
 *  read-only. Making it editable would put an editable array of coordinates in the details panel of
 *  anything holding this struct as a member. */
USTRUCT(BlueprintType)
struct FRopeWrapPreviewData
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Rope|Preview")
	TArray<FVector> Points;

	UPROPERTY(BlueprintReadOnly, Category = "Rope|Preview", meta = (ClampMin = "0.1", Units = "cm"))
	float Radius = 2.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Rope|Preview", meta = (ClampMin = "3", ClampMax = "32"))
	int32 NumSides = 8;

	bool IsValid() const
	{
		return Points.Num() >= 2 && Radius > KINDA_SMALL_NUMBER;
	}
};

/** Throw and launch parameters for the flight stage. */
USTRUCT(BlueprintType)
struct FRopeThrowParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeThrowFrameMode FrameMode = ERopeThrowFrameMode::Owner;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeSwingPlane SwingPlane = ERopeSwingPlane::AimAndFrameUp;

	/** Fallback speed used when RopeComponent::Throw is called directly rather than through a
	 *  wielder. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "1.0", UIMin = "100.0", UIMax = "5000.0", Units = "cm/s", DisplayName = "Throw Speed"))
	float ThrowSpeed = 1500.0f;

	/** If nothing is captured within this time after the whip ends, the throw is treated as a failure
	 *  and the rope returns to Free (s). 0 uses the component's default failure cooldown,
	 *  ReleaseCooldownSeconds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (ClampMin = "0.0", Units = "s", DisplayName = "Return If No Contact"))
	float FlightNoContactReturnTime = 0.0f;

	/**
	 * Tip boost multiplier for the velocity injected by a throw: 1 distributes evenly, and above 1
	 * drives the far end harder so the tip runs ahead like a whip.
	 * It is not a mass. No mass of any kind reaches the solver; this is purely a presentation
	 * multiplier applied when distributing Verlet velocity at the moment of the throw. The value is
	 * the multiplier directly, and consumers clamp it to the range 0.25 to 3.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (ClampMin = "0.25", ClampMax = "3.0"))
	float TipVelocityBoost = 1.0f;

	/**
	 * The apex height of the GuidedThrow flight arc, as this fraction of the distance from the hand to
	 * the target. The tip traces an upward parabola to the target, which is either the point it embeds
	 * in or the far end of the ray for a throw into open space. 0 disables the arc and interpolates
	 * straight from the loaded pose to the target.
	 * The arc offset grows linearly towards the tip, and is 0 at an alpha of 1, so the landing point is
	 * preserved exactly.
	 * The upper limit of 0.5 puts the apex at half the hand-to-target distance. Beyond that the
	 * trajectory reads as being lobbed upwards and falling rather than flying to the target, and the
	 * visible flight diverges noticeably from the aiming line.
	 * Blueprint writes at runtime bypass this metadata, so consumers clamp to the same range.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning",
		meta = (ClampMin = "0.0", ClampMax = "0.5", DisplayName = "Arc Height"))
	float GuidedThrowArcHeightRatio = 0.25f;

	/**
	 * How much of the character's movement velocity the rope inherits on a throw, an exaggerated
	 * inertia for game feel. The physically true value is 1, but the default is 5 so the rope visibly
	 * runs ahead when thrown while sprinting. 0 inherits nothing, which matches throwing from
	 * standing.
	 * The hand socket's animation swing, that is the hand velocity relative to the character with its
	 * movement removed, is always carried at 1 regardless of this multiplier. The socket's world
	 * velocity already includes character movement, so that share is subtracted in
	 * ComputeThrowInheritedVelocity to avoid counting it twice.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (ClampMin = "0.0", DisplayName = "Motion Inheritance"))
	float MotionInheritance = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom", DisplayName = "Custom Forward"))
	FVector CustomFrameForward = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom", DisplayName = "Custom Up"))
	FVector CustomFrameUp = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom", DisplayName = "Custom Right"))
	FVector CustomFrameRight = FVector::RightVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (EditCondition = "SwingPlane == ERopeSwingPlane::CustomNormal", DisplayName = "Custom Plane Normal"))
	FVector CustomSwingPlaneNormal = FVector::RightVector;
};

/** Tuning for the whip swing at the start of a throw (FRopeWhipGuide). The runtime state is owned by
 *  URopeComponent::WhipGuide.
 *  The swing duration is derived from the throw speed against an internal reference of 0.35 s at
 *  1500 cm/s, giving EffectiveDuration = 0.35 * 1500 / ThrowSpeed; see MakeWhipGuideConfig and
 *  RopeWhipGuide::ResolveGuideDuration. Faster throws swing for less time, so throw speed alone sets
 *  both the swing and the flight strength and there is no separate duration knob. */
USTRUCT(BlueprintType)
struct FRopeWhipConfig
{
	GENERATED_BODY()

	/** The fraction of the rope length the guide controls, from 0 to 1, which shapes the swing
	 *  trajectory. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.1", ClampMax = "0.95"))
	float GuidedLength = 0.65f;

	/** The angle swept from the starting angle, opposite the aim, round to the aim direction. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "1.0", ClampMax = "180.0", Units = "deg", DisplayName = "Sweep Angle"))
	float SweepAngleDegrees = 180.0f;

	/**
	 * The fraction of the rope length at the hand end handed to the solver during an aim-hit flight.
	 * At 0 the central spline governs right up to the hand.
	 * The same value is also the range over which the hand socket offset is blended into the root
	 * stretch (see AimRootSocketInfluence in FRopeWhipGuide), so at 0 the guide is pinned to the origin
	 * captured at the moment of the throw and does not follow the hand animation.
	 * Fixed at this default and not exposed to designers. Consumers clamp it to the range 0 to 0.45.
	 */
	float AimHitRootSolverFraction = 0.20f;

	/**
	 * Bias of the hit direction blend. 1 is linear; larger values turn the spline towards the hit
	 * direction sooner at the same point in the flight.
	 * The aim-hit branch of RopeMath::BuildWhipGuideRawPoints uses a straight spline and does not read
	 * this value. It is kept as the place to reinstate when curved interpolation is added to that
	 * branch. Not exposed to designers.
	 */
	float AimHitDirectionBias = 2.0f;

	/** The fraction of the rope length at the free end handed to the solver during an aim-hit flight.
	 *  Larger values make the tip move more freely under its own inertia. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip|Tuning|Aim Hit", meta = (ClampMin = "0.0", ClampMax = "0.45", DisplayName = "Tip Physics Blend"))
	float AimHitTipSolverFraction = 0.25f;

	/** During an aim-hit flight, keeps the distance, bending and damping solvers but disables collider
	 *  push-out. Contact detection continues to run. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip|Tuning|Aim Hit", meta = (DisplayName = "Skip Collision"))
	bool bAimHitCollisionFreeSolve = true;
};

/** The prepared preview a GuaranteedWrap throw commits to at the moment of input, used both to render
 *  and as the input to GuidedThrow and the transition to Wrapped. */
struct DYNAMICROPE_API FRopePreparedThrowPreview
{
	bool bValid = false;

	/** The throw reference used to build the preview. Stored so the frame and origin from the moment
	 *  of input are preserved even when a montage delays the throw. */
	FRopeThrowContext ThrowContext;

	/** The preview centreline shown on screen. During GuidedThrow these points double as the actual
	 *  target positions for the nodes. */
	FRopeWrapPreviewData RenderPreview;

	/** A path that must be decoupled from socket animation, as with aim ray targeting, is also kept in
	 *  owner-local space. */
	bool bUseGuideFrameLocal = false;
	TWeakObjectPtr<const USceneComponent> GuideFrameComponent = nullptr;
	TArray<FVector> GuideFrameLocalPoints;
	FVector GuideFrameLocalOrigin = FVector::ZeroVector;

	/** The bone-local pinning information needed for the final transition to Wrapped. Points alone
	 *  cannot follow the character's movement. */
	FRopeSurfaceAnchor LatchAnchor;
	TArray<FRopeSurfaceAnchor> Anchors;

	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;
	FName Bone = NAME_None;

	void Reset()
	{
		*this = FRopePreparedThrowPreview();
	}

	bool IsValid() const
	{
		return bValid && RenderPreview.IsValid() && Mesh.IsValid() && !Bone.IsNone() && LatchAnchor.NodeIndex != INDEX_NONE;
	}

	/** Stores the world preview as it was at creation in coordinates local to the wielder's owner, so
	 *  it is decoupled from socket animation. */
	void StoreGuideFrameLocal(const USceneComponent* InGuideFrame);

	/** Whether both the owner-local guide frame and the local point data are valid. */
	bool HasGuideFrameLocal() const;

	/** Restores the stored owner-local origin to world space against the current owner transform. */
	FVector ResolveGuideOriginWorld() const;

	/** Restores the given owner-local spline point to world space against the current owner
	 *  transform. */
	FVector ResolveGuidePointWorld(int32 PointIndex) const;

	/** Resolves the whole preview into world data matching the current owner transform, for
	 *  rendering. */
	FRopeWrapPreviewData ResolveRenderPreviewWorld() const;
};

/** Working state of the GuidedThrow phase: the progress of authoritatively following the cached
 *  preview path. */
struct FRopeGuidedThrowState
{
	bool bActive = false;

	/** The prepared preview the wielder committed to. This phase does not search for contacts again
	 *  and follows this data alone. */
	FRopePreparedThrowPreview Prepared;

	/**
	 * A throw into open space, with no target: an arcing flight towards the far end of the ray. It
	 * follows the straight RenderPreview alone, with no target mesh, bone or anchor, and on completion
	 * falls to Free rather than embedding and becoming Wrapped. When false this is an ordinary aimed
	 * throw that embeds.
	 */
	bool bFreeThrow = false;

	/** The rope's actual positions at the moment GuidedThrow started, which are the starting point for
	 *  interpolating every node towards RenderPreview.Points. */
	TArray<FVector> StartPositions;
	float Elapsed = 0.0f;
	float Duration = 0.18f;

	void Reset()
	{
		*this = FRopeGuidedThrowState();
	}
};
