// Copyright Epic Games, Inc. All Rights Reserved.
//
// The opt-in for wrapping static meshes. Add it to a wrappable static or movable actor, such as a
// pillar, a lamppost or a hook, where it acts as both a marker and a collider provider. Each frame it
// builds and serves wrappable analytic colliders around the target geometry. Unlike the push-out
// colliders of the static world provider these report IsWorldStatic() as false, so they take part in
// contact detection, and they report a synthetic virtual bone name plus a source mesh, which is the
// target component, so they follow the existing contact-to-wrap path with no change to the frozen
// FRopeContact contract.
//
// Once wrapped, the anchors follow the target component's transform through the static branch of
// ResolveBindingWorld. A static target never moves, which makes holding trivial, and a movable prop
// such as an elevator pillar is supported for free by following the component.
//
// Scope: with Shape set to Auto and simple collision present, it serves the full set, meaning every
// sphyl, sphere and box element of the simple collision, plus the OBB fallback for convexes, each
// attributed to the virtual bone. The rope then wraps with exactly the precision
// URopeStaticBodyProvider extracts; genuine convex elements wrap too, through the detect kernel's
// convex loop. Forcing Shape to Capsule or Box, or having no
// simple collision at all, falls back to a single shape approximated from the dominant primitive or
// the bounds. Wrapping a group, such as both legs of a character, is separate work.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
// FCapsuleCollider and FRopeBoxCollider, held by value as members.
#include "Collision/RopeCollider.h"
#include "Collision/RopeStaticCollider.h"
#include "RopeWrapTargetComponent.generated.h"

class USceneComponent;

/** The long axis of the wrap capsule, in the target component's local space. EAxis::Type is not a
 *  UENUM and cannot be used in a UPROPERTY, hence this dedicated enum. */
UENUM(BlueprintType)
enum class ERopeWrapAxis : uint8
{
	X,
	Y,
	Z
};

/** The shape used for the wrap target. */
UENUM(BlueprintType)
enum class ERopeWrapShape : uint8
{
	/** Chosen automatically from the dominant primitive of the simple collision: a box becomes Box, a
	 *  sphyl or sphere becomes Capsule, and with none at all it becomes Capsule. */
	Auto,

	/** Force a capsule, for cylinders and pillars. */
	Capsule,

	/** Force a box, as an OBB. */
	Box
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeWrapTargetComponent : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeWrapTargetComponent();

	//~ UActorComponent. Registers with and unregisters from the RopeSimSubsystem's central registry,
	//~ which gathers once per frame.
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** The geometry to be wrapped, static or movable. Leave it empty to use the owner's first static
	 *  mesh component, or the root component when there is none. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target")
	TObjectPtr<USceneComponent> TargetComponent = nullptr;

	/** An override for the wrap capsule radius (cm). At or below 0 it is derived automatically from the
	 *  authored collision or the bounds. In full-set mode it is applied uniformly to every extracted
	 *  capsule element, leaving box elements unaffected; on the single-shape path it behaves as
	 *  before. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target", meta = (ClampMin = "0.0", Units = "cm"))
	float Radius = 0.0f;

	/** Whether to pick the capsule's long axis automatically as the longest axis of the target bounds.
	 *  Turn it off to use Axis below. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target")
	bool bAutoAxis = true;

	/** The capsule's long axis in the target component's local space: Z for a pillar, which is the
	 *  default, or X or Y for a crossbeam. Used only while bAutoAxis is off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target", meta = (EditCondition = "!bAutoAxis"))
	ERopeWrapAxis Axis = ERopeWrapAxis::Z;

	/** The wrap target shape. Auto, the default, serves the full set of simple collision, so every
	 *  sphyl, sphere and box can be wrapped at the precision of the authored collision, falling back to
	 *  a bounds capsule when there is no simple collision. Capsule and Box force the single-shape path,
	 *  approximated from the dominant primitive, as a designer escape hatch. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target")
	ERopeWrapShape Shape = ERopeWrapShape::Auto;

	/**
	 * The synthetic virtual bone name for this wrap target, used as the FRopeContact::Bone attribution
	 * that contact aggregation binds the wrap to. Leave it empty to issue one automatically from the
	 * target component's name. Naming an actual socket on the static mesh makes the hold follow that
	 * socket; any other, virtual, name makes it follow the component transform.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target")
	FName WrapBoneName = NAME_None;

	//~ IRopeColliderProvider
	// ProvidesWorldStaticColliders keeps its default of false: this is a wrap target rather than a
	// static world push-out provider, so it must take part in detection and is subject to the same
	// owner exclusion rule as the skeletal providers.
	virtual void GatherColliders(FRopeColliderGatherContext& Gather) override;

private:
	/** The backing storage, rebuilt once per frame; the pointers handed out stay valid until that
	 *  frame's solve finishes. Capsule is served when Shape is Capsule and Box when Shape is Box, the
	 *  latter as a wrappable OBB carrying the virtual bone and source mesh. */
	FCapsuleCollider Capsule;
	FRopeBoxCollider Box;
	FName            ResolvedBone = NAME_None;
	uint64           BuiltFrame = static_cast<uint64>(-1);

	/**
	 * The wrappable backing storage for full-set mode, used when Shape is Auto and simple collision
	 * exists. Every sphyl and sphere, served as capsules, and every box, plus the OBB fallback for
	 * convexes, takes part in detection attributed to the virtual bone and source mesh, at the same
	 * precision the static body provider extracts. Genuine convex elements are attributed the same
	 * way into WrapConvexes and wrap as well. Rebuilt once per frame, with pointers valid until that
	 * frame's solve finishes.
	 */
	TArray<FRopeBoxCollider>           WrapBoxes;
	TArray<FRopeStaticCapsuleCollider> WrapCapsules;
	TArray<FRopeConvexCollider>        WrapConvexes;

	/** Whether this frame is being served in full-set mode, in which case the Wrap arrays are used
	 *  instead of the single capsule or box. */
	bool bServeFullSet = false;

	/** For the surface velocity of a movable prop: the previous frame's endpoints plus the reciprocal
	 *  delta time. A static target leaves InvDeltaTime at 0, giving zero velocity. */
	FVector PrevA = FVector::ZeroVector;
	FVector PrevB = FVector::ZeroVector;
	bool    bHasPrevEndpoints = false;

	/**
	 * The detailed push-out backing storage. When the target sits on a channel outside the static body
	 * provider's scan, such as PhysicsBody for a movable simulating prop that is dragged around, its
	 * entire simple collision is placed here and served alongside the wrap shapes. These have no bone,
	 * which makes IsWorldStatic() true and excludes them from detection, leaving them push-out only,
	 * while only the single capsule or box takes part in wrapping. It is rebuilt once per frame, so the
	 * pointers stay valid until the solve finishes, on the same contract as the static body provider.
	 */
	TArray<FRopeBoxCollider>            PushOutBoxes;
	TArray<FRopeStaticCapsuleCollider> PushOutCapsules;
	TArray<FRopeConvexCollider>        PushOutConvexes;

	/** For the surface velocity of movable push-out shapes: the previous frame's component world
	 *  transform, plus the reciprocal delta time. A static target leaves that at 0. */
	FTransform PrevCompTM = FTransform::Identity;
	bool       bHasPrevCompTM = false;

	/** A one-shot guard so the diagnostic log, covering registration, the target and the capsule state,
	 *  is written once rather than spamming. */
	bool    bDiagnosticsLogged = false;

	/** Whether the last capsule came from simple collision, when true, or the bounds fallback, when
	 *  false. For the diagnostic log. */
	bool    bUsedSimpleCollision = false;

	/** The shape being served this frame: true for a box and false for a capsule. Under Auto,
	 *  EffectiveServeBox decides it from the simple collision. */
	bool    bServeBox = false;

	/** Resolves the target component and settles the virtual bone name. Null on failure. */
	USceneComponent* ResolveTarget();

	/** Builds the wrap capsule into Capsule, carrying the virtual bone and the source mesh. It prefers
	 *  simple collision and falls back to the bounds. */
	void BuildCapsule(USceneComponent* Comp);

	/**
	 * Builds a wrappable box into Box, carrying the virtual bone and the source mesh. It uses the
	 * largest box element of the simple collision as a tight OBB, falling back to the component's local
	 * bounds OBB.
	 */
	void BuildBox(USceneComponent* Comp);

	/**
	 * Under Shape set to Auto, decides whether to serve a box for this target: true when the dominant,
	 * that is largest, primitive of the simple collision is a box, and false for a sphyl or sphere.
	 * With no simple collision it returns false, giving the capsule bounds fallback. Forcing Capsule or
	 * Box uses that choice directly.
	 */
	bool EffectiveServeBox(USceneComponent* Comp) const;

	/**
	 * Builds full-set mode: extracts the target's simple collision with attribution into WrapBoxes,
	 * WrapCapsules and WrapConvexes, all wrappable. Returns false when not a single wrappable element
	 * results, in which case the caller falls back to the single-shape path, as with a target that has
	 * no authored collision.
	 */
	bool BuildFullSet(USceneComponent* Comp);

	/**
	 * Extracts world-space capsule endpoints and a radius from the target's UBodySetup simple collision,
	 * covering sphyls, boxes and spheres. Being tight to the visual mesh removes any floating gap.
	 * Returns false when there is no authored collision or it holds convexes only, in which case the
	 * caller falls back to the bounds.
	 */
	bool BuildCapsuleFromSimpleCollision(USceneComponent* Comp, FVector& OutA, FVector& OutB, float& OutRadius) const;

	/** Approximates an axis-aligned wrap capsule, as endpoints and a radius, from the target's local
	 *  bounds. This is the fallback when there is no simple collision. */
	void BuildCapsuleFromBounds(USceneComponent* Comp, FVector& OutA, FVector& OutB, float& OutRadius) const;
};
