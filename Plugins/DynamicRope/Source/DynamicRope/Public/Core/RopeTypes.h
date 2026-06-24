// Copyright Epic Games, Inc. All Rights Reserved.
//
// Core data types for the Dynamic Rope system. Plain POD where it lives in the hot loop
// (sim/contact/wrap state); USTRUCT only for designer-facing config.

#pragma once

#include "CoreMinimal.h"
#include "RopeTypes.generated.h"

class USkeletalMeshComponent;

/** Lifecycle phase. Free/Flight/Contacting = physics (solver). Wrapped/Releasing = logic (wrap controller). */
UENUM(BlueprintType)
enum class ERopePhase : uint8
{
	Free,
	Flight,
	Contacting,
	Wrapped,
	Releasing
};

/** Why a wrap was released. */
UENUM(BlueprintType)
enum class ERopeReleaseReason : uint8
{
	Manual,
	Distance,
	Tension,
	Broken
};

/**
 * Narrow-phase contact: one rope node vs one collider, returned by IRopeCollider::Query.
 *
 * CONTRACT — FROZEN 2026-06-24. Every IRopeCollider (capsule, bone-SDF, world-GDF) MUST honor it.
 * Describes a single (node, collider) pair; aggregation is the caller's job (solver sums push-outs,
 * DecideWrap picks the deepest-penetration bone per node).
 *
 *   bHit         node sphere (center = query WorldPos, radius = query Radius) overlaps the collider.
 *                false => ALL other fields are undefined; callers must ignore them.
 *   Normal       UNIT, points OUT of the collider toward the node (the push-out direction).
 *                Invariant: NodePos += Normal*Penetration lands the node ON the surface.
 *                *** Sign is load-bearing: an inward normal sucks the rope into the body. ***
 *                Degenerate (node on the medial axis) => any stable unit vector (capsule: +Z).
 *   Penetration  overlap depth along Normal, > 0 when bHit. Measured against the QUERY radius:
 *                (ColliderRadius + QueryRadius) - Distance. Callers pass QueryRadius 0 for the
 *                solver push-out and WrapConfig.ContactRadius for the wrap-decision skin.
 *   SurfacePoint nearest point ON the collider surface to the node (aux/debug). Not required by
 *                the solver; fill it when cheap.
 *   Bone         REQUIRED non-None for skeletal colliders — the bone-attribution DecideWrap wraps
 *                on. A multi-bone SDF MUST report which bone owns the nearest surface. world => None.
 *   SourceMesh   skeletal mesh that owns Bone; carries cross-actor follow (-> FRopeWrapState::Mesh).
 *                null for non-skeletal colliders.
 */
struct FRopeContact
{
	bool    bHit = false;
	FVector Normal = FVector::UpVector;
	float   Penetration = 0.0f;
	FVector SurfacePoint = FVector::ZeroVector;
	FName   Bone = NAME_None;
	const USkeletalMeshComponent* SourceMesh = nullptr;
};

/** A latched wrap node, fixed in bone-local space so it follows skinning without re-collision. */
struct FRopeLatchNode
{
	int32   NodeIndex = INDEX_NONE;
	FName   Bone = NAME_None;
	FVector BoneLocalPos = FVector::ZeroVector;
};

/** Post-wrap data model (binding semantics). Produced at the physics→logic handoff. */
struct FRopeWrapState
{
	FName                   BoneName = NAME_None;
	TArray<FRopeLatchNode>  Latched;
	float                   WrapTurns = 0.0f;
	float                   Tension = 0.0f;
	float                   TimeWrapped = 0.0f;
	float                   AnchorDistance = 0.0f;

	// Mesh that owns BoneName; the wrap is held/followed against this mesh (may be a different
	// actor than the rope's owner). Resolved from the contact at decision time.
	const USkeletalMeshComponent* Mesh = nullptr;

	bool IsWrapped() const { return Latched.Num() > 0; }
	void Reset() { *this = FRopeWrapState(); }
};

/** The rope centerline: a chain of particles. Single source of truth for solver / logic / render. */
struct FRopeSimState
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<float>   InvMass;
	float           SegmentLength = 0.0f;
	float           RopeLength = 0.0f;

	// Pinned start (hand/socket). The solver sweeps it Prev->Target across substeps so a fast
	// anchor jump is absorbed instead of injecting energy (which would explode the chain).
	bool            bStartPinned = false;
	FVector         StartPinPrev = FVector::ZeroVector;
	FVector         StartPinTarget = FVector::ZeroVector;

	int32 Num() const { return Positions.Num(); }
	void  Reset() { Positions.Reset(); PrevPositions.Reset(); InvMass.Reset(); }
};

/** XPBD solver tuning (designer-facing). */
USTRUCT(BlueprintType)
struct FRopeSolverConfig
{
	GENERATED_BODY()

	/** Physics substeps per frame (anti-tunneling; "small steps" > more iterations). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "1", ClampMax = "16"))
	int32 Substeps = 4;

	/** Constraint iterations per substep. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "1"))
	int32 Iterations = 4;

	/** XPBD stretch compliance (inverse stiffness). 0 = inextensible. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0"))
	float StretchCompliance = 0.0f;

	/** XPBD bending compliance. Higher = floppier. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0"))
	float BendCompliance = 0.02f;

	/** Tangential friction [0..1] against colliders. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Friction = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.02f;
};

/** Contact-decision tuning: when does a draped rope count as "wrapped" on a limb? */
USTRUCT(BlueprintType)
struct FRopeWrapConfig
{
	GENERATED_BODY()

	/** Node radius used for the contact-decision query (cm). Separate from the visual tube radius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "cm"))
	float ContactRadius = 3.0f;

	/** Minimum number of rope nodes touching one bone to treat it as a catch (not a glancing brush). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "1"))
	int32 MinLatchNodes = 3;

	/** Contact must persist on the same bone this long before committing the wrap (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap", meta = (ClampMin = "0.0", Units = "s"))
	float WrapDecisionTime = 0.15f;
};

/** Throw / launch parameters for the flight phase. */
USTRUCT(BlueprintType)
struct FRopeThrowParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float ThrowSpeed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.1"))
	float TipMass = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0", Units = "cm"))
	float AimAssistRadius = 100.0f;
};
