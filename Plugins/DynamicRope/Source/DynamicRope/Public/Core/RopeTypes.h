// Copyright Epic Games, Inc. All Rights Reserved.
//
// Core data types for the Dynamic Rope system. Plain POD where it lives in the hot loop
// (sim/contact/wrap state); USTRUCT only for designer-facing config.

#pragma once

#include "CoreMinimal.h"
#include "RopeTypes.generated.h"

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

/** Narrow-phase contact result from an IRopeCollider. */
struct FRopeContact
{
	bool    bHit = false;
	FVector Normal = FVector::UpVector;
	float   Penetration = 0.0f;
	FVector SurfacePoint = FVector::ZeroVector;
	FName   Bone = NAME_None;
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
