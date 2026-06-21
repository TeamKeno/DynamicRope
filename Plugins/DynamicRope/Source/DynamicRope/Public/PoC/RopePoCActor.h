// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. Everything under PoC/ is disposable.
// S0: straight-rope PBD/Verlet solver + spline-mesh rendering. No body collision yet.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PoC/RopeCapsuleProvider.h"
#include "RopePoCActor.generated.h"

class USplineMeshComponent;
class UStaticMesh;
class UMaterialInterface;
class ARopePoCCapsuleActor;

/**
 * Proof-of-concept straight rope.
 * Simulates a chain of particles with Verlet integration + Position-Based-Dynamics
 * distance constraints, and renders the result as a chain of spline meshes.
 *
 * Ticks in the editor viewport (no PIE required) for fast iteration.
 */
UCLASS()
class DYNAMICROPE_API ARopePoCActor : public AActor
{
	GENERATED_BODY()

public:
	ARopePoCActor();

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual bool ShouldTickIfViewportsOnly() const override { return true; } // tick in editor viewport
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	//~ Setup -------------------------------------------------------------
	/** Number of simulated particles along the rope (>= 2). */
	UPROPERTY(EditAnywhere, Category = "Rope|Setup", meta = (ClampMin = "2", UIMin = "2"))
	int32 NumParticles = 24;

	/** Total rest length of the rope (cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Setup", meta = (ClampMin = "1.0", UIMin = "1.0", Units = "cm"))
	float RopeLength = 200.0f;

	//~ Solver ------------------------------------------------------------
	/** Constraint solver iterations per frame. More = stiffer / more stable. */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "1", UIMin = "1"))
	int32 SolverIterations = 12;

	/** Gravity applied to free particles. */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	/** Velocity damping per frame [0..1]. 0 = no damping. */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.02f;

	//~ Endpoints ---------------------------------------------------------
	/** Pin the first particle to this actor's origin. */
	UPROPERTY(EditAnywhere, Category = "Rope|Endpoints")
	bool bPinStart = true;

	/** Pin the last particle to EndAnchorActor (if set). Drag that actor to move the rope's free end. */
	UPROPERTY(EditAnywhere, Category = "Rope|Endpoints")
	bool bPinEnd = false;

	/** Optional actor the last particle is pinned to when bPinEnd is true. */
	UPROPERTY(EditAnywhere, Category = "Rope|Endpoints")
	TObjectPtr<AActor> EndAnchorActor = nullptr;

	//~ Collision ---------------------------------------------------------
	/** Explicit capsule providers (test capsule actors). Leave empty and rely on bAutoFindColliders. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision")
	TArray<TObjectPtr<ARopePoCCapsuleActor>> Colliders;

	/** Also collide against every IRopeCapsuleProvider (test capsules, skeletal limbs) found in the level. */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision")
	bool bAutoFindColliders = true;

	/** Contact thickness of the rope used for collision push-out (cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float RopeCollisionRadius = 2.0f;

	/**
	 * How strongly a contacting rope sticks to the (moving) capsule surface.
	 * 0 = frictionless (slides off), 1 = fully grips (moves with the limb → wrap stays). Key knob for "감김 유지".
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WrapFriction = 0.6f;

	/** Distance beyond the capsule surface still treated as "in contact" for friction (cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float FrictionContactBand = 2.0f;

	//~ Render ------------------------------------------------------------
	/** Mesh used per segment. Defaults to the engine cylinder if left empty. */
	UPROPERTY(EditAnywhere, Category = "Rope|Render")
	TObjectPtr<UStaticMesh> RopeMesh = nullptr;

	/** Material applied to the rope segments. Defaults to a basic material if left empty. */
	UPROPERTY(EditAnywhere, Category = "Rope|Render")
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	/** Visual rope radius (cm). Assumes a cylinder mesh of base radius 50. */
	UPROPERTY(EditAnywhere, Category = "Rope|Render", meta = (ClampMin = "0.1", UIMin = "0.1", Units = "cm"))
	float RopeRadius = 2.0f;

	//~ Debug -------------------------------------------------------------
	/** Draw the particle chain as debug lines/points. */
	UPROPERTY(EditAnywhere, Category = "Rope|Debug")
	bool bDrawDebug = true;

private:
	/** Root so the rope can be placed/moved as a whole. */
	UPROPERTY()
	TObjectPtr<USceneComponent> RopeRoot = nullptr;

	/** One spline mesh per segment (NumParticles - 1). Transient — rebuilt, never saved. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<USplineMeshComponent>> SegmentMeshes;

	// --- Transient simulation state (world space) ---
	TArray<FVector> Positions;
	TArray<FVector> OldPositions;
	TArray<float>   InvMasses;
	float           SegmentLength = 0.0f;
	bool            bInitialized = false;

	// Perf readout for the S2 GO/NO-GO budget check.
	float           LastSolveMs = 0.0f;
	float           AvgSolveMs = 0.0f;

	/** Capsule providers used this run (explicit list + auto-found), resolved on init. */
	TArray<TWeakObjectPtr<UObject>> CapsuleProviders;

	/** Capsules gathered from providers once per frame, reused across solver iterations. */
	TArray<FRopeCapsule> FrameCapsules;

	/** Last frame's capsules (same order), used to estimate surface velocity for friction. */
	TArray<FRopeCapsule> PrevFrameCapsules;

	void InitializeRope();
	void RebuildSegmentMeshes();
	void GatherProviders();
	void BuildFrameCapsules();
	void SimulateStep(float DeltaSeconds);
	void SolveConstraints();
	void SolveCollisions();
	void ApplyFriction();
	void ApplyPinning();
	void UpdateSegmentMeshes();
	void DrawDebugRope() const;

	FVector GetStartWorld() const;
	FVector GetEndWorld() const;
};
