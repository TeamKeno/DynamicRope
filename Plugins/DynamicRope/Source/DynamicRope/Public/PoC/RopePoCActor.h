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

	/**
	 * Physics substeps per frame. The frame is split into N steps, sweeping the pinned ends
	 * and the capsules between their previous and current poses each step. Higher = no
	 * tunneling at speed (the main fix for "fast rope passes through"). Cost scales ~linearly.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "1", ClampMax = "16", UIMin = "1", UIMax = "16"))
	int32 SimSubsteps = 4;

	/** Gravity applied to free particles. */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	/** Velocity damping per frame [0..1]. 0 = no damping. */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.02f;

	/**
	 * Bending stiffness [0..1] via Jakobsen support sticks (i↔i+2 distance constraints).
	 * 0 = limp chain, 1 = stiff rope that resists bending and holds its shape. Key knob for
	 * "looks like a rope, not a noodle" (RDR2-quality gap).
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Solver", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BendStiffness = 0.3f;

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
	 * Collide the rope SEGMENTS (not just the nodes) against the capsule, distributing the
	 * push-out to both end nodes (Jakobsen §5.2). Stops the rope sinking through between nodes
	 * on a curved limb — the key "wrap looks convincing" detail. Off = legacy node-only test.
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision")
	bool bUseSegmentCollision = true;

	/**
	 * How strongly a contacting rope sticks to the (moving) capsule surface.
	 * 0 = frictionless (slides off), 1 = fully grips (moves with the limb → wrap stays). Key knob for "감김 유지".
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WrapFriction = 0.6f;

	/** Distance beyond the capsule surface still treated as "in contact" for friction (cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Collision", meta = (ClampMin = "0.0", UIMin = "0.0", Units = "cm"))
	float FrictionContactBand = 2.0f;

	//~ Pull / two-way coupling (S4) --------------------------------------
	/**
	 * S4: feed the rope's contact reaction back into the capsule providers, so pulling the
	 * rope's end actually drags the wrapped limb. Needs a draggable provider (test capsule).
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Pull")
	bool bEnableTwoWayPull = true;

	/** Scales the reaction (collision push-out + latched-node tension) into the impulse handed to the capsule. */
	UPROPERTY(EditAnywhere, Category = "Rope|Pull", meta = (EditCondition = "bEnableTwoWayPull", ClampMin = "0.0"))
	float PullReactionGain = 8.0f;

	//~ Wrap latch / Hold state (S4 — doc 4.1) ----------------------------
	/**
	 * Once a contact node has stayed on a capsule long enough, latch it to that surface as
	 * data: it then holds the wrap regardless of tension/gravity (the production "Hold" model),
	 * follows the moving limb, and transmits pull to the capsule. The fix for "감아도 바로 풀림".
	 */
	UPROPERTY(EditAnywhere, Category = "Rope|Wrap")
	bool bEnableWrapLatch = true;

	/** Continuous contact time before a node latches (s). */
	UPROPERTY(EditAnywhere, Category = "Rope|Wrap", meta = (EditCondition = "bEnableWrapLatch", ClampMin = "0.0", Units = "s"))
	float LatchContactTime = 0.15f;

	/** Extra distance beyond the capsule surface that still counts as contact for latching (cm). */
	UPROPERTY(EditAnywhere, Category = "Rope|Wrap", meta = (EditCondition = "bEnableWrapLatch", ClampMin = "0.0", Units = "cm"))
	float LatchContactBand = 1.5f;

	/** Break the latch when an adjacent segment is stretched past this multiple of its rest length (yanked off). */
	UPROPERTY(EditAnywhere, Category = "Rope|Wrap", meta = (EditCondition = "bEnableWrapLatch", ClampMin = "1.0"))
	float LatchReleaseStrain = 1.8f;

	/** Release every latched wrap (e.g. on an "unwrap" input). doc 4.3 explicit release. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Wrap")
	void ReleaseAllWraps();

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

	// --- Wrap latch state (per particle) ---
	/** Capsule index this particle is latched to, or -1 if free. */
	TArray<int32>   LatchCapsule;
	/** Latched contact expressed relative to the capsule: distance along the axis from A... */
	TArray<float>   LatchAlong;
	/** ...and a radial direction + distance (world space, rotated to follow the axis each step). */
	TArray<FVector> LatchRadialDir;
	TArray<float>   LatchRadialDist;
	/** Capsule axis at the last update, to compute the incremental rotation as the limb moves. */
	TArray<FVector> LatchAxis;
	/** How long each particle has been continuously in contact (for the latch dwell test). */
	TArray<float>   ContactDwell;

	// Perf readout for the S2 GO/NO-GO budget check.
	float           LastSolveMs = 0.0f;
	float           AvgSolveMs = 0.0f;

	/** Capsule providers used this run (explicit list + auto-found), resolved on init. */
	TArray<TWeakObjectPtr<UObject>> CapsuleProviders;

	/** Capsules gathered from providers once per frame (current pose). */
	TArray<FRopeCapsule> FrameCapsules;

	/** Provider index (into CapsuleProviders) that produced each FrameCapsules entry. */
	TArray<int32> FrameCapsuleOwner;

	/** Last frame's capsules (same order), the start pose substeps interpolate from. */
	TArray<FRopeCapsule> PrevFrameCapsules;

	/** Capsules at the current substep (interpolated Prev→Frame); what collision/friction read. */
	TArray<FRopeCapsule> ActiveCapsules;
	/** Capsules at the previous substep, for friction's surface-velocity estimate. */
	TArray<FRopeCapsule> PrevActiveCapsules;

	/** Pinned-endpoint targets from last frame, so substeps can sweep the ends (anti-tunneling). */
	FVector PrevStartWorld = FVector::ZeroVector;
	FVector PrevEndWorld = FVector::ZeroVector;
	bool    bHasPrevPins = false;

	// --- S4 pull reaction accumulators (per FrameCapsules entry, reset each frame) ---
	/** Sum of reaction impulses the rope exerts on each capsule this frame. */
	TArray<FVector> CapsuleReaction;
	/** Contact-weighted application point for each capsule's reaction (world space). */
	TArray<FVector> CapsuleReactionPoint;
	/** Total contact weight, to average the application point. */
	TArray<float> CapsuleReactionWeight;

	void InitializeRope();
	void RebuildSegmentMeshes();
	void GatherProviders();
	void BuildFrameCapsules();
	void SimulateStep(float DeltaSeconds);
	void SolveConstraints(bool bReverse);
	void SolveBendingConstraints(bool bReverse);
	void SolveCollisions();
	void ApplyFriction();
	void ApplyPullReaction();
	void ApplyPinning();
	/** Pin endpoints to explicit world targets (used while sweeping ends across substeps). */
	void SetPinnedTargets(const FVector& StartW, const FVector& EndW);

	/** Re-place latched particles on their (moving) capsule surface; keep them pinned. */
	void UpdateLatchedPositions();
	/** Latch new long-contact nodes; break over-stretched ones. Once per frame. */
	void ManageWrapLatch(float FrameDt);
	/** Feed latched-node tension back to the capsules as pull reaction. Once per frame. */
	void AccumulateLatchReaction();
	void UpdateSegmentMeshes();
	void DrawDebugRope() const;

	FVector GetStartWorld() const;
	FVector GetEndWorld() const;
};
