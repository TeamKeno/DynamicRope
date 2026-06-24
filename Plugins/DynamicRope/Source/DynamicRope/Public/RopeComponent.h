// Copyright Epic Games, Inc. All Rights Reserved.
//
// The single UE integration point (Facade). Owns the sim state, solver, wrap controller and
// the phase state machine that routes physics vs logic. Drop on a character; Throw() to use.

#pragma once

#include "CoreMinimal.h"
#include "Components/MeshComponent.h"
#include "Core/RopeTypes.h"
#include "Solver/RopeXPBDSolver.h"
#include "Logic/RopeWrapController.h"
#include "RopeComponent.generated.h"

class AActor;
class IRopeCollider;
class IRopeColliderProvider;
class UMaterialInterface;
class USkeletalMeshComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnWrapped, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRopeOnCaptured, FName, Bone);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeOnReleased, FName, Bone, ERopeReleaseReason, Reason);

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeComponent : public UMeshComponent
{
	GENERATED_BODY()

public:
	URopeComponent();

	//~ UActorComponent
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void SendRenderDynamicData_Concurrent() override;

	//~ UPrimitiveComponent / UMeshComponent
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual int32 GetNumMaterials() const override;
	virtual UMaterialInterface* GetMaterial(int32 ElementIndex) const override;
	virtual void SetMaterial(int32 ElementIndex, UMaterialInterface* Material) override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

	//~ Setup -------------------------------------------------------------
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "2"))
	int32 NumParticles = 24;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeSolverConfig SolverConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeThrowParams ThrowParams;

	/** Contact-decision tuning for the physics → logic (wrap) handoff. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	FRopeWrapConfig WrapConfig;

	/** Skeletal mesh the rope can wrap onto. Auto-resolved from the owner if left null. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	TObjectPtr<USkeletalMeshComponent> WrapTargetMesh = nullptr;

	/**
	 * Actors whose IRopeColliderProvider components feed this rope. Set this when the rope lives on a
	 * *different* actor than the body it should catch (e.g. rope anchored to a static prop, wrapping
	 * a separate character). If empty, falls back to this component's own owner.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	TArray<TObjectPtr<AActor>> ColliderSourceActors;

	//~ Render ------------------------------------------------------------
	/** Visual tube radius (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render", meta = (ClampMin = "0.1", Units = "cm"))
	float Radius = 2.0f;

	/** Cross-section sides of the tube. Higher = rounder. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render", meta = (ClampMin = "3", ClampMax = "32"))
	int32 NumSides = 8;

	/** Material applied to the rope tube. Defaults to the engine default material if unset. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render")
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	/** Draw the simulated centerline as debug lines (ground-truth position vs the rendered tube). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render")
	bool bDrawDebugCenterline = false;

	//~ API ---------------------------------------------------------------
	/** Launch the rope: enters Flight phase with an initial tip velocity along AimDir. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw(const FVector& AimDir);

	/** Manually release the current wrap (Releasing phase). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ReleaseWrap();

	UFUNCTION(BlueprintCallable, Category = "Rope")
	ERopePhase GetPhase() const { return Phase; }

	/**
	 * Debug: immediately commit a wrap onto whichever bone the rope is currently nearest/touching,
	 * bypassing the sustained-contact gate (MinLatchNodes / WrapDecisionTime). Lets you observe the
	 * BeginWrap handoff and Hold (bone-follow) without tuning the throw. Returns false if no contact.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope|Debug")
	bool DebugForceWrap();

	//~ Events ------------------------------------------------------------
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnWrapped OnRopeWrapped;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnCaptured OnRopeCaptured;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnReleased OnRopeReleased;

private:
	ERopePhase Phase = ERopePhase::Free;

	// Non-UObject sim/solver/logic — owned by value, not GC-tracked (POD).
	FRopeSimState       Sim;
	FRopeXPBDSolver     Solver;
	FRopeWrapController WrapController;

	/** Sources that feed colliders (skeletal bones, world) to the solver each frame. */
	UPROPERTY()
	TArray<TScriptInterface<IRopeColliderProvider>> ColliderProviders;

	void InitRope();
	void GatherFrameColliders(TArray<IRopeCollider*>& OutColliders) const;

	/** Collect IRopeColliderProvider components from the owner (cached) into ColliderProviders. */
	void EnsureColliderProviders();

	/** Resolve (and cache) the skeletal mesh the rope wraps onto: explicit WrapTargetMesh or owner's. */
	USkeletalMeshComponent* ResolveWrapTargetMesh();
};
