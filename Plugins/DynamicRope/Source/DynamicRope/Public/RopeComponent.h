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

class IRopeCollider;
class IRopeColliderProvider;
class UMaterialInterface;

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
};
