// Copyright Epic Games, Inc. All Rights Reserved.
//
// 단일 UE 통합 지점(Facade). sim 상태, solver, wrap controller, 그리고 physics와 logic을
// 분기하는 phase state machine을 소유한다. 캐릭터에 붙이고 Throw()로 사용한다.

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

	//~ Setup(설정) -------------------------------------------------------
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "2"))
	int32 NumParticles = 24;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeSolverConfig SolverConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	FRopeThrowParams ThrowParams;

	/** physics → logic (wrap) 핸드오프를 위한 contact-decision 튜닝 값. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	FRopeWrapConfig WrapConfig;

	/** rope가 wrap될 수 있는 skeletal mesh. null로 두면 owner로부터 자동 해석된다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	TObjectPtr<USkeletalMeshComponent> WrapTargetMesh = nullptr;

	/**
	 * IRopeColliderProvider 컴포넌트가 이 rope에 collider를 공급하는 actor들. rope가 잡아야 할 body와
	 * *다른* actor 위에 존재할 때 설정한다(예: static prop에 고정된 rope가 별개의 캐릭터를 wrap하는 경우).
	 * 비어 있으면 이 컴포넌트 자신의 owner로 폴백한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap")
	TArray<TObjectPtr<AActor>> ColliderSourceActors;

	//~ Render(렌더) ------------------------------------------------------
	/** 시각적 tube 반지름(cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render", meta = (ClampMin = "0.1", Units = "cm"))
	float Radius = 2.0f;

	/** tube 단면의 변 개수. 높을수록 더 둥글어진다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render", meta = (ClampMin = "3", ClampMax = "32"))
	int32 NumSides = 8;

	/** rope tube에 적용되는 material. 설정하지 않으면 엔진 기본 material을 사용한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render")
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	/** 시뮬레이션된 centerline을 debug line으로 그린다(ground-truth 위치 vs 렌더링된 tube). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Render")
	bool bDrawDebugCenterline = false;

	//~ API ---------------------------------------------------------------
	/** rope를 발사한다: AimDir 방향의 초기 tip 속도를 가지고 Flight phase로 진입한다. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void Throw(const FVector& AimDir);

	/** 현재 wrap을 수동으로 해제한다(Releasing phase). */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void ReleaseWrap();

	UFUNCTION(BlueprintCallable, Category = "Rope")
	ERopePhase GetPhase() const { return Phase; }

	/**
	 * Debug: sustained-contact gate(MinLatchNodes / WrapDecisionTime)를 우회하여, rope가 현재 가장
	 * 가깝거나 접촉 중인 bone에 즉시 wrap을 commit한다. throw를 튜닝하지 않고도 BeginWrap 핸드오프와
	 * Hold(bone-follow)를 관찰할 수 있게 해 준다. 접촉이 없으면 false를 반환한다.
	 */
	UFUNCTION(BlueprintCallable, Category = "Rope|Debug")
	bool DebugForceWrap();

	//~ Events(이벤트) ----------------------------------------------------
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnWrapped OnRopeWrapped;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnCaptured OnRopeCaptured;

	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FRopeOnReleased OnRopeReleased;

private:
	ERopePhase Phase = ERopePhase::Free;

	// Non-UObject sim/solver/logic — 값으로 소유하며, GC 추적 대상이 아니다(POD).
	FRopeSimState       Sim;
	FRopeXPBDSolver     Solver;
	FRopeWrapController WrapController;

	/** 매 frame solver에 collider(skeletal bone, world)를 공급하는 source들. */
	UPROPERTY()
	TArray<TScriptInterface<IRopeColliderProvider>> ColliderProviders;

	void InitRope();
	void GatherFrameColliders(TArray<IRopeCollider*>& OutColliders) const;

	/** owner로부터 IRopeColliderProvider 컴포넌트를 모아 ColliderProviders에 캐싱한다. */
	void EnsureColliderProviders();

	/** rope가 wrap할 skeletal mesh를 해석(및 캐싱)한다: 명시적 WrapTargetMesh 또는 owner의 것. */
	USkeletalMeshComponent* ResolveWrapTargetMesh();
};
