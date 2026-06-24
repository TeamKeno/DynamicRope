// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — 실험용이며 출시 대상 아님. PoC/ 아래의 모든 것은 폐기 가능한 코드다.
// S1: rope가 얹힐 수 있는 단일 capsule collider. S2에서 "움직이는 arm"으로 재사용된다.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PoC/RopeCapsuleProvider.h"
#include "RopePoCCapsuleActor.generated.h"

class UCapsuleComponent;

/**
 * Proof-of-concept capsule collider.
 * 월드 공간 swept-sphere segment(두 중심 + radius)를 노출해 rope solver가 파티클을
 * 밖으로 밀어낼 수 있게 한다. 캐릭터 limb의 physics-asset capsule을 대신하는 stand-in.
 */
UCLASS()
class DYNAMICROPE_API ARopePoCCapsuleActor : public AActor, public IRopeCapsuleProvider
{
	GENERATED_BODY()

public:
	ARopePoCCapsuleActor();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual bool ShouldTickIfViewportsOnly() const override { return true; } // 에디터 뷰포트에서 swing
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	/** Capsule radius(cm). */
	UPROPERTY(EditAnywhere, Category = "Capsule", meta = (ClampMin = "1.0", UIMin = "1.0", Units = "cm"))
	float Radius = 30.0f;

	/** 반구형 cap을 포함한 capsule half-height(cm). */
	UPROPERTY(EditAnywhere, Category = "Capsule", meta = (ClampMin = "1.0", UIMin = "1.0", Units = "cm"))
	float HalfHeight = 60.0f;

	//~ Auto-swing (S2) — capsule을 움직이는 limb처럼 애니메이션 -----------

	/** capsule을 pivot을 중심으로 진동시켜, rope가 움직이는 target을 따라가야 하게 만든다. */
	UPROPERTY(EditAnywhere, Category = "Swing")
	bool bAutoSwing = false;

	/** 한쪽으로의 최대 swing 각도(deg). */
	UPROPERTY(EditAnywhere, Category = "Swing", meta = (EditCondition = "bAutoSwing", ClampMin = "0.0", Units = "deg"))
	float SwingAngleDeg = 60.0f;

	/** 왕복 한 번에 걸리는 시간(s). 작을수록 빠르고 solver에 더 부담. */
	UPROPERTY(EditAnywhere, Category = "Swing", meta = (EditCondition = "bAutoSwing", ClampMin = "0.05", Units = "s"))
	float SwingPeriod = 2.0f;

	/** capsule이 회전하는 로컬 axis(기본 Y → X-Z 평면에서 swing). */
	UPROPERTY(EditAnywhere, Category = "Swing", meta = (EditCondition = "bAutoSwing"))
	FVector SwingAxis = FVector(0.0f, 1.0f, 0.0f);

	/** 로컬 공간의 pivot 지점. 기본값 (0,0,HalfHeight) ≈ top cap = "어깨"라서 arm처럼 sweep한다. */
	UPROPERTY(EditAnywhere, Category = "Swing", meta = (EditCondition = "bAutoSwing"))
	FVector SwingPivotOffset = FVector(0.0f, 0.0f, 60.0f);

	//~ Pull / draggable (S4) — rope의 접촉 반작용에 반응 ------

	/** rope가 이 capsule을 끌 수 있게 한다("limb가 당겨짐"). 반작용은 soft body로 integration된다. */
	UPROPERTY(EditAnywhere, Category = "Pull")
	bool bDraggable = true;

	/** 무거울수록 끌기 어렵다. 반작용 impulse를 이 값으로 나눈다. 작으면 limb가 끌려 들어온다. */
	UPROPERTY(EditAnywhere, Category = "Pull", meta = (EditCondition = "bDraggable", ClampMin = "0.1"))
	float Mass = 10.0f;

	/** drag offset의 속도 damping(초당). 클수록 더 빨리 안정되고/drift가 적다. */
	UPROPERTY(EditAnywhere, Category = "Pull", meta = (EditCondition = "bDraggable", ClampMin = "0.0"))
	float DragDamping = 2.0f;

	/**
	 * capsule을 rest 포즈로 되돌리는 spring(초^2당).
	 * 0 = limb가 끌려간 자리에 머문다(capture 느낌 — 권장). 탄성 snap-back을 원하면 값을 올린다.
	 */
	UPROPERTY(EditAnywhere, Category = "Pull", meta = (EditCondition = "bDraggable", ClampMin = "0.0"))
	float ReturnStiffness = 0.0f;

	/**
	 * capsule의 월드 공간 내부 segment(두 sphere 중심)와 그 radius.
	 * 어떤 점이 segment [OutA, OutB]까지의 거리가 OutRadius보다 작으면 capsule 내부에 있다.
	 */
	void GetCapsuleSegment(FVector& OutA, FVector& OutB, float& OutRadius) const;

	//~ IRopeCapsuleProvider
	virtual void GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const override;
	virtual void ApplyRopeReaction(const FVector& WorldImpulse, const FVector& WorldLocation) override;

private:
	UPROPERTY(VisibleAnywhere, Category = "Capsule")
	TObjectPtr<UCapsuleComponent> Capsule = nullptr;

	/** swing이 중심으로 진동하는, 캡처된 rest 포즈. */
	FTransform RestTransform = FTransform::Identity;
	float SwingElapsed = 0.0f;

	// --- S4 drag 상태(월드 공간) ---
	/** 마지막 Tick 이후 rope로부터 받은 impulse. */
	FVector PendingImpulse = FVector::ZeroVector;
	/** 현재 drag 속도와 rest/swing 포즈로부터 누적된 offset. */
	FVector DragVelocity = FVector::ZeroVector;
	FVector DragOffset = FVector::ZeroVector;
};
