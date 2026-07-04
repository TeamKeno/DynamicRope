// Copyright Epic Games, Inc. All Rights Reserved.
//
// Wrapping 페이즈 로직: Contacting에서 확정된 latch anchor로부터 감김 경로를 점진 생성하고
// (AnalyticHelix / SurfaceVectorField), 로프 앞단(front)을 경로 따라 이동시키며, 감긴 노드의
// 질량을 마스킹하고, 커밋 시드(FRopeWrapState)를 조립한다. 물리가 아니라 LOGIC이다 —
// FRopeWrapController(Wrapped 이후)의 앞 단계에 해당한다.
//
// FRopeWrapController와 같은 패턴: 작업 상태(FRopeWrappingState)를 값으로 소유하는
// UObject 비의존 클래스. 페이즈 전이/이벤트 브로드캐스트는 URopeComponent가 결정하고,
// 여기는 상태와 지오메트리만 다룬다. UObject 컨텍스트(WrapConfig, collider 스냅샷,
// 경로 모드, 로그용 이름)는 FContext로 호출마다 주입받는다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class USkeletalMeshComponent;
class IRopeCollider;

class DYNAMICROPE_API FRopeWrappingPhase
{
public:
	/** Wrapping 작업 상태(POD). 전이 시 URopeComponent::ResetTransientPhaseState가 Reset한다. */
	FRopeWrappingState State;

	// 한 호출의 컨텍스트(비소유 참조 묶음 — 호출 동안만 유효).
	// 디자이너 설정 원본(UPROPERTY)은 URopeComponent에 남고 여기로 참조만 넘어온다.
	struct FContext
	{
		const FRopeWrapConfig& Config;             // 감김 튜닝(pitch/tail delay/step budget/contact radius)
		const TArray<IRopeCollider*>& Colliders;   // 표면 투영용 프레임 collider 스냅샷
		ERopeWrappingPathMode PathMode;            // UDynamicRopeSettings에서 컴포넌트가 해석해 전달
		float SurfaceOffset;                       // 튜브 반지름(표면에서 로프 중심까지 띄우는 거리)
		FString OwnerName;                         // 로그 컨텍스트(컴포넌트 이름)
	};

	/**
	 * Wrapping 시작: 상태를 시드(bone/mesh/duration)로 채우고 latch anchor에서 progressive
	 * 경로 빌드를 개시한다(첫 경로점+앵커 확보까지). 성공 시 안정 추적도 초기화한다.
	 * @return 경로 빌드를 시작할 수 없으면 false — 호출자는 상태를 버리고 Flight로 돌아가야 한다.
	 */
	bool Begin(const FRopeSurfaceAnchor& LatchAnchor, const USkeletalMeshComponent* Mesh, FName Bone,
		float Duration, const FRopeSimState& Sim, const FContext& Ctx);

	/** wrapping을 계속할 수 있는 상태인가(활성 + mesh 생존 + bone 유효). */
	bool IsStillValid() const
	{
		return State.IsActive() && State.Mesh.IsValid() && !State.BoneName.IsNone();
	}

	/** 프레임 예산(WrappingPathBuildStepsPerFrame)만큼 경로/앵커 점진 생성을 전진시킨다. */
	void AdvancePathBuild(const FRopeSimState& Sim, const FContext& Ctx);

	/** front를 경로 따라 전진시키고 latch 이후 노드들의 경로 위 타깃(+표면 오프셋)을 OutFrame에 담는다(G2). */
	void ApplyFrontMotion(const FRopeSimState& Sim, float DeltaTime, const FContext& Ctx, FRopeNodeOverrideFrame& OutFrame);

	/** 감긴(anchor) 노드 InvMass=0, 나머지 1(시작 핀 유지)을 OutFrame에 담는다 — 솔버가 감긴 구간을 건드리지 않게. */
	void ApplyMassMask(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const;

	/** 앵커 span이 변하지 않는 시간(StableTime)을 누적한다. 커밋 판정 보조 지표. */
	void UpdateStability(float DeltaTime);

	/** 커밋 조건: 앵커 확보 + 경로 빌드 종료 + front 도달 + (모션 완료 또는 settle 타임아웃). */
	bool IsReadyToCommit(const FRopeSimState& Sim, const FRopeWrapConfig& Config) const;

	/**
	 * 경로 생성이 실패했을 때, 마지막으로 성공한 지점이 helix 기준 최소 회전량에 못 미치면
	 * "감긴 척 붙는" 상태로 커밋하지 않도록 abort 여부를 알려준다.
	 */
	bool ShouldAbortFailedShortWrap(const FRopeSimState& Sim, const FContext& Ctx,
		float MinRequiredTurns, float& OutTurns) const;

	/**
	 * 현재 앵커들로 Wrapped 핸드오프용 시드를 조립한다(FRopeWrapController::BeginWrap 입력).
	 * 유효 노드가 없으면 Anchors가 빈 시드가 반환된다 — 호출자가 검사해 abort한다.
	 */
	FRopeWrapState BuildCommitSeed(const FRopeSimState& Sim, const USkeletalMeshComponent* Mesh) const;

	/** abort 시 앵커 노드들의 솔버 복귀(InvMass=1 + Prev=Pos 튐 방지)를 OutFrame에 담는다. */
	void ReturnNodesToSolver(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const;

private:
	//~ progressive 경로 빌드(프레임 분할). Begin이 개시하고 AdvancePathBuild가 예산만큼 전진.
	bool BeginProgressiveWrapPathBuild(const FRopeSurfaceAnchor& LatchAnchor,
		const FRopeSimState& Sim, const FContext& Ctx);

	bool AppendAnalyticProgressiveWrapPathPoint(int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx);

	bool InitializeSurfaceVectorFieldProgressiveWrapPath(const FRopeSurfaceAnchor& LatchAnchor,
		const FRopeSimState& Sim, const FContext& Ctx);

	bool AdvanceSurfaceVectorFieldProgressiveWrapPath(int32 StepBudget, const FRopeSimState& Sim, const FContext& Ctx);

	bool AppendWrappingAnchorFromPathPoint(int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx);

	//~ 감김 지오메트리
	FVector ComputeSurfaceVectorFieldTangent(const FVector& AxisOrigin, const FVector& AxisDirection,
		const FVector& LatchRadial, float WindingSign, const FVector& SurfaceWorld,
		const FVector& NormalWorld, const FContext& Ctx, FVector& InOutCircumferenceDir) const;

	bool ComputeAnalyticHelixWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
		const FRopeSimState& Sim, const FContext& Ctx,
		FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const;

	bool ComputeSurfaceVectorFieldWrapTarget(const FRopeSurfaceAnchor& LatchAnchor, float DistanceFromLatch,
		const FRopeSimState& Sim, const FContext& Ctx,
		FVector& OutSurfaceWorld, FVector& OutNormalWorld, FVector& OutTangentWorld) const;

	/** 마지막 성공 path/anchor 거리를 helix 공식에 넣어 누적 회전 수를 계산한다. */
	bool ComputeHelixTurnsAtLastBuiltPoint(const FRopeSimState& Sim, const FContext& Ctx, float& OutTurns) const;

	bool ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor,
		FVector& OutAxisOrigin, FVector& OutAxisDirection) const;

	void OrientWrappingAxisByTail(const FRopeSurfaceAnchor& LatchAnchor, const FRopeSimState& Sim,
		const USkeletalMeshComponent* Mesh, FVector& InOutAxisDirection) const;

	bool ProjectWrapPointToSurface(FName Bone, const USkeletalMeshComponent* Mesh,
		const FRopeSimState& Sim, const FContext& Ctx,
		FVector& InOutSurfaceWorld, FVector& InOutNormalWorld) const;

	//~ front 이동
	void AdvanceWrappingFront(float DeltaTime, const FRopeSimState& Sim, const FContext& Ctx);

	bool SampleWrappingPath(float DistanceFromLatch, FRopeWrapPathPoint& OutPoint) const;
};
