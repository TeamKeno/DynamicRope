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

// 랩 대상 추상화(Decision 0): mesh 파라미터를 USceneComponent로 일반화. 스켈레탈 그래프 API가 필요한
// 함수(축/후보 본)는 RopeWrapTargets:: 구조 질의(부모/자식 키)로 해석한다 — 정적 대상은 그래프가
// 비어 단일 본으로 자연 폴백(스켈레탈 가정은 Core/RopeWrapTarget.cpp에 격리).
class USceneComponent;
class IRopeCollider;

class DYNAMICROPE_API FRopeWrappingPhase
{
public:
	/** Wrapping 작업 상태(POD). 전이 시 URopeComponent::ResetTransientPhaseState가 Reset한다. */
	FRopeWrappingState State;

	/**
	 * 한 호출의 컨텍스트(비소유 참조 묶음 — 호출 동안만 유효).
	 * 디자이너 설정 원본(UPROPERTY)은 URopeComponent에 남고 여기로 참조만 넘어온다.
	 */
	struct FContext
	{
		/** 감김 튜닝(pitch/tail delay/step budget/contact radius). */
		const FRopeWrapConfig& Config;

		/** 표면 투영용 프레임 collider 스냅샷. */
		const TArray<IRopeCollider*>& Colliders;

		/** UDynamicRopeSettings에서 컴포넌트가 해석해 전달. */
		ERopeWrappingPathMode PathMode;

		/** 튜브 반지름(표면에서 로프 중심까지 띄우는 거리). */
		float SurfaceOffset;

		/** 로그 컨텍스트(컴포넌트 이름). */
		FString OwnerName;

		/** preview처럼 partial path를 정상 결과로 쓰는 호출에서 true. */
		bool bSuppressPathFailureLog = false;

		/** Flight whip guide spline 평면 normal을 runtime wrapping에 전달한다. */
		bool bHasGuidePlaneNormal = false;
		FVector GuidePlaneNormal = FVector::RightVector;

		/**
		 * 캡처 순간의 진행 좌표계 스냅샷(비소유 — 컴포넌트의 CaptureTravelFrame, 유효할 때만 non-null).
		 * TravelPlaneFirst에서만 소비된다: 축 origin을 latch 본 위치 대신 접촉 영역 중심(RegionCenter)에
		 * 두고(양다리에서 축이 쌍의 중심을 지나게), winding 부호를 latch tangent 대신 캡처 속도로 정한다.
		 * ShapeAxisFirst(기본)에서는 읽지 않는다 — 기존 동작 불변.
		 */
		const FRopeCaptureTravelFrame* TravelFrame = nullptr;
	};

	/**
	 * Wrapping 시작: 상태를 시드(bone/mesh/duration)로 채우고 latch anchor에서 progressive
	 * 경로 빌드를 개시한다(첫 경로점+앵커 확보까지). 성공 시 안정 추적도 초기화한다.
	 * 시드 다중화(MaxWrapSeeds > 1): 보조 시드 앵커는 State.SecondarySeedAnchors에 *이 호출 전에*
	 * 실어야 한다 — 경로 길이(NumTailNodes)를 첫 보조 노드 앞까지로 클램프하는 데 여기서 읽는다.
	 * @return 경로 빌드를 시작할 수 없으면 false — 호출자는 상태를 버리고 Flight로 돌아가야 한다.
	 */
	bool Begin(const FRopeSurfaceAnchor& LatchAnchor, const USceneComponent* Mesh, FName Bone,
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
	 * 경로 생성이 실패했을 때, 마지막으로 성공한 지점까지의 감싼 각도(도)가 최소치에 못 미치면
	 * "감긴 척 붙는" 상태로 커밋하지 않도록 abort 여부를 알려준다. 기준이 회전 수가 아니라 각도인
	 * 이유는 FRopeWrapConfig::FailedWrapMinAngleDeg 주석 참고(대상 크기 무관 척도).
	 */
	bool ShouldAbortFailedShortWrap(const FRopeSimState& Sim, const FContext& Ctx,
		float MinRequiredAngleDeg, float& OutAngleDeg) const;

	/** 마지막 성공 path/anchor 거리를 helix 공식에 넣어 누적 감싼 각도(도)를 계산한다.
	 *  실패 조기 abort(위)와 커밋 품질 관문(CommitMinWrapAngleDeg — URopeComponent::CommitWrapping)이
	 *  같은 척도를 쓰도록 공개한다. 커밋 로그의 angle 표기도 이 값. */
	bool ComputeWrappedAngleAtLastBuiltPoint(const FRopeSimState& Sim, const FContext& Ctx, float& OutAngleDeg) const;

	/**
	 * 형상 기준 묶임 척도(5단계): 감김 축 둘레에서 경로점들이 실제로 둘러싼 각도 커버리지(도, 0~360).
	 * 경로점 각도를 정렬해 최대 공백을 찾고 360°에서 뺀다 — 누적 각도와 달리 표면 위 진동/왕복으로
	 * 부풀지 않아 "대상이 빠져나갈 공백이 남았는가"를 직접 답한다. 브리지(chord) 점도 포함한다:
	 * chord가 가로지른 구간도 로프가 막고 있는 방향이다. 커밋 관문(CommitMinWrapCoverageDeg)과
	 * 전이 로그가 소비한다.
	 * @return 경로점이 2개 미만이거나 축이 축퇴(모든 점이 축 위)면 false — 호출자는 관문을 건너뛴다.
	 */
	bool ComputeWrapEnclosureCoverage(float& OutCoverageDeg) const;

	/**
	 * 현재 앵커들로 Wrapped 핸드오프용 시드를 조립한다(FRopeWrapController::BeginWrap 입력).
	 * 유효 노드가 없으면 Anchors가 빈 시드가 반환된다 — 호출자가 검사해 abort한다.
	 */
	FRopeWrapState BuildCommitSeed(const FRopeSimState& Sim, const USceneComponent* Mesh) const;

	/** abort 시 앵커 노드들의 솔버 복귀(InvMass=1 + Prev=Pos 튐 방지)를 OutFrame에 담는다. */
	void ReturnNodesToSolver(const FRopeSimState& Sim, FRopeNodeOverrideFrame& OutFrame) const;

	/** Builds a complete target centerline using the same helix/vector-field path code as runtime wrapping. */
	bool BuildPreviewCenterline(const FRopeSurfaceAnchor& LatchAnchor, const USceneComponent* Mesh, FName Bone,
		const FRopeSimState& Sim, const FContext& Ctx, TArray<FVector>& OutCenterline) const;

private:
	struct FSurfaceVectorFieldBoneCandidate
	{
		FName Bone = NAME_None;
		int32 Depth = 0;
		float GraphCost = 0.0f;
		bool bCurrentBone = false;
	};

	//~ progressive 경로 빌드(프레임 분할). Begin이 개시하고 AdvancePathBuild가 예산만큼 전진.
	bool BeginProgressiveWrapPathBuild(const FRopeSurfaceAnchor& LatchAnchor,
		const FRopeSimState& Sim, const FContext& Ctx);

	bool AppendAnalyticProgressiveWrapPathPoint(int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx);

	bool InitializeSurfaceVectorFieldProgressiveWrapPath(const FRopeSurfaceAnchor& LatchAnchor,
		const FRopeSimState& Sim, const FContext& Ctx);

	bool AdvanceSurfaceVectorFieldProgressiveWrapPath(int32 StepBudget, const FRopeSimState& Sim, const FContext& Ctx);

	bool AppendWrappingAnchorFromPathPoint(int32 PathIndex, const FRopeSimState& Sim, const FContext& Ctx);

	/** 경로 빌드 종료 기록(성공=Complete / 실패=Failed, 둘 다 Active 해제). 커밋 판정 등 독자들은
	 *  세 플래그를 "빌드가 끝났나"(OR)로만 소비한다 — 개별 조합을 구분해 읽는 곳은 없다. */
	void FinishPathBuild(bool bFailed);

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

	/**
	 * 감김 축 유도. 우선순위(앞머리는 Config.WrappingAxisSource가 결정 — ERopeWrappingAxisSource):
	 *  0) [TravelPlaneFirst일 때만] 로프 진행(스윙) 평면 normal 축 — 감김 원주를 로프의 운동 평면에
	 *     고정한다(여러 본에 걸친 랩 대비). 없으면 아래로 폴백.
	 *  1) latch 본에 귀속된 collider의 *형상 축*(캡슐 세그먼트/박스 최장축/SDF bounds 최장축) —
	 *     본→부모 벡터는 짧고 두꺼운 본(몸통)·체인 본(목/꼬리)·특이 임포트 축에서 지오메트리 장축과
	 *     어긋나 나선 반지름이 실제 단면과 틀어졌다(드래곤 wrap 실패의 핵심). 형상 축은 실제 충돌
	 *     지오메트리에서 나오므로 본 그래프 형태와 무관하게 맞고, 축 origin도 지오메트리 중심축 위라
	 *     helix 반지름(latch↔축 거리)이 정확해진다.
	 *  2) [ShapeAxisFirst일 때] rope spline guide 평면 normal + bone 위치로 만든 가상 축.
	 *  3) 본→부모 축(스켈레탈), 비-스켈레탈은 컴포넌트 기저축 중 latch normal에 가장 수직인 축.
	 *  4) 본 로컬 X.
	 * SurfaceVectorField에서는 latch 시 1회로 끝나지 않는다 — 본 전환마다
	 * ReseedWrappingAxisOnBoneTransition이 새 본 기준으로 재호출한다(rolling axis).
	 * TravelPlaneFirst에서는 재시드에서도 진행 평면 축이 이겨 축 방향이 운동 평면에 고정된다.
	 */
	bool ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx,
		FVector& OutAxisOrigin, FVector& OutAxisDirection) const;

	/**
	 * 본 전환 직후 감김 축을 새 본 기준으로 재해석한다(rolling axis). 필드가 옛 본 축을 계속 돌면
	 * 전환 뒤 표면과 어긋나 projection 실패로 조기 종료되기 쉽다 — 현재 경로 지점의 표면 프레임을
	 * 새 본 로컬로 옮긴 합성 anchor로 ResolveWrappingAxis를 다시 돌리고, 새 축의 부호는 이전 축과
	 * 정렬(피치 드리프트 연속), winding은 현재 진행 tangent 기준으로 재선출해 전환점에서 감김
	 * 방향이 뒤집히지 않게 한다. 축 유도 실패 시 기존 축을 유지한다(종전 단일 축 동작 폴백).
	 */
	void ReseedWrappingAxisOnBoneTransition(FName Bone, const USceneComponent* Mesh, const FContext& Ctx);

	/** 1)의 구현: Ctx.Colliders에서 (Bone, Mesh)에 귀속된 collider를 찾아 형상 축을 돌려준다. */
	static bool FindColliderShapeAxis(const FContext& Ctx, FName Bone, const USceneComponent* Mesh,
		FVector& OutAxisOrigin, FVector& OutAxisDirection);

	/** 2)의 구현: Flight guided spline 평면 normal을 bone 위치에 세운 가상 축으로 돌려준다. */
	static bool FindGuidePlaneAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx, const USceneComponent* Mesh,
		FVector& OutAxisOrigin, FVector& OutAxisDirection);

	void OrientWrappingAxisByTail(const FRopeSurfaceAnchor& LatchAnchor, const FRopeSimState& Sim,
		const USceneComponent* Mesh, FVector& InOutAxisDirection) const;

	/** SurfaceVectorField 후보 본 수집. parent/child graph를 제한 비용 탐색해 전환 비용을 함께 넘긴다. */
	void GatherSurfaceVectorFieldBoneCandidates(FName CurrentBone, const USceneComponent* Mesh,
		TArray<FSurfaceVectorFieldBoneCandidate>& OutCandidates, const FContext& Ctx) const;

	/** 후보 본들의 표면 projection을 graph 비용/hysteresis와 함께 점수화해 path point의 Bone/Mesh를 선택한다. */
	bool ProjectWrapPointToSurfaceMultiBone(FName CurrentBone, const USceneComponent* Mesh,
		const FRopeSimState& Sim, const FContext& Ctx,
		FName PreviousBone, float DistanceSinceLastTransition, const FVector& RopeNodeWorld,
		const FVector& PreviousNormalWorld, const FVector& PreviousTangentWorld,
		FVector& InOutSurfaceWorld, FVector& InOutNormalWorld, FVector& InOutTangentWorld,
		FVector& InOutCircumferenceDir, FName& InOutBone, const USceneComponent*& OutMesh) const;

	bool ProjectWrapPointToSurface(FName Bone, const USceneComponent* Mesh,
		const FRopeSimState& Sim, const FContext& Ctx,
		FVector& InOutSurfaceWorld, FVector& InOutNormalWorld) const;

	//~ front 이동
	void AdvanceWrappingFront(float DeltaTime, const FRopeSimState& Sim, const FContext& Ctx);

	bool SampleWrappingPath(float DistanceFromLatch, FRopeWrapPathPoint& OutPoint) const;
};
