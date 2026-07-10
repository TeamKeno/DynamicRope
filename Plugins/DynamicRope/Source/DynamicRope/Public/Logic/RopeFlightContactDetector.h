// Copyright Epic Games, Inc. All Rights Reserved.
//
// Flight 중 접촉 후보 감지 파이프라인: 솔브 결과의 이동 경로에서 실제 접촉을 찾고(Detect),
// 빠른 노드/whip 가이드 노드의 다음 위치를 외삽해 예측 접촉을 추가하고(AddPredicted),
// 표면 대비 상대운동을 평가한 뒤(EvaluateRelativeMotion), 캡처 여부를 판정한다(ShouldCapture).
// FinalizeSimFrame(GT)에서 매 프레임 호출된다 — Flight → Contacting 전이의 입력을 만드는 곳.
//
// 솔버/랩 컨트롤러와 같은 UObject 비의존 패턴. 상태가 없어 전부 static이다.
// 입력은 POD(FRopeSimState) + collider 스냅샷 + 파라미터 스냅샷(FParams) + whip 가이드
// 데이터 뷰(FWhipGuideView)뿐이라 월드 없이 단위 테스트할 수 있고, 위치가 CPU 솔버에서
// 오든 GPU 미러에서 오든 동작이 같다(GPU 전환 후에도 그대로 살아남는 레이어).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class IRopeCollider;

class DYNAMICROPE_API FRopeFlightContactDetector
{
public:
	// 검출 파라미터 스냅샷. 디자이너 원본(UPROPERTY)은 URopeComponent에 남고 호출 시 복사한다.
	struct FParams
	{
		float ContactRadius = 3.0f;           // WrapConfig.ContactRadius: 접촉 질의 반경
		float RopeRadius = 2.0f;              // 튜브 반지름(broad-phase 바운즈 여유에 합산)
		float PredictiveContactFrames = 0.0f; // WrapConfig.PredictiveContactFrames: 예측 외삽 프레임 수
		int32 MinLatchNodes = 1;              // WrapConfig.MinLatchNodes: 캡처에 필요한 최소 접촉 노드 수
		FVector FallbackForward = FVector::ForwardVector; // ExpectedWrapTangent 퇴화 케이스용(컴포넌트 전방)
		// 프레임 dt(초): SurfaceVelocity(FROZEN 계약 — cm/s)를 로프 Verlet 속도(cm/프레임)와 같은
		// 단위로 환산하는 다리. sim 호출자(FinalizeSimFrame)는 실제 프레임 dt를 넣는다 — 기본값은
		// dt가 없는 호출자(preview: 정적 스냅샷이라 표면속도 0)용 보수치. 환산 없이 빼면 움직이는
		// 본에서 표면속도가 ~1/dt배(60fps 기준 ~60배) 과대 반영된다(2026-07-09 발견 버그).
		float DeltaTime = 1.0f / 60.0f;
	};

	// whip 가이드 프레임 데이터 뷰(예측 접촉의 가이드 노드 분기 입력). 포인터는 소유하지 않으며
	// 호출 동안만 유효하면 된다. 가이드 비활성이면 기본값(전부 nullptr) 그대로 넘긴다.
	// NextTargets는 다음 프레임 시점의 가이드 타깃(FRopeWhipGuide::PreviewNextTargets 결과)으로,
	// 호출자가 미리 계산해 넣는다 — 검출기는 가이드 클래스가 아니라 데이터만 본다.
	struct FWhipGuideView
	{
		const TArray<uint8>* GuidedNodeMask = nullptr;
		const TArray<FVector>* CurrentTargets = nullptr;
		const TArray<FVector>* PrevTargets = nullptr;
		const TArray<FVector>* NextTargets = nullptr;

		bool HasGuidedNodes() const { return GuidedNodeMask && GuidedNodeMask->Num() > 0; }
		bool IsGuidedNode(int32 NodeIndex) const
		{
			return GuidedNodeMask && GuidedNodeMask->IsValidIndex(NodeIndex) && (*GuidedNodeMask)[NodeIndex] != 0;
		}
	};

	/** 솔브 전후 위치(Sim.PrevPositions → Positions)의 이동 경로에서 실제 접촉 후보를 수집한다. */
	static void DetectContactCandidates(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
		const FParams& Params, TArray<FRopeContactCandidate>& OutCandidates);

	/**
	 * 예측 접촉 후보 추가: 빠른/tail/가이드 노드의 다음 위치를 외삽한 경로를 스윕해, 아직 닿지
	 * 않았지만 곧 닿을 접촉을 같은 후보 파이프라인으로 승격한다. 같은 (node, bone, mesh) 후보는
	 * SourceMask를 합치고 우선순위(Guided > Actual > Free)로 Source를 갱신한다.
	 */
	static void AddPredictedContactCandidates(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
		const FParams& Params, const FWhipGuideView& Whip, TArray<FRopeContactCandidate>& InOutCandidates);

	/** 각 후보의 표면 대비 상대 접선 속도와 감김 방향 점수(WrapDirectionScore)를 채운다. */
	static void EvaluateRelativeMotion(const FRopeSimState& Sim, const FParams& Params,
		TArray<FRopeContactCandidate>& Candidates);

	/** dominant bone에 MinLatchNodes 이상이 접촉했으면 캡처(Flight → Contacting) 판정. */
	static bool ShouldCapture(const TArray<FRopeContactCandidate>& Candidates, const FParams& Params);

	/**
	 * 접촉 후보가 "그냥 닿음"을 넘어 실제 감김으로 볼 만한지 검사하는 품질 게이트.
	 * 현재는 감김 폴리싱 우선이라 항상 통과시키고, 나중에 점수/속도/방향 기준을 여기 안에 채운다.
	 */
	static bool PassesCaptureQualityGate(const FRopeContactTracker& Tracker,
		const TArray<FRopeContactCandidate>& Candidates, const FParams& Params);

	//~ 개별 헬퍼 — 디버그 수집(FinalizeSimFrame)과 Contacting 시드 빌드에서도 쓰인다.
	/** 로프 끝(tail) 근처 노드인가(마지막 4개). tail은 속도와 무관하게 항상 검사 대상. */
	static bool IsTailNode(const FRopeSimState& Sim, int32 NodeIndex);

	/** 노드의 프레임 이동 거리(Verlet 속도 크기). */
	static float NodeSpeed(const FRopeSimState& Sim, int32 NodeIndex);

	/** 세그먼트(Prev→Pos) 바운즈가 어떤 collider 바운즈와도 겹치는가(broad phase). */
	static bool IsNearAnyColliderSegment(const FVector& PrevPosition, const FVector& Position,
		const TArray<IRopeCollider*>& Colliders, const FParams& Params);

	/** 세그먼트 바운즈와 겹치는 collider만 추린다(broad phase). */
	static void GatherNearbyColliders(const FVector& PrevPosition, const FVector& Position,
		const TArray<IRopeCollider*>& Colliders, const FParams& Params, TArray<IRopeCollider*>& OutNearbyColliders);

	/** 이동 경로를 몇 개 샘플로 나눠 질의해 가장 깊은 접촉을 고른다(캡슐/SDF 공통 경로). */
	static FRopeContact SweepOrSampleContact(const FRopeSimState& Sim, const FVector& PrevPosition,
		const FVector& Position, const TArray<IRopeCollider*>& Colliders, const FParams& Params);

	/** FRopeContact → 후보 변환(SurfaceVelocity 포함). */
	static FRopeContactCandidate MakeCandidate(int32 NodeIndex, const FRopeContact& Contact);

	/** 접촉점에서 손(node 0) 쪽으로 향하는, 표면 접선면에 투영된 기대 감김 방향. */
	static FVector ExpectedWrapTangent(const FRopeSimState& Sim, const FRopeContactCandidate& Candidate,
		const FVector& FallbackForward);

	static bool IsWrappableBone(FName Bone) { return !Bone.IsNone(); }

private:
	/** 가이드 활성 프레임엔 가이드/tail/빠른 노드만 예측 검사를 돌린다(비용 절약). */
	static bool ShouldRunPredictiveContactForNode(const FRopeSimState& Sim, const FWhipGuideView& Whip,
		int32 NodeIndex, const FVector& FrameDisplacement);
};
