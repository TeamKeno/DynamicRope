// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그의 한 프레임 스냅샷. sim tick(GT)이 채워 URopeDebugSubsystem에 제출하고,
// FGameplayDebuggerCategory_Rope가 읽어 AddShape/AddTextLine으로 그린다. 즉시모드 DrawDebug*를
// 대체하는 데이터 운반체 — 모두 POD라 UObject 결합이 없다(솔버/로직 철학과 동일).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

// flight 노드별 디버그: 이전→현재 이동 + (필요 시) 접촉. 캡처 대상 로프에서만 채워진다.
struct FRopeFlightNodeDebug
{
	int32 NodeIndex = INDEX_NONE;
	FVector PrevPosition = FVector::ZeroVector;
	FVector Position = FVector::ZeroVector;
	float NodeSpeed = 0.0f;
	bool bFast = false;
	bool bNearBody = false;
	FRopeContact Contact;
};

// collider 시각화 한 개. analytic capsule이면 A-B 세그먼트+반지름, 그 외(SDF 등)는 월드 bounds 박스.
struct FRopeDebugCollider
{
	bool bIsCapsule = false;
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float Radius = 0.0f;
	FBox Bounds = FBox(ForceInit);
};

// 한 로프의 한 프레임 디버그 스냅샷. centerline/flight/whip/wrapped/collider를 한데 담는다.
// 비어 있는 섹션은 b*Has 플래그로 구분(예: bHasFlight는 Flight phase에서만 true).
struct FRopeDebugSnapshot
{
	// 제출 프레임(GFrameCounter). 카테고리는 너무 오래된 스냅샷을 무시한다(대상 해제 후 잔상 방지).
	uint64 FrameStamp = 0;

	//~ centerline(항상) ---------------------------------------------------
	ERopePhase Phase = ERopePhase::Free;
	TArray<FVector> Positions;
	TArray<int32> LatchedNodes; // centerline 상에서 강조할 latch 노드 인덱스.

	//~ flight(Flight phase에서만) ----------------------------------------
	bool bHasFlight = false;
	bool bSolveThisFrame = false;
	bool bShouldCapture = false;
	int32 FrameColliderCount = 0;
	int32 MinLatchNodes = 0;
	FName TrackerBone = NAME_None;
	TArray<int32> TrackerNodes;
	TArray<FRopeFlightNodeDebug> NodeDebug;
	TArray<FRopeContactCandidate> Candidates;

	//~ whip guide(whip 활성 시) ------------------------------------------
	bool bWhipActive = false;
	float WhipGuidedEnd = 0.0f;
	TArray<int32> WhipGuideNodeIndices;
	TArray<FVector> WhipGuideTargets;

	//~ wrapped(Wrapped phase에서만) --------------------------------------
	bool bHasWrapped = false;
	FName WrapBone = NAME_None;
	FString MeshName;
	TArray<FRopeLatchNode> Latched;
	float WrapTension = 0.0f;         // 최대 세그먼트 장력(FRopeWrapState::Tension 미러)
	float TensionReleaseForce = 0.0f; // 임계 장력(0=비활성) — 표시용
	bool bPullValid = false;          // ComputePull 성공(앵커/방향 유효 — 장력 0이어도 true)
	FVector PullPoint = FVector::ZeroVector;     // 힘 인가점(앵커 월드)
	FVector PullDirection = FVector::ZeroVector; // 당김 단위 방향(EMA 스무딩 후 — 실제 인가 방향)
	FVector PullDirRaw = FVector::ZeroVector;    // 스무딩 전 look-ahead 방향(원본) — 지터 진단용(스무딩 대비)
	FVector PullAimPoint = FVector::ZeroVector;  // 첫 직선 다리 끝(walk가 멈춘 노드 월드) — 방향 조준점
	int32 PullAimNode = INDEX_NONE;              // 위 조준 노드 인덱스(프레임마다 튀면 방향 불안정 신호)
	float PullTension = 0.0f;                    // 앵커 세그먼트 장력
	float TetherResponse = 0.0f;                 // 테더 반응(0=비활성) — 표시용
	float TetherOvershoot = 0.0f;                // 가용 로프 길이 초과분(cm, 0=팽팽하지 않음)
	float ActivePullForce = 0.0f;                // 능동 Pull 힘(0=입력 없음)
	float DistanceReleaseSlack = 0.0f;           // 거리 release 한계(cm, 0=비활성) — 표시용

	//~ colliders(이 로프가 이번 프레임 질의한 collider들) ----------------
	TArray<FRopeDebugCollider> Colliders;
};
